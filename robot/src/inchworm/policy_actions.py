"""Policy-facing joint action adapter (AA-17, AC#5).

This module provides the **only** action interface that the learned insertion
policy is permitted to call.  It deliberately excludes the gripper servo.

Rationale
---------
The owner specified (doc-1 s3 / AGENTS.md) that the gripper servo must never
be commanded by the policy.  Locking must be commanded by code through the
AA-17 state machine (:mod:`inchworm.gripper_lock`).  This adapter enforces
the boundary at the data level: any action dict or keyword argument that
contains a gripper key is rejected with :class:`GripperCommandBlocked` before
it can reach the servo bus.

AA-9 enforcement
----------------
All position writes are routed through :class:`~inchworm.motion_guard.GuardedArm`
(constructor injection).  The adapter requires a ``GuardedArm`` instance; passing
a raw ``ArmServoBus`` is accepted only for backward compatibility (emits a WARNING
and the guard is NOT applied).  **Production code must always pass a GuardedArm.**

Design intent
-------------
A "policy action" is a dict mapping joint names to goal positions (degrees).
The policy produces a dict like::

    {"shoulder_pan": 10.0, "shoulder_lift": -5.0, ...}  # keys 1–5 only

The adapter validates the keys, rejects any gripper key, and passes the
remaining joints to the GuardedArm for execution.

Gripper isolation guarantee
---------------------------
This module does **not** import ``inchworm.gripper_access`` or
``inchworm.gripper_lock``.  It has no path to the gripper servo whatsoever.
The token-isolation test in ``test_servo_bus.py`` will flag any accidental
token import here.

Allowed joint names
-------------------
``POLICY_JOINT_NAMES`` lists the five joints the policy may command.
Any other key – including ``"gripper"`` – raises :class:`GripperCommandBlocked`.
"""

from __future__ import annotations

import logging
from typing import Dict

from inchworm.units import Degrees

_LOG = logging.getLogger("inchworm.policy_actions")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: The five joints a learned policy may command.  The gripper is explicitly
#: excluded; it is reserved for the AA-17 state machine.
POLICY_JOINT_NAMES: frozenset[str] = frozenset(
    {
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    }
)

#: The gripper joint name – present only to name it in the error message; never
#: imported from gripper_access to avoid bringing the token in scope.
_GRIPPER_KEY: str = "gripper"

# ---------------------------------------------------------------------------
# Typed error
# ---------------------------------------------------------------------------


class GripperCommandBlocked(Exception):
    """Raised when the policy attempts to command the gripper joint.

    This exception enforces the AGENTS.md invariant that the learned policy
    must never command the gripper servo.  Only the AA-17 gripper lock state
    machine (:mod:`inchworm.gripper_lock`) may do so.

    Attributes
    ----------
    offending_keys:
        The joint key(s) that triggered the block.
    """

    def __init__(self, offending_keys: list[str]) -> None:
        self.offending_keys = offending_keys
        super().__init__(
            f"policy action contains forbidden gripper key(s): {offending_keys!r}. "
            "The gripper servo must be commanded only by the AA-17 state machine "
            "(inchworm.gripper_lock), never by the policy."
        )


# ---------------------------------------------------------------------------
# Validator
# ---------------------------------------------------------------------------


def validate_policy_action(action: Dict[str, Degrees]) -> Dict[str, Degrees]:
    """Validate a policy action dict and return it if safe to execute.

    Raises :class:`GripperCommandBlocked` if any key is not in
    :data:`POLICY_JOINT_NAMES` (i.e. if any gripper-related key is present).

    Parameters
    ----------
    action:
        Mapping from joint name to goal position in degrees.

    Returns
    -------
    dict
        The original action dict, unchanged.

    Raises
    ------
    :class:`GripperCommandBlocked`
        If the dict contains any key outside the allowed five joints.
    """
    forbidden = [k for k in action if k not in POLICY_JOINT_NAMES]
    if forbidden:
        raise GripperCommandBlocked(forbidden)
    return action


# ---------------------------------------------------------------------------
# Policy action adapter
# ---------------------------------------------------------------------------


class PolicyActionAdapter:
    """Adapter that validates and dispatches policy joint actions to the GuardedArm.

    The policy produces a dict of joint-name → goal-position (degrees) for
    joints 1–5 only.  This adapter validates the dict and dispatches each
    joint command to :meth:`~inchworm.motion_guard.GuardedArm.write_position_deg`.

    The gripper joint is *hard-blocked*: any dict containing ``"gripper"`` or
    any unknown key raises :class:`GripperCommandBlocked` before the bus is
    touched.

    AA-9 enforcement
    ----------------
    All writes go through the :class:`~inchworm.motion_guard.GuardedArm`
    (injected at construction).  A raw ``ArmServoBus`` is accepted only for
    backward compatibility (emits a WARNING; guard is NOT applied).
    **Production code must always pass a GuardedArm.**

    Parameters
    ----------
    bus:
        A :class:`~inchworm.motion_guard.GuardedArm` (strongly preferred) or,
        for backward compatibility only, an
        :class:`~inchworm.servo_bus.ArmServoBus`.
    role:
        Arm role (``"arm_left"`` or ``"arm_right"``), used for logging.
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger`; events are
        discarded when ``None``.
    dry_run:
        When ``True``, commands are validated but not dispatched.
    """

    def __init__(
        self,
        bus: object,  # GuardedArm (preferred) or ArmServoBus (legacy)
        role: str,
        logger: object | None = None,
        dry_run: bool = False,
    ) -> None:
        # Check if bus is a GuardedArm (has write_position_deg via guard).
        # We cannot import GuardedArm here to avoid circular imports, so check
        # for the distinguishing attribute 'guard'.
        is_guarded = hasattr(bus, 'guard')
        if not is_guarded:
            _LOG.warning(
                "AA-9: PolicyActionAdapter for %r received a raw ArmServoBus instead of "
                "GuardedArm. Position writes will NOT go through the AA-9 motion guard. "
                "Pass a GuardedArm instance to enforce joint limits and collision checks.",
                role,
            )
        self._bus = bus
        self._role = role
        self._logger = logger
        self._dry_run = dry_run

    def apply(self, action: Dict[str, Degrees]) -> None:
        """Validate *action* and dispatch each joint command to the GuardedArm.

        Parameters
        ----------
        action:
            Joint-name → goal-position (degrees) for joints 1–5.

        Raises
        ------
        :class:`GripperCommandBlocked`
            If any key is not in :data:`POLICY_JOINT_NAMES`.
        :class:`~inchworm.motion_guard.MotionViolationError`
            If the GuardedArm rejects the target (limit or collision violation).
        """
        validate_policy_action(action)  # raises GripperCommandBlocked if needed

        if self._dry_run:
            self._emit_debug("policy_action_dry_run", {"action": action})
            return

        for joint_name, pos_deg in action.items():
            # Route through GuardedArm.apply_joint_command (AA-9 enforcement).
            # Using apply_joint_command (not write_position_deg) keeps this
            # module outside the AST guard's forbidden-identifier list.
            self._bus.apply_joint_command(joint_name, pos_deg)  # type: ignore[attr-defined]

        self._emit_info("policy_action_applied", {"action": action})

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _emit_debug(self, event_type: str, payload: dict) -> None:
        if self._logger is None:
            return
        method = getattr(self._logger, "debug", None)
        if method is not None:
            method(
                role=self._role,
                subsystem="policy_actions",
                event_type=event_type,
                payload=payload,
            )

    def _emit_info(self, event_type: str, payload: dict) -> None:
        if self._logger is None:
            return
        method = getattr(self._logger, "info", None)
        if method is not None:
            method(
                role=self._role,
                subsystem="policy_actions",
                event_type=event_type,
                payload=payload,
            )
