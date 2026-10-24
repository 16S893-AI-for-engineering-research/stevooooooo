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

Design intent
-------------
A "policy action" is a dict mapping joint names to goal positions (degrees).
The policy produces a dict like::

    {"shoulder_pan": 10.0, "shoulder_lift": -5.0, ...}  # keys 1–5 only

The adapter validates the keys, rejects any gripper key, and passes the
remaining joints to the servo bus for execution.

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

from typing import Dict

from inchworm.servo_bus import ArmServoBus
from inchworm.units import Degrees

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
    """Adapter that validates and dispatches policy joint actions to the servo bus.

    The policy produces a dict of joint-name → goal-position (degrees) for
    joints 1–5 only.  This adapter validates the dict and dispatches each
    joint command to :meth:`~inchworm.servo_bus.ArmServoBus.write_position_deg`.

    The gripper joint is *hard-blocked*: any dict containing ``"gripper"`` or
    any unknown key raises :class:`GripperCommandBlocked` before the bus is
    touched.

    Parameters
    ----------
    bus:
        The :class:`~inchworm.servo_bus.ArmServoBus` for the arm the policy
        is controlling.
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
        bus: ArmServoBus,
        role: str,
        logger: object | None = None,
        dry_run: bool = False,
    ) -> None:
        self._bus = bus
        self._role = role
        self._logger = logger
        self._dry_run = dry_run

    def apply(self, action: Dict[str, Degrees]) -> None:
        """Validate *action* and dispatch each joint command to the servo bus.

        Parameters
        ----------
        action:
            Joint-name → goal-position (degrees) for joints 1–5.

        Raises
        ------
        :class:`GripperCommandBlocked`
            If any key is not in :data:`POLICY_JOINT_NAMES`.
        """
        validate_policy_action(action)  # raises GripperCommandBlocked if needed

        if self._dry_run:
            self._emit_debug("policy_action_dry_run", {"action": action})
            return

        for joint_name, pos_deg in action.items():
            self._bus.write_position_deg(joint_name, pos_deg)

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
