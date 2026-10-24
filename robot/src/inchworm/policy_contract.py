"""AA-20: Documented code-policy-code handoff contract.

This module is the **canonical specification** of the boundary between
scripted robot code and the learned insertion policy.  All parts of the
boundary are expressed as code so that changes to the contract are visible
in version control and immediately break any implementation that violates them.

CONTRACT OVERVIEW
=================

Control flow
------------
::

    [scripted code]
        │  approach → pre-insertion pose
        │  release one gripper (AA-17)
        │  hand control to PolicySupervisor
        ▼
    [PolicySupervisor] ─── enforce workspace, step/time limits, magnitude clamp
        │  call policy in a loop: step(obs) → action
        │  apply action via PolicyActionAdapter (AA-17) ── gripper ALWAYS rejected
        │  check seating detector (AA-24) after each step
        ▼
        │  on success  → call lock_callable (→ gripper_lock.lock, AA-17)
        │  on abort    → retract to safe pose, log typed event, return control
        │  on retry    → re-approach (approach_callable), repeat loop
        ▼
    [scripted code] ← control returned

Observation specification (CODE-SIDE GUARANTEE)
-------------------------------------------------
Code provides exactly one wrist-camera mono frame per step:

- **Image**: single-channel (monochrome) uint8 NumPy array, shape ``(H, W)``
  where H and W are set at supervisor construction time and fixed for the
  duration of the handoff (see :data:`DEFAULT_FRAME_HEIGHT`,
  :data:`DEFAULT_FRAME_WIDTH`).
- **Joint positions**: 5-vector of float32 joint positions in **degrees**,
  ordered proximal-to-distal::

      [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]

  The gripper joint is **excluded**.
- Frame source: injected AA-6/AA-14 callable (never direct camera access).

Action specification (POLICY CONTRACT)
---------------------------------------
The policy returns a mapping of joint-name → target position in **degrees**
for any subset of the five allowed joints.

- Allowed keys: see :data:`POLICY_JOINT_NAMES` (mirrors
  :data:`inchworm.policy_actions.POLICY_JOINT_NAMES`).
- Forbidden keys: any key not in that set, especially ``"gripper"``.
  The supervisor immediately rejects such actions and aborts with
  :attr:`~inchworm.supervisor.AbortReason.GRIPPER_COMMAND_BLOCKED`.
- Action magnitude: the Euclidean norm of ``[Δ_j1, ..., Δ_j5]`` (step deltas
  in degrees) must not exceed
  :attr:`~inchworm.config.PolicySupervisorConfig.max_action_magnitude_deg`.
  The supervisor clamps each step before applying it; if the clamped magnitude
  is less than the raw magnitude, a ``supervisor_action_clamped`` event is
  logged.

Pre-insertion pose (HANDOVER PRECONDITION)
-------------------------------------------
Control is handed to the supervisor only when the arm TCP is within
``workspace_radius_m`` of the target attachment point (in Cartesian space,
computed via FK in the arm base frame).  The supervisor verifies this at
handover and aborts immediately if the precondition is not met.

Success condition (CODE-SIDE GUARANTEE)
-----------------------------------------
Success is declared by an injected **seating detector** callable that is
independent of the policy's own output.  The policy **never** declares its
own success.  The seating detector is a placeholder in this milestone (AA-24
will implement it) and defaults to always returning False so the supervisor
always exercises its step/time/abort paths in tests.

Abort conditions (SUPERVISOR GUARANTEE)
-----------------------------------------
The supervisor aborts and returns control to code on any of:

1. Step count exceeds ``max_steps``.
2. Wall-clock time exceeds ``max_wall_time_s``.
3. FK of current joint angles places the TCP outside the Cartesian workspace
   sphere of radius ``workspace_radius_m`` around ``target_attachment_point_m``.
4. Per-step action magnitude (pre-clamp) exceeds ``max_action_magnitude_deg``
   by more than the clamping tolerance (only when clamping is disabled; default
   is to clamp, not abort, on magnitude violation).
5. The policy attempts to command the gripper joint.

On abort:
- Arm retracts to ``retract_pose_q`` (injected via motion executor).
- A :class:`~inchworm.events._SupervisorAbortPayload` event is logged via
  AA-7 telemetry.
- Control returns to code; no operator input required.

Retry policy (SUPERVISOR GUARANTEE)
--------------------------------------
After abort, the supervisor may retry up to ``max_retries`` times.  Each
retry calls the injected ``approach_callable`` (which performs the re-approach;
AA-18 will implement this later), then restarts the policy loop.  After all
retries are exhausted, the supervisor logs a ``supervisor_halt`` event and
returns :attr:`~inchworm.supervisor.SupervisorOutcome.HALT`.

Code-side guarantees (never violated by the supervisor)
---------------------------------------------------------
1. The gripper servo is **never** commanded by this module or any callable
   it owns.  The ``lock_callable`` is the only path to the gripper, and it is
   called only on verified success.
2. Camera access is always through the injected ``frame_source``; this module
   never opens a camera directly (AA-6 arbiter / AA-14 capture invariant).
3. The supervisor is fully testable with mock callables; no hardware import is
   required.
4. ``dry_run=True`` logs all actions but does not call ``motion_executor`` or
   ``lock_callable``; ``approach_callable`` and ``seating_detector`` are still
   called so their logic can be exercised.

CONTRACT VERSION
----------------
Increment :data:`CONTRACT_VERSION` whenever the observation spec, action spec,
or success/abort semantics change in a backward-incompatible way.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence

import numpy as np
import numpy.typing as npt

# Re-export from policy_actions for a single point of truth.
from inchworm.policy_actions import POLICY_JOINT_NAMES, GripperCommandBlocked

__all__ = [
    "CONTRACT_VERSION",
    "DEFAULT_FRAME_HEIGHT",
    "DEFAULT_FRAME_WIDTH",
    "N_JOINTS",
    "JOINT_ORDER",
    "POLICY_JOINT_NAMES",
    "GripperCommandBlocked",
    "Observation",
    "PolicyAction",
    "ContractViolation",
    "validate_observation",
    "validate_action",
]

# ---------------------------------------------------------------------------
# Contract constants
# ---------------------------------------------------------------------------

#: Increment when the observation spec, action spec, or handoff semantics
#: change in a backward-incompatible way.
CONTRACT_VERSION: int = 1

#: Default wrist camera frame height (pixels).
DEFAULT_FRAME_HEIGHT: int = 480

#: Default wrist camera frame width (pixels).
DEFAULT_FRAME_WIDTH: int = 640

#: Number of joints in the action/observation space.
N_JOINTS: int = 5

#: Canonical joint order (proximal → distal).  Used to index the joint
#: position vector in :class:`Observation`.
JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

# ---------------------------------------------------------------------------
# Typed data structures
# ---------------------------------------------------------------------------


@dataclass
class Observation:
    """One observation delivered to the policy per step.

    Attributes
    ----------
    frame:
        Monochrome uint8 wrist-camera frame, shape ``(H, W)``.  Height and
        width are fixed at supervisor construction time (see
        :data:`DEFAULT_FRAME_HEIGHT` / :data:`DEFAULT_FRAME_WIDTH`).
    joint_positions_deg:
        5-element float32 array, indexed by :data:`JOINT_ORDER`.  Units:
        degrees.  The gripper is always excluded.
    step_index:
        Zero-based step counter within the current attempt.
    """

    frame: npt.NDArray[np.uint8]
    joint_positions_deg: npt.NDArray[np.float32]
    step_index: int

    def __post_init__(self) -> None:
        if self.frame.ndim != 2:
            raise ContractViolation(
                f"Observation.frame must be 2-D (H, W) uint8; got shape {self.frame.shape}"
            )
        if self.frame.dtype != np.uint8:
            raise ContractViolation(
                f"Observation.frame must be uint8; got dtype {self.frame.dtype}"
            )
        arr = np.asarray(self.joint_positions_deg, dtype=np.float32)
        if arr.shape != (N_JOINTS,):
            raise ContractViolation(
                f"Observation.joint_positions_deg must have shape ({N_JOINTS},); "
                f"got {arr.shape}"
            )
        object.__setattr__(
            self, "joint_positions_deg", arr
        ) if hasattr(type(self), "__dataclass_fields__") else None
        self.joint_positions_deg = arr


#: Type alias for a policy action dict: joint_name → target position (degrees).
PolicyAction = Dict[str, float]


# ---------------------------------------------------------------------------
# ContractViolation exception
# ---------------------------------------------------------------------------


class ContractViolation(Exception):
    """Raised when the contract is violated (wrong shapes, wrong types, etc.)."""


# ---------------------------------------------------------------------------
# Validators
# ---------------------------------------------------------------------------


def validate_observation(obs: Observation) -> Observation:
    """Validate an :class:`Observation` and return it unchanged.

    Raises :class:`ContractViolation` on any shape/dtype mismatch.
    """
    if obs.frame.ndim != 2:
        raise ContractViolation(
            f"frame must be 2-D; got shape {obs.frame.shape}"
        )
    if obs.frame.dtype != np.uint8:
        raise ContractViolation(
            f"frame must be uint8; got {obs.frame.dtype}"
        )
    jp = np.asarray(obs.joint_positions_deg)
    if jp.shape != (N_JOINTS,):
        raise ContractViolation(
            f"joint_positions_deg must have shape ({N_JOINTS},); got {jp.shape}"
        )
    return obs


def validate_action(action: PolicyAction) -> PolicyAction:
    """Validate a policy action dict and return it unchanged.

    Raises :class:`GripperCommandBlocked` if any key is outside
    :data:`POLICY_JOINT_NAMES`.
    """
    from inchworm.policy_actions import validate_policy_action
    return validate_policy_action(action)  # type: ignore[return-value]
