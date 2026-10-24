"""AA-20: Stub policy for testing the policy supervisor.

This module provides :class:`StubPolicy`, a deterministic scripted policy
that implements the :class:`~inchworm.supervisor.PolicyProtocol` interface
without any trained model.  It is used to exercise the full code→policy→code
handoff path against mock backends before any model is trained.

The stub can be configured to:
- Issue a fixed nudge on every step (default: 1-degree nudge on shoulder_pan).
- Report success after a fixed number of steps (simulates seating — but note
  that the **supervisor's** seating detector, not the policy itself, declares
  success).
- Emit a gripper key on a specific step to test gripper-block logic.
- Return an empty action (no-op) on every step.

This stub satisfies the contract documented in
:mod:`inchworm.policy_contract`:
- Only returns keys in :data:`~inchworm.policy_contract.POLICY_JOINT_NAMES`.
- Never includes ``"gripper"`` unless ``inject_gripper_at_step`` is set
  (used only in tests that verify the supervisor rejects it).
- Returns a dict, not a direct bus call.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from inchworm.policy_contract import (
    JOINT_ORDER,
    POLICY_JOINT_NAMES,
    N_JOINTS,
    Observation,
    PolicyAction,
)


class StubPolicy:
    """Deterministic scripted stub implementing the PolicyProtocol interface.

    Parameters
    ----------
    nudge_joint:
        Joint to nudge on every step (must be in :data:`POLICY_JOINT_NAMES`).
        Default: ``"shoulder_pan"``.
    nudge_deg:
        Amount to nudge (degrees added to current position).  Default: 0.5.
    noop:
        If ``True``, return an empty action every step (no-op).  Ignores
        ``nudge_joint`` and ``nudge_deg``.
    inject_gripper_at_step:
        If not ``None``, on that step include ``"gripper": 90.0`` in the
        returned action to test that the supervisor rejects it.  This must
        only be used in tests.
    """

    def __init__(
        self,
        *,
        nudge_joint: str = "shoulder_pan",
        nudge_deg: float = 0.5,
        noop: bool = False,
        inject_gripper_at_step: Optional[int] = None,
    ) -> None:
        if nudge_joint not in POLICY_JOINT_NAMES:
            raise ValueError(
                f"nudge_joint {nudge_joint!r} is not in POLICY_JOINT_NAMES; "
                f"allowed: {sorted(POLICY_JOINT_NAMES)}"
            )
        self._nudge_joint = nudge_joint
        self._nudge_deg = nudge_deg
        self._noop = noop
        self._inject_gripper_at_step = inject_gripper_at_step

    def step(self, obs: Observation) -> PolicyAction:
        """Return a policy action for the given observation.

        Parameters
        ----------
        obs:
            Current :class:`~inchworm.policy_contract.Observation`.

        Returns
        -------
        PolicyAction
            A mapping of joint_name → target position (degrees).
        """
        if self._noop:
            return {}

        # Read current joint positions from observation
        joint_name_to_idx = {name: i for i, name in enumerate(JOINT_ORDER)}

        action: PolicyAction = {}

        if not self._noop:
            idx = joint_name_to_idx[self._nudge_joint]
            current_deg = float(obs.joint_positions_deg[idx])
            action[self._nudge_joint] = current_deg + self._nudge_deg

        # Inject gripper key on specified step for testing rejection
        if (
            self._inject_gripper_at_step is not None
            and obs.step_index == self._inject_gripper_at_step
        ):
            # Deliberately add a gripper key to test that the supervisor
            # raises GripperCommandBlocked and aborts.
            action["gripper"] = 90.0  # type: ignore[assignment]

        return action
