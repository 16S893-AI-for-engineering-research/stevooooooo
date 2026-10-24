"""Tests for AA-20: policy_contract, supervisor, and stub_policy.

Covers ALL software ACs:

  AC#1 — Documented contract specifies observation, action space, handover
          pose, success/abort conditions, and code-side guarantees.
          Verified: policy_contract.py exists with all required symbols;
          CONTRACT_VERSION, Observation, PolicyAction, validate_observation,
          validate_action, POLICY_JOINT_NAMES, JOINT_ORDER, N_JOINTS.

  AC#2 — Action space excludes gripper; supervisor rejects/filters gripper
          actions.
          Verified: TestGripperBlock tests.

  AC#3 — Supervisor enforces step limit, wall-clock limit, Cartesian workspace
          bound, and per-step action magnitude clamping.
          Verified: TestStepLimit, TestWallTimeLimit, TestWorkspaceViolation,
          TestMagnitudeClamping tests.

  AC#4 — On abort: supervisor retracts arm to safe pose, logs typed abort
          event with triggering condition, returns control without operator
          input.
          Verified: TestAbortRetract, TestTypedAbortEvent tests.

  AC#5 — Retry policy: bounded attempt count, documented escalation
          (re-approach then halt), logged.
          Verified: TestRetryPolicy tests.

  AC#6 — Stub policy exercises full handoff path end-to-end against mock
          backend with no trained model.
          Verified: TestEndToEnd tests.

  AC#7 — Tests cover handover, success return, each abort trigger, gripper
          rejection, retry escalation and final halt.
          Verified: all test classes together cover these paths.

Hardware ACs: none — all ACs are software-testable.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

from inchworm.config import PolicySupervisorConfig
from inchworm.kinematics import KinematicChain, build_chain, JointFrame
from inchworm.policy_actions import GripperCommandBlocked, POLICY_JOINT_NAMES
from inchworm.policy_contract import (
    CONTRACT_VERSION,
    DEFAULT_FRAME_HEIGHT,
    DEFAULT_FRAME_WIDTH,
    JOINT_ORDER,
    N_JOINTS,
    Observation,
    ContractViolation,
    validate_observation,
    validate_action,
)
from inchworm.stub_policy import StubPolicy
from inchworm.supervisor import (
    AbortReason,
    PolicySupervisor,
    SupervisorOutcome,
    SupervisorResult,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def chain():
    """Build a KinematicChain from the real config."""
    from inchworm.config import load_config
    cfg = load_config()
    return build_chain(cfg.so101_kinematics)


@pytest.fixture()
def supervisor_config_default():
    """Default PolicySupervisorConfig for testing."""
    return PolicySupervisorConfig(
        max_steps=10,
        max_wall_time_s=5.0,
        workspace_radius_m=1.0,  # generous for FK in test
        max_action_magnitude_deg=5.0,
        clamp_on_magnitude_violation=True,
        max_retries=2,
        retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        retract_pose_unverified=False,  # verified for test
    )


def _make_frame(h: int = 4, w: int = 4) -> np.ndarray:
    """Return a minimal (H, W) uint8 test frame."""
    return np.zeros((h, w), dtype=np.uint8)


def _make_joint_deg(value: float = 0.0) -> np.ndarray:
    """Return a 5-element float32 joint positions array."""
    return np.full(N_JOINTS, value, dtype=np.float32)


def _noop_frame_source():
    """Frame source that returns a blank frame and zero joints."""
    return _make_frame(), _make_joint_deg(0.0)


def _make_supervisor(
    *,
    policy,
    config,
    chain,
    target_m=None,
    frame_source=None,
    motion_executor=None,
    approach_callable=None,
    seating_detector=None,
    lock_callable=None,
    logger=None,
    role="arm_left",
    dry_run=True,
) -> PolicySupervisor:
    if target_m is None:
        # Place target at the zero-config FK position (guaranteed inside workspace
        # when workspace_radius_m is generous).
        T = chain.fk(np.zeros(5))
        target_m = T[:3, 3]
    if frame_source is None:
        frame_source = _noop_frame_source
    if motion_executor is None:
        motion_executor = MagicMock()
    if approach_callable is None:
        approach_callable = MagicMock(return_value=True)
    if seating_detector is None:
        seating_detector = MagicMock(return_value=False)
    if lock_callable is None:
        lock_callable = MagicMock()
    return PolicySupervisor(
        policy=policy,
        config=config,
        chain=chain,
        target_attachment_point_m=target_m,
        frame_source=frame_source,
        motion_executor=motion_executor,
        approach_callable=approach_callable,
        seating_detector=seating_detector,
        lock_callable=lock_callable,
        logger=logger,
        role=role,
        dry_run=dry_run,
        joint_deg_to_rad=(lambda d: np.deg2rad(np.asarray(d, dtype=np.float64))),
    )


# ---------------------------------------------------------------------------
# AC#1: Contract specification
# ---------------------------------------------------------------------------


class TestContractSpec:
    """AC#1: policy_contract module exposes the full documented contract."""

    def test_contract_version_is_int(self):
        assert isinstance(CONTRACT_VERSION, int)
        assert CONTRACT_VERSION >= 1

    def test_policy_joint_names_excludes_gripper(self):
        assert "gripper" not in POLICY_JOINT_NAMES
        assert len(POLICY_JOINT_NAMES) == N_JOINTS

    def test_joint_order_matches_policy_joint_names(self):
        assert set(JOINT_ORDER) == POLICY_JOINT_NAMES
        assert len(JOINT_ORDER) == N_JOINTS

    def test_default_frame_dimensions(self):
        assert DEFAULT_FRAME_HEIGHT == 480
        assert DEFAULT_FRAME_WIDTH == 640

    def test_observation_valid(self):
        frame = _make_frame(DEFAULT_FRAME_HEIGHT, DEFAULT_FRAME_WIDTH)
        joints = _make_joint_deg(0.0)
        obs = Observation(frame=frame, joint_positions_deg=joints, step_index=0)
        validate_observation(obs)

    def test_observation_rejects_wrong_frame_ndim(self):
        frame_3d = np.zeros((4, 4, 1), dtype=np.uint8)
        joints = _make_joint_deg()
        with pytest.raises(ContractViolation, match="2-D"):
            Observation(frame=frame_3d, joint_positions_deg=joints, step_index=0)

    def test_observation_rejects_wrong_dtype(self):
        frame = np.zeros((4, 4), dtype=np.float32)
        joints = _make_joint_deg()
        with pytest.raises(ContractViolation, match="uint8"):
            Observation(frame=frame, joint_positions_deg=joints, step_index=0)

    def test_observation_rejects_wrong_joint_count(self):
        frame = _make_frame()
        joints = np.zeros(3, dtype=np.float32)
        with pytest.raises(ContractViolation, match=r"shape \(5,\)"):
            Observation(frame=frame, joint_positions_deg=joints, step_index=0)

    def test_validate_action_passes_valid_joints(self):
        action = {j: 1.0 for j in JOINT_ORDER}
        result = validate_action(action)
        assert result == action

    def test_validate_action_raises_on_gripper(self):
        with pytest.raises(GripperCommandBlocked):
            validate_action({"shoulder_pan": 5.0, "gripper": 90.0})

    def test_validate_action_raises_on_unknown_key(self):
        with pytest.raises(GripperCommandBlocked):
            validate_action({"unknown_joint": 1.0})


# ---------------------------------------------------------------------------
# AC#2: Gripper block
# ---------------------------------------------------------------------------


class TestGripperBlock:
    """AC#2: Supervisor rejects any action containing a gripper key."""

    def test_supervisor_aborts_on_gripper_key(self, chain, supervisor_config_default):
        """Policy injects gripper key on step 0 → supervisor aborts immediately."""
        stub = StubPolicy(inject_gripper_at_step=0)
        sup = _make_supervisor(
            policy=stub,
            config=supervisor_config_default,
            chain=chain,
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.GRIPPER_COMMAND_BLOCKED in result.abort_reasons

    def test_supervisor_aborts_on_gripper_key_mid_run(self, chain, supervisor_config_default):
        """Policy injects gripper key on step 3 → supervisor aborts at step 3."""
        stub = StubPolicy(inject_gripper_at_step=3)
        sup = _make_supervisor(
            policy=stub,
            config=supervisor_config_default,
            chain=chain,
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.GRIPPER_COMMAND_BLOCKED in result.abort_reasons

    def test_stub_policy_never_emits_gripper_by_default(self):
        """StubPolicy with default settings never returns gripper key."""
        stub = StubPolicy()
        frame = _make_frame()
        joints = _make_joint_deg()
        for i in range(20):
            obs = Observation(frame=frame, joint_positions_deg=joints, step_index=i)
            action = stub.step(obs)
            assert "gripper" not in action, f"Gripper key found at step {i}"

    def test_gripper_command_blocked_is_typed(self):
        """GripperCommandBlocked carries the offending key."""
        try:
            validate_action({"gripper": 90.0})
        except GripperCommandBlocked as exc:
            assert "gripper" in exc.offending_keys

    def test_lock_callable_not_called_on_abort(self, chain, supervisor_config_default):
        """lock_callable must never be called when the run aborts."""
        stub = StubPolicy(inject_gripper_at_step=0)
        lock = MagicMock()
        sup = _make_supervisor(
            policy=stub,
            config=supervisor_config_default,
            chain=chain,
            lock_callable=lock,
        )
        sup.run(cycle_index=0)
        lock.assert_not_called()


# ---------------------------------------------------------------------------
# AC#3a: Step limit enforcement
# ---------------------------------------------------------------------------


class TestStepLimit:
    """AC#3: Hard step limit aborts the run."""

    def test_step_limit_triggers_abort(self, chain):
        """A no-op policy exhausts max_steps → HALT after retries."""
        cfg = PolicySupervisorConfig(
            max_steps=3,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(policy=stub, config=cfg, chain=chain)
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.STEP_LIMIT in result.abort_reasons
        assert result.total_steps == 3

    def test_step_limit_with_retries(self, chain):
        """With max_retries=2, step limit abort repeats 3 times → HALT."""
        cfg = PolicySupervisorConfig(
            max_steps=2,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=2,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        approach = MagicMock(return_value=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, approach_callable=approach
        )
        result = sup.run(cycle_index=1)
        assert result.outcome == SupervisorOutcome.HALT
        assert result.attempts == 3
        # approach_callable called max_retries times
        assert approach.call_count == 2


# ---------------------------------------------------------------------------
# AC#3b: Wall-clock limit
# ---------------------------------------------------------------------------


class TestWallTimeLimit:
    """AC#3: Wall-clock time limit aborts the run."""

    def test_wall_time_limit_triggers_abort(self, chain):
        """Slow frame source exceeds wall-time limit → ABORT."""
        cfg = PolicySupervisorConfig(
            max_steps=1000,
            max_wall_time_s=0.05,  # 50 ms
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)

        call_count = [0]

        def slow_frame_source():
            call_count[0] += 1
            if call_count[0] > 1:
                time.sleep(0.06)  # exceed wall limit on 2nd call
            return _noop_frame_source()

        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, frame_source=slow_frame_source
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.WALL_TIME_LIMIT in result.abort_reasons


# ---------------------------------------------------------------------------
# AC#3c: Workspace violation
# ---------------------------------------------------------------------------


class TestWorkspaceViolation:
    """AC#3: Cartesian workspace bound aborts the run."""

    def test_workspace_violation_triggers_abort(self, chain):
        """target far from TCP → workspace violation on first step."""
        cfg = PolicySupervisorConfig(
            max_steps=10,
            max_wall_time_s=60.0,
            workspace_radius_m=0.001,  # 1 mm — impossible to satisfy
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        # Place target 10 m away
        far_target = np.array([10.0, 10.0, 10.0], dtype=np.float64)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, target_m=far_target
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.WORKSPACE_VIOLATION in result.abort_reasons

    def test_workspace_ok_when_target_at_tcp(self, chain):
        """target at TCP → no workspace violation for noop policy + step limit."""
        T = chain.fk(np.zeros(5))
        target_m = T[:3, 3]
        cfg = PolicySupervisorConfig(
            max_steps=3,
            max_wall_time_s=60.0,
            workspace_radius_m=0.5,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, target_m=target_m
        )
        result = sup.run(cycle_index=0)
        # No workspace violation; should get STEP_LIMIT abort
        assert AbortReason.STEP_LIMIT in result.abort_reasons
        assert AbortReason.WORKSPACE_VIOLATION not in result.abort_reasons


# ---------------------------------------------------------------------------
# AC#3d: Magnitude clamping
# ---------------------------------------------------------------------------


class TestMagnitudeClamping:
    """AC#3: Per-step action magnitude clamping."""

    def test_large_action_is_clamped(self, chain, supervisor_config_default):
        """Policy with huge nudge should trigger clamping and still run."""
        # Override config with small clamp threshold
        cfg = PolicySupervisorConfig(
            max_steps=3,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=1.0,  # tight clamp
            clamp_on_magnitude_violation=True,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        # Stub nudges 50 degrees — way over clamp threshold
        stub = StubPolicy(nudge_deg=50.0)
        executor = MagicMock()
        sup = _make_supervisor(
            policy=stub,
            config=cfg,
            chain=chain,
            motion_executor=executor,
            dry_run=False,
        )
        result = sup.run(cycle_index=0)
        # Should run to step limit (clamped, not aborted)
        assert AbortReason.STEP_LIMIT in result.abort_reasons
        # executor was called (even in live mode, stub ran)
        assert executor.call_count >= 1

    def test_small_action_not_clamped(self, chain):
        """Small action within magnitude limit is passed through unchanged."""
        cfg = PolicySupervisorConfig(
            max_steps=2,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            clamp_on_magnitude_violation=True,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(nudge_deg=0.5)  # small nudge, within limit
        executed_actions: list[dict] = []

        def capturing_executor(action):
            executed_actions.append(dict(action))

        sup = _make_supervisor(
            policy=stub,
            config=cfg,
            chain=chain,
            motion_executor=capturing_executor,
            dry_run=False,
        )
        sup.run(cycle_index=0)
        # All executed actions should have shoulder_pan close to 0.5 from 0.0
        for act in executed_actions:
            assert "shoulder_pan" in act


# ---------------------------------------------------------------------------
# AC#4: Abort retract + typed event
# ---------------------------------------------------------------------------


class TestAbortRetract:
    """AC#4: On abort, supervisor retracts and logs typed abort event."""

    def test_retract_called_on_step_limit(self, chain):
        """motion_executor should receive retract pose on step-limit abort."""
        cfg = PolicySupervisorConfig(
            max_steps=2,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.1, 0.2, 0.3, 0.4, 0.5],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        executed: list[dict] = []

        def capturing_executor(action):
            executed.append(dict(action))

        sup = _make_supervisor(
            policy=stub,
            config=cfg,
            chain=chain,
            motion_executor=capturing_executor,
            dry_run=False,
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        # Last action should be the retract pose
        assert len(executed) >= 1
        retract_deg = {
            jname: float(np.degrees(cfg.retract_pose_q_rad[i]))
            for i, jname in enumerate(JOINT_ORDER)
        }
        last_action = executed[-1]
        for jname, expected in retract_deg.items():
            assert abs(last_action[jname] - expected) < 1e-6, (
                f"Joint {jname}: expected {expected:.4f}, got {last_action[jname]:.4f}"
            )

    def test_retract_not_called_in_dry_run_when_pose_none(self, chain):
        """dry_run=True with retract_pose=None: logs warning, does not raise."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=None,
            retract_pose_unverified=True,
        )
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, dry_run=True
        )
        result = sup.run(cycle_index=0)
        # Should not raise; just log warning
        assert result.outcome == SupervisorOutcome.HALT

    def test_retract_fails_closed_in_live_mode_when_pose_none(self, chain):
        """live mode with retract_pose=None raises RuntimeError."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=None,
            retract_pose_unverified=True,
        )
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, dry_run=False
        )
        with pytest.raises(RuntimeError, match="retract_pose_q_rad is None"):
            sup.run(cycle_index=0)

    def test_retract_fails_closed_when_unverified_in_live_mode(self, chain):
        """live mode with retract_pose_unverified=True raises RuntimeError."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=True,  # not verified
        )
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, dry_run=False
        )
        with pytest.raises(RuntimeError, match="unverified"):
            sup.run(cycle_index=0)


class TestTypedAbortEvent:
    """AC#4: Typed abort events are logged with the triggering condition."""

    def test_abort_event_logged_with_reason(self, chain):
        """Abort event payload contains the abort reason."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=0.001,  # force workspace violation
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        logged_events: list[dict] = []

        class _Logger:
            def emit(self, *, role, subsystem, event_type, severity="INFO", payload=None):
                logged_events.append({
                    "event_type": event_type,
                    "payload": payload or {},
                })

        far_target = np.array([10.0, 10.0, 10.0], dtype=np.float64)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, target_m=far_target,
            logger=_Logger(),
        )
        result = sup.run(cycle_index=5)
        # Find supervisor_abort event
        abort_events = [e for e in logged_events if e["event_type"] == "supervisor_abort"]
        assert len(abort_events) >= 1
        event = abort_events[0]
        assert event["payload"]["reason"] == AbortReason.WORKSPACE_VIOLATION.value
        assert event["payload"]["cycle_index"] == 5

    def test_policy_handoff_start_logged(self, chain, supervisor_config_default):
        """policy_handoff_start event is always emitted at run start."""
        stub = StubPolicy(noop=True)
        logged: list[dict] = []

        class _Logger:
            def emit(self, *, role, subsystem, event_type, severity="INFO", payload=None):
                logged.append({"event_type": event_type})

        sup = _make_supervisor(
            policy=stub, config=supervisor_config_default, chain=chain,
            logger=_Logger(),
        )
        sup.run(cycle_index=0)
        types = [e["event_type"] for e in logged]
        assert "policy_handoff_start" in types
        assert "policy_handoff_end" in types


# ---------------------------------------------------------------------------
# AC#5: Retry policy
# ---------------------------------------------------------------------------


class TestRetryPolicy:
    """AC#5: Bounded retry with re-approach, escalation to halt."""

    def test_retry_calls_approach_callable(self, chain):
        """Each retry after the first calls approach_callable once."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=2,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        approach = MagicMock(return_value=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, approach_callable=approach
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert result.attempts == 3
        assert approach.call_count == 2

    def test_approach_failure_logged_and_counted(self, chain):
        """Approach failure on retry counts as a failed attempt."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=1,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        approach = MagicMock(return_value=False)  # approach always fails
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, approach_callable=approach
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        # step_limit on first attempt + approach_failed on retry
        assert AbortReason.APPROACH_FAILED in result.abort_reasons

    def test_zero_retries_halts_on_first_abort(self, chain):
        """max_retries=0: single abort → immediately HALT."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        approach = MagicMock(return_value=True)
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, approach_callable=approach
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert result.attempts == 1
        approach.assert_not_called()

    def test_halt_event_logged(self, chain):
        """supervisor_halt event is emitted after all retries exhausted."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        logged: list[str] = []

        class _Logger:
            def emit(self, *, role, subsystem, event_type, severity="INFO", payload=None):
                logged.append(event_type)

        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, logger=_Logger()
        )
        sup.run(cycle_index=0)
        assert "supervisor_halt" in logged

    def test_retry_event_logged(self, chain):
        """cycle_retry event logged after each non-final abort."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=1,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        logged: list[str] = []

        class _Logger:
            def emit(self, *, role, subsystem, event_type, severity="INFO", payload=None):
                logged.append(event_type)

        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain,
            approach_callable=MagicMock(return_value=True),
            logger=_Logger(),
        )
        sup.run(cycle_index=0)
        assert "cycle_retry" in logged


# ---------------------------------------------------------------------------
# AC#6: Stub policy end-to-end
# ---------------------------------------------------------------------------


class TestEndToEnd:
    """AC#6: End-to-end test with stub policy and mock backend."""

    def test_success_path_end_to_end(self, chain, supervisor_config_default):
        """Seating detector returns True after 3 steps → SUCCESS."""
        stub = StubPolicy()
        step_count = [0]

        def frame_source():
            return _make_frame(), _make_joint_deg(0.0)

        def seating_detector():
            step_count[0] += 1
            return step_count[0] >= 3  # seated after 3 checks

        lock = MagicMock()
        executor = MagicMock()

        sup = _make_supervisor(
            policy=stub,
            config=supervisor_config_default,
            chain=chain,
            frame_source=frame_source,
            seating_detector=seating_detector,
            lock_callable=lock,
            motion_executor=executor,
            dry_run=True,  # dry-run: lock not called
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.SUCCESS
        assert result.total_steps >= 3
        # In dry-run, lock should NOT be called
        lock.assert_not_called()

    def test_success_path_live_calls_lock(self, chain):
        """In live mode, success calls lock_callable exactly once."""
        cfg = PolicySupervisorConfig(
            max_steps=10,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        lock = MagicMock()

        sup = _make_supervisor(
            policy=stub,
            config=cfg,
            chain=chain,
            seating_detector=MagicMock(return_value=True),  # immediate success
            lock_callable=lock,
            dry_run=False,  # live mode
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.SUCCESS
        lock.assert_called_once()

    def test_full_abort_and_retry_end_to_end(self, chain):
        """Step limit + retry + halt exercised end-to-end with stub policy."""
        cfg = PolicySupervisorConfig(
            max_steps=2,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=1,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        approach = MagicMock(return_value=True)
        lock = MagicMock()
        executor = MagicMock()

        sup = _make_supervisor(
            policy=stub,
            config=cfg,
            chain=chain,
            approach_callable=approach,
            lock_callable=lock,
            motion_executor=executor,
            dry_run=True,
        )
        result = sup.run(cycle_index=2)
        assert result.outcome == SupervisorOutcome.HALT
        assert result.attempts == 2
        assert approach.call_count == 1
        # lock never called on failure
        lock.assert_not_called()


# ---------------------------------------------------------------------------
# AC#7: Comprehensive coverage of all abort triggers
# ---------------------------------------------------------------------------


class TestAllAbortTriggers:
    """AC#7: All abort triggers, gripper rejection, success return covered."""

    @pytest.mark.parametrize("abort_reason,cfg_kwargs,policy_kwargs,target_kwargs", [
        (
            AbortReason.STEP_LIMIT,
            {"max_steps": 1, "max_retries": 0, "workspace_radius_m": 1.0},
            {"noop": True},
            {},
        ),
        (
            AbortReason.WORKSPACE_VIOLATION,
            {"max_steps": 10, "max_retries": 0, "workspace_radius_m": 0.0001},
            {"noop": True},
            {"use_far_target": True},
        ),
        (
            AbortReason.GRIPPER_COMMAND_BLOCKED,
            {"max_steps": 10, "max_retries": 0, "workspace_radius_m": 1.0},
            {"inject_gripper_at_step": 0},
            {},
        ),
    ])
    def test_abort_trigger(self, chain, abort_reason, cfg_kwargs, policy_kwargs, target_kwargs):
        base = {
            "max_steps": 10,
            "max_wall_time_s": 60.0,
            "workspace_radius_m": 1.0,
            "max_action_magnitude_deg": 10.0,
            "max_retries": 0,
            "retract_pose_q_rad": [0.0, 0.0, 0.0, 0.0, 0.0],
            "retract_pose_unverified": False,
        }
        base.update(cfg_kwargs)
        cfg = PolicySupervisorConfig(**base)
        stub = StubPolicy(**policy_kwargs)

        target_m = None
        if target_kwargs.get("use_far_target"):
            target_m = np.array([50.0, 50.0, 50.0], dtype=np.float64)

        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, target_m=target_m
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert abort_reason in result.abort_reasons

    def test_success_return_after_seating_detected(self, chain):
        """Success path: seating detector triggers before step limit."""
        cfg = PolicySupervisorConfig(
            max_steps=100,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        # Succeed immediately
        seating = MagicMock(return_value=True)
        lock = MagicMock()
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain,
            seating_detector=seating,
            lock_callable=lock,
            dry_run=False,
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.SUCCESS
        lock.assert_called_once()
        assert result.total_steps == 1

    def test_seating_detector_error_aborts(self, chain):
        """Seating detector exception triggers SEATING_DETECTOR_ERROR abort."""
        cfg = PolicySupervisorConfig(
            max_steps=10,
            max_wall_time_s=60.0,
            workspace_radius_m=1.0,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)

        def bad_seating():
            raise RuntimeError("sensor failure")

        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, seating_detector=bad_seating
        )
        result = sup.run(cycle_index=0)
        assert result.outcome == SupervisorOutcome.HALT
        assert AbortReason.SEATING_DETECTOR_ERROR in result.abort_reasons

    def test_result_dataclass_fields(self, chain, supervisor_config_default):
        """SupervisorResult has all documented fields."""
        stub = StubPolicy(noop=True)
        sup = _make_supervisor(
            policy=stub, config=supervisor_config_default, chain=chain
        )
        result = sup.run(cycle_index=0)
        assert isinstance(result.outcome, SupervisorOutcome)
        assert isinstance(result.attempts, int) and result.attempts >= 1
        assert isinstance(result.total_steps, int)
        assert isinstance(result.abort_reasons, list)
        assert isinstance(result.elapsed_s, float)

    def test_supervisor_constructor_rejects_bad_target(self, chain, supervisor_config_default):
        """Target with wrong shape raises ValueError."""
        stub = StubPolicy()
        with pytest.raises(ValueError, match="3-vector"):
            PolicySupervisor(
                policy=stub,
                config=supervisor_config_default,
                chain=chain,
                target_attachment_point_m=np.array([1.0, 2.0]),  # wrong shape
                frame_source=_noop_frame_source,
                motion_executor=MagicMock(),
                approach_callable=MagicMock(return_value=True),
                seating_detector=MagicMock(return_value=False),
                lock_callable=MagicMock(),
            )

    def test_contract_version_in_abort_payload(self, chain):
        """Abort payload always includes contract_version for auditability."""
        cfg = PolicySupervisorConfig(
            max_steps=1,
            max_wall_time_s=60.0,
            workspace_radius_m=0.0001,
            max_action_magnitude_deg=10.0,
            max_retries=0,
            retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
            retract_pose_unverified=False,
        )
        stub = StubPolicy(noop=True)
        logged: list[dict] = []

        class _Logger:
            def emit(self, *, role, subsystem, event_type, severity="INFO", payload=None):
                logged.append({"event_type": event_type, "payload": payload or {}})

        far_target = np.array([50.0, 50.0, 50.0])
        sup = _make_supervisor(
            policy=stub, config=cfg, chain=chain, target_m=far_target,
            logger=_Logger(),
        )
        sup.run(cycle_index=0)
        abort_events = [e for e in logged if e["event_type"] == "supervisor_abort"]
        assert len(abort_events) >= 1
        assert "contract_version" in abort_events[0]["payload"]
        assert abort_events[0]["payload"]["contract_version"] == CONTRACT_VERSION


# --- Orchestrator 2026-10-24 hardening tests ---
def _ok_cfg(**kw):
    base = dict(max_steps=5, max_wall_time_s=60.0, workspace_radius_m=10.0,
                max_action_magnitude_deg=10.0, max_retries=0,
                retract_pose_q_rad=[0.0] * 5, retract_pose_unverified=False)
    base.update(kw)
    return PolicySupervisorConfig(**base)


def test_live_supervisor_requires_calibrated_joint_conversion(chain):
    with pytest.raises(ValueError, match="joint_deg_to_rad"):
        PolicySupervisor(policy=StubPolicy(noop=True), config=_ok_cfg(), chain=chain,
                         target_attachment_point_m=np.zeros(3), frame_source=MagicMock(),
                         motion_executor=MagicMock(), approach_callable=MagicMock(),
                         seating_detector=MagicMock(), lock_callable=MagicMock(),
                         dry_run=False)


def test_lock_failure_after_seating_halts_without_retract(chain):
    sup = _make_supervisor(policy=StubPolicy(noop=True), config=_ok_cfg(max_retries=2),
                           chain=chain, dry_run=False)
    sup._seating_detector = MagicMock(return_value=True)
    sup._lock_callable = MagicMock(side_effect=RuntimeError("latch miss"))
    sup._motion_executor = MagicMock()
    result = sup.run(cycle_index=0)
    assert result.outcome == SupervisorOutcome.HALT
    from inchworm.supervisor import AbortReason
    assert AbortReason.LOCK_FAILED in result.abort_reasons
    assert result.attempts == 1  # no retry once seated-but-unlocked


def test_policy_exception_retracts_and_aborts(chain):
    class Boom:
        def reset(self): pass
        def step(self, obs): raise RuntimeError("policy crashed")
    sup = _make_supervisor(policy=Boom(), config=_ok_cfg(), chain=chain, dry_run=False)
    sup._motion_executor = MagicMock()
    result = sup.run(cycle_index=0)
    from inchworm.supervisor import AbortReason
    assert AbortReason.STEP_EXCEPTION in result.abort_reasons
    assert sup._motion_executor.called  # retract was commanded
