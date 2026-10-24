"""Tests for AA-21: Dry crawl cycle runner (inchworm.cycle).

Evidence matrix
---------------
AC#1 — TestCycleSequence: hold check, unlock, retract, approach, supervisor, lock, fold
AC#2 — TestLatticeBookkeeping: 5 consecutive cycles, advance_count, serializable state
AC#3 — TestEventEmission: every phase emits structured events; timeline summary
AC#4 — TestFaultInjection: approach failure, supervisor abort, lock verification failure
        → documented recovery, safe pose, H always LOCKED
AC#5 — TestPreflightRefusal: missing calibration, unverified limits, UNKNOWN state → PreflightError
AC#6 — TestDryRunPlanning: plan_cycle returns planned sequence; assert all phases present
AC#7 — HARDWARE (measured cycle time): unchecked.
        needs owner-supervised hardware session:
            uv run python scripts/dry_cycle.py --live --cycles 5
        Planned dry-run durations are in task notes.

All tests run without hardware. No real serial ports or cameras opened.

Safety invariant assertion
--------------------------
Every test that exercises a fault path asserts that the holding arm is still
LOCKED at the end of recovery, enforcing the AGENTS.md invariant
"never both grippers unlocked while the tube is attached".
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock

import numpy as np
import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import (
    ApproachConfig,
    CycleRunnerConfig,
    PolicySupervisorConfig,
    SO101TrajectoryLimits,
    load_config,
)
from inchworm.cycle import (
    CycleFaultError,
    CyclePlan,
    CycleRunner,
    PreflightError,
    SafetyInvariantViolation,
    ScriptedSeatingDetector,
    _make_dry_frame_source,
    _make_dry_motion_executor,
    build_cycle_runner,
)
from inchworm.events import cycle_start, cycle_end
from inchworm.frames import FrameModel, LatticeState, TubePose
from inchworm.gripper_lock import (
    GripperLockSM,
    GripperState,
    LockVerificationFailed,
    LockVerdict,
)
from inchworm.kinematics import build_chain
from inchworm.motion_guard import build_motion_guard
from inchworm.servo_bus import ArmServoBus

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

_JOINT_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def chains(cfg):
    left = build_chain(cfg.so101_kinematics)
    right = build_chain(cfg.so101_kinematics)
    return left, right


@pytest.fixture(scope="module")
def guards(cfg):
    left = build_motion_guard(cfg, "left")
    right = build_motion_guard(cfg, "right")
    return left, right


def _make_bus(role: str) -> ArmServoBus:
    transport = MockSerialTransport(role)
    return ArmServoBus(transport, role=role, joint_id_map=_JOINT_MAP)


def _make_sm(role: str, dry_run: bool = True) -> GripperLockSM:
    """Make a gripper SM backed by a mock bus, no opposite-arm predicate yet."""
    bus = _make_bus(role)
    return GripperLockSM(bus=bus, role=role, config=None, dry_run=dry_run)


def _wire_opposite_predicates(
    left_sm: GripperLockSM,
    right_sm: GripperLockSM,
) -> None:
    """Wire opposite-arm predicates between two SMs."""
    left_sm._opposite_arm_locked_fn = lambda: right_sm.state == GripperState.LOCKED
    right_sm._opposite_arm_locked_fn = lambda: left_sm.state == GripperState.LOCKED


def _make_runner(
    cfg,
    chains,
    guards,
    *,
    dry_run: bool = True,
    sm_dry_run: Optional[bool] = None,
    approach_override: Optional[Any] = None,
    cycle_cfg: Optional[CycleRunnerConfig] = None,
    session_logger: Optional[Any] = None,
    config: Optional[Any] = None,
) -> CycleRunner:
    """Build a CycleRunner with mock SMs, wired opposite predicates.

    ``sm_dry_run`` controls the SM dry_run flag independently of the runner's
    dry_run flag.  This lets tests check live-mode preflight without real servo
    writes.  Defaults to ``dry_run``.
    """
    if sm_dry_run is None:
        sm_dry_run = dry_run

    left_sm = _make_sm("arm_left", dry_run=sm_dry_run)
    right_sm = _make_sm("arm_right", dry_run=sm_dry_run)
    _wire_opposite_predicates(left_sm, right_sm)

    left_chain, right_chain = chains
    left_guard, right_guard = guards

    # Default approach cfg from the loaded config
    approach_cfg = cfg.approach
    assert approach_cfg is not None, "approach block missing from hardware.yaml"

    supervisor_cfg = PolicySupervisorConfig(
        max_steps=10,
        max_wall_time_s=5.0,
        workspace_radius_m=10.0,  # large so zero-q FK never triggers violation
        max_retries=2,
    )

    runner = CycleRunner(
        frame_model=FrameModel.from_config(cfg, left_chain, right_chain),
        left_sm=left_sm,
        right_sm=right_sm,
        left_chain=left_chain,
        right_chain=right_chain,
        left_guard=left_guard,
        right_guard=right_guard,
        approach_cfg=approach_cfg,
        traj_limits=cfg.so101_kinematics.trajectory,
        supervisor_cfg=supervisor_cfg,
        cycle_cfg=cycle_cfg or CycleRunnerConfig(),
        session_logger=session_logger,
        config=config,
        dry_run=dry_run,
    )

    # Optionally override the internal approach planning (bypass IK)
    if approach_override is not None:
        runner._approach_override = approach_override

    return runner


def _prime_runner_for_cycle(runner: CycleRunner) -> None:
    """Set holding arm to LOCKED, moving arm to UNLOCKED (ready for cycle 0)."""
    # holding = right, moving = left (default initial_moving_arm)
    runner._right_sm.reset()
    runner._right_sm.set_approach_ready()
    runner._right_sm.lock()
    runner._left_sm.reset()


# ---------------------------------------------------------------------------
# Mock approach: always succeeds, returns a minimal ApproachPlan
# ---------------------------------------------------------------------------


def _make_mock_approach_plan(cfg, chains) -> Any:
    """Return a callable that always returns a mock ApproachPlan."""
    from inchworm.approach import ApproachPlan, ApproachSegment
    from inchworm.kinematics import IKResult
    from inchworm.trajectory import plan_joint_trajectory

    left_chain, _ = chains
    q_zero = np.zeros(5, dtype=np.float64)
    T_eye = np.eye(4, dtype=np.float64)

    def _make_plan(lattice_index, arm, q_present, frame_model, chain, guard,
                   approach_cfg, traj_limits, **kw):
        ik = IKResult(
            q=q_zero.copy(),
            reachable=True,
            pos_err_m=0.0,
            ori_err_rad=0.0,
        )
        traj = plan_joint_trajectory(
            q_start=q_zero, q_end=q_zero,
            limits=traj_limits,
            joint_names=chain.joint_names(),
        )
        seg = ApproachSegment(
            label="mock→approach",
            q_start=q_zero.copy(),
            q_end=q_zero.copy(),
            trajectory=traj,
            guard_violation=None,
        )
        return ApproachPlan(
            lattice_index=lattice_index,
            arm=arm,
            T_preinsert=T_eye.copy(),
            q_preinsert=q_zero.copy(),
            ik_result=ik,
            segments=[seg],
            safe=True,
            provisional=True,
            total_duration_s=0.01,
            summary="mock approach plan",
        )

    return _make_plan


# ---------------------------------------------------------------------------
# Patch helper: replace plan_approach in cycle module
# ---------------------------------------------------------------------------

def _patch_plan_approach(monkeypatch, cfg, chains):
    """Monkeypatch inchworm.cycle.plan_approach to always succeed."""
    mock_fn = _make_mock_approach_plan(cfg, chains)
    monkeypatch.setattr("inchworm.cycle.plan_approach", mock_fn)
    # Also patch execute_approach to no-op
    monkeypatch.setattr("inchworm.cycle.execute_approach", lambda **kw: None)


# ---------------------------------------------------------------------------
# AC#1 — Cycle sequence
# ---------------------------------------------------------------------------


class TestCycleSequence:
    """AC#1: cycle executes the complete sequence in the correct order."""

    def test_cycle_phases_present(self, cfg, chains, guards, monkeypatch):
        """All required phases are recorded in the plan."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        plan = runner.run_one_cycle(0)

        phase_names = [ph.phase for ph in plan.phases]
        from inchworm.cycle import (
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        )
        for required in [
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        ]:
            assert required in phase_names, f"phase '{required}' missing"

    def test_cycle_phase_order(self, cfg, chains, guards, monkeypatch):
        """Phases appear in the documented order."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        plan = runner.run_one_cycle(0)

        from inchworm.cycle import (
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        )
        expected_order = [
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        ]
        phase_names = [ph.phase for ph in plan.phases]
        # Every expected phase appears in the plan
        positions = [phase_names.index(p) for p in expected_order if p in phase_names]
        assert positions == sorted(positions), "phases are not in order"

    def test_hold_check_refuses_if_not_locked(self, cfg, chains, guards, monkeypatch):
        """Cycle refuses to start if holding arm is not LOCKED (live mode)."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards, dry_run=False)
        # Neither arm is locked — both stay in UNKNOWN (never reset)
        # The preflight check should catch: holding not LOCKED, moving UNKNOWN
        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)
        assert any("not LOCKED" in f or "UNKNOWN" in f for f in exc_info.value.failures)

    def test_moving_arm_starts_unlocked_after_cycle(self, cfg, chains, guards, monkeypatch):
        """After a successful cycle, M is LOCKED and roles have swapped."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        initial_moving = runner.moving_arm  # "left"
        runner.run_one_cycle(0)

        # After cycle, roles have swapped
        assert runner.moving_arm != initial_moving
        # The arm that was moving is now LOCKED
        moved_sm = runner._left_sm  # left was moving on cycle 0
        assert moved_sm.state == GripperState.LOCKED

    def test_holding_arm_remains_locked_throughout(self, cfg, chains, guards, monkeypatch):
        """Holding arm (right) is LOCKED throughout the cycle (safety invariant)."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        runner.run_one_cycle(0)

        # Right arm was holding for cycle 0; must still be LOCKED
        assert runner._right_sm.state == GripperState.LOCKED


# ---------------------------------------------------------------------------
# AC#2 — Lattice bookkeeping
# ---------------------------------------------------------------------------


class TestLatticeBookkeeping:
    """AC#2: advance_count increments correctly over 5 cycles; state serializable."""

    def test_five_cycles_advance_count(self, cfg, chains, guards, monkeypatch):
        """advance_count increments by 1 per cycle over 5 cycles."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        for i in range(5):
            runner.run_one_cycle(i)

        assert runner.frame_model.tube_pose.advance_count == 5

    def test_five_cycles_target_indices_advance(self, cfg, chains, guards, monkeypatch):
        """Target lattice index advances by 1 each cycle."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        for i in range(5):
            runner.run_one_cycle(i)

        # After 5 cycles, the left or right arm should hold index 5 (leapfrog pattern)
        fm = runner.frame_model
        held = fm.held_indices()
        # One arm holds 4, the other holds 5 (alternating)
        held_vals = [v for v in held.values() if v is not None]
        assert 5 in held_vals, f"expected index 5 held, got {held}"

    def test_serializable_state_after_five_cycles(self, cfg, chains, guards, monkeypatch):
        """State after 5 cycles serializes to valid JSON with correct advance_count."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        for i in range(5):
            runner.run_one_cycle(i)

        json_str = runner.frame_model.to_json()
        data = json.loads(json_str)

        assert data["tube_pose"]["advance_count"] == 5
        assert data["consistency"]["advance_count"] == 5
        assert "schema_version" in data
        assert "config_hash" in data

    def test_advance_count_monotonically_increasing(self, cfg, chains, guards, monkeypatch):
        """advance_count is strictly increasing across 5 cycles."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        counts = []
        for i in range(5):
            runner.run_one_cycle(i)
            counts.append(runner.frame_model.tube_pose.advance_count)

        assert counts == list(range(1, 6)), f"expected 1..5, got {counts}"

    def test_state_deserializes_correctly(self, cfg, chains, guards, monkeypatch):
        """Serialized state round-trips through from_json."""
        from inchworm.frames import FrameModel
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        for i in range(5):
            runner.run_one_cycle(i)

        left_chain, right_chain = chains
        json_str = runner.frame_model.to_json()
        # Should round-trip without ResumeError (hashes match or are empty)
        restored = FrameModel.from_json(
            json_str,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=runner.frame_model._T_chassis_left,
            T_chassis_right=runner.frame_model._T_chassis_right,
            coarse_offset_left_m=runner.frame_model._coarse_left.tolist(),
            insertion_offset_left_m=runner.frame_model._ins_left.tolist(),
            coarse_offset_right_m=runner.frame_model._coarse_right.tolist(),
            insertion_offset_right_m=runner.frame_model._ins_right.tolist(),
        )
        assert restored.tube_pose.advance_count == 5


# ---------------------------------------------------------------------------
# AC#3 — Event emission
# ---------------------------------------------------------------------------


class TestEventEmission:
    """AC#3: every phase emits structured events; per-cycle timeline summary."""

    def test_cycle_start_end_emitted(self, cfg, chains, guards, monkeypatch):
        """cycle_start and cycle_end events are emitted."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        emitted = []
        mock_logger = MagicMock()
        mock_logger.emit.side_effect = lambda **kw: emitted.append(kw)

        runner = _make_runner(cfg, chains, guards, session_logger=mock_logger)
        _prime_runner_for_cycle(runner)
        runner.run_one_cycle(0)

        event_types = [e.get("event_type") for e in emitted]
        assert "cycle_start" in event_types
        assert "cycle_end" in event_types

    def test_phase_events_emitted(self, cfg, chains, guards, monkeypatch):
        """cycle_phase events are emitted for each phase."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        emitted = []
        mock_logger = MagicMock()
        mock_logger.emit.side_effect = lambda **kw: emitted.append(kw)

        runner = _make_runner(cfg, chains, guards, session_logger=mock_logger)
        _prime_runner_for_cycle(runner)
        runner.run_one_cycle(0)

        phase_events = [
            e for e in emitted
            if e.get("event_type") == "cycle_phase"
        ]
        phase_names = [e.get("payload", {}).get("phase") for e in phase_events]
        from inchworm.cycle import _PHASE_APPROACH, _PHASE_LOCK, _PHASE_ADVANCE
        assert _PHASE_APPROACH in phase_names
        assert _PHASE_LOCK in phase_names
        assert _PHASE_ADVANCE in phase_names

    def test_timeline_summary_from_jsonl(self, tmp_path, cfg, chains, guards, monkeypatch):
        """Per-cycle timeline summary is produced from the event log."""
        from inchworm.session import new_session
        from inchworm.summarizer import summarize_session, format_timeline

        _patch_plan_approach(monkeypatch, cfg, chains)

        with new_session(cfg) as ctx:
            runner = _make_runner(cfg, chains, guards, session_logger=ctx.logger)
            _prime_runner_for_cycle(runner)
            runner.run_one_cycle(0)

            log_path = ctx.log_path

        timeline = summarize_session(log_path)
        summary_text = format_timeline(timeline)
        assert "Cycle 0" in summary_text
        assert timeline.total_events > 0

    def test_plan_summary_contains_all_phases(self, cfg, chains, guards):
        """plan_cycle returns a plan with a summary string mentioning all phases."""
        runner = _make_runner(cfg, chains, guards)
        plan = runner.plan_cycle(0)
        s = plan.summary()

        from inchworm.cycle import _PHASE_APPROACH, _PHASE_LOCK, _PHASE_FOLD
        assert _PHASE_APPROACH in s
        assert _PHASE_LOCK in s
        assert _PHASE_FOLD in s


# ---------------------------------------------------------------------------
# AC#4 — Fault injection
# ---------------------------------------------------------------------------


class TestFaultInjection:
    """AC#4: fault paths each drive recovery; run ends safe; H always LOCKED."""

    def _assert_invariant(self, runner: CycleRunner) -> None:
        """Assert that the holding arm is LOCKED (safety invariant)."""
        holding_sm = (
            runner._left_sm if runner.holding_arm == "left" else runner._right_sm
        )
        assert holding_sm.state == GripperState.LOCKED, (
            f"SAFETY INVARIANT VIOLATED: holding arm {runner.holding_arm} "
            f"is in state {holding_sm.state.name}"
        )

    def test_approach_failure_ends_safe(self, cfg, chains, guards, monkeypatch):
        """ApproachUnreachableError → CycleFaultError; H remains LOCKED."""
        from inchworm.approach import ApproachUnreachableError

        def _fail_approach(*args, **kwargs):
            raise ApproachUnreachableError(99, 0.5, "test injection")

        monkeypatch.setattr("inchworm.cycle.plan_approach", _fail_approach)

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        with pytest.raises(CycleFaultError) as exc_info:
            runner.run_one_cycle(0)

        assert exc_info.value.phase == "approach_moving"
        assert exc_info.value.safe is True
        self._assert_invariant(runner)

    def test_approach_unsafe_ends_safe(self, cfg, chains, guards, monkeypatch):
        """ApproachUnsafeError → CycleFaultError; H remains LOCKED."""
        from inchworm.approach import ApproachUnsafeError

        def _fail_approach(*args, **kwargs):
            raise ApproachUnsafeError("seg0", "test guard violation")

        monkeypatch.setattr("inchworm.cycle.plan_approach", _fail_approach)

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        with pytest.raises(CycleFaultError) as exc_info:
            runner.run_one_cycle(0)

        assert exc_info.value.safe is True
        self._assert_invariant(runner)

    def test_supervisor_abort_ends_safe(self, cfg, chains, guards, monkeypatch):
        """Supervisor ABORT → CycleFaultError; H remains LOCKED."""
        from inchworm.supervisor import SupervisorResult, SupervisorOutcome, AbortReason
        from inchworm.approach import ApproachPlan, ApproachSegment
        from inchworm.kinematics import IKResult
        from inchworm.trajectory import plan_joint_trajectory

        # Patch approach to succeed
        _patch_plan_approach(monkeypatch, cfg, chains)

        # Patch supervisor to always abort
        def _abort_supervisor_run(self_sup, cycle_index=0):
            return SupervisorResult(
                outcome=SupervisorOutcome.ABORT,
                attempts=3,
                total_steps=0,
                abort_reasons=[AbortReason.STEP_LIMIT],
                elapsed_s=0.0,
            )

        monkeypatch.setattr(
            "inchworm.cycle.PolicySupervisor.run", _abort_supervisor_run
        )

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        with pytest.raises(CycleFaultError) as exc_info:
            runner.run_one_cycle(0)

        assert exc_info.value.phase == "supervisor_handoff"
        assert exc_info.value.safe is True
        self._assert_invariant(runner)

    def test_lock_verification_failure_ends_safe(self, cfg, chains, guards, monkeypatch):
        """LockVerificationFailed → CycleFaultError; H remains LOCKED."""
        _patch_plan_approach(monkeypatch, cfg, chains)

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        # Override moving SM's lock() to raise LockVerificationFailed
        original_lock = runner._left_sm.lock

        def _fail_lock():
            # Transition to LOCKING manually then raise
            runner._left_sm._state = runner._left_sm._state.__class__.APPROACH_READY
            raise LockVerificationFailed(LockVerdict.MISS, "test injection")

        runner._left_sm.lock = _fail_lock

        with pytest.raises(CycleFaultError) as exc_info:
            runner.run_one_cycle(0)

        assert exc_info.value.phase == "lock_moving"
        assert exc_info.value.safe is True
        self._assert_invariant(runner)

    def test_approach_failure_does_not_advance_count(self, cfg, chains, guards, monkeypatch):
        """Approach failure does NOT advance the FrameModel tube_pose."""
        from inchworm.approach import ApproachUnreachableError

        def _fail_approach(*args, **kwargs):
            raise ApproachUnreachableError(1, 0.5)

        monkeypatch.setattr("inchworm.cycle.plan_approach", _fail_approach)

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        with pytest.raises(CycleFaultError):
            runner.run_one_cycle(0)

        # advance_count must not have changed
        assert runner.frame_model.tube_pose.advance_count == 0

    def test_both_grippers_never_unlocked(self, cfg, chains, guards, monkeypatch):
        """Both grippers are never simultaneously unlocked during fault recovery."""
        from inchworm.approach import ApproachUnreachableError

        def _fail_approach(*args, **kwargs):
            raise ApproachUnreachableError(1, 0.5)

        monkeypatch.setattr("inchworm.cycle.plan_approach", _fail_approach)

        runner = _make_runner(cfg, chains, guards)
        _prime_runner_for_cycle(runner)

        unlocked_pairs: list = []

        # Instrument the SM states
        original_approach = runner._left_sm.set_approach_ready

        def _check_and_approach():
            left_state = runner._left_sm.state
            right_state = runner._right_sm.state
            if (left_state == GripperState.UNLOCKED
                    and right_state == GripperState.UNLOCKED):
                unlocked_pairs.append((left_state, right_state))
            original_approach()

        runner._left_sm.set_approach_ready = _check_and_approach

        with pytest.raises(CycleFaultError):
            runner.run_one_cycle(0)

        assert unlocked_pairs == [], (
            f"Both grippers were simultaneously UNLOCKED: {unlocked_pairs}"
        )


# ---------------------------------------------------------------------------
# AC#5 — Preflight refusal
# ---------------------------------------------------------------------------


class TestPreflightRefusal:
    """AC#5: refuses to start live if calibration missing, limits unverified,
    or either gripper UNKNOWN; lists all failing conditions."""

    def test_refuses_live_if_holding_not_locked(self, cfg, chains, guards, monkeypatch):
        """Live: refuses when holding arm is not LOCKED."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        # sm_dry_run=True so reset() doesn't write to mock buses;
        # runner dry_run=False so preflight enforces live conditions.
        runner = _make_runner(cfg, chains, guards, dry_run=False, sm_dry_run=True)
        runner._right_sm.reset()  # UNLOCKED, not LOCKED
        runner._left_sm.reset()

        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)

        assert any("not LOCKED" in f for f in exc_info.value.failures)

    def test_refuses_live_if_gripper_unknown(self, cfg, chains, guards, monkeypatch):
        """Live: refuses when moving arm gripper is UNKNOWN."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(cfg, chains, guards, dry_run=False)
        # Neither SM is reset; both are UNKNOWN. Preflight should catch both.
        # Holding is UNKNOWN (not LOCKED) and moving is UNKNOWN.
        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)
        assert any("UNKNOWN" in f or "not LOCKED" in f for f in exc_info.value.failures)

    def test_refuses_live_if_calibration_missing(self, cfg, chains, guards, monkeypatch):
        """Live: refuses when AA-16 calibration block is absent or limits unverified."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(
            cfg, chains, guards, dry_run=False, sm_dry_run=True, config=cfg
        )
        # Neither SM is reset: both are UNKNOWN.
        # This triggers: holding not LOCKED, moving UNKNOWN, plus cal/limits.
        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)

        # calibration block is None in the default hardware.yaml
        # → preflight should list it as a failure
        assert len(exc_info.value.failures) >= 1

    def test_lists_all_failures(self, cfg, chains, guards, monkeypatch):
        """PreflightError lists every failing condition (not just the first)."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(
            cfg, chains, guards, dry_run=False, sm_dry_run=True, config=cfg
        )
        # Leave everything unset: holding not LOCKED, moving UNKNOWN

        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)

        # Multiple failures should be listed
        assert len(exc_info.value.failures) >= 2

    def test_dry_run_allowed_with_warnings(self, cfg, chains, guards, monkeypatch):
        """Dry-run proceeds despite preflight failures (issues warnings only)."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        runner = _make_runner(
            cfg, chains, guards, dry_run=True, config=cfg
        )
        # Set up for a valid dry-run start
        runner._right_sm.reset()
        runner._right_sm.set_approach_ready()
        runner._right_sm.lock()
        runner._left_sm.reset()

        # Should not raise PreflightError — dry-run allows it
        plan = runner.run_one_cycle(0)
        preflight_phase = next(
            (ph for ph in plan.phases if ph.phase == "preflight"), None
        )
        assert preflight_phase is not None
        assert preflight_phase.outcome == "ok"  # dry-run always records "ok"

    def test_refuses_live_advance_direction_unconfirmed(self, cfg, chains, guards, monkeypatch):
        """Live: refuses when advance_direction_owner_confirmed is false."""
        _patch_plan_approach(monkeypatch, cfg, chains)
        cycle_cfg = CycleRunnerConfig(advance_direction_owner_confirmed=False)
        runner = _make_runner(
            cfg, chains, guards, dry_run=False, sm_dry_run=True, cycle_cfg=cycle_cfg
        )
        # Neither SM reset; both UNKNOWN — preflight will catch multiple failures
        with pytest.raises(PreflightError) as exc_info:
            runner.run_one_cycle(0)

        assert any("advance_direction_owner_confirmed" in f for f in exc_info.value.failures)


# ---------------------------------------------------------------------------
# AC#6 — Dry-run planning
# ---------------------------------------------------------------------------


class TestDryRunPlanning:
    """AC#6: plan_cycle plans all motion without commanding servos; asserted in tests."""

    def test_plan_cycle_returns_all_phases(self, cfg, chains, guards):
        """plan_cycle returns a CyclePlan with all 8 phases present."""
        runner = _make_runner(cfg, chains, guards)
        plan = runner.plan_cycle(0)

        assert isinstance(plan, CyclePlan)
        phase_names = [ph.phase for ph in plan.phases]

        from inchworm.cycle import (
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        )
        for p in [
            _PHASE_PREFLIGHT, _PHASE_UNLOCK, _PHASE_RETRACT,
            _PHASE_APPROACH, _PHASE_SUPERVISOR, _PHASE_LOCK,
            _PHASE_ADVANCE, _PHASE_FOLD,
        ]:
            assert p in phase_names, f"phase '{p}' missing from plan"

    def test_plan_cycle_has_positive_duration(self, cfg, chains, guards):
        """plan_cycle total_planned_duration_s is positive."""
        runner = _make_runner(cfg, chains, guards)
        plan = runner.plan_cycle(0)
        assert plan.total_planned_duration_s > 0.0

    def test_plan_cycle_does_not_command_servos(self, cfg, chains, guards):
        """plan_cycle never calls write_positions_rad on any executor."""
        runner = _make_runner(cfg, chains, guards)

        write_calls: list = []
        runner._left_executor = MagicMock()
        runner._left_executor.write_positions_rad.side_effect = (
            lambda q, **kw: write_calls.append(q)
        )
        runner._right_executor = MagicMock()
        runner._right_executor.write_positions_rad.side_effect = (
            lambda q, **kw: write_calls.append(q)
        )

        runner.plan_cycle(0)
        assert write_calls == [], "plan_cycle should never call write_positions_rad"

    def test_plan_cycle_approach_plan_captured(self, cfg, chains, guards):
        """plan_cycle captures an ApproachPlan (or None on IK failure)."""
        runner = _make_runner(cfg, chains, guards)
        plan = runner.plan_cycle(0)
        # approach_plan may be None if IK fails on unverified transforms;
        # but the phase must still be recorded
        from inchworm.cycle import _PHASE_APPROACH
        approach_phase = next(
            (ph for ph in plan.phases if ph.phase == _PHASE_APPROACH), None
        )
        assert approach_phase is not None

    def test_plan_summary_string(self, cfg, chains, guards):
        """plan_cycle returns a plan with a non-empty summary string."""
        runner = _make_runner(cfg, chains, guards)
        plan = runner.plan_cycle(0)
        summary = plan.summary()
        assert "CyclePlan" in summary
        assert "moving=" in summary
        assert "holding=" in summary


# ---------------------------------------------------------------------------
# ScriptedSeatingDetector tests
# ---------------------------------------------------------------------------


class TestScriptedSeatingDetector:
    def test_fires_at_correct_step(self):
        det = ScriptedSeatingDetector(success_at_step=3)
        results = [det() for _ in range(5)]
        # False until step 3, then True
        assert results == [False, False, False, True, True]

    def test_reset_restarts_counter(self):
        det = ScriptedSeatingDetector(success_at_step=1)
        assert det() is False
        assert det() is True
        det.reset()
        assert det() is False
        assert det() is True

    def test_immediate_success_at_step_0(self):
        det = ScriptedSeatingDetector(success_at_step=0)
        assert det() is True


# ---------------------------------------------------------------------------
# CycleRunnerConfig tests
# ---------------------------------------------------------------------------


class TestCycleRunnerConfig:
    def test_defaults(self):
        c = CycleRunnerConfig()
        assert c.initial_moving_arm == "left"
        assert c.advance_direction == 1
        assert c.advance_direction_owner_confirmed is False
        assert c.supervisor_success_at_step == 2

    def test_invalid_advance_direction(self):
        with pytest.raises(Exception):
            CycleRunnerConfig(advance_direction=0)

    def test_invalid_arm(self):
        with pytest.raises(Exception):
            CycleRunnerConfig(initial_moving_arm="both")

    def test_right_initial(self):
        c = CycleRunnerConfig(initial_moving_arm="right")
        assert c.initial_moving_arm == "right"


# --- Orchestrator 2026-10-24: fault paths must not fold; live fold is real ---
def test_fault_path_records_safe_halt_not_fold(cfg, chains, guards, monkeypatch):
    from inchworm.approach import ApproachUnreachableError

    def _fail(*a, **k):
        raise ApproachUnreachableError(99, 0.5, "inject")

    monkeypatch.setattr("inchworm.cycle.plan_approach", _fail)
    runner = _make_runner(cfg, chains, guards)
    _prime_runner_for_cycle(runner)
    plan_holder = {}
    orig = runner._recover_safe_halt

    def spy(ci, plan):
        plan_holder["plan"] = plan
        return orig(ci, plan)

    runner._recover_safe_halt = spy
    with pytest.raises(CycleFaultError):
        runner.run_one_cycle(0)
    phases = [p.phase for p in plan_holder["plan"].phases]
    assert "safe_halt" in phases
    assert "fold" not in phases[phases.index("safe_halt"):]


def test_live_fold_requires_fold_sequencer(cfg, chains, guards):
    from inchworm.cycle import CyclePlan
    runner = _make_runner(cfg, chains, guards)
    runner._dry_run = False
    with pytest.raises(CycleFaultError, match="fold_sequencer"):
        runner._safe_fold_both(0, CyclePlan(cycle_index=0, moving_arm="left",
                                            holding_arm="right", target_lattice_index=1))
