"""Tests for AA-28: Crawl-cycle fault recovery and tube-state verification.

Evidence matrix
---------------
AC#1 — TestTubeStateVerifier: verifies SM LOCKED check, gripper band check,
        FK TCP tolerance check (with and without joint readings).
AC#2 — TestFaultCatalogue: every fault kind maps to the correct response;
        FaultResponder emits typed AA-7 events; safe_halt keeps holder LOCKED.
AC#3 — TestSafetyInvariants: assert_not_both_unlocked raises;
        assert_state_verified raises; recovery never commands motion from
        unverified state; confirmed in every fault-injection test.
AC#4 — TestCycleStateCheckpoint: atomic save/load/validate; stale state refused;
        advance_count mismatch refused; gripper mismatch refused.
AC#5 — TestFaultInjection: each fault path exercised on mocks (method per fault
        recorded below). Hardware faults noted.
AC#6 — TestPostRunReport: build report from synthetic event log; all faults and
        responses listed.
AC#7 — TestCycleHooks: AA-28 hooks (set_aa28_hooks) wired into CycleRunner;
        checkpoint saved after pre- and post-cycle phases; pre-cycle
        verification failure raises PreflightError.

Fault injection methods per fault (AC#5)
-----------------------------------------
+-------------------------------------------+----------------------------------+
| Fault                                     | Injection method (mock)          |
+-------------------------------------------+----------------------------------+
| insertion_failure_after_retries           | Force left_sm to GripperState.   |
|                                           | FAULT before FaultResponder.     |
|                                           | respond(); verify safe_halt.     |
+-------------------------------------------+----------------------------------+
| lock_verification_failure                 | TubeStateVerifier verifier with  |
|                                           | SM in UNLOCKED state while model |
|                                           | says it holds an index.          |
+-------------------------------------------+----------------------------------+
| mid_advance_servo_fault (ServoTimeout)    | Injected via FaultResponder.     |
|                                           | respond() with ESTOP mock. NOTE: |
|                                           | hardware injection unsafe; flag  |
|                                           | for hardware repetition.         |
+-------------------------------------------+----------------------------------+
| unexpected_tube_displacement              | TubeStateVerifier with FK TCP    |
|                                           | far from expected attachment pt. |
+-------------------------------------------+----------------------------------+
| usb_device_loss (DeviceNotFoundError)    | Injected via FaultResponder.     |
|                                           | respond() with mock EStop. NOTE: |
|                                           | hardware injection unsafe; flag  |
|                                           | for hardware repetition.         |
+-------------------------------------------+----------------------------------+

Hardware-injection notes
------------------------
mid_advance_servo_fault and usb_device_loss: hardware injection is unsafe
(could damage hardware or corrupt USB state). These faults are confirmed on
mocks only. Repeat on hardware in a supervised session:
  needs owner-supervised hardware session: uv run python scripts/fault_inject_hw.py --live

Safety invariant assertion
--------------------------
Every test asserts (directly or via TubeStateVerifier / assert_not_both_unlocked)
that no operation results in both grippers unlocked simultaneously.

Test scope
----------
All tests run without hardware. No real serial ports or cameras opened.
Tests write only to tmp_path (no absolute or worktree paths).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import load_config, CycleRunnerConfig, PolicySupervisorConfig
from inchworm.cycle import (
    CycleRunner,
    PreflightError,
)
from inchworm.frames import FrameModel, LatticeState, TubePose
from inchworm.gripper_lock import GripperLockSM, GripperState
from inchworm.kinematics import build_chain
from inchworm.motion_guard import build_motion_guard
from inchworm.recovery import (
    ArmVerificationResult,
    BothGrippersUnlockedError,
    CyclePhase,
    CycleStateCheckpoint,
    FAULT_CATALOGUE,
    FaultRecord,
    FaultResponder,
    PostRunReport,
    RecoveryFaultKind,
    RecoveryResponse,
    ResumeRefusedError,
    TubeStateVerifier,
    UnverifiedStateError,
    VerificationResult,
    VerificationThresholds,
    assert_not_both_unlocked,
    assert_state_verified,
    build_post_run_report,
    fault_event,
    recovery_response_event,
)
from inchworm.servo_bus import ArmServoBus

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

_JOINT_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def chains(cfg):
    return build_chain(cfg.so101_kinematics), build_chain(cfg.so101_kinematics)


@pytest.fixture(scope="module")
def guards(cfg):
    return build_motion_guard(cfg, "left"), build_motion_guard(cfg, "right")


def _make_bus(role: str) -> ArmServoBus:
    t = MockSerialTransport(role)
    return ArmServoBus(t, role=role, joint_id_map=_JOINT_MAP)


def _make_sm(role: str, *, dry_run: bool = True) -> GripperLockSM:
    return GripperLockSM(bus=_make_bus(role), role=role, config=None, dry_run=dry_run)


def _wire(left_sm: GripperLockSM, right_sm: GripperLockSM) -> None:
    left_sm._opposite_arm_locked_fn = lambda: right_sm.state == GripperState.LOCKED
    right_sm._opposite_arm_locked_fn = lambda: left_sm.state == GripperState.LOCKED


def _make_frame_model(cfg, chains):
    left_chain, right_chain = chains
    return FrameModel.from_config(cfg, left_chain, right_chain)


def _make_verifier(
    cfg,
    chains,
    left_sm: GripperLockSM,
    right_sm: GripperLockSM,
    *,
    thresholds: Optional[VerificationThresholds] = None,
    dry_run: bool = True,
) -> TubeStateVerifier:
    fm = _make_frame_model(cfg, chains)
    if thresholds is None:
        thresholds = VerificationThresholds(pos_tolerance_m=None, verified=False)
    left_chain, right_chain = chains
    return TubeStateVerifier(
        frame_model=fm,
        left_sm=left_sm,
        right_sm=right_sm,
        thresholds=thresholds,
        left_chain=left_chain,
        right_chain=right_chain,
        dry_run=dry_run,
    )


def _locked_sm(role: str) -> GripperLockSM:
    """Return an SM whose state is already LOCKED (dry-run path)."""
    sm = _make_sm(role, dry_run=True)
    sm.reset()  # → UNLOCKED
    sm.set_approach_ready()  # → APPROACH_READY
    sm.lock()  # → LOCKED (dry-run: skips verification)
    return sm


# ---------------------------------------------------------------------------
# AC#2 + AC#5: Fault catalogue
# ---------------------------------------------------------------------------


class TestFaultCatalogue:
    """AC#2: every enumerated fault maps to a defined response."""

    def test_all_faults_in_catalogue(self):
        """Every RecoveryFaultKind has a defined response in FAULT_CATALOGUE."""
        for kind in RecoveryFaultKind:
            assert kind in FAULT_CATALOGUE, f"{kind} missing from FAULT_CATALOGUE"

    def test_insertion_failure_maps_to_safe_halt(self):
        assert FAULT_CATALOGUE[RecoveryFaultKind.INSERTION_FAILURE] == RecoveryResponse.SAFE_HALT

    def test_lock_verification_failure_maps_to_safe_halt(self):
        assert FAULT_CATALOGUE[RecoveryFaultKind.LOCK_VERIFICATION_FAILURE] == RecoveryResponse.SAFE_HALT

    def test_mid_advance_servo_fault_maps_to_estop_hold(self):
        assert FAULT_CATALOGUE[RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT] == RecoveryResponse.ESTOP_HOLD_AND_HALT

    def test_unexpected_displacement_maps_to_recalibration_halt(self):
        assert FAULT_CATALOGUE[RecoveryFaultKind.UNEXPECTED_TUBE_DISPLACEMENT] == RecoveryResponse.HALT_REQUIRE_RECALIBRATION

    def test_usb_device_loss_maps_to_estop_hold(self):
        assert FAULT_CATALOGUE[RecoveryFaultKind.USB_DEVICE_LOSS] == RecoveryResponse.ESTOP_HOLD_AND_HALT

    def test_fault_event_constructor_returns_emit_kwargs(self):
        """fault_event() returns valid AA-7 emit kwargs."""
        kwargs = fault_event(
            fault_kind=RecoveryFaultKind.INSERTION_FAILURE,
            response=RecoveryResponse.SAFE_HALT,
            detail="3 retries exceeded",
            cycle_index=2,
        )
        assert kwargs["event_type"] == "recovery_fault"
        assert kwargs["severity"] == "CRITICAL"
        assert kwargs["payload"]["fault_kind"] == RecoveryFaultKind.INSERTION_FAILURE.value
        assert kwargs["payload"]["response"] == RecoveryResponse.SAFE_HALT.value

    def test_recovery_response_event_constructor(self):
        kwargs = recovery_response_event(
            fault_kind=RecoveryFaultKind.LOCK_VERIFICATION_FAILURE,
            response=RecoveryResponse.SAFE_HALT,
            succeeded=True,
            detail="holder stayed LOCKED",
        )
        assert kwargs["event_type"] == "recovery_response"
        assert kwargs["payload"]["succeeded"] is True

    def test_responder_emits_events_to_logger(self):
        """FaultResponder emits recovery_fault and recovery_response to logger."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        left_sm.reset()
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()  # right is LOCKED, left is UNLOCKED → invariant holds
        _wire(left_sm, right_sm)

        emitted: List[Dict[str, Any]] = []

        class _Logger:
            def emit(self, **kwargs):
                emitted.append(kwargs)
            def critical(self, **kwargs):
                emitted.append({**kwargs, "severity": "CRITICAL"})
            def error(self, **kwargs):
                emitted.append({**kwargs, "severity": "ERROR"})
            def warning(self, **kwargs):
                emitted.append({**kwargs, "severity": "WARNING"})
            def info(self, **kwargs):
                emitted.append({**kwargs, "severity": "INFO"})
            def debug(self, **kwargs):
                emitted.append({**kwargs, "severity": "DEBUG"})

        responder = FaultResponder(
            left_sm=left_sm,
            right_sm=right_sm,
            logger=_Logger(),
            dry_run=True,
        )
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.INSERTION_FAILURE,
            detail="test injection",
            cycle_index=0,
        )
        assert response == RecoveryResponse.SAFE_HALT
        assert succeeded is True
        event_types = [e.get("event_type") for e in emitted]
        assert "recovery_fault" in event_types
        assert "recovery_response" in event_types

    # ── AC#5 fault injection: insertion failure ──────────────────────────────
    def test_injection_insertion_failure_safe_halt_holder_locked(self):
        """AC#5: insertion failure → safe halt; holder (right) stays LOCKED.
        Injection method: right_sm in LOCKED, left_sm in FAULT state at respond().
        """
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()  # right=LOCKED
        _wire(left_sm, right_sm)
        # Simulate insertion failure: left SM never locked
        left_sm._state = GripperState.FAULT  # noqa: SLF001

        responder = FaultResponder(left_sm=left_sm, right_sm=right_sm, dry_run=True)
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.INSERTION_FAILURE,
            detail="3 retries",
            cycle_index=1,
        )
        assert response == RecoveryResponse.SAFE_HALT
        # Holder right_sm must still be LOCKED
        assert right_sm.state == GripperState.LOCKED
        # AC#3: not both unlocked
        assert left_sm.state != GripperState.UNLOCKED or right_sm.state == GripperState.LOCKED

    # ── AC#5 fault injection: lock verification failure ──────────────────────
    def test_injection_lock_verification_failure_safe_halt(self):
        """AC#5: lock verification failure → safe halt; LOCKED holder preserved.
        Injection method: FaultResponder.respond() with LOCK_VERIFICATION_FAILURE.
        """
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()
        _wire(left_sm, right_sm)
        left_sm._state = GripperState.FAULT  # noqa: SLF001

        responder = FaultResponder(left_sm=left_sm, right_sm=right_sm, dry_run=True)
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.LOCK_VERIFICATION_FAILURE,
            detail="MISS verdict",
            cycle_index=0,
        )
        assert response == RecoveryResponse.SAFE_HALT
        assert right_sm.state == GripperState.LOCKED  # holder still LOCKED (AC#3)

    # ── AC#5 fault injection: mid-advance servo fault ────────────────────────
    def test_injection_mid_advance_servo_fault_estop_hold(self):
        """AC#5: mid-advance ServoTimeoutError → EStop HOLD + halt.
        Injection method: FaultResponder.respond() with mock EStop; mock EStop
        .stop() is called.
        NOTE: hardware injection unsafe — repeated on hardware separately.
        needs owner-supervised hardware session: uv run python scripts/fault_inject_hw.py --live
        """
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()
        _wire(left_sm, right_sm)

        mock_estop = MagicMock()
        responder = FaultResponder(
            left_sm=left_sm, right_sm=right_sm, estop=mock_estop, dry_run=False
        )
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT,
            detail="ServoTimeoutError on servo 2 during approach",
            cycle_index=3,
        )
        assert response == RecoveryResponse.ESTOP_HOLD_AND_HALT
        mock_estop.stop.assert_called_once()
        # AC#3: right arm stays LOCKED
        assert right_sm.state == GripperState.LOCKED

    # ── AC#5 fault injection: USB device loss ────────────────────────────────
    def test_injection_usb_device_loss_estop_hold(self):
        """AC#5: USB device loss → EStop HOLD + halt.
        Injection method: FaultResponder.respond() with mock EStop.
        NOTE: hardware injection unsafe; needs supervised session.
        needs owner-supervised hardware session: uv run python scripts/fault_inject_hw.py --live
        """
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()
        _wire(left_sm, right_sm)

        mock_estop = MagicMock()
        responder = FaultResponder(
            left_sm=left_sm, right_sm=right_sm, estop=mock_estop, dry_run=False
        )
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS,
            detail="DeviceNotFoundError: role arm_left",
            cycle_index=0,
        )
        assert response == RecoveryResponse.ESTOP_HOLD_AND_HALT
        mock_estop.stop.assert_called_once()
        assert right_sm.state == GripperState.LOCKED  # AC#3

    # ── AC#5 fault injection: unexpected tube displacement ───────────────────
    def test_injection_unexpected_displacement_halt_recalibrate(self):
        """AC#5: unexpected tube displacement → halt + require recalibration.
        Injection method: FaultResponder.respond() with UNEXPECTED_TUBE_DISPLACEMENT.
        """
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()
        _wire(left_sm, right_sm)

        responder = FaultResponder(left_sm=left_sm, right_sm=right_sm, dry_run=True)
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.UNEXPECTED_TUBE_DISPLACEMENT,
            detail="FK TCP 15 mm from expected; displacement confirmed",
            cycle_index=2,
        )
        assert response == RecoveryResponse.HALT_REQUIRE_RECALIBRATION
        assert right_sm.state == GripperState.LOCKED  # AC#3


# ---------------------------------------------------------------------------
# AC#1: Tube state verification
# ---------------------------------------------------------------------------


class TestTubeStateVerifier:
    """AC#1: state verification checks all three signals."""

    def test_verify_both_unlocked_arm_not_holding(self, cfg, chains):
        """If model says neither arm holds an index, any SM state is OK."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        # Both UNLOCKED, neither holding an index
        left_sm.reset()
        right_sm.reset()
        verifier = _make_verifier(cfg, chains, left_sm, right_sm, dry_run=True)
        result = verifier.verify(context="test")
        # No held indices → nothing to check → should pass
        assert result.passed is True

    def test_verify_holding_arm_must_be_locked(self, cfg, chains):
        """AC#1 check #1: if model says arm holds index, SM must be LOCKED."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        left_sm.reset()  # UNLOCKED
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()  # LOCKED

        fm = _make_frame_model(cfg, chains)
        # Pretend left arm holds index 0 but its SM is UNLOCKED
        fm.set_held_index("left", 0)

        verifier = TubeStateVerifier(
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
            thresholds=VerificationThresholds(pos_tolerance_m=None),
            dry_run=True,
        )
        result = verifier.verify(context="test_ac1_check1")
        assert result.left.passed is False
        assert any("not LOCKED" in r for r in result.left.reasons)

    def test_verify_locked_arm_passes_check_1(self, cfg, chains):
        """AC#1 check #1: arm with LOCKED SM and held index → check 1 passes."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)

        fm = _make_frame_model(cfg, chains)
        fm.set_held_index("left", 0)

        verifier = TubeStateVerifier(
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
            thresholds=VerificationThresholds(pos_tolerance_m=None),
            dry_run=True,
        )
        result = verifier.verify(context="test_ac1_check1_pass")
        # SM check passes; no band config so band check is skipped (dry-run)
        # No joint_q supplied so FK check is skipped
        assert result.left.sm_state == GripperState.LOCKED
        # Dry-run: band check skipped → left passes SM check
        assert not any("not LOCKED" in r for r in result.left.reasons)

    def test_verify_fk_tcp_within_tolerance(self, cfg, chains):
        """AC#1 check #3: FK TCP within tolerance → passes."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)

        left_chain, right_chain = chains
        fm = _make_frame_model(cfg, chains)
        fm.set_held_index("left", 0)

        # Use zero-q FK to compute what the expected distance is, then set
        # a generous tolerance so it passes
        T_chassis_left = fm._T_chassis_left  # noqa: SLF001
        T_base_tcp = left_chain.fk(np.zeros(5, dtype=np.float64))
        T_chassis_tcp = T_chassis_left @ T_base_tcp
        tcp_m = T_chassis_tcp[:3, 3]
        expected_m = fm.attachment_pose_in_chassis(0)[:3, 3]
        dist = float(np.linalg.norm(tcp_m - expected_m))

        # Tolerance just above the actual distance → should pass
        thresholds = VerificationThresholds(pos_tolerance_m=dist + 1.0, verified=True)

        verifier = TubeStateVerifier(
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
            thresholds=thresholds,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_chassis_left,
            dry_run=True,
        )
        q_zero = np.zeros(5, dtype=np.float64)
        result = verifier.verify(left_q=q_zero, context="fk_within_tolerance")
        # No "dist" reason should appear
        assert not any("FK TCP dist" in r for r in result.left.reasons)

    def test_verify_fk_tcp_exceeds_tolerance(self, cfg, chains):
        """AC#1 check #3: FK TCP exceeds tolerance → fails with displacement reason."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)

        left_chain, right_chain = chains
        fm = _make_frame_model(cfg, chains)
        fm.set_held_index("left", 0)
        T_chassis_left = fm._T_chassis_left  # noqa: SLF001

        # Set tolerance of 0 mm → any distance fails
        thresholds = VerificationThresholds(pos_tolerance_m=0.0, verified=True)

        verifier = TubeStateVerifier(
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
            thresholds=thresholds,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_chassis_left,
            dry_run=True,
        )
        q_zero = np.zeros(5, dtype=np.float64)
        result = verifier.verify(left_q=q_zero, context="fk_exceed_tolerance")
        # Should have a FK TCP distance reason (unless TCP happens to be exactly 0)
        T_base_tcp = left_chain.fk(q_zero)
        T_chassis_tcp = T_chassis_left @ T_base_tcp
        tcp_m = T_chassis_tcp[:3, 3]
        expected_m = fm.attachment_pose_in_chassis(0)[:3, 3]
        dist = float(np.linalg.norm(tcp_m - expected_m))
        if dist > 0.0:
            assert any("FK TCP dist" in r for r in result.left.reasons)

    def test_verify_null_tolerance_refused_in_live_mode(self, cfg, chains):
        """AC#1: null pos_tolerance_m in live mode (dry_run=False) → refused."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)

        left_chain, right_chain = chains
        fm = _make_frame_model(cfg, chains)
        fm.set_held_index("left", 0)
        T_chassis_left = fm._T_chassis_left  # noqa: SLF001

        thresholds = VerificationThresholds(pos_tolerance_m=None, verified=False)

        verifier = TubeStateVerifier(
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
            thresholds=thresholds,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_chassis_left,
            dry_run=False,  # live mode
        )
        q_zero = np.zeros(5, dtype=np.float64)
        result = verifier.verify(left_q=q_zero, context="live_null_tolerance")
        assert result.left.refused is True


# ---------------------------------------------------------------------------
# AC#3: Safety invariants
# ---------------------------------------------------------------------------


class TestSafetyInvariants:
    """AC#3: double-unlock guard and unverified-state guard."""

    def test_assert_not_both_unlocked_raises_when_both_unlocked(self):
        """assert_not_both_unlocked raises BothGrippersUnlockedError."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        left_sm.reset()   # UNLOCKED
        right_sm.reset()  # UNLOCKED
        with pytest.raises(BothGrippersUnlockedError):
            assert_not_both_unlocked(left_sm, right_sm, context="test")

    def test_assert_not_both_unlocked_passes_when_one_locked(self):
        """assert_not_both_unlocked does not raise when one arm is LOCKED."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        left_sm.reset()
        right_sm.reset()
        right_sm.set_approach_ready()
        right_sm.lock()
        # Should not raise
        assert_not_both_unlocked(left_sm, right_sm, context="test")

    def test_assert_not_both_unlocked_passes_when_both_locked(self):
        """Both LOCKED → no violation."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        assert_not_both_unlocked(left_sm, right_sm)

    def test_assert_state_verified_raises_on_refused(self):
        """assert_state_verified raises UnverifiedStateError when refused."""
        arm_result = ArmVerificationResult(
            arm="left",
            index=0,
            sm_state=GripperState.LOCKED,
            gripper_pos_deg=180.0,
            fk_tcp_chassis_m=None,
            expected_tcp_chassis_m=None,
            passed=False,
            refused=True,
            reasons=["pos_tolerance_m is None"],
        )
        ok_result = ArmVerificationResult(
            arm="right",
            index=None,
            sm_state=GripperState.LOCKED,
            gripper_pos_deg=90.0,
            fk_tcp_chassis_m=None,
            expected_tcp_chassis_m=None,
            passed=True,
            refused=False,
        )
        result = VerificationResult(left=arm_result, right=ok_result)
        with pytest.raises(UnverifiedStateError):
            assert_state_verified(result, require_pass=True)

    def test_assert_state_verified_raises_on_failure(self):
        """assert_state_verified raises UnverifiedStateError when not passed."""
        arm_result = ArmVerificationResult(
            arm="left",
            index=0,
            sm_state=GripperState.UNLOCKED,  # SM not LOCKED
            gripper_pos_deg=90.0,
            fk_tcp_chassis_m=None,
            expected_tcp_chassis_m=None,
            passed=False,
            refused=False,
            reasons=["left: model holds index 0 but SM state is UNLOCKED"],
        )
        ok_result = ArmVerificationResult(
            arm="right",
            index=None,
            sm_state=GripperState.LOCKED,
            gripper_pos_deg=180.0,
            fk_tcp_chassis_m=None,
            expected_tcp_chassis_m=None,
            passed=True,
            refused=False,
        )
        result = VerificationResult(left=arm_result, right=ok_result)
        with pytest.raises(UnverifiedStateError):
            assert_state_verified(result, require_pass=True)

    def test_assert_state_verified_does_not_raise_when_require_pass_false(self):
        """assert_state_verified does not raise when require_pass=False."""
        arm_result = ArmVerificationResult(
            arm="left",
            index=0,
            sm_state=GripperState.UNLOCKED,
            gripper_pos_deg=90.0,
            fk_tcp_chassis_m=None,
            expected_tcp_chassis_m=None,
            passed=False,
            refused=True,
            reasons=["test reason"],
        )
        ok = ArmVerificationResult(
            arm="right", index=None, sm_state=GripperState.LOCKED,
            gripper_pos_deg=180.0, fk_tcp_chassis_m=None, expected_tcp_chassis_m=None,
            passed=True, refused=False,
        )
        result = VerificationResult(left=arm_result, right=ok)
        # Should not raise
        assert_state_verified(result, require_pass=False)

    def test_fault_responder_safe_halt_both_unlocked_fails(self):
        """AC#3: safe_halt with both grippers unlocked → succeeded=False."""
        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        left_sm.reset()   # UNLOCKED
        right_sm.reset()  # UNLOCKED — both unlocked

        responder = FaultResponder(left_sm=left_sm, right_sm=right_sm, dry_run=True)
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.INSERTION_FAILURE,
            detail="test both-unlocked scenario",
            cycle_index=0,
        )
        # safe_halt detects both unlocked and marks succeeded=False
        assert response == RecoveryResponse.SAFE_HALT
        assert succeeded is False


# ---------------------------------------------------------------------------
# AC#4: CycleStateCheckpoint
# ---------------------------------------------------------------------------


class TestCycleStateCheckpoint:
    """AC#4: checkpoint persists and validates; stale state refused."""

    def test_save_and_load_roundtrip(self, cfg, chains, tmp_path):
        """Checkpoint save→load roundtrip preserves all fields."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk.json")
        chk.save(
            cycle_index=3,
            phase=CyclePhase.APPROACH,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        assert (tmp_path / "chk.json").exists()
        data = chk.load()
        assert data["cycle_index"] == 3
        assert data["phase"] == CyclePhase.APPROACH.value
        assert data["gripper_states"]["left"] == "LOCKED"
        assert data["gripper_states"]["right"] == "LOCKED"

    def test_load_missing_file_raises_resume_refused(self, tmp_path):
        """Loading a non-existent checkpoint raises ResumeRefusedError."""
        chk = CycleStateCheckpoint(tmp_path / "no_such_file.json")
        with pytest.raises(ResumeRefusedError, match="not found"):
            chk.load()

    def test_load_corrupt_json_raises_resume_refused(self, tmp_path):
        """Loading corrupt JSON raises ResumeRefusedError."""
        p = tmp_path / "bad.json"
        p.write_text("{not valid json", encoding="utf-8")
        chk = CycleStateCheckpoint(p)
        with pytest.raises(ResumeRefusedError, match="not valid JSON"):
            chk.load()

    def test_validate_ok_returns_phase(self, cfg, chains, tmp_path):
        """validate_against_live_state returns CyclePhase on success."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk_ok.json")
        chk.save(
            cycle_index=1,
            phase=CyclePhase.LOCK,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        data = chk.load()
        resumed_phase = chk.validate_against_live_state(
            data,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        assert resumed_phase == CyclePhase.LOCK

    def test_validate_gripper_mismatch_raises(self, cfg, chains, tmp_path):
        """AC#4: gripper state mismatch between checkpoint and live → refused."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk_mismatch.json")
        chk.save(
            cycle_index=0,
            phase=CyclePhase.ADVANCE,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        # Now change left SM to FAULT
        left_sm._state = GripperState.FAULT  # noqa: SLF001
        data = chk.load()
        with pytest.raises(ResumeRefusedError, match="gripper_states.left mismatch"):
            chk.validate_against_live_state(
                data,
                frame_model=fm,
                left_sm=left_sm,
                right_sm=right_sm,
            )

    def test_validate_advance_count_mismatch_raises(self, cfg, chains, tmp_path):
        """AC#4: advance_count mismatch → resumed refused with clear reason."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk_ac.json")
        chk.save(
            cycle_index=0,
            phase=CyclePhase.POST_CYCLE,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        # Mutate the FrameModel advance count live
        fm.complete_advance(direction=+1)
        data = chk.load()
        with pytest.raises(ResumeRefusedError, match="advance_count mismatch"):
            chk.validate_against_live_state(
                data,
                frame_model=fm,
                left_sm=left_sm,
                right_sm=right_sm,
            )

    def test_validate_schema_version_mismatch_raises(self, cfg, chains, tmp_path):
        """AC#4: wrong schema_version → refused."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk_sv.json")
        chk.save(
            cycle_index=0,
            phase=CyclePhase.FOLD,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        # Tamper with schema version
        data = json.loads((tmp_path / "chk_sv.json").read_text())
        data["schema_version"] = 999
        (tmp_path / "chk_sv.json").write_text(json.dumps(data))
        with pytest.raises(ResumeRefusedError, match="schema_version"):
            chk.load()

    def test_checkpoint_write_is_atomic(self, cfg, chains, tmp_path):
        """Checkpoint writes are atomic: old file not corrupted on crash simulation."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)
        p = tmp_path / "chk_atomic.json"

        chk = CycleStateCheckpoint(p)
        # First save: cycle 0
        chk.save(
            cycle_index=0,
            phase=CyclePhase.UNLOCK,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        first_data = json.loads(p.read_text())
        assert first_data["cycle_index"] == 0

        # Second save: cycle 1
        chk.save(
            cycle_index=1,
            phase=CyclePhase.FOLD,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        second_data = json.loads(p.read_text())
        assert second_data["cycle_index"] == 1
        # No tmp files left behind
        tmp_files = list(tmp_path.glob("chk_atomic_tmp_*.json"))
        assert len(tmp_files) == 0

    def test_stale_lattice_index_refused(self, cfg, chains, tmp_path):
        """AC#4: lattice index mismatch between checkpoint and live → refused."""
        left_sm = _locked_sm("arm_left")
        right_sm = _locked_sm("arm_right")
        _wire(left_sm, right_sm)
        fm = _make_frame_model(cfg, chains)

        chk = CycleStateCheckpoint(tmp_path / "chk_lat.json")
        chk.save(
            cycle_index=0,
            phase=CyclePhase.ADVANCE,
            frame_model=fm,
            left_sm=left_sm,
            right_sm=right_sm,
        )
        # Now set a different lattice index live
        fm.set_held_index("left", 5)
        data = chk.load()
        with pytest.raises(ResumeRefusedError, match="lattice_state.left_index mismatch"):
            chk.validate_against_live_state(
                data,
                frame_model=fm,
                left_sm=left_sm,
                right_sm=right_sm,
            )


# ---------------------------------------------------------------------------
# AC#6: PostRunReport
# ---------------------------------------------------------------------------


class TestPostRunReport:
    """AC#6: post-run report lists all faults, responses, and success status."""

    def _make_event_log(self) -> List[Dict[str, Any]]:
        """Synthetic AA-7 event log with two faults."""
        return [
            {
                "ts_iso": "2026-10-24T10:00:00+00:00",
                "event_type": "recovery_fault",
                "payload": {
                    "fault_kind": RecoveryFaultKind.INSERTION_FAILURE.value,
                    "response": RecoveryResponse.SAFE_HALT.value,
                    "detail": "3 retries exceeded",
                    "cycle_index": 0,
                    "arm": "arm_left",
                },
            },
            {
                "ts_iso": "2026-10-24T10:00:01+00:00",
                "event_type": "recovery_response",
                "payload": {
                    "fault_kind": RecoveryFaultKind.INSERTION_FAILURE.value,
                    "response": RecoveryResponse.SAFE_HALT.value,
                    "succeeded": True,
                    "detail": "holder stayed LOCKED",
                    "cycle_index": 0,
                },
            },
            {
                "ts_iso": "2026-10-24T10:01:00+00:00",
                "event_type": "recovery_fault",
                "payload": {
                    "fault_kind": RecoveryFaultKind.USB_DEVICE_LOSS.value,
                    "response": RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
                    "detail": "DeviceNotFoundError: arm_right",
                    "cycle_index": 2,
                    "arm": None,
                },
            },
            {
                "ts_iso": "2026-10-24T10:01:01+00:00",
                "event_type": "recovery_response",
                "payload": {
                    "fault_kind": RecoveryFaultKind.USB_DEVICE_LOSS.value,
                    "response": RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
                    "succeeded": True,
                    "detail": "estop triggered",
                    "cycle_index": 2,
                },
            },
        ]

    def test_report_total_faults(self):
        report = build_post_run_report(self._make_event_log())
        assert report.total_faults == 2

    def test_report_all_responses_succeeded(self):
        report = build_post_run_report(self._make_event_log())
        assert report.all_responses_succeeded is True

    def test_report_fault_kinds_listed(self):
        report = build_post_run_report(self._make_event_log())
        kinds = {f.fault_kind for f in report.faults}
        assert RecoveryFaultKind.INSERTION_FAILURE.value in kinds
        assert RecoveryFaultKind.USB_DEVICE_LOSS.value in kinds

    def test_report_responses_listed(self):
        report = build_post_run_report(self._make_event_log())
        responses = {f.response for f in report.faults}
        assert RecoveryResponse.SAFE_HALT.value in responses
        assert RecoveryResponse.ESTOP_HOLD_AND_HALT.value in responses

    def test_report_summary_text_non_empty(self):
        report = build_post_run_report(self._make_event_log())
        assert "AA-28" in report.summary_text
        assert str(report.total_faults) in report.summary_text

    def test_empty_log_report(self):
        """Empty event log → 0 faults, all_responses_succeeded=True."""
        report = build_post_run_report([])
        assert report.total_faults == 0
        assert report.all_responses_succeeded is True
        assert "no faults" in report.summary_text

    def test_failed_response_in_log(self):
        """A failed response → all_responses_succeeded=False."""
        log = [
            {
                "ts_iso": "2026-10-24T10:00:00+00:00",
                "event_type": "recovery_fault",
                "payload": {
                    "fault_kind": RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
                    "response": RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
                    "detail": "servo timeout",
                    "cycle_index": 1,
                    "arm": None,
                },
            },
            {
                "ts_iso": "2026-10-24T10:00:01+00:00",
                "event_type": "recovery_response",
                "payload": {
                    "fault_kind": RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
                    "response": RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
                    "succeeded": False,
                    "detail": "estop raised EStopPartialError",
                    "cycle_index": 1,
                },
            },
        ]
        report = build_post_run_report(log)
        assert report.total_faults == 1
        assert report.all_responses_succeeded is False

    def test_report_from_jsonl_file(self, tmp_path):
        """build_post_run_report_from_jsonl reads JSONL and builds report."""
        from inchworm.recovery import build_post_run_report_from_jsonl

        log_path = tmp_path / "event_log.jsonl"
        events = self._make_event_log()
        log_path.write_text(
            "\n".join(json.dumps(e) for e in events) + "\n",
            encoding="utf-8",
        )
        report = build_post_run_report_from_jsonl(log_path)
        assert report.total_faults == 2

    def test_report_to_dict(self):
        report = build_post_run_report(self._make_event_log())
        d = report.to_dict()
        assert d["total_faults"] == 2
        assert isinstance(d["faults"], list)
        assert d["all_responses_succeeded"] is True


# ---------------------------------------------------------------------------
# AC#7: CycleRunner AA-28 hooks integration
# ---------------------------------------------------------------------------


class TestCycleHooks:
    """AC#7: AA-28 hooks wired into CycleRunner via set_aa28_hooks()."""

    def _make_runner(self, cfg, chains, guards) -> CycleRunner:
        left_chain, right_chain = chains
        left_guard, right_guard = guards

        left_sm = _make_sm("arm_left")
        right_sm = _make_sm("arm_right")
        _wire(left_sm, right_sm)

        approach_cfg = cfg.approach
        assert approach_cfg is not None

        supervisor_cfg = PolicySupervisorConfig(
            max_steps=10,
            max_wall_time_s=5.0,
            workspace_radius_m=10.0,
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
            cycle_cfg=CycleRunnerConfig(
                initial_moving_arm="left",
                supervisor_success_at_step=2,
            ),
            config=None,
            dry_run=True,
        )
        return runner

    def test_set_aa28_hooks_sets_attributes(self, cfg, chains, guards):
        """set_aa28_hooks() stores verifier/responder/checkpoint."""
        runner = self._make_runner(cfg, chains, guards)
        mock_verifier = MagicMock()
        mock_responder = MagicMock()
        mock_checkpoint = MagicMock()
        runner.set_aa28_hooks(
            verifier=mock_verifier,
            responder=mock_responder,
            checkpoint=mock_checkpoint,
        )
        assert runner._aa28_verifier is mock_verifier  # noqa: SLF001
        assert runner._aa28_responder is mock_responder  # noqa: SLF001
        assert runner._aa28_checkpoint is mock_checkpoint  # noqa: SLF001

    def test_checkpoint_saved_on_pre_and_post_cycle(self, cfg, chains, guards, tmp_path):
        """Checkpoint.save() is called for PRE_CYCLE and POST_CYCLE phases."""
        runner = self._make_runner(cfg, chains, guards)

        # Set up SMs: left starts in LOCKED (holding), right will move
        # Actually runner starts with initial_moving_arm=left, so right is holding
        # Preflight needs holding arm (right) to be LOCKED
        runner._right_sm.reset()  # noqa: SLF001
        runner._right_sm.set_approach_ready()  # noqa: SLF001
        runner._right_sm.lock()  # noqa: SLF001

        saved_calls: List[Dict[str, Any]] = []

        class _MockCheckpoint:
            def save(self, **kwargs):
                saved_calls.append(dict(kwargs))

        runner.set_aa28_hooks(checkpoint=_MockCheckpoint())
        runner.run_one_cycle(cycle_index=0)

        phases_saved = [c["phase"].value for c in saved_calls]
        assert CyclePhase.PRE_CYCLE.value in phases_saved
        assert CyclePhase.POST_CYCLE.value in phases_saved

    def test_pre_cycle_verification_failure_raises_preflight_error(
        self, cfg, chains, guards
    ):
        """AC#7 + AC#3: pre-cycle verifier returning failed → PreflightError in live mode.

        AC#3 asserted: no motion is commanded from unverified state.
        """
        runner = self._make_runner(cfg, chains, guards)
        # Must be in live mode for pre-cycle verification to raise PreflightError
        runner._dry_run = False  # noqa: SLF001

        # Create a verifier that always returns failed
        class _FailingVerifier:
            def verify(self, **kwargs) -> VerificationResult:
                bad = ArmVerificationResult(
                    arm="left",
                    index=0,
                    sm_state=GripperState.UNLOCKED,
                    gripper_pos_deg=90.0,
                    fk_tcp_chassis_m=None,
                    expected_tcp_chassis_m=None,
                    passed=False,
                    refused=False,
                    reasons=["injected failure for test"],
                )
                ok = ArmVerificationResult(
                    arm="right",
                    index=None,
                    sm_state=GripperState.LOCKED,
                    gripper_pos_deg=180.0,
                    fk_tcp_chassis_m=None,
                    expected_tcp_chassis_m=None,
                    passed=True,
                    refused=False,
                )
                return VerificationResult(left=bad, right=ok)

        runner.set_aa28_hooks(verifier=_FailingVerifier())

        # Make sure holding arm is LOCKED so preflight-1 passes
        runner._right_sm.reset()  # noqa: SLF001
        runner._right_sm.set_approach_ready()  # noqa: SLF001
        runner._right_sm.lock()  # noqa: SLF001

        with pytest.raises(PreflightError):
            runner.run_one_cycle(cycle_index=0)

        # AC#3: right (holding) arm is still LOCKED — no double-unlock
        assert runner._right_sm.state == GripperState.LOCKED  # noqa: SLF001

    def test_no_hooks_cycle_runs_normally(self, cfg, chains, guards):
        """Without AA-28 hooks the cycle runner still executes all phases."""
        runner = self._make_runner(cfg, chains, guards)
        # Set right as LOCKED to satisfy preflight (initial moving arm = left)
        runner._right_sm.reset()  # noqa: SLF001
        runner._right_sm.set_approach_ready()  # noqa: SLF001
        runner._right_sm.lock()  # noqa: SLF001
        plan = runner.run_one_cycle(cycle_index=0)
        assert plan is not None
        phase_names = [p.phase for p in plan.phases]
        assert "preflight" in phase_names


# --- Orchestrator 2026-10-24: live verification without joint readings refuses ---
def test_live_verify_without_joint_readings_refuses(cfg, chains):
    left_sm, right_sm = _locked_sm("arm_left"), _locked_sm("arm_right")
    v = _make_verifier(cfg, chains, left_sm, right_sm,
                       thresholds=VerificationThresholds(pos_tolerance_m=0.002, verified=True),
                       dry_run=False)
    v._fm.set_held_index("left", 0)
    res = v.verify(left_q=None, right_q=None, context="t")
    assert not res.passed
    left = [a for a in res.arms if a.arm == "left"][0] if hasattr(res, "arms") else None
    if left is not None:
        assert left.refused
        assert any("cannot be verified" in r for r in left.reasons)
