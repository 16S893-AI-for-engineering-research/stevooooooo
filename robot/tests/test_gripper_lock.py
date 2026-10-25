"""Tests for inchworm.gripper_lock (AA-17) and inchworm.policy_actions (AC#5).

Covers all software ACs:

  AC#2  - State machine exposes unlock/approach/lock/hold transitions; rejects
          illegal transitions with typed errors; is the only code path to the
          gripper servo.
  AC#3  - Lock verification distinguishes LATCHED / JAM / MISS / UNVERIFIED from
          JointState.load_raw and current_raw; reports typed failure instead of
          assuming success.
  AC#4  - Active hold-monitor (hold_tick) re-asserts position on drift; logs each
          correction; raises on non-LOCKED state.
  AC#5  - policy_actions adapter hard-blocks gripper keys; test that no module
          except gripper_lock*.py imports _GRIPPER_TOKEN.
  AC#6  - Unlock refused when opposite-arm predicate returns False/raises; unlock
          proceeds when predicate returns True.
  AC#7  - Band logic (null → BandsNotConfigured in live; pass in dry-run),
          illegal transitions, lock verification, drift re-assertion, policy block.

Hardware ACs left unchecked:
  AC#1  - Empirical band measurement (hardware session required).
"""

from __future__ import annotations

import struct
from typing import Dict
from unittest.mock import MagicMock, patch

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import (
    GripperBandConfig,
    GripperLockConfig,
    GripperVerificationThresholds,
)
from inchworm.gripper_lock import (
    BandsNotConfigured,
    GripperLockError,
    GripperLockSM,
    GripperState,
    IllegalTransition,
    LockVerdict,
    LockVerificationFailed,
    UnlockRefused,
    _bands_configured,
    _in_band,
    verify_lock_state,
)
from inchworm.policy_actions import (
    POLICY_JOINT_NAMES,
    GripperCommandBlocked,
    PolicyActionAdapter,
    validate_policy_action,
)
from inchworm.servo_bus import ArmServoBus, JointState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_JOINT_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# A fully-configured band config for left arm
_BANDS_LEFT = GripperBandConfig(
    unlocked_low_deg=80.0,
    unlocked_high_deg=100.0,
    engaged_low_deg=110.0,
    engaged_high_deg=130.0,
    locked_low_deg=145.0,
    locked_high_deg=165.0,
    # overtravel fields: must satisfy locked_high < overtravel_goal < closed_empty
    lock_overtravel_goal_deg=200.0,
    closed_empty_deg=250.0,
    lock_torque_limit_raw=300,
    unverified=False,
)

_BANDS_RIGHT = GripperBandConfig(
    unlocked_low_deg=80.0,
    unlocked_high_deg=100.0,
    engaged_low_deg=110.0,
    engaged_high_deg=130.0,
    locked_low_deg=145.0,
    locked_high_deg=165.0,
    lock_overtravel_goal_deg=200.0,
    closed_empty_deg=250.0,
    lock_torque_limit_raw=300,
    unverified=False,
)

_THRESHOLDS = GripperVerificationThresholds(
    jam_load_threshold_raw=200,
    latch_load_max_raw=150,
    latch_current_max_raw=300,
    unverified=False,
)

_LOCK_CONFIG = GripperLockConfig(
    arm_left=_BANDS_LEFT,
    arm_right=_BANDS_RIGHT,
    drift_threshold_deg=2.0,
    verification=_THRESHOLDS,
)

_LOCK_CONFIG_NULL_BANDS = GripperLockConfig(
    arm_left=None,
    arm_right=None,
    drift_threshold_deg=None,
    verification=None,
)


def _make_transport() -> MockSerialTransport:
    return MockSerialTransport("arm_left")


def _make_bus(transport: MockSerialTransport, role: str = "arm_left") -> ArmServoBus:
    return ArmServoBus(
        transport,
        role=role,
        joint_id_map=_JOINT_MAP,
        timeout_s=0.05,
    )


def _status_packet(servo_id: int, error: int, payload: bytes) -> bytes:
    length = len(payload) + 2
    chk_data = bytes([error]) + payload
    total = servo_id + length + sum(chk_data)
    chk = (~total) & 0xFF
    return b"\xff\xff" + bytes([servo_id, length, error]) + payload + bytes([chk])


def _write_response(servo_id: int) -> bytes:
    return _status_packet(servo_id, 0, b"")


def _read_response(servo_id: int, payload: bytes) -> bytes:
    return _status_packet(servo_id, 0, payload)


def _joint_state_response(
    servo_id: int,
    pos_ticks: int = 1706,  # ~150 deg (locked band midpoint)
    speed_raw16: int = 0,
    load_raw16: int = 50,
    voltage_raw: int = 120,
    temperature: int = 35,
    current_raw: int = 100,
) -> bytes:
    block = (
        struct.pack("<H", pos_ticks)
        + struct.pack("<H", speed_raw16)
        + struct.pack("<H", load_raw16)
        + bytes([voltage_raw, temperature])
        + bytes(5)
        + struct.pack("<H", current_raw)
    )
    assert len(block) == 15
    return _read_response(servo_id, block)


def _pos_read_response(servo_id: int, ticks: int) -> bytes:
    return _read_response(servo_id, struct.pack("<H", ticks))


# ---------------------------------------------------------------------------
# SM factory helpers
# ---------------------------------------------------------------------------


def _dry_sm(transport=None, role="arm_left", lock_config=None, opposite_fn=None):
    """Return a dry-run GripperLockSM (no real hardware)."""
    if transport is None:
        transport = _make_transport()
    bus = _make_bus(transport, role=role)
    return GripperLockSM(
        bus=bus,
        role=role,
        config=lock_config or _LOCK_CONFIG,
        opposite_arm_locked_fn=opposite_fn or (lambda: True),
        dry_run=True,
    )


def _live_sm(transport, role="arm_left", lock_config=None, opposite_fn=None):
    """Return a live GripperLockSM (mock transport)."""
    bus = _make_bus(transport, role=role)
    return GripperLockSM(
        bus=bus,
        role=role,
        config=lock_config or _LOCK_CONFIG,
        opposite_arm_locked_fn=opposite_fn or (lambda: True),
        dry_run=False,
    )


# ---------------------------------------------------------------------------
# AC#2: State machine transitions
# ---------------------------------------------------------------------------


class TestStateMachineTransitions:
    """AC#2: Explicit state machine; illegal transitions raise typed errors."""

    def test_initial_state_is_unknown(self):
        sm = _dry_sm()
        assert sm.state == GripperState.UNKNOWN

    def test_reset_from_unknown_to_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        assert sm.state == GripperState.UNLOCKED

    def test_reset_from_fault_to_unlocked(self):
        sm = _dry_sm()
        # Force to FAULT artificially
        sm._state = GripperState.FAULT
        sm.reset()
        assert sm.state == GripperState.UNLOCKED

    def test_set_approach_ready_from_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        assert sm.state == GripperState.APPROACH_READY

    def test_lock_from_approach_ready_reaches_locked(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        verdict = sm.lock()
        assert sm.state == GripperState.LOCKED
        assert verdict == LockVerdict.LATCHED

    def test_unlock_from_locked_reaches_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()
        assert sm.state == GripperState.UNLOCKED

    def test_illegal_approach_ready_from_unknown(self):
        sm = _dry_sm()
        with pytest.raises(IllegalTransition) as exc_info:
            sm.set_approach_ready()
        assert exc_info.value.current_state == GripperState.UNKNOWN
        assert exc_info.value.requested_op == "set_approach_ready"

    def test_illegal_approach_ready_from_locked(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(IllegalTransition):
            sm.set_approach_ready()

    def test_illegal_lock_from_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        with pytest.raises(IllegalTransition) as exc_info:
            sm.lock()
        assert exc_info.value.current_state == GripperState.UNLOCKED

    def test_illegal_lock_from_locked(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(IllegalTransition):
            sm.lock()

    def test_illegal_unlock_from_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        with pytest.raises(IllegalTransition):
            sm.unlock()

    def test_illegal_unlock_from_approach_ready(self):
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        with pytest.raises(IllegalTransition):
            sm.unlock()

    def test_illegal_hold_tick_from_unlocked(self):
        sm = _dry_sm()
        sm.reset()
        with pytest.raises(IllegalTransition):
            sm.hold_tick()

    def test_illegal_hold_tick_from_unknown(self):
        sm = _dry_sm()
        with pytest.raises(IllegalTransition):
            sm.hold_tick()

    def test_illegal_transitions_are_gripper_lock_errors(self):
        sm = _dry_sm()
        with pytest.raises(GripperLockError):
            sm.unlock()

    def test_approach_ready_after_unlock_cycle(self):
        """Full unlock cycle returns to UNLOCKED; can then approach again."""
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()
        assert sm.state == GripperState.UNLOCKED
        sm.set_approach_ready()
        assert sm.state == GripperState.APPROACH_READY


# ---------------------------------------------------------------------------
# AC#7: Band logic (null bands = fail closed in live mode, pass in dry-run)
# ---------------------------------------------------------------------------


class TestBandLogic:
    """AC#7: Null bands refuse live operations; dry-run skips null check."""

    def test_null_bands_dry_run_lock_succeeds(self):
        """Dry-run with null bands: lock succeeds (band check skipped)."""
        sm = _dry_sm(lock_config=_LOCK_CONFIG_NULL_BANDS)
        sm.reset()
        sm.set_approach_ready()
        verdict = sm.lock()
        assert verdict == LockVerdict.LATCHED

    def test_null_bands_dry_run_unlock_succeeds(self):
        """Dry-run with null bands: unlock succeeds after lock."""
        sm = _dry_sm(lock_config=_LOCK_CONFIG_NULL_BANDS)
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()
        assert sm.state == GripperState.UNLOCKED

    def test_null_bands_live_lock_raises(self):
        """Live mode with null bands: lock raises BandsNotConfigured."""
        transport = _make_transport()
        sm = _live_sm(transport, lock_config=_LOCK_CONFIG_NULL_BANDS)
        sm._state = GripperState.APPROACH_READY
        with pytest.raises(BandsNotConfigured) as exc_info:
            sm.lock()
        assert exc_info.value.role == "arm_left"
        assert "lock" in exc_info.value.operation

    def test_null_bands_live_unlock_raises(self):
        """Live mode with null bands: unlock raises BandsNotConfigured."""
        transport = _make_transport()
        sm = _live_sm(transport, lock_config=_LOCK_CONFIG_NULL_BANDS)
        sm._state = GripperState.LOCKED
        with pytest.raises(BandsNotConfigured) as exc_info:
            sm.unlock()
        assert exc_info.value.role == "arm_left"

    def test_null_bands_live_approach_raises(self):
        """Live mode with null bands: set_approach_ready raises BandsNotConfigured."""
        transport = _make_transport()
        sm = _live_sm(transport, lock_config=_LOCK_CONFIG_NULL_BANDS)
        sm._state = GripperState.UNLOCKED
        with pytest.raises(BandsNotConfigured):
            sm.set_approach_ready()

    def test_bands_configured_returns_false_for_none(self):
        assert _bands_configured(None) is False

    def test_bands_configured_returns_false_for_partial(self):
        partial = GripperBandConfig(
            unlocked_low_deg=80.0,
            unlocked_high_deg=None,
            engaged_low_deg=None,
            engaged_high_deg=None,
            locked_low_deg=None,
            locked_high_deg=None,
        )
        assert _bands_configured(partial) is False

    def test_bands_configured_returns_true_for_full(self):
        assert _bands_configured(_BANDS_LEFT) is True

    def test_in_band_true_within_range(self):
        assert _in_band(90.0, 80.0, 100.0) is True

    def test_in_band_false_below_range(self):
        assert _in_band(79.9, 80.0, 100.0) is False

    def test_in_band_false_above_range(self):
        assert _in_band(100.1, 80.0, 100.0) is False

    def test_in_band_false_for_none_low(self):
        assert _in_band(90.0, None, 100.0) is False

    def test_in_band_false_for_none_high(self):
        assert _in_band(90.0, 80.0, None) is False


# ---------------------------------------------------------------------------
# AC#3: Lock verification
# ---------------------------------------------------------------------------


class TestLockVerification:
    """AC#3: Lock verification distinguishes LATCHED / JAM / MISS / UNVERIFIED."""

    def _make_state(self, pos_deg: float, load_raw: int = 50, current_raw: int = 100) -> JointState:
        return JointState(
            servo_id=6,
            position_deg=pos_deg,
            speed_raw=0,
            load_raw=load_raw,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=current_raw,
        )

    def test_latched_when_pos_in_band_and_nominal_load(self):
        state = self._make_state(155.0, load_raw=50, current_raw=100)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.LATCHED

    def test_miss_when_pos_not_in_locked_band(self):
        """Position outside locked band -> MISS."""
        state = self._make_state(90.0, load_raw=50, current_raw=100)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.MISS

    def test_jam_when_load_exceeds_latch_max(self):
        """load_raw above latch_load_max_raw -> JAM."""
        state = self._make_state(155.0, load_raw=200, current_raw=100)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.JAM

    def test_jam_when_current_exceeds_latch_max(self):
        """current_raw above latch_current_max_raw -> JAM."""
        state = self._make_state(155.0, load_raw=50, current_raw=400)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.JAM

    def test_unverified_when_bands_none(self):
        state = self._make_state(155.0)
        verdict = verify_lock_state(state, None, _THRESHOLDS)
        assert verdict == LockVerdict.UNVERIFIED

    def test_unverified_when_thresholds_none(self):
        state = self._make_state(155.0)
        verdict = verify_lock_state(state, _BANDS_LEFT, None)
        assert verdict == LockVerdict.UNVERIFIED

    def test_unverified_when_thresholds_have_unverified_true(self):
        unverified_thresh = GripperVerificationThresholds(unverified=True)
        state = self._make_state(155.0)
        verdict = verify_lock_state(state, _BANDS_LEFT, unverified_thresh)
        assert verdict == LockVerdict.UNVERIFIED

    def test_unverified_when_bands_not_configured(self):
        partial = GripperBandConfig(
            unlocked_low_deg=None,
            unlocked_high_deg=None,
            engaged_low_deg=None,
            engaged_high_deg=None,
            locked_low_deg=None,
            locked_high_deg=None,
        )
        state = self._make_state(155.0)
        verdict = verify_lock_state(state, partial, _THRESHOLDS)
        assert verdict == LockVerdict.UNVERIFIED

    def test_jam_uses_absolute_load_negative_load(self):
        """Negative load_raw (sign-magnitude convention) uses abs value for comparison."""
        # -200 load -> abs(200) > latch_load_max_raw(150) -> JAM
        state = self._make_state(155.0, load_raw=-200, current_raw=100)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.JAM

    def test_latched_with_negative_load_within_threshold(self):
        """Small negative load (|load| <= latch_load_max) -> LATCHED."""
        state = self._make_state(155.0, load_raw=-50, current_raw=100)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.LATCHED

    def test_live_lock_failure_raises_typed_error(self):
        """live lock() raises LockVerificationFailed (subtype of GripperLockError)."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.APPROACH_READY

        # Feed write responses: torque-limit write + goal-position write.
        # Use queue_reply so each response is released only after the matching write.
        transport.queue_reply(_write_response(6))  # torque limit
        transport.queue_reply(_write_response(6))  # goal position
        miss_state = JointState(
            servo_id=6,
            position_deg=90.0,  # outside locked band (145-165) -> MISS
            speed_raw=0,
            load_raw=50,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )
        with patch.object(sm._bus, "read_joint_state", return_value=miss_state):
            with pytest.raises(LockVerificationFailed) as exc_info:
                sm.lock()
        assert exc_info.value.verdict == LockVerdict.MISS
        assert isinstance(exc_info.value, GripperLockError)
        assert sm.state == GripperState.FAULT

    def test_live_lock_jam_reaches_fault(self):
        """live lock() with JAM verdict transitions to FAULT."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.APPROACH_READY

        # Use queue_reply for half-duplex correctness: torque limit + goal position
        transport.queue_reply(_write_response(6))  # torque limit
        transport.queue_reply(_write_response(6))  # goal position
        # Position in locked band (155 deg), but load_raw = 200 > latch_load_max(150) = JAM
        jam_state = JointState(
            servo_id=6,
            position_deg=155.0,  # in locked band 145-165
            speed_raw=0,
            load_raw=200,  # > latch_load_max_raw(150) -> JAM
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )
        with patch.object(sm._bus, "read_joint_state", return_value=jam_state):
            with pytest.raises(LockVerificationFailed) as exc_info:
                sm.lock()
        assert exc_info.value.verdict == LockVerdict.JAM
        assert sm.state == GripperState.FAULT

    def test_live_lock_success_reaches_locked(self):
        """live lock() with LATCHED verdict transitions to LOCKED."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.APPROACH_READY

        # Use queue_reply for half-duplex correctness: torque limit + goal position
        transport.queue_reply(_write_response(6))  # torque limit
        transport.queue_reply(_write_response(6))  # goal position
        # Position at 155 deg (within locked band 145-165), load=50 < 150 = LATCHED
        latched_state = JointState(
            servo_id=6,
            position_deg=155.0,
            speed_raw=0,
            load_raw=50,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )
        with patch.object(sm._bus, "read_joint_state", return_value=latched_state):
            verdict = sm.lock()
        assert verdict == LockVerdict.LATCHED
        assert sm.state == GripperState.LOCKED


# ---------------------------------------------------------------------------
# AC#4: Active hold (drift re-assertion)
# ---------------------------------------------------------------------------


class TestActiveHold:
    """AC#4: hold_tick re-asserts position on drift; logs each correction."""

    def test_hold_tick_dry_run_no_correction(self):
        """Dry-run hold_tick: no read, no write, returns False."""
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        result = sm.hold_tick()
        assert result is False

    def test_hold_tick_no_drift_returns_false(self):
        """Live hold_tick with position at commanded value: returns False (no correction)."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 155.0

        # Mock read_position_deg to return the commanded position (no drift)
        with patch.object(sm._bus, "read_position_deg", return_value=155.0):
            result = sm.hold_tick()
        assert result is False

    def test_hold_tick_drift_triggers_correction(self):
        """Live hold_tick with position below locked_low_deg: returns True and re-asserts overtravel goal.

        With the overtravel design, hold is healthy while position is inside the
        locked band.  A position BELOW locked_low_deg indicates the servo drifted
        toward open; the correction re-asserts the overtravel goal.
        """
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0  # overtravel goal (matches _BANDS_LEFT.lock_overtravel_goal_deg)

        # Feed write response for the re-assert command
        transport.feed(_write_response(6))
        # Mock read to return position below locked_low_deg (145.0) -> drift toward open
        with patch.object(sm._bus, "read_position_deg", return_value=140.0):
            result = sm.hold_tick()
        assert result is True

    def test_hold_tick_correction_writes_commanded_position(self):
        """Correction re-commands the overtravel goal position (not the drifted one).

        When the servo drifts below locked_low_deg, the correction must write the
        overtravel goal (lock_overtravel_goal_deg) to drive it back into the band.
        """
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0  # overtravel goal

        transport.feed(_write_response(6))
        with patch.object(sm._bus, "read_position_deg", return_value=140.0):  # below locked_low
            sm.hold_tick()

        # The correction write should encode the overtravel goal (200.0 deg)
        written = transport.written[-1]
        tick_bytes = written[6:8]
        import struct as st
        written_ticks = st.unpack("<H", tick_bytes)[0]
        from inchworm.servo_bus import ticks_to_degrees
        written_deg = ticks_to_degrees(written_ticks)
        assert abs(written_deg - 200.0) < 1.0  # within tick-rounding tolerance

    def test_hold_tick_illegal_from_non_locked(self):
        """hold_tick raises IllegalTransition when not in LOCKED state."""
        sm = _dry_sm()
        sm.reset()
        with pytest.raises(IllegalTransition) as exc_info:
            sm.hold_tick()
        assert exc_info.value.current_state == GripperState.UNLOCKED
        assert exc_info.value.requested_op == "hold_tick"

    def test_hold_tick_logs_correction_event(self):
        """Correction emits a gripper_hold_correction event to the logger.

        When position drifts below locked_low_deg, the overtravel-aware hold
        logic re-asserts and logs a gripper_hold_correction event.
        """
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=False,
        )
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0  # overtravel goal

        transport.feed(_write_response(6))
        # Position below locked_low_deg (145.0) -> drift toward open -> correction
        with patch.object(sm._bus, "read_position_deg", return_value=140.0):
            sm.hold_tick()

        # Logger should have been called with hold correction event
        called_event_types = [
            call.kwargs.get("event_type")
            for call in mock_logger.warning.call_args_list
            if "event_type" in call.kwargs
        ]
        assert "gripper_hold_correction" in called_event_types

    def test_hold_tick_no_commanded_pos_returns_false(self):
        """hold_tick with no commanded position yet: returns False immediately."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = None
        result = sm.hold_tick()
        assert result is False


# ---------------------------------------------------------------------------
# AC#6: Unlock safety (opposite-arm holding predicate)
# ---------------------------------------------------------------------------


class TestUnlockSafety:
    """AC#6: Unlock refused when opposite arm is not LOCKED."""

    def test_unlock_refused_when_no_predicate(self):
        """No opposite_arm_locked_fn injected -> unlock always refused."""
        sm = _dry_sm(opposite_fn=None)
        sm._opposite_arm_locked_fn = None
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(UnlockRefused) as exc_info:
            sm.unlock()
        assert exc_info.value.role == "arm_left"
        assert "predicate" in str(exc_info.value).lower()

    def test_unlock_refused_when_opposite_not_locked(self):
        """Opposite-arm predicate returns False -> unlock refused."""
        sm = _dry_sm(opposite_fn=lambda: False)
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(UnlockRefused) as exc_info:
            sm.unlock()
        assert "opposite arm" in str(exc_info.value).lower()

    def test_unlock_refused_when_predicate_raises(self):
        """Opposite-arm predicate raises -> unlock refused with the cause."""
        def bad_fn():
            raise RuntimeError("arm bus offline")

        sm = _dry_sm(opposite_fn=bad_fn)
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(UnlockRefused) as exc_info:
            sm.unlock()
        assert "arm bus offline" in str(exc_info.value)

    def test_unlock_succeeds_when_opposite_locked(self):
        """Opposite-arm predicate returns True -> unlock succeeds."""
        sm = _dry_sm(opposite_fn=lambda: True)
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()
        assert sm.state == GripperState.UNLOCKED

    def test_unlock_refused_is_gripper_lock_error(self):
        sm = _dry_sm(opposite_fn=lambda: False)
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        with pytest.raises(GripperLockError):
            sm.unlock()

    def test_unlock_under_load_logs_warning(self):
        """When load exceeds jam_load_threshold_raw, a WARNING event is emitted."""
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=False,
        )
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 155.0

        # Under-load state: load_raw=300 > jam_load_threshold_raw(200) -> warning
        under_load_state = JointState(
            servo_id=6,
            position_deg=155.0,
            speed_raw=0,
            load_raw=300,  # > jam_load_threshold_raw(200)
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )
        # Feed write response for unlock command
        transport.feed(_write_response(6))
        with patch.object(sm._bus, "read_joint_state", return_value=under_load_state):
            sm.unlock()

        called_event_types = [
            call.kwargs.get("event_type")
            for call in mock_logger.warning.call_args_list
            if "event_type" in call.kwargs
        ]
        assert "gripper_unlock_under_load" in called_event_types


# ---------------------------------------------------------------------------
# AC#5: Policy-facing adapter hard-blocks gripper keys
# ---------------------------------------------------------------------------


class TestPolicyActions:
    """AC#5: Policy adapter hard-blocks gripper; no token import outside AA-17."""

    def test_validate_policy_action_passes_valid_joints(self):
        action = {"shoulder_pan": 10.0, "elbow_flex": 5.0}
        result = validate_policy_action(action)
        assert result == action

    def test_validate_policy_action_raises_on_gripper_key(self):
        with pytest.raises(GripperCommandBlocked) as exc_info:
            validate_policy_action({"shoulder_pan": 10.0, "gripper": 90.0})
        assert "gripper" in exc_info.value.offending_keys
        assert "gripper" in str(exc_info.value)

    def test_validate_policy_action_raises_on_unknown_key(self):
        """Any unknown key (not in POLICY_JOINT_NAMES) is blocked."""
        with pytest.raises(GripperCommandBlocked) as exc_info:
            validate_policy_action({"unknown_joint": 10.0})
        assert "unknown_joint" in exc_info.value.offending_keys

    def test_validate_policy_action_raises_on_gripper_only(self):
        """Gripper-only action dict raises."""
        with pytest.raises(GripperCommandBlocked):
            validate_policy_action({"gripper": 45.0})

    def test_validate_policy_action_all_five_joints_passes(self):
        action = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        }
        result = validate_policy_action(action)
        assert result == action

    def test_policy_joint_names_excludes_gripper(self):
        assert "gripper" not in POLICY_JOINT_NAMES

    def test_policy_joint_names_includes_five_joints(self):
        assert len(POLICY_JOINT_NAMES) == 5

    def test_policy_adapter_apply_calls_write(self):
        """PolicyActionAdapter.apply dispatches through GuardedArm to the bus."""
        import numpy as np
        from inchworm.config import load_config
        from inchworm.motion_guard import build_guarded_arm
        transport = _make_transport()
        bus = _make_bus(transport)
        # Build a GuardedArm (live=False so unverified limits don't block).
        # Provide present_pose_source to avoid bus reads in the guard check.
        cfg = load_config()
        guarded = build_guarded_arm(
            bus=bus, cfg=cfg, arm_role="left", live=False,
            present_pose_source=lambda: np.zeros(5),
        )
        adapter = PolicyActionAdapter(bus=guarded, role="arm_left", dry_run=False)
        transport.feed(_write_response(1))
        # Use 180.0 deg (= 0 rad with fallback calibration) — within URDF limits.
        adapter.apply({"shoulder_pan": 180.0})
        assert len(transport.written) >= 1

    def test_policy_adapter_apply_blocks_gripper(self):
        """PolicyActionAdapter.apply raises GripperCommandBlocked for gripper key."""
        transport = _make_transport()
        bus = _make_bus(transport)
        adapter = PolicyActionAdapter(bus=bus, role="arm_left", dry_run=False)
        with pytest.raises(GripperCommandBlocked):
            adapter.apply({"gripper": 90.0})

    def test_policy_adapter_dry_run_does_not_write(self):
        """PolicyActionAdapter in dry-run: validates but does not send commands."""
        transport = _make_transport()
        bus = _make_bus(transport)
        adapter = PolicyActionAdapter(bus=bus, role="arm_left", dry_run=True)
        adapter.apply({"shoulder_pan": 10.0})
        assert transport.written == []

    def test_policy_adapter_dry_run_still_blocks_gripper(self):
        """Dry-run still raises on gripper key (validate before dry-run check)."""
        transport = _make_transport()
        bus = _make_bus(transport)
        adapter = PolicyActionAdapter(bus=bus, role="arm_left", dry_run=True)
        with pytest.raises(GripperCommandBlocked):
            adapter.apply({"gripper": 90.0})

    def test_gripper_command_blocked_is_not_gripper_lock_error(self):
        """GripperCommandBlocked is independent of gripper_lock errors."""
        assert not issubclass(GripperCommandBlocked, GripperLockError)

    def test_no_token_import_outside_aa17(self):
        """_GRIPPER_TOKEN is not imported by policy_actions or any module outside AA-17.

        This extends the existing test in test_servo_bus.py by also verifying
        that policy_actions.py does not import the token, and performing the
        full scan across all src/inchworm modules.

        Uses AST-based import scanning to avoid false positives from docstring
        mentions of the token name.
        """
        import ast
        import pathlib

        src_root = pathlib.Path(__file__).parent.parent / "src" / "inchworm"

        violations: list[str] = []
        for py_file in sorted(src_root.glob("*.py")):
            stem = py_file.stem
            # Allow: gripper_access.py (defines it), servo_bus.py (gate),
            # gripper_lock*.py (AA-17 authorised caller)
            if stem in ("gripper_access", "servo_bus") or stem.startswith("gripper_lock"):
                continue
            source = py_file.read_text(encoding="utf-8")
            # AST scan: look for any import of gripper_access
            try:
                tree = ast.parse(source)
            except SyntaxError:
                continue
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    if node.module and "gripper_access" in node.module:
                        violations.append(str(py_file.relative_to(src_root.parent.parent)))
                        break
                    if node.names:
                        for alias in node.names:
                            if "_GRIPPER_TOKEN" in (alias.name or ""):
                                violations.append(
                                    str(py_file.relative_to(src_root.parent.parent))
                                )
                                break
                elif isinstance(node, ast.Import):
                    for alias in node.names:
                        if "gripper_access" in alias.name:
                            violations.append(
                                str(py_file.relative_to(src_root.parent.parent))
                            )
                            break

        assert violations == [], (
            f"_GRIPPER_TOKEN imported outside allowed modules: {violations!r}. "
            "Only gripper_access.py, servo_bus.py, and gripper_lock*.py may use it."
        )

    def test_policy_actions_does_not_import_gripper_access(self):
        """policy_actions.py must not import inchworm.gripper_access."""
        import pathlib
        import ast
        src_root = pathlib.Path(__file__).parent.parent / "src" / "inchworm"
        policy_src = (src_root / "policy_actions.py").read_text(encoding="utf-8")
        # Parse imports only (not docstrings or comments)
        tree = ast.parse(policy_src)
        import_names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                import_names.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    import_names.append(node.module)
        assert not any("gripper_access" in name for name in import_names), (
            f"policy_actions.py imports gripper_access: {import_names!r}. "
            "Violates isolation contract."
        )

    def test_policy_actions_does_not_import_gripper_lock(self):
        """policy_actions.py must not import inchworm.gripper_lock."""
        import pathlib
        import ast
        src_root = pathlib.Path(__file__).parent.parent / "src" / "inchworm"
        policy_src = (src_root / "policy_actions.py").read_text(encoding="utf-8")
        tree = ast.parse(policy_src)
        import_names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                import_names.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    import_names.append(node.module)
        assert not any("gripper_lock" in name for name in import_names), (
            f"policy_actions.py imports gripper_lock: {import_names!r}. "
            "Violates isolation contract."
        )


# ---------------------------------------------------------------------------
# Additional integration / edge-case tests
# ---------------------------------------------------------------------------


class TestGripperLockIntegration:
    """AC#7 (additional): integration and edge cases."""

    def test_properties_exposed(self):
        sm = _dry_sm()
        assert sm.role == "arm_left"
        assert sm.dry_run is True
        assert sm.state == GripperState.UNKNOWN

    def test_reset_clears_fault(self):
        sm = _dry_sm()
        sm._state = GripperState.FAULT
        sm.reset()
        assert sm.state == GripperState.UNLOCKED

    def test_lock_config_none_dry_run_passes(self):
        """No config at all (None), dry-run: all transitions succeed."""
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=None,
            opposite_arm_locked_fn=lambda: True,
            dry_run=True,
        )
        sm.reset()
        sm.set_approach_ready()
        verdict = sm.lock()
        assert verdict == LockVerdict.LATCHED
        sm.unlock()
        assert sm.state == GripperState.UNLOCKED

    def test_invalid_role_raises(self):
        transport = _make_transport()
        bus = _make_bus(transport)
        with pytest.raises(ValueError, match="role must be"):
            GripperLockSM(bus=bus, role="trunk", config=None, dry_run=True)

    def test_full_cycle_with_logger(self):
        """Full dry-run cycle with a mock logger: events are emitted."""
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=True,
        )
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()
        # Logger should have been called at least a few times
        assert mock_logger.emit.called or mock_logger.info.called or mock_logger.warning.called

    def test_aa7_gripper_lock_event_emitted_on_lock(self):
        """emit() is called with the gripper_lock event type after a successful lock."""
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=True,
        )
        sm.reset()
        sm.set_approach_ready()
        sm.lock()

        # Inspect mock_logger.emit calls
        emit_calls = mock_logger.emit.call_args_list
        event_types = [
            call.kwargs.get("event_type", "")
            for call in emit_calls
        ]
        assert "gripper_lock" in event_types

    def test_aa7_gripper_unlock_event_emitted_on_unlock(self):
        """emit() is called with the gripper_unlock event type after unlock."""
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=True,
        )
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        sm.unlock()

        emit_calls = mock_logger.emit.call_args_list
        event_types = [
            call.kwargs.get("event_type", "")
            for call in emit_calls
        ]
        assert "gripper_unlock" in event_types

    def test_lock_verification_failed_subclass(self):
        """LockVerificationFailed is a GripperLockError."""
        assert issubclass(LockVerificationFailed, GripperLockError)

    def test_bands_not_configured_subclass(self):
        assert issubclass(BandsNotConfigured, GripperLockError)

    def test_unlock_refused_subclass(self):
        assert issubclass(UnlockRefused, GripperLockError)

    def test_illegal_transition_subclass(self):
        assert issubclass(IllegalTransition, GripperLockError)


# --- Orchestrator 2026-10-24: hold loss must escalate, not stay "LOCKED" ---
def test_hold_tick_escalates_to_fault_after_repeated_read_failures():
    from inchworm.gripper_lock import GripperLockError, _HOLD_MAX_READ_FAILURES
    from inchworm.servo_bus import ServoTimeoutError
    transport = _make_transport()
    sm = _live_sm(transport)
    sm._state = GripperState.LOCKED
    sm._last_commanded_pos_deg = 155.0
    with patch.object(sm._bus, "read_position_deg", side_effect=ServoTimeoutError(6, "read")):
        for _ in range(_HOLD_MAX_READ_FAILURES - 1):
            assert sm.hold_tick() is False
            assert sm.state == GripperState.LOCKED
        with pytest.raises(GripperLockError, match="cannot be verified"):
            sm.hold_tick()
    assert sm.state == GripperState.FAULT


def test_hold_tick_read_failure_counter_resets_on_success():
    from inchworm.servo_bus import ServoTimeoutError
    transport = _make_transport()
    sm = _live_sm(transport)
    sm._state = GripperState.LOCKED
    sm._last_commanded_pos_deg = 155.0
    with patch.object(sm._bus, "read_position_deg", side_effect=ServoTimeoutError(6, "read")):
        sm.hold_tick()
    with patch.object(sm._bus, "read_position_deg", return_value=155.0):
        sm.hold_tick()
    assert sm._hold_read_failures == 0


# --- Orchestrator 2026-10-24 audit finding 6 ---
def test_unlock_refuses_when_gripper_state_unreadable():
    from inchworm.gripper_lock import UnlockRefused
    from inchworm.servo_bus import ServoTimeoutError
    transport = _make_transport()
    sm = _live_sm(transport)
    sm._state = GripperState.LOCKED
    sm._opposite_arm_locked_fn = lambda: True
    with patch.object(sm._bus, "read_joint_state", side_effect=ServoTimeoutError(6, "read")):
        with pytest.raises(UnlockRefused, match="cannot read"):
            sm.unlock()
    assert sm.state == GripperState.LOCKED


# ---------------------------------------------------------------------------
# AA-17 overtravel / key-detection tests (2026-10-25)
# ---------------------------------------------------------------------------


# --- Config ordering validation ---

class TestOvertravelConfigOrdering:
    """Ordering validator in GripperBandConfig rejects invalid orderings at load time."""

    def _valid_bands(self, **overrides):
        fields = dict(
            unlocked_low_deg=80.0,
            unlocked_high_deg=100.0,
            engaged_low_deg=80.0,
            engaged_high_deg=100.0,
            locked_low_deg=145.0,
            locked_high_deg=165.0,
            lock_overtravel_goal_deg=200.0,
            closed_empty_deg=250.0,
            lock_torque_limit_raw=300,
            unverified=False,
        )
        fields.update(overrides)
        return GripperBandConfig(**fields)

    def test_valid_ordering_accepted(self):
        """Correctly ordered config loads without error."""
        bands = self._valid_bands()
        assert bands.lock_overtravel_goal_deg == 200.0
        assert bands.closed_empty_deg == 250.0

    def test_overtravel_equal_to_locked_high_rejected(self):
        """lock_overtravel_goal_deg == locked_high_deg must be rejected."""
        from pydantic import ValidationError as PydanticValidationError
        with pytest.raises((ValueError, PydanticValidationError)):
            self._valid_bands(lock_overtravel_goal_deg=165.0)  # == locked_high

    def test_overtravel_less_than_locked_high_rejected(self):
        """lock_overtravel_goal_deg < locked_high_deg must be rejected."""
        from pydantic import ValidationError as PydanticValidationError
        with pytest.raises((ValueError, PydanticValidationError)):
            self._valid_bands(lock_overtravel_goal_deg=150.0)  # < locked_high

    def test_overtravel_equal_to_closed_empty_rejected(self):
        """lock_overtravel_goal_deg == closed_empty_deg must be rejected."""
        from pydantic import ValidationError as PydanticValidationError
        with pytest.raises((ValueError, PydanticValidationError)):
            self._valid_bands(lock_overtravel_goal_deg=250.0)  # == closed_empty

    def test_overtravel_greater_than_closed_empty_rejected(self):
        """lock_overtravel_goal_deg > closed_empty_deg must be rejected."""
        from pydantic import ValidationError as PydanticValidationError
        with pytest.raises((ValueError, PydanticValidationError)):
            self._valid_bands(lock_overtravel_goal_deg=260.0)  # > closed_empty

    def test_unlocked_high_above_locked_low_rejected(self):
        """unlocked_high_deg >= locked_low_deg must be rejected."""
        from pydantic import ValidationError as PydanticValidationError
        with pytest.raises((ValueError, PydanticValidationError)):
            self._valid_bands(unlocked_high_deg=150.0)  # > locked_low

    def test_null_overtravel_skips_ordering_check(self):
        """When lock_overtravel_goal_deg is null, ordering check is skipped."""
        bands = GripperBandConfig(
            locked_low_deg=145.0,
            locked_high_deg=165.0,
            lock_overtravel_goal_deg=None,
            closed_empty_deg=None,
        )
        assert bands.lock_overtravel_goal_deg is None

    def test_hardware_yaml_bands_pass_ordering(self):
        """The values from hardware.yaml (2026-10-25) pass the ordering validator."""
        from inchworm.config import load_config
        cfg = load_config()
        assert cfg.gripper_lock is not None
        left = cfg.gripper_lock.arm_left
        right = cfg.gripper_lock.arm_right
        assert left is not None and right is not None
        # Ordering is validated at load; reaching here means it passed.
        assert left.lock_overtravel_goal_deg is not None
        assert right.lock_overtravel_goal_deg is not None
        assert left.locked_high_deg < left.lock_overtravel_goal_deg < left.closed_empty_deg
        assert right.locked_high_deg < right.lock_overtravel_goal_deg < right.closed_empty_deg


# --- Live lock: overtravel goal commanded, torque limit applied before goal ---

class TestOvertravelLock:
    """lock() commands overtravel goal; torque limit applied before goal in live mode."""

    def test_lock_commands_overtravel_goal_not_midpoint(self):
        """In dry-run, lock() commands lock_overtravel_goal_deg, not the locked-band midpoint."""
        sm = _dry_sm()
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        # overtravel goal = 200.0 deg; midpoint would be (145+165)/2 = 155.0
        assert sm._last_commanded_pos_deg == 200.0

    def test_live_lock_applies_torque_limit_before_goal(self):
        """live lock() calls set_torque_limit with lock_torque_limit_raw before write_position_deg."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.APPROACH_READY

        calls = []

        original_set_torque = sm._bus.set_torque_limit
        original_write_pos = sm._channel.write_position_deg

        def recording_torque(servo_id, torque_limit_raw):
            calls.append(("torque", servo_id, torque_limit_raw))
            transport.queue_reply(_write_response(servo_id))
            original_set_torque(servo_id, torque_limit_raw)

        def recording_write_pos(pos_deg):
            calls.append(("goal_pos", pos_deg))
            transport.queue_reply(_write_response(6))
            original_write_pos(pos_deg)

        latched_state = JointState(
            servo_id=6,
            position_deg=155.0,
            speed_raw=0,
            load_raw=50,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )

        with patch.object(sm._bus, "set_torque_limit", side_effect=recording_torque), \
             patch.object(sm._channel, "write_position_deg", side_effect=recording_write_pos), \
             patch.object(sm._bus, "read_joint_state", return_value=latched_state):
            sm.lock()

        assert len(calls) == 2, f"Expected 2 calls (torque + goal), got: {calls!r}"
        assert calls[0][0] == "torque", f"First call should be torque limit, got: {calls[0]}"
        assert calls[1][0] == "goal_pos", f"Second call should be goal pos, got: {calls[1]}"
        # torque limit value should match config
        assert calls[0][2] == 300  # _BANDS_LEFT.lock_torque_limit_raw
        # goal position should be overtravel goal
        assert calls[1][1] == 200.0  # _BANDS_LEFT.lock_overtravel_goal_deg

    def test_live_lock_null_overtravel_goal_refuses(self):
        """Live lock raises OvertravelNotConfigured when lock_overtravel_goal_deg is null."""
        from inchworm.gripper_lock import OvertravelNotConfigured
        from inchworm.config import GripperBandConfig, GripperLockConfig, GripperVerificationThresholds
        bands_no_ot = GripperBandConfig(
            unlocked_low_deg=80.0,
            unlocked_high_deg=100.0,
            engaged_low_deg=80.0,
            engaged_high_deg=100.0,
            locked_low_deg=145.0,
            locked_high_deg=165.0,
            lock_overtravel_goal_deg=None,
            closed_empty_deg=250.0,
            lock_torque_limit_raw=300,
            unverified=False,
        )
        cfg = GripperLockConfig(
            arm_left=bands_no_ot,
            arm_right=bands_no_ot,
            verification=_THRESHOLDS,
        )
        transport = _make_transport()
        sm = _live_sm(transport, lock_config=cfg)
        sm._state = GripperState.APPROACH_READY
        with pytest.raises(OvertravelNotConfigured) as exc_info:
            sm.lock()
        assert "lock_overtravel_goal_deg" in str(exc_info.value)
        assert isinstance(exc_info.value, GripperLockError)

    def test_live_lock_null_torque_limit_refuses(self):
        """Live lock raises OvertravelNotConfigured when lock_torque_limit_raw is null."""
        from inchworm.gripper_lock import OvertravelNotConfigured
        from inchworm.config import GripperBandConfig, GripperLockConfig
        bands_no_tl = GripperBandConfig(
            unlocked_low_deg=80.0,
            unlocked_high_deg=100.0,
            engaged_low_deg=80.0,
            engaged_high_deg=100.0,
            locked_low_deg=145.0,
            locked_high_deg=165.0,
            lock_overtravel_goal_deg=200.0,
            closed_empty_deg=250.0,
            lock_torque_limit_raw=None,
            unverified=False,
        )
        cfg = GripperLockConfig(
            arm_left=bands_no_tl,
            arm_right=bands_no_tl,
            verification=_THRESHOLDS,
        )
        transport = _make_transport()
        sm = _live_sm(transport, lock_config=cfg)
        sm._state = GripperState.APPROACH_READY
        with pytest.raises(OvertravelNotConfigured) as exc_info:
            sm.lock()
        assert "lock_torque_limit_raw" in str(exc_info.value)

    def test_dry_run_null_overtravel_still_succeeds(self):
        """Dry-run with null overtravel fields: lock succeeds (overtravel check skipped)."""
        from inchworm.config import GripperBandConfig, GripperLockConfig
        bands_no_ot = GripperBandConfig(
            unlocked_low_deg=80.0,
            unlocked_high_deg=100.0,
            engaged_low_deg=80.0,
            engaged_high_deg=100.0,
            locked_low_deg=145.0,
            locked_high_deg=165.0,
            lock_overtravel_goal_deg=None,
            closed_empty_deg=None,
            unverified=True,
        )
        cfg = GripperLockConfig(arm_left=bands_no_ot, arm_right=bands_no_ot)
        sm = _dry_sm(lock_config=cfg)
        sm.reset()
        sm.set_approach_ready()
        verdict = sm.lock()
        assert verdict == LockVerdict.LATCHED


# --- Verification: LATCHED requires position within locked band ---

class TestOvertravelVerification:
    """verify_lock_state correctly classifies key-present vs no-key scenarios."""

    def _make_state(self, pos_deg, load_raw=50, current_raw=100):
        return JointState(
            servo_id=6,
            position_deg=pos_deg,
            speed_raw=0,
            load_raw=load_raw,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=current_raw,
        )

    def test_position_in_locked_band_is_latched(self):
        """Key present: servo stalled in band -> LATCHED."""
        state = self._make_state(155.0)  # inside [145, 165]
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.LATCHED

    def test_position_beyond_locked_high_is_miss_no_key_overtravel(self):
        """No key: servo reached overtravel goal (beyond locked_high) -> MISS_NO_KEY_OVERTRAVEL."""
        state = self._make_state(200.0)  # > locked_high (165) and < closed_empty (250)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.MISS_NO_KEY_OVERTRAVEL

    def test_miss_no_key_overtravel_reason_in_lock_failed_detail(self):
        """LockVerificationFailed raised with no_key_overtravel in detail."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.APPROACH_READY

        transport.queue_reply(_write_response(6))  # torque limit
        transport.queue_reply(_write_response(6))  # goal position
        overtravel_state = JointState(
            servo_id=6,
            position_deg=200.0,  # beyond locked_high (165) -> no key
            speed_raw=0,
            load_raw=50,
            voltage_v=12.0,
            temperature_c=35,
            current_raw=100,
        )
        with patch.object(sm._bus, "read_joint_state", return_value=overtravel_state):
            with pytest.raises(LockVerificationFailed) as exc_info:
                sm.lock()
        assert exc_info.value.verdict == LockVerdict.MISS_NO_KEY_OVERTRAVEL
        assert "no_key_overtravel" in exc_info.value.detail
        assert sm.state == GripperState.FAULT

    def test_position_below_locked_low_is_miss(self):
        """Position below locked band (e.g. jammed open) -> MISS (not overtravel)."""
        state = self._make_state(90.0)  # below locked_low (145)
        verdict = verify_lock_state(state, _BANDS_LEFT, _THRESHOLDS)
        assert verdict == LockVerdict.MISS

    def test_miss_no_key_overtravel_is_gripper_lock_error(self):
        """LockVerificationFailed(MISS_NO_KEY_OVERTRAVEL) is a GripperLockError."""
        exc = LockVerificationFailed(LockVerdict.MISS_NO_KEY_OVERTRAVEL, "no_key_overtravel: pos=200")
        assert isinstance(exc, GripperLockError)
        assert exc.verdict == LockVerdict.MISS_NO_KEY_OVERTRAVEL


# --- Hold monitor: overtravel-aware band-based hold ---

class TestOvertravelHold:
    """hold_tick uses band-based logic when overtravel is configured."""

    def test_hold_healthy_while_stalled_in_band_no_reassert(self):
        """Position inside locked band while stalled on key: no correction, returns False."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0  # overtravel goal

        # Position inside locked band [145, 165]: healthy hold, no correction
        with patch.object(sm._bus, "read_position_deg", return_value=155.0):
            result = sm.hold_tick()
        assert result is False
        assert sm.state == GripperState.LOCKED

    def test_hold_healthy_multiple_ticks_no_spam(self):
        """Multiple hold_tick calls while stalled in band: none trigger correction."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0

        with patch.object(sm._bus, "read_position_deg", return_value=158.0):
            for _ in range(10):
                result = sm.hold_tick()
                assert result is False
        assert sm.state == GripperState.LOCKED
        # No writes should have been made
        assert transport.written == []

    def test_slip_toward_closed_faults_with_hold_slip_fault(self):
        """Position beyond locked_high toward closed_empty: HoldSlipFault raised, state=FAULT."""
        from inchworm.gripper_lock import HoldSlipFault
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0

        # Position beyond locked_high (165) -> key lost/slipped
        with patch.object(sm._bus, "read_position_deg", return_value=180.0):
            with pytest.raises(HoldSlipFault) as exc_info:
                sm.hold_tick()
        assert sm.state == GripperState.FAULT
        assert exc_info.value.role == "arm_left"
        assert exc_info.value.present_deg == 180.0
        assert isinstance(exc_info.value, GripperLockError)

    def test_slip_toward_closed_logs_fault_event(self):
        """HoldSlipFault emits gripper_hold_slip_fault error event to logger."""
        from inchworm.gripper_lock import HoldSlipFault
        mock_logger = MagicMock()
        transport = _make_transport()
        bus = _make_bus(transport)
        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=_LOCK_CONFIG,
            opposite_arm_locked_fn=lambda: True,
            logger=mock_logger,
            dry_run=False,
        )
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0

        with patch.object(sm._bus, "read_position_deg", return_value=180.0):
            with pytest.raises(HoldSlipFault):
                sm.hold_tick()

        called_event_types = [
            call.kwargs.get("event_type")
            for call in mock_logger.error.call_args_list
            if "event_type" in call.kwargs
        ]
        assert "gripper_hold_slip_fault" in called_event_types

    def test_drift_toward_open_reasserts_overtravel_goal(self):
        """Position below locked_low: hold re-asserts overtravel goal and returns True."""
        transport = _make_transport()
        sm = _live_sm(transport)
        sm._state = GripperState.LOCKED
        sm._last_commanded_pos_deg = 200.0

        transport.feed(_write_response(6))
        # Position below locked_low (145) -> drift toward open
        with patch.object(sm._bus, "read_position_deg", return_value=140.0):
            result = sm.hold_tick()
        assert result is True
        assert sm.state == GripperState.LOCKED

        # The write should encode the overtravel goal (200.0 deg)
        import struct as st
        written = transport.written[-1]
        tick_bytes = written[6:8]
        written_ticks = st.unpack("<H", tick_bytes)[0]
        from inchworm.servo_bus import ticks_to_degrees
        written_deg = ticks_to_degrees(written_ticks)
        assert abs(written_deg - 200.0) < 1.0

    def test_hold_slip_fault_is_gripper_lock_error(self):
        """HoldSlipFault is a GripperLockError."""
        from inchworm.gripper_lock import HoldSlipFault
        assert issubclass(HoldSlipFault, GripperLockError)

    def test_overtravel_not_configured_is_gripper_lock_error(self):
        """OvertravelNotConfigured is a GripperLockError."""
        from inchworm.gripper_lock import OvertravelNotConfigured
        assert issubclass(OvertravelNotConfigured, GripperLockError)
