"""Tests for inchworm.homing (AA-10 AC#1 and AC#2).

All tests run without hardware. No real serial ports are opened.

AC#1: Homing brings arm to neutral pose with velocity/current limits applied,
      and refuses to run live if joint limits are not verified.

AC#2: Gripper locked band is checked; homing aborts with a typed reason if
      the gripper reads within the locked band or the band is unknown.

Coverage:
- Dry-run homing executes without servo errors (no bytes sent)
- Live homing refused when joint_limits_verified = False
- Gripper locked-band null -> HomingAbortedError (fail-closed) when servos present
- Gripper locked-band null + no servos in map -> check skipped (test scenario)
- Gripper within locked band -> HomingAbortedError
- Gripper outside locked band -> homing proceeds
- Missing homing config -> HomingConfigMissingError
- home_all returns per-arm result dict
"""

from __future__ import annotations

from typing import Dict, Optional

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import load_config, HardwareConfig
from inchworm.homing import (
    HomingAbortedError,
    HomingConfigMissingError,
    HomingLimitsNotVerifiedError,
    HomingRoutine,
)
from inchworm.servo_bus import ArmServoBus


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

JOINT_MAP_FULL = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

JOINT_MAP_EMPTY: Dict[str, Optional[int]] = {
    "shoulder_pan": None,
    "shoulder_lift": None,
    "elbow_flex": None,
    "wrist_flex": None,
    "wrist_roll": None,
    "gripper": None,
}


def _make_bus(
    role: str = "arm_left",
    joint_map: Optional[dict] = None,
    dry_run: bool = True,
) -> ArmServoBus:
    t = MockSerialTransport(role)
    jmap = joint_map if joint_map is not None else {k: None for k in JOINT_MAP_FULL}
    return ArmServoBus(t, role=role, joint_id_map=jmap, dry_run=dry_run)


def _config_with_band(
    low: Optional[float], high: Optional[float]
) -> HardwareConfig:
    """Load config and patch fault_thresholds gripper locked band."""
    import copy
    cfg = load_config()
    # Rebuild with patched fault_thresholds using model_copy (pydantic v2)
    if cfg.fault_thresholds is not None:
        ft = cfg.fault_thresholds.model_copy(
            update={
                "gripper_locked_band_deg_low": low,
                "gripper_locked_band_deg_high": high,
            }
        )
    else:
        from inchworm.config import FaultThresholds
        ft = FaultThresholds(
            gripper_locked_band_deg_low=low,
            gripper_locked_band_deg_high=high,
        )
    return cfg.model_copy(update={"fault_thresholds": ft})


# ---------------------------------------------------------------------------
# Tests: basic dry-run
# ---------------------------------------------------------------------------


class TestHomingDryRun:
    def test_dry_run_succeeds_with_null_servo_ids(self):
        """Dry-run homing with null servo IDs (nothing to command)."""
        cfg = load_config()
        bus = _make_bus()
        routine = HomingRoutine({"arm_left": bus}, cfg)
        # With null servo IDs AND null gripper band -> fail-closed.
        # But with null servo IDs and no servos in map, check is skipped.
        routine.home_arm("arm_left", dry_run=True)

    def test_dry_run_with_full_joint_map(self):
        """Dry-run homing with a full joint map and known non-locked band."""
        cfg = _config_with_band(100.0, 200.0)
        # Give all joints IDs but use dry_run on the bus so no transport writes.
        bus = _make_bus(joint_map=JOINT_MAP_FULL, dry_run=True)
        routine = HomingRoutine({"arm_left": bus}, cfg)
        # Gripper position not in [100, 200] band -- dry_run skips actual read.
        routine.home_arm("arm_left", dry_run=True)

    def test_home_all_dry_run(self):
        cfg = _config_with_band(100.0, 200.0)
        bus_l = _make_bus("arm_left", dry_run=True)
        bus_r = _make_bus("arm_right", dry_run=True)
        routine = HomingRoutine({"arm_left": bus_l, "arm_right": bus_r}, cfg)
        results = routine.home_all(dry_run=True)
        assert results["arm_left"] is None
        assert results["arm_right"] is None


# ---------------------------------------------------------------------------
# Tests: AC#1 -- refuses live without joint limits
# ---------------------------------------------------------------------------


class TestHomingLiveRefused:
    def test_live_refused_when_limits_not_verified(self):
        """AC#1: live homing must be refused if joint_limits_verified = False."""
        cfg = load_config()
        # Default config has joint_limits_verified = False.
        assert cfg.homing is not None
        assert cfg.homing.joint_limits_verified is False

        cfg2 = _config_with_band(100.0, 200.0)
        bus = _make_bus(dry_run=True)
        routine = HomingRoutine({"arm_left": bus}, cfg2)
        with pytest.raises(HomingLimitsNotVerifiedError):
            routine.home_arm("arm_left", dry_run=False)

    def test_dry_run_allowed_when_limits_not_verified(self):
        """Dry-run is always permitted regardless of joint_limits_verified."""
        cfg = load_config()
        bus = _make_bus()
        routine = HomingRoutine({"arm_left": bus}, cfg)
        # Should not raise.
        routine.home_arm("arm_left", dry_run=True)


# ---------------------------------------------------------------------------
# Tests: AC#2 -- gripper locked band
# ---------------------------------------------------------------------------


class TestGripperLockedBand:
    def test_null_band_no_servos_skips_check(self):
        """Null band with no servo IDs in map: gripper check is skipped."""
        cfg = _config_with_band(None, None)
        # All null servo IDs -> no servos -> check skipped.
        bus = _make_bus(joint_map={k: None for k in JOINT_MAP_FULL})
        routine = HomingRoutine({"arm_left": bus}, cfg)
        # Should not raise (check skipped because no servo IDs).
        routine.home_arm("arm_left", dry_run=True)

    def test_null_band_with_servos_aborts(self):
        """AC#2: null locked-band + servo IDs present -> abort (fail-closed)."""
        cfg = _config_with_band(None, None)
        bus = _make_bus(joint_map=JOINT_MAP_FULL, dry_run=True)
        routine = HomingRoutine({"arm_left": bus}, cfg)
        with pytest.raises(HomingAbortedError) as exc_info:
            routine.home_arm("arm_left", dry_run=True)
        assert exc_info.value.reason == "gripper_possibly_locked"
        assert "null" in exc_info.value.detail.lower() or "band" in exc_info.value.detail.lower()

    def test_known_band_gripper_inside_aborts(self):
        """AC#2: gripper reads within locked band -> HomingAbortedError."""
        # Band: [90, 110] degrees.
        cfg = _config_with_band(90.0, 110.0)
        # We need a bus that returns gripper position within the band when read.
        # Use a real (non-dry_run) bus but feed mock response bytes.
        t = MockSerialTransport("arm_left")

        # Feed a valid STS3215 read response for the gripper servo (id=6).
        # We need a read of present_position: 15 bytes starting at 0x38.
        # Position at ~100 degrees: 100/360 * 4096 = 1138 ticks.
        # Build a minimal fake status packet.
        # Format: 0xFF 0xFF ID LEN ERR payload... CHK
        import struct

        def _status_packet(servo_id: int, payload: bytes) -> bytes:
            length = len(payload) + 2  # payload + ERR + CHK
            err = 0
            chk_input = servo_id + length + err + sum(payload)
            chk = (~chk_input) & 0xFF
            return bytes([0xFF, 0xFF, servo_id, length, err]) + payload + bytes([chk])

        # 15-byte block: position=1138 ticks, rest zeros.
        pos_ticks = round(100.0 / 360 * 4096)  # ~100 deg
        payload = struct.pack("<H", pos_ticks) + bytes(13)  # 2 + 13 = 15 bytes
        packet = _status_packet(6, payload)
        t.feed(packet)

        bus = ArmServoBus(t, role="arm_left", joint_id_map=JOINT_MAP_FULL, dry_run=False)
        routine = HomingRoutine({"arm_left": bus}, cfg)

        # In dry-run mode the actual read is skipped (just logged).
        # To test the band check with a real read we need dry_run=False
        # but the limits check blocks us.  Patch joint_limits_verified.
        from inchworm.config import HomingConfig
        homing_verified = cfg.homing.model_copy(update={"joint_limits_verified": True})
        cfg2 = cfg.model_copy(update={"homing": homing_verified})
        routine2 = HomingRoutine({"arm_left": bus}, cfg2)

        with pytest.raises(HomingAbortedError) as exc_info:
            routine2.home_arm("arm_left", dry_run=False)
        assert exc_info.value.reason == "gripper_possibly_locked"
        assert "band" in exc_info.value.detail.lower()

    def test_known_band_gripper_outside_proceeds(self):
        """AC#2: gripper reads OUTSIDE locked band -> no abort.

        We verify the gripper-check logic specifically using a bus whose
        read_position_deg is mocked directly, so the test does not depend on
        the full STS3215 mock-transport protocol.
        """
        # Band: [90, 110]. We simulate gripper at ~180 deg (outside band).
        cfg = _config_with_band(90.0, 110.0)
        bus = _make_bus(joint_map=JOINT_MAP_FULL, dry_run=True)

        # Patch read_position_deg to return 180 deg for the gripper servo.
        original_read = bus.read_position_deg
        def _patched_read(servo_id: int) -> float:
            if servo_id == JOINT_MAP_FULL["gripper"]:
                return 180.0
            return original_read(servo_id)
        bus.read_position_deg = _patched_read  # type: ignore[method-assign]

        # With dry_run=True on the bus, the actual read is skipped by
        # _check_gripper_not_locked (which calls bus.read_position_deg only
        # when dry_run=False). So the check passes in dry_run mode.
        # To test the band logic directly, call _check_gripper_not_locked
        # with dry_run=False using the patched bus.
        cfg2 = cfg.model_copy(update={
            "homing": cfg.homing.model_copy(update={"joint_limits_verified": True})
        })
        routine = HomingRoutine({"arm_left": bus}, cfg2)

        # Call the gripper check with dry_run=False to exercise the band logic.
        # Should NOT raise HomingAbortedError because 180 is outside [90, 110].
        try:
            routine._check_gripper_not_locked("arm_left", bus, dry_run=False)
        except HomingAbortedError:
            pytest.fail("HomingAbortedError should not be raised when gripper is outside band")

    def test_abort_does_not_unlock_gripper(self):
        """AC#2: aborted homing must never write to the gripper servo."""
        cfg = _config_with_band(None, None)
        t = MockSerialTransport("arm_left")
        bus = ArmServoBus(t, role="arm_left", joint_id_map=JOINT_MAP_FULL, dry_run=True)
        routine = HomingRoutine({"arm_left": bus}, cfg)

        with pytest.raises(HomingAbortedError):
            routine.home_arm("arm_left", dry_run=True)

        # Nothing should have been written to the transport.
        assert t.written == []


# ---------------------------------------------------------------------------
# Tests: missing config
# ---------------------------------------------------------------------------


class TestMissingConfig:
    def test_missing_homing_config_raises(self):
        cfg = load_config()
        cfg_no_homing = cfg.model_copy(update={"homing": None})
        bus = _make_bus()
        with pytest.raises(HomingConfigMissingError):
            HomingRoutine({"arm_left": bus}, cfg_no_homing)

    def test_unknown_role_raises(self):
        cfg = load_config()
        bus = _make_bus()
        routine = HomingRoutine({"arm_left": bus}, cfg)
        with pytest.raises(KeyError, match="unknown arm role"):
            routine.home_arm("arm_right", dry_run=True)
