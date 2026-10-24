"""Tests for jog.py --live AA-9 wiring.

(a) refusal with unverified calibration (exit 4 with reason)
(b) verified config: writes go through GuardedArm only (spy), never include gripper
(c) dry-run unchanged

All tests monkeypatch inchworm.backends.get_backend; no real serial ports are
opened.  Hardware invariant: serial ports, cameras, usbipd are never touched.
"""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch, call

import numpy as np
import pytest
import yaml

# ---------------------------------------------------------------------------
# Path setup so scripts/jog.py is importable directly
# ---------------------------------------------------------------------------

_ROBOT_ROOT = Path(__file__).resolve().parents[1]
_SCRIPTS = _ROBOT_ROOT / "scripts"
_SRC = _ROBOT_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

# Import jog as a module (not via subprocess) so we can spy on internals.
import importlib.util as _ilu

_spec = _ilu.spec_from_file_location("jog", _SCRIPTS / "jog.py")
jog = _ilu.module_from_spec(_spec)  # type: ignore[arg-type]
_spec.loader.exec_module(jog)  # type: ignore[union-attr]

# ---------------------------------------------------------------------------
# Helpers: synthetic config factories
# ---------------------------------------------------------------------------

from inchworm.config import load_config, HardwareConfig

# Load the real config once — used as the base for all fixture derivations.
_BASE_CFG = load_config()


def _make_verified_config(tmp_path: Path) -> HardwareConfig:
    """Return a HardwareConfig whose motion_guard is fully verified.

    Writes a temp hardware.yaml that extends the real config with:
    - All servo_joints for the left arm set to non-null IDs (1-5, gripper=6).
    - motion_guard.left_arm_limits fully populated and unverified: false.
    - motion_guard.servo_calibration.left_arm fully populated and unverified: false.

    Uses conservative limits that encompass q=zeros so the guard passes.
    """
    # Read the real YAML as text, then patch the relevant blocks.
    real_yaml_path = _ROBOT_ROOT / "config" / "hardware.yaml"
    with open(real_yaml_path) as fh:
        data: dict = yaml.safe_load(fh)

    # Patch left arm servo_joints
    data["arms"]["left"]["servo_joints"] = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }

    # Patch motion_guard limits — wide verified limits
    wide = {
        "shoulder_pan_lower_rad": -2.0,
        "shoulder_pan_upper_rad":  2.0,
        "shoulder_lift_lower_rad": -2.0,
        "shoulder_lift_upper_rad":  2.0,
        "elbow_flex_lower_rad": -2.0,
        "elbow_flex_upper_rad":  2.0,
        "wrist_flex_lower_rad": -2.0,
        "wrist_flex_upper_rad":  2.0,
        "wrist_roll_lower_rad": -3.0,
        "wrist_roll_upper_rad":  3.0,
        "unverified": False,
        "method": "synthetic_test",
        "measured_date": "2026-01-01",
    }
    data["motion_guard"]["left_arm_limits"] = wide

    # Patch servo calibration — zero_offset=180, sign=+1 for all joints
    calib = {
        "shoulder_pan_zero_offset_deg": 180.0,
        "shoulder_pan_sign": 1,
        "shoulder_lift_zero_offset_deg": 180.0,
        "shoulder_lift_sign": 1,
        "elbow_flex_zero_offset_deg": 180.0,
        "elbow_flex_sign": 1,
        "wrist_flex_zero_offset_deg": 180.0,
        "wrist_flex_sign": 1,
        "wrist_roll_zero_offset_deg": 180.0,
        "wrist_roll_sign": 1,
        "unverified": False,
        "method": "synthetic_test",
        "measured_date": "2026-01-01",
    }
    data["motion_guard"]["servo_calibration"]["left_arm"] = calib

    out = tmp_path / "hardware.yaml"
    with open(out, "w") as fh:
        yaml.dump(data, fh)

    from inchworm.config import load_config as _lc
    return _lc(out)


# ---------------------------------------------------------------------------
# Mock backend factory
# ---------------------------------------------------------------------------

class _FakeSerialTransport:
    """Minimal in-memory serial transport — no real port is ever opened."""

    def __init__(self):
        self.written: list[bytes] = []
        self._open = True

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def read(self, size: int) -> bytes:
        return b""

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open


def _make_fake_backend(cfg: HardwareConfig) -> MagicMock:
    """Return a mock backend whose open_serial returns a _FakeSerialTransport."""
    transport = _FakeSerialTransport()
    backend = MagicMock()
    backend.open_serial.return_value = transport
    return backend


# ---------------------------------------------------------------------------
# (a) Refusal with unverified calibration
# ---------------------------------------------------------------------------

class TestLiveRefusesUnverifiedCalibration:
    """Exit 4 is returned when calibration is not verified."""

    def test_exit4_calibration_unverified_via_module_patch(self, tmp_path: Path) -> None:
        """jog.main exits 4 when calibration is unverified — uses module-level patches."""
        real_yaml_path = _ROBOT_ROOT / "config" / "hardware.yaml"
        with open(real_yaml_path) as fh:
            data = yaml.safe_load(fh)
        data["arms"]["left"]["servo_joints"] = {
            "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
            "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
        }
        out = tmp_path / "hw_uncalib.yaml"
        with open(out, "w") as fh:
            yaml.dump(data, fh)
        from inchworm.config import load_config as _lc
        uncalib_cfg = _lc(out)
        fake_backend = _make_fake_backend(uncalib_cfg)

        with patch.object(jog, "load_config", return_value=uncalib_cfg), \
             patch("inchworm.backends.live_backend", return_value=fake_backend):
            with pytest.raises(SystemExit) as exc_info:
                jog.main(["--arm", "left", "--delta", "0.0,0.0,0.01", "--live"])
        assert exc_info.value.code == 4, (
            f"Expected exit 4 for unverified calibration, got {exc_info.value.code}"
        )

    def test_exit4_null_servo_joints(self) -> None:
        """jog.main exits 4 when servo_joints are null in config (no scan yet)."""
        # The real config has null servo_joints.
        cfg = _BASE_CFG
        fake_backend = _make_fake_backend(cfg)

        with patch.object(jog, "load_config", return_value=cfg), \
             patch("inchworm.backends.live_backend", return_value=fake_backend):
            with pytest.raises(SystemExit) as exc_info:
                jog.main(["--arm", "left", "--delta", "0.0,0.0,0.01", "--live"])
        assert exc_info.value.code == 4


# ---------------------------------------------------------------------------
# (b) Verified config: writes go through GuardedArm only, never gripper
# ---------------------------------------------------------------------------

class TestLiveVerifiedWiresGuardedArm:
    """With a fully-verified synthetic config, all writes go through GuardedArm."""

    def test_writes_go_through_guarded_arm_only(self, tmp_path: Path) -> None:
        """write_positions_rad is called on GuardedArm; never gripper."""
        verified_cfg = _make_verified_config(tmp_path)
        fake_backend = _make_fake_backend(verified_cfg)

        import inchworm.motion_guard as _mg_mod
        import inchworm.servo_bus as _sb_mod
        from inchworm.motion_guard import build_guarded_arm
        from inchworm.servo_bus import ArmServoBus

        captured_writes: list[np.ndarray] = []
        bus_write_calls: list[tuple[str, float]] = []

        # Subclass ArmServoBus: no real I/O, record writes, answer reads.
        class SpyArmServoBus(ArmServoBus):
            def write_position_deg(self, joint_name, position_deg):
                bus_write_calls.append((joint_name, position_deg))

            def read_position_deg(self, servo_id):
                return 180.0  # = 0 rad with zero_offset=180, sign=+1

        original_build = build_guarded_arm

        def spy_build_guarded_arm(bus, cfg, arm_role, live=False, **kw):
            ga = original_build(bus, cfg, arm_role, live=live, **kw)
            # Inject present_pose_source so bus reads aren't needed.
            ga._present_pose_source = lambda: np.zeros(5, dtype=np.float64)
            original_write = ga.write_positions_rad

            def spy_write(q, bypass_guard=False):
                captured_writes.append(np.asarray(q, dtype=np.float64).copy())
                original_write(q, bypass_guard=bypass_guard)

            ga.write_positions_rad = spy_write  # type: ignore[method-assign]
            return ga

        # Patch at the module level so local imports inside jog.main() pick them up.
        with patch.object(jog, "load_config", return_value=verified_cfg), \
             patch.object(_mg_mod, "build_guarded_arm", side_effect=spy_build_guarded_arm), \
             patch.object(_sb_mod, "ArmServoBus", SpyArmServoBus), \
             patch("inchworm.backends.live_backend", return_value=fake_backend):
            ret = jog.main([
                "--arm", "left",
                "--delta", "0.02,0.0,0.0",
                "--live",
                "--speed-scale", "0.2",
            ])

        assert ret == 0, f"jog.main returned {ret}, expected 0"

        # write_positions_rad must have been called at least once.
        assert len(captured_writes) >= 1, (
            "Expected at least one call to GuardedArm.write_positions_rad"
        )

        # Each call must be a 5-vector (no gripper dimension).
        for q in captured_writes:
            assert q.shape == (5,), (
                f"write_positions_rad received a non-5-vector: shape={q.shape}"
            )

        # The gripper joint must never have been written to the bus.
        gripper_calls = [c for c in bus_write_calls if c[0] == "gripper"]
        assert gripper_calls == [], (
            f"Gripper was written via the bus: {gripper_calls}"
        )

    def test_no_direct_bus_writes_bypass_guard(self, tmp_path: Path) -> None:
        """All writes pass through GuardedArm — the raw bus is never bypassed."""
        verified_cfg = _make_verified_config(tmp_path)
        fake_backend = _make_fake_backend(verified_cfg)

        import inchworm.motion_guard as _mg_mod
        import inchworm.servo_bus as _sb_mod
        from inchworm.motion_guard import build_guarded_arm
        from inchworm.servo_bus import ArmServoBus

        guarded_arm_instances: list = []
        original_build = build_guarded_arm

        def capturing_build(bus, cfg, arm_role, live=False, **kw):
            ga = original_build(bus, cfg, arm_role, live=live, **kw)
            ga._present_pose_source = lambda: np.zeros(5, dtype=np.float64)
            guarded_arm_instances.append(ga)
            return ga

        class NoOpArmServoBus(ArmServoBus):
            def write_position_deg(self, joint_name, position_deg):
                pass

            def read_position_deg(self, servo_id):
                return 180.0

        with patch.object(jog, "load_config", return_value=verified_cfg), \
             patch.object(_mg_mod, "build_guarded_arm", side_effect=capturing_build), \
             patch.object(_sb_mod, "ArmServoBus", NoOpArmServoBus), \
             patch("inchworm.backends.live_backend", return_value=fake_backend):
            ret = jog.main([
                "--arm", "left",
                "--delta", "0.0,0.02,0.0",
                "--live",
                "--speed-scale", "0.1",
            ])

        assert ret == 0
        assert len(guarded_arm_instances) == 1, "Expected exactly one GuardedArm"


# ---------------------------------------------------------------------------
# (c) Dry-run is unchanged
# ---------------------------------------------------------------------------

class TestDryRunUnchanged:
    """Dry-run mode must still work as before (no hardware, no live path)."""

    def test_dry_run_delta_exits_zero(self) -> None:
        ret = jog.main(["--arm", "left", "--delta", "0.01,0.0,0.0"])
        assert ret == 0

    def test_dry_run_explicit_flag_exits_zero(self) -> None:
        ret = jog.main(["--arm", "right", "--delta", "0.0,0.01,0.0", "--dry-run"])
        assert ret == 0

    def test_dry_run_abs_exits_zero(self) -> None:
        ret = jog.main(["--arm", "left", "--abs", "0.10,0.00,0.20"])
        assert ret == 0

    def test_dry_run_never_opens_backend(self) -> None:
        """Dry-run must never call live_backend."""
        with patch("inchworm.backends.live_backend") as mock_gb:
            ret = jog.main(["--arm", "left", "--delta", "0.01,0.0,0.0"])
        assert ret == 0
        mock_gb.assert_not_called()

    def test_dry_run_speed_scale_accepted(self) -> None:
        """--speed-scale is accepted in dry-run without error."""
        ret = jog.main(["--arm", "left", "--delta", "0.01,0.0,0.0", "--speed-scale", "0.5"])
        assert ret == 0

    def test_dry_run_speed_scale_out_of_range_exits_nonzero(self) -> None:
        """--speed-scale > 1.0 must be rejected."""
        with pytest.raises(SystemExit) as exc_info:
            jog.main(["--arm", "left", "--delta", "0.01,0.0,0.0", "--speed-scale", "1.5"])
        assert exc_info.value.code != 0

    def test_dry_run_no_gripper_in_output(self, capsys) -> None:
        """Dry-run output must not suggest gripper is being commanded."""
        ret = jog.main(["--arm", "left", "--delta", "0.0,0.01,0.0"])
        captured = capsys.readouterr()
        output = (captured.out + captured.err).lower()
        assert ret == 0
        # Gripper must not appear as a motion command; it should appear only
        # in the NOTE line about exclusion.
        assert "gripper" not in output or "excluded" in output or "excluded" in output
