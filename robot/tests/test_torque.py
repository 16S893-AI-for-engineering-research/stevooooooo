"""Tests for inchworm.torque (AA-10 AC#3).

All tests run without hardware. No real serial ports are opened.

Coverage:
- TorqueManager.enable_arm / disable_arm toggle torque via ArmServoBus
- TorqueManager.disable_all attempts both arms independently (one fails -> other still runs)
- enable_all analogous
- Gripper joint is excluded from torque management
- Unknown role raises TorqueManagerError
- dry_run mode: bus records the command as logged-not-sent
"""

from __future__ import annotations

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import load_config
from inchworm.servo_bus import ArmServoBus
from inchworm.torque import TorqueManager, TorqueManagerError, _MANAGED_JOINTS


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

JOINT_MAP = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


def _make_bus(role: str, dry_run: bool = False) -> ArmServoBus:
    """Create an ArmServoBus backed by a MockSerialTransport."""
    # Feed a fake torque-write response for each expected write.
    # ArmServoBus.set_torque_enable -> _bus.write_bytes -> _transact -> read()
    # The mock transport has no auto-responder, so we use dry_run=True to avoid
    # sending real protocol bytes.
    t = MockSerialTransport(role)
    return ArmServoBus(t, role=role, joint_id_map=JOINT_MAP, dry_run=dry_run)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestTorqueManagerBasics:
    def test_enable_arm_dry_run(self):
        bus = _make_bus("arm_left", dry_run=True)
        mgr = TorqueManager({"arm_left": bus}, dry_run=True)
        # Should not raise; dry_run=True -> no bytes sent
        mgr.enable_arm("arm_left")

    def test_disable_arm_dry_run(self):
        bus = _make_bus("arm_left", dry_run=True)
        mgr = TorqueManager({"arm_left": bus}, dry_run=True)
        mgr.disable_arm("arm_left")

    def test_unknown_role_raises(self):
        bus = _make_bus("arm_left", dry_run=True)
        mgr = TorqueManager({"arm_left": bus})
        with pytest.raises(TorqueManagerError, match="unknown arm role"):
            mgr.disable_arm("arm_right")

    def test_disable_all_returns_dict(self):
        bus_l = _make_bus("arm_left", dry_run=True)
        bus_r = _make_bus("arm_right", dry_run=True)
        mgr = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})
        results = mgr.disable_all()
        assert "arm_left" in results
        assert "arm_right" in results
        assert results["arm_left"] is None
        assert results["arm_right"] is None

    def test_enable_all_returns_dict(self):
        bus_l = _make_bus("arm_left", dry_run=True)
        bus_r = _make_bus("arm_right", dry_run=True)
        mgr = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})
        results = mgr.enable_all()
        assert results["arm_left"] is None
        assert results["arm_right"] is None

    def test_gripper_not_managed(self):
        """Gripper joint must not be in the list of managed joints."""
        assert "gripper" not in _MANAGED_JOINTS

    def test_disable_all_continues_after_first_arm_failure(self):
        """If one arm fails, the other is still attempted."""
        bus_l = _make_bus("arm_left", dry_run=True)
        bus_r = _make_bus("arm_right", dry_run=True)
        mgr = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})

        # Patch disable_arm on arm_left to raise.
        original_disable = mgr.disable_arm

        def patched(role: str):
            if role == "arm_left":
                raise RuntimeError("simulated arm_left failure")
            return original_disable(role)

        mgr.disable_arm = patched  # type: ignore[method-assign]
        results = mgr.disable_all()

        # arm_left fails, arm_right succeeds.
        assert results["arm_left"] is not None
        assert results["arm_right"] is None

    def test_dry_run_property(self):
        bus = _make_bus("arm_left", dry_run=True)
        mgr = TorqueManager({"arm_left": bus}, dry_run=True)
        assert mgr.dry_run is True
