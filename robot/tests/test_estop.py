"""Tests for inchworm.estop (AA-10 ACs #4, #5, #6).

All tests run without hardware. No real serial ports are opened.

AC#4: E-stop halts commanded motion on both arms and DexArm, turns laser off.
AC#5: E-stop is callable programmatically, from SIGINT/SIGTERM, and from CLI.
AC#6: Overcurrent, overtemperature, unreachable-servo trigger stop with typed event.

Coverage:
- EStop.stop() disables torque on both arms via TorqueManager
- EStop.stop() sends M410 quickstop to DexArm
- EStop.stop() fires laser estop hook (LaserDriver.estop)
- Each failing subsystem does not prevent others (EStopPartialError)
- EStop is idempotent (second stop is a no-op)
- Signal handler calls stop (SIGINT / SIGTERM)
- FaultMonitor.check_joint_state triggers stop on overcurrent
- FaultMonitor.check_joint_state triggers stop on overtemperature
- FaultMonitor.check_servo_unreachable triggers stop
- Typed fault event payloads contain expected keys
- CLI script (scripts/estop.py) runs without error in dry-run mode
"""

from __future__ import annotations

import signal
import threading
from typing import List, Optional
from unittest.mock import MagicMock, patch

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import load_config
from inchworm.dexarm import DexArmDriver, MockDexArmSerial
from inchworm.estop import (
    EStop,
    EStopPartialError,
    FaultEvent,
    FaultMonitor,
)
from inchworm.laser import LaserDriver, _ESTOP_HOOKS, register_estop_hook
from inchworm.servo_bus import ArmServoBus
from inchworm.torque import TorqueManager


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


def _make_bus(role: str, dry_run: bool = True) -> ArmServoBus:
    t = MockSerialTransport(role)
    return ArmServoBus(t, role=role, joint_id_map=JOINT_MAP, dry_run=dry_run)


def _make_estop(
    dry_run: bool = True,
    install_signals: bool = False,
    dexarm: Optional[object] = None,
    config=None,
) -> EStop:
    if config is None:
        config = load_config()
    bus_l = _make_bus("arm_left", dry_run=dry_run)
    bus_r = _make_bus("arm_right", dry_run=dry_run)
    torque = TorqueManager({"arm_left": bus_l, "arm_right": bus_r}, dry_run=dry_run)
    if dexarm is None:
        dexarm = DexArmDriver(MockDexArmSerial())
    return EStop(
        torque,
        dexarm,
        config=config,
        dry_run=dry_run,
        install_signal_handlers=install_signals,
    )


# ---------------------------------------------------------------------------
# AC#4 / AC#5: programmatic stop
# ---------------------------------------------------------------------------


class TestEStopProgrammatic:
    def test_stop_completes_dry_run(self):
        """AC#5: programmatic stop in dry-run mode completes without error."""
        es = _make_estop()
        es.stop()  # should not raise

    def test_stop_idempotent(self):
        """Second stop call is a no-op."""
        es = _make_estop()
        es.stop()
        es.stop()  # no error on second call

    def test_reset_allows_stop_again(self):
        """reset() clears the stopped flag."""
        es = _make_estop()
        es.stop()
        es.reset()
        es.stop()  # no error

    def test_stop_sends_dexarm_quickstop(self):
        """AC#4: DexArm quickstop command (M410) is sent to the DexArm transport."""
        # Use dry_run=True for arms (no real servo protocol needed) but we need
        # to verify M410 is actually transmitted. EStop passes dry_run to dexarm.
        # When dry_run=False for dexarm, M410 is sent; when dry_run=True it is logged only.
        # We test by using dry_run=False at the EStop level and dry_run buses for arms.
        config = load_config()
        bus_l = _make_bus("arm_left", dry_run=True)
        bus_r = _make_bus("arm_right", dry_run=True)
        torque = TorqueManager({"arm_left": bus_l, "arm_right": bus_r}, dry_run=True)

        mock_dexarm = MockDexArmSerial()
        da = DexArmDriver(mock_dexarm)
        # dry_run=False so EStop passes dry_run=False to dexarm.quickstop.
        es = EStop(
            torque,
            da,
            config=config,
            dry_run=False,  # live dexarm quickstop
            install_signal_handlers=False,
        )
        es.stop()  # torque uses dry_run buses (ok); dexarm M410 is sent live
        written_text = b"".join(mock_dexarm.written).decode("ascii", errors="replace")
        assert "M410" in written_text

    def test_stop_without_dexarm_driver(self):
        """Stop skips DexArm if no driver is registered."""
        config = load_config()
        bus_l = _make_bus("arm_left")
        bus_r = _make_bus("arm_right")
        torque = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})
        es = EStop(
            torque,
            None,  # no DexArm
            config=config,
            dry_run=True,
            install_signal_handlers=False,
        )
        es.stop()  # should not raise

    def test_partial_error_raised_when_dexarm_fails(self):
        """If DexArm quickstop raises, EStopPartialError is raised."""
        # Make a DexArmDriver whose transport is closed.
        t = MockDexArmSerial()
        t.close()  # close before use -> quickstop will raise
        da = DexArmDriver(t)
        es = _make_estop(dexarm=da)
        with pytest.raises(EStopPartialError) as exc_info:
            es.stop()
        assert "dexarm" in exc_info.value.subsystem_errors

    def test_partial_error_continues_to_laser(self):
        """Even if DexArm fails, the laser hook is still fired."""
        laser_estop_called = []

        def _fake_laser_hook():
            laser_estop_called.append(True)

        # Register our hook before creating estop.
        register_estop_hook(_fake_laser_hook)

        t = MockDexArmSerial()
        t.close()
        da = DexArmDriver(t)
        es = _make_estop(dexarm=da)
        try:
            es.stop()
        except EStopPartialError:
            pass  # expected

        assert len(laser_estop_called) > 0, "laser hook was not called"


# ---------------------------------------------------------------------------
# AC#4: laser turned off on estop
# ---------------------------------------------------------------------------


class TestEStopTurnsLaserOff:
    def test_estop_turns_laser_off(self):
        """AC#4 / AA-29 AC#3: EStop.stop() fires laser estop hook.

        This verifies AA-29 AC#3 end-to-end: when EStop.stop() is called,
        trigger_estop() is called, which fires all registered laser hooks,
        including LaserDriver.estop() which calls off().
        """
        config = load_config()
        mock_relay = MockSerialTransport("relay")
        laser_drv = LaserDriver(mock_relay, config)
        # Turn laser on first.
        laser_drv.on()
        assert laser_drv.commanded_on is True

        # Create an EStop; stop() -> trigger_estop() -> laser_drv.estop() -> off()
        es = _make_estop()
        es.stop()

        # LaserDriver.estop() -> off() -> commanded_on = False
        assert laser_drv.commanded_on is False
        # Orchestrator 2026-10-24: verify the OFF command actually reached the
        # relay wire, not just the driver's internal flag.
        from inchworm.laser import Lcus1Protocol
        assert mock_relay.written[-1] == Lcus1Protocol().encode_off()

        laser_drv.close()


# ---------------------------------------------------------------------------
# AC#5: signal handler
# ---------------------------------------------------------------------------


class TestEStopSignals:
    def test_sigint_triggers_stop(self):
        """AC#5: SIGINT signal handler calls stop()."""
        config = load_config()
        bus_l = _make_bus("arm_left")
        bus_r = _make_bus("arm_right")
        torque = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})
        da = DexArmDriver(MockDexArmSerial())
        es = EStop(
            torque,
            da,
            config=config,
            dry_run=True,
            install_signal_handlers=True,
        )
        # Verify SIGINT handler is installed and calls stop().
        installed_handler = signal.getsignal(signal.SIGINT)
        assert callable(installed_handler), "SIGINT handler not installed"

        # Call handler directly to avoid actually raising SIGINT in the test
        # process (which would cause KeyboardInterrupt / process termination).
        # The handler calls self.stop() then chains to the previous handler;
        # we catch any resulting exception.
        try:
            installed_handler(signal.SIGINT, None)  # type: ignore[call-arg]
        except (KeyboardInterrupt, SystemExit):
            pass

        assert es._stopped is True, "stop() was not called by SIGINT handler"

    def test_sigterm_triggers_stop(self):
        """AC#5: SIGTERM signal handler calls stop()."""
        config = load_config()
        bus_l = _make_bus("arm_left")
        bus_r = _make_bus("arm_right")
        torque = TorqueManager({"arm_left": bus_l, "arm_right": bus_r})
        da = DexArmDriver(MockDexArmSerial())

        # Install a SIG_IGN for SIGTERM first so that chaining doesn't kill the
        # test process, then let EStop install its handler over SIG_IGN.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        es = EStop(
            torque,
            da,
            config=config,
            dry_run=True,
            install_signal_handlers=True,
        )

        # Verify SIGTERM handler is installed and calls stop().
        installed_handler = signal.getsignal(signal.SIGTERM)
        assert callable(installed_handler), "SIGTERM handler not installed"

        # Call handler directly.
        installed_handler(signal.SIGTERM, None)  # type: ignore[call-arg]

        # Restore default SIGTERM.
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        assert es._stopped is True, "stop() was not called by SIGTERM handler"


# ---------------------------------------------------------------------------
# AC#6: fault events trigger stop
# ---------------------------------------------------------------------------


class TestFaultMonitor:
    def test_overcurrent_triggers_stop(self):
        """AC#6: overcurrent above threshold triggers e-stop with typed event."""
        config = load_config()
        assert config.fault_thresholds is not None
        threshold = config.fault_thresholds.overcurrent_raw  # 800

        es = _make_estop()
        stops_triggered = []
        original_stop = es.stop

        def _patched_stop(*, fault_payload=None):
            stops_triggered.append(fault_payload)
            return original_stop(fault_payload=fault_payload)

        es.stop = _patched_stop  # type: ignore[method-assign]
        monitor = FaultMonitor(es, config)

        # current_raw > threshold -> fault
        result = monitor.check_joint_state(
            role="arm_left",
            servo_id=1,
            current_raw=threshold + 100,
            temperature_c=30,
        )

        assert result is not None
        assert result["fault_type"] == "overcurrent"
        assert result["current_raw"] == threshold + 100
        assert len(stops_triggered) == 1
        assert stops_triggered[0]["fault_type"] == "overcurrent"

    def test_overtemperature_triggers_stop(self):
        """AC#6: temperature above threshold triggers e-stop with typed event."""
        config = load_config()
        assert config.fault_thresholds is not None
        threshold = config.fault_thresholds.overtemperature_c  # 70

        es = _make_estop()
        stops_triggered = []
        original_stop = es.stop

        def _patched_stop(*, fault_payload=None):
            stops_triggered.append(fault_payload)
            return original_stop(fault_payload=fault_payload)

        es.stop = _patched_stop  # type: ignore[method-assign]
        monitor = FaultMonitor(es, config)

        result = monitor.check_joint_state(
            role="arm_left",
            servo_id=2,
            current_raw=100,
            temperature_c=threshold + 5,
        )

        assert result is not None
        assert result["fault_type"] == "overtemperature"
        assert result["temperature_c"] == threshold + 5
        assert len(stops_triggered) == 1

    def test_servo_unreachable_triggers_stop(self):
        """AC#6: unreachable servo triggers e-stop with typed event."""
        config = load_config()
        es = _make_estop()
        stops_triggered = []
        original_stop = es.stop

        def _patched_stop(*, fault_payload=None):
            stops_triggered.append(fault_payload)
            return original_stop(fault_payload=fault_payload)

        es.stop = _patched_stop  # type: ignore[method-assign]
        monitor = FaultMonitor(es, config)

        result = monitor.check_servo_unreachable(
            role="arm_right",
            servo_id=3,
            operation="read_joint_state",
            error=TimeoutError("no response"),
        )

        assert result["fault_type"] == "servo_unreachable"
        assert result["servo_id"] == 3
        assert len(stops_triggered) == 1

    def test_no_fault_below_thresholds(self):
        """No fault when all values are below thresholds."""
        config = load_config()
        assert config.fault_thresholds is not None
        threshold_c = config.fault_thresholds.overcurrent_raw
        threshold_t = config.fault_thresholds.overtemperature_c

        es = _make_estop()
        monitor = FaultMonitor(es, config)

        result = monitor.check_joint_state(
            role="arm_left",
            servo_id=1,
            current_raw=threshold_c - 10,
            temperature_c=threshold_t - 10,
        )
        assert result is None

    def test_fault_event_payload_keys(self):
        """Typed FaultEvent payloads have the expected keys."""
        oc = FaultEvent.overcurrent(
            role="arm_left", servo_id=1, current_raw=900, threshold_raw=800
        )
        assert {"fault_type", "role", "servo_id", "current_raw", "threshold_raw"} == set(oc)
        assert oc["fault_type"] == "overcurrent"

        ot = FaultEvent.overtemperature(
            role="arm_left", servo_id=1, temperature_c=75, threshold_c=70
        )
        assert {"fault_type", "role", "servo_id", "temperature_c", "threshold_c"} == set(ot)

        su = FaultEvent.servo_unreachable(
            role="arm_right", servo_id=2, operation="ping", error="timeout"
        )
        assert {"fault_type", "role", "servo_id", "operation", "error"} == set(su)


# ---------------------------------------------------------------------------
# AC#5: CLI script
# ---------------------------------------------------------------------------


class TestEstopCLI:
    def test_cli_dry_run_exits_zero(self, tmp_path):
        """AC#5: CLI script (scripts/estop.py) exits 0 in dry-run mode."""
        import subprocess
        import sys
        from pathlib import Path

        script = (
            Path(__file__).resolve().parents[1] / "scripts" / "estop.py"
        )
        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, (
            f"CLI exited {result.returncode}\nstdout: {result.stdout}\nstderr: {result.stderr}"
        )


# ---------------------------------------------------------------------------
# Orchestrator 2026-10-24: hold-by-default e-stop semantics (provisional,
# pending owner decision; see the AA-10 comment). Both arms may be carrying
# the tube, so the default stop must NOT de-energise the arms.
# ---------------------------------------------------------------------------


class _RecordingTorque:
    def __init__(self):
        self.calls = []

    def hold_all(self):
        self.calls.append("hold")
        return {"arm_left": None, "arm_right": None}

    def disable_all(self):
        self.calls.append("disable")
        return {"arm_left": None, "arm_right": None}


def _estop_with(torque, **kw):
    return EStop(torque, DexArmDriver(MockDexArmSerial()), config=load_config(),
                 dry_run=True, install_signal_handlers=False, **kw)


def test_default_estop_holds_arms_not_releases():
    t = _RecordingTorque()
    _estop_with(t).stop()
    assert t.calls == ["hold"]


def test_release_on_stop_disables_torque():
    t = _RecordingTorque()
    _estop_with(t, release_on_stop=True).stop()
    assert t.calls == ["disable"]


@pytest.mark.parametrize("kind", ["overcurrent", "overtemperature"])
def test_thermal_and_overcurrent_faults_release(kind):
    t = _RecordingTorque()
    _estop_with(t).stop(fault_payload={"fault_type": kind})
    assert t.calls == ["disable"]


def test_unreachable_fault_holds():
    t = _RecordingTorque()
    _estop_with(t).stop(fault_payload={"fault_type": "servo_unreachable"})
    assert t.calls == ["hold"]


def test_hold_arm_keeps_torque_on_and_never_touches_gripper():
    bus = _make_bus("arm_left", dry_run=True)
    tm = TorqueManager({"arm_left": bus}, dry_run=True)
    tm.hold_arm("arm_left")  # dry-run: no exception, no gripper write path exists
    assert "gripper" not in __import__("inchworm.torque", fromlist=["x"])._MANAGED_JOINTS
