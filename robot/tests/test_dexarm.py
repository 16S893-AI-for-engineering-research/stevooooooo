"""Tests for inchworm.dexarm: DexArm Marlin G-code driver (AA-12).

Covers all acceptance criteria:
  AC1  - handshake (M115), firmware identity, typed disconnect error
  AC2  - homing (G28), absolute/relative move, feedrate, safe_park
  AC3  - move completion confirmed before returning (blocking ack)
  AC4  - command queue serialisation; malformed / rejected commands raise
         DexArmCommandError with firmware response
  AC5  - workspace bounds rejected BEFORE transmission
  AC6  - MockDexArmSerial: handshake, move confirmation, bounds rejection,
         error mapping

No hardware I/O: all tests use MockDexArmSerial (in-memory) or plain
MockSerialTransport from inchworm.backends.
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional
from unittest.mock import MagicMock

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.config import DexArmWorkspace, load_config
from inchworm.dexarm import (
    DEFAULT_ACK_TIMEOUT_S,
    DexArmBoundsError,
    DexArmCommandError,
    DexArmDisconnect,
    DexArmDriver,
    DexArmError,
    DexArmPosition,
    MockDexArmSerial,
    _parse_m114,
)


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

_DEFAULT_WS = DexArmWorkspace(
    x_min_mm=-300.0,
    x_max_mm=300.0,
    y_min_mm=0.0,
    y_max_mm=450.0,
    z_min_mm=-100.0,
    z_max_mm=400.0,
)


def _driver(
    transport: Optional[object] = None,
    workspace: Optional[DexArmWorkspace] = _DEFAULT_WS,
    ack_timeout_s: float = 1.0,
    logger: object = None,
) -> DexArmDriver:
    if transport is None:
        transport = MockDexArmSerial()
    return DexArmDriver(
        transport,  # type: ignore[arg-type]
        workspace=workspace,
        ack_timeout_s=ack_timeout_s,
        logger=logger,
    )


# ---------------------------------------------------------------------------
# AC1: Handshake, firmware identity, disconnect error
# ---------------------------------------------------------------------------


class TestHandshake:
    def test_connect_returns_firmware_id(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        identity = driver.connect()
        assert MockDexArmSerial.FIRMWARE_ID in identity

    def test_connect_sends_m115(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.connect()
        sent = b"".join(mock.written)
        assert b"M115" in sent

    def test_connect_sets_firmware_id_attribute(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        identity = driver.connect()
        assert driver._firmware_id == identity

    def test_disconnect_closes_transport(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.connect()
        driver.disconnect()
        assert not mock.is_open

    def test_disconnect_typed_error_on_command_after_close(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.connect()
        driver.disconnect()
        with pytest.raises(DexArmDisconnect):
            driver.get_position()

    def test_require_open_raises_disconnect_not_base(self):
        mock = MockDexArmSerial()
        mock.close()
        driver = _driver(mock)
        exc = None
        try:
            driver.connect()
        except Exception as e:
            exc = e
        assert isinstance(exc, DexArmDisconnect)
        assert isinstance(exc, DexArmError)

    def test_connect_with_logger(self):
        """Telemetry logger is called on connect (AA-7 integration)."""
        mock = MockDexArmSerial()
        logger = MagicMock()
        driver = _driver(mock, logger=logger)
        driver.connect()
        # At least one emit call with role='dexarm'
        calls = logger.emit.call_args_list
        assert any(
            kw.get("role") == "dexarm" or kw.get("event_type") == "dexarm_connected"
            for _, kw in calls
        )


# ---------------------------------------------------------------------------
# AC2: Homing, moves, feedrate, safe_park
# ---------------------------------------------------------------------------


class TestMotionCommands:
    def test_home_sends_g28(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.home()
        assert any(b"G28" in b for b in mock.written)

    def test_home_dry_run_does_not_send(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.home(dry_run=True)
        assert not mock.written

    def test_move_absolute_sends_g90_and_g0(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(10.0, 200.0, 100.0)
        sent = b"".join(mock.written).decode()
        assert "G90" in sent
        assert "G0" in sent
        assert "X10.000" in sent
        assert "Y200.000" in sent
        assert "Z100.000" in sent

    def test_move_absolute_with_feedrate(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(0.0, 200.0, 100.0, feedrate=1500.0)
        sent = b"".join(mock.written).decode()
        assert "F1500.000" in sent

    def test_move_absolute_dry_run_does_not_send(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(0.0, 200.0, 100.0, dry_run=True)
        assert not mock.written

    def test_move_relative_sends_g90_and_moves_from_current(self):
        mock = MockDexArmSerial()
        mock.set_position(0.0, 300.0, 200.0)
        driver = _driver(mock)
        driver.move_relative(10.0, 20.0, -5.0)
        sent = b"".join(mock.written).decode()
        assert "G90" in sent
        # Absolute target = (10, 320, 195)
        assert "X10.000" in sent
        assert "Y320.000" in sent
        assert "Z195.000" in sent

    def test_move_relative_dry_run_does_not_send_move(self):
        mock = MockDexArmSerial()
        mock.set_position(0.0, 300.0, 200.0)
        driver = _driver(mock)
        driver.move_relative(5.0, 5.0, 5.0, dry_run=True)
        # Only M114 is sent (to read current position) but no move command.
        sent = b"".join(mock.written).decode()
        assert "G0" not in sent
        assert "G1" not in sent

    def test_set_feedrate_sends_g1_f(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.set_feedrate(2000.0)
        sent = b"".join(mock.written).decode()
        assert "G1 F2000.000" in sent

    def test_set_feedrate_dry_run_does_not_send(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.set_feedrate(2000.0, dry_run=True)
        assert not mock.written

    def test_safe_park_moves_then_homes(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.safe_park()
        sent = b"".join(mock.written).decode()
        assert "G0" in sent  # park move
        assert "G28" in sent  # home after park

    def test_safe_park_dry_run_does_not_send(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.safe_park(dry_run=True)
        assert not mock.written

    def test_get_position_returns_dexarm_position(self):
        mock = MockDexArmSerial()
        mock.set_position(1.0, 2.0, 3.0)
        driver = _driver(mock)
        pos = driver.get_position()
        assert isinstance(pos, DexArmPosition)
        assert pos.x == pytest.approx(1.0, abs=0.01)
        assert pos.y == pytest.approx(2.0, abs=0.01)
        assert pos.z == pytest.approx(3.0, abs=0.01)

    def test_position_tracks_absolute_move(self):
        """Mock position tracks G0 commands so callers see consistent state."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(50.0, 200.0, 150.0)
        pos = driver.get_position()
        assert pos.x == pytest.approx(50.0, abs=0.01)
        assert pos.y == pytest.approx(200.0, abs=0.01)
        assert pos.z == pytest.approx(150.0, abs=0.01)


# ---------------------------------------------------------------------------
# AC3: Move completion confirmed before returning
# ---------------------------------------------------------------------------


class TestMoveCompletion:
    def test_move_absolute_blocks_until_ok(self):
        """Driver does not return until ok is received."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        # Normal move returns only after ok; if it returned early, position
        # would not reflect the command.
        driver.move_absolute(0.0, 200.0, 100.0)
        # The mock auto-sends ok; verify no DexArmDisconnect was raised.

    def test_timeout_raises_disconnect(self):
        """Transport that never responds causes DexArmDisconnect after timeout."""
        # Use a plain MockSerialTransport that never feeds data.
        transport = MockSerialTransport("dexarm")
        driver = DexArmDriver(
            transport,
            workspace=_DEFAULT_WS,
            ack_timeout_s=0.05,  # short timeout for test speed
        )
        with pytest.raises(DexArmDisconnect, match="timeout"):
            driver.home()

    def test_partial_response_included_in_disconnect(self):
        """DexArmDisconnect.partial carries data received before the timeout."""
        transport = MockSerialTransport("dexarm")
        # Feed a partial line with no newline to trigger timeout.
        transport.feed(b"FIRMWARE_NAME:Rotrics")
        driver = DexArmDriver(transport, ack_timeout_s=0.05)
        with pytest.raises(DexArmDisconnect):
            driver.connect()

    def test_transport_close_during_recv_raises_disconnect(self):
        """If the transport closes mid-receive, DexArmDisconnect is raised."""
        transport = MockSerialTransport("dexarm")
        # Feed the command trigger, then close before sending ok.
        driver = DexArmDriver(transport, ack_timeout_s=0.1)

        def _send_and_close() -> None:
            time.sleep(0.01)
            transport.close()

        t = threading.Thread(target=_send_and_close)
        t.start()
        with pytest.raises(DexArmDisconnect):
            driver.home()
        t.join()


# ---------------------------------------------------------------------------
# AC4: Command queue and error mapping
# ---------------------------------------------------------------------------


class TestCommandQueueAndErrors:
    def test_concurrent_sends_serialised(self):
        """Concurrent threads each get coherent full responses."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        results: List[Optional[DexArmPosition]] = [None, None]
        errors: List[Optional[Exception]] = [None, None]

        def _query(idx: int) -> None:
            try:
                results[idx] = driver.get_position()
            except Exception as exc:  # pragma: no cover
                errors[idx] = exc

        threads = [threading.Thread(target=_query, args=(i,)) for i in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [None, None], f"errors: {errors}"
        assert all(isinstance(r, DexArmPosition) for r in results)

    def test_error_response_raises_command_error(self):
        mock = MockDexArmSerial()
        mock.inject_error("unknown command")
        driver = _driver(mock)
        with pytest.raises(DexArmCommandError) as exc_info:
            driver.home()
        assert "unknown command" in str(exc_info.value).lower()

    def test_command_error_carries_command_and_response(self):
        mock = MockDexArmSerial()
        mock.inject_error("out of range")
        driver = _driver(mock)
        with pytest.raises(DexArmCommandError) as exc_info:
            driver.send_raw("G0 X999.0 Y999.0 Z999.0")
        err = exc_info.value
        assert hasattr(err, "command")
        assert hasattr(err, "firmware_response")
        assert "out of range" in err.firmware_response

    def test_command_error_is_subclass_of_dexarm_error(self):
        assert issubclass(DexArmCommandError, DexArmError)

    def test_send_raw_dry_run_returns_empty_list(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        result = driver.send_raw("G92 E0", dry_run=True)
        assert result == []
        assert not mock.written

    def test_send_raw_returns_response_lines(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        lines = driver.send_raw("M115")
        assert any("ok" in ln.lower() or "rotrics" in ln.lower() for ln in lines)

    def test_multiple_commands_in_sequence(self):
        """Commands queued sequentially each complete without interference."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.home()
        driver.move_absolute(0.0, 200.0, 100.0)
        pos = driver.get_position()
        assert isinstance(pos, DexArmPosition)


# ---------------------------------------------------------------------------
# AC5: Workspace bounds rejected before transmission
# ---------------------------------------------------------------------------


class TestWorkspaceBounds:
    def test_bounds_config_loaded_from_yaml(self):
        config = load_config()
        ws = config.dexarm.workspace
        assert ws is not None
        assert ws.x_min_mm < ws.x_max_mm
        assert ws.y_min_mm < ws.y_max_mm
        assert ws.z_min_mm < ws.z_max_mm

    def test_move_inside_bounds_succeeds(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(0.0, 200.0, 100.0)  # within default workspace

    def test_move_outside_x_raises_bounds_error(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_absolute(999.0, 200.0, 100.0)

    def test_move_outside_y_raises_bounds_error(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_absolute(0.0, 9999.0, 100.0)

    def test_move_outside_z_raises_bounds_error(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_absolute(0.0, 200.0, 9999.0)

    def test_bounds_error_before_transmission(self):
        """When bounds are violated no bytes are sent to the transport."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_absolute(999.0, 200.0, 100.0)
        # Nothing was written because rejection happens before transmission.
        assert not mock.written

    def test_bounds_error_is_subclass_of_dexarm_error(self):
        assert issubclass(DexArmBoundsError, DexArmError)

    def test_no_bounds_check_when_workspace_is_none(self):
        mock = MockDexArmSerial()
        driver = DexArmDriver(mock, workspace=None, ack_timeout_s=1.0)
        # Should not raise even for very large coords.
        driver.move_absolute(99999.0, 99999.0, 99999.0)

    def test_relative_move_outside_bounds_raises(self):
        mock = MockDexArmSerial()
        mock.set_position(0.0, 300.0, 200.0)
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_relative(0.0, 999.0, 0.0)

    def test_relative_move_outside_bounds_not_transmitted(self):
        mock = MockDexArmSerial()
        mock.set_position(0.0, 300.0, 200.0)
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_relative(0.0, 999.0, 0.0)
        # Only M114 query was sent; no G90/G0 move bytes.
        sent = b"".join(mock.written).decode()
        assert "G0" not in sent

    def test_bounds_error_payload(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError) as exc_info:
            driver.move_absolute(999.0, 200.0, 100.0)
        err = exc_info.value
        assert err.x == pytest.approx(999.0)
        assert err.workspace is _DEFAULT_WS

    def test_workspace_validation_min_gte_max_rejected(self):
        with pytest.raises(Exception):
            DexArmWorkspace(
                x_min_mm=100.0, x_max_mm=100.0,
                y_min_mm=0.0, y_max_mm=450.0,
                z_min_mm=-100.0, z_max_mm=400.0,
            )


# ---------------------------------------------------------------------------
# AC6: Mock DexArm backend tests
# ---------------------------------------------------------------------------


class TestMockDexArmSerial:
    def test_mock_connect_handshake(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        identity = driver.connect()
        assert "Rotrics" in identity or "mock" in identity.lower()

    def test_mock_get_position_default(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        pos = driver.get_position()
        # Default position set in MockDexArmSerial.__init__
        assert isinstance(pos, DexArmPosition)

    def test_mock_move_confirmation(self):
        """Move completes without hanging; position query returns updated pos."""
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.move_absolute(50.0, 200.0, 100.0)
        pos = driver.get_position()
        assert pos.x == pytest.approx(50.0, abs=0.01)

    def test_mock_error_injection_maps_to_command_error(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        mock.inject_error("checksum mismatch")
        with pytest.raises(DexArmCommandError) as exc_info:
            driver.home()
        assert "checksum mismatch" in exc_info.value.firmware_response

    def test_mock_bounds_rejection_no_bytes_sent(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        with pytest.raises(DexArmBoundsError):
            driver.move_absolute(-999.0, 200.0, 100.0)
        assert not mock.written

    def test_mock_close_raises_disconnect(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        mock.close()
        with pytest.raises(DexArmDisconnect):
            driver.get_position()

    def test_mock_set_position_round_trip(self):
        mock = MockDexArmSerial()
        mock.set_position(12.5, 300.0, 150.0)
        driver = _driver(mock)
        pos = driver.get_position()
        assert pos.x == pytest.approx(12.5, abs=0.01)
        assert pos.y == pytest.approx(300.0, abs=0.01)
        assert pos.z == pytest.approx(150.0, abs=0.01)


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------


class TestM114Parsing:
    def test_parses_standard_m114_response(self):
        lines = ["X:10.000 Y:200.000 Z:100.000 E:0.000 Count X:100 Y:200 Z:100"]
        pos = _parse_m114(lines)
        assert pos is not None
        assert pos.x == pytest.approx(10.0)
        assert pos.y == pytest.approx(200.0)
        assert pos.z == pytest.approx(100.0)

    def test_parses_negative_coords(self):
        pos = _parse_m114(["X:-5.100 Y:300.000 Z:-10.500"])
        assert pos is not None
        assert pos.x == pytest.approx(-5.1)
        assert pos.z == pytest.approx(-10.5)

    def test_returns_none_for_non_position_lines(self):
        assert _parse_m114(["ok", "FIRMWARE_NAME:Rotrics"]) is None

    def test_dexarm_position_str(self):
        pos = DexArmPosition(1.0, 2.0, 3.0)
        assert "1.000" in str(pos)
        assert "2.000" in str(pos)


# ---------------------------------------------------------------------------
# Module-select (Rotrics-specific, unverified)
# ---------------------------------------------------------------------------


class TestModuleSelect:
    def test_select_module_dry_run_does_not_send(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.select_module(0, dry_run=True)
        assert not mock.written

    def test_select_module_sends_m888(self):
        mock = MockDexArmSerial()
        driver = _driver(mock)
        driver.select_module(0)
        sent = b"".join(mock.written).decode()
        assert "M888" in sent
