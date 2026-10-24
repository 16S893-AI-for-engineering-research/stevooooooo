"""Tests for inchworm.servo_bus: Feetech STS3215 protocol driver.

Covers:
  AC1  - ping, read position, read current/load, read temperature/voltage,
         write goal position, torque enable/disable
  AC3  - checksum errors, timeouts, and out-of-range responses as distinct
         typed exceptions; no silent stale or partial reads
  AC5  - position values in degrees at the interface; tick conversion isolated
  AC6  - mock servo bus backend; error-mapping paths

Gripper restriction:
  - write_position_deg("gripper", ...) raises GripperWriteNotAllowed
  - gripper_channel(_GRIPPER_TOKEN).write_position_deg() succeeds
  - gripper_channel(wrong_token) raises GripperWriteNotAllowed
  - token not imported outside gripper_access.py and AA-17 module path

Sign-magnitude decoding (P1 bug fix):
  - present_speed and present_load are sign-magnitude, NOT two's complement
"""

from __future__ import annotations

import struct
import time
from typing import Dict, List

import pytest

from inchworm.backends import MockSerialTransport
from inchworm.servo_bus import (
    JOINT_NAMES,
    ArmServoBus,
    GripperChannel,
    GripperWriteNotAllowed,
    JointState,
    ServoChecksumError,
    ServoError,
    ServoRangeError,
    ServoStatusError,
    ServoTimeoutError,
    _build_request,
    _ServoBus,
    _checksum,
    _decode_load_sign_magnitude,
    _decode_speed_sign_magnitude,
    _INST_PING,
    _INST_READ,
    _INST_WRITE,
    _REG_GOAL_POSITION,
    _REG_PRESENT_POSITION,
    _REG_TORQUE_ENABLE,
    degrees_to_ticks,
    ticks_to_degrees,
)


# ---------------------------------------------------------------------------
# Helpers to build synthetic SCS status packets
# ---------------------------------------------------------------------------

def _status_packet(servo_id: int, error: int, payload: bytes) -> bytes:
    """Build a valid SCS status packet."""
    length = len(payload) + 2  # error + payload + checksum (LEN field)
    # Checksum: (~(ID + LEN + error + sum(payload))) & 0xFF
    chk_data = bytes([error]) + payload
    total = servo_id + length + sum(chk_data)
    chk = (~total) & 0xFF
    return b"\xff\xff" + bytes([servo_id, length, error]) + payload + bytes([chk])


def _ping_response(servo_id: int) -> bytes:
    """A valid ping response (no payload, no error)."""
    return _status_packet(servo_id, 0, b"")


def _read_response(servo_id: int, payload: bytes) -> bytes:
    """A valid read response."""
    return _status_packet(servo_id, 0, payload)


def _write_response(servo_id: int) -> bytes:
    """A valid write-ack response."""
    return _status_packet(servo_id, 0, b"")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

_JOINT_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


@pytest.fixture()
def transport() -> MockSerialTransport:
    return MockSerialTransport("arm_left")


@pytest.fixture()
def bus(transport: MockSerialTransport) -> ArmServoBus:
    return ArmServoBus(
        transport,
        role="arm_left",
        joint_id_map=_JOINT_MAP,
        timeout_s=0.05,
    )


# ---------------------------------------------------------------------------
# Tick <-> degrees conversion  (AC5)
# ---------------------------------------------------------------------------


class TestTickConversion:
    def test_zero_ticks_is_zero_degrees(self):
        assert ticks_to_degrees(0) == pytest.approx(0.0)

    def test_4096_ticks_is_360_degrees(self):
        assert ticks_to_degrees(4096) == pytest.approx(360.0)

    def test_2048_ticks_is_180_degrees(self):
        assert ticks_to_degrees(2048) == pytest.approx(180.0)

    def test_degrees_to_ticks_roundtrip(self):
        for deg in (0.0, 45.0, 90.0, 135.0, 180.0, 270.0, 360.0):
            ticks = degrees_to_ticks(deg)
            back = ticks_to_degrees(ticks)
            assert abs(back - deg) < 0.1, f"roundtrip failed at {deg} deg"

    def test_degrees_out_of_range_raises(self):
        with pytest.raises(ValueError, match="\\[0, 360\\]"):
            degrees_to_ticks(-1.0)
        with pytest.raises(ValueError, match="\\[0, 360\\]"):
            degrees_to_ticks(361.0)

    def test_conversion_is_isolated(self):
        """Only ticks_to_degrees and degrees_to_ticks do the math (AC5)."""
        import inchworm.servo_bus as sb
        # Confirm functions exist and return expected values
        assert sb.ticks_to_degrees(1024) == pytest.approx(90.0)
        assert sb.degrees_to_ticks(90.0) == 1024


# ---------------------------------------------------------------------------
# Sign-magnitude decoding (P1 bug fix)
# ---------------------------------------------------------------------------


class TestSignMagnitudeDecode:
    """Verify sign-magnitude decoding with known raw values from the task spec."""

    # present_load: bit 10 = sign, bits 0-9 = magnitude

    def test_load_positive(self):
        """Raw 100 (no sign bit set) -> +100."""
        assert _decode_load_sign_magnitude(100) == 100

    def test_load_negative(self):
        """Raw 0x0400 | 100 -> -100 (bit 10 set = negative)."""
        assert _decode_load_sign_magnitude(0x0400 | 100) == -100

    def test_load_zero(self):
        assert _decode_load_sign_magnitude(0) == 0

    def test_load_max_positive(self):
        """Magnitude 0x03FF = 1023, no sign bit -> +1023."""
        assert _decode_load_sign_magnitude(0x03FF) == 1023

    def test_load_max_negative(self):
        """0x0400 | 0x03FF = -1023."""
        assert _decode_load_sign_magnitude(0x07FF) == -1023

    # present_speed: bit 15 = sign, bits 0-14 = magnitude

    def test_speed_positive(self):
        """Raw 50 -> +50."""
        assert _decode_speed_sign_magnitude(50) == 50

    def test_speed_negative(self):
        """Raw 0x8000 | 50 -> -50 (bit 15 set = negative)."""
        assert _decode_speed_sign_magnitude(0x8000 | 50) == -50

    def test_speed_zero(self):
        assert _decode_speed_sign_magnitude(0) == 0

    def test_speed_max_positive(self):
        """Magnitude 0x7FFF = 32767, no sign bit -> +32767."""
        assert _decode_speed_sign_magnitude(0x7FFF) == 32767

    def test_speed_max_negative(self):
        """0x8000 | 0x7FFF = -32767."""
        assert _decode_speed_sign_magnitude(0xFFFF) == -32767

    def test_two_complement_would_differ(self):
        """Confirm that two's-complement interpretation differs from sign-magnitude.

        If the OLD (wrong) struct '<h' decoding were used:
          - 0x0400 | 100 = 0x0464 -> struct '<h' would give a positive number (1124)
          - 0x8000 | 50  = 0x8032 -> struct '<h' would give a negative number (-32718)

        The sign-magnitude values must differ from those wrong answers.
        """
        import struct
        wrong_load = struct.unpack("<h", struct.pack("<H", 0x0400 | 100))[0]
        assert wrong_load != -100, "two's complement gives wrong answer for load"
        correct_load = _decode_load_sign_magnitude(0x0400 | 100)
        assert correct_load == -100

        wrong_speed = struct.unpack("<h", struct.pack("<H", 0x8000 | 50))[0]
        assert wrong_speed != -50, "two's complement gives wrong answer for speed"
        correct_speed = _decode_speed_sign_magnitude(0x8000 | 50)
        assert correct_speed == -50


# ---------------------------------------------------------------------------
# Ping  (AC1)
# ---------------------------------------------------------------------------


class TestPing:
    def test_ping_returns_true_on_response(self, transport, bus):
        transport.feed(_ping_response(1))
        assert bus.ping(servo_id=1) is True

    def test_ping_returns_false_on_timeout(self, bus):
        # No data fed -> timeout
        result = bus.ping(servo_id=99)
        assert result is False

    def test_ping_sends_correct_packet(self, transport, bus):
        transport.feed(_ping_response(1))
        bus.ping(servo_id=1)
        assert len(transport.written) == 1
        packet = transport.written[0]
        assert packet[:2] == b"\xff\xff"
        assert packet[2] == 1     # servo ID
        assert packet[4] == _INST_PING


# ---------------------------------------------------------------------------
# Read position  (AC1, AC5)
# ---------------------------------------------------------------------------


class TestReadPosition:
    def _make_read_response(self, servo_id: int, ticks: int) -> bytes:
        payload = struct.pack("<H", ticks)
        return _read_response(servo_id, payload)

    def test_read_position_returns_degrees(self, transport, bus):
        transport.feed(self._make_read_response(1, 2048))
        deg = bus.read_position_deg(servo_id=1)
        assert deg == pytest.approx(180.0)

    def test_read_position_zero(self, transport, bus):
        transport.feed(self._make_read_response(1, 0))
        assert bus.read_position_deg(servo_id=1) == pytest.approx(0.0)

    def test_read_position_max(self, transport, bus):
        transport.feed(self._make_read_response(1, 4095))
        assert bus.read_position_deg(servo_id=1) == pytest.approx(359.912, abs=0.01)

    def test_read_position_out_of_range_raises(self, transport, bus):
        # 4096 is out of range for present position (0-4095)
        transport.feed(self._make_read_response(1, 4096))
        with pytest.raises(ServoRangeError) as exc_info:
            bus.read_position_deg(servo_id=1)
        assert exc_info.value.servo_id == 1

    def test_read_position_timeout_raises(self, bus):
        with pytest.raises(ServoTimeoutError) as exc_info:
            bus.read_position_deg(servo_id=2)
        assert exc_info.value.servo_id == 2
        assert "timeout" in str(exc_info.value).lower()


# ---------------------------------------------------------------------------
# Read joint state (position + current/load + temperature/voltage)  (AC1)
# ---------------------------------------------------------------------------


def _joint_state_responses(
    servo_id: int,
    pos_ticks: int = 2048,
    speed_raw16: int = 100,   # raw 16-bit value before sign-mag decode
    load_raw16: int = 50,     # raw 16-bit value before sign-mag decode
    voltage_raw: int = 120,   # 12.0 V
    temperature: int = 35,
    current_raw: int = 75,
) -> bytes:
    """Return a single read response for the 15-byte block read_joint_state uses.

    read_joint_state reads 15 bytes starting at 0x38 (covers position,
    speed, load, voltage, temperature, 5 reserved bytes, and current at 0x45).
    """
    # Build the 15-byte payload:
    #   offset 0-1  : present_position
    #   offset 2-3  : present_speed (raw u16)
    #   offset 4-5  : present_load  (raw u16)
    #   offset 6    : present_voltage
    #   offset 7    : present_temperature
    #   offset 8-12 : reserved (5 bytes, zeroed)
    #   offset 13-14: present_current (u16)
    block = (
        struct.pack("<H", pos_ticks)
        + struct.pack("<H", speed_raw16)
        + struct.pack("<H", load_raw16)
        + bytes([voltage_raw, temperature])
        + bytes(5)  # reserved registers 0x40-0x44
        + struct.pack("<H", current_raw)
    )
    assert len(block) == 15
    return _read_response(servo_id, block)


class TestReadJointState:
    def test_read_joint_state_returns_joint_state(self, transport, bus):
        transport.feed(_joint_state_responses(1))
        state = bus.read_joint_state(servo_id=1)
        assert isinstance(state, JointState)
        assert state.servo_id == 1

    def test_position_in_degrees(self, transport, bus):
        transport.feed(_joint_state_responses(1, pos_ticks=1024))
        state = bus.read_joint_state(servo_id=1)
        assert state.position_deg == pytest.approx(90.0)

    def test_voltage_converted_to_volts(self, transport, bus):
        transport.feed(_joint_state_responses(1, voltage_raw=120))
        state = bus.read_joint_state(servo_id=1)
        assert state.voltage_v == pytest.approx(12.0)

    def test_temperature_degrees_c(self, transport, bus):
        transport.feed(_joint_state_responses(1, temperature=42))
        state = bus.read_joint_state(servo_id=1)
        assert state.temperature_c == 42

    def test_speed_sign_magnitude_positive(self, transport, bus):
        """speed_raw16=100 -> speed_raw=+100 (no sign bit)."""
        transport.feed(_joint_state_responses(1, speed_raw16=100))
        state = bus.read_joint_state(servo_id=1)
        assert state.speed_raw == 100

    def test_speed_sign_magnitude_negative(self, transport, bus):
        """speed_raw16=0x8000|50 -> speed_raw=-50 (bit 15 set)."""
        transport.feed(_joint_state_responses(1, speed_raw16=0x8000 | 50))
        state = bus.read_joint_state(servo_id=1)
        assert state.speed_raw == -50

    def test_load_sign_magnitude_positive(self, transport, bus):
        """load_raw16=100 -> load_raw=+100 (no sign bit)."""
        transport.feed(_joint_state_responses(1, load_raw16=100))
        state = bus.read_joint_state(servo_id=1)
        assert state.load_raw == 100

    def test_load_sign_magnitude_negative(self, transport, bus):
        """load_raw16=0x0400|100 -> load_raw=-100 (bit 10 set)."""
        transport.feed(_joint_state_responses(1, load_raw16=0x0400 | 100))
        state = bus.read_joint_state(servo_id=1)
        assert state.load_raw == -100

    def test_speed_and_load_raw(self, transport, bus):
        """Basic non-negative values pass through as positive integers."""
        transport.feed(_joint_state_responses(1, speed_raw16=512, load_raw16=200))
        state = bus.read_joint_state(servo_id=1)
        assert state.speed_raw == 512
        assert state.load_raw == 200

    def test_current_raw_returned(self, transport, bus):
        """present_current (register 0x45) is read and stored in JointState."""
        transport.feed(_joint_state_responses(1, current_raw=300))
        state = bus.read_joint_state(servo_id=1)
        assert state.current_raw == 300

    def test_current_raw_zero(self, transport, bus):
        transport.feed(_joint_state_responses(1, current_raw=0))
        state = bus.read_joint_state(servo_id=1)
        assert state.current_raw == 0

    def test_out_of_range_position_raises_range_error(self, transport, bus):
        # Craft a response with ticks=4096 (out of range)
        payload = (
            struct.pack("<H", 4096)
            + struct.pack("<H", 0)
            + struct.pack("<H", 0)
            + bytes([100, 30])
        )
        transport.feed(_read_response(1, payload))
        with pytest.raises(ServoRangeError):
            bus.read_joint_state(servo_id=1)

    def test_timeout_raises_timeout_error(self, bus):
        with pytest.raises(ServoTimeoutError):
            bus.read_joint_state(servo_id=1)


# ---------------------------------------------------------------------------
# Torque enable / disable  (AC1)
# ---------------------------------------------------------------------------


class TestTorqueEnable:
    def test_enable_sends_correct_packet(self, transport, bus):
        transport.feed(_write_response(1))
        bus.set_torque_enable(servo_id=1, enable=True)
        assert len(transport.written) == 1
        packet = transport.written[0]
        assert packet[4] == _INST_WRITE  # instruction byte

    def test_disable_sends_correct_packet(self, transport, bus):
        transport.feed(_write_response(1))
        bus.set_torque_enable(servo_id=1, enable=False)
        assert len(transport.written) == 1

    def test_dry_run_does_not_send_packet(self, transport):
        dry_bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
            dry_run=True,
        )
        dry_bus.set_torque_enable(servo_id=1, enable=True)
        assert transport.written == []

    def test_torque_enable_timeout_raises(self, bus):
        with pytest.raises(ServoTimeoutError):
            bus.set_torque_enable(servo_id=1, enable=True)


# ---------------------------------------------------------------------------
# Write position  (AC1, AC5)
# ---------------------------------------------------------------------------


class TestWritePosition:
    def test_write_position_sends_packet(self, transport, bus):
        transport.feed(_write_response(1))
        bus.write_position_deg("shoulder_pan", 90.0)
        assert len(transport.written) == 1
        packet = transport.written[0]
        assert packet[4] == _INST_WRITE

    def test_write_position_encodes_degrees_as_ticks(self, transport, bus):
        transport.feed(_write_response(1))
        bus.write_position_deg("shoulder_pan", 90.0)
        packet = transport.written[0]
        # Params start at byte 5: [register, tick_low, tick_high]
        tick_bytes = packet[6:8]
        ticks = struct.unpack("<H", tick_bytes)[0]
        assert ticks == degrees_to_ticks(90.0)

    def test_write_position_dry_run_skips_packet(self, transport):
        dry_bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
            dry_run=True,
        )
        dry_bus.write_position_deg("shoulder_pan", 45.0)
        assert transport.written == []

    def test_write_position_unknown_joint_raises(self, bus):
        with pytest.raises(ServoError, match="unknown joint"):
            bus.write_position_deg("nonexistent_joint", 90.0)

    def test_write_position_timeout_raises(self, bus):
        with pytest.raises(ServoTimeoutError):
            bus.write_position_deg("shoulder_pan", 45.0)


# ---------------------------------------------------------------------------
# Gripper restriction and capability object  (AC6, safety invariant)
# ---------------------------------------------------------------------------


class TestGripperCapability:
    def test_write_position_deg_gripper_raises_not_allowed(self, bus):
        """General callers must not be able to move the gripper (AC6 / doc-1 s3)."""
        with pytest.raises(GripperWriteNotAllowed) as exc_info:
            bus.write_position_deg("gripper", 90.0)
        assert "AA-17" in str(exc_info.value)

    def test_write_position_deg_gripper_is_a_servo_error(self, bus):
        """GripperWriteNotAllowed is a subtype of ServoError (catchable as such)."""
        with pytest.raises(ServoError):
            bus.write_position_deg("gripper", 90.0)

    def test_gripper_channel_wrong_token_raises(self, bus):
        """Passing any object other than _GRIPPER_TOKEN raises GripperWriteNotAllowed."""
        with pytest.raises(GripperWriteNotAllowed):
            bus.gripper_channel("not_the_token")

    def test_gripper_channel_none_token_raises(self, bus):
        with pytest.raises(GripperWriteNotAllowed):
            bus.gripper_channel(None)

    def test_gripper_channel_correct_token_returns_channel(self, bus):
        """Passing _GRIPPER_TOKEN returns a GripperChannel instance."""
        from inchworm.gripper_access import _GRIPPER_TOKEN
        ch = bus.gripper_channel(_GRIPPER_TOKEN)
        assert isinstance(ch, GripperChannel)

    def test_gripper_channel_write_position_sends_packet(self, transport, bus):
        """GripperChannel.write_position_deg sends the goal-position packet."""
        from inchworm.gripper_access import _GRIPPER_TOKEN
        transport.feed(_write_response(6))
        ch = bus.gripper_channel(_GRIPPER_TOKEN)
        ch.write_position_deg(90.0)
        assert len(transport.written) == 1

    def test_gripper_channel_dry_run_skips_packet(self, transport):
        """GripperChannel.write_position_deg is a no-op in dry-run mode."""
        from inchworm.gripper_access import _GRIPPER_TOKEN
        dry_bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
            dry_run=True,
        )
        ch = dry_bus.gripper_channel(_GRIPPER_TOKEN)
        ch.write_position_deg(90.0)
        assert transport.written == []

    def test_gripper_channel_out_of_range_raises(self, bus):
        """GripperChannel.write_position_deg raises ServoRangeError on bad deg."""
        from inchworm.gripper_access import _GRIPPER_TOKEN
        ch = bus.gripper_channel(_GRIPPER_TOKEN)
        with pytest.raises(ServoRangeError):
            ch.write_position_deg(400.0)

    def test_no_token_import_outside_aa17(self):
        """Scan src/inchworm for any import or use of _GRIPPER_TOKEN outside the
        allowed files:
          - gripper_access.py (defines the token)
          - servo_bus.py (verifies the token inside gripper_channel via local import)
          - gripper_lock*.py (AA-17 state machine, the sole authorised caller)

        This test FAILS if any *other* module imports or uses the sentinel,
        indicating a caller is bypassing the capability gate.
        """
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
            if "_GRIPPER_TOKEN" in source:
                violations.append(str(py_file.relative_to(src_root.parent.parent)))

        assert violations == [], (
            f"_GRIPPER_TOKEN referenced outside allowed modules: {violations!r}. "
            "Only gripper_access.py, servo_bus.py, and gripper_lock*.py may use it."
        )


# ---------------------------------------------------------------------------
# Typed errors  (AC3)
# ---------------------------------------------------------------------------


class TestTypedErrors:
    def test_checksum_error_is_distinct(self, transport):
        """Corrupted checksum raises ServoChecksumError on the raw _ServoBus layer."""
        valid = _read_response(1, struct.pack("<H", 2048))
        corrupt = valid[:-1] + bytes([(valid[-1] ^ 0xFF)])
        transport.feed(corrupt)
        sbus = _ServoBus(transport, timeout_s=0.05)
        with pytest.raises(ServoChecksumError) as exc_info:
            sbus.read_bytes(1, _REG_PRESENT_POSITION, 2)
        assert exc_info.value.servo_id == 1
        assert exc_info.value.expected != exc_info.value.actual

    def test_timeout_error_is_distinct(self, bus):
        """No data -> ServoTimeoutError (not a generic IOError subclass)."""
        with pytest.raises(ServoTimeoutError) as exc_info:
            bus.read_position_deg(servo_id=5)
        assert exc_info.value.servo_id == 5

    def test_range_error_is_distinct(self, transport, bus):
        """Out-of-range tick value raises ServoRangeError specifically."""
        transport.feed(_read_response(1, struct.pack("<H", 4096)))
        with pytest.raises(ServoRangeError) as exc_info:
            bus.read_position_deg(servo_id=1)
        assert exc_info.value.servo_id == 1

    def test_status_error_is_distinct(self, transport):
        """Non-zero error byte in status packet raises ServoStatusError."""
        bad_status = _status_packet(1, error=0x04, payload=b"")
        transport.feed(bad_status)
        sbus = _ServoBus(transport, timeout_s=0.05)
        with pytest.raises(ServoStatusError) as exc_info:
            sbus.read_bytes(1, _REG_PRESENT_POSITION, 2)
        assert exc_info.value.error_byte == 0x04

    def test_all_errors_are_servo_error_subclasses(self):
        """All typed errors share the ServoError base (catchable uniformly)."""
        assert issubclass(ServoTimeoutError, ServoError)
        assert issubclass(ServoChecksumError, ServoError)
        assert issubclass(ServoRangeError, ServoError)
        assert issubclass(ServoStatusError, ServoError)
        assert issubclass(GripperWriteNotAllowed, ServoError)

    def test_no_silent_partial_read(self, transport):
        """A truncated payload raises ServoError, never returns a partial value."""
        # Feed only 1 byte of a 2-byte payload
        transport.feed(_read_response(1, b"\x00"))  # only 1 byte payload
        sbus = _ServoBus(transport, timeout_s=0.05)
        with pytest.raises(ServoRangeError, match="expected 2 bytes"):
            sbus.read_bytes(1, _REG_PRESENT_POSITION, 2)

    def test_degrees_out_of_range_is_range_error(self, transport, bus):
        """degrees_to_ticks ValueError is wrapped as ServoRangeError."""
        transport.feed(_write_response(1))
        with pytest.raises(ServoRangeError):
            # ArmServoBus._write_goal_position wraps the ValueError
            bus._write_goal_position(1, "shoulder_pan", 500.0)


# ---------------------------------------------------------------------------
# ArmServoBus construction guards
# ---------------------------------------------------------------------------


class TestBusConstruction:
    def test_invalid_role_raises(self, transport):
        with pytest.raises(ValueError, match="role must be"):
            ArmServoBus(transport, role="trunk", joint_id_map=_JOINT_MAP)

    def test_missing_gripper_joint_raises(self, transport):
        no_gripper = {k: v for k, v in _JOINT_MAP.items() if k != "gripper"}
        with pytest.raises(ValueError, match="gripper"):
            ArmServoBus(transport, role="arm_left", joint_id_map=no_gripper)

    def test_dry_run_property(self, transport):
        dry = ArmServoBus(
            transport,
            role="arm_right",
            joint_id_map=_JOINT_MAP,
            dry_run=True,
        )
        assert dry.dry_run is True
        assert dry.role == "arm_right"


# ---------------------------------------------------------------------------
# Mock backend integration (AC6)
# ---------------------------------------------------------------------------


class TestMockBackendIntegration:
    """Verify the driver works against the inchworm MockBackend."""

    def test_mock_backend_open_and_ping(self):
        from inchworm.backends import MockBackend, MockSerialTransport
        from inchworm.config import load_config

        config = load_config()
        backend = MockBackend(config)
        transport = backend.open_serial("arm_left")

        bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
        )
        # No data fed -> ping returns False (timeout)
        assert bus.ping(1) is False

        # Feed a valid ping response
        transport.feed(_ping_response(1))
        assert bus.ping(1) is True

    def test_mock_backend_read_position(self):
        from inchworm.backends import MockBackend

        config = __import__("inchworm.config", fromlist=["load_config"]).load_config()
        backend = MockBackend(config)
        transport = backend.open_serial("arm_right")
        bus = ArmServoBus(
            transport,
            role="arm_right",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
        )
        transport.feed(_read_response(3, struct.pack("<H", 1024)))
        deg = bus.read_position_deg(servo_id=3)
        assert deg == pytest.approx(90.0)

    def test_mock_backend_write_then_read(self):
        from inchworm.backends import MockBackend
        from inchworm.config import load_config

        config = load_config()
        backend = MockBackend(config)
        transport = backend.open_serial("arm_left")
        bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
        )
        # Write position
        transport.feed(_write_response(1))
        bus.write_position_deg("shoulder_pan", 180.0)
        assert len(transport.written) == 1

        # Verify the packet encodes 180.0 deg as 2048 ticks
        packet = transport.written[0]
        ticks = struct.unpack("<H", packet[6:8])[0]
        assert ticks == degrees_to_ticks(180.0)

    def test_mock_backend_torque_enable(self):
        from inchworm.backends import MockBackend
        from inchworm.config import load_config

        config = load_config()
        backend = MockBackend(config)
        transport = backend.open_serial("arm_left")
        bus = ArmServoBus(
            transport,
            role="arm_left",
            joint_id_map=_JOINT_MAP,
            timeout_s=0.05,
        )
        transport.feed(_write_response(2))
        bus.set_torque_enable(servo_id=2, enable=True)
        assert len(transport.written) == 1


# ---------------------------------------------------------------------------
# Config schema: servo_joints field  (AC2 partial -- schema side)
# ---------------------------------------------------------------------------


class TestConfigServoJoints:
    def test_config_loads_with_null_servo_joints(self):
        """hardware.yaml loads cleanly with null servo_joint IDs."""
        from inchworm.config import load_config

        config = load_config()
        assert config.arms.left.servo_joints is not None
        assert config.arms.right.servo_joints is not None
        # All null initially
        assert config.arms.left.servo_joints.gripper is None
        assert config.arms.left.servo_joints.shoulder_pan is None

    def test_config_accepts_integer_servo_ids(self):
        """Config schema accepts non-null integer IDs once scanned."""
        import yaml
        from inchworm.config import parse_config, DEFAULT_CONFIG_PATH

        data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
        # Inject specific IDs
        data["arms"]["left"]["servo_joints"] = {
            "shoulder_pan": 1,
            "shoulder_lift": 2,
            "elbow_flex": 3,
            "wrist_flex": 4,
            "wrist_roll": 5,
            "gripper": 6,
        }
        config = parse_config(data)
        assert config.arms.left.servo_joints.shoulder_pan == 1
        assert config.arms.left.servo_joints.gripper == 6
