"""SO-101 Feetech STS3215 servo-bus driver (Feetech SCS serial protocol).

This is the lowest control layer for the SO-101 arms.  It speaks the
Feetech SCS wire protocol over any :class:`~inchworm.backends.SerialTransport`
and exposes a clean, degrees-at-the-boundary API.

Architecture
------------
- :class:`ArmServoBus` is the public entry point.  Construct one per arm
  (``role`` is ``"arm_left"`` or ``"arm_right"``).
- All position values cross the public API boundary in **degrees**.
- Raw tick <-> degrees conversion is isolated in :func:`ticks_to_degrees` and
  :func:`degrees_to_ticks`.
- Typed exceptions are raised for every fault; no silent partial read is
  ever returned.
- Gripper servo position writes are gated behind a capability object.
  :meth:`ArmServoBus.gripper_channel` returns a :class:`GripperChannel`
  **only** when passed the module-private sentinel imported from
  :mod:`inchworm.gripper_access`.  That module is importable only by the
  AA-17 state machine (``inchworm/gripper_lock*.py``).  General callers
  receive :class:`GripperWriteNotAllowed` if they attempt to reach joint 6
  via :meth:`write_position_deg`; sync-write paths reject the gripper ID
  as well.

Feetech SCS protocol summary
-----------------------------
Packet structure (request)::

    0xFF 0xFF  ID  LENGTH  INSTRUCTION  [PARAMETERS ...]  CHECKSUM

  LENGTH = number_of_parameters + 2
  CHECKSUM = (~(ID + LENGTH + INSTRUCTION + sum(PARAMETERS))) & 0xFF

Status packet (response)::

    0xFF 0xFF  ID  LENGTH  ERROR  [PARAMETERS ...]  CHECKSUM

Register map (STS3215, verified against rustypot src/servo/feetech/sts3215.rs)
-------------------------------------------------------------------------------
  0x00  firmware major, 0x01 minor; 0x03 model number L/H (STS3215 = 777)
  0x28  torque enable          (1 byte,  write)
  0x2A  goal position L/H      (2 bytes, write)
  0x37  lock                   (1 byte,  write)
  0x38  present position L/H   (2 bytes, read)
  0x3A  present speed L/H      (2 bytes, read, sign-mag: bit 15 = sign)
  0x3C  present load L/H       (2 bytes, read, sign-mag: bit 10 = sign)
  0x3E  present voltage        (1 byte,  read, unit: 0.1 V)
  0x3F  present temperature    (1 byte,  read, unit: deg C)
  0x45  present current        (2 bytes, read, u16)

Sign-magnitude decoding (present_speed and present_load)
---------------------------------------------------------
The STS3215 encodes these as sign-magnitude, **not** two's complement:
  - present_speed  (0x3A): sign = bit 15; magnitude = bits 0-14
  - present_load   (0x3C): sign = bit 10; magnitude = bits  0-9

Position encoding: 0-4095 ticks maps to 0-360 deg.

Units: degrees at public interface; ticks internally.
"""

from __future__ import annotations

import struct
import time
from typing import Dict, List, Optional, Sequence

from inchworm.backends import SerialTransport
from inchworm.units import Degrees

# ---------------------------------------------------------------------------
# Constants -- STS3215 hardware
# ---------------------------------------------------------------------------

#: Full 360 deg maps to 4096 ticks (0-4095).
_TICKS_PER_REVOLUTION: int = 4096

#: The six joint names in order.  Joint 6 is the custom gripper servo.
#: This name list matches the hardware.yaml ``servo_joints`` keys.
JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

#: The gripper is joint index 5 (0-based) -- the sixth servo.
_GRIPPER_JOINT_NAME: str = "gripper"

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

_HEADER = b"\xff\xff"
_INST_PING: int = 0x01
_INST_READ: int = 0x02
_INST_WRITE: int = 0x03
_INST_SYNC_READ: int = 0x82

# Register addresses (verified against rustypot src/servo/feetech/sts3215.rs)
_REG_MODEL_L: int = 0x03        # model number L/H (2 bytes) [3]; STS3215 = 777. Registers 0-1 are firmware major/minor (rustypot sts3215.rs). Orchestrator fix 2026-10-24: read was at 0, verified live.
_REG_TORQUE_ENABLE: int = 0x28  # torque enable (1 byte)                       [40]
_REG_GOAL_POSITION: int = 0x2A  # goal position low byte (2 bytes)             [42]
_REG_PRESENT_POSITION: int = 0x38  # present position low byte (2 bytes)       [56]
_REG_PRESENT_SPEED: int = 0x3A    # present speed low byte (2 bytes, sign-mag) [58]
_REG_PRESENT_LOAD: int = 0x3C     # present load low byte (2 bytes, sign-mag)  [60]
_REG_PRESENT_VOLTAGE: int = 0x3E  # present voltage (1 byte, unit: 0.1 V)      [62]
_REG_PRESENT_TEMPERATURE: int = 0x3F  # present temperature (1 byte, deg C)    [63]
_REG_PRESENT_CURRENT: int = 0x45  # present current (2 bytes, u16)             [69]

# ---------------------------------------------------------------------------
# AA-10: velocity/current limiting registers
# ---------------------------------------------------------------------------

# --- AA-10 ---
# STS3215 motion-limiting registers (verified against rustypot sts3215.rs):
#   goal_speed      register 0x2E (46): maximum servo speed, raw ticks/s
#   acceleration    register 0x29 (41): acceleration ramp, raw ticks/s^2
#   torque_limit    register 0x30 (48): maximum torque / load, raw units 0-1000
_REG_ACCELERATION: int = 0x29   # acceleration (1 byte)                        [41]
_REG_GOAL_SPEED: int = 0x2E     # goal speed (2 bytes, L/H)                    [46]
_REG_TORQUE_LIMIT: int = 0x30   # torque limit (2 bytes, L/H, range 0-1000)    [48]
# --- AA-10 end ---

# STS3215 response timeout in seconds
_DEFAULT_TIMEOUT_S: float = 0.1

# Maximum number of bytes in a single status packet (generous upper bound)
_MAX_PACKET: int = 32

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class ServoError(IOError):
    """Base class for all servo-bus errors.

    Callers that want to catch *any* servo fault catch this.
    """


class ServoTimeoutError(ServoError):
    """No response received within the timeout window."""

    def __init__(self, servo_id: int, operation: str) -> None:
        self.servo_id = servo_id
        self.operation = operation
        super().__init__(f"servo {servo_id}: timeout waiting for response to {operation!r}")


class ServoChecksumError(ServoError):
    """Received packet has an invalid checksum."""

    def __init__(self, servo_id: int, expected: int, actual: int) -> None:
        self.servo_id = servo_id
        self.expected = expected
        self.actual = actual
        super().__init__(
            f"servo {servo_id}: checksum error (expected 0x{expected:02X}, got 0x{actual:02X})"
        )


class ServoRangeError(ServoError):
    """A position or register value is outside the valid range for that servo."""

    def __init__(self, servo_id: int, value: object, context: str) -> None:
        self.servo_id = servo_id
        self.value = value
        self.context = context
        super().__init__(f"servo {servo_id}: range error -- {context} (got {value!r})")


class ServoStatusError(ServoError):
    """The servo returned a non-zero error byte in its status packet."""

    def __init__(self, servo_id: int, error_byte: int) -> None:
        self.servo_id = servo_id
        self.error_byte = error_byte
        super().__init__(f"servo {servo_id}: status error byte 0x{error_byte:02X}")


class GripperWriteNotAllowed(ServoError):
    """Raised when a general caller attempts to write the gripper joint position.

    Gripper position writes must go through the AA-17 state machine via
    :meth:`ArmServoBus.gripper_channel` (capability-token API).
    """

    def __init__(self) -> None:
        super().__init__(
            "gripper joint position writes are reserved for the AA-17 state machine; "
            "obtain a GripperChannel via ArmServoBus.gripper_channel(_GRIPPER_TOKEN)"
        )


# ---------------------------------------------------------------------------
# Raw <-> degrees conversion  (isolated here -- only place in the codebase)
# ---------------------------------------------------------------------------


def ticks_to_degrees(ticks: int) -> Degrees:
    """Convert raw STS3215 position ticks (0-4095) to degrees (0.0-360.0).

    This is the single conversion point; all other code calls here.
    """
    return float(ticks) * 360.0 / _TICKS_PER_REVOLUTION


def degrees_to_ticks(degrees: Degrees) -> int:
    """Convert degrees (0.0-360.0) to raw STS3215 ticks (0-4095).

    Raises :class:`ValueError` if ``degrees`` is not in [0, 360].
    This is the single conversion point; all other code calls here.
    """
    if not (0.0 <= degrees <= 360.0):
        raise ValueError(f"degrees must be in [0, 360], got {degrees!r}")
    return round(degrees * _TICKS_PER_REVOLUTION / 360.0)


# ---------------------------------------------------------------------------
# Sign-magnitude decoding helpers
# ---------------------------------------------------------------------------


def _decode_speed_sign_magnitude(raw16: int) -> int:
    """Decode a 16-bit sign-magnitude present_speed value (bit 15 = sign).

    The STS3215 encodes present_speed as sign-magnitude, NOT two's complement
    (verified against rustypot src/servo/feetech/sts3215.rs).
    Bit 15 is the sign bit; bits 0-14 are the magnitude.
    """
    sign = (raw16 >> 15) & 1
    magnitude = raw16 & 0x7FFF
    return -magnitude if sign else magnitude


def _decode_load_sign_magnitude(raw16: int) -> int:
    """Decode a 16-bit sign-magnitude present_load value (bit 10 = sign).

    The STS3215 encodes present_load as sign-magnitude, NOT two's complement
    (verified against rustypot src/servo/feetech/sts3215.rs).
    Bit 10 is the sign bit; bits 0-9 are the magnitude.
    """
    sign = (raw16 >> 10) & 1
    magnitude = raw16 & 0x03FF
    return -magnitude if sign else magnitude


# ---------------------------------------------------------------------------
# SCS protocol framing helpers
# ---------------------------------------------------------------------------


def _checksum(id_: int, length: int, instruction: int, params: bytes) -> int:
    """Feetech SCS checksum: (~(ID + LENGTH + INST + sum(params))) & 0xFF."""
    total = id_ + length + instruction + sum(params)
    return (~total) & 0xFF


def _build_request(id_: int, instruction: int, params: bytes) -> bytes:
    """Build a complete SCS request packet."""
    length = len(params) + 2  # params + instruction + checksum
    chk = _checksum(id_, length, instruction, params)
    return _HEADER + bytes([id_, length, instruction]) + params + bytes([chk])


def _parse_status_packet(data: bytes, expected_id: int) -> tuple[int, bytes]:
    """Parse a status packet; return ``(error_byte, payload)``.

    Raises :class:`ServoChecksumError` on checksum mismatch,
    :class:`ServoStatusError` on a non-zero error byte.
    """
    # Minimum valid packet: 0xFF 0xFF ID LEN ERR CHK -> 6 bytes
    if len(data) < 6:
        raise ServoError(
            f"servo {expected_id}: status packet too short ({len(data)} bytes)"
        )
    # Find header
    idx = data.find(b"\xff\xff")
    if idx == -1:
        raise ServoError(f"servo {expected_id}: no header found in response")
    data = data[idx:]
    if len(data) < 6:
        raise ServoError(f"servo {expected_id}: packet too short after header strip")

    pkt_id = data[2]
    length = data[3]
    error_byte = data[4]
    payload = data[5 : 4 + length - 1]  # LEN counts: ERROR + params + CHK; CHK excluded
    received_chk = data[4 + length - 1]

    # Recompute checksum over ID, LENGTH, ERROR, params
    chk_input_params = bytes([error_byte]) + payload
    expected_chk = _checksum(pkt_id, length, 0x00, chk_input_params)
    # The SCS checksum for a status packet is (~(ID+LEN+ERR+sum(data))) & 0xFF
    # which is identical to the request formula with instruction=0 and
    # params = [error_byte] + payload_bytes
    if received_chk != expected_chk:
        raise ServoChecksumError(expected_id, expected_chk, received_chk)
    if error_byte != 0:
        raise ServoStatusError(pkt_id, error_byte)
    return error_byte, payload


# ---------------------------------------------------------------------------
# Low-level bus (transport-level framing)
# ---------------------------------------------------------------------------


class _ServoBus:
    """Low-level SCS framing layer over a :class:`SerialTransport`.

    Used internally by :class:`ArmServoBus`.  Not part of the public API.
    """

    def __init__(
        self,
        transport: SerialTransport,
        timeout_s: float = _DEFAULT_TIMEOUT_S,
    ) -> None:
        self._transport = transport
        self._timeout_s = timeout_s

    def _transact(self, request: bytes, servo_id: int, operation: str) -> bytes:
        """Write ``request`` and read back a status packet.

        Returns the raw bytes of the response (including header).
        Raises :class:`ServoTimeoutError` if no response arrives.
        """
        self._transport.write(request)
        deadline = time.monotonic() + self._timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._transport.read(_MAX_PACKET)
            if chunk:
                buf.extend(chunk)
                # A status packet is complete when we have the header + LEN bytes
                if len(buf) >= 4 and b"\xff\xff" in buf:
                    idx = buf.find(b"\xff\xff")
                    if idx != -1 and len(buf) >= idx + 4:
                        length = buf[idx + 3]
                        if len(buf) >= idx + 4 + length:
                            break
        if not buf:
            raise ServoTimeoutError(servo_id, operation)
        return bytes(buf)

    def ping(self, servo_id: int) -> bool:
        """Return True if servo responds to a PING; raises on communication error."""
        request = _build_request(servo_id, _INST_PING, b"")
        try:
            raw = self._transact(request, servo_id, "ping")
            _parse_status_packet(raw, servo_id)
            return True
        except ServoTimeoutError:
            return False

    def read_bytes(self, servo_id: int, register: int, length: int) -> bytes:
        """Read ``length`` bytes starting at ``register`` from ``servo_id``."""
        params = bytes([register, length])
        request = _build_request(servo_id, _INST_READ, params)
        raw = self._transact(request, servo_id, f"read reg=0x{register:02X} len={length}")
        _, payload = _parse_status_packet(raw, servo_id)
        if len(payload) < length:
            raise ServoRangeError(
                servo_id,
                len(payload),
                f"expected {length} bytes from register 0x{register:02X}, got {len(payload)}",
            )
        return payload[:length]

    def write_bytes(self, servo_id: int, register: int, data: bytes) -> None:
        """Write ``data`` to ``register`` on ``servo_id``."""
        params = bytes([register]) + data
        request = _build_request(servo_id, _INST_WRITE, params)
        raw = self._transact(request, servo_id, f"write reg=0x{register:02X}")
        _parse_status_packet(raw, servo_id)

    def sync_read(
        self, servo_ids: Sequence[int], register: int, length: int
    ) -> Dict[int, bytes]:
        """Sync-read ``length`` bytes at ``register`` from multiple servos.

        Returns a dict mapping each servo_id to its payload bytes.
        Only servos that respond are included in the result.
        """
        # SCS sync-read: 0x82, start_addr, data_len, ID1, ID2, ...
        params = bytes([register, length]) + bytes(servo_ids)
        request = _build_request(0xFE, _INST_SYNC_READ, params)
        self._transport.write(request)
        deadline = time.monotonic() + self._timeout_s * len(servo_ids)
        results: Dict[int, bytes] = {}
        remaining = set(servo_ids)
        buf = bytearray()
        while time.monotonic() < deadline and remaining:
            chunk = self._transport.read(_MAX_PACKET)
            if chunk:
                buf.extend(chunk)
            # Try to parse packets out of buf
            while b"\xff\xff" in buf:
                idx = buf.find(b"\xff\xff")
                if idx + 4 > len(buf):
                    break
                pkt_len = buf[idx + 3]
                pkt_end = idx + 4 + pkt_len
                if pkt_end > len(buf):
                    break
                pkt = bytes(buf[idx:pkt_end])
                del buf[: pkt_end]
                pkt_id = pkt[2]
                try:
                    _, payload = _parse_status_packet(pkt, pkt_id)
                    if pkt_id in remaining:
                        results[pkt_id] = payload[:length]
                        remaining.discard(pkt_id)
                except ServoError:
                    # Skip malformed packets from other servos
                    pass
        return results


# ---------------------------------------------------------------------------
# Joint state dataclass
# ---------------------------------------------------------------------------


class JointState:
    """Snapshot of one servo's sensor readings.

    All values at the public interface boundary:
      - ``position_deg``:  joint angle in **degrees** (0-360)
      - ``speed_raw``:     signed raw speed ticks, sign-magnitude decoded
                           (unit: steps/s, hardware-defined)
      - ``load_raw``:      signed raw load value, sign-magnitude decoded
                           (hardware-defined units)
      - ``voltage_v``:     bus voltage in **volts**
      - ``temperature_c``: temperature in **deg C**
      - ``current_raw``:   raw current register value (u16, hardware units)
    """

    __slots__ = (
        "servo_id",
        "position_deg",
        "speed_raw",
        "load_raw",
        "voltage_v",
        "temperature_c",
        "current_raw",
    )

    def __init__(
        self,
        servo_id: int,
        position_deg: Degrees,
        speed_raw: int,
        load_raw: int,
        voltage_v: float,
        temperature_c: int,
        current_raw: int = 0,
    ) -> None:
        self.servo_id = servo_id
        self.position_deg = position_deg
        self.speed_raw = speed_raw
        self.load_raw = load_raw
        self.voltage_v = voltage_v
        self.temperature_c = temperature_c
        self.current_raw = current_raw

    def __repr__(self) -> str:
        return (
            f"JointState(servo_id={self.servo_id}, "
            f"position_deg={self.position_deg:.2f}, "
            f"speed_raw={self.speed_raw}, load_raw={self.load_raw}, "
            f"voltage_v={self.voltage_v:.1f}, temperature_c={self.temperature_c}, "
            f"current_raw={self.current_raw})"
        )


# ---------------------------------------------------------------------------
# Gripper capability object
# ---------------------------------------------------------------------------


class GripperChannel:
    """Capability object that grants write access to the gripper servo.

    Instances are obtained exclusively via
    :meth:`ArmServoBus.gripper_channel` by passing the sentinel token from
    :mod:`inchworm.gripper_access`.  This object is the **only** path to
    moving the gripper servo; it is intended for the AA-17 state machine.

    In dry-run mode all writes are logged but never sent to the transport.
    """

    def __init__(self, bus: "ArmServoBus") -> None:
        self._bus = bus

    def write_position_deg(self, position_deg: Degrees) -> None:
        """Write a goal position (in degrees) to the gripper servo.

        In dry-run mode the command is logged but not sent.
        """
        servo_id = self._bus._servo_id_for_joint(_GRIPPER_JOINT_NAME)
        if self._bus._dry_run:
            self._bus._emit(
                "info",
                "gripper_position_dry_run",
                {"servo_id": servo_id, "position_deg": position_deg},
            )
            return
        try:
            ticks = degrees_to_ticks(position_deg)
        except ValueError as exc:
            raise ServoRangeError(servo_id, position_deg, str(exc)) from exc
        data = struct.pack("<H", ticks)
        self._bus._bus.write_bytes(servo_id, _REG_GOAL_POSITION, data)
        self._bus._emit(
            "info",
            "gripper_position_written",
            {"servo_id": servo_id, "position_deg": position_deg, "ticks": ticks},
        )


# ---------------------------------------------------------------------------
# Public driver
# ---------------------------------------------------------------------------


class ArmServoBus:
    """SO-101 servo-bus driver for one arm.

    Parameters
    ----------
    transport:
        An open :class:`~inchworm.backends.SerialTransport` for this arm's
        CH343 adapter.
    role:
        Logical role: ``"arm_left"`` or ``"arm_right"``.
    joint_id_map:
        Mapping from joint name to servo ID, e.g.
        ``{"shoulder_pan": 1, "shoulder_lift": 2, ...}``.
        Joint names must match :data:`JOINT_NAMES`.  The gripper joint
        (``"gripper"``) **must** be present.
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger` for AA-7 event
        emission.  When omitted no events are emitted.
    timeout_s:
        Per-operation read timeout in seconds.
    dry_run:
        When True, position-write and torque commands are logged but never
        sent to the transport (doc-1 s8 dry-run requirement).

    Usage::

        bus = ArmServoBus(transport, role="arm_left",
                          joint_id_map={"shoulder_pan": 1, ...})
        bus.ping(servo_id=1)
        state = bus.read_joint_state(servo_id=1)
        bus.write_position_deg("shoulder_pan", 90.0)
        bus.set_torque_enable(servo_id=1, enable=True)

        # Gripper writes (AA-17 only):
        from inchworm.gripper_access import _GRIPPER_TOKEN
        ch = bus.gripper_channel(_GRIPPER_TOKEN)
        ch.write_position_deg(45.0)
    """

    def __init__(
        self,
        transport: SerialTransport,
        role: str,
        joint_id_map: Dict[str, int],
        logger: Optional[object] = None,
        timeout_s: float = _DEFAULT_TIMEOUT_S,
        dry_run: bool = False,
    ) -> None:
        if role not in ("arm_left", "arm_right"):
            raise ValueError(f"role must be 'arm_left' or 'arm_right', got {role!r}")
        if _GRIPPER_JOINT_NAME not in joint_id_map:
            raise ValueError(
                f"joint_id_map must include the '{_GRIPPER_JOINT_NAME}' joint "
                f"(required for the AA-17 state machine)"
            )
        self._bus = _ServoBus(transport, timeout_s=timeout_s)
        self._role = role
        self._joint_id_map: Dict[str, int] = dict(joint_id_map)
        self._logger = logger
        self._dry_run = dry_run

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @property
    def role(self) -> str:
        return self._role

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def _servo_id_for_joint(self, joint_name: str) -> int:
        try:
            return self._joint_id_map[joint_name]
        except KeyError:
            known = ", ".join(sorted(self._joint_id_map))
            raise ServoError(
                f"unknown joint {joint_name!r} on {self._role}; known: {known}"
            ) from None

    def _emit(self, severity: str, event_type: str, payload: dict) -> None:
        if self._logger is not None:
            method = getattr(self._logger, severity.lower(), None)
            if method is not None:
                method(
                    role=self._role,
                    subsystem="servo_bus",
                    event_type=event_type,
                    payload=payload,
                )

    # ------------------------------------------------------------------
    # Ping
    # ------------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """Return True if servo_id is present on the bus."""
        result = self._bus.ping(servo_id)
        self._emit(
            "debug",
            "servo_ping",
            {"servo_id": servo_id, "responded": result},
        )
        return result

    # ------------------------------------------------------------------
    # Read position
    # ------------------------------------------------------------------

    def read_position_deg(self, servo_id: int) -> Degrees:
        """Read present position of ``servo_id`` in degrees."""
        raw = self._bus.read_bytes(servo_id, _REG_PRESENT_POSITION, 2)
        ticks = struct.unpack_from("<H", raw)[0]
        if ticks > 4095:
            raise ServoRangeError(servo_id, ticks, "present position ticks out of 0-4095")
        deg = ticks_to_degrees(ticks)
        self._emit(
            "debug",
            "servo_position_read",
            {"servo_id": servo_id, "ticks": ticks, "degrees": deg},
        )
        return deg

    # ------------------------------------------------------------------
    # Read sensors
    # ------------------------------------------------------------------

    def read_joint_state(self, servo_id: int) -> JointState:
        """Read position, speed, load, voltage, temperature, and current.

        A single 15-byte block read starting at 0x38 covers all registers
        through 0x46 (present_current high byte):

          offset  0-1  : present_position (0x38)
          offset  2-3  : present_speed    (0x3A, sign-magnitude)
          offset  4-5  : present_load     (0x3C, sign-magnitude)
          offset  6    : present_voltage  (0x3E)
          offset  7    : present_temp     (0x3F)
          offset  8-12 : reserved / other registers (0x40-0x44)
          offset 13-14 : present_current  (0x45, u16)

        Speed and load are sign-magnitude decoded per the STS3215 register
        spec (verified against rustypot src/servo/feetech/sts3215.rs).
        """
        # 15 bytes: 0x38 through 0x46 inclusive (0x46 - 0x38 + 1 = 15)
        _BLOCK_LEN = 15
        _CURRENT_OFFSET = 13  # 0x45 - 0x38
        raw = self._bus.read_bytes(servo_id, _REG_PRESENT_POSITION, _BLOCK_LEN)
        pos_ticks = struct.unpack_from("<H", raw, 0)[0]
        # present_speed: bit 15 = sign, bits 0-14 = magnitude
        speed_raw16 = struct.unpack_from("<H", raw, 2)[0]
        speed_raw = _decode_speed_sign_magnitude(speed_raw16)
        # present_load: bit 10 = sign, bits 0-9 = magnitude
        load_raw16 = struct.unpack_from("<H", raw, 4)[0]
        load_raw = _decode_load_sign_magnitude(load_raw16)
        voltage_raw = raw[6]
        temperature_c = raw[7]
        current_raw = struct.unpack_from("<H", raw, _CURRENT_OFFSET)[0]

        if pos_ticks > 4095:
            raise ServoRangeError(servo_id, pos_ticks, "present position ticks out of 0-4095")

        pos_deg = ticks_to_degrees(pos_ticks)
        voltage_v = voltage_raw * 0.1

        state = JointState(
            servo_id=servo_id,
            position_deg=pos_deg,
            speed_raw=speed_raw,
            load_raw=load_raw,
            voltage_v=voltage_v,
            temperature_c=temperature_c,
            current_raw=current_raw,
        )
        self._emit(
            "debug",
            "servo_joint_state_read",
            {
                "servo_id": servo_id,
                "position_deg": pos_deg,
                "speed_raw": speed_raw,
                "load_raw": load_raw,
                "voltage_v": voltage_v,
                "temperature_c": temperature_c,
                "current_raw": current_raw,
            },
        )
        return state

    def read_all_joint_states(self) -> Dict[str, JointState]:
        """Read joint states for every joint in the map.

        Returns a dict mapping joint name -> :class:`JointState`.
        """
        servo_ids = list(self._joint_id_map.values())
        # 15-byte block: 0x38..0x46 covers position, speed, load, voltage,
        # temperature, and current in one contiguous read.
        _BLOCK_LEN = 15
        _CURRENT_OFFSET = 13  # 0x45 - 0x38

        # Orchestrator 2026-10-24, measured live on both arms: SYNC_READ
        # replies collide on this bus (servo 3's reply is truncated by servo
        # 4's, firmware 3.9 vs 3.10), so the parser waited out its deadline
        # (~600 ms) and fell back anyway. Sequential per-servo block reads are
        # reliable and fast: 6 servos x 15 B in a mean of 5.1 ms (~195 Hz),
        # 0 errors over 200 cycles per arm. sync_read stays available but
        # is not used here.
        raw_map: Dict[int, bytes] = {}
        for sid in servo_ids:
            raw_map[sid] = self._bus.read_bytes(sid, _REG_PRESENT_POSITION, _BLOCK_LEN)

        result: Dict[str, JointState] = {}
        for joint_name, servo_id in self._joint_id_map.items():
            if servo_id in raw_map:
                raw = raw_map[servo_id]
                pos_ticks = struct.unpack_from("<H", raw, 0)[0]
                # Sign-magnitude decode for speed and load
                speed_raw = _decode_speed_sign_magnitude(
                    struct.unpack_from("<H", raw, 2)[0]
                )
                load_raw = _decode_load_sign_magnitude(
                    struct.unpack_from("<H", raw, 4)[0]
                )
                voltage_raw = raw[6]
                temperature_c = raw[7]
                current_raw = struct.unpack_from("<H", raw, _CURRENT_OFFSET)[0]
                if pos_ticks > 4095:
                    raise ServoRangeError(
                        servo_id, pos_ticks, "present position ticks out of 0-4095"
                    )
                result[joint_name] = JointState(
                    servo_id=servo_id,
                    position_deg=ticks_to_degrees(pos_ticks),
                    speed_raw=speed_raw,
                    load_raw=load_raw,
                    voltage_v=voltage_raw * 0.1,
                    temperature_c=temperature_c,
                    current_raw=current_raw,
                )
            else:
                # Fall back to individual read if sync-read did not respond
                result[joint_name] = self.read_joint_state(servo_id)

        self._emit(
            "debug",
            "servo_all_joint_states_read",
            {"joint_count": len(result)},
        )
        return result

    # ------------------------------------------------------------------
    # Torque enable / disable
    # ------------------------------------------------------------------

    def set_torque_enable(self, servo_id: int, enable: bool) -> None:
        """Enable or disable torque on ``servo_id``.

        In dry-run mode the command is logged but not sent.
        """
        if self._dry_run:
            self._emit(
                "info",
                "servo_torque_dry_run",
                {"servo_id": servo_id, "enable": enable},
            )
            return
        if enable:
            # SAFETY (orchestrator 2026-10-24, found on live hardware before
            # any motion): with torque off, goal_position (reg 42) is stale.
            # It read 0 deg on all 12 servos while present positions were
            # 3-275 deg. Enabling torque makes the servo drive at full speed
            # to the stale goal, so the joint would slam up to 275 deg. ALWAYS
            # set goal := present (read back and verified) before enabling
            # torque. There is no way to enable torque without this step.
            present = self._bus.read_bytes(servo_id, _REG_PRESENT_POSITION, 2)
            self._bus.write_bytes(servo_id, _REG_GOAL_POSITION, present)
            echo = self._bus.read_bytes(servo_id, _REG_GOAL_POSITION, 2)
            if echo != present:
                raise ServoRangeError(
                    servo_id, echo.hex(),
                    f"goal_position read-back {echo.hex()} != present {present.hex()}; "
                    "refusing to enable torque",
                )
        data = bytes([1 if enable else 0])
        self._bus.write_bytes(servo_id, _REG_TORQUE_ENABLE, data)
        self._emit(
            "info",
            "servo_torque_enabled" if enable else "servo_torque_disabled",
            {"servo_id": servo_id},
        )

    # ------------------------------------------------------------------
    # Position write -- public (non-gripper joints only)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Position write -- public (non-gripper joints only)
    # ------------------------------------------------------------------

    # --- AA-10 ---
    def set_goal_speed(self, servo_id: int, speed_ticks: int) -> None:
        """Set the goal speed (velocity limit) for ``servo_id``.

        Writes STS3215 goal_speed register 0x2E (46) as a 2-byte little-endian
        value. This limits how fast the servo moves toward goal_position.

        Register verified against rustypot src/servo/feetech/sts3215.rs.

        Parameters
        ----------
        servo_id:
            Target servo ID.
        speed_ticks:
            Raw speed limit in ticks/s (0 = maximum speed, hardware defined).
        """
        if self._dry_run:
            self._emit(
                "info",
                "servo_goal_speed_dry_run",
                {"servo_id": servo_id, "speed_ticks": speed_ticks},
            )
            return
        data = struct.pack("<H", speed_ticks)
        self._bus.write_bytes(servo_id, _REG_GOAL_SPEED, data)
        self._emit(
            "info",
            "servo_goal_speed_set",
            {"servo_id": servo_id, "speed_ticks": speed_ticks},
        )

    def set_acceleration(self, servo_id: int, acceleration_ticks: int) -> None:
        """Set the acceleration ramp for ``servo_id``.

        Writes STS3215 acceleration register 0x29 (41) as a 1-byte value.
        This controls how quickly the servo ramps up to goal_speed.

        Register verified against rustypot src/servo/feetech/sts3215.rs.

        Parameters
        ----------
        servo_id:
            Target servo ID.
        acceleration_ticks:
            Raw acceleration (0-254, hardware-defined units). Smaller = gentler ramp.
        """
        if self._dry_run:
            self._emit(
                "info",
                "servo_acceleration_dry_run",
                {"servo_id": servo_id, "acceleration_ticks": acceleration_ticks},
            )
            return
        data = bytes([acceleration_ticks & 0xFF])
        self._bus.write_bytes(servo_id, _REG_ACCELERATION, data)
        self._emit(
            "info",
            "servo_acceleration_set",
            {"servo_id": servo_id, "acceleration_ticks": acceleration_ticks},
        )

    def set_torque_limit(self, servo_id: int, torque_limit_raw: int) -> None:
        """Set the torque (load) limit for ``servo_id``.

        Writes STS3215 torque_limit register 0x30 (48) as a 2-byte little-endian
        value. Range: 0-1000 (hardware-defined). 1000 = full torque.

        Register verified against rustypot src/servo/feetech/sts3215.rs.

        Parameters
        ----------
        servo_id:
            Target servo ID.
        torque_limit_raw:
            Raw torque limit (0-1000).
        """
        if torque_limit_raw < 0 or torque_limit_raw > 1000:
            raise ServoRangeError(
                servo_id,
                torque_limit_raw,
                "torque_limit_raw must be in [0, 1000]",
            )
        if self._dry_run:
            self._emit(
                "info",
                "servo_torque_limit_dry_run",
                {"servo_id": servo_id, "torque_limit_raw": torque_limit_raw},
            )
            return
        data = struct.pack("<H", torque_limit_raw)
        self._bus.write_bytes(servo_id, _REG_TORQUE_LIMIT, data)
        self._emit(
            "info",
            "servo_torque_limit_set",
            {"servo_id": servo_id, "torque_limit_raw": torque_limit_raw},
        )
    # --- AA-10 end ---

    def write_position_deg(self, joint_name: str, position_deg: Degrees) -> None:
        """Write a goal position (in degrees) for a named joint.

        Raises :class:`GripperWriteNotAllowed` if ``joint_name`` is
        ``"gripper"``.  Gripper position writes must go through
        :meth:`gripper_channel` (reserved for AA-17).
        """
        if joint_name == _GRIPPER_JOINT_NAME:
            raise GripperWriteNotAllowed()
        servo_id = self._servo_id_for_joint(joint_name)
        self._write_goal_position(servo_id, joint_name, position_deg)

    # ------------------------------------------------------------------
    # Gripper capability gate (AA-17 state machine only)
    # ------------------------------------------------------------------

    def gripper_channel(self, token: object) -> GripperChannel:
        """Return a :class:`GripperChannel` capability object.

        The ``token`` argument **must** be the singleton
        ``inchworm.gripper_access._GRIPPER_TOKEN``.  Any other value raises
        :class:`GripperWriteNotAllowed`.

        This gate enforces that only the AA-17 state machine
        (``inchworm/gripper_lock*.py``) -- which imports the token explicitly
        -- can obtain write access to the gripper servo.
        """
        from inchworm.gripper_access import _GRIPPER_TOKEN  # local import

        if token is not _GRIPPER_TOKEN:
            raise GripperWriteNotAllowed()
        return GripperChannel(self)

    # ------------------------------------------------------------------
    # Internal: shared goal-position write
    # ------------------------------------------------------------------

    def _write_goal_position(
        self, servo_id: int, joint_name: str, position_deg: Degrees
    ) -> None:
        if self._dry_run:
            self._emit(
                "info",
                "servo_position_dry_run",
                {
                    "servo_id": servo_id,
                    "joint_name": joint_name,
                    "position_deg": position_deg,
                },
            )
            return
        try:
            ticks = degrees_to_ticks(position_deg)
        except ValueError as exc:
            raise ServoRangeError(servo_id, position_deg, str(exc)) from exc
        data = struct.pack("<H", ticks)
        self._bus.write_bytes(servo_id, _REG_GOAL_POSITION, data)
        self._emit(
            "info",
            "servo_position_written",
            {
                "servo_id": servo_id,
                "joint_name": joint_name,
                "position_deg": position_deg,
                "ticks": ticks,
            },
        )

    # ------------------------------------------------------------------
    # Scan (discovery helper)
    # ------------------------------------------------------------------

    def scan_bus(
        self, id_range: range = range(1, 253)
    ) -> List[Dict[str, object]]:
        """Scan ``id_range`` and return info for every responding servo.

        Returns a list of dicts with keys ``servo_id``, ``model_l``,
        ``position_deg``.  Servos that do not respond are silently skipped.

        This is a slow sequential scan intended for initial setup only.
        """
        found = []
        for sid in id_range:
            if self._bus.ping(sid):
                try:
                    model_raw = self._bus.read_bytes(sid, _REG_MODEL_L, 2)
                    model_num = struct.unpack_from("<H", model_raw)[0]
                except ServoError:
                    model_num = 0
                try:
                    pos_deg = self.read_position_deg(sid)
                except ServoError:
                    pos_deg = float("nan")
                entry = {
                    "servo_id": sid,
                    "model_num": model_num,
                    "position_deg": pos_deg,
                }
                found.append(entry)
                self._emit(
                    "info",
                    "servo_discovered",
                    entry,
                )
        return found
