"""Driver backend abstraction.

Drivers (SO-101 bus, DexArm, relay, cameras) never open devices themselves;
they ask a :class:`Backend` for a transport to a named device from the
hardware config. The mock backend serves in-memory transports so all driver
and orchestration logic runs with zero hardware attached.

Selection: ``$INCHWORM_BACKEND`` if set, else the config's ``backend`` field.
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, List, Optional, Tuple

from inchworm.config import BackendName, DeviceIdentity, HardwareConfig

BACKEND_ENV_VAR = "INCHWORM_BACKEND"


class BackendError(RuntimeError):
    """Raised for unknown backends, unknown devices or unavailable transports."""


class DeviceKind(str, Enum):
    SERIAL = "serial"
    CAMERA = "camera"


def device_table(config: HardwareConfig) -> Dict[str, Tuple[DeviceKind, DeviceIdentity]]:
    """Every addressable device by stable name, with its transport kind and identity."""
    return {
        "arm_left": (DeviceKind.SERIAL, config.arms.left.identity),
        "arm_right": (DeviceKind.SERIAL, config.arms.right.identity),
        "dexarm": (DeviceKind.SERIAL, config.dexarm.identity),
        "relay": (DeviceKind.SERIAL, config.relay.identity),
        "camera_left_arm": (DeviceKind.CAMERA, config.cameras.left_arm.identity),
        "camera_right_arm": (DeviceKind.CAMERA, config.cameras.right_arm.identity),
        "camera_scanner": (DeviceKind.CAMERA, config.cameras.scanner.identity),
    }


class SerialTransport(ABC):
    """Byte-level serial link to one device."""

    @abstractmethod
    def write(self, data: bytes) -> int: ...

    @abstractmethod
    def read(self, size: int) -> bytes: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...


class CameraTransport(ABC):
    """Frame source for one camera. Closing releases the device (doc-1 s6)."""

    @abstractmethod
    def read_frame(self) -> bytes: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...


class Backend(ABC):
    name: BackendName

    def __init__(self, config: HardwareConfig) -> None:
        self.config = config
        self._devices = device_table(config)

    def _lookup(self, device: str, kind: DeviceKind) -> DeviceIdentity:
        try:
            device_kind, identity = self._devices[device]
        except KeyError:
            known = ", ".join(sorted(self._devices))
            raise BackendError(f"unknown device {device!r}; known devices: {known}") from None
        if device_kind is not kind:
            raise BackendError(f"device {device!r} is a {device_kind.value} device, not {kind.value}")
        return identity

    def open_serial(self, device: str) -> SerialTransport:
        return self._open_serial(device, self._lookup(device, DeviceKind.SERIAL))

    def open_camera(self, device: str) -> CameraTransport:
        return self._open_camera(device, self._lookup(device, DeviceKind.CAMERA))

    @abstractmethod
    def _open_serial(self, device: str, identity: DeviceIdentity) -> SerialTransport: ...

    @abstractmethod
    def _open_camera(self, device: str, identity: DeviceIdentity) -> CameraTransport: ...


class MockSerialTransport(SerialTransport):
    """Records everything written; serves bytes queued with :meth:`feed`."""

    def __init__(self, device: str) -> None:
        self.device = device
        self.written: List[bytes] = []
        self._rx = bytearray()
        self._open = True

    def feed(self, data: bytes) -> None:
        self._rx.extend(data)

    def _require_open(self) -> None:
        if not self._open:
            raise BackendError(f"mock serial {self.device!r} is closed")

    def write(self, data: bytes) -> int:
        self._require_open()
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int) -> bytes:
        self._require_open()
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open


class MockCameraTransport(CameraTransport):
    """Serves blank mono (Y8) frames of a fixed size."""

    def __init__(self, device: str, width_px: int = 64, height_px: int = 48) -> None:
        self.device = device
        self.width_px = width_px
        self.height_px = height_px
        self.frames_read = 0
        self._open = True

    def read_frame(self) -> bytes:
        if not self._open:
            raise BackendError(f"mock camera {self.device!r} is closed")
        self.frames_read += 1
        return bytes(self.width_px * self.height_px)

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open


class MockBackend(Backend):
    """In-memory transports; touches no device node."""

    name = BackendName.MOCK

    def __init__(self, config: HardwareConfig) -> None:
        super().__init__(config)
        self.serial: Dict[str, MockSerialTransport] = {}
        self.cameras: Dict[str, MockCameraTransport] = {}

    def _open_serial(self, device: str, identity: DeviceIdentity) -> MockSerialTransport:
        transport = MockSerialTransport(device)
        self.serial[device] = transport
        return transport

    def _open_camera(self, device: str, identity: DeviceIdentity) -> MockCameraTransport:
        transport = MockCameraTransport(device)
        self.cameras[device] = transport
        return transport


class HardwareBackend(Backend):
    """Real device transports.

    Device discovery is AA-5 and the per-device drivers are AA-8 (SO-101),
    AA-12 (DexArm), AA-14 (cameras) and AA-29 (relay). Until those land this
    backend refuses to open anything rather than guessing at device nodes.
    """

    name = BackendName.HARDWARE

    def _open_serial(self, device: str, identity: DeviceIdentity) -> SerialTransport:
        raise BackendError(
            f"hardware serial transport for {device!r} is not implemented yet "
            "(device discovery is AA-5); use the mock backend"
        )

    def _open_camera(self, device: str, identity: DeviceIdentity) -> CameraTransport:
        raise BackendError(
            f"hardware camera transport for {device!r} is not implemented yet "
            "(camera capture is AA-14, gated by the AA-6 arbiter); use the mock backend"
        )


_BACKENDS = {BackendName.MOCK: MockBackend, BackendName.HARDWARE: HardwareBackend}


def selected_backend_name(config: HardwareConfig) -> BackendName:
    """``$INCHWORM_BACKEND`` overrides the config's ``backend`` field."""
    raw: Optional[str] = os.environ.get(BACKEND_ENV_VAR)
    if raw is None or raw == "":
        return config.backend
    try:
        return BackendName(raw.strip().lower())
    except ValueError:
        valid = ", ".join(b.value for b in BackendName)
        raise BackendError(f"{BACKEND_ENV_VAR}={raw!r} is not a backend; expected one of: {valid}") from None


def create_backend(config: HardwareConfig) -> Backend:
    return _BACKENDS[selected_backend_name(config)](config)
