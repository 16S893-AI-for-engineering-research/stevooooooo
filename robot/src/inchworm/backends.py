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

    # --- AA-14 ---
    def apply_settings(self, settings: "object") -> "object":
        """Apply camera settings and return an AppliedSettings read-back.

        Default implementation: returns settings values unchanged (no-op).
        Subclasses (V4L2, MockMjpegCameraTransport) override this.
        The signature uses ``object`` to avoid importing CameraSettings/
        AppliedSettings from camera_capture at the module level.
        """
        from inchworm.camera_capture import AppliedSettings
        s = settings  # type: ignore[assignment]
        return AppliedSettings(
            width=getattr(s, "width", None),
            height=getattr(s, "height", None),
            fps=getattr(s, "fps", None),
            exposure=getattr(s, "exposure", None),
            gain=getattr(s, "gain", None),
            auto_exposure=getattr(s, "auto_exposure", None),
            auto_gain=getattr(s, "auto_gain", None),
        )

    def read_frame_with_counter(self) -> "tuple[bytes, Optional[int]]":
        """Return ``(raw_bytes, transport_counter)``.

        Default implementation delegates to :meth:`read_frame` with
        ``counter=None`` (no transport-layer counter available).
        Subclasses may return an incrementing counter for drop detection.
        """
        return self.read_frame(), None
    # --- AA-14 end ---


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
        # --- AA-14 ---
        # V4L2CameraTransport.open() implements the real camera path; it is
        # invoked by callers that have already confirmed hardware is available
        # (i.e. scripts/camera_probe.py --live, camera_cli --live).
        # HardwareBackend still raises here to preserve the existing test
        # contract; scripts bypass this by calling V4L2CameraTransport.open()
        # directly after AA-5 discovery, or by passing a V4L2 transport to
        # open_capture().
        raise BackendError(
            f"hardware camera transport for {device!r}: use V4L2CameraTransport.open() "
            "directly after AA-5 discovery, or pass --live to camera_cli / camera_probe"
        )
        # --- AA-14 end ---


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


# --- AA-14 ---


class MockMjpegCameraTransport(CameraTransport):
    """Mock camera transport that produces synthetic MJPEG frames (AA-14).

    Produces real MJPEG bytes (via cv2.imencode if available, otherwise a
    minimal JPEG substitute) so that :func:`~inchworm.camera_capture._decode_to_gray`
    can round-trip them through :func:`cv2.imdecode`.

    Supports:
    - Settings read-back (width, height, fps, exposure, gain, auto_exposure,
      auto_gain).
    - Monotonic timestamps per frame.
    - A frame counter, with configurable skip_next_n to simulate counter gaps
      (dropped-frame detection testing).
    - Configurable corrupt_next_n to produce undecipherable bytes.
    """

    def __init__(
        self,
        device: str,
        *,
        width: int = 64,
        height: int = 48,
        fps: float = 30.0,
        exposure: int = 100,
        gain: int = 1,
        auto_exposure: bool = False,
        auto_gain: bool = False,
    ) -> None:
        self.device = device
        # Applied settings (initial values)
        self._width = width
        self._height = height
        self._fps = fps
        self._exposure = exposure
        self._gain = gain
        self._auto_exposure = auto_exposure
        self._auto_gain = auto_gain
        # Transport-layer frame counter (starts at 1, increments each frame)
        self._transport_counter = 0
        self._open = True
        # Test hooks
        self.skip_next_n: int = 0  # simulate transport counter gaps
        self.corrupt_next_n: int = 0  # produce undecodable bytes

    # ------------------------------------------------------------------
    # CameraTransport protocol
    # ------------------------------------------------------------------

    def read_frame(self) -> bytes:
        """Return raw MJPEG bytes (or corrupt bytes if test hook is set)."""
        raw, _ = self.read_frame_with_counter()
        return raw

    def read_frame_with_counter(self) -> "tuple[bytes, int]":
        """Return (raw_bytes, transport_counter)."""
        if not self._open:
            raise BackendError(f"mock camera {self.device!r} is closed")
        # Simulate counter gaps (dropped frames).
        if self.skip_next_n > 0:
            self._transport_counter += self.skip_next_n + 1
            self.skip_next_n = 0
        else:
            self._transport_counter += 1

        if self.corrupt_next_n > 0:
            self.corrupt_next_n -= 1
            return b"\xff\xd8\xff\x00CORRUPT_NOT_JPEG", self._transport_counter

        raw = _make_synthetic_mjpeg(self._width, self._height)
        return raw, self._transport_counter

    def apply_settings(self, settings: "object") -> "object":
        """Apply settings and return an AppliedSettings read-back."""
        from inchworm.camera_capture import AppliedSettings, CameraSettings
        s: CameraSettings = settings  # type: ignore[assignment]
        if s.width is not None:
            self._width = s.width
        if s.height is not None:
            self._height = s.height
        if s.fps is not None:
            self._fps = s.fps
        if s.exposure is not None:
            self._exposure = s.exposure
        if s.gain is not None:
            self._gain = s.gain
        if s.auto_exposure is not None:
            self._auto_exposure = s.auto_exposure
        if s.auto_gain is not None:
            self._auto_gain = s.auto_gain
        return AppliedSettings(
            width=self._width,
            height=self._height,
            fps=self._fps,
            exposure=self._exposure,
            gain=self._gain,
            auto_exposure=self._auto_exposure,
            auto_gain=self._auto_gain,
        )

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open


def _make_synthetic_mjpeg(width: int, height: int) -> bytes:
    """Return minimal JPEG bytes for a grayscale synthetic frame.

    Uses cv2.imencode when available; falls back to a minimal valid JPEG
    byte sequence so tests that check decode roundtrip work without cv2
    installed (the round-trip will simply fail to decode, but the synthetic
    bytes at least exercise the corrupt-detection path cleanly).
    """
    try:
        import cv2 as _cv2
        import numpy as _np

        # Gray ramp image: pixel value = (x+y) mod 256.
        y_idx = _np.arange(height, dtype=_np.uint8).reshape(height, 1)
        x_idx = _np.arange(width, dtype=_np.uint8).reshape(1, width)
        gray = ((y_idx.astype(_np.uint16) + x_idx.astype(_np.uint16)) % 256).astype(_np.uint8)
        ok, buf = _cv2.imencode(".jpg", gray, [_cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    # Minimal valid JPEG (SOI + EOI) — will fail imdecode but won't panic.
    return b"\xff\xd8\xff\xd9"


class V4L2CameraTransport(CameraTransport):
    """Hardware camera transport via OpenCV V4L2 backend (AA-14).

    Opens the device node resolved by AA-5 discovery (never a raw integer
    index).  Decodes MJPEG frames using OpenCV's VideoCapture.read().

    V4L2 control IDs used:
    - CAP_PROP_FRAME_WIDTH / CAP_PROP_FRAME_HEIGHT / CAP_PROP_FPS
    - V4L2_CID_EXPOSURE_AUTO → cv2.CAP_PROP_AUTO_EXPOSURE (mapped value 3=auto, 1=manual)
    - V4L2_CID_EXPOSURE_ABSOLUTE → cv2.CAP_PROP_EXPOSURE
    - V4L2_CID_GAIN → cv2.CAP_PROP_GAIN
    """

    def __init__(self, device: str, cap: "object") -> None:
        """Not called directly; use :meth:`open`."""
        self.device = device
        self._cap = cap  # cv2.VideoCapture
        self._transport_counter = 0
        self._open = True

    @classmethod
    def open(cls, device: str, identity: "DeviceIdentity") -> "V4L2CameraTransport":
        """Open the V4L2 camera at the device node resolved by AA-5 discovery.

        Raises :class:`BackendError` if cv2 is not available or the device
        cannot be opened.
        """
        try:
            import cv2 as _cv2
        except ImportError:
            raise BackendError(
                f"opencv-python-headless is not installed; cannot open hardware camera {device!r}. "
                "Run: uv add opencv-python-headless"
            ) from None

        # Orchestrator hardening 2026-10-24: refuse to open unless called from
        # CameraArbiter.acquire() while the global single-camera lock is held.
        from inchworm.camera_arbiter import global_lock_held_for
        if not global_lock_held_for(device):
            raise BackendError(
                f"refusing to open hardware camera {device!r} without the global "
                "camera lock; use CameraArbiter.acquire()"
            )

        # Resolve the device node via AA-5 discovery.
        from inchworm.discovery import require_device
        from inchworm.config import IdentifyBy

        # Map backend device name → discovery role name.
        _BACKEND_TO_DISCOVERY_ROLE = {
            "camera_left_arm": "cam_left",
            "camera_right_arm": "cam_right",
            "camera_scanner": "cam_scanner",
        }
        discovery_role = _BACKEND_TO_DISCOVERY_ROLE.get(device, device)
        dev_node = require_device(discovery_role)

        cap = _cv2.VideoCapture(dev_node, _cv2.CAP_V4L2)
        if not cap.isOpened():
            raise BackendError(
                f"Failed to open camera {device!r} at {dev_node!r} (V4L2 backend). "
                "Is the device attached and forwarded via usbipd?"
            )
        return cls(device, cap)

    def read_frame(self) -> bytes:
        raw, _ = self.read_frame_with_counter()
        return raw

    def read_frame_with_counter(self) -> "tuple[bytes, int]":
        import cv2 as _cv2
        import numpy as _np

        if not self._open:
            raise BackendError(f"V4L2 camera {self.device!r} is closed")

        ret, frame = self._cap.read()
        self._transport_counter += 1

        if not ret or frame is None or frame.size == 0:
            # Return empty bytes; CameraCapture will flag as corrupt.
            return b"", self._transport_counter

        # Encode the decoded frame back to JPEG for uniform handling in
        # _decode_to_gray (which will decode it back to gray).  This is a
        # minor double-decode; the alternative is returning the raw numpy
        # array directly, but CameraTransport contracts bytes.
        if len(frame.shape) == 3:
            # Convert BGR → grayscale before encoding to save bandwidth.
            gray = _cv2.cvtColor(frame, _cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        ok, buf = _cv2.imencode(".jpg", gray, [_cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            return b"", self._transport_counter
        return buf.tobytes(), self._transport_counter

    def apply_settings(self, settings: "object") -> "object":
        """Apply settings via V4L2/OpenCV and read them back."""
        import cv2 as _cv2
        from inchworm.camera_capture import AppliedSettings, CameraSettings

        s: CameraSettings = settings  # type: ignore[assignment]
        cap = self._cap

        if s.width is not None:
            cap.set(_cv2.CAP_PROP_FRAME_WIDTH, float(s.width))
        if s.height is not None:
            cap.set(_cv2.CAP_PROP_FRAME_HEIGHT, float(s.height))
        if s.fps is not None:
            cap.set(_cv2.CAP_PROP_FPS, float(s.fps))
        if s.auto_exposure is not None:
            # V4L2: CAP_PROP_AUTO_EXPOSURE: 3 = auto, 1 = manual
            cap.set(_cv2.CAP_PROP_AUTO_EXPOSURE, 3.0 if s.auto_exposure else 1.0)
        if s.exposure is not None:
            cap.set(_cv2.CAP_PROP_EXPOSURE, float(s.exposure))
        if s.gain is not None:
            cap.set(_cv2.CAP_PROP_GAIN, float(s.gain))
        # auto_gain: V4L2 has no standard property; skip silently.

        # Read back
        applied_w = int(cap.get(_cv2.CAP_PROP_FRAME_WIDTH))
        applied_h = int(cap.get(_cv2.CAP_PROP_FRAME_HEIGHT))
        applied_fps = cap.get(_cv2.CAP_PROP_FPS)
        applied_ae = cap.get(_cv2.CAP_PROP_AUTO_EXPOSURE)
        applied_exp = int(cap.get(_cv2.CAP_PROP_EXPOSURE))
        applied_gain = int(cap.get(_cv2.CAP_PROP_GAIN))

        # Interpret auto_exposure read-back: 3 = aperture-priority/auto, 1 = manual
        ae_bool: Optional[bool] = None
        if applied_ae >= 0:
            ae_bool = applied_ae >= 2.0

        return AppliedSettings(
            width=applied_w if applied_w > 0 else None,
            height=applied_h if applied_h > 0 else None,
            fps=applied_fps if applied_fps > 0 else None,
            exposure=applied_exp if applied_exp >= 0 else None,
            gain=applied_gain if applied_gain >= 0 else None,
            auto_exposure=ae_bool,
            auto_gain=None,  # no standard V4L2 read-back
        )

    def close(self) -> None:
        if self._open:
            self._cap.release()
            self._open = False

    @property
    def is_open(self) -> bool:
        return self._open


class MockMjpegBackend(Backend):
    """In-memory backend that opens MockMjpegCameraTransport for camera devices (AA-14).

    Use in camera_cli / camera_probe dry-run mode so the full capture pipeline
    (settings read-back, MJPEG decode, counter tracking) is exercised without
    hardware.  Serial devices still use MockSerialTransport.
    """

    name = BackendName.MOCK  # same BackendName; distinguished by type only

    def __init__(self, config: HardwareConfig) -> None:
        super().__init__(config)
        self.serial: Dict[str, MockSerialTransport] = {}
        self.cameras: Dict[str, MockMjpegCameraTransport] = {}

    def _open_serial(self, device: str, identity: DeviceIdentity) -> MockSerialTransport:
        transport = MockSerialTransport(device)
        self.serial[device] = transport
        return transport

    def _open_camera(self, device: str, identity: DeviceIdentity) -> MockMjpegCameraTransport:
        transport = MockMjpegCameraTransport(device)
        self.cameras[device] = transport
        return transport


# --- AA-14 end ---
