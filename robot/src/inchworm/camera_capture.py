"""Camera capture pipeline — AA-14.

Provides :class:`CameraCapture`, the single shared capture layer used by both
wrist-observation (policy) and laser-line capture (leveling).  Every caller
must hold the AA-6 global camera lock; :class:`CameraCapture` is constructible
**only** inside an :meth:`~inchworm.camera_arbiter.CameraArbiter.acquire`
context to enforce this invariant by construction.

Hardware notes (doc-1 s4):
- Arm cameras (32e6:9251, Waveshare OV9281):  MJPEG only, resolutions
  640×480 / 800×600 / 1280×720.
- Arducam scanner (0c45:6366): MJPEG and YUY2, up to 1280×800.
- None output Y8/GREY natively; grayscale is produced in software by decoding
  MJPEG/YUY2 and converting to single-channel (uint8) with OpenCV.
- Cameras are resolved ONLY via AA-5 discovery (role → /dev/videoN by usbipd
  BusId).  Never use raw OpenCV integer indices or /dev/video by-id paths
  without going through discovery first.

Arbiter enforcement:
:class:`CameraCapture` is created by :func:`open_capture`, which requires a
:class:`~inchworm.backends.CameraTransport` that is already open (i.e. the
caller holds the arbiter lock).  There is no public constructor that accepts a
device name string; the only path is through the arbiter context.

Usage::

    arbiter = CameraArbiter(backend)
    with arbiter.acquire("camera_left_arm") as transport:
        capture = open_capture(
            transport,
            role="camera_left_arm",
            logger=session_ctx.logger,
        )
        capture.apply_settings(width=640, height=480, fps=30)
        for _ in range(10):
            result = capture.grab()
            print(result.counter, result.ts_monotonic, result.data.shape)
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional, Sequence

if TYPE_CHECKING:
    import numpy as np

from inchworm.backends import CameraTransport

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

# Sentinel: unchanged when reading back settings.  Transport may clamp values.
_UNSET = object()


@dataclass(frozen=True)
class CameraSettings:
    """Requested camera settings; all fields are optional (None = leave unchanged).

    Arm cameras (32e6:9251) support these resolutions: 640×480, 800×600, 1280×720.
    Scanner (0c45:6366) additionally supports 1280×800.
    """

    width: Optional[int] = None
    """Frame width in pixels."""

    height: Optional[int] = None
    """Frame height in pixels."""

    fps: Optional[float] = None
    """Target frame rate (frames per second)."""

    exposure: Optional[int] = None
    """Exposure value in camera-native units (device-specific).  For V4L2 this
    is V4L2_CID_EXPOSURE_ABSOLUTE in 100 µs units."""

    gain: Optional[int] = None
    """Analogue gain in camera-native units."""

    auto_exposure: Optional[bool] = None
    """True = enable auto-exposure; False = manual (required for laser-line capture)."""

    auto_gain: Optional[bool] = None
    """True = enable auto-gain; False = manual."""


@dataclass(frozen=True)
class AppliedSettings:
    """Settings actually applied and read back from the transport after set.

    A ``None`` value means the transport did not report a readable value.
    These are logged via the AA-7 telemetry channel.
    """

    width: Optional[int]
    height: Optional[int]
    fps: Optional[float]
    exposure: Optional[int]
    gain: Optional[int]
    auto_exposure: Optional[bool]
    auto_gain: Optional[bool]


@dataclass(frozen=True)
class FrameResult:
    """One delivered frame.

    Attributes
    ----------
    data:
        Single-channel uint8 grayscale numpy array, shape (height, width).
        Arm cameras (32e6:9251) output MJPEG natively; grayscale is produced
        by decoding MJPEG and calling cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).
    ts_monotonic:
        Capture timestamp: time.monotonic_ns() / 1e9 at the moment grab() was
        called.  Suitable for duration measurement within a session, not for
        absolute wall-clock correlation.
    counter:
        Monotonically increasing frame counter starting at 1.  A gap in
        successive counters indicates dropped frames (detected and reported
        via telemetry).
    dropped:
        True if one or more frames were dropped before this one (counter gap
        detected relative to the previous successful frame).
    corrupt:
        True if the raw bytes could not be decoded (JPEG decode failure or
        zero-size output from imdecode).  The data field contains an empty
        array (shape 0×0) when corrupt is True.
    """

    data: "np.ndarray"
    ts_monotonic: float
    counter: int
    dropped: bool = False
    corrupt: bool = False


# ---------------------------------------------------------------------------
# Capture errors
# ---------------------------------------------------------------------------


class CaptureError(RuntimeError):
    """Raised for unrecoverable capture failures (transport closed, etc.)."""


class ArbiterLockRequiredError(CaptureError):
    """Raised when CameraCapture is constructed without holding the arbiter lock.

    :func:`open_capture` verifies that *transport* is open (meaning the caller
    holds the global camera lock); if the transport is already closed this
    error is raised.
    """


# ---------------------------------------------------------------------------
# CameraCapture
# ---------------------------------------------------------------------------


class CameraCapture:
    """Frame source that delivers single-channel grayscale frames.

    Not constructed directly; use :func:`open_capture`.

    Parameters
    ----------
    transport:
        An open :class:`~inchworm.backends.CameraTransport` (arbiter lock held).
    role:
        Logical camera role (e.g. ``"camera_left_arm"``); used in telemetry.
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger` for structured
        event emission.  If None, falls back to the ``inchworm.camera_capture``
        stdlib logger only.
    """

    def __init__(
        self,
        transport: CameraTransport,
        role: str,
        logger: Optional[object] = None,
    ) -> None:
        # Invariant: transport must be open (caller holds arbiter lock).
        if not transport.is_open:
            raise ArbiterLockRequiredError(
                f"CameraCapture for role {role!r}: transport is not open. "
                "Construct CameraCapture only inside an arbiter.acquire() context."
            )
        self._transport = transport
        self._role = role
        self._logger = logger
        self._counter = 0
        self._last_counter = 0  # last successfully delivered frame counter

    # ------------------------------------------------------------------
    # Settings
    # ------------------------------------------------------------------

    def apply_settings(self, settings: Optional[CameraSettings] = None, **kwargs) -> AppliedSettings:
        """Apply *settings* to the transport and read back the result.

        Can be called as ``capture.apply_settings(CameraSettings(width=640))``
        or as keyword arguments: ``capture.apply_settings(width=640, fps=30)``.

        The applied values are logged via the AA-7 telemetry channel.

        Parameters
        ----------
        settings:
            A :class:`CameraSettings` instance; merged with any kwargs.
        **kwargs:
            Override individual fields of *settings*.

        Returns
        -------
        AppliedSettings
            Values read back from the transport after applying.
        """
        if settings is None:
            settings = CameraSettings(**kwargs)
        elif kwargs:
            # Merge kwargs over provided settings.
            merged = {
                f: kwargs.get(f, getattr(settings, f))
                for f in CameraSettings.__dataclass_fields__
            }
            settings = CameraSettings(**merged)

        applied = self._transport.apply_settings(settings)

        self._emit_settings_applied(settings, applied)
        return applied

    # ------------------------------------------------------------------
    # Frame grab
    # ------------------------------------------------------------------

    def grab(self) -> FrameResult:
        """Grab one frame from the transport.

        Decodes the raw bytes (MJPEG or YUY2) to a single-channel uint8
        grayscale numpy array.

        Returns
        -------
        FrameResult
            Frame data and metadata.

        Raises
        ------
        CaptureError
            If the transport is closed.
        """
        if not self._transport.is_open:
            raise CaptureError(
                f"Camera {self._role!r}: transport is closed; cannot grab frame."
            )

        ts = time.monotonic_ns() / 1e9

        # Read raw bytes and the transport's internal counter.
        raw_bytes, transport_counter = self._transport.read_frame_with_counter()

        self._counter += 1
        local_counter = self._counter

        # Detect counter gap (dropped frames at the transport layer).
        dropped = False
        if transport_counter is not None and self._last_counter > 0:
            expected = self._last_counter + 1
            if transport_counter != expected:
                gap = transport_counter - self._last_counter - 1
                dropped = True
                self._emit_drop(transport_counter, gap)

        if transport_counter is not None:
            self._last_counter = transport_counter
        else:
            self._last_counter = local_counter

        # Decode MJPEG/YUY2 → grayscale numpy array.
        data, corrupt = _decode_to_gray(raw_bytes)

        if corrupt:
            self._emit_corrupt(local_counter)

        return FrameResult(
            data=data,
            ts_monotonic=ts,
            counter=local_counter,
            dropped=dropped,
            corrupt=corrupt,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def role(self) -> str:
        """Logical camera role."""
        return self._role

    @property
    def is_open(self) -> bool:
        """True while the underlying transport is open."""
        return self._transport.is_open

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _emit(
        self,
        event_type: str,
        severity: str = "INFO",
        payload: Optional[dict] = None,
    ) -> None:
        if self._logger is not None:
            try:
                self._logger.emit(
                    role=self._role,
                    subsystem="camera_capture",
                    event_type=event_type,
                    severity=severity,
                    payload=payload or {},
                )
            except Exception:
                pass  # never let telemetry crash the capture loop
        else:
            log.log(
                logging.getLevelName(severity),
                "%s/%s %s %r",
                self._role,
                "camera_capture",
                event_type,
                payload or {},
            )

    def _emit_settings_applied(
        self, requested: CameraSettings, applied: AppliedSettings
    ) -> None:
        self._emit(
            "camera_settings_applied",
            severity="INFO",
            payload={
                "requested": {
                    "width": requested.width,
                    "height": requested.height,
                    "fps": requested.fps,
                    "exposure": requested.exposure,
                    "gain": requested.gain,
                    "auto_exposure": requested.auto_exposure,
                    "auto_gain": requested.auto_gain,
                },
                "applied": {
                    "width": applied.width,
                    "height": applied.height,
                    "fps": applied.fps,
                    "exposure": applied.exposure,
                    "gain": applied.gain,
                    "auto_exposure": applied.auto_exposure,
                    "auto_gain": applied.auto_gain,
                },
            },
        )

    def _emit_drop(self, transport_counter: int, gap: int) -> None:
        self._emit(
            "camera_frame_dropped",
            severity="WARNING",
            payload={
                "transport_counter": transport_counter,
                "frames_dropped": gap,
            },
        )

    def _emit_corrupt(self, local_counter: int) -> None:
        self._emit(
            "camera_frame_corrupt",
            severity="WARNING",
            payload={"local_counter": local_counter},
        )


# ---------------------------------------------------------------------------
# Factory — the ONLY public constructor path
# ---------------------------------------------------------------------------


def open_capture(
    transport: CameraTransport,
    *,
    role: str,
    logger: Optional[object] = None,
) -> CameraCapture:
    """Create a :class:`CameraCapture` from an already-open transport.

    This is the ONLY public constructor; it enforces that the caller holds the
    AA-6 arbiter lock by verifying that *transport* is open.

    Parameters
    ----------
    transport:
        An open :class:`~inchworm.backends.CameraTransport`.  Must be obtained
        from :meth:`~inchworm.camera_arbiter.CameraArbiter.acquire`; passing a
        closed transport raises :class:`ArbiterLockRequiredError`.
    role:
        Logical camera role name (e.g. ``"camera_left_arm"``).  Used for
        telemetry and error messages only; the device node was already
        resolved through AA-5 discovery before the transport was opened.
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger` for structured
        events.

    Returns
    -------
    CameraCapture
    """
    return CameraCapture(transport, role, logger)


# ---------------------------------------------------------------------------
# Frame decode helper
# ---------------------------------------------------------------------------


def _decode_to_gray(raw_bytes: bytes) -> "tuple[np.ndarray, bool]":
    """Decode MJPEG/YUY2 raw bytes to a single-channel uint8 grayscale array.

    Returns
    -------
    (data, corrupt)
        *data* is shape (H, W) uint8.  *corrupt* is True if decode fails.

    Notes
    -----
    Arm cameras (32e6:9251) output MJPEG only.  The Arducam scanner (0c45:6366)
    may output MJPEG or YUY2.  Neither outputs native Y8/GREY; grayscale is
    always produced in software.
    """
    import numpy as _np

    try:
        import cv2 as _cv2
    except ImportError:
        # No cv2: return corrupt=True with empty array.
        return _np.empty((0, 0), dtype=_np.uint8), True

    if not raw_bytes:
        return _np.empty((0, 0), dtype=_np.uint8), True

    buf = _np.frombuffer(raw_bytes, dtype=_np.uint8)
    # Try MJPEG (JPEG decode).
    frame = _cv2.imdecode(buf, _cv2.IMREAD_GRAYSCALE)
    if frame is not None and frame.size > 0:
        return frame, False

    # Not MJPEG; attempt YUY2 decode:
    # YUY2 has 2 bytes per pixel; total bytes = H * W * 2.
    # We extract the Y channel (every other byte starting at 0).
    # Requires knowing the shape, which YUY2 streams don't embed in the buffer.
    # Try common resolutions.
    for (w, h) in [
        (640, 480),
        (800, 600),
        (1280, 720),
        (1280, 800),
        (320, 240),
    ]:
        if len(raw_bytes) == w * h * 2:
            yuyv = _np.frombuffer(raw_bytes, dtype=_np.uint8).reshape(h, w * 2)
            gray = yuyv[:, 0::2]  # Y bytes are at even indices
            return gray.copy(), False

    # Could not decode.
    return _np.empty((0, 0), dtype=_np.uint8), True
