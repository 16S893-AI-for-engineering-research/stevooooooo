"""Robot-side camera bridge transport (AA-53): frames from a Windows DirectShow reader.

Why: over usbip the arm cameras deliver only ~5-8 new images/s; read through
Windows DirectShow the same cameras deliver 60/s (AA-53 step 1). The Windows
program ``robot/windows/camera_bridge_server.py`` reads ONE camera and streams
JPEG frames to this transport over TCP. Servos and the DexArm stay on usbip.

Connection direction (measured 2026-11-19): WSL uses NAT networking and
Windows -> WSL works only as an outbound connection from Windows; a Windows
listener was not reachable (firewall). So this side LISTENS and the Windows
server CONNECTS OUT. The listener binds the WSL address (``listen_host: auto``
= first address of ``hostname -I``) or an explicit address such as 127.0.0.1;
the wildcard addresses 0.0.0.0 / :: are rejected by the config loader. The
listener exists only while :meth:`BridgeCameraTransport.open` waits for the
server and is closed as soon as one server has connected, so a second
concurrent stream cannot connect.

Wire protocol (little-endian; constants mirrored in the server, a test checks
they match). Every message is ``MSG_HEADER`` = (magic ``b"IWCB"``, version u8,
msg type u8, payload length u32) followed by the payload:

- robot -> server ``MSG_REQUEST`` (JSON): ``device``, ``camera`` selector
  (``{"dshow_index": N}``), ``width``, ``height``, ``fps``, ``exposure``,
  ``auto_exposure``, ``gain``. Sent once right after the server connects.
- server -> robot ``MSG_REPLY`` (JSON): ``ok``, ``error``, ``applied`` read-back.
- robot -> server ``MSG_SETTINGS`` (JSON, same settings keys) at any time;
  server -> robot ``MSG_SETTINGS_APPLIED`` (JSON read-back).
- server -> robot ``MSG_FRAME``: ``FRAME_HEADER`` = (seq u64, capture
  monotonic ns i64 [Windows clock], capture wall-clock ns i64, width u16,
  height u16, encoding u8) followed by the JPEG payload.

Latest-frame semantics (owner 2026-11-19): a background thread receives frames
and keeps only the newest one; :meth:`read_frame_with_counter` returns the
newest frame not yet returned (waiting up to ``frame_timeout_s``), never a
backlog. Server-side drops show up as seq gaps (:attr:`dropped_frames`);
frames replaced unread on this side are counted in :attr:`superseded_frames`.
The returned counter increments once per read, like V4L2CameraTransport, so
skipping stale frames is not reported as a camera drop by CameraCapture.

Failure behaviour: no server within ``connect_timeout_s`` -> BackendError from
open(). If the server disconnects mid-stream, the next read raises
BackendError; there is no automatic reconnect (the caller re-acquires the
camera through CameraArbiter, which re-opens and re-handshakes).

Corrupt frames: a payload that is not a complete JPEG (no SOI/EOI) sets
``last_integrity_error`` and, with ``reject_corrupt`` (all configured cameras),
is returned as ``b""``, the same contract as V4L2CameraTransport.
"""

from __future__ import annotations

import json
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from inchworm.backends import _REJECT_CORRUPT_DEVICES, BackendError, CameraTransport

PROTOCOL_MAGIC = b"IWCB"
PROTOCOL_VERSION = 1
MSG_HEADER = struct.Struct("<4sBBI")
FRAME_HEADER = struct.Struct("<QqqHHB")
MSG_REQUEST = 1
MSG_REPLY = 2
MSG_SETTINGS = 3
MSG_SETTINGS_APPLIED = 4
MSG_FRAME = 5
ENCODING_JPEG = 1
#: Upper bound on one message payload; a larger length means a broken stream.
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024

#: Backend device name -> key of the per-camera selector in config camera_bridge.cameras.
_DEVICE_TO_SELECTOR_KEY = {
    "camera_left_arm": "left_arm",
    "camera_right_arm": "right_arm",
    "camera_scanner": "scanner",
}

#: Device of the bridge stream currently open in this process (single-stream guard).
_ACTIVE_DEVICE: Optional[str] = None
_ACTIVE_LOCK = threading.Lock()


@dataclass(frozen=True)
class BridgeFrameInfo:
    """Metadata of one received frame. Capture times use the Windows clocks."""

    seq: int
    capture_monotonic_ns: int
    capture_wall_ns: int
    received_monotonic_ns: int
    width: int
    height: int


def pack_message(msg_type: int, payload: bytes) -> bytes:
    return MSG_HEADER.pack(PROTOCOL_MAGIC, PROTOCOL_VERSION, msg_type, len(payload)) + payload


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    chunks = []
    while n:
        chunk = sock.recv(min(n, 1 << 20))
        if not chunk:
            raise ConnectionError("bridge stream closed by peer")
        chunks.append(chunk)
        n -= len(chunk)
    return b"".join(chunks)


def recv_message(sock: socket.socket) -> Tuple[int, bytes]:
    magic, version, msg_type, length = MSG_HEADER.unpack(_recv_exact(sock, MSG_HEADER.size))
    if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION:
        raise ConnectionError(f"bad bridge header magic={magic!r} version={version}")
    if length > MAX_PAYLOAD_BYTES:
        raise ConnectionError(f"bridge payload length {length} exceeds {MAX_PAYLOAD_BYTES}")
    return msg_type, _recv_exact(sock, length)


def resolve_listen_host(listen_host: str) -> str:
    """``auto`` -> first address printed by ``hostname -I`` (the WSL eth0 address)."""
    if listen_host != "auto":
        return listen_host
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as err:
        raise BackendError(f"camera bridge: cannot resolve WSL address via hostname -I: {err}") from None
    addresses = out.split()
    if not addresses:
        raise BackendError("camera bridge: hostname -I returned no address; set camera_bridge.listen_host")
    return addresses[0]


def _settings_payload(settings: object) -> dict:
    return {
        key: getattr(settings, key, None)
        for key in ("width", "height", "fps", "exposure", "auto_exposure", "gain")
    }


class BridgeCameraTransport(CameraTransport):
    """CameraTransport fed by the Windows DirectShow bridge server (AA-53)."""

    def __init__(
        self,
        device: str,
        sock: socket.socket,
        *,
        frame_timeout_s: float,
        settings_timeout_s: float,
    ) -> None:
        """Not called directly; use :meth:`open`."""
        self.device = device
        self._sock = sock
        self._frame_timeout_s = frame_timeout_s
        self._settings_timeout_s = settings_timeout_s
        self._cond = threading.Condition()
        self._latest: Optional[Tuple[bytes, BridgeFrameInfo]] = None
        self._latest_returned = True
        self._last_seq: Optional[int] = None
        self._applied: Optional[dict] = None
        self._stream_error: Optional[str] = None
        self._transport_counter = 0
        self._open = True
        #: Frames the server dropped before sending (sum of seq gaps).
        self.dropped_frames = 0
        #: Frames received here but replaced by a newer one before being read.
        self.superseded_frames = 0
        #: Metadata of the frame returned by the last read (capture timestamps).
        self.last_frame: Optional[BridgeFrameInfo] = None
        #: Same contract as V4L2CameraTransport (AA-14 AC#3).
        self.last_integrity_error: Optional[str] = None
        self.reject_corrupt: bool = device in _REJECT_CORRUPT_DEVICES
        self._receiver = threading.Thread(
            target=self._receive_loop, name=f"camera-bridge-{device}", daemon=True
        )

    # ------------------------------------------------------------------
    # Opening
    # ------------------------------------------------------------------

    @classmethod
    def open(cls, device: str, bridge_config: "object") -> "BridgeCameraTransport":
        """Listen, wait for the Windows server, send the camera request, start receiving.

        *bridge_config* is :class:`inchworm.config.CameraBridgeConfig`.
        """
        global _ACTIVE_DEVICE
        from inchworm.camera_arbiter import global_lock_held_for

        if not global_lock_held_for(device):
            raise BackendError(
                f"refusing to open bridge camera {device!r} without the global "
                "camera lock; use CameraArbiter.acquire()"
            )
        selector_key = _DEVICE_TO_SELECTOR_KEY.get(device)
        selector = getattr(bridge_config.cameras, selector_key, None) if selector_key else None
        if selector is None:
            raise BackendError(f"camera bridge: no selector for {device!r} in camera_bridge.cameras")
        with _ACTIVE_LOCK:
            if _ACTIVE_DEVICE is not None:
                raise BackendError(
                    f"camera bridge: stream for {_ACTIVE_DEVICE!r} is still open; "
                    "only one camera may stream at a time (doc-1 s6)"
                )
            _ACTIVE_DEVICE = device
        try:
            sock = cls._accept_server(bridge_config)
            transport = cls(
                device,
                sock,
                frame_timeout_s=bridge_config.frame_timeout_s,
                settings_timeout_s=bridge_config.settings_timeout_s,
            )
            transport._handshake(selector, bridge_config)
        except BaseException:
            with _ACTIVE_LOCK:
                _ACTIVE_DEVICE = None
            raise
        return transport

    @staticmethod
    def _accept_server(bridge_config: "object") -> socket.socket:
        host = resolve_listen_host(bridge_config.listen_host)
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                listener.bind((host, bridge_config.listen_port))
            except OSError as err:
                raise BackendError(
                    f"camera bridge: cannot listen on {host}:{bridge_config.listen_port}: {err}"
                ) from None
            listener.listen(1)
            listener.settimeout(bridge_config.connect_timeout_s)
            try:
                sock, _addr = listener.accept()
            except socket.timeout:
                raise BackendError(
                    f"camera bridge: no server connected to {host}:{bridge_config.listen_port} "
                    f"within {bridge_config.connect_timeout_s} s; is camera_bridge_server.py "
                    "running on Windows with --connect pointing here?"
                ) from None
        finally:
            # Closing the listener refuses every further (second) server connection.
            listener.close()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return sock

    def _handshake(self, selector: "object", bridge_config: "object") -> None:
        request = {
            "device": self.device,
            "camera": {"dshow_index": selector.dshow_index},
            "width": bridge_config.frame_width,
            "height": bridge_config.frame_height,
            "fps": bridge_config.fps,
            "exposure": None,
            "auto_exposure": None,
            "gain": None,
        }
        try:
            self._sock.settimeout(bridge_config.connect_timeout_s)
            self._sock.sendall(pack_message(MSG_REQUEST, json.dumps(request).encode()))
            msg_type, payload = recv_message(self._sock)
        except (OSError, ConnectionError) as err:
            self._sock.close()
            raise BackendError(f"camera bridge: handshake with server failed: {err}") from None
        reply = json.loads(payload) if msg_type == MSG_REPLY else {}
        if msg_type != MSG_REPLY or not reply.get("ok"):
            self._sock.close()
            raise BackendError(
                f"camera bridge: server refused {self.device!r}: {reply.get('error', f'message type {msg_type}')}"
            )
        self._applied = reply.get("applied")
        self._sock.settimeout(None)
        self._receiver.start()

    # ------------------------------------------------------------------
    # Receive thread
    # ------------------------------------------------------------------

    def _receive_loop(self) -> None:
        try:
            while True:
                msg_type, payload = recv_message(self._sock)
                if msg_type == MSG_FRAME:
                    self._store_frame(payload)
                elif msg_type == MSG_SETTINGS_APPLIED:
                    with self._cond:
                        self._applied = json.loads(payload)
                        self._cond.notify_all()
                else:
                    raise ConnectionError(f"unexpected bridge message type {msg_type}")
        except (OSError, ConnectionError, ValueError, struct.error) as err:
            with self._cond:
                self._stream_error = str(err) or type(err).__name__
                self._cond.notify_all()

    def _store_frame(self, payload: bytes) -> None:
        if len(payload) < FRAME_HEADER.size:
            raise ConnectionError("bridge frame shorter than its header")
        seq, cap_mono, cap_wall, width, height, encoding = FRAME_HEADER.unpack_from(payload)
        info = BridgeFrameInfo(seq, cap_mono, cap_wall, time.monotonic_ns(), width, height)
        data = payload[FRAME_HEADER.size:] if encoding == ENCODING_JPEG else b""
        with self._cond:
            if self._last_seq is not None and seq > self._last_seq + 1:
                self.dropped_frames += seq - self._last_seq - 1
            self._last_seq = seq
            if not self._latest_returned:
                self.superseded_frames += 1
            self._latest = (data, info)
            self._latest_returned = False
            self._cond.notify_all()

    # ------------------------------------------------------------------
    # CameraTransport protocol
    # ------------------------------------------------------------------

    def read_frame(self) -> bytes:
        raw, _ = self.read_frame_with_counter()
        return raw

    def read_frame_with_counter(self) -> "tuple[bytes, int]":
        if not self._open:
            raise BackendError(f"bridge camera {self.device!r} is closed")
        self.last_integrity_error = None
        deadline = time.monotonic() + self._frame_timeout_s
        with self._cond:
            while self._latest_returned:
                if self._stream_error is not None:
                    raise BackendError(
                        f"bridge camera {self.device!r}: stream ended ({self._stream_error})"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise BackendError(
                        f"bridge camera {self.device!r}: no new frame within {self._frame_timeout_s} s"
                    )
                self._cond.wait(remaining)
            data, info = self._latest  # type: ignore[misc]
            self._latest_returned = True
        self._transport_counter += 1
        self.last_frame = info
        if not (data[:2] == b"\xff\xd8" and data.endswith(b"\xff\xd9")):
            self.last_integrity_error = (
                f"bridge frame seq {info.seq}: payload is not a complete JPEG ({len(data)} bytes)"
            )
            if self.reject_corrupt:
                return b"", self._transport_counter
        return data, self._transport_counter

    def apply_settings(self, settings: "object") -> "object":
        """Send settings to the server and return its read-back as AppliedSettings."""
        from inchworm.camera_capture import AppliedSettings

        if not self._open:
            raise BackendError(f"bridge camera {self.device!r} is closed")
        with self._cond:
            self._applied = None
            try:
                self._sock.sendall(pack_message(MSG_SETTINGS, json.dumps(_settings_payload(settings)).encode()))
            except OSError as err:
                raise BackendError(f"bridge camera {self.device!r}: cannot send settings: {err}") from None
            deadline = time.monotonic() + self._settings_timeout_s
            while self._applied is None:
                if self._stream_error is not None:
                    raise BackendError(f"bridge camera {self.device!r}: stream ended ({self._stream_error})")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise BackendError(
                        f"bridge camera {self.device!r}: no settings read-back within {self._settings_timeout_s} s"
                    )
                self._cond.wait(remaining)
            applied = dict(self._applied)
        return AppliedSettings(
            width=applied.get("width"),
            height=applied.get("height"),
            fps=applied.get("fps"),
            exposure=applied.get("exposure"),
            gain=applied.get("gain"),
            auto_exposure=applied.get("auto_exposure"),
            auto_gain=None,  # DirectShow has no auto-gain property in OpenCV
        )

    def close(self) -> None:
        global _ACTIVE_DEVICE
        if not self._open:
            return
        self._open = False
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()
        if self._receiver.is_alive():
            self._receiver.join(timeout=2.0)
        with _ACTIVE_LOCK:
            if _ACTIVE_DEVICE == self.device:
                _ACTIVE_DEVICE = None

    @property
    def is_open(self) -> bool:
        return self._open
