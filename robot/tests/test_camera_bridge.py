"""Tests for inchworm.camera_bridge (AA-53) with in-process fake bridge servers.

No camera or Windows host is touched: the fake servers are threads that connect
to the robot-side listener on 127.0.0.1, either speaking the protocol by hand
(``_RawServer``) or running the real Windows ``serve_stream`` with a fake camera.
"""

from __future__ import annotations

import importlib.util
import json
import socket
import threading
import time
from pathlib import Path

import pytest

from inchworm import camera_bridge as cb
from inchworm.backends import BACKEND_ENV_VAR, BackendError, HardwareBackend
from inchworm.camera_arbiter import CameraArbiter
from inchworm.camera_capture import CameraSettings, open_capture
from inchworm.config import (
    BackendName,
    CameraBridgeConfig,
    CameraTransportName,
    ConfigError,
    load_config,
    parse_config,
)

SERVER_PATH = Path(__file__).resolve().parents[1] / "windows" / "camera_bridge_server.py"


def _load_server_module():
    spec = importlib.util.spec_from_file_location("camera_bridge_server", SERVER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


server_mod = _load_server_module()


def _jpeg(value: int = 128) -> bytes:
    import cv2
    import numpy as np

    ok, buf = cv2.imencode(".jpg", np.full((48, 64), value, np.uint8))
    assert ok
    return buf.tobytes()


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(autouse=True)
def _clear_backend_env(monkeypatch):
    monkeypatch.delenv(BACKEND_ENV_VAR, raising=False)


@pytest.fixture()
def bridge_config():
    port = _free_port()
    return load_config().camera_bridge.model_copy(
        update={"listen_host": "127.0.0.1", "listen_port": port, "connect_timeout_s": 3.0,
                "frame_timeout_s": 0.5, "settings_timeout_s": 2.0}
    )


@pytest.fixture()
def bridge_backend(bridge_config, tmp_path):
    cfg = load_config().model_copy(
        update={"backend": BackendName.HARDWARE, "camera_transport": CameraTransportName.BRIDGE,
                "camera_bridge": bridge_config}
    )
    return CameraArbiter(HardwareBackend(cfg), lock_dir=tmp_path / "locks")


def _connect_with_retry(port: int, deadline_s: float = 3.0) -> socket.socket:
    end = time.monotonic() + deadline_s
    while True:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=1.0)
        except OSError:
            if time.monotonic() > end:
                raise
            time.sleep(0.02)


class _RawServer(threading.Thread):
    """Fake Windows server: connects out, answers the request, sends scripted frames."""

    def __init__(self, port: int, frames=(), *, refuse: str = "", close_after: bool = False,
                 hold_s: float = 2.0) -> None:
        super().__init__(daemon=True)
        self.port, self.frames, self.refuse = port, list(frames), refuse
        self.close_after, self.hold_s = close_after, hold_s
        self.request = None
        self.go = threading.Event()  # set by the test to release the scripted frames

    def run(self) -> None:
        sock = _connect_with_retry(self.port)
        try:
            _, payload = cb.recv_message(sock)
            self.request = json.loads(payload)
            reply = {"ok": False, "error": self.refuse} if self.refuse else {"ok": True, "applied": {}}
            sock.sendall(cb.pack_message(cb.MSG_REPLY, json.dumps(reply).encode()))
            if self.refuse:
                return
            self.go.wait(3.0)
            for seq, data in self.frames:
                header = cb.FRAME_HEADER.pack(seq, 1000 + seq, 2000 + seq, 64, 48, cb.ENCODING_JPEG)
                sock.sendall(cb.pack_message(cb.MSG_FRAME, header + data))
            if not self.close_after:
                time.sleep(self.hold_s)
        finally:
            sock.close()


class _FakeCamera:
    """Stands in for server_mod.Camera (DirectShow) inside the real serve_stream."""

    instances: list = []

    def __init__(self, dshow_index: int) -> None:
        self.dshow_index = dshow_index
        self.settings: list = []
        self.released = False
        _FakeCamera.instances.append(self)

    def apply(self, s: dict) -> dict:
        self.settings.append(s)
        return {"width": s.get("width") or 640, "height": s.get("height") or 480,
                "fps": float(s.get("fps") or 60), "exposure": s.get("exposure") or 0,
                "gain": s.get("gain") or 0, "auto_exposure": s.get("auto_exposure")}

    def read_jpeg(self):
        time.sleep(0.005)
        return _jpeg(), 64, 48

    def release(self) -> None:
        self.released = True


def _real_server(port: int) -> threading.Thread:
    def run() -> None:
        sock = _connect_with_retry(port)
        try:
            server_mod.serve_stream(sock, open_camera=_FakeCamera)
        finally:
            sock.close()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def test_protocol_constants_match_windows_server():
    for name in ("PROTOCOL_MAGIC", "PROTOCOL_VERSION", "MSG_REQUEST", "MSG_REPLY", "MSG_SETTINGS",
                 "MSG_SETTINGS_APPLIED", "MSG_FRAME", "ENCODING_JPEG", "MAX_PAYLOAD_BYTES"):
        assert getattr(server_mod, name) == getattr(cb, name), name
    assert server_mod.MSG_HEADER.format == cb.MSG_HEADER.format
    assert server_mod.FRAME_HEADER.format == cb.FRAME_HEADER.format


def test_windows_server_uses_directshow_and_never_listens():
    text = SERVER_PATH.read_text()
    assert "cv2.CAP_DSHOW" in text and "CAP_MSMF" not in text
    assert ".listen(" not in text and ".bind(" not in text


def test_frames_settings_handshake_and_incrementing_counter(bridge_backend, bridge_config):
    _FakeCamera.instances.clear()
    _real_server(bridge_config.listen_port)
    with bridge_backend.acquire("camera_left_arm") as transport:
        assert isinstance(transport, cb.BridgeCameraTransport)
        counters = [transport.read_frame_with_counter()[1] for _ in range(5)]
        assert counters == [1, 2, 3, 4, 5]
        assert transport.last_frame is not None and transport.last_frame.width == 64
        assert transport.last_frame.capture_monotonic_ns > 0
        capture = open_capture(transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(width=640, height=480, fps=30, exposure=50, gain=2))
        assert (applied.width, applied.height, applied.fps, applied.exposure, applied.gain) == (640, 480, 30.0, 50, 2)
        result = capture.grab()
        assert not result.corrupt and result.data.shape == (48, 64)
    cam = _FakeCamera.instances[0]
    assert cam.dshow_index == bridge_config.cameras.left_arm.dshow_index
    assert cam.settings[0]["width"] == bridge_config.frame_width
    assert cam.settings[0]["fps"] == bridge_config.fps
    assert cam.settings[1]["exposure"] == 50
    time.sleep(0.2)
    assert cam.released  # closing the robot side ends the stream and releases the camera


def test_slow_reader_gets_newest_frame_not_oldest(bridge_config):
    from inchworm import camera_arbiter

    srv = _RawServer(bridge_config.listen_port, [(seq, _jpeg(seq)) for seq in range(10)])
    srv.start()
    camera_arbiter._LOCK_HELD_DEVICE = "camera_left_arm"
    try:
        transport = cb.BridgeCameraTransport.open("camera_left_arm", bridge_config)
    finally:
        camera_arbiter._LOCK_HELD_DEVICE = None
    try:
        srv.go.set()
        time.sleep(0.3)  # slow reader: all ten frames arrive before the first read
        data, counter = transport.read_frame_with_counter()
        assert transport.last_frame.seq == 9 and counter == 1
        assert data == _jpeg(9)
        assert transport.superseded_frames == 9
        with pytest.raises(BackendError, match="no new frame"):
            transport.read_frame()  # never re-delivers an already-returned frame
    finally:
        transport.close()


def test_seq_gaps_counted_as_dropped_frames(bridge_config):
    from inchworm import camera_arbiter

    srv = _RawServer(bridge_config.listen_port, [(0, _jpeg()), (1, _jpeg()), (5, _jpeg())])
    srv.start()
    camera_arbiter._LOCK_HELD_DEVICE = "camera_left_arm"
    try:
        transport = cb.BridgeCameraTransport.open("camera_left_arm", bridge_config)
    finally:
        camera_arbiter._LOCK_HELD_DEVICE = None
    try:
        srv.go.set()
        time.sleep(0.3)
        transport.read_frame()
        assert transport.dropped_frames == 3
    finally:
        transport.close()


def test_corrupt_payload_rejected_like_v4l2(bridge_backend, bridge_config):
    truncated = _jpeg()[:-50]
    srv = _RawServer(bridge_config.listen_port, [(0, truncated)])
    srv.start()
    with bridge_backend.acquire("camera_left_arm") as transport:
        srv.go.set()
        assert transport.reject_corrupt
        raw, counter = transport.read_frame_with_counter()
        assert raw == b"" and counter == 1
        assert "not a complete JPEG" in transport.last_integrity_error
    # Through CameraCapture the same frame is reported corrupt (AA-14 AC#3 path).
    srv = _RawServer(bridge_config.listen_port, [(0, truncated)])
    srv.start()
    with bridge_backend.acquire("camera_left_arm") as transport:
        srv.go.set()
        time.sleep(0.2)
        result = open_capture(transport, role="camera_left_arm").grab()
        assert result.corrupt and result.data.size == 0


def test_server_disconnect_raises_backend_error(bridge_backend, bridge_config):
    srv = _RawServer(bridge_config.listen_port, [(0, _jpeg())], close_after=True)
    srv.start()
    with bridge_backend.acquire("camera_left_arm") as transport:
        srv.go.set()
        srv.join(3.0)
        time.sleep(0.1)
        transport.read_frame()  # the frame received before the disconnect is still delivered
        with pytest.raises(BackendError, match="stream ended"):
            transport.read_frame()
    # No automatic reconnect: a new acquire re-opens and re-handshakes.
    _real_server(bridge_config.listen_port)
    with bridge_backend.acquire("camera_left_arm") as transport:
        transport.read_frame()


def test_no_server_connect_timeout(bridge_backend, bridge_config):
    cfg = bridge_config.model_copy(update={"connect_timeout_s": 0.3})
    from inchworm import camera_arbiter

    camera_arbiter._LOCK_HELD_DEVICE = "camera_left_arm"
    try:
        start = time.monotonic()
        with pytest.raises(BackendError, match="no server connected"):
            cb.BridgeCameraTransport.open("camera_left_arm", cfg)
        assert time.monotonic() - start < 2.0
    finally:
        camera_arbiter._LOCK_HELD_DEVICE = None
    # The failed open released the single-stream guard.
    assert cb._ACTIVE_DEVICE is None


def test_server_refusal_raises_backend_error(bridge_backend, bridge_config):
    _RawServer(bridge_config.listen_port, refuse="cannot open DirectShow camera index 1").start()
    with pytest.raises(BackendError, match="cannot open DirectShow"):
        bridge_backend.acquire("camera_left_arm")


def test_second_concurrent_stream_refused(bridge_backend, bridge_config):
    srv = _RawServer(bridge_config.listen_port, [(0, _jpeg())], hold_s=3.0)
    srv.start()
    with bridge_backend.acquire("camera_left_arm") as transport:
        # A second Windows server cannot connect: the listener closed after the first.
        with pytest.raises(OSError):
            socket.create_connection(("127.0.0.1", bridge_config.listen_port), timeout=0.5)
        # A second bridge stream in this process is refused even past the arbiter.
        from inchworm import camera_arbiter

        camera_arbiter._LOCK_HELD_DEVICE = "camera_right_arm"
        try:
            with pytest.raises(BackendError, match="only one camera"):
                cb.BridgeCameraTransport.open("camera_right_arm", bridge_config)
        finally:
            camera_arbiter._LOCK_HELD_DEVICE = None
        assert transport.is_open
    assert cb._ACTIVE_DEVICE is None


def test_open_refused_without_arbiter_lock(bridge_config):
    with pytest.raises(BackendError, match="global camera lock"):
        cb.BridgeCameraTransport.open("camera_left_arm", bridge_config)


def test_closed_transport_read_raises(bridge_backend, bridge_config):
    _RawServer(bridge_config.listen_port, [(0, _jpeg())]).start()
    with bridge_backend.acquire("camera_left_arm") as transport:
        pass
    assert not transport.is_open
    with pytest.raises(BackendError, match="closed"):
        transport.read_frame()


# --- config ---


def _raw_config() -> dict:
    import yaml

    return yaml.safe_load(Path(load_config.__globals__["DEFAULT_CONFIG_PATH"]).read_text())


def test_default_config_selects_usbip():
    cfg = load_config()
    assert cfg.camera_transport is CameraTransportName.USBIP
    raw = _raw_config()
    del raw["camera_transport"]
    del raw["camera_bridge"]
    assert parse_config(raw).camera_transport is CameraTransportName.USBIP


def test_usbip_transport_keeps_hardware_backend_camera_refusal():
    with pytest.raises(BackendError, match="V4L2CameraTransport.open"):
        HardwareBackend(load_config())._open_camera("camera_left_arm", None)


def test_bridge_transport_requires_bridge_config():
    raw = _raw_config()
    raw["camera_transport"] = "bridge"
    del raw["camera_bridge"]
    with pytest.raises(ConfigError, match="camera_bridge is not configured"):
        parse_config(raw)


def test_unknown_camera_transport_rejected():
    raw = _raw_config()
    raw["camera_transport"] = "msmf"
    with pytest.raises(ConfigError, match="camera_transport"):
        parse_config(raw)


@pytest.mark.parametrize("host", ["0.0.0.0", "::"])
def test_wildcard_listen_host_rejected(host):
    raw = _raw_config()
    raw["camera_bridge"]["listen_host"] = host
    with pytest.raises(ConfigError, match="every interface"):
        parse_config(raw)


@pytest.mark.parametrize(
    "field,value",
    [("listen_port", 80), ("listen_port", 70000), ("connect_timeout_s", 0), ("fps", 0)],
)
def test_bad_bridge_values_rejected(field, value):
    raw = _raw_config()
    raw["camera_bridge"][field] = value
    with pytest.raises(ConfigError, match=field):
        parse_config(raw)


def test_unknown_bridge_field_rejected():
    raw = _raw_config()
    raw["camera_bridge"]["cameras"]["left_arm"]["serial"] = "x"
    with pytest.raises(ConfigError, match="unknown field"):
        parse_config(raw)


def test_bridge_selector_config_parsed():
    bridge = load_config().camera_bridge
    assert isinstance(bridge, CameraBridgeConfig)
    assert bridge.listen_host == "auto"
    assert bridge.cameras.left_arm.dshow_index >= 0
