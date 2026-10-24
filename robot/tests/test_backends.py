"""Tests for inchworm.backends: mock-backend selection and zero-hardware transports."""

from __future__ import annotations

import pytest

from inchworm.backends import (
    BACKEND_ENV_VAR,
    BackendError,
    HardwareBackend,
    MockBackend,
    PySerialTransport,
    create_backend,
    device_table,
    get_backend,
    selected_backend_name,
)
from inchworm.config import BackendName, load_config


@pytest.fixture(autouse=True)
def _clear_backend_env(monkeypatch):
    monkeypatch.delenv(BACKEND_ENV_VAR, raising=False)


@pytest.fixture()
def config():
    return load_config()


def test_mock_backend_selected_by_config(config):
    assert config.backend is BackendName.MOCK
    assert isinstance(create_backend(config), MockBackend)


def test_hardware_backend_selected_by_config(config):
    hw_config = config.model_copy(update={"backend": BackendName.HARDWARE})
    assert isinstance(create_backend(hw_config), HardwareBackend)


def test_env_var_overrides_config_to_mock(config, monkeypatch):
    hw_config = config.model_copy(update={"backend": BackendName.HARDWARE})
    monkeypatch.setenv(BACKEND_ENV_VAR, "mock")
    assert selected_backend_name(hw_config) is BackendName.MOCK
    assert isinstance(create_backend(hw_config), MockBackend)


def test_env_var_overrides_config_to_hardware(config, monkeypatch):
    monkeypatch.setenv(BACKEND_ENV_VAR, "HARDWARE")
    assert isinstance(create_backend(config), HardwareBackend)


def test_empty_env_var_falls_back_to_config(config, monkeypatch):
    monkeypatch.setenv(BACKEND_ENV_VAR, "")
    assert isinstance(create_backend(config), MockBackend)


def test_invalid_env_var_rejected(config, monkeypatch):
    monkeypatch.setenv(BACKEND_ENV_VAR, "sim")
    with pytest.raises(BackendError, match=r"INCHWORM_BACKEND='sim' is not a backend"):
        create_backend(config)


def test_device_table_covers_all_seven_devices(config):
    assert sorted(device_table(config)) == [
        "arm_left",
        "arm_right",
        "camera_left_arm",
        "camera_right_arm",
        "camera_scanner",
        "dexarm",
        "relay",
    ]


@pytest.mark.parametrize("device", ["arm_left", "arm_right", "dexarm", "relay"])
def test_mock_serial_round_trip(config, device):
    backend = create_backend(config)
    link = backend.open_serial(device)
    assert link.write(b"G28\n") == 4
    link.feed(b"ok\n")
    assert link.read(16) == b"ok\n"
    assert backend.serial[device].written == [b"G28\n"]
    link.close()
    assert not link.is_open
    with pytest.raises(BackendError, match="closed"):
        link.write(b"x")


@pytest.mark.parametrize("device", ["camera_left_arm", "camera_right_arm", "camera_scanner"])
def test_mock_camera_frames(config, device):
    backend = create_backend(config)
    cam = backend.open_camera(device)
    frame = cam.read_frame()
    assert len(frame) == cam.width_px * cam.height_px
    cam.close()
    with pytest.raises(BackendError, match="closed"):
        cam.read_frame()


def test_unknown_device_rejected(config):
    with pytest.raises(BackendError, match="unknown device 'lidar'"):
        create_backend(config).open_serial("lidar")


def test_wrong_device_kind_rejected(config):
    backend = create_backend(config)
    with pytest.raises(BackendError, match="camera device, not serial"):
        backend.open_serial("camera_scanner")
    with pytest.raises(BackendError, match="serial device, not camera"):
        backend.open_camera("dexarm")


def test_hardware_backend_refuses_cameras(config):
    """HardwareBackend still refuses camera opens (requires AA-6 arbiter).

    Serial devices are now opened via discovery + PySerialTransport; cameras
    still require the AA-6 CameraArbiter.
    """
    backend = HardwareBackend(config)
    # --- AA-14 ---
    # HardwareBackend._open_camera raises BackendError directing callers to
    # V4L2CameraTransport.open().
    with pytest.raises(BackendError):
        backend.open_camera("camera_scanner")
    # --- AA-14 end ---


# --- serial-transport ---


class _FakeSerial:
    """Minimal fake serial.Serial for monkeypatching."""

    def __init__(self, *, port, baudrate, bytesize, parity, stopbits, timeout, exclusive):
        self.port = port
        self.baudrate = baudrate
        self.exclusive = exclusive
        self.is_open = True
        self._rx = bytearray()
        self.written = []

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def close(self) -> None:
        self.is_open = False


_ROLE_BAUD = {
    "arm_left": 1_000_000,
    "arm_right": 1_000_000,
    "dexarm": 115_200,
    "relay": 9_600,
}


def _make_fake_serial_module(captured: list):
    """Return a fake 'serial' module whose Serial records construction kwargs."""
    import types
    m = types.SimpleNamespace()
    m.EIGHTBITS = 8
    m.PARITY_NONE = "N"
    m.STOPBITS_ONE = 1

    class FakeSerialCls:
        def __init__(self, **kwargs):
            captured.append(kwargs)
            self._fake = _FakeSerial(**kwargs)
            self.is_open = self._fake.is_open
            self.baudrate = kwargs["baudrate"]
            self.exclusive = kwargs["exclusive"]
            self.port = kwargs["port"]
            self.written = self._fake.written

        def write(self, data):
            return self._fake.write(data)

        def read(self, size):
            return self._fake.read(size)

        def close(self):
            self._fake.close()
            self.is_open = False

    class FakeSerialException(Exception):
        pass

    m.Serial = FakeSerialCls
    m.SerialException = FakeSerialException
    return m


@pytest.mark.parametrize("device,expected_baud", [
    ("arm_left",  1_000_000),
    ("arm_right", 1_000_000),
    ("dexarm",    115_200),
    ("relay",     9_600),
])
def test_pyserial_transport_discovery_role_and_baud(monkeypatch, config, device, expected_baud):
    """PySerialTransport calls require_device with the right role and opens at the expected baud."""
    captured_nodes = []
    captured_serial = []

    def fake_require_device(role, *args, **kwargs):
        captured_nodes.append(role)
        return f"/dev/fake_{role}"

    fake_serial_mod = _make_fake_serial_module(captured_serial)

    import sys
    sys.modules["serial"] = fake_serial_mod  # type: ignore[assignment]

    try:
        transport = PySerialTransport.open(device, f"/dev/fake_{device}", None)
    finally:
        del sys.modules["serial"]

    assert len(captured_serial) == 1
    kwargs = captured_serial[0]
    assert kwargs["baudrate"] == expected_baud
    assert kwargs["exclusive"] is True
    assert kwargs["port"] == f"/dev/fake_{device}"


def test_pyserial_transport_exclusive_flag(monkeypatch):
    """PySerialTransport always opens with exclusive=True."""
    captured = []
    fake_serial_mod = _make_fake_serial_module(captured)
    import sys
    sys.modules["serial"] = fake_serial_mod  # type: ignore[assignment]
    try:
        PySerialTransport.open("arm_left", "/dev/ttyUSB0", None)
    finally:
        del sys.modules["serial"]
    assert captured[0]["exclusive"] is True


def test_pyserial_transport_config_baud_override(monkeypatch):
    """A per-device serial config block overrides the default baud rate."""
    captured = []
    fake_serial_mod = _make_fake_serial_module(captured)

    import types
    fake_cfg = types.SimpleNamespace(baudrate=500_000, timeout_s=None)

    import sys
    sys.modules["serial"] = fake_serial_mod  # type: ignore[assignment]
    try:
        PySerialTransport.open("arm_left", "/dev/ttyUSB0", fake_cfg)
    finally:
        del sys.modules["serial"]

    assert captured[0]["baudrate"] == 500_000


def test_hardware_backend_serial_calls_discovery(monkeypatch, config):
    """HardwareBackend._open_serial calls require_device with the correct role."""
    discovered_roles = []
    captured_serial = []

    def fake_require_device(role, *args, **kwargs):
        discovered_roles.append(role)
        return f"/dev/fake_{role}"

    fake_serial_mod = _make_fake_serial_module(captured_serial)

    import sys
    sys.modules["serial"] = fake_serial_mod  # type: ignore[assignment]
    monkeypatch.setattr("inchworm.discovery.require_device", fake_require_device)

    try:
        hw_config = config.model_copy(update={"backend": BackendName.HARDWARE})
        backend = HardwareBackend(hw_config)
        transport = backend.open_serial("arm_left")
    finally:
        del sys.modules["serial"]

    assert "arm_left" in discovered_roles
    assert len(captured_serial) == 1
    assert captured_serial[0]["port"] == "/dev/fake_arm_left"


@pytest.mark.parametrize("device,expected_role", [
    ("arm_left",  "arm_left"),
    ("arm_right", "arm_right"),
    ("dexarm",    "dexarm"),
    ("relay",     "relay"),
])
def test_hardware_backend_discovery_role_mapping(monkeypatch, config, device, expected_role):
    """Backend device names map to the correct discovery role names."""
    discovered = []
    captured = []

    def fake_require_device(role, *args, **kwargs):
        discovered.append(role)
        return "/dev/fake"

    import sys
    sys.modules["serial"] = _make_fake_serial_module(captured)  # type: ignore[assignment]
    monkeypatch.setattr("inchworm.discovery.require_device", fake_require_device)

    try:
        hw_config = config.model_copy(update={"backend": BackendName.HARDWARE})
        HardwareBackend(hw_config).open_serial(device)
    finally:
        del sys.modules["serial"]

    assert discovered == [expected_role]


def test_get_backend_alias(config):
    """get_backend(cfg) is equivalent to create_backend(cfg)."""
    assert type(get_backend(config)) is type(create_backend(config))


# --- serial-transport end ---


# --- Orchestrator 2026-10-24: --live paths always use real hardware ---
def test_live_backend_ignores_mock_config_and_refuses_mock_env(monkeypatch, default_config_for_live=None):
    import pytest
    from inchworm.backends import BackendError, HardwareBackend, live_backend
    from inchworm.config import load_config
    cfg = load_config()  # hardware.yaml says backend: mock
    monkeypatch.delenv("INCHWORM_BACKEND", raising=False)
    assert isinstance(live_backend(cfg), HardwareBackend)
    monkeypatch.setenv("INCHWORM_BACKEND", "mock")
    with pytest.raises(BackendError, match="refusing"):
        live_backend(cfg)


def test_conftest_blocks_real_serial():
    import pytest, serial
    with pytest.raises(AssertionError, match="REAL serial port"):
        serial.Serial("/dev/ttyACM0")


# --- Orchestrator 2026-10-24: PySerialTransport.read must not wait out the timeout ---
def test_pyserial_transport_read_returns_buffered_bytes_without_waiting():
    from inchworm.backends import PySerialTransport

    class FakePort:
        is_open = True
        def __init__(self, data): self.buf = bytearray(data); self.blocking_reads = 0
        @property
        def in_waiting(self): return len(self.buf)
        def read(self, n):
            if n > len(self.buf): self.blocking_reads += 1  # would block until timeout
            out = bytes(self.buf[:n]); del self.buf[:n]; return out

    port = FakePort(b"\xff\xff\x01\x03\x00\x00\xfb")  # 7-byte reply, caller asks for 32
    t = PySerialTransport.__new__(PySerialTransport)
    t.device = "arm_left"; t._port = port
    assert t.read(32) == b"\xff\xff\x01\x03\x00\x00\xfb"
    assert port.blocking_reads == 0
