"""Tests for inchworm.backends: mock-backend selection and zero-hardware transports."""

from __future__ import annotations

import pytest

from inchworm.backends import (
    BACKEND_ENV_VAR,
    BackendError,
    HardwareBackend,
    MockBackend,
    create_backend,
    device_table,
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


def test_hardware_backend_refuses_to_open_devices(config):
    backend = HardwareBackend(config)
    with pytest.raises(BackendError, match="not implemented yet"):
        backend.open_serial("arm_left")
    # --- AA-14 ---
    # HardwareBackend._open_camera now raises BackendError with a different
    # message directing callers to V4L2CameraTransport.open().
    with pytest.raises(BackendError):
        backend.open_camera("camera_scanner")
    # --- AA-14 end ---
