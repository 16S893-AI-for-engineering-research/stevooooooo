"""Tests for inchworm.config: loading the real hardware.yaml and rejecting bad configs."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from inchworm.config import (
    DEFAULT_CONFIG_PATH,
    BackendName,
    ConfigError,
    HardwareConfig,
    IdentifyBy,
    load_config,
    parse_config,
)
from inchworm.units import UNIT_SUFFIXES


@pytest.fixture()
def raw_config() -> dict:
    return yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))


def test_load_default_config_success():
    config = load_config()
    assert isinstance(config, HardwareConfig)
    assert config.schema_version == 1
    assert config.backend is BackendName.MOCK


def test_config_describes_every_device_with_identity():
    config = load_config()
    identities = {
        "arm_left": config.arms.left.identity,
        "arm_right": config.arms.right.identity,
        "dexarm": config.dexarm.identity,
        "relay": config.relay.identity,
        "camera_left_arm": config.cameras.left_arm.identity,
        "camera_right_arm": config.cameras.right_arm.identity,
        "camera_scanner": config.cameras.scanner.identity,
    }
    assert {k: v.vid_pid for k, v in identities.items()} == {
        "arm_left": "1a86:55d3",
        "arm_right": "1a86:55d3",
        "dexarm": "0483:5740",
        "relay": "1a86:7523",
        "camera_left_arm": "32e6:9251",
        "camera_right_arm": "32e6:9251",
        "camera_scanner": "0c45:6366",
    }
    assert config.arms.left.identity.usb_serial == "5B91045044"
    assert config.arms.right.identity.usb_serial == "5B91030298"
    assert config.dexarm.identity.usb_serial == "306133643438"
    # Both OV9281 cameras share serial 5423534724; neither is identified by serial.
    assert config.cameras.left_arm.identity.usb_serial is None
    assert config.cameras.left_arm.identity.identify_by is IdentifyBy.USBIP_BUSID
    assert config.cameras.left_arm.identity.windows_busid == "9-4"
    assert config.cameras.scanner.identity.usb_serial == "UC762"
    # Devices without a USB serial must be identified positionally.
    assert config.relay.identity.usb_serial is None
    assert config.relay.identity.identify_by is IdentifyBy.VID_PID
    assert config.cameras.right_arm.identity.usb_serial is None
    assert config.cameras.right_arm.identity.identify_by is IdentifyBy.USBIP_BUSID
    assert config.cameras.right_arm.identity.windows_busid == "10-2"
    assert config.laser.switched_by == "relay"


def test_serial_identities_are_unique_per_vid_pid():
    config = load_config()
    keys = [
        (i.vid_pid, i.usb_serial)
        for i in (
            config.arms.left.identity,
            config.arms.right.identity,
            config.dexarm.identity,
            config.cameras.left_arm.identity,
            config.cameras.scanner.identity,
        )
    ]
    assert len(keys) == len(set(keys))


def test_load_config_from_explicit_path(tmp_path: Path, raw_config: dict):
    path = tmp_path / "hw.yaml"
    path.write_text(yaml.safe_dump(raw_config), encoding="utf-8")
    assert load_config(path) == load_config()


def test_load_config_honours_env_var(tmp_path: Path, raw_config: dict, monkeypatch):
    raw_config["backend"] = "hardware"
    path = tmp_path / "hw.yaml"
    path.write_text(yaml.safe_dump(raw_config), encoding="utf-8")
    monkeypatch.setenv("INCHWORM_CONFIG", str(path))
    assert load_config().backend is BackendName.HARDWARE


def test_unknown_top_level_field_rejected(raw_config: dict):
    raw_config["lidar"] = {"port": "/dev/ttyUSB9"}
    with pytest.raises(ConfigError, match=r"lidar: unknown field"):
        parse_config(raw_config)


def test_unknown_nested_field_rejected(raw_config: dict):
    raw_config["arms"]["left"]["identity"]["bus_id"] = "10-4"
    with pytest.raises(ConfigError, match=r"arms\.left\.identity\.bus_id: unknown field"):
        parse_config(raw_config)


@pytest.mark.parametrize(
    "path",
    [
        ("geometry", "attachment_pitch_mm"),
        ("arms", "right"),
        ("dexarm", "identity", "usb_serial"),
        ("cameras", "scanner"),
        ("relay",),
        ("laser", "design", "standoff_mm"),
        ("usb", "max_concurrent_camera_streams"),
        ("schema_version",),
    ],
)
def test_missing_required_field_rejected(raw_config: dict, path: tuple):
    data = copy.deepcopy(raw_config)
    node = data
    for key in path[:-1]:
        node = node[key]
    del node[path[-1]]
    dotted = r"\.".join(path)
    with pytest.raises(ConfigError, match=dotted + ": missing required field"):
        parse_config(data)


def test_unsupported_schema_version_rejected(raw_config: dict):
    raw_config["schema_version"] = 2
    with pytest.raises(ConfigError, match=r"unsupported schema_version 2"):
        parse_config(raw_config)


def test_second_concurrent_camera_stream_rejected(raw_config: dict):
    raw_config["usb"]["max_concurrent_camera_streams"] = 2
    with pytest.raises(ConfigError, match=r"usb\.max_concurrent_camera_streams"):
        parse_config(raw_config)


def test_wrong_type_rejected(raw_config: dict):
    raw_config["geometry"]["attachment_pitch_mm"] = "183.676"
    with pytest.raises(ConfigError, match=r"geometry\.attachment_pitch_mm"):
        parse_config(raw_config)


def test_identify_by_serial_without_serial_rejected(raw_config: dict):
    raw_config["dexarm"]["identity"]["usb_serial"] = None
    with pytest.raises(ConfigError, match=r"usb_serial is null"):
        parse_config(raw_config)


def test_bad_vendor_id_rejected(raw_config: dict):
    raw_config["relay"]["identity"]["vendor_id"] = "VID_1A86"
    with pytest.raises(ConfigError, match=r"relay\.identity\.vendor_id"):
        parse_config(raw_config)


def test_unknown_backend_rejected(raw_config: dict):
    raw_config["backend"] = "simulation"
    with pytest.raises(ConfigError, match=r"backend"):
        parse_config(raw_config)


def test_non_mapping_and_unreadable_files_rejected(tmp_path: Path):
    with pytest.raises(ConfigError, match="top level must be a mapping"):
        parse_config(["not", "a", "mapping"])
    with pytest.raises(ConfigError, match="cannot read hardware config"):
        load_config(tmp_path / "missing.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("geometry: [unclosed", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(bad)


def _float_fields(model: type) -> list:
    """Every float-typed field name in the schema, recursively."""
    from pydantic import BaseModel

    names = []
    for name, field in model.model_fields.items():
        ann = field.annotation
        if isinstance(ann, type) and issubclass(ann, BaseModel):
            names.extend(_float_fields(ann))
        elif ann is float:
            names.append(name)
    return names


def test_dimensional_fields_carry_unit_suffix():
    names = _float_fields(HardwareConfig)
    assert names, "expected float fields in the schema"
    for name in names:
        assert name.endswith(UNIT_SUFFIXES), f"{name} lacks a unit suffix {UNIT_SUFFIXES}"
