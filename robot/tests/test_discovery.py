"""Tests for inchworm.discovery: role resolution and error paths via fixtures.

All tests use fixture directories or monkeypatching; no hardware is required.
Role resolution is driven by the config's linux_by_id or windows_busid fields,
never by enumeration order of /dev/ttyACM* or /dev/video* nodes.

cam_left and cam_right use identify_by: usbip_busid; they are resolved via
a fake vhci_hcd.0/status file and fake sysfs trees (no real hardware needed).
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from inchworm.config import load_config
from inchworm.discovery import (
    CamRightAmbiguousError,
    DeviceNotAttachedError,
    DeviceNotFoundError,
    DeviceStatus,
    DuplicateDeviceNodeError,
    DuplicateUsbDeviceError,
    RelayAmbiguousError,
    discover,
    format_human,
    format_json,
    require_device,
)


# ---------------------------------------------------------------------------
# Fixture helpers
# ---------------------------------------------------------------------------


def _make_serial_by_id(
    tmp_path: Path,
    present: dict[str, str],  # {symlink_name: target_basename}
    target_exists: bool = True,
) -> Path:
    """Create a fake /dev/serial/by-id directory under tmp_path."""
    serial_dir = tmp_path / "serial" / "by-id"
    serial_dir.mkdir(parents=True)
    dev_dir = tmp_path / "dev"
    dev_dir.mkdir(exist_ok=True)

    for name, target_base in present.items():
        target = dev_dir / target_base
        if target_exists:
            target.touch()
        (serial_dir / name).symlink_to(target)

    return serial_dir


def _make_v4l_by_id(
    tmp_path: Path,
    present: dict[str, str],
    target_exists: bool = True,
) -> Path:
    """Create a fake /dev/v4l/by-id directory under tmp_path."""
    v4l_dir = tmp_path / "v4l" / "by-id"
    v4l_dir.mkdir(parents=True)
    dev_dir = tmp_path / "dev"
    dev_dir.mkdir(exist_ok=True)

    for name, target_base in present.items():
        target = dev_dir / target_base
        if target_exists:
            target.touch()
        (v4l_dir / name).symlink_to(target)

    return v4l_dir


def _make_serial_by_path(
    tmp_path: Path,
    present: dict,
    target_exists: bool = True,
) -> Path:
    """Create a fake /dev/serial/by-path directory under tmp_path."""
    path_dir = tmp_path / "serial" / "by-path"
    path_dir.mkdir(parents=True, exist_ok=True)
    dev_dir = tmp_path / "dev"
    dev_dir.mkdir(exist_ok=True)
    for name, target_base in present.items():
        target = dev_dir / target_base
        if target_exists:
            target.touch()
        (path_dir / name).symlink_to(target)
    return path_dir


def _make_v4l_by_path(
    tmp_path: Path,
    present: dict,
    target_exists: bool = True,
) -> Path:
    """Create a fake /dev/v4l/by-path directory under tmp_path."""
    path_dir = tmp_path / "v4l" / "by-path"
    path_dir.mkdir(parents=True, exist_ok=True)
    dev_dir = tmp_path / "dev"
    dev_dir.mkdir(exist_ok=True)
    for name, target_base in present.items():
        target = dev_dir / target_base
        if target_exists:
            target.touch()
        (path_dir / name).symlink_to(target)
    return path_dir


def _make_vhci_sysfs(
    tmp_path: Path,
    vhci_status_text: str,
    *,
    cam_left_video: str = "video0",
    cam_right_video: str = "video2",
    cam_scanner_video: str = "video4",
) -> tuple[Path, Path, Path]:
    """Create a minimal fake vhci_hcd sysfs tree.

    Returns (vhci_status_path, usb_devices_dir, v4l_class_dir).

    The returned fake sysfs has:
      - vhci_status_path: a file with vhci_status_text
      - usb_devices_dir/1-5/1-5:1.0/video4linux/{cam_left_video}
      - usb_devices_dir/1-6/1-6:1.0/video4linux/{cam_right_video}
      - usb_devices_dir/1-7/1-7:1.0/video4linux/{cam_scanner_video}  (for scanner, not used by vhci)
      - v4l_class_dir/{videoN}/index with value "0" for capture nodes, "1" for others

    The vhci status maps:
      dev=00090004 (9-4) -> local_busid 1-5  (cam_left)
      dev=000a0002 (10-2) -> local_busid 1-6  (cam_right)
    """
    # vhci status file
    vhci_dir = tmp_path / "sys" / "devices" / "platform" / "vhci_hcd.0"
    vhci_dir.mkdir(parents=True)
    vhci_status_path = vhci_dir / "status"
    vhci_status_path.write_text(vhci_status_text)

    usb_devices_dir = tmp_path / "sys" / "bus" / "usb" / "devices"
    v4l_class_dir = tmp_path / "sys" / "class" / "video4linux"

    # cam_left: local 1-5
    _make_video_iface(usb_devices_dir, "1-5", cam_left_video, v4l_class_dir)
    # cam_right: local 1-6
    _make_video_iface(usb_devices_dir, "1-6", cam_right_video, v4l_class_dir)
    # cam_scanner: local 1-7 (not resolved via vhci but present for completeness)
    _make_video_iface(usb_devices_dir, "1-7", cam_scanner_video, v4l_class_dir)

    return vhci_status_path, usb_devices_dir, v4l_class_dir


def _make_video_iface(
    usb_devices_dir: Path,
    local_busid: str,
    capture_node: str,
    v4l_class_dir: Path,
) -> None:
    """Create sysfs entries for a USB video device at local_busid."""
    iface_path = usb_devices_dir / local_busid / f"{local_busid}:1.0" / "video4linux"
    iface_path.mkdir(parents=True)
    # Orchestrator 2026-10-24: real sysfs exposes idVendor/idProduct; discovery
    # now checks them. Arm cams (1-5, 1-6) are 32e6:9251; the scanner is 0c45:6366.
    vid, pid = ("0c45", "6366") if local_busid == "1-7" else ("32e6", "9251")
    (usb_devices_dir / local_busid / "idVendor").write_text(vid + "\n")
    (usb_devices_dir / local_busid / "idProduct").write_text(pid + "\n")
    # Capture node (index 0)
    (iface_path / capture_node).mkdir()
    _make_v4l_index(v4l_class_dir, capture_node, "0")
    # Metadata node (index 1)
    meta_node = capture_node[:-1] + str(int(capture_node[-1]) + 1)
    (iface_path / meta_node).mkdir()
    _make_v4l_index(v4l_class_dir, meta_node, "1")


def _make_v4l_index(v4l_class_dir: Path, video_name: str, index: str) -> None:
    """Create /sys/class/video4linux/{video_name}/index file."""
    node_dir = v4l_class_dir / video_name
    node_dir.mkdir(parents=True, exist_ok=True)
    (node_dir / "index").write_text(index)


# Canonical vhci status text from the real system (2026-10-24):
VHCI_STATUS_SAMPLE = """\
hub port sta spd dev      sockfd local_busid
hs  0000 006 002 000a0004 000003 1-1
hs  0001 006 002 00090003 000003 1-2
hs  0002 006 002 000b0003 000003 1-3
hs  0003 006 002 00090002 000003 1-4
hs  0004 006 003 00090004 000003 1-5
hs  0005 006 003 000a0002 000003 1-6
hs  0006 006 003 000b0004 000003 1-7
hs  0007 004 000 00000000 000000 0-0
"""


# Symlink names taken directly from hardware.yaml.
_ARM_LEFT_BY_ID = "usb-1a86_USB_Single_Serial_5B91045044-if00"
_ARM_RIGHT_BY_ID = "usb-1a86_USB_Single_Serial_5B91030298-if00"
_DEXARM_BY_ID = "usb-STMicroelectronics_STM32F407ZG_CDC_in_FS_Mode_306133643438-if00"
_RELAY_BY_ID = "usb-1a86_USB_Serial-if00-port0"
_CAM_SCANNER_BY_ID = (
    "usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
)
# cam_left and cam_right are identified by windows_busid, not by-id names.
_CAM_LEFT_BUSID = "9-4"
_CAM_RIGHT_BUSID = "10-2"
_RELAY_BY_PATH = "platform-vhci_hcd.0-usb-0:4:1.0-port0"

_ALL_SERIAL = {
    _ARM_LEFT_BY_ID: "ttyACM1",
    _ARM_RIGHT_BY_ID: "ttyACM0",
    _DEXARM_BY_ID: "ttyACM2",
    _RELAY_BY_ID: "ttyUSB0",
}
_ALL_V4L = {
    _CAM_SCANNER_BY_ID: "video4",
}


@pytest.fixture()
def config():
    return load_config()


# ---------------------------------------------------------------------------
# AC #2: config-driven role resolution; swapped port order does not matter
# ---------------------------------------------------------------------------


class TestRoleResolutionConfigDriven:
    """Role names are derived from config, not enumeration index."""

    def test_all_present_all_attached(self, tmp_path: Path, config):
        """All 7 serial/video devices present → all ATTACHED."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL, target_exists=True)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L, target_exists=True)
        serial_path_dir = _make_serial_by_path(
            tmp_path, {_RELAY_BY_PATH: "ttyUSB0"}, target_exists=True
        )
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            serial_by_path_dir=serial_path_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["arm_left"].status is DeviceStatus.ATTACHED
        assert by_role["arm_right"].status is DeviceStatus.ATTACHED
        assert by_role["dexarm"].status is DeviceStatus.ATTACHED
        assert by_role["relay"].status is DeviceStatus.ATTACHED
        assert by_role["cam_left"].status is DeviceStatus.ATTACHED
        assert by_role["cam_right"].status is DeviceStatus.ATTACHED
        assert by_role["cam_scanner"].status is DeviceStatus.ATTACHED

    def test_roles_resolve_to_correct_by_id_names(self, tmp_path: Path, config):
        """Each role maps to the correct by-id symlink name or windows_busid."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["arm_left"].by_id_name == _ARM_LEFT_BY_ID
        assert by_role["arm_right"].by_id_name == _ARM_RIGHT_BY_ID
        assert by_role["dexarm"].by_id_name == _DEXARM_BY_ID
        assert by_role["relay"].by_id_name == _RELAY_BY_ID
        assert by_role["cam_scanner"].by_id_name == _CAM_SCANNER_BY_ID
        # cam_left and cam_right use windows_busid as the identity key.
        assert by_role["cam_left"].by_id_name == _CAM_LEFT_BUSID
        assert by_role["cam_right"].by_id_name == _CAM_RIGHT_BUSID

    def test_cam_left_resolves_to_video0(self, tmp_path: Path, config):
        """cam_left (BusId 9-4 → local 1-5) resolves to /dev/video0."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["cam_left"].status is DeviceStatus.ATTACHED
        assert by_role["cam_left"].dev_node == "/dev/video0"

    def test_cam_right_resolves_to_video2(self, tmp_path: Path, config):
        """cam_right (BusId 10-2 → local 1-6) resolves to /dev/video2."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["cam_right"].status is DeviceStatus.ATTACHED
        assert by_role["cam_right"].dev_node == "/dev/video2"

    def test_swapped_port_order_roles_unchanged(self, tmp_path: Path, config):
        """When arm_right appears as ttyACM0 and arm_left as ttyACM1 (the live
        observed state), roles must still resolve correctly — arm_left ≠ ttyACM0.
        """
        swapped = {
            _ARM_RIGHT_BY_ID: "ttyACM0",
            _ARM_LEFT_BY_ID: "ttyACM1",
            _DEXARM_BY_ID: "ttyACM2",
            _RELAY_BY_ID: "ttyUSB0",
        }
        serial_dir = _make_serial_by_id(tmp_path, swapped, target_exists=True)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["arm_left"].dev_node is not None
        assert "ttyACM1" in by_role["arm_left"].dev_node
        assert by_role["arm_right"].dev_node is not None
        assert "ttyACM0" in by_role["arm_right"].dev_node

    def test_serial_numbers_in_results(self, tmp_path: Path, config):
        """DeviceResult carries the USB serial from config for each role."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["arm_left"].usb_serial == "5B91045044"
        assert by_role["arm_right"].usb_serial == "5B91030298"
        assert by_role["dexarm"].usb_serial == "306133643438"
        assert by_role["cam_scanner"].usb_serial == "UC762"
        # Both OV9281 cameras have no distinguishable serial; usb_serial is null.
        assert by_role["cam_left"].usb_serial is None
        assert by_role["cam_right"].usb_serial is None
        # relay has no serial
        assert by_role["relay"].usb_serial is None

    def test_cam_left_absent_when_vhci_missing(self, tmp_path: Path, config):
        """cam_left is ABSENT when the vhci_hcd status file does not exist."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        by_role = {r.role: r for r in results}

        assert by_role["cam_left"].status is DeviceStatus.ABSENT
        assert by_role["cam_right"].status is DeviceStatus.ABSENT

    def test_cam_not_attached_when_busid_missing_from_status(self, tmp_path: Path, config):
        """cam_left is NOT_ATTACHED when its BusId is absent from vhci status."""
        # Status with arm devices but no cam_left (9-4 missing)
        status_without_cam_left = """\
hub port sta spd dev      sockfd local_busid
hs  0000 006 002 000a0004 000003 1-1
hs  0001 006 002 00090003 000003 1-2
hs  0003 006 002 00090002 000003 1-4
hs  0005 006 003 000a0002 000003 1-6
hs  0007 004 000 00000000 000000 0-0
"""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, status_without_cam_left,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        assert by_role["cam_left"].status is DeviceStatus.NOT_ATTACHED
        assert by_role["cam_left"].remediation is not None
        assert "usbipd" in by_role["cam_left"].remediation


# ---------------------------------------------------------------------------
# AC #5: missing/unattached device errors name the role and print remediation
# ---------------------------------------------------------------------------


class TestDeviceErrors:
    """Missing and unattached devices produce errors that name the role."""

    def test_absent_device_names_role_in_error(self, tmp_path: Path, config):
        """DeviceNotFoundError message includes the role name."""
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})

        with pytest.raises(DeviceNotFoundError) as exc_info:
            require_device(
                "arm_left",
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
            )
        err = exc_info.value
        assert err.role == "arm_left"
        assert "arm_left" in str(err)

    def test_absent_device_remediation_contains_usbipd(self, tmp_path: Path, config):
        """DeviceNotFoundError.remediation contains an usbipd command."""
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})

        with pytest.raises(DeviceNotFoundError) as exc_info:
            require_device(
                "arm_right",
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
            )
        err = exc_info.value
        assert "usbipd" in err.remediation

    def test_not_attached_error_names_role_and_by_id(self, tmp_path: Path, config):
        """DeviceNotAttachedError names the role and the by-id symlink."""
        serial_dir = _make_serial_by_id(
            tmp_path, {_ARM_LEFT_BY_ID: "ttyACM1"}, target_exists=False
        )
        v4l_dir = _make_v4l_by_id(tmp_path, {})

        with pytest.raises(DeviceNotAttachedError) as exc_info:
            require_device(
                "arm_left",
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
            )
        err = exc_info.value
        assert err.role == "arm_left"
        assert _ARM_LEFT_BY_ID in err.by_id_name
        assert "arm_left" in str(err)

    def test_not_attached_remediation_contains_usbipd_attach(self, tmp_path: Path, config):
        """DeviceNotAttachedError.remediation contains 'usbipd attach'."""
        serial_dir = _make_serial_by_id(
            tmp_path, {_DEXARM_BY_ID: "ttyACM2"}, target_exists=False
        )
        v4l_dir = _make_v4l_by_id(tmp_path, {})

        with pytest.raises(DeviceNotAttachedError) as exc_info:
            require_device(
                "dexarm",
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
            )
        err = exc_info.value
        assert "usbipd attach" in err.remediation

    def test_discover_absent_device_has_remediation(self, tmp_path: Path, config):
        """discover() ABSENT result includes a remediation string."""
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        by_role = {r.role: r for r in results}

        relay_result = by_role["relay"]
        assert relay_result.status is DeviceStatus.ABSENT
        assert relay_result.role == "relay"
        assert relay_result.remediation is not None
        assert "usbipd" in relay_result.remediation

    def test_discover_not_attached_device_has_remediation(self, tmp_path: Path, config):
        """discover() NOT_ATTACHED result has remediation with usbipd."""
        serial_dir = _make_serial_by_id(
            tmp_path, {_DEXARM_BY_ID: "ttyACM2"}, target_exists=False
        )
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        by_role = {r.role: r for r in results}

        dexarm_result = by_role["dexarm"]
        assert dexarm_result.status is DeviceStatus.NOT_ATTACHED
        assert dexarm_result.role == "dexarm"
        assert dexarm_result.remediation is not None
        assert "usbipd attach" in dexarm_result.remediation

    def test_invalid_role_raises_value_error(self, tmp_path: Path, config):
        """require_device with an unknown role raises ValueError."""
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        with pytest.raises(ValueError, match="Unknown device role"):
            require_device(
                "nonexistent_role",
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
            )

    def test_relay_ambiguous_raises_error(self, tmp_path: Path, config):
        """Two CH340 1a86:7523 devices in by-id raises RelayAmbiguousError."""
        two_ch340 = dict(_ALL_SERIAL)
        two_ch340["usb-1a86_USB_Serial-if00-port1"] = "ttyUSB1"
        serial_dir = _make_serial_by_id(tmp_path, two_ch340)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)

        with pytest.raises(RelayAmbiguousError) as exc_info:
            discover(config, serial_by_id_dir=serial_dir, v4l_by_id_dir=v4l_dir)
        err = exc_info.value
        assert err.role == "relay"
        assert len(err.found_entries) == 2
        assert "relay" in str(err)
        assert "safety" in str(err).lower() or "laser" in str(err).lower()

    def test_cam_right_absent_when_busid_not_in_vhci(self, tmp_path: Path, config):
        """cam_right is NOT_ATTACHED when its BusId is absent from vhci status."""
        # Status without cam_right (10-2 missing)
        status_no_cam_right = """\
hub port sta spd dev      sockfd local_busid
hs  0004 006 003 00090004 000003 1-5
hs  0007 004 000 00000000 000000 0-0
"""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, status_no_cam_right,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        by_role = {r.role: r for r in results}

        r = by_role["cam_right"]
        assert r.status is DeviceStatus.NOT_ATTACHED
        assert r.remediation is not None


# ---------------------------------------------------------------------------
# Global duplicate invariant: DuplicateDeviceNodeError and DuplicateUsbDeviceError
# ---------------------------------------------------------------------------


class TestDuplicateInvariant:
    """Discovery raises if two roles resolve to the same dev node or USB device."""

    def test_duplicate_dev_node_raises_error(self, tmp_path: Path, config):
        """If two cameras (cam_left and cam_right) resolve to the same /dev/videoN,
        DuplicateDeviceNodeError is raised.
        """
        # Give both cam_left (1-5) and cam_right (1-6) the same video node.
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)

        # Make a vhci tree where 1-5 and 1-6 both claim video0.
        vhci_dir = tmp_path / "dup" / "vhci"
        vhci_dir.mkdir(parents=True)
        vhci_status_path = vhci_dir / "status"
        vhci_status_path.write_text(VHCI_STATUS_SAMPLE)

        usb_dev_dir = tmp_path / "dup" / "usb"
        v4l_class_dir = tmp_path / "dup" / "v4l_class"

        # Both 1-5 and 1-6 map to video0 (same node — collision!)
        _make_video_iface(usb_dev_dir, "1-5", "video0", v4l_class_dir)
        _make_video_iface(usb_dev_dir, "1-6", "video0", v4l_class_dir)
        _make_video_iface(usb_dev_dir, "1-7", "video4", v4l_class_dir)

        with pytest.raises(DuplicateDeviceNodeError) as exc_info:
            discover(
                config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
                vhci_status_path=vhci_status_path,
                usb_devices_dir=usb_dev_dir,
                v4l_class_dir=v4l_class_dir,
            )
        err = exc_info.value
        assert err.node == "/dev/video0"
        assert "cam_left" in err.roles
        assert "cam_right" in err.roles

    def test_duplicate_usb_device_raises_error(self, tmp_path: Path, config):
        """If two roles have the same windows_busid (cam_left and cam_right both 9-4),
        DuplicateUsbDeviceError is raised.
        """
        import copy
        import yaml
        from inchworm.config import parse_config, DEFAULT_CONFIG_PATH

        raw = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
        # Set cam_right to the same BusId as cam_left (9-4).
        raw["cameras"]["right_arm"]["identity"]["windows_busid"] = "9-4"
        bad_config = parse_config(raw)

        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        with pytest.raises((DuplicateDeviceNodeError, DuplicateUsbDeviceError)):
            discover(
                bad_config,
                serial_by_id_dir=serial_dir,
                v4l_by_id_dir=v4l_dir,
                vhci_status_path=vhci_status,
                usb_devices_dir=usb_dev_dir,
                v4l_class_dir=v4l_class,
            )

    def test_no_duplicate_when_all_roles_distinct(self, tmp_path: Path, config):
        """Normal configuration with all distinct nodes passes without error."""
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        # Should not raise.
        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        assert len(results) == 7


# ---------------------------------------------------------------------------
# AC #6: machine-readable JSON output
# ---------------------------------------------------------------------------


class TestJsonOutput:
    """JSON output is well-formed and contains all expected fields."""

    def test_json_output_is_valid_json(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        output = format_json(results)
        parsed = json.loads(output)
        assert isinstance(parsed, list)

    def test_json_contains_all_seven_roles(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        parsed = json.loads(format_json(results))
        roles = {item["role"] for item in parsed}
        expected = {"arm_left", "arm_right", "dexarm", "relay", "cam_left", "cam_right", "cam_scanner"}
        assert roles == expected

    def test_json_entry_has_required_fields(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        parsed = json.loads(format_json(results))
        required_fields = {"role", "status", "by_id_name", "dev_node", "vid_pid", "usb_serial", "remediation"}
        for entry in parsed:
            missing = required_fields - entry.keys()
            assert not missing, f"Entry for {entry.get('role')} is missing fields: {missing}"

    def test_json_status_values_are_strings(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        parsed = json.loads(format_json(results))
        valid_statuses = {"attached", "not_attached", "absent", "no_udev_rule"}
        for entry in parsed:
            assert entry["status"] in valid_statuses

    def test_json_attached_device_has_dev_node(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        parsed = json.loads(format_json(results))
        by_role = {item["role"]: item for item in parsed}
        for role in ("arm_left", "arm_right", "dexarm", "relay", "cam_scanner", "cam_left", "cam_right"):
            assert by_role[role]["status"] == "attached"
            assert by_role[role]["dev_node"] is not None

    def test_json_absent_device_has_remediation(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        parsed = json.loads(format_json(results))
        by_role = {item["role"]: item for item in parsed}
        for role in ("arm_left", "arm_right", "dexarm", "relay"):
            assert by_role[role]["status"] == "absent"
            assert by_role[role]["remediation"] is not None
            assert "usbipd" in by_role[role]["remediation"]


# ---------------------------------------------------------------------------
# AC #1: discovery command output contains all roles
# ---------------------------------------------------------------------------


class TestHumanOutput:
    """Human-readable output mentions all expected roles."""

    def test_human_output_contains_all_roles(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        output = format_human(results)
        for role in ("arm_left", "arm_right", "dexarm", "relay", "cam_left", "cam_right", "cam_scanner"):
            assert role in output

    def test_human_output_contains_by_id_names(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        output = format_human(results)
        assert _ARM_LEFT_BY_ID in output
        assert _ARM_RIGHT_BY_ID in output

    def test_human_output_shows_attached_status(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        v4l_dir = _make_v4l_by_id(tmp_path, _ALL_V4L)
        vhci_status, usb_dev_dir, v4l_class = _make_vhci_sysfs(
            tmp_path, VHCI_STATUS_SAMPLE,
        )

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=vhci_status,
            usb_devices_dir=usb_dev_dir,
            v4l_class_dir=v4l_class,
        )
        output = format_human(results)
        assert "attached" in output

    def test_human_output_shows_remediation_for_absent(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, {})
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        output = format_human(results)
        assert "usbipd" in output


# ---------------------------------------------------------------------------
# AC #7: no-by-id-dir graceful degradation
# ---------------------------------------------------------------------------


class TestGracefulDegradation:
    """When the by-id directory doesn't exist, all serial roles show ABSENT."""

    def test_missing_serial_dir_all_absent(self, tmp_path: Path, config):
        nonexistent = tmp_path / "does_not_exist"
        v4l_dir = _make_v4l_by_id(tmp_path, {})
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=nonexistent,
            v4l_by_id_dir=v4l_dir,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        by_role = {r.role: r for r in results}

        for role in ("arm_left", "arm_right", "dexarm", "relay"):
            assert by_role[role].status is DeviceStatus.ABSENT

    def test_missing_v4l_dir_cam_scanner_absent(self, tmp_path: Path, config):
        serial_dir = _make_serial_by_id(tmp_path, _ALL_SERIAL)
        nonexistent = tmp_path / "no_v4l"
        nonexistent_vhci = tmp_path / "no_vhci_status"

        results = discover(
            config,
            serial_by_id_dir=serial_dir,
            v4l_by_id_dir=nonexistent,
            vhci_status_path=nonexistent_vhci,
            usb_devices_dir=tmp_path / "no_usb",
            v4l_class_dir=tmp_path / "no_v4l_class",
        )
        by_role = {r.role: r for r in results}

        assert by_role["cam_scanner"].status is DeviceStatus.ABSENT



# --- Orchestrator 2026-10-24: a different device on the camera's dock port is refused ---
def test_busid_holding_wrong_device_is_refused(tmp_path):
    import pytest
    from inchworm.vhci import resolve_busid_to_video_node
    status = ("hub port sta spd dev      sockfd local_busid\n"
              "hs  0004 006 003 00090004 000003 1-5\n")
    vhci, usb, v4l = _make_vhci_sysfs(tmp_path, status)
    (usb / "1-5" / "idVendor").write_text("1a86\n")
    (usb / "1-5" / "idProduct").write_text("7523\n")
    with pytest.raises(LookupError, match="dock port changed"):
        resolve_busid_to_video_node("9-4", expected_vid_pid="32e6:9251",
                                    vhci_status_path=vhci, usb_devices_dir=usb, v4l_class_dir=v4l)
