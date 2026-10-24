"""Tests for inchworm.vhci: vhci_hcd status parser and video node resolver.

All tests use fixture files; no hardware is required.

Tests cover:
  - vhci status parser with the real sample from the live system
  - _busid_to_dev_hex conversion
  - resolve_busid_to_video_node with fake sysfs
  - BusIdNotFoundError for missing BusId
  - VhciParseError for unreadable status file
  - Video node not found (device in vhci but no video4linux sysfs child)
"""

from __future__ import annotations

from pathlib import Path

import pytest

from inchworm.vhci import (
    BusIdNotFoundError,
    VhciParseError,
    _busid_to_dev_hex,
    _parse_vhci_status,
    resolve_busid_to_video_node,
)


# Real sample from the live system (2026-10-24).
# dev column: (busnum << 16) | devnum, hex.
# hs  0004 006 003 00090004 000003 1-5  <- BusId 9-4  -> cam_left
# hs  0005 006 003 000a0002 000003 1-6  <- BusId 10-2 -> cam_right
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
ss  0008 004 000 00000000 000000 0-0
"""


# ---------------------------------------------------------------------------
# _busid_to_dev_hex
# ---------------------------------------------------------------------------

class TestBusidToDevHex:
    def test_busid_9_4(self):
        """BusId 9-4 -> (9 << 16) | 4 = 0x00090004."""
        assert _busid_to_dev_hex("9-4") == "00090004"

    def test_busid_10_2(self):
        """BusId 10-2 -> (10 << 16) | 2 = 0x000a0002."""
        assert _busid_to_dev_hex("10-2") == "000a0002"

    def test_busid_10_4(self):
        """BusId 10-4 -> 0x000a0004."""
        assert _busid_to_dev_hex("10-4") == "000a0004"

    def test_busid_11_4(self):
        """BusId 11-4 -> 0x000b0004."""
        assert _busid_to_dev_hex("11-4") == "000b0004"

    def test_busid_9_2(self):
        """BusId 9-2 -> 0x00090002."""
        assert _busid_to_dev_hex("9-2") == "00090002"

    def test_invalid_busid_raises_value_error(self):
        with pytest.raises(ValueError, match="Invalid Windows BusId"):
            _busid_to_dev_hex("9")  # no dash

    def test_invalid_busid_non_integer_raises(self):
        with pytest.raises(ValueError, match="Invalid Windows BusId"):
            _busid_to_dev_hex("9-x")


# ---------------------------------------------------------------------------
# _parse_vhci_status
# ---------------------------------------------------------------------------

class TestParseVhciStatus:
    def test_parses_all_attached_lines(self):
        """Attached lines (sta=006) are all parsed; idle lines (sta=004) skipped."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        # 7 attached devices (sta=006), 2 idle (sta=004, dev=00000000)
        assert len(result) == 7

    def test_cam_left_busid_mapped(self):
        """BusId 9-4 (dev=00090004) maps to local 1-5."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        assert result["00090004"] == "1-5"

    def test_cam_right_busid_mapped(self):
        """BusId 10-2 (dev=000a0002) maps to local 1-6."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        assert result["000a0002"] == "1-6"

    def test_arm_left_busid_mapped(self):
        """BusId 10-4 (dev=000a0004) maps to local 1-1."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        assert result["000a0004"] == "1-1"

    def test_idle_ports_excluded(self):
        """Ports with dev=00000000 are not in the result."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        assert "00000000" not in result

    def test_header_line_skipped(self):
        """Header line is skipped gracefully."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        # No entry for the literal string "hub"
        assert "hub" not in result

    def test_empty_status_returns_empty_dict(self):
        assert _parse_vhci_status("") == {}
        assert _parse_vhci_status("hub port sta spd dev sockfd local_busid\n") == {}

    def test_cam_scanner_busid_mapped(self):
        """BusId 11-4 (dev=000b0004) maps to local 1-7."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        assert result["000b0004"] == "1-7"

    def test_both_cameras_in_same_status(self):
        """Both OV9281 cameras (sharing serial 5423534724) appear at distinct local busids."""
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        # cam_left = 9-4 -> 1-5; cam_right = 10-2 -> 1-6 — different local busids.
        assert result["00090004"] == "1-5"
        assert result["000a0002"] == "1-6"
        assert result["00090004"] != result["000a0002"]

    def test_colliding_serials_not_a_problem_for_vhci(self):
        """vhci parser does not care about USB serials; BusIds are distinct."""
        # Both cameras report serial 5423534724 to Linux but the vhci table
        # maps them to distinct local busids via their physical dock ports.
        result = _parse_vhci_status(VHCI_STATUS_SAMPLE)
        local_cam_left = result["00090004"]
        local_cam_right = result["000a0002"]
        assert local_cam_left != local_cam_right, (
            "Colliding USB serials must not cause vhci to map both cameras to the same local busid"
        )


# ---------------------------------------------------------------------------
# resolve_busid_to_video_node with fake sysfs
# ---------------------------------------------------------------------------

def _make_sysfs(tmp_path: Path, vhci_text: str, mapping: dict) -> tuple:
    """Create a minimal fake sysfs for resolve_busid_to_video_node tests.

    mapping: {local_busid: (capture_video_name, index_value)}
    """
    vhci_dir = tmp_path / "sys" / "vhci"
    vhci_dir.mkdir(parents=True)
    vhci_path = vhci_dir / "status"
    vhci_path.write_text(vhci_text)

    usb_dir = tmp_path / "sys" / "usb"
    v4l_dir = tmp_path / "sys" / "v4l"

    for local_busid, (video_name, index_val) in mapping.items():
        iface_path = usb_dir / local_busid / f"{local_busid}:1.0" / "video4linux"
        iface_path.mkdir(parents=True)
        (iface_path / video_name).mkdir()
        vnode_dir = v4l_dir / video_name
        vnode_dir.mkdir(parents=True)
        (vnode_dir / "index").write_text(index_val)

    return vhci_path, usb_dir, v4l_dir


class TestResolveBusidToVideoNode:
    def test_cam_left_resolves_to_video0(self, tmp_path: Path):
        """BusId 9-4 resolves to /dev/video0."""
        vhci_path, usb_dir, v4l_dir = _make_sysfs(
            tmp_path,
            VHCI_STATUS_SAMPLE,
            {"1-5": ("video0", "0"), "1-6": ("video2", "0")},
        )
        result = resolve_busid_to_video_node(
            "9-4", vhci_status_path=vhci_path, usb_devices_dir=usb_dir, v4l_class_dir=v4l_dir
        )
        assert result == "/dev/video0"

    def test_cam_right_resolves_to_video2(self, tmp_path: Path):
        """BusId 10-2 resolves to /dev/video2."""
        vhci_path, usb_dir, v4l_dir = _make_sysfs(
            tmp_path,
            VHCI_STATUS_SAMPLE,
            {"1-5": ("video0", "0"), "1-6": ("video2", "0")},
        )
        result = resolve_busid_to_video_node(
            "10-2", vhci_status_path=vhci_path, usb_devices_dir=usb_dir, v4l_class_dir=v4l_dir
        )
        assert result == "/dev/video2"

    def test_missing_busid_raises_not_found(self, tmp_path: Path):
        """BusId not in vhci status raises BusIdNotFoundError."""
        vhci_path, usb_dir, v4l_dir = _make_sysfs(
            tmp_path,
            VHCI_STATUS_SAMPLE,
            {"1-5": ("video0", "0")},
        )
        with pytest.raises(BusIdNotFoundError) as exc_info:
            resolve_busid_to_video_node(
                "5-1",  # not in sample
                vhci_status_path=vhci_path,
                usb_devices_dir=usb_dir,
                v4l_class_dir=v4l_dir,
            )
        assert exc_info.value.windows_busid == "5-1"

    def test_missing_vhci_file_raises_parse_error(self, tmp_path: Path):
        """Non-existent vhci status file raises VhciParseError."""
        with pytest.raises(VhciParseError):
            resolve_busid_to_video_node(
                "9-4",
                vhci_status_path=tmp_path / "missing_status",
                usb_devices_dir=tmp_path / "usb",
                v4l_class_dir=tmp_path / "v4l",
            )

    def test_busid_found_but_no_video_node_raises_lookup_error(self, tmp_path: Path):
        """BusId present in vhci status but no video4linux child raises LookupError."""
        vhci_dir = tmp_path / "vhci"
        vhci_dir.mkdir()
        vhci_path = vhci_dir / "status"
        vhci_path.write_text(VHCI_STATUS_SAMPLE)

        # 1-5 has no video4linux directory.
        usb_dir = tmp_path / "usb"
        v4l_dir = tmp_path / "v4l"
        usb_dir.mkdir()
        v4l_dir.mkdir()

        with pytest.raises(LookupError):
            resolve_busid_to_video_node(
                "9-4",
                vhci_status_path=vhci_path,
                usb_devices_dir=usb_dir,
                v4l_class_dir=v4l_dir,
            )

    def test_index0_node_selected_over_index1(self, tmp_path: Path):
        """When both index 0 and index 1 nodes exist, index 0 is returned."""
        vhci_dir = tmp_path / "vhci"
        vhci_dir.mkdir()
        vhci_path = vhci_dir / "status"
        vhci_path.write_text(VHCI_STATUS_SAMPLE)

        usb_dir = tmp_path / "usb"
        v4l_dir = tmp_path / "v4l"

        local_busid = "1-5"
        iface_path = usb_dir / local_busid / f"{local_busid}:1.0" / "video4linux"
        iface_path.mkdir(parents=True)
        (iface_path / "video0").mkdir()
        (iface_path / "video1").mkdir()

        (v4l_dir / "video0").mkdir(parents=True)
        (v4l_dir / "video0" / "index").write_text("0")
        (v4l_dir / "video1").mkdir(parents=True)
        (v4l_dir / "video1" / "index").write_text("1")

        result = resolve_busid_to_video_node(
            "9-4",
            vhci_status_path=vhci_path,
            usb_devices_dir=usb_dir,
            v4l_class_dir=v4l_dir,
        )
        assert result == "/dev/video0"

    def test_different_enumeration_order_both_resolve_correctly(self, tmp_path: Path):
        """If cam_right enumerates before cam_left in /dev, BusIds still resolve correctly.

        Real concern: video2 might be cam_left and video0 might be cam_right depending
        on attach order. BusId-based resolution is invariant to enumeration order.
        """
        vhci_path, usb_dir, v4l_dir = _make_sysfs(
            tmp_path,
            VHCI_STATUS_SAMPLE,
            # Swapped: cam_left at video2, cam_right at video0
            {"1-5": ("video2", "0"), "1-6": ("video0", "0")},
        )
        cam_left = resolve_busid_to_video_node(
            "9-4", vhci_status_path=vhci_path, usb_devices_dir=usb_dir, v4l_class_dir=v4l_dir
        )
        cam_right = resolve_busid_to_video_node(
            "10-2", vhci_status_path=vhci_path, usb_devices_dir=usb_dir, v4l_class_dir=v4l_dir
        )
        # cam_left must be video2 and cam_right must be video0 in this swapped scenario.
        assert cam_left == "/dev/video2"
        assert cam_right == "/dev/video0"
        assert cam_left != cam_right
