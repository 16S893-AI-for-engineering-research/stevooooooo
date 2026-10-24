"""vhci_hcd status parser: maps Windows usbipd BusIds to /dev/videoN nodes.

Both Waveshare OV9281 cameras (32e6:9251) report the same USB serial
5423534724 to Linux, so they cannot be identified by serial, VID:PID, or
/dev/v4l/by-id.  The only stable distinguisher is the physical dock port,
encoded as the Windows usbipd BusId (e.g. "9-4", "10-2").

This module resolves a BusId to a /dev/videoN capture node (index 0) by:

  1. Parsing /sys/devices/platform/vhci_hcd.0/status
       Columns: hub port sta spd dev sockfd local_busid
       ``dev`` = (busnum << 16) | devnum of the remote device, hex.
       ``local_busid`` = the Linux USB bus:device path (e.g. "1-5").
     A line "hs  0004 006 003 00090004 000003 1-5" maps remote device
     busnum=9 devnum=4 (BusId "9-4") to local device "1-5".

  2. Reading /sys/bus/usb/devices/<local_busid>/<local_busid>:1.0/video4linux/
     to find the videoN children, then selecting the one with
     /sys/class/video4linux/videoN/index == "0".

  3. Returning "/dev/videoN".

Reference sample from live system (2026-10-24):
  hub port sta spd dev      sockfd local_busid
  hs  0000 006 002 000a0004 000003 1-1     <- 10-4 arm_left
  hs  0001 006 002 00090003 000003 1-2     <- 9-3  arm_right
  hs  0002 006 002 000b0003 000003 1-3     <- 11-3 dexarm
  hs  0003 006 002 00090002 000003 1-4     <- 9-2  relay
  hs  0004 006 003 00090004 000003 1-5     <- 9-4  cam_left
  hs  0005 006 003 000a0002 000003 1-6     <- 10-2 cam_right
  hs  0006 006 003 000b0004 000003 1-7     <- 11-4 cam_scanner
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

# Default paths; overridable in tests.
VHCI_STATUS_PATH = Path("/sys/devices/platform/vhci_hcd.0/status")
USB_DEVICES_DIR = Path("/sys/bus/usb/devices")
VIDEO4LINUX_CLASS_DIR = Path("/sys/class/video4linux")


class VhciParseError(RuntimeError):
    """Raised when the vhci_hcd status file cannot be parsed."""


class BusIdNotFoundError(LookupError):
    """Raised when the requested Windows BusId is not in the vhci status.

    Attributes
    ----------
    windows_busid:
        The BusId that was not found (e.g. ``"9-4"``).
    """

    def __init__(self, windows_busid: str) -> None:
        self.windows_busid = windows_busid
        super().__init__(
            f"Windows BusId {windows_busid!r} not found in vhci_hcd.0/status. "
            "Is the device attached to WSL via usbipd?"
        )


def _busid_to_dev_hex(windows_busid: str) -> str:
    """Convert a Windows BusId string to the hex ``dev`` field in vhci status.

    ``windows_busid`` is "busnum-devnum" (e.g. "9-4").  The ``dev`` column is
    ``(busnum << 16) | devnum`` as an 8-digit lowercase hex string.

    >>> _busid_to_dev_hex("9-4")
    '00090004'
    >>> _busid_to_dev_hex("10-2")
    '000a0002'
    """
    parts = windows_busid.split("-")
    if len(parts) != 2:
        raise ValueError(
            f"Invalid Windows BusId {windows_busid!r}; expected 'busnum-devnum'"
        )
    try:
        busnum = int(parts[0])
        devnum = int(parts[1])
    except ValueError:
        raise ValueError(
            f"Invalid Windows BusId {windows_busid!r}; busnum and devnum must be integers"
        )
    return f"{(busnum << 16) | devnum:08x}"


def _parse_vhci_status(text: str) -> dict[str, str]:
    """Parse the vhci_hcd status file text into a ``{dev_hex: local_busid}`` map.

    Only attached lines (sta == "006") are included; idle ports (sta == "004",
    dev == "00000000") are skipped.

    Parameters
    ----------
    text:
        Content of ``/sys/devices/platform/vhci_hcd.0/status``.

    Returns
    -------
    dict
        Mapping from 8-digit hex ``dev`` column to ``local_busid`` string
        (e.g. ``{"00090004": "1-5", "000a0002": "1-6"}``).
    """
    result: dict[str, str] = {}
    lines = text.strip().splitlines()
    for line in lines:
        # Skip header and blank lines.
        if not line or line.startswith("hub"):
            continue
        parts = line.split()
        # Columns: hub port sta spd dev sockfd local_busid
        if len(parts) < 7:
            continue
        sta = parts[2]
        dev_hex = parts[4].lower()
        local_busid = parts[6]
        # Skip idle/unconnected ports.
        if dev_hex == "00000000" or sta == "004":
            continue
        result[dev_hex] = local_busid
    return result


def _find_capture_node(
    local_busid: str,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    v4l_class_dir: Path = VIDEO4LINUX_CLASS_DIR,
) -> Optional[str]:
    """Find the index-0 video capture node for a local USB device.

    Walks ``/sys/bus/usb/devices/<local_busid>/<local_busid>:1.0/video4linux/``
    and returns the videoN name whose ``/sys/class/video4linux/videoN/index``
    reads as ``"0"``, then maps it to ``/dev/videoN``.

    Returns ``None`` if no video4linux interface is found (device is not a
    camera or is not yet enumerated).
    """
    # Interface 1.0 is the UVC video capture interface for OV9281 cameras.
    iface_path = usb_devices_dir / local_busid / f"{local_busid}:1.0" / "video4linux"
    if not iface_path.exists():
        return None

    for entry in iface_path.iterdir():
        video_name = entry.name  # e.g. "video0", "video2"
        index_path = v4l_class_dir / video_name / "index"
        try:
            index_val = index_path.read_text().strip()
        except OSError:
            continue
        if index_val == "0":
            return f"/dev/{video_name}"

    return None


def resolve_busid_to_video_node(
    windows_busid: str,
    *,
    vhci_status_path: Path = VHCI_STATUS_PATH,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    v4l_class_dir: Path = VIDEO4LINUX_CLASS_DIR,
) -> str:
    """Resolve a Windows usbipd BusId to a ``/dev/videoN`` capture node.

    This is the primary entry point for camera role resolution when
    ``identify_by: usbip_busid`` is set in the hardware config.

    Parameters
    ----------
    windows_busid:
        The BusId from the hardware config (e.g. ``"9-4"``).
    vhci_status_path:
        Override for the vhci_hcd status file path (used in tests).
    usb_devices_dir:
        Override for ``/sys/bus/usb/devices`` (used in tests).
    v4l_class_dir:
        Override for ``/sys/class/video4linux`` (used in tests).

    Returns
    -------
    str
        The ``/dev/videoN`` path for the capture (index 0) node.

    Raises
    ------
    VhciParseError
        If the vhci_hcd status file cannot be read or parsed.
    BusIdNotFoundError
        If the BusId is not present in the vhci status (device not attached).
    LookupError
        If the local sysfs device is found but has no video4linux child at
        index 0 (unexpected enumeration or device is not a camera).
    """
    # Read the vhci status file.
    try:
        text = vhci_status_path.read_text()
    except OSError as err:
        raise VhciParseError(
            f"Cannot read vhci_hcd status at {vhci_status_path}: {err}"
        ) from err

    # Parse the status table.
    try:
        dev_map = _parse_vhci_status(text)
    except Exception as err:
        raise VhciParseError(
            f"Failed to parse vhci_hcd status: {err}"
        ) from err

    # Look up the BusId.
    dev_hex = _busid_to_dev_hex(windows_busid)
    if dev_hex not in dev_map:
        raise BusIdNotFoundError(windows_busid)

    local_busid = dev_map[dev_hex]

    # Find the capture node.
    dev_node = _find_capture_node(local_busid, usb_devices_dir, v4l_class_dir)
    if dev_node is None:
        raise LookupError(
            f"BusId {windows_busid!r} maps to local device {local_busid!r} "
            "but no video4linux index-0 node was found in sysfs. "
            "Is this device a camera and is it fully enumerated?"
        )

    return dev_node
