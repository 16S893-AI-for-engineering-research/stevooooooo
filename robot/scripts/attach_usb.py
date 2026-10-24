#!/usr/bin/env python3
"""WSL USB attach helper for the inchworm print-crawl robot.

Resolves each required device's Windows BusId from ``usbipd state`` JSON
(keyed on VID:PID + USB serial where available, VID:PID alone for port-path
devices), then calls::

    powershell.exe -NoProfile -Command "usbipd attach --wsl --busid X"

for every device.  Reports per-device success or failure and exits nonzero if
any required device is missing from the Windows device list or fails to attach.

BusIds are never hardcoded; they are resolved fresh from ``usbipd state`` each
run because usbipd BusIds change across reconnects.

Usage
-----
    cd robot
    uv run python scripts/attach_usb.py [--dry-run] [--config PATH]

Options
-------
--dry-run    Resolve BusIds and print the attach commands without executing them.
--config     Path to hardware.yaml (default: robot/config/hardware.yaml).
--state-json Path to a JSON file containing the output of ``usbipd state``
             (for testing without a Windows host).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Allow running directly from the repository root without installing the package.
_HERE = Path(__file__).resolve()
_ROBOT_ROOT = _HERE.parents[1]
sys.path.insert(0, str(_ROBOT_ROOT / "src"))

from inchworm.config import HardwareConfig, load_config  # noqa: E402


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class _DeviceSpec:
    """The properties needed to find a device in ``usbipd state`` output."""

    role: str
    """Logical role name (e.g. ``arm_left``)."""

    vid: str
    """4-hex-digit vendor ID, lower-case (e.g. ``1a86``)."""

    pid: str
    """4-hex-digit product ID, lower-case (e.g. ``55d3``)."""

    usb_serial: Optional[str]
    """USB serial number from hardware.yaml, or ``None`` for port-path devices."""


@dataclass
class _AttachResult:
    """Outcome of one attach attempt."""

    role: str
    busid: Optional[str]
    """Windows BusId resolved from usbipd state, or None if not found."""

    found: bool
    """True if the device was present in the usbipd device list."""

    already_attached: bool
    """True if usbipd reported the device as already attached to WSL."""

    success: bool
    """True if the attach command succeeded (or device was already attached)."""

    message: str
    """Human-readable status message."""


# ---------------------------------------------------------------------------
# usbipd state parsing
# ---------------------------------------------------------------------------


def _get_usbipd_state_json(state_json_path: Optional[Path] = None) -> dict:
    """Return the parsed JSON from ``usbipd state``.

    Parameters
    ----------
    state_json_path:
        If provided, read JSON from this file instead of running powershell.
        Used for offline testing.
    """
    if state_json_path is not None:
        text = state_json_path.read_text(encoding="utf-8")
        return json.loads(text)

    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-Command", "usbipd state"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"'usbipd state' failed (exit {result.returncode}):\n{result.stderr}"
        )
    return json.loads(result.stdout)


def _parse_instance_id(instance_id: str) -> tuple[str, str, Optional[str]]:
    """Extract (vid, pid, serial_or_none) from a usbipd InstanceId string.

    InstanceId format (from Windows USB device tree):
      ``USB\\VID_XXXX&PID_YYYY\\SERIAL``

    The third component is the USB serial number when the device reports one.
    For devices without a USB serial the third component is a Windows-internal
    path segment (e.g. ``7&165AD49D&0&2``).  A Windows-internal path segment
    always begins with a digit followed by ``&``; a real serial does not match
    that pattern.

    Returns
    -------
    (vid, pid, usb_serial_or_none)
        All lower-case.  ``usb_serial_or_none`` is ``None`` when the third
        component looks like a Windows path segment rather than a real serial.
    """
    parts = instance_id.replace("\\\\", "\\").split("\\")
    # parts[0] = "USB", parts[1] = "VID_XXXX&PID_YYYY", parts[2] = serial or path
    if len(parts) < 2:
        return "", "", None

    vid_pid = parts[1]  # "VID_1A86&PID_55D3"
    vid_part, _, pid_part = vid_pid.partition("&")
    vid = vid_part.removeprefix("VID_").lower()
    pid = pid_part.removeprefix("PID_").lower()

    usb_serial: Optional[str] = None
    if len(parts) >= 3:
        raw_serial = parts[2]
        # Windows-internal path segments look like "7&165AD49D&0&2" — they
        # start with a digit followed immediately by "&".
        if len(raw_serial) > 1 and raw_serial[0].isdigit() and raw_serial[1] == "&":
            usb_serial = None  # port-path device
        else:
            usb_serial = raw_serial  # real USB serial

    return vid, pid, usb_serial


def _resolve_busid(
    spec: _DeviceSpec,
    devices: list[dict],
) -> Optional[str]:
    """Return the BusId for *spec* from the usbipd device list, or None.

    Matching strategy:
    - If spec has a usb_serial: match VID + PID + serial (case-insensitive).
    - If spec has no usb_serial (port-path device): match VID + PID only.
      If multiple entries share the same VID:PID (e.g. two openaicam cameras),
      the first one without a real serial is chosen.
    """
    target_vid = spec.vid.lower()
    target_pid = spec.pid.lower()
    target_serial = spec.usb_serial.upper() if spec.usb_serial else None

    for dev in devices:
        instance_id = dev.get("InstanceId", "")
        vid, pid, serial = _parse_instance_id(instance_id)

        if vid != target_vid or pid != target_pid:
            continue

        if target_serial is not None:
            # Must match the serial exactly (case-insensitive).
            if serial and serial.upper() == target_serial:
                return dev["BusId"]
        else:
            # Port-path device: match first entry with no real serial for this VID:PID.
            if serial is None:
                return dev["BusId"]

    return None


def _is_attached(busid: str, devices: list[dict]) -> bool:
    """Return True if the device with *busid* already has a WSL client IP."""
    for dev in devices:
        if dev.get("BusId") == busid:
            return dev.get("ClientIPAddress") is not None
    return False


# ---------------------------------------------------------------------------
# Device spec extraction from hardware config
# ---------------------------------------------------------------------------


def _device_specs_from_config(config: HardwareConfig) -> list[_DeviceSpec]:
    """Return the list of device specs for all required devices."""
    raw = [
        ("arm_left", config.arms.left.identity),
        ("arm_right", config.arms.right.identity),
        ("dexarm", config.dexarm.identity),
        ("relay", config.relay.identity),
        ("cam_left", config.cameras.left_arm.identity),
        ("cam_right", config.cameras.right_arm.identity),
        ("cam_scanner", config.cameras.scanner.identity),
    ]
    return [
        _DeviceSpec(
            role=role,
            vid=identity.vendor_id.lower(),
            pid=identity.product_id.lower(),
            usb_serial=identity.usb_serial,
        )
        for role, identity in raw
    ]


# ---------------------------------------------------------------------------
# Attach logic
# ---------------------------------------------------------------------------


def _attach_device(
    spec: _DeviceSpec,
    devices: list[dict],
    dry_run: bool,
) -> _AttachResult:
    """Resolve BusId and attach one device.  Returns an :class:`_AttachResult`."""
    busid = _resolve_busid(spec, devices)

    if busid is None:
        return _AttachResult(
            role=spec.role,
            busid=None,
            found=False,
            already_attached=False,
            success=False,
            message=(
                f"[MISSING] {spec.role}: not found in usbipd device list "
                f"(VID:PID {spec.vid}:{spec.pid}, serial={spec.usb_serial!r}). "
                f"Ensure the device is plugged into Windows."
            ),
        )

    if _is_attached(busid, devices):
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=True,
            success=True,
            message=f"[OK]      {spec.role}: already attached (BusId {busid})",
        )

    cmd = ["powershell.exe", "-NoProfile", "-Command", f"usbipd attach --wsl --busid {busid}"]

    if dry_run:
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=True,  # dry-run counts as would-succeed
            message=f"[DRY-RUN] {spec.role}: would attach BusId {busid} — {' '.join(cmd[3:])}",
        )

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=True,
            message=f"[OK]      {spec.role}: attached BusId {busid}",
        )
    else:
        stderr = result.stderr.strip()
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=False,
            message=(
                f"[FAIL]    {spec.role}: usbipd attach BusId {busid} failed "
                f"(exit {result.returncode}): {stderr}"
            ),
        )


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def attach_all(
    config: HardwareConfig,
    *,
    dry_run: bool = False,
    state_json_path: Optional[Path] = None,
) -> list[_AttachResult]:
    """Attach all required devices and return per-device results."""
    state = _get_usbipd_state_json(state_json_path)
    devices: list[dict] = state.get("Devices", [])
    specs = _device_specs_from_config(config)
    return [_attach_device(spec, devices, dry_run) for spec in specs]


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Attach all inchworm USB devices to WSL via usbipd.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve BusIds and print attach commands without executing them.",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        help="Path to hardware.yaml (default: robot/config/hardware.yaml).",
    )
    parser.add_argument(
        "--state-json",
        metavar="PATH",
        help="Use this JSON file as usbipd state output instead of running powershell.exe.",
    )
    args = parser.parse_args(argv)

    config = load_config(args.config)
    state_json_path = Path(args.state_json) if args.state_json else None

    try:
        results = attach_all(config, dry_run=args.dry_run, state_json_path=state_json_path)
    except RuntimeError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1

    any_failure = False
    for res in results:
        print(res.message)
        if not res.success:
            any_failure = True

    print()
    n_ok = sum(1 for r in results if r.success)
    n_total = len(results)
    print(f"{n_ok}/{n_total} devices OK")

    return 1 if any_failure else 0


if __name__ == "__main__":
    sys.exit(main())
