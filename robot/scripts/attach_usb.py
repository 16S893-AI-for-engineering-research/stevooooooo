#!/usr/bin/env python3
"""WSL USB attach helper for the inchworm print-crawl robot.

Resolves each required device's Windows BusId from ``usbipd state`` JSON
(keyed on VID:PID + USB serial where available, VID:PID alone for port-path
devices), then calls::

    powershell.exe -NoProfile -Command "usbipd attach --wsl --busid X"

for every device.  After each attach, verifies that the device appears in
``/sys/devices/platform/vhci_hcd.0/status`` (polling up to
``ATTACH_VERIFY_TIMEOUT_S`` with ``ATTACH_VERIFY_POLL_S`` intervals).  If the
device does not appear within the timeout, the attach is retried once.  Reports
per-device ``[OK]``, ``[RETRY-OK]``, or ``[FAIL]`` and exits nonzero if any
device fails.

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
--vhci-status
             Path to the vhci_hcd status file
             (default: /sys/devices/platform/vhci_hcd.0/status).
             Intended for testing; do not use in production.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

# Allow running directly from the repository root without installing the package.
_HERE = Path(__file__).resolve()
_ROBOT_ROOT = _HERE.parents[1]
sys.path.insert(0, str(_ROBOT_ROOT / "src"))

from inchworm.config import HardwareConfig, load_config  # noqa: E402
from inchworm.vhci import (
    VHCI_STATUS_PATH,
    _busid_to_dev_hex,
    _parse_vhci_status,
)

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------

#: Seconds to poll vhci status after each attach before giving up / retrying.
ATTACH_VERIFY_TIMEOUT_S: float = 5.0

#: Polling interval for vhci presence check.
ATTACH_VERIFY_POLL_S: float = 0.5


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

    windows_busid: Optional[str] = None
    """For ``identify_by: usbip_busid`` devices (the two arm cameras share a
    serial): the configured dock-port BusId. Matched by BusId + VID:PID, never by
    'first device without a serial'. Orchestrator fix 2026-10-24: after a power
    cycle the old rule picked the right camera's port for cam_left."""


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

    retried: bool
    """True if the first vhci verification failed and the attach was retried."""

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

    if spec.windows_busid is not None:
        # Positional identity: the configured BusId, verified against VID:PID.
        for dev in devices:
            if dev.get("BusId") == spec.windows_busid:
                vid, pid, _ = _parse_instance_id(dev.get("InstanceId", ""))
                return dev["BusId"] if (vid, pid) == (target_vid, target_pid) else None
        return None

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
            windows_busid=(getattr(identity, "windows_busid", None)
                           if getattr(getattr(identity, "identify_by", None), "value", None) == "usbip_busid"
                           else None),
        )
        for role, identity in raw
    ]


# ---------------------------------------------------------------------------
# Attach logic
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# vhci presence verification
# ---------------------------------------------------------------------------


def _busid_present_in_vhci(
    busid: str,
    vhci_status_reader: Callable[[], str],
) -> bool:
    """Return True if *busid* appears as an attached entry in vhci_hcd status.

    Parameters
    ----------
    busid:
        Windows BusId string (e.g. ``"9-4"``).
    vhci_status_reader:
        Callable that returns the current text of the vhci_hcd status file.
        Injected so tests can monkeypatch without touching the filesystem.
    """
    try:
        text = vhci_status_reader()
    except OSError:
        return False
    try:
        dev_map = _parse_vhci_status(text)
    except Exception:
        return False
    dev_hex = _busid_to_dev_hex(busid)
    return dev_hex in dev_map


def _wait_for_vhci(
    busid: str,
    vhci_status_reader: Callable[[], str],
    *,
    timeout_s: float = ATTACH_VERIFY_TIMEOUT_S,
    poll_s: float = ATTACH_VERIFY_POLL_S,
) -> bool:
    """Poll vhci status until *busid* appears or the timeout expires.

    Returns True if the device is seen before the deadline, False otherwise.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        if _busid_present_in_vhci(busid, vhci_status_reader):
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(poll_s, remaining))


# ---------------------------------------------------------------------------
# Attach logic
# ---------------------------------------------------------------------------


def _attach_device(
    spec: _DeviceSpec,
    devices: list[dict],
    dry_run: bool,
    *,
    vhci_status_reader: Optional[Callable[[], str]] = None,
    verify_timeout_s: float = ATTACH_VERIFY_TIMEOUT_S,
    verify_poll_s: float = ATTACH_VERIFY_POLL_S,
) -> _AttachResult:
    """Resolve BusId and attach one device.  Returns an :class:`_AttachResult`.

    After a live (non-dry-run) attach command succeeds, the device's BusId is
    verified against ``/sys/devices/platform/vhci_hcd.0/status``.  If absent
    after *verify_timeout_s*, the attach is retried once.  The final label is
    ``[OK]``, ``[RETRY-OK]``, or ``[FAIL]``.

    Parameters
    ----------
    vhci_status_reader:
        Optional callable returning the vhci status text.  Defaults to reading
        :data:`inchworm.vhci.VHCI_STATUS_PATH`.  Override in tests.
    """
    if vhci_status_reader is None:
        _vhci_path = VHCI_STATUS_PATH
        vhci_status_reader = lambda: _vhci_path.read_text()  # noqa: E731

    busid = _resolve_busid(spec, devices)

    if busid is None:
        return _AttachResult(
            role=spec.role,
            busid=None,
            found=False,
            already_attached=False,
            success=False,
            retried=False,
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
            retried=False,
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
            retried=False,
            message=f"[DRY-RUN] {spec.role}: would attach BusId {busid} — {' '.join(cmd[3:])}",
        )

    # --- first attach attempt ---
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=False,
            retried=False,
            message=(
                f"[FAIL]    {spec.role}: usbipd attach BusId {busid} failed "
                f"(exit {result.returncode}): {stderr}"
            ),
        )

    # Attach command succeeded — verify the device appears in vhci status.
    if _wait_for_vhci(
        busid,
        vhci_status_reader,
        timeout_s=verify_timeout_s,
        poll_s=verify_poll_s,
    ):
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=True,
            retried=False,
            message=f"[OK]      {spec.role}: attached BusId {busid}",
        )

    # --- vhci not seen: retry once ---
    result2 = subprocess.run(cmd, capture_output=True, text=True)
    if result2.returncode != 0:
        stderr2 = result2.stderr.strip()
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=False,
            retried=True,
            message=(
                f"[FAIL]    {spec.role}: retry attach BusId {busid} failed "
                f"(exit {result2.returncode}): {stderr2}"
            ),
        )

    if _wait_for_vhci(
        busid,
        vhci_status_reader,
        timeout_s=verify_timeout_s,
        poll_s=verify_poll_s,
    ):
        return _AttachResult(
            role=spec.role,
            busid=busid,
            found=True,
            already_attached=False,
            success=True,
            retried=True,
            message=f"[RETRY-OK] {spec.role}: attached BusId {busid} (needed retry)",
        )

    return _AttachResult(
        role=spec.role,
        busid=busid,
        found=True,
        already_attached=False,
        success=False,
        retried=True,
        message=(
            f"[FAIL]    {spec.role}: BusId {busid} never appeared in vhci status "
            f"after attach + retry (timeout {verify_timeout_s:.1f} s)"
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
    vhci_status_reader: Optional[Callable[[], str]] = None,
    verify_timeout_s: float = ATTACH_VERIFY_TIMEOUT_S,
    verify_poll_s: float = ATTACH_VERIFY_POLL_S,
) -> list[_AttachResult]:
    """Attach all required devices and return per-device results.

    Parameters
    ----------
    vhci_status_reader:
        Optional callable returning the current vhci_hcd status text.
        Defaults to reading :data:`inchworm.vhci.VHCI_STATUS_PATH`.
        Pass a custom reader in tests to avoid touching the filesystem.
    verify_timeout_s:
        How long (seconds) to wait for a device to appear in vhci status
        after an attach before retrying.
    verify_poll_s:
        Polling interval (seconds) for vhci presence checks.
    """
    state = _get_usbipd_state_json(state_json_path)
    devices: list[dict] = state.get("Devices", [])
    specs = _device_specs_from_config(config)
    return [
        _attach_device(
            spec,
            devices,
            dry_run,
            vhci_status_reader=vhci_status_reader,
            verify_timeout_s=verify_timeout_s,
            verify_poll_s=verify_poll_s,
        )
        for spec in specs
    ]


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
    parser.add_argument(
        "--vhci-status",
        metavar="PATH",
        help=(
            "Path to the vhci_hcd status file "
            "(default: /sys/devices/platform/vhci_hcd.0/status). "
            "Intended for testing."
        ),
    )
    args = parser.parse_args(argv)

    config = load_config(args.config)
    state_json_path = Path(args.state_json) if args.state_json else None

    vhci_status_reader: Optional[Callable[[], str]] = None
    if args.vhci_status:
        _vhci_path = Path(args.vhci_status)
        vhci_status_reader = lambda: _vhci_path.read_text()  # noqa: E731

    try:
        results = attach_all(
            config,
            dry_run=args.dry_run,
            state_json_path=state_json_path,
            vhci_status_reader=vhci_status_reader,
        )
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
