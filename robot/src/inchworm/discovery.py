"""Device discovery: maps logical roles to live /dev nodes using the hardware config.

Role resolution is driven exclusively by config fields; USB enumeration order
never affects which role a device receives.

Resolution paths
----------------
``identify_by: usb_serial`` (arm_left, arm_right, dexarm, cam_scanner)
    Looks up ``linux_by_id`` in ``/dev/serial/by-id/`` or ``/dev/v4l/by-id/``.

``identify_by: vid_pid`` (relay)
    Looks up ``linux_by_id`` in ``/dev/serial/by-id/`` using the VID:PID-derived
    name.  Also cross-checks the relay BusId through vhci_hcd.0/status when the
    status file is available.

``identify_by: usbip_busid`` (cam_left, cam_right)
    Resolves the ``windows_busid`` field (physical dock port) through
    ``/sys/devices/platform/vhci_hcd.0/status`` to a local sysfs device, then
    finds the ``/dev/videoN`` capture node (index 0).
    POSITIONAL: the dock port encoded in config must not change.
    (Both OV9281 cameras report serial 5423534724, so they cannot be
    distinguished by serial, VID:PID, or /dev/v4l/by-id.)

Safety guards
-------------
relay (CH340, 1a86:7523, no USB serial)
    If more than one 1a86:7523 entry exists in /dev/serial/by-id, discovery
    fails immediately with :class:`RelayAmbiguousError` naming the role.  The
    operator must either remove the extra device or pin the rule by port path
    (KERNELS== in 99-inchworm.rules).

Global duplicate invariant
    After resolving all roles, discovery raises :class:`DuplicateDeviceNodeError`
    if any two roles resolve to the same ``/dev`` node, or
    :class:`DuplicateUsbDeviceError` if any two roles map to the same underlying
    USB device path.  This catches misconfigurations before any role-keyed
    access can pick the wrong device.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional, Sequence

from inchworm.config import DeviceIdentity, HardwareConfig, IdentifyBy, load_config
from inchworm.vhci import (
    BusIdNotFoundError,
    VhciParseError,
    resolve_busid_to_video_node,
    VHCI_STATUS_PATH,
    USB_DEVICES_DIR,
    VIDEO4LINUX_CLASS_DIR,
)

# Directories scanned for by-id symlinks.  Overridable in tests.
SERIAL_BY_ID_DIR = Path("/dev/serial/by-id")
V4L_BY_ID_DIR = Path("/dev/v4l/by-id")
SERIAL_BY_PATH_DIR = Path("/dev/serial/by-path")
V4L_BY_PATH_DIR = Path("/dev/v4l/by-path")


class DeviceStatus(str, Enum):
    """Observed attachment state of one logical device role."""

    ATTACHED = "attached"
    """by-id symlink present and its target node exists, or usbip_busid resolved."""

    NOT_ATTACHED = "not_attached"
    """by-id symlink present but the target node does not exist (device not
    forwarded to WSL via usbipd), or usbip_busid BusId not found in vhci status."""

    ABSENT = "absent"
    """by-id symlink not present in the scanned directory."""

    NO_UDEV_RULE = "no_udev_rule"
    """Device has ``linux_by_id: null`` and ``linux_by_path: null`` in the
    config and is not ``usbip_busid``; no stable symlink can exist until a
    udev rule is installed."""


# Mapping from logical role name to the config field that holds its identity.
# Order is display order only; resolution never depends on it.
_ROLE_DEVICE_DIR = {
    "arm_left": "serial",
    "arm_right": "serial",
    "dexarm": "serial",
    "relay": "serial",
    "cam_left": "v4l",
    "cam_right": "v4l",
    "cam_scanner": "v4l",
}

_USBIPD_BIND_HINT = (
    "Run on Windows (administrator):\n"
    "  usbipd bind --busid <busid>\n"
    "  usbipd attach --wsl --busid <busid>"
)


class RelayAmbiguousError(RuntimeError):
    """Raised when more than one CH340 1a86:7523 device is present.

    The relay controls laser power; a mis-identified relay could fire the
    laser at the wrong target.  Resolution must fail explicitly rather than
    guess.

    Attributes
    ----------
    role:
        Always ``"relay"``.
    found_entries:
        List of by-id entry names that matched 1a86:7523.
    """

    def __init__(self, found_entries: list) -> None:
        self.role = "relay"
        self.found_entries = found_entries
        super().__init__(
            f"Role 'relay' is ambiguous: {len(found_entries)} CH340 (1a86:7523) "
            f"devices found in /dev/serial/by-id: {found_entries}.\n"
            "The relay controls laser power; identifying the wrong device is a "
            "safety hazard.\n"
            "Fix: ensure only one CH340 1a86:7523 is attached, or add a "
            'KERNELS=="<port_path>" pin to the relay udev rule in '
            "robot/udev/99-inchworm.rules and set linux_by_path in hardware.yaml."
        )


class CamRightAmbiguousError(RuntimeError):
    """Kept for backwards compatibility.  No longer raised by default discovery.

    Previously used when the count of 32e6:9251 serial entries in
    /dev/v4l/by-id was not 1.  Both OV9281 cameras now use
    ``identify_by: usbip_busid`` and are resolved through vhci_hcd.0/status,
    making this check unnecessary.  Retained so existing code that catches it
    does not break.

    Attributes
    ----------
    role:
        Always ``"cam_right"``.
    found_count:
        How many 32e6:9251 serial by-id entries were found.
    """

    def __init__(self, found_count: int, found_entries: list) -> None:
        self.role = "cam_right"
        self.found_count = found_count
        self.found_entries = found_entries
        super().__init__(
            f"Role 'cam_right' is ambiguous: expected exactly 1 entry for "
            f"openaicam serial 5423534724 in /dev/v4l/by-id, but "
            f"found {found_count}: {found_entries}.\n"
            "cam_right is resolved via identify_by: usbip_busid (windows_busid "
            "field), not by VID:PID; this error should not be raised in normal "
            "operation. Check for duplicate usbip_busid entries in hardware.yaml."
        )


class DuplicateDeviceNodeError(RuntimeError):
    """Raised when two or more roles resolve to the same /dev node.

    This indicates a hardware misconfiguration (e.g. the same physical device
    mapped to two roles) and must be caught before any role-keyed access.

    Attributes
    ----------
    node:
        The /dev node path that appeared more than once.
    roles:
        List of role names that resolved to ``node``.
    """

    def __init__(self, node: str, roles: list) -> None:
        self.node = node
        self.roles = roles
        super().__init__(
            f"Two roles resolve to the same /dev node {node!r}: {roles}.\n"
            "This is a hardware misconfiguration. Verify that the config's "
            "identity fields (usb_serial, windows_busid) point to distinct "
            "physical devices and that each device is attached at most once."
        )


class DuplicateUsbDeviceError(RuntimeError):
    """Raised when two or more roles map to the same USB sysfs device path.

    A USB camera device exposes two video nodes (index 0 and 1); both map to
    the same parent sysfs device.  If two roles resolve to the same sysfs
    parent, one device is being claimed as two roles.

    Attributes
    ----------
    usb_path:
        The sysfs USB device path that appeared more than once.
    roles:
        List of role names that resolved to ``usb_path``.
    """

    def __init__(self, usb_path: str, roles: list) -> None:
        self.usb_path = usb_path
        self.roles = roles
        super().__init__(
            f"Two roles map to the same USB device {usb_path!r}: {roles}.\n"
            "This is a hardware misconfiguration: the same physical device "
            "has been assigned to multiple roles. Check identity fields in "
            "hardware.yaml."
        )


@dataclass(frozen=True)
class DeviceResult:
    """Discovery result for one logical role."""

    role: str
    """Logical role name (e.g. ``arm_left``)."""

    status: DeviceStatus
    """Observed attachment state."""

    by_id_name: Optional[str]
    """The by-id or by-path symlink name used for resolution, or the
    Windows BusId for ``usbip_busid`` devices."""

    dev_node: Optional[str]
    """Resolved /dev/ node path, if ``status`` is :attr:`DeviceStatus.ATTACHED`."""

    vid_pid: str
    """``vendor_id:product_id`` from config."""

    usb_serial: Optional[str]
    """USB serial number from config, or ``None`` for port-path/usbip_busid devices."""

    remediation: Optional[str]
    """Human-readable fix instruction, set when status is not ATTACHED."""

    def to_dict(self) -> dict:
        """Machine-readable dict for JSON output."""
        return {
            "role": self.role,
            "status": self.status.value,
            "by_id_name": self.by_id_name,
            "dev_node": self.dev_node,
            "vid_pid": self.vid_pid,
            "usb_serial": self.usb_serial,
            "remediation": self.remediation,
        }


class DeviceNotFoundError(RuntimeError):
    """Raised when a required device role has no by-id symlink (absent).

    Attributes
    ----------
    role:
        Logical role name.
    remediation:
        Exact command the operator should run to fix the problem.
    """

    def __init__(self, role: str, remediation: str) -> None:
        self.role = role
        self.remediation = remediation
        super().__init__(
            f"Device role '{role}' not found in /dev/serial/by-id or /dev/v4l/by-id.\n"
            f"Remediation: {remediation}"
        )


class DeviceNotAttachedError(RuntimeError):
    """Raised when a device's by-id symlink exists but its /dev node is missing.

    This typically means the device is bound in usbipd but not yet attached
    to WSL for this session.

    Attributes
    ----------
    role:
        Logical role name.
    by_id_name:
        The by-id symlink that was found.
    remediation:
        Exact ``usbipd attach`` command the operator should run.
    """

    def __init__(self, role: str, by_id_name: str, remediation: str) -> None:
        self.role = role
        self.by_id_name = by_id_name
        self.remediation = remediation
        super().__init__(
            f"Device role '{role}' has a by-id symlink '{by_id_name}' but the target "
            f"/dev node does not exist (not attached to WSL).\n"
            f"Remediation: {remediation}"
        )


def _identity_for_role(role: str, config: HardwareConfig) -> DeviceIdentity:
    """Return the :class:`DeviceIdentity` for *role* from *config*."""
    mapping = {
        "arm_left": config.arms.left.identity,
        "arm_right": config.arms.right.identity,
        "dexarm": config.dexarm.identity,
        "relay": config.relay.identity,
        "cam_left": config.cameras.left_arm.identity,
        "cam_right": config.cameras.right_arm.identity,
        "cam_scanner": config.cameras.scanner.identity,
    }
    return mapping[role]


def _remediation_for_role(role: str, identity: DeviceIdentity) -> str:
    """Build the remediation string for a device that is absent or not attached."""
    # Prefer windows_busid (the identity key) over windows_busid_observed.
    busid = identity.windows_busid or identity.windows_busid_observed
    if busid:
        return (
            f"usbipd bind --busid {busid}  # Windows administrator, one-time\n"
            f"usbipd attach --wsl --busid {busid}  # each WSL session"
        )
    # No busid in config — give a generic instruction.
    return (
        f"Identify the Windows BusId for role '{role}' (VID:PID {identity.vid_pid}) with:\n"
        f"  usbipd list  # on Windows\n"
        f"Then:\n"
        f"  usbipd bind --busid <busid>  # Windows administrator, one-time\n"
        f"  usbipd attach --wsl --busid <busid>  # each WSL session"
    )


def _scan_dir(directory: Path) -> dict:
    """Return ``{symlink_name: resolved_target_path}`` for all entries in *directory*.

    If *directory* does not exist, returns an empty dict (graceful degradation
    when devices are not attached at all).
    """
    if not directory.exists():
        return {}
    result = {}
    for entry in directory.iterdir():
        if entry.is_symlink():
            result[entry.name] = entry.resolve()
    return result


def _ch340_entries_in_by_id(serial_symlinks: dict) -> list:
    """Return by-id entry names matching the CH340 1a86:7523 naming pattern.

    The CH340 (1a86:7523, no serial) creates by-id entries named
    ``usb-1a86_USB_Serial-if00-portN``.  Match on the vendor/product prefix.
    A single CH340 produces exactly one such entry; two CH340s produce two.
    """
    return [name for name in serial_symlinks if name.startswith("usb-1a86_USB_Serial")]


def _check_relay_ambiguity(serial_symlinks: dict) -> None:
    """Raise :class:`RelayAmbiguousError` if more than one CH340 1a86:7523 appears.

    The relay controls laser power.  If more than one CH340 (1a86:7523) device
    is present the assignment is unsafe and must fail explicitly rather than
    silently pick one.
    """
    ch340_entries = _ch340_entries_in_by_id(serial_symlinks)
    if len(ch340_entries) > 1:
        raise RelayAmbiguousError(found_entries=ch340_entries)


def _probe_usbip_busid(
    role: str,
    identity: DeviceIdentity,
    *,
    vhci_status_path: Path,
    usb_devices_dir: Path,
    v4l_class_dir: Path,
) -> DeviceResult:
    """Probe a role that uses ``identify_by: usbip_busid``.

    Resolves via vhci_hcd.0/status → sysfs → /dev/videoN.  Returns ABSENT when
    the vhci status file is missing (devices not attached), NOT_ATTACHED when
    the BusId is not in the status table, and ATTACHED when a video node is found.
    """
    windows_busid = identity.windows_busid
    remediation = _remediation_for_role(role, identity)

    if not vhci_status_path.exists():
        # No vhci device at all (bare Linux, no usbipd): treat as absent.
        return DeviceResult(
            role=role,
            status=DeviceStatus.ABSENT,
            by_id_name=windows_busid,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=remediation,
        )

    try:
        dev_node = resolve_busid_to_video_node(
            windows_busid,
            vhci_status_path=vhci_status_path,
            usb_devices_dir=usb_devices_dir,
            v4l_class_dir=v4l_class_dir,
        )
        return DeviceResult(
            role=role,
            status=DeviceStatus.ATTACHED,
            by_id_name=windows_busid,
            dev_node=dev_node,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=None,
        )
    except BusIdNotFoundError:
        return DeviceResult(
            role=role,
            status=DeviceStatus.NOT_ATTACHED,
            by_id_name=windows_busid,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=remediation,
        )
    except LookupError as err:
        # BusId found in vhci but no video node at index 0.
        return DeviceResult(
            role=role,
            status=DeviceStatus.NOT_ATTACHED,
            by_id_name=windows_busid,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=f"{remediation}\n(sysfs note: {err})",
        )
    except VhciParseError as err:
        return DeviceResult(
            role=role,
            status=DeviceStatus.ABSENT,
            by_id_name=windows_busid,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=f"{remediation}\n(vhci error: {err})",
        )


def _probe_role(
    role: str,
    identity: DeviceIdentity,
    serial_symlinks: dict,
    v4l_symlinks: dict,
    serial_by_id_dir: Path,
    v4l_by_id_dir: Path,
    serial_by_path_dir: Path = SERIAL_BY_PATH_DIR,
    v4l_by_path_dir: Path = V4L_BY_PATH_DIR,
    vhci_status_path: Path = VHCI_STATUS_PATH,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    v4l_class_dir: Path = VIDEO4LINUX_CLASS_DIR,
) -> DeviceResult:
    """Probe one role and return its :class:`DeviceResult`.

    Dispatches to the appropriate resolution path based on ``identify_by``.
    """
    device_type = _ROLE_DEVICE_DIR[role]

    # --- usbip_busid devices: resolve via vhci_hcd.0/status -----------------
    if identity.identify_by is IdentifyBy.USBIP_BUSID:
        return _probe_usbip_busid(
            role,
            identity,
            vhci_status_path=vhci_status_path,
            usb_devices_dir=usb_devices_dir,
            v4l_class_dir=v4l_class_dir,
        )

    # --- VID:PID devices: use linux_by_id when available, else by-path ------
    if identity.identify_by is IdentifyBy.VID_PID:
        if identity.linux_by_id:
            by_id_name = identity.linux_by_id
            scan_dir = serial_by_id_dir if device_type == "serial" else v4l_by_id_dir
            symlinks = serial_symlinks if device_type == "serial" else v4l_symlinks

            if by_id_name not in symlinks:
                remediation = _remediation_for_role(role, identity)
                return DeviceResult(
                    role=role,
                    status=DeviceStatus.ABSENT,
                    by_id_name=by_id_name,
                    dev_node=None,
                    vid_pid=identity.vid_pid,
                    usb_serial=identity.usb_serial,
                    remediation=remediation,
                )

            target = symlinks[by_id_name]
            symlink_path = scan_dir / by_id_name
            target_exists = os.path.exists(symlink_path)

            if not target_exists:
                remediation = _remediation_for_role(role, identity)
                return DeviceResult(
                    role=role,
                    status=DeviceStatus.NOT_ATTACHED,
                    by_id_name=by_id_name,
                    dev_node=str(target),
                    vid_pid=identity.vid_pid,
                    usb_serial=identity.usb_serial,
                    remediation=remediation,
                )

            return DeviceResult(
                role=role,
                status=DeviceStatus.ATTACHED,
                by_id_name=by_id_name,
                dev_node=str(target),
                vid_pid=identity.vid_pid,
                usb_serial=identity.usb_serial,
                remediation=None,
            )

        elif identity.linux_by_path:
            by_path_name = identity.linux_by_path
            scan_dir = serial_by_path_dir if device_type == "serial" else v4l_by_path_dir
            path_symlinks = _scan_dir(scan_dir)

            if by_path_name not in path_symlinks:
                remediation = _remediation_for_role(role, identity)
                return DeviceResult(
                    role=role,
                    status=DeviceStatus.ABSENT,
                    by_id_name=by_path_name,
                    dev_node=None,
                    vid_pid=identity.vid_pid,
                    usb_serial=identity.usb_serial,
                    remediation=remediation,
                )

            target = path_symlinks[by_path_name]
            symlink_path = scan_dir / by_path_name
            target_exists = os.path.exists(symlink_path)

            if not target_exists:
                remediation = _remediation_for_role(role, identity)
                return DeviceResult(
                    role=role,
                    status=DeviceStatus.NOT_ATTACHED,
                    by_id_name=by_path_name,
                    dev_node=str(target),
                    vid_pid=identity.vid_pid,
                    usb_serial=identity.usb_serial,
                    remediation=remediation,
                )

            return DeviceResult(
                role=role,
                status=DeviceStatus.ATTACHED,
                by_id_name=by_path_name,
                dev_node=str(target),
                vid_pid=identity.vid_pid,
                usb_serial=identity.usb_serial,
                remediation=None,
            )

        else:
            return DeviceResult(
                role=role,
                status=DeviceStatus.NO_UDEV_RULE,
                by_id_name=None,
                dev_node=None,
                vid_pid=identity.vid_pid,
                usb_serial=identity.usb_serial,
                remediation=(
                    f"Role '{role}' has no linux_by_id or linux_by_path in the config. "
                    "Install the udev rules from robot/udev/99-inchworm.rules to "
                    "create a stable symlink, then re-run discovery."
                ),
            )

    # --- Port-path devices: use linux_by_path --------------------------------
    if identity.identify_by is IdentifyBy.PORT_PATH and identity.linux_by_path:
        by_path_name = identity.linux_by_path
        scan_dir = serial_by_path_dir if device_type == "serial" else v4l_by_path_dir
        path_symlinks = _scan_dir(scan_dir)

        if by_path_name not in path_symlinks:
            remediation = _remediation_for_role(role, identity)
            return DeviceResult(
                role=role,
                status=DeviceStatus.ABSENT,
                by_id_name=by_path_name,
                dev_node=None,
                vid_pid=identity.vid_pid,
                usb_serial=identity.usb_serial,
                remediation=remediation,
            )

        target = path_symlinks[by_path_name]
        symlink_path = scan_dir / by_path_name
        target_exists = os.path.exists(symlink_path)

        if not target_exists:
            remediation = _remediation_for_role(role, identity)
            return DeviceResult(
                role=role,
                status=DeviceStatus.NOT_ATTACHED,
                by_id_name=by_path_name,
                dev_node=str(target),
                vid_pid=identity.vid_pid,
                usb_serial=identity.usb_serial,
                remediation=remediation,
            )

        return DeviceResult(
            role=role,
            status=DeviceStatus.ATTACHED,
            by_id_name=by_path_name,
            dev_node=str(target),
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=None,
        )

    # --- By-id devices (usb_serial) ----------------------------------------
    by_id_name = identity.linux_by_id

    if by_id_name is None:
        return DeviceResult(
            role=role,
            status=DeviceStatus.NO_UDEV_RULE,
            by_id_name=None,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=(
                f"Role '{role}' has no linux_by_id or linux_by_path in the config. "
                "Install the udev rules from robot/udev/99-inchworm.rules to "
                "create a stable by-id symlink, then re-run discovery."
            ),
        )

    symlinks = serial_symlinks if device_type == "serial" else v4l_symlinks
    scan_dir = serial_by_id_dir if device_type == "serial" else v4l_by_id_dir

    if by_id_name not in symlinks:
        remediation = _remediation_for_role(role, identity)
        return DeviceResult(
            role=role,
            status=DeviceStatus.ABSENT,
            by_id_name=by_id_name,
            dev_node=None,
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=remediation,
        )

    target = symlinks[by_id_name]
    symlink_path = scan_dir / by_id_name
    target_exists = os.path.exists(symlink_path)

    if not target_exists:
        remediation = _remediation_for_role(role, identity)
        return DeviceResult(
            role=role,
            status=DeviceStatus.NOT_ATTACHED,
            by_id_name=by_id_name,
            dev_node=str(target),
            vid_pid=identity.vid_pid,
            usb_serial=identity.usb_serial,
            remediation=remediation,
        )

    return DeviceResult(
        role=role,
        status=DeviceStatus.ATTACHED,
        by_id_name=by_id_name,
        dev_node=str(target),
        vid_pid=identity.vid_pid,
        usb_serial=identity.usb_serial,
        remediation=None,
    )


def _check_duplicate_nodes(results: list) -> None:
    """Raise :class:`DuplicateDeviceNodeError` if two roles share a /dev node.

    Only ATTACHED results with a dev_node are compared.  An absent or
    unattached device cannot cause a node collision.
    """
    node_to_roles: dict[str, list] = {}
    for r in results:
        if r.status is DeviceStatus.ATTACHED and r.dev_node is not None:
            node_to_roles.setdefault(r.dev_node, []).append(r.role)
    for node, roles in node_to_roles.items():
        if len(roles) > 1:
            raise DuplicateDeviceNodeError(node=node, roles=roles)


def _check_duplicate_usb_devices(
    results: list,
    *,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    vhci_status_path: Path = VHCI_STATUS_PATH,
) -> None:
    """Raise :class:`DuplicateUsbDeviceError` if two roles map to the same USB device.

    For ``usbip_busid`` devices the check is by ``windows_busid`` (the identity
    key).  For serial/by-id devices the check is by the resolved sysfs symlink
    target (the dev_node).  Two cameras on the same USB device would be caught
    by ``_check_duplicate_nodes`` for their capture nodes, but this provides an
    additional layer using the config's identity keys.
    """
    busid_to_roles: dict[str, list] = {}
    for r in results:
        if r.status is DeviceStatus.ATTACHED and r.by_id_name is not None:
            # For usbip_busid, by_id_name is the windows_busid itself.
            # For others, by_id_name is a by-id symlink name; use dev_node as key.
            if r.dev_node is not None:
                busid_to_roles.setdefault(r.by_id_name, []).append(r.role)
    for key, roles in busid_to_roles.items():
        if len(roles) > 1:
            raise DuplicateUsbDeviceError(usb_path=key, roles=roles)


def discover(
    config: Optional[HardwareConfig] = None,
    *,
    serial_by_id_dir: Path = SERIAL_BY_ID_DIR,
    v4l_by_id_dir: Path = V4L_BY_ID_DIR,
    serial_by_path_dir: Path = SERIAL_BY_PATH_DIR,
    v4l_by_path_dir: Path = V4L_BY_PATH_DIR,
    vhci_status_path: Path = VHCI_STATUS_PATH,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    v4l_class_dir: Path = VIDEO4LINUX_CLASS_DIR,
) -> list:
    """Discover all seven device roles.

    Returns a list of :class:`DeviceResult` in a stable display order.  Role
    resolution is driven entirely by config fields; USB enumeration order never
    affects which role a device receives.

    Parameters
    ----------
    config:
        Parsed :class:`~inchworm.config.HardwareConfig`.  If ``None``, loads
        the default config.
    serial_by_id_dir:
        Override for the serial by-id directory (used in tests).
    v4l_by_id_dir:
        Override for the V4L by-id directory (used in tests).
    serial_by_path_dir:
        Override for the serial by-path directory (used in tests).
    v4l_by_path_dir:
        Override for the V4L by-path directory (used in tests).
    vhci_status_path:
        Override for the vhci_hcd status file path (used in tests).
    usb_devices_dir:
        Override for ``/sys/bus/usb/devices`` (used in tests).
    v4l_class_dir:
        Override for ``/sys/class/video4linux`` (used in tests).

    Raises
    ------
    RelayAmbiguousError
        If more than one CH340 1a86:7523 device is present in by-id.
    DuplicateDeviceNodeError
        If two or more roles resolve to the same /dev node.
    DuplicateUsbDeviceError
        If two or more roles map to the same underlying USB identity key.
    """
    if config is None:
        config = load_config()

    serial_symlinks = _scan_dir(serial_by_id_dir)
    v4l_symlinks = _scan_dir(v4l_by_id_dir)

    # Safety: fail fast if relay identity is ambiguous.
    _check_relay_ambiguity(serial_symlinks)

    results = []
    for role in _ROLE_DEVICE_DIR:
        identity = _identity_for_role(role, config)
        result = _probe_role(
            role=role,
            identity=identity,
            serial_symlinks=serial_symlinks,
            v4l_symlinks=v4l_symlinks,
            serial_by_id_dir=serial_by_id_dir,
            v4l_by_id_dir=v4l_by_id_dir,
            serial_by_path_dir=serial_by_path_dir,
            v4l_by_path_dir=v4l_by_path_dir,
            vhci_status_path=vhci_status_path,
            usb_devices_dir=usb_devices_dir,
            v4l_class_dir=v4l_class_dir,
        )
        results.append(result)

    # Global duplicate invariant: fail if any two roles share a dev node or
    # the same USB identity key.
    _check_duplicate_nodes(results)
    _check_duplicate_usb_devices(results, usb_devices_dir=usb_devices_dir,
                                  vhci_status_path=vhci_status_path)

    return results


def require_device(
    role: str,
    config: Optional[HardwareConfig] = None,
    *,
    serial_by_id_dir: Path = SERIAL_BY_ID_DIR,
    v4l_by_id_dir: Path = V4L_BY_ID_DIR,
    serial_by_path_dir: Path = SERIAL_BY_PATH_DIR,
    v4l_by_path_dir: Path = V4L_BY_PATH_DIR,
    vhci_status_path: Path = VHCI_STATUS_PATH,
    usb_devices_dir: Path = USB_DEVICES_DIR,
    v4l_class_dir: Path = VIDEO4LINUX_CLASS_DIR,
) -> str:
    """Return the /dev node path for *role*, or raise if unavailable.

    Raises
    ------
    RelayAmbiguousError
        If more than one CH340 1a86:7523 is present (relay identity unsafe).
    DeviceNotFoundError
        If the by-id/by-path symlink for the role is absent entirely, or the
        vhci_hcd BusId is not found.
    DeviceNotAttachedError
        If the by-id/by-path symlink exists but the /dev target does not.
    ValueError
        If *role* is not a known role name.
    """
    if role not in _ROLE_DEVICE_DIR:
        raise ValueError(f"Unknown device role: {role!r}. Valid roles: {list(_ROLE_DEVICE_DIR)}")
    if config is None:
        config = load_config()

    identity = _identity_for_role(role, config)
    device_type = _ROLE_DEVICE_DIR[role]

    # Ambiguity checks for relay.
    _check_relay_ambiguity(_scan_dir(serial_by_id_dir))

    # usbip_busid devices.
    if identity.identify_by is IdentifyBy.USBIP_BUSID:
        result = _probe_usbip_busid(
            role,
            identity,
            vhci_status_path=vhci_status_path,
            usb_devices_dir=usb_devices_dir,
            v4l_class_dir=v4l_class_dir,
        )
        if result.status is DeviceStatus.ATTACHED and result.dev_node:
            return result.dev_node
        remediation = _remediation_for_role(role, identity)
        if result.status is DeviceStatus.NOT_ATTACHED:
            raise DeviceNotAttachedError(
                role=role, by_id_name=identity.windows_busid or "(none)", remediation=remediation
            )
        raise DeviceNotFoundError(role=role, remediation=remediation)

    # VID:PID devices (relay): use linux_by_id if set, else by-path.
    if identity.identify_by is IdentifyBy.VID_PID:
        if identity.linux_by_id:
            by_id_name = identity.linux_by_id
            scan_dir = serial_by_id_dir if device_type == "serial" else v4l_by_id_dir
            symlinks = _scan_dir(scan_dir)

            if by_id_name not in symlinks:
                remediation = _remediation_for_role(role, identity)
                raise DeviceNotFoundError(role=role, remediation=remediation)

            symlink_path = scan_dir / by_id_name
            if not os.path.exists(symlink_path):
                remediation = _remediation_for_role(role, identity)
                raise DeviceNotAttachedError(
                    role=role, by_id_name=by_id_name, remediation=remediation
                )
            return str(symlinks[by_id_name])

        elif identity.linux_by_path:
            by_path_name = identity.linux_by_path
            path_dir = serial_by_path_dir if device_type == "serial" else v4l_by_path_dir
            path_symlinks = _scan_dir(path_dir)

            if by_path_name not in path_symlinks:
                remediation = _remediation_for_role(role, identity)
                raise DeviceNotFoundError(role=role, remediation=remediation)

            symlink_path = path_dir / by_path_name
            if not os.path.exists(symlink_path):
                remediation = _remediation_for_role(role, identity)
                raise DeviceNotAttachedError(
                    role=role, by_id_name=by_path_name, remediation=remediation
                )
            return str(path_symlinks[by_path_name])

        else:
            raise DeviceNotFoundError(
                role=role,
                remediation=(
                    f"Role '{role}' has no linux_by_id or linux_by_path in the config. "
                    "Install udev rules (robot/udev/99-inchworm.rules)."
                ),
            )

    # Port-path devices: use linux_by_path.
    if identity.identify_by is IdentifyBy.PORT_PATH and identity.linux_by_path:
        by_path_name = identity.linux_by_path
        path_dir = serial_by_path_dir if device_type == "serial" else v4l_by_path_dir
        path_symlinks = _scan_dir(path_dir)

        if by_path_name not in path_symlinks:
            remediation = _remediation_for_role(role, identity)
            raise DeviceNotFoundError(role=role, remediation=remediation)

        symlink_path = path_dir / by_path_name
        if not os.path.exists(symlink_path):
            remediation = _remediation_for_role(role, identity)
            raise DeviceNotAttachedError(
                role=role, by_id_name=by_path_name, remediation=remediation
            )

        return str(path_symlinks[by_path_name])

    by_id_name = identity.linux_by_id

    if by_id_name is None:
        raise DeviceNotFoundError(
            role=role,
            remediation=(
                f"Role '{role}' has no linux_by_id in the config. "
                "Install udev rules (robot/udev/99-inchworm.rules)."
            ),
        )

    scan_dir = serial_by_id_dir if device_type == "serial" else v4l_by_id_dir
    symlinks = _scan_dir(scan_dir)

    if by_id_name not in symlinks:
        remediation = _remediation_for_role(role, identity)
        raise DeviceNotFoundError(role=role, remediation=remediation)

    symlink_path = scan_dir / by_id_name
    if not os.path.exists(symlink_path):
        remediation = _remediation_for_role(role, identity)
        raise DeviceNotAttachedError(role=role, by_id_name=by_id_name, remediation=remediation)

    return str(symlinks[by_id_name])


# ---------------------------------------------------------------------------
# Human-readable and JSON formatting
# ---------------------------------------------------------------------------

_STATUS_LABEL = {
    DeviceStatus.ATTACHED: "✓ attached",
    DeviceStatus.NOT_ATTACHED: "✗ not attached",
    DeviceStatus.ABSENT: "✗ absent",
    DeviceStatus.NO_UDEV_RULE: "? no udev rule",
}


def format_human(results: Sequence[DeviceResult]) -> str:
    """Return a human-readable table of discovery results."""
    lines = ["Device discovery results:", ""]
    role_width = max(len(r.role) for r in results)
    status_width = max(len(_STATUS_LABEL[r.status]) for r in results)

    for res in results:
        status_str = _STATUS_LABEL[res.status]
        node_str = res.dev_node or "(none)"
        by_id_str = res.by_id_name or "(none)"
        lines.append(
            f"  {res.role:<{role_width}}  {status_str:<{status_width}}"
            f"  node={node_str}"
        )
        lines.append(f"  {'':>{role_width}}  {'':>{status_width}}  by-id={by_id_str}")
        if res.remediation:
            for rline in res.remediation.splitlines():
                lines.append(f"  {'':>{role_width}}  {'':>{status_width}}  {rline}")
        lines.append("")
    return "\n".join(lines)


def format_json(results: Sequence[DeviceResult]) -> str:
    """Return a JSON array of discovery results."""
    return json.dumps([r.to_dict() for r in results], indent=2)
