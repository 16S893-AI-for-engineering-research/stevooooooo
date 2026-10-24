"""Typed loader and schema for ``robot/config/hardware.yaml``.

The YAML file is the single declarative description of the hardware. This
module only validates and exposes it; it holds no hardware facts itself.
Unknown fields, missing required fields and wrong types are all rejected with
a :class:`ConfigError` naming the offending field path.
"""

from __future__ import annotations

import os
from enum import Enum
from pathlib import Path
from typing import Literal, Optional, Union

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from inchworm.units import Degrees, Millimeters, MillimetersPerPixel

SUPPORTED_SCHEMA_VERSION = 1

CONFIG_PATH_ENV_VAR = "INCHWORM_CONFIG"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "hardware.yaml"


class ConfigError(ValueError):
    """Raised when the hardware config cannot be read or fails validation."""


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class IdentifyBy(str, Enum):
    """How a device is told apart from others with the same VID:PID."""

    USB_SERIAL = "usb_serial"
    PORT_PATH = "port_path"
    VID_PID = "vid_pid"
    USBIP_BUSID = "usbip_busid"
    """Resolved via the Windows usbipd BusId (physical dock port) mapped through
    /sys/devices/platform/vhci_hcd.0/status to a local sysfs device and then to
    its /dev/videoN capture node.  POSITIONAL: the dock port must not change."""


class BackendName(str, Enum):
    MOCK = "mock"
    HARDWARE = "hardware"


class DeviceIdentity(_Strict):
    """Stable USB identity of one device.

    ``windows_busid`` is the identity key for ``identify_by: usbip_busid``
    devices; it encodes the physical dock port (e.g. ``9-4``) and is resolved
    at runtime through ``/sys/devices/platform/vhci_hcd.0/status``.
    POSITIONAL: the dock port encoded here must not change.

    ``windows_busid_observed`` is informational for all other devices: usbipd
    BusIds change across reconnects for non-positional devices and must never
    be used as an identity key for those roles.
    """

    vendor_id: str = Field(pattern=r"^[0-9a-f]{4}$")
    product_id: str = Field(pattern=r"^[0-9a-f]{4}$")
    usb_serial: Optional[str]
    identify_by: IdentifyBy = Field(strict=False)
    linux_by_id: Optional[str]
    linux_by_path: Optional[str] = None
    windows_busid: Optional[str] = None
    windows_busid_observed: Optional[str]

    @model_validator(mode="after")
    def _serial_matches_identify_by(self) -> "DeviceIdentity":
        if self.identify_by is IdentifyBy.USB_SERIAL and not self.usb_serial:
            raise ValueError("identify_by is 'usb_serial' but usb_serial is null")
        if self.identify_by is IdentifyBy.PORT_PATH and self.usb_serial:
            raise ValueError(
                "identify_by is 'port_path' but a usb_serial is set; key on the serial instead"
            )
        if self.identify_by is IdentifyBy.VID_PID and self.usb_serial:
            raise ValueError(
                "identify_by is 'vid_pid' but a usb_serial is set; key on the serial instead"
            )
        if self.identify_by is IdentifyBy.USBIP_BUSID and not self.windows_busid:
            raise ValueError(
                "identify_by is 'usbip_busid' but windows_busid is null; "
                "set the physical dock BusId (e.g. '9-4')"
            )
        return self

    @property
    def vid_pid(self) -> str:
        return f"{self.vendor_id}:{self.product_id}"


class ServoJointMap(_Strict):
    """Joint-to-servo-ID map for one arm.

    All values are ``None`` until a live bus scan (``scan_servos.py --live``)
    has been run and the owner has assigned joint names to the discovered IDs.
    A ``None`` ID means the servo has not yet been located on the bus.
    """

    shoulder_pan: Optional[int] = None
    shoulder_lift: Optional[int] = None
    elbow_flex: Optional[int] = None
    wrist_flex: Optional[int] = None
    wrist_roll: Optional[int] = None
    gripper: Optional[int] = None


class ArmConfig(_Strict):
    adapter_chip: str
    role_verification: Literal["owner_reported", "motion_verified"]
    identity: DeviceIdentity
    # Populated by scan_servos.py --live; all null until the owner runs the scan.
    servo_joints: ServoJointMap = Field(default_factory=ServoJointMap)


class ArmsConfig(_Strict):
    left: ArmConfig
    right: ArmConfig


# --- AA-12 ---
class DexArmWorkspace(_Strict):
    """Axis-aligned workspace bounds for the DexArm (mm, in arm frame).

    Moves outside these limits are rejected *before* transmission by the
    :class:`~inchworm.dexarm.DexArmDriver`.
    """

    x_min_mm: Millimeters
    x_max_mm: Millimeters
    y_min_mm: Millimeters
    y_max_mm: Millimeters
    z_min_mm: Millimeters
    z_max_mm: Millimeters

    @model_validator(mode="after")
    def _ranges_positive(self) -> "DexArmWorkspace":
        for axis in ("x", "y", "z"):
            lo = getattr(self, f"{axis}_min_mm")
            hi = getattr(self, f"{axis}_max_mm")
            if lo >= hi:
                raise ValueError(f"{axis}_min_mm ({lo}) must be < {axis}_max_mm ({hi})")
        return self
# --- AA-12 end ---


class DexArmConfig(_Strict):
    identity: DeviceIdentity
    # --- AA-12 ---
    workspace: Optional[DexArmWorkspace] = None
    # --- AA-12 end ---


class CameraConfig(_Strict):
    spec_model: str
    observed_usb_product: str
    identity: DeviceIdentity


class CamerasConfig(_Strict):
    left_arm: CameraConfig
    right_arm: CameraConfig
    scanner: CameraConfig


class RelayConfig(_Strict):
    adapter_chip: str
    identity: DeviceIdentity


class LaserDesign(_Strict):
    """Owner design values for the laser triangulation (not calibrated)."""

    laser_to_camera_angle_deg: Degrees = Field(gt=0, lt=90)
    baseline_offset_mm: Millimeters = Field(gt=0)
    standoff_mm: Millimeters = Field(gt=0)
    height_sensitivity_mm_per_px: MillimetersPerPixel = Field(gt=0)


class LaserConfig(_Strict):
    switched_by: Literal["relay"]
    design: LaserDesign
    # --- AA-29 ---
    # Maximum continuous laser-on duration (seconds). LaserDriver raises
    # LaserDurationExceededError if a single on-period would exceed this limit.
    # Default 30 s matches typical scan duration; lower for safety testing.
    max_continuous_on_s: float = Field(default=30.0, gt=0)
    # --- AA-29 end ---


class GeometryConfig(_Strict):
    attachment_pitch_mm: Millimeters = Field(gt=0)


class UsbConfig(_Strict):
    # Hard invariant (doc-1 s6): exactly one camera stream at a time.
    max_concurrent_camera_streams: Literal[1]


class HardwareConfig(_Strict):
    schema_version: int
    backend: BackendName = Field(strict=False)
    geometry: GeometryConfig
    usb: UsbConfig
    arms: ArmsConfig
    dexarm: DexArmConfig
    cameras: CamerasConfig
    relay: RelayConfig
    laser: LaserConfig

    @model_validator(mode="before")
    @classmethod
    def _check_schema_version(cls, data: object) -> object:
        if isinstance(data, dict) and "schema_version" in data:
            version = data["schema_version"]
            if version != SUPPORTED_SCHEMA_VERSION:
                raise ValueError(
                    f"unsupported schema_version {version!r}; "
                    f"this loader supports {SUPPORTED_SCHEMA_VERSION}"
                )
        return data


def _format_validation_error(source: str, err: ValidationError) -> str:
    lines = [f"invalid hardware config {source}:"]
    for item in err.errors():
        path = ".".join(str(part) for part in item["loc"]) or "<root>"
        if item["type"] == "extra_forbidden":
            msg = "unknown field"
        elif item["type"] == "missing":
            msg = "missing required field"
        else:
            msg = item["msg"]
        lines.append(f"  {path}: {msg}")
    return "\n".join(lines)


def parse_config(data: object, source: str = "<data>") -> HardwareConfig:
    """Validate an already-parsed mapping into a :class:`HardwareConfig`."""
    if not isinstance(data, dict):
        raise ConfigError(f"invalid hardware config {source}: top level must be a mapping")
    try:
        return HardwareConfig.model_validate(data)
    except ValidationError as err:
        raise ConfigError(_format_validation_error(source, err)) from None


def resolve_config_path(path: Union[str, Path, None] = None) -> Path:
    """Explicit path, else ``$INCHWORM_CONFIG``, else ``robot/config/hardware.yaml``."""
    if path is not None:
        return Path(path)
    env = os.environ.get(CONFIG_PATH_ENV_VAR)
    return Path(env) if env else DEFAULT_CONFIG_PATH


def load_config(path: Union[str, Path, None] = None) -> HardwareConfig:
    """Read and validate the hardware config file."""
    config_path = resolve_config_path(path)
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as err:
        raise ConfigError(f"cannot read hardware config {config_path}: {err}") from None
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as err:
        raise ConfigError(f"invalid YAML in hardware config {config_path}: {err}") from None
    return parse_config(data, source=str(config_path))
