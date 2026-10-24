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

    ``unverified`` is ``True`` when the bounds are a placeholder from the
    Rotrics specification and have not been measured against the as-built
    chassis envelope.  The driver logs a WARNING at connect when this flag
    is set; the hardware session must replace the values with the measured
    envelope and set ``unverified: false``.
    """

    x_min_mm: Millimeters
    x_max_mm: Millimeters
    y_min_mm: Millimeters
    y_max_mm: Millimeters
    z_min_mm: Millimeters
    z_max_mm: Millimeters
    # True when bounds are a spec placeholder; hardware session must measure
    # the real envelope and set this to false.
    unverified: bool = False

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


# --- AA-10 ---
class HomingNeutralPose(_Strict):
    """Named neutral pose for homing (degrees, STS3215 range 0-360).

    UNVERIFIED: placeholder values until a hardware session measures safe
    neutral positions for both arms. The homing routine refuses to run live
    (``--live`` flag required) if ``joint_limits_verified`` is ``False``.
    """

    shoulder_pan: Degrees
    shoulder_lift: Degrees
    elbow_flex: Degrees
    wrist_flex: Degrees
    wrist_roll: Degrees


class HomingConfig(_Strict):
    """Configuration for the AA-10 homing routine.

    All numeric fields are UNVERIFIED placeholders until measured on hardware.
    """

    neutral_pose_deg: HomingNeutralPose
    # True only after AA-9 joint limits are measured and populated.
    joint_limits_verified: bool = False
    # Velocity limit applied during homing (STS3215 goal_speed register 0x2E / 46).
    # Units: raw ticks/s. UNVERIFIED placeholder.
    goal_speed_ticks: int = Field(default=200, ge=0)
    # Acceleration limit applied during homing (STS3215 acceleration reg 0x29 / 41).
    # Units: raw ticks/s^2 (hardware-defined). UNVERIFIED placeholder.
    acceleration_ticks: int = Field(default=50, ge=0)
    # Torque limit during homing (STS3215 torque_limit register 0x30 / 48).
    # Units: raw load units (0-1000). UNVERIFIED placeholder.
    torque_limit_raw: int = Field(default=500, ge=0, le=1000)


class FaultThresholds(_Strict):
    """Thresholds for overcurrent, overtemperature and gripper locked-band detection.

    All values are UNVERIFIED placeholders until confirmed on hardware.
    The gripper locked-band fields are ``None`` when unknown; the homing routine
    treats ``None`` as "assume possibly locked → abort" (fail-closed).
    """

    # Overcurrent: raw present_current register value (STS3215 reg 0x45, u16).
    # UNVERIFIED placeholder.
    overcurrent_raw: int = Field(default=800, ge=0)
    # Overtemperature: degrees C (STS3215 present_temperature reg 0x3F).
    # UNVERIFIED placeholder.
    overtemperature_c: int = Field(default=70, ge=0)
    # Gripper locked band (degrees). null = unknown -> fail closed.
    gripper_locked_band_deg_low: Optional[Degrees] = None
    gripper_locked_band_deg_high: Optional[Degrees] = None
# --- AA-10 end ---
# --- AA-11 ---
from typing import List  # noqa: E402 – local import for the marked block


class SO101JointConfig(_Strict):
    """One revolute joint in the 5-DoF SO-101 kinematic chain.

    Values are taken verbatim from so101_new_calib.urdf (hash a758567c).
    ``origin_xyz`` is in metres; ``origin_rpy`` and ``limit_*_rad`` are in
    radians.  ``axis`` is always ``[0, 0, 1]`` for every joint in this URDF.
    """

    name: str
    parent: str
    child: str
    origin_xyz: List[float]
    origin_rpy: List[float]
    axis: List[int]
    limit_lower_rad: float
    limit_upper_rad: float


class SO101FrameOffset(_Strict):
    """Compact xyz+rpy representation of a rigid-body transform.

    ``unverified: true`` means the values are placeholders; the hardware
    session (AA-16) must measure and update them.
    """

    xyz: List[float]
    rpy: List[float]
    unverified: bool = False


class SO101TrajectoryLimits(_Strict):
    """Conservative joint-space trajectory limits for the SO-101 arms."""

    max_velocity_rad_per_s: float = Field(gt=0)
    max_acceleration_rad_per_s2: float = Field(gt=0)


class SO101KinematicsConfig(_Strict):
    """SO-101 5-DoF kinematic chain parameters loaded from hardware.yaml.

    The 5-DoF chain is shoulder_pan → shoulder_lift → elbow_flex →
    wrist_flex → wrist_roll.  The gripper joint is excluded; it is
    commanded only via the AA-17/AA-8 gripper_channel token.

    Parameters are sourced from the TheRobotStudio URDF
    (robot/vendor/so101/so101_new_calib.urdf, hash a758567c).  The
    tool-frame offset and base-to-chassis transforms are marked
    ``unverified: true`` until measured in AA-16.
    """

    joints: List[SO101JointConfig]
    tool_frame_offset: SO101FrameOffset
    left_arm_base_to_chassis: SO101FrameOffset
    right_arm_base_to_chassis: SO101FrameOffset
    trajectory: SO101TrajectoryLimits
# --- AA-11 end ---
# --- AA-15 ---


class AA15ArmApproachOffsets(_Strict):
    """Coarse-approach and insertion offsets for one arm's attachment interaction.

    Both offsets are expressed in the **attachment-point frame** (z-axis points
    into the tube along the attachment axis).  They describe where the TCP must
    be positioned *before* descending into the attachment hole.

    ``unverified: true`` means these are placeholder values; a hardware session
    (AA-16 calibration) must measure the real offsets and set ``unverified: false``.
    When ``unverified`` is ``True`` the frame model will refuse live motion (fail
    closed); dry-run mode still works.
    """

    # Offset from attachment-point origin to coarse-approach TCP target (metres).
    # UNVERIFIED placeholder — measure in AA-16.
    coarse_approach_offset_m: List[float]
    # Offset from attachment-point origin to insertion TCP target (metres).
    # UNVERIFIED placeholder — measure in AA-16.
    insertion_offset_m: List[float]
    unverified: bool = True


class AA15FramesConfig(_Strict):
    """AA-15 frame model configuration: per-arm approach offsets.

    See inchworm.frames for the full chassis/tube/lattice model.
    """

    left: AA15ArmApproachOffsets
    right: AA15ArmApproachOffsets


# --- AA-15 end ---


# --- AA-17 ---
class GripperBandConfig(_Strict):
    """Position bands (degrees) for one arm's custom gripper servo.

    All band boundaries are ``None`` until empirically measured in a live
    hardware session using ``scripts/gripper_bands.py --live``.  The AA-17
    state machine **refuses** every live lock or unlock when any band boundary
    is ``None`` (fail-closed).  Dry-run mode skips the null check.

    Measurement method: record present_position_deg while commanding the
    servo through its full range; observe three regions:
    - **unlocked**: jaw open, latch fully retracted.
    - **engaged_but_unlocked**: jaw contacting attachment but latch not seated.
    - **locked**: latch positively seated and holding.

    Mark ``unverified: true`` until confirmed on hardware; set to ``false``
    after the owner-supervised measurement session.
    """

    # unlocked band: [low_deg, high_deg]
    unlocked_low_deg: Optional[Degrees] = None   # unverified: true
    unlocked_high_deg: Optional[Degrees] = None  # unverified: true
    # engaged-but-unlocked band
    engaged_low_deg: Optional[Degrees] = None    # unverified: true
    engaged_high_deg: Optional[Degrees] = None   # unverified: true
    # locked band
    locked_low_deg: Optional[Degrees] = None     # unverified: true
    locked_high_deg: Optional[Degrees] = None    # unverified: true
    # True when any value above is a placeholder (not yet measured).
    unverified: bool = True


class GripperVerificationThresholds(_Strict):
    """Thresholds used by the AA-17 lock-verification logic.

    All values are ``None`` until measured; null -> state machine cannot
    distinguish latch/jam/miss in live mode (treated as FAULT).
    Mark ``unverified: true`` until confirmed on hardware.
    """

    # Minimum |load_raw| indicating the servo is under significant load.
    # Below this value while in the locked band -> probable miss (latch not engaged).
    jam_load_threshold_raw: Optional[int] = None   # unverified: true
    # Maximum |load_raw| expected for a true latch (not a jam).
    # Above this value while in the locked band -> probable jam.
    latch_load_max_raw: Optional[int] = None       # unverified: true
    # Maximum current_raw for a true latch.  Spike above -> probable jam.
    latch_current_max_raw: Optional[int] = None    # unverified: true
    # True when any value above is a placeholder (not yet measured).
    unverified: bool = True


class GripperLockConfig(_Strict):
    """Per-arm gripper lock configuration for the AA-17 state machine.

    ``bands`` is ``None`` until the owner runs the measurement script;
    the state machine treats ``None`` bands as fail-closed.
    ``drift_threshold_deg`` is the maximum allowed position drift before
    the hold-monitor re-asserts (UNVERIFIED placeholder).
    """

    # Per-arm band config: keyed by arm role ('arm_left', 'arm_right').
    # Each value is a GripperBandConfig, or None if not yet measured.
    arm_left: Optional[GripperBandConfig] = None
    arm_right: Optional[GripperBandConfig] = None
    # Drift threshold for active hold-monitor (degrees). UNVERIFIED placeholder.
    drift_threshold_deg: Optional[Degrees] = None   # unverified: true
    # Thresholds for latch/jam/miss classification. UNVERIFIED placeholders.
    verification: Optional[GripperVerificationThresholds] = None
# --- AA-17 end ---


# --- AA-9 ---
class SoftJointLimits(_Strict):
    """Soft per-joint travel limits for one arm (radians).

    ``null`` values mean the limits have not been measured; the guard
    falls back to the URDF limits from ``so101_kinematics`` but marks them
    as unverified and refuses LIVE motion until real limits with
    ``method`` and ``measured_date`` are recorded.

    ``unverified`` must be ``False`` before LIVE motion is allowed.  It is
    set by the owner after running ``scripts/measure_limits.py --live``.
    """

    shoulder_pan_lower_rad: Optional[float] = None
    shoulder_pan_upper_rad: Optional[float] = None
    shoulder_lift_lower_rad: Optional[float] = None
    shoulder_lift_upper_rad: Optional[float] = None
    elbow_flex_lower_rad: Optional[float] = None
    elbow_flex_upper_rad: Optional[float] = None
    wrist_flex_lower_rad: Optional[float] = None
    wrist_flex_upper_rad: Optional[float] = None
    wrist_roll_lower_rad: Optional[float] = None
    wrist_roll_upper_rad: Optional[float] = None
    # True until a hardware session with measure_limits.py --live populates
    # all fields and sets this to False.
    unverified: bool = True
    # How limits were measured and when. null = not yet measured.
    method: Optional[str] = None
    measured_date: Optional[str] = None


class KeepOutBox(_Strict):
    """Axis-aligned keep-out box in the chassis fixed frame (metres).

    Poses whose FK link points fall inside this box are rejected.
    ``unverified: true`` means the bounds are a conservative placeholder;
    the owner must measure and set ``unverified: false``.
    """

    label: str
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float
    unverified: bool = True

    @model_validator(mode="after")
    def _ranges_positive(self) -> "KeepOutBox":
        for axis in ("x", "y", "z"):
            lo = getattr(self, f"{axis}_min_m")
            hi = getattr(self, f"{axis}_max_m")
            if lo >= hi:
                raise ValueError(f"{axis}_min_m ({lo}) must be < {axis}_max_m ({hi})")
        return self


class KeepOutCapsule(_Strict):
    """Capsule keep-out primitive in the chassis fixed frame (metres).

    Defined as the swept sphere of radius ``radius_m`` along the segment
    from ``p0_xyz`` to ``p1_xyz``.  Any FK link point within ``radius_m`` of
    the segment is rejected.
    ``unverified: true`` means the values are a conservative placeholder.
    """

    label: str
    p0_xyz: List[float]  # [x, y, z] metres
    p1_xyz: List[float]  # [x, y, z] metres
    radius_m: float
    unverified: bool = True


class NamedPose(_Strict):
    """A named joint configuration (radians, 5-DoF, proximal→distal).

    ``unverified: true`` means the angles are a placeholder; the guard
    refuses to use this pose for real motion until
    ``unverified`` is ``False`` and the pose passes validation.
    """

    name: str
    q_rad: List[float]  # [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
    description: str = ""
    unverified: bool = True

    @model_validator(mode="after")
    def _five_dof(self) -> "NamedPose":
        if len(self.q_rad) != 5:
            raise ValueError(
                f"Named pose '{self.name}' must have exactly 5 joint angles, "
                f"got {len(self.q_rad)}"
            )
        return self


class CollisionConfig(_Strict):
    """Collision model configuration for the AA-9 motion guard.

    All keep-out primitives are UNVERIFIED placeholders.  A hardware session
    with measured arm dimensions is required to replace them with real values.

    Sampling resolution for trajectory checks: the straight-line joint-space
    interpolation is sampled at ``interpolation_samples`` evenly-spaced points
    (including endpoints).  50 samples is the documented default; decrease for
    speed, increase for tighter safety.
    """

    # Dense sampling of straight-line joint interpolations for collision check.
    # 50 samples -> step ≤ max_joint_range/50 ≈ 0.11 rad for the widest joint.
    # DOCUMENTED: this is the guaranteed check resolution.
    interpolation_samples: int = Field(default=50, ge=2)
    # Keep-out boxes in the chassis fixed frame.
    keep_out_boxes: List[KeepOutBox] = Field(default_factory=list)
    # Keep-out capsules in the chassis fixed frame.
    keep_out_capsules: List[KeepOutCapsule] = Field(default_factory=list)


class JointServoCalibration(_Strict):
    """Per-joint servo calibration for one arm.

    For each of the five joints, records the STS3215 servo degree value that
    corresponds to URDF joint_rad = 0 (``zero_offset_deg``) and the sign of
    the mapping (``sign``: +1 if increasing servo_deg → increasing joint_rad,
    -1 otherwise).

    All fields default to ``None`` (not yet measured).  ``unverified: true``
    until an owner-supervised calibration session measures and confirms them.
    Live conversion (via :mod:`inchworm.joint_calibration`) **fails closed**
    when any field is null or when ``unverified`` is True.
    """

    # --- shoulder_pan ---
    shoulder_pan_zero_offset_deg: Optional[float] = None   # unverified: true
    shoulder_pan_sign: Optional[int] = None                # unverified: true
    # --- shoulder_lift ---
    shoulder_lift_zero_offset_deg: Optional[float] = None  # unverified: true
    shoulder_lift_sign: Optional[int] = None               # unverified: true
    # --- elbow_flex ---
    elbow_flex_zero_offset_deg: Optional[float] = None     # unverified: true
    elbow_flex_sign: Optional[int] = None                  # unverified: true
    # --- wrist_flex ---
    wrist_flex_zero_offset_deg: Optional[float] = None     # unverified: true
    wrist_flex_sign: Optional[int] = None                  # unverified: true
    # --- wrist_roll ---
    wrist_roll_zero_offset_deg: Optional[float] = None     # unverified: true
    wrist_roll_sign: Optional[int] = None                  # unverified: true
    # True until a hardware session measures and sets all fields, then False.
    unverified: bool = True
    # How/when calibration was measured. null = not yet measured.
    method: Optional[str] = None
    measured_date: Optional[str] = None


class ServoCalibrationConfig(_Strict):
    """Per-arm servo calibration blocks for both SO-101 arms.

    Used by :mod:`inchworm.joint_calibration` to convert between STS3215
    servo degrees and URDF joint radians.  Both blocks default to ``None``
    (no calibration measured yet); live motion is blocked until both are
    populated with ``unverified: false``.
    """

    left_arm: Optional[JointServoCalibration] = None
    right_arm: Optional[JointServoCalibration] = None


class MotionGuardConfig(_Strict):
    """AA-9 motion guard configuration.

    Contains per-arm soft joint limits, named poses, the collision model,
    and the per-joint servo calibration (servo-degree ↔ joint-radian mapping).
    If this block is absent from hardware.yaml, the guard uses URDF limits
    (from ``so101_kinematics``) and marks everything as unverified.
    """

    # Per-arm soft limits. null arms → fall back to URDF (unverified).
    left_arm_limits: Optional[SoftJointLimits] = None
    right_arm_limits: Optional[SoftJointLimits] = None
    # Collision model (primitives + sampling config).
    collision: CollisionConfig = Field(default_factory=CollisionConfig)
    # Named poses used by the motion planner (neutral, folded_transit, etc.).
    named_poses: List[NamedPose] = Field(default_factory=list)
    # Per-joint servo calibration (servo_deg ↔ joint_rad conversion).
    # null = not yet measured; live conversion blocked until unverified: false.
    servo_calibration: Optional[ServoCalibrationConfig] = None
# --- AA-9 end ---
# --- AA-20 ---


class PolicySupervisorConfig(_Strict):
    """Configuration for the AA-20 policy supervisor (PolicySupervisor).

    All numeric limits are documented safe defaults; override in
    ``hardware.yaml`` for the specific deployment.

    ``workspace_radius_m``
        Radius of the Cartesian sphere centred on the target attachment
        point (in arm base frame) within which the TCP must remain.
        UNVERIFIED placeholder — tighten after AA-16 calibration.

    ``retract_pose_q_rad``
        5-element joint configuration (radians, shoulder_pan first) the
        supervisor commands on abort.  UNVERIFIED placeholder — must be
        verified on hardware before live use.

    ``max_action_magnitude_deg``
        Per-step action magnitude clamp threshold (degrees).  Policy
        outputs whose joint-space Euclidean norm exceeds this value are
        *scaled down* to this magnitude before execution.  A WARNING is
        logged on each clamp.

    ``clamp_on_magnitude_violation``
        When ``True`` (default), magnitude violations are clamped and
        logged rather than triggering an abort.  Set to ``False`` to
        abort instead.
    """

    # Maximum number of policy steps before abort.
    max_steps: int = Field(default=200, ge=1)
    # Maximum wall-clock time (seconds) before abort.
    max_wall_time_s: float = Field(default=30.0, gt=0)
    # Cartesian workspace radius around target attachment point (metres).
    # UNVERIFIED placeholder — measure in AA-16.
    workspace_radius_m: float = Field(default=0.15, gt=0)
    # Per-step maximum action magnitude (degrees, Euclidean norm over 5 joints).
    # UNVERIFIED placeholder.
    max_action_magnitude_deg: float = Field(default=5.0, gt=0)
    # When True, clamp magnitude violations; when False, abort instead.
    clamp_on_magnitude_violation: bool = True
    # Maximum retry attempts before halting the cycle.
    max_retries: int = Field(default=2, ge=0)
    # Safe retract pose (radians, 5-DoF).  UNVERIFIED placeholder.
    # null → supervisor logs a WARNING and skips retract in dry-run,
    # fails closed (raises) in live mode.
    retract_pose_q_rad: Optional[List[float]] = None
    # Flag: must be set to False after retract pose is verified on hardware.
    retract_pose_unverified: bool = True


# --- AA-20 end ---


# --- AA-19 ---
class FoldWaypoint(_Strict):
    """One intermediate waypoint in the fold or unfold sequence (radians, 5-DoF).

    ``q_rad`` is ordered [shoulder_pan, shoulder_lift, elbow_flex,
    wrist_flex, wrist_roll], matching JOINT_NAMES in motion_guard.py.

    ``description`` documents *why* this waypoint exists (clearance,
    joint ordering rationale, etc.).

    ``unverified: true`` until a hardware session confirms the waypoint
    is collision-free and within safe joint limits.  Live motion FAILS
    CLOSED on unverified waypoints.
    """

    q_rad: List[float]  # 5-DoF
    description: str = ""
    unverified: bool = True

    @model_validator(mode="after")
    def _five_dof(self) -> "FoldWaypoint":
        if len(self.q_rad) != 5:
            raise ValueError(
                f"FoldWaypoint must have exactly 5 joint angles, got {len(self.q_rad)}"
            )
        return self


class FoldSequenceConfig(_Strict):
    """AA-19 fold/unfold sequence configuration.

    Both ``fold_waypoints`` and ``unfold_waypoints`` are lists of
    :class:`FoldWaypoint`.  Each waypoint is individually collision-checked
    before execution.  The sequence is executed in list order; joints are
    NOT moved simultaneously — each waypoint is a complete arm configuration.

    ``unverified: true`` on any waypoint means live execution is blocked
    for that waypoint until an owner-supervised hardware session confirms it.
    """

    # Ordered intermediate waypoints on the way TO folded_transit.
    # The final waypoint is implicitly the folded_transit named pose from AA-9.
    # ALL UNVERIFIED PLACEHOLDERS — owner must verify on hardware.
    fold_waypoints: List[FoldWaypoint] = Field(default_factory=list)
    # Ordered intermediate waypoints on the way FROM folded_transit.
    # The start pose is implicitly the current arm position.
    # ALL UNVERIFIED PLACEHOLDERS.
    unfold_waypoints: List[FoldWaypoint] = Field(default_factory=list)
# --- AA-19 end ---

# --- AA-31 ---


class LaserLineConfigSchema(_Strict):
    """Thresholds for laser-line extraction (AA-31).

    Loaded from the ``laser_line`` block in hardware.yaml.  All values are
    **unverified** defaults; calibrate against real scanner frames.

    Field semantics match :class:`inchworm.laser_line.LaserLineConfig`.
    """

    # Half-width (px) of the per-column search window. unverified: true
    window_half_width_px: int = Field(default=15, ge=1)
    # Min count of 255-valued rows to trigger SATURATED. unverified: true
    saturated_plateau_px: int = Field(default=2, ge=1)
    # Min SNR below which a column is WEAK. unverified: true
    weak_snr_threshold: float = Field(default=3.0, gt=0)
    # Fraction of peak above which a second peak triggers MULTI_PEAK. unverified: true
    multi_peak_fraction: float = Field(default=0.4, gt=0, lt=1)
    # Absolute minimum pixel value for the line to be considered present. unverified: true
    min_peak_value: int = Field(default=20, ge=0)
    # Rows used for background: 0 = all rows outside the window.
    background_rows: int = Field(default=0, ge=0)


# --- AA-31 end ---


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
    # --- AA-10 ---
    homing: Optional["HomingConfig"] = None
    fault_thresholds: Optional["FaultThresholds"] = None
    # --- AA-10 end ---
    # --- AA-11 ---
    so101_kinematics: SO101KinematicsConfig
    # --- AA-11 end ---
    # --- AA-17 ---
    gripper_lock: Optional[GripperLockConfig] = None
    # --- AA-17 end ---
    # --- AA-15 ---
    frames_model: Optional["AA15FramesConfig"] = None
    # --- AA-15 end ---
    # --- AA-9 ---
    motion_guard: Optional["MotionGuardConfig"] = None
    # --- AA-9 end ---
    # --- AA-20 ---
    policy_supervisor: Optional["PolicySupervisorConfig"] = None
    # --- AA-20 end ---
    # --- AA-19 ---
    fold_sequence: Optional["FoldSequenceConfig"] = None
    # --- AA-19 end ---
    # --- AA-31 ---
    laser_line: Optional[LaserLineConfigSchema] = None
    # --- AA-31 end ---

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
