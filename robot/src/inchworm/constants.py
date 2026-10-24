"""System constants, sourced once from the hardware config.

Other modules import these names instead of restating literals. The values
live in ``robot/config/hardware.yaml`` (the config named by
``$INCHWORM_CONFIG`` if set); this module only exposes them.
"""

from __future__ import annotations

from inchworm.config import load_config
from inchworm.units import Degrees, Millimeters, MillimetersPerPixel

_CONFIG = load_config()

ATTACHMENT_PITCH_MM: Millimeters = _CONFIG.geometry.attachment_pitch_mm

LASER_TO_CAMERA_ANGLE_DEG: Degrees = _CONFIG.laser.design.laser_to_camera_angle_deg
LASER_BASELINE_OFFSET_MM: Millimeters = _CONFIG.laser.design.baseline_offset_mm
LASER_STANDOFF_MM: Millimeters = _CONFIG.laser.design.standoff_mm
LASER_HEIGHT_SENSITIVITY_MM_PER_PX: MillimetersPerPixel = (
    _CONFIG.laser.design.height_sensitivity_mm_per_px
)

# Shared USB 2.0 uplink: only one camera may stream at a time (doc-1 s6).
MAX_CONCURRENT_CAMERA_STREAMS: int = _CONFIG.usb.max_concurrent_camera_streams
