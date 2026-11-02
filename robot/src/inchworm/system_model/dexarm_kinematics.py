"""
dexarm_kinematics.py
====================
Rotrics DexArm firmware kinematics and its mapping onto the owner cell URDF.

Source: Rotrics-Dev/Marlin_For_DexArm (GitHub), files
``Marlin/src/module/dexarm/dexarm.cpp`` (``forward_kinematics_DEXARM``,
``inverse_kinematics_dexarm``, ``dexarm_apply_leveling``) and
``dexarm_position_config.h`` (constants). Read 2026-10-28; firmware on this
DexArm reports "Marlin 2.0.5.4", module "3D" (FRONT_MODULE_OFFSET 92.60).

Firmware angles ("DEXARM Theta A/B/C" in the M114 reply, degrees):
    l1 = 150 cos(B0 - b) + 150 sin(C0 + c) + offset
    z  = 150 sin(B0 - b) - 150 cos(C0 + c)
    x  = l1 sin(a),  y = l1 cos(a)
with B0 = 105.24, C0 = 99.60, offset = module offset + 12.5 (OFFSET_ROT).
Verified against this DexArm 2026-10-28: Theta (-2.29, 12.57, -79.63) gives
(-5.97, 149.22, 8.86) by this formula vs the firmware's M895 (-5.96, 149.45,
8.87) and M114 X/Y (-5.95, 149.22).

Leveling: commanded Z is shifted by ``sx*x + sy*(y-300)`` before IK
(firmware M891/M892 slopes, stored in DexArm EEPROM).

Mapping to the cell URDF (``CAD/robot_urdf_new``) joints, fitted 2026-10-28
by least squares over 60 random firmware angle triples (wrist point vs the
URDF ``EE`` joint origin, plus a constant offset in the rotating frame):
    bases       = -a
    B_Rot->In_1 =  b - 19.4968 deg
    Join->In_2  = -c - 32.378  deg
residual 0.10 mm RMS, 0.31 mm max. The URDF tool (the community gripper
module) is an approximation of the real 3D print head + laser camera (owner,
2026-10-27), so the mapping fixes the ARM links, not the tool shape.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

ARM_LINK_MM = 150.0
START_B_DEG = 105.24
START_C_DEG = 99.60
OFFSET_ROT_MM = 12.5
MODULE_OFFSET_3D_MM = 92.6
LEVELING_Y_REF_MM = 300.0
Z_MIN_MM, Z_MAX_MM = -127.0, 167.0  # firmware ZMINARM / ZMAXARM

# Firmware angle -> cell URDF joint (rad) = sign * radians(theta) + offset
_MAP = {
    "bases": (-1.0, 0.0),
    "B_Rot->In_1": (1.0, math.radians(-19.4968)),
    "Join->In_2": (-1.0, math.radians(-32.378)),
}


class DexArmKinematicsError(ValueError):
    """Target is unreachable for the firmware IK (it would refuse or misbehave)."""


@dataclass(frozen=True)
class DexArmAngles:
    a: float  # base rotation, deg
    b: float  # upper arm, deg
    c: float  # forearm, deg


def module_offset_mm(module_offset: float = MODULE_OFFSET_3D_MM) -> float:
    return module_offset + OFFSET_ROT_MM


def forward(angles: DexArmAngles, module_offset: float = MODULE_OFFSET_3D_MM) -> Tuple[float, float, float]:
    """Firmware FK (no leveling): tool point (x, y, z) mm in the DexArm frame."""
    b0 = math.radians(START_B_DEG - angles.b)
    c0 = math.radians(START_C_DEG + angles.c)
    l1 = ARM_LINK_MM * math.cos(b0) + ARM_LINK_MM * math.sin(c0) + module_offset_mm(module_offset)
    z = ARM_LINK_MM * math.sin(b0) - ARM_LINK_MM * math.cos(c0)
    a = math.radians(angles.a)
    return l1 * math.sin(a), l1 * math.cos(a), z


def leveling_dz(x: float, y: float, slopes: Tuple[float, float]) -> float:
    return slopes[0] * x + slopes[1] * (y - LEVELING_Y_REF_MM)


def inverse(
    x: float, y: float, z: float,
    slopes: Tuple[float, float],
    module_offset: float = MODULE_OFFSET_3D_MM,
) -> DexArmAngles:
    """Firmware IK for a commanded (logical) position, leveling applied as the firmware does."""
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise DexArmKinematicsError("non-finite target")
    if z < Z_MIN_MM or z > Z_MAX_MM:
        raise DexArmKinematicsError(f"Z {z} outside firmware range [{Z_MIN_MM}, {Z_MAX_MM}]")
    if y < 0:
        raise DexArmKinematicsError("Y < 0 is refused by the firmware")
    zl = z + leveling_dz(x, y, slopes)
    r = math.hypot(x, y)
    off = module_offset_mm(module_offset)
    if r <= off:
        raise DexArmKinematicsError(f"target radius {r:.1f} mm inside the module offset {off:.1f} mm")
    xs, ys = x - x * off / r, y - y * off / r
    ys = max(ys, 0.01)
    if x == 0:
        rot = 90.0
    elif x < 0:
        rot = -math.degrees(math.atan(ys / xs))
    else:
        rot = 180.0 - math.degrees(math.atan(ys / xs))
    s = math.hypot(xs, ys)
    d = math.sqrt(xs * xs + ys * ys + zl * zl)
    if d > 2 * ARM_LINK_MM:
        raise DexArmKinematicsError(f"target {d:.1f} mm from the shoulder, beyond reach {2*ARM_LINK_MM}")
    thb1 = math.degrees(math.acos(d / 2 / ARM_LINK_MM))
    thb2 = math.degrees(math.atan(zl / s))
    left = thb1 + thb2
    right = 90.0 + (thb2 - thb1)
    # firmware: delta = (rot - START_A, START_B - left, right - START_C); START_A = 90
    return DexArmAngles(a=rot - 90.0, b=START_B_DEG - left, c=right - START_C_DEG)


def to_cell_joints(angles: DexArmAngles) -> Dict[str, float]:
    vals = {"bases": angles.a, "B_Rot->In_1": angles.b, "Join->In_2": angles.c}
    return {j: s * math.radians(vals[j]) + o for j, (s, o) in _MAP.items()}


def parse_m114_angles(lines) -> Optional[DexArmAngles]:
    """Theta A/B/C from an M114 reply ('DEXARM Theta A:-2.00  Theta B:12.00  Theta C:-79.00')."""
    import re
    rx = re.compile(r"Theta A:\s*(-?[\d.]+)\s+Theta B:\s*(-?[\d.]+)\s+Theta C:\s*(-?[\d.]+)")
    for line in lines:
        m = rx.search(line)
        if m:
            return DexArmAngles(*(float(m.group(i)) for i in (1, 2, 3)))
    return None
