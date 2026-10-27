"""
arm_placer.py
=============
Derives each SO-101 arm's base_link pose in the STEP/chassis frame.

Method
------
1. Parse STEP placements (step_parser.parse_step_placements).
2. Identify the two Base_SO101 v12 instances by name; sort by Y to assign
   LEFT (nearer DexArm, lower Y ≈ 204 mm) and RIGHT (higher Y ≈ 386 mm).
3. Compute the offset between the CAD part origin (= STEP placement) and the
   URDF base_link origin using bounding-box alignment of base_so101_v2.stl.
4. Return T_world_baselink (4×4, mm) for both arms and the residual info.

Validation
----------
Forward-kinematics at the owner-measured folded pose q = [0, −1.25, 1.64, −1.31, 1.36] rad
should produce link mesh centroids close to the CAD bboxes in the collision report.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Rotation / transform helpers
# ---------------------------------------------------------------------------

def _rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    """XYZ intrinsic Euler angles → 3×3 rotation matrix."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _make_tf(xyz_m: np.ndarray, rpy: Tuple[float, float, float]) -> np.ndarray:
    """Build a 4×4 transform from xyz (metres) and rpy (rad)."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz_m
    return T


def _tf_mm(xyz_mm: np.ndarray, rpy: Tuple[float, float, float]) -> np.ndarray:
    """Build a 4×4 transform from xyz (mm) and rpy (rad)."""
    return _make_tf(xyz_mm / 1000.0, rpy)


def compose_transforms(parent: np.ndarray, child: np.ndarray) -> np.ndarray:
    """Compose two 4×4 transforms: T_parent @ T_child."""
    return parent @ child


def invert_tf(T: np.ndarray) -> np.ndarray:
    """Invert a rigid-body 4×4 transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# ---------------------------------------------------------------------------
# URDF kinematics (FK)
# ---------------------------------------------------------------------------

# URDF joint parameters parsed statically.
# Joint order for the 5 revolute joints (shoulder_pan .. wrist_roll + gripper):
# Index: 0=shoulder_pan, 1=shoulder_lift, 2=elbow_flex, 3=wrist_flex, 4=wrist_roll
# (gripper is joint 5, not part of the planning chain)

# Each joint: (xyz_m, rpy, axis_unit_vec) in parent link frame
_URDF_JOINTS = {
    "shoulder_pan": {
        "parent": "base_link",
        "child": "shoulder_link",
        "xyz": np.array([0.0388353, -8.97657e-09, 0.0624]),
        "rpy": (math.pi, 4.18253e-17, -math.pi),
        "axis": np.array([0.0, 0.0, 1.0]),
    },
    "shoulder_lift": {
        "parent": "shoulder_link",
        "child": "upper_arm_link",
        "xyz": np.array([-0.0303992, -0.0182778, -0.0542]),
        "rpy": (-math.pi / 2, -math.pi / 2, 0.0),
        "axis": np.array([0.0, 0.0, 1.0]),
    },
    "elbow_flex": {
        "parent": "upper_arm_link",
        "child": "lower_arm_link",
        "xyz": np.array([-0.11257, -0.028, 1.73763e-16]),
        "rpy": (-3.63608e-16, 8.74301e-16, math.pi / 2),
        "axis": np.array([0.0, 0.0, 1.0]),
    },
    "wrist_flex": {
        "parent": "lower_arm_link",
        "child": "wrist_link",
        "xyz": np.array([-0.1349, 0.0052, 3.62355e-17]),
        "rpy": (4.02456e-15, 8.67362e-16, -math.pi / 2),
        "axis": np.array([0.0, 0.0, 1.0]),
    },
    "wrist_roll": {
        "parent": "wrist_link",
        "child": "gripper_link",
        "xyz": np.array([5.55112e-17, -0.0611, 0.0181]),
        "rpy": (math.pi / 2, 0.0486795, math.pi),
        "axis": np.array([0.0, 0.0, 1.0]),
    },
}

# Joint names in order (matching owner q vector convention)
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Link names in FK order (starting from base_link)
LINK_NAMES = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link",
              "wrist_link", "gripper_link"]


def _rot_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula → 3×3 matrix."""
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def forward_kinematics(
    T_world_base: np.ndarray,
    q: List[float],
) -> Dict[str, np.ndarray]:
    """Compute forward kinematics for the SO-101 arm.

    Parameters
    ----------
    T_world_base : 4×4 array
        Transform from world (STEP/chassis) frame to URDF base_link, in METRES.
    q : list of 5 floats
        Joint angles [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
        in radians (URDF convention, positive = rotation around +Z of joint frame).

    Returns
    -------
    dict : {link_name: T_world_link (4×4, metres)}
    """
    assert len(q) == 5, f"Expected 5 joint angles, got {len(q)}"

    frames: Dict[str, np.ndarray] = {"base_link": T_world_base.copy()}

    current_T = T_world_base.copy()
    for i, jname in enumerate(JOINT_NAMES):
        jd = _URDF_JOINTS[jname]
        xyz = jd["xyz"]
        rpy = jd["rpy"]
        axis = jd["axis"]
        angle = q[i]

        # Fixed part of joint transform (from parent link to joint frame at q=0)
        T_fixed = _make_tf(xyz, rpy)

        # Revolute joint rotation around axis
        R_joint = _rot_axis_angle(axis, angle)
        T_revolute = np.eye(4)
        T_revolute[:3, :3] = R_joint

        # T_world_child = T_world_parent @ T_fixed @ T_revolute
        T_child = current_T @ T_fixed @ T_revolute
        frames[jd["child"]] = T_child
        current_T = T_child

    return frames


# ---------------------------------------------------------------------------
# Bounding-box alignment: find T_world_baselink from CAD placement
# ---------------------------------------------------------------------------

def _mesh_bounds_in_urdf_base_link(
    stl_path: Path,
    visual_origin_xyz_m: np.ndarray,
    visual_origin_rpy: Tuple[float, float, float],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (bbox_min, bbox_max) of an STL mesh transformed by its URDF visual origin,
    in the URDF base_link frame, in metres.
    """
    import trimesh  # optional dep
    mesh = trimesh.load(str(stl_path))
    T = _make_tf(visual_origin_xyz_m, visual_origin_rpy)
    mesh_copy = mesh.copy()
    mesh_copy.apply_transform(T)
    return mesh_copy.bounds


def compute_T_world_baselink_from_cad(
    cad_part_origin_mm: np.ndarray,
    vendor_dir: Path,
) -> Tuple[np.ndarray, dict]:
    """Compute T_world_baselink (4×4, metres) from the CAD part placement.

    Uses bounding-box alignment of base_so101_v2.stl to find the offset between
    the STEP part origin and the URDF base_link joint frame.

    Parameters
    ----------
    cad_part_origin_mm : (3,) array in mm
        STEP placement origin of the Base_SO101 v12 instance.
    vendor_dir : Path
        Path to robot/vendor/so101/ containing assets/.

    Returns
    -------
    T_world_baselink : 4×4 transform in metres
    info : dict with alignment details

    Rotation convention
    -------------------
    Both arms are assembled with URDF +X pointing in the STEP −Y direction
    (toward the DexArm), confirmed by the observation that the shoulder_link
    bbox centre is at STEP Y < base_Y (≈ 25 mm below the base).  This
    corresponds to a world-to-base_link rotation of Rz(−90°).

    T_world_baselink.R = Rz(−90°):  URDF X → STEP −Y, URDF Y → STEP +X.

    Translation
    -----------
    Derived by bbox-min alignment of base_so101_v2.stl.  The URDF base_link
    bounds in STEP frame (after Rz(−90°)) are offset from the CAD local bounds
    by approximately (+31.4, +2.8) mm in (Y, Z).  So:

        P_urdf = P_cad + (cad_local_min − urdf_rotated_min)
    """
    stl_path = vendor_dir / "assets" / "base_so101_v2.stl"

    # base_so101_v2 visual origin in URDF base_link frame (from URDF)
    vis_xyz = np.array([-0.00636471, -8.97657e-09, -0.0024])
    vis_rpy = (math.pi / 2, 0.0, math.pi / 2)

    # Mesh bounds in base_link frame (metres)
    bl_min, bl_max = _mesh_bounds_in_urdf_base_link(stl_path, vis_xyz, vis_rpy)

    # Convert to mm
    bl_min_mm = bl_min * 1000.0
    bl_max_mm = bl_max * 1000.0

    # The URDF base_link is rotated Rz(−90°) relative to the STEP world frame:
    # URDF X → STEP −Y, URDF Y → STEP +X, URDF Z → STEP +Z
    # Transform all 8 corners by Rz(−90°)
    corners_urdf = np.array([
        [bl_min_mm[0], bl_min_mm[1], bl_min_mm[2]],
        [bl_max_mm[0], bl_min_mm[1], bl_min_mm[2]],
        [bl_min_mm[0], bl_max_mm[1], bl_min_mm[2]],
        [bl_max_mm[0], bl_max_mm[1], bl_min_mm[2]],
        [bl_min_mm[0], bl_min_mm[1], bl_max_mm[2]],
        [bl_max_mm[0], bl_min_mm[1], bl_max_mm[2]],
        [bl_min_mm[0], bl_max_mm[1], bl_max_mm[2]],
        [bl_max_mm[0], bl_max_mm[1], bl_max_mm[2]],
    ])
    # Rz(−90°): (x,y) → (y,−x)
    Rzn90 = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)
    corners_step = (Rzn90 @ corners_urdf.T).T
    urdf_in_step_min = corners_step.min(axis=0)
    urdf_in_step_max = corners_step.max(axis=0)

    # CAD bbox in part local frame (from collision report)
    cad_local_min = np.array([-55.5, -33.2, 0.4])   # mm
    cad_local_max = np.array([55.5, 54.1, 72.6])     # mm

    # Correct formula:
    # P_urdf.{axis} + urdf_in_step_min.{axis} = P_cad.{axis} + cad_local_min.{axis}
    # => offset = P_urdf - P_cad = cad_local_min - urdf_in_step_min
    offset_mm = cad_local_min - urdf_in_step_min

    # URDF base_link origin in STEP world frame
    P_urdf_mm = cad_part_origin_mm + offset_mm

    # Build T_world_baselink (STEP frame is the world frame)
    # Rotation: Rz(−90°) maps URDF axes to STEP world
    rpy_world_base = (0.0, 0.0, -math.pi / 2)
    T = _tf_mm(P_urdf_mm, rpy_world_base)

    info = {
        "cad_part_origin_mm": cad_part_origin_mm.tolist(),
        "urdf_baselink_origin_mm": P_urdf_mm.tolist(),
        "offset_mm": offset_mm.tolist(),
        "urdf_in_step_min_mm": urdf_in_step_min.tolist(),
        "urdf_in_step_max_mm": urdf_in_step_max.tolist(),
        "cad_local_min_mm": cad_local_min.tolist(),
        "cad_local_max_mm": cad_local_max.tolist(),
        "rpy_world_base_deg": (0.0, 0.0, -90.0),
        "note": "Rz(-90 deg): URDF +X = STEP -Y (arm points toward DexArm at q=0)",
    }
    return T, info


# ---------------------------------------------------------------------------
# Link-mesh centroid: compute where each URDF link mesh centroid is in world frame
# ---------------------------------------------------------------------------

# URDF visual origins for each link's primary structural mesh (m, rpy)
# (only the structural SO-101 part mesh, not servo/hardware)
_URDF_LINK_MESHES = {
    "base_link": {
        "file": "base_so101_v2.stl",
        "xyz": np.array([-0.00636471, -8.97657e-09, -0.0024]),
        "rpy": (math.pi / 2, 0.0, math.pi / 2),
    },
    "shoulder_link": {
        "file": "rotation_pitch_so101_v1.stl",
        # URDF: xyz=0.0122008 2.22413e-05 0.0464, rpy=-1.5708 0 0
        "xyz": np.array([0.0122008, 2.22413e-05, 0.0464]),
        "rpy": (-math.pi / 2, 0.0, 0.0),
    },
    "upper_arm_link": {
        "file": "upper_arm_so101_v1.stl",
        # URDF: xyz=-0.065085 0.012 0.0182, rpy=3.14159 0 0
        "xyz": np.array([-0.065085, 0.012, 0.0182]),
        "rpy": (math.pi, 0.0, 0.0),
    },
    "lower_arm_link": {
        "file": "under_arm_so101_v1.stl",
        # URDF: xyz=-0.0648499 -0.032 0.0182, rpy=3.14159 0 0
        "xyz": np.array([-0.0648499, -0.032, 0.0182]),
        "rpy": (math.pi, 0.0, 0.0),
    },
    "wrist_link": {
        "file": "wrist_roll_pitch_so101_v2.stl",
        # URDF: xyz=0 -0.028 0.0181, rpy=-1.5708 -1.5708 0
        "xyz": np.array([0.0, -0.028, 0.0181]),
        "rpy": (-math.pi / 2, -math.pi / 2, 0.0),
    },
}


@dataclass
class ArmPlacements:
    """World placements for both SO-101 arms in the STEP/chassis frame."""
    T_world_left_mm: np.ndarray    # 4×4, translation in mm (non-standard but consistent with report)
    T_world_right_mm: np.ndarray
    left_info: dict
    right_info: dict
    left_cad_origin_mm: np.ndarray
    right_cad_origin_mm: np.ndarray


def extract_arm_placements(
    step_path: Path,
    vendor_dir: Path,
) -> ArmPlacements:
    """Extract both arm base_link placements from the STEP file.

    Returns :class:`ArmPlacements` with T_world_left_mm and T_world_right_mm
    (4×4 transforms, translation column in mm, rotation in world/STEP frame).

    Left arm = nearer DexArm (lower STEP Y ≈ 204 mm).
    Right arm = further from DexArm (higher STEP Y ≈ 386 mm).
    """
    from .step_parser import parse_step_placements

    placements = parse_step_placements(step_path)
    base_instances = [p for p in placements if "Base_SO101 v12" in p.component_name]

    if len(base_instances) != 2:
        raise ValueError(
            f"Expected 2 Base_SO101 v12 instances, found {len(base_instances)}"
        )

    # Sort by Y: lower Y = left (nearer DexArm)
    sorted_instances = sorted(base_instances, key=lambda p: p.origin_mm[1])
    left_inst = sorted_instances[0]
    right_inst = sorted_instances[1]

    T_left, left_info = compute_T_world_baselink_from_cad(
        left_inst.origin_mm.copy(), vendor_dir
    )
    T_right, right_info = compute_T_world_baselink_from_cad(
        right_inst.origin_mm.copy(), vendor_dir
    )

    # Convert to mm-unit transforms (scale translation only)
    def to_mm(T):
        T_mm = T.copy()
        T_mm[:3, 3] *= 1000.0
        return T_mm

    return ArmPlacements(
        T_world_left_mm=to_mm(T_left),
        T_world_right_mm=to_mm(T_right),
        left_info=left_info,
        right_info=right_info,
        left_cad_origin_mm=left_inst.origin_mm.copy(),
        right_cad_origin_mm=right_inst.origin_mm.copy(),
    )


# ---------------------------------------------------------------------------
# FK overlay check: compute per-link centroid error vs CAD bboxes
# ---------------------------------------------------------------------------

# CAD bboxes (world frame, mm) for arm A (left, lower Y) at folded pose:
# From collision report table:
_ARM_A_CAD_CENTROIDS_MM = {
    "Base_SO101 v12:1__Body5":       np.array([-114.8, 214.35, 50.1]),
    "Rotation_Pitch_SO101 v8:1__Body1": np.array([-114.9, 178.9, 75.0]),
    "Motor_holder_SO101_Base v6:1__Body1": np.array([-114.0, 163.9, 103.1]),
    "Upper_arm_SO101 v9:1__Body1":   np.array([-111.1, 223.7, 134.7]),
    "Motor_holder_SO101_Wrist v8:1__Body1": np.array([-108.5, 172.9, 164.5]),
    "Under_arm_SO101 v12:1__Body1":  np.array([-109.1, 224.5, 162.0]),
    "Wrist_Roll_Pitch_SO100 v15:1__Body1": np.array([-106.6, 144.2, 193.2]),
}

# Mapping URDF link → CAD part name (approximation; base_link uses base_so101_v2)
_LINK_TO_CAD_PART = {
    "base_link":       "Base_SO101 v12:1__Body5",
    "shoulder_link":   "Rotation_Pitch_SO101 v8:1__Body1",
    "upper_arm_link":  "Upper_arm_SO101 v9:1__Body1",
    "lower_arm_link":  "Under_arm_SO101 v12:1__Body1",
    "wrist_link":      "Wrist_Roll_Pitch_SO100 v15:1__Body1",
}


def compute_fold_residuals(
    T_world_left: np.ndarray,
    q_fold: List[float],
    vendor_dir: Path,
) -> List[dict]:
    """Compute per-link centroid error (mm) at the folded pose for the left arm.

    Parameters
    ----------
    T_world_left : 4×4 transform (translation in metres)
    q_fold : 5 joint angles in radians (shoulder_pan .. wrist_roll)

    Returns
    -------
    list of dicts with link_name, cad_centroid_mm, fk_centroid_mm, error_mm
    """
    frames = forward_kinematics(T_world_left, q_fold)
    results = []

    for link_name, cad_part in _LINK_TO_CAD_PART.items():
        cad_ctr = _ARM_A_CAD_CENTROIDS_MM[cad_part]
        frame = frames.get(link_name)
        if frame is None:
            continue

        # FK gives T_world_link; link origin in world frame
        fk_origin_mm = frame[:3, 3] * 1000.0

        # For most links the FK origin ≈ joint frame, not mesh centroid.
        # We need the mesh centroid relative to the link frame.
        mesh_info = _URDF_LINK_MESHES.get(link_name)
        if mesh_info:
            stl_path = vendor_dir / "assets" / mesh_info["file"]
            if stl_path.exists():
                import trimesh
                mesh = trimesh.load(str(stl_path))
                T_vis = _make_tf(mesh_info["xyz"], mesh_info["rpy"])
                mesh_copy = mesh.copy()
                mesh_copy.apply_transform(T_vis)
                # centroid in link frame (m)
                ctr_link_m = mesh_copy.centroid
                # transform to world frame (m)
                ctr_world_m = frame[:3, :3] @ ctr_link_m + frame[:3, 3]
                fk_centroid_mm = ctr_world_m * 1000.0
            else:
                fk_centroid_mm = fk_origin_mm
        else:
            fk_centroid_mm = fk_origin_mm

        error_mm = np.linalg.norm(fk_centroid_mm - cad_ctr)
        results.append({
            "link_name": link_name,
            "cad_part": cad_part,
            "cad_centroid_mm": cad_ctr.tolist(),
            "fk_centroid_mm": fk_centroid_mm.tolist(),
            "error_mm": float(error_mm),
        })

    return results
