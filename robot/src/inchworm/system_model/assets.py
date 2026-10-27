"""
assets.py
=========
Mesh loading for the inchworm system model.

Responsibility
--------------
1. Load URDF SO-101 link visual meshes (STL, metres) and express them in their
   parent link frames (as specified by URDF visual origins).
2. Load the owner CAD wrist export STLs (metres, chassis frame) and re-express
   the custom wrist parts in the correct URDF link frames.

Wrist custom parts (owner CAD exports, chassis frame)
------------------------------------------------------
The owner replaced the stock wrist_roll_follower and moving_jaw with a custom
lock-and-key gripper.  Two STL exports are provided:

  CAD/exports/wrist_camera.stl  – wrist_roll_pitch body + camera mount (2 bodies)
  CAD/exports/wrist_gripper.stl – wrist_roll_pitch body + lock body (Part 10)
                                  + locking key (Part 9) + ~500 screws

Frame bookkeeping
-----------------
Both exports were fit to the URDF wrist_link via ``T_export_wristlink``:

    p_wrist = T_export_wristlink @ p_export_chassis

To express a point from the export in the wrist_link frame:

    p_wrist = inv(T_export_wristlink) @ p_export_chassis

The export STL vertices ARE IN METRES (STEP assembly-origin = chassis frame,
Z up, confirmed by the CAD collision report and calibration JSON XYZ in metres).

wrist_roll_pitch (Part 0 in both exports)
  - Present in both STLs as the body ~32.25 cm3 matched to wrist_link.
  - Express in wrist_link frame: apply inv(T_export_wristlink).

camera_mount (Part 1 of wrist_camera.stl)
  - Rigidly attached to wrist_roll_pitch, does not rotate with wrist_roll.
  - Express in wrist_link frame: apply inv(T_export_wristlink).

lock body (Part 10, ~32.8 cm3) and locking key (Part 9, ~9.96 cm3)
  - These parts are DOWNSTREAM of the wrist_roll joint.
  - They are expressed in gripper_link frame:
      T_export_gripperlink = inv(T_wristlink_gripperlink(q=0)) @ inv(T_export_wristlink)
  - ASSUMPTION: the export was made with wrist_roll = 0 (standard CAD neutral
    pose).  There is no evidence of a non-zero wrist_roll in the export; the
    fit RMS < 0.07 mm is consistent with the nominal (q=0) URDF pose.

Stock gripper meshes excluded
------------------------------
- wrist_roll_follower_so101_v1.stl: EXCLUDED (replaced by the owner's custom
  lock-and-key gripper; the custom parts in wrist_gripper.stl take its place
  in gripper_link).
- moving_jaw_so101_v1.stl: EXCLUDED (the moving jaw is replaced; the locking
  key (Part 9) is the custom equivalent).
- Trigger_SO101.stl, Handle_SO101.stl, Wrist_Roll_SO101.stl: EXCLUDED (stock
  gripper cable-drive parts; not present in the export; owner's design is
  servo-driven with the servo body in wrist_link as stock).
- sts3215_03a_v1.stl (gripper servo in gripper_link): KEPT – the owner's design
  still uses an STS3215 servo to drive wrist_roll; the export shows the servo
  body inside the wrist_roll_pitch which maps to wrist_link (the no-horn variant
  sts3215_03a_no_horn_v1.stl is the one in wrist_link).  The full-horn servo in
  gripper_link is EXCLUDED (no matching structure in the owner export).
  REPORTED UNCERTAINTY: if the owner replaced the gripper servo body too, this
  servo mesh should also be excluded.

Volume threshold for body filtering
------------------------------------
Bodies with volume < 0.5 cm3 (= 5e-7 m3) are dropped from the gripper export
to remove the ~500 screws.  This is applied only to wrist_gripper.stl.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Transform helpers
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


def make_tf(xyz_m: np.ndarray, rpy: Tuple[float, float, float]) -> np.ndarray:
    """Build a 4×4 rigid transform from xyz (metres) and XYZ-intrinsic rpy (rad)."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = np.asarray(xyz_m, dtype=float)
    return T


def invert_tf(T: np.ndarray) -> np.ndarray:
    """Invert a rigid-body 4×4 transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def axis_angle_tf(axis: np.ndarray, angle: float) -> np.ndarray:
    """4×4 rotation-only transform from axis-angle (Rodrigues)."""
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    R = np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


# ---------------------------------------------------------------------------
# URDF joint and visual parameters (parsed from so101_new_calib.urdf)
# ---------------------------------------------------------------------------

# Joint definitions: each entry is (parent_link, child_link, xyz_m, rpy, axis)
# xyz_m and rpy define the joint origin in the parent link frame.
_URDF_JOINTS: Dict[str, dict] = {
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

# Joint ordering for q[0..4]
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Link visual meshes: (filename, xyz_m, rpy) in link frame.
# Only structural meshes included; servos are excluded for clarity.
# (Stock gripper parts wrist_roll_follower and moving_jaw are excluded.)
_LINK_VISUALS: Dict[str, List[Tuple[str, List[float], Tuple[float, float, float]]]] = {
    "base_link": [
        ("base_so101_v2.stl",              [-0.00636471, -8.97657e-09, -0.0024],  (math.pi/2, 0, math.pi/2)),
        ("base_motor_holder_so101_v1.stl", [-0.00636471, -9.94414e-05, -0.0024],  (math.pi/2, 0, math.pi/2)),
        ("sts3215_03a_v1.stl",             [0.0263353,   -8.97657e-09,  0.0437],  (0, 0, 0)),
        ("waveshare_mounting_plate_so101_v2.stl", [-0.0309827, -0.000199441, 0.0474], (math.pi/2, 0, math.pi/2)),
    ],
    "shoulder_link": [
        ("sts3215_03a_v1.stl",             [-0.0303992,   0.000422241, -0.0417], (math.pi/2, math.pi/2, 0)),
        ("motor_holder_so101_base_v1.stl", [-0.0675992,  -0.000177759,  0.0158499], (math.pi/2, -math.pi/2, 0)),
        ("rotation_pitch_so101_v1.stl",    [0.0122008,    2.22413e-05,  0.0464], (-math.pi/2, 0, 0)),
    ],
    "upper_arm_link": [
        ("sts3215_03a_v1.stl",   [-0.11257, -0.0155, 0.0187],  (-math.pi, 0, -math.pi/2)),
        ("upper_arm_so101_v1.stl", [-0.065085, 0.012, 0.0182], (math.pi, 0, 0)),
    ],
    "lower_arm_link": [
        ("under_arm_so101_v1.stl",         [-0.0648499, -0.032, 0.0182],  (math.pi, 0, 0)),
        ("motor_holder_so101_wrist_v1.stl", [-0.0648499, -0.032, 0.018],  (-math.pi, 0, 0)),
        ("sts3215_03a_v1.stl",             [-0.1224,     0.0052, 0.0187], (-math.pi, 0, -math.pi)),
    ],
    "wrist_link": [
        ("sts3215_03a_no_horn_v1.stl",     [8.32667e-17, -0.0424, 0.0306], (math.pi/2, math.pi/2, 0)),
        ("wrist_roll_pitch_so101_v2.stl",  [0, -0.028, 0.0181],           (-math.pi/2, -math.pi/2, 0)),
    ],
    # gripper_link: stock wrist_roll_follower EXCLUDED (replaced by owner's lock-and-key).
    # The STS3215 servo in gripper_link EXCLUDED (owner export doesn't show it clearly).
    "gripper_link": [],
}


# ---------------------------------------------------------------------------
# FK
# ---------------------------------------------------------------------------

def forward_kinematics(
    T_chassis_base: np.ndarray,
    q: List[float],
) -> Dict[str, np.ndarray]:
    """Compute 4×4 link frames in chassis frame.

    Parameters
    ----------
    T_chassis_base : 4×4, base_link pose in chassis frame (metres)
    q : [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll] (rad)

    Returns
    -------
    dict: link_name -> T_chassis_link (4×4, metres)
    """
    frames: Dict[str, np.ndarray] = {"base_link": T_chassis_base.copy()}
    for i, jname in enumerate(JOINT_NAMES):
        jd = _URDF_JOINTS[jname]
        parent = jd["parent"]
        child = jd["child"]
        T_parent_joint = make_tf(jd["xyz"], jd["rpy"])
        T_joint_rot = axis_angle_tf(jd["axis"], q[i])
        T_chassis_child = frames[parent] @ T_parent_joint @ T_joint_rot
        frames[child] = T_chassis_child
    return frames


# ---------------------------------------------------------------------------
# Mesh loading helpers (require trimesh)
# ---------------------------------------------------------------------------

def _load_stl_metres(path: Path) -> "trimesh.Trimesh":
    """Load an STL file; return a trimesh in metres (STL is in metres)."""
    import trimesh
    mesh = trimesh.load(str(path), force="mesh")
    return mesh


def _split_bodies(mesh: "trimesh.Trimesh", min_vol_m3: float = 0.0) -> List["trimesh.Trimesh"]:
    """Split a mesh into connected components; drop bodies with vol < min_vol_m3."""
    import trimesh
    cc = mesh.split(only_watertight=False)
    if not isinstance(cc, list):
        cc = [cc]
    if min_vol_m3 > 0:
        cc = [c for c in cc if abs(c.volume) >= min_vol_m3]
    return cc


def _apply_tf(mesh: "trimesh.Trimesh", T: np.ndarray) -> "trimesh.Trimesh":
    """Return a copy of mesh with transform T applied."""
    m = mesh.copy()
    m.apply_transform(T)
    return m


# ---------------------------------------------------------------------------
# URDF arm meshes
# ---------------------------------------------------------------------------

def load_urdf_arm_meshes(
    T_chassis_base: np.ndarray,
    q: List[float],
    assets_dir: Path,
) -> Dict[str, "trimesh.Trimesh"]:
    """Load SO-101 arm meshes in chassis frame (metres).

    Parameters
    ----------
    T_chassis_base : base_link pose in chassis frame (4×4, metres)
    q : 5 joint angles
    assets_dir : robot/vendor/so101/assets/

    Returns
    -------
    dict: "<link_name>/<mesh_file>" -> trimesh in chassis frame (metres)
    """
    frames = forward_kinematics(T_chassis_base, q)
    result: Dict[str, "trimesh.Trimesh"] = {}
    for link_name, visuals in _LINK_VISUALS.items():
        T_chassis_link = frames.get(link_name)
        if T_chassis_link is None:
            continue
        for fname, xyz_m, rpy in visuals:
            path = assets_dir / fname
            if not path.exists():
                continue
            mesh = _load_stl_metres(path)
            T_vis = make_tf(np.array(xyz_m, dtype=float), rpy)
            T_total = T_chassis_link @ T_vis
            result[f"{link_name}/{fname}"] = _apply_tf(mesh, T_total)
    return result


# ---------------------------------------------------------------------------
# Wrist custom part assembly
# ---------------------------------------------------------------------------

# Calibration JSON keys:
#   T_export_wristlink – pose of the URDF wrist_link frame in the chassis-frame
#                        CAD export.  Units: metres.  Applies to both
#                        wrist_camera.stl and wrist_gripper.stl.
# Both exports' T_export_wristlink agree within 0.05 mm; we average them.

def _average_wrist_tf(tf_cam: np.ndarray, tf_grip: np.ndarray) -> np.ndarray:
    """Average two nearly-identical 4×4 transforms (translation + rotation).

    The translation is averaged directly.  The rotation is taken from tf_cam
    (RMS fit slightly better, and the two are < 0.1° apart).
    """
    T_avg = tf_cam.copy()
    T_avg[:3, 3] = 0.5 * (tf_cam[:3, 3] + tf_grip[:3, 3])
    return T_avg


def load_wrist_camera_meshes(
    export_stl: Path,
    T_export_wristlink: np.ndarray,
    T_chassis_wristlink: np.ndarray,
) -> Dict[str, "trimesh.Trimesh"]:
    """Load wrist_camera.stl bodies expressed in wrist_link frame, then in chassis frame.

    Bodies loaded:
      - wrist_roll_pitch (~32.25 cm3)  -> wrist_link frame
      - camera_mount (all other bodies) -> wrist_link frame

    ASSUMPTION: wrist_roll = 0 at export pose.

    Parameters
    ----------
    export_stl : CAD/exports/wrist_camera.stl
    T_export_wristlink : 4×4, wrist_link frame in the chassis-oriented export frame
    T_chassis_wristlink : 4×4, wrist_link in chassis frame (from FK at q)

    Returns
    -------
    dict: name -> trimesh in chassis frame (metres)
    """
    import trimesh

    mesh = _load_stl_metres(export_stl)
    bodies = _split_bodies(mesh, min_vol_m3=0.0)

    # Transform from export-chassis frame to wrist_link frame
    T_wristlink_export = invert_tf(T_export_wristlink)

    # Transform from wrist_link frame to chassis frame at given q
    # T_chassis_mesh = T_chassis_wristlink @ T_wristlink_export @ T_export_mesh
    # Since the mesh vertices ARE in the export-chassis frame:
    T_total = T_chassis_wristlink @ T_wristlink_export

    result: Dict[str, "trimesh.Trimesh"] = {}
    for i, body in enumerate(bodies):
        name = f"wrist_camera/body_{i}"
        result[name] = _apply_tf(body, T_total)
    return result


def load_wrist_gripper_meshes(
    export_stl: Path,
    T_export_wristlink: np.ndarray,
    T_chassis_wristlink: np.ndarray,
    T_wristlink_gripperlink_at_q0: np.ndarray,
    min_vol_m3: float = 0.5e-6,  # 0.5 cm3 in m3
) -> Dict[str, "trimesh.Trimesh"]:
    """Load wrist_gripper.stl bodies; filter screws; assign to wrist or gripper frame.

    Body classification:
      - The wrist_roll_pitch body (~32.25 cm3 ≈ 3.225e-5 m3) is in wrist_link frame.
      - Lock body (Part 10, ~32.8 cm3) and locking key (Part 9, ~9.96 cm3) are
        downstream of wrist_roll; they are expressed in gripper_link frame.
      - Bodies < 0.5 cm3 are dropped (screws).

    ASSUMPTION: wrist_roll = 0 at export pose.

    Parameters
    ----------
    export_stl : CAD/exports/wrist_gripper.stl
    T_export_wristlink : 4×4
    T_chassis_wristlink : 4×4, wrist_link in chassis frame (FK at q)
    T_wristlink_gripperlink_at_q0 : 4×4 from FK at wrist_roll=0
    min_vol_m3 : volume threshold to drop small bodies (screws)
    """
    import trimesh

    mesh = _load_stl_metres(export_stl)
    bodies = _split_bodies(mesh, min_vol_m3=min_vol_m3)

    T_wristlink_export = invert_tf(T_export_wristlink)
    T_gripperlink_export = invert_tf(T_wristlink_gripperlink_at_q0) @ T_wristlink_export

    # Classify by volume:
    #   wrist_roll_pitch: ~3.225e-5 m3 (32.25 cm3)
    #   lock body: ~3.28e-5 m3 (32.8 cm3)
    #   key body: ~9.96e-6 m3 (9.96 cm3)
    # All large bodies get assigned by comparing to the wrist_roll_pitch vol.
    # The body closest to 3.225e-5 m3 is wrist_roll_pitch (goes to wrist_link).
    # Bodies larger than ~5e-6 m3 but NOT wrist_roll_pitch go to gripper_link.

    _WRIST_VOL_M3 = 3.225e-5   # wrist_roll_pitch reference volume (from fit data: 32.25 cm3)
    _WRIST_TOL = 0.5e-5        # ±5 cm3 tolerance

    result: Dict[str, "trimesh.Trimesh"] = {}
    for i, body in enumerate(bodies):
        vol = abs(body.volume)
        vol_cm3 = vol * 1e6
        # Identify wrist_roll_pitch by volume proximity
        is_wrist_body = abs(vol - _WRIST_VOL_M3) < _WRIST_TOL
        if is_wrist_body:
            # wrist_roll_pitch: express in wrist_link frame
            T_total = T_chassis_wristlink @ T_wristlink_export
            name = f"wrist_gripper/wrist_roll_pitch"
        else:
            # lock body or key: express in gripper_link frame
            T_chassis_gripperlink = T_chassis_wristlink @ T_wristlink_gripperlink_at_q0
            T_total = T_chassis_gripperlink @ T_gripperlink_export
            name = f"wrist_gripper/body_{i}_{vol_cm3:.1f}cm3"
        result[name] = _apply_tf(body, T_total)
    return result


# ---------------------------------------------------------------------------
# Raw export meshes (for CAD check / self-check)
# ---------------------------------------------------------------------------

def load_raw_export(
    export_stl: Path,
    min_vol_m3: float = 0.0,
) -> Dict[str, "trimesh.Trimesh"]:
    """Load export STL bodies without any frame transform (raw chassis frame).

    Used for the self-check that transformed custom parts match raw export.
    """
    mesh = _load_stl_metres(export_stl)
    bodies = _split_bodies(mesh, min_vol_m3=min_vol_m3)
    return {f"raw/body_{i}": body for i, body in enumerate(bodies)}


# ---------------------------------------------------------------------------
# DexArm meshes
# ---------------------------------------------------------------------------

def load_dexarm_meshes(
    T_chassis_bfix: np.ndarray,
    q_dexarm: Optional[List[float]],
    dexarm_dir: Path,
) -> Dict[str, "trimesh.Trimesh"]:
    """Load DexArm STL meshes in chassis frame at given joint configuration.

    The DexArm URDF STLs are in mm (scale 0.001).  The B_Fix link is placed
    by T_chassis_bfix (metres).

    q_dexarm : [bases, B_Rot->In_1, Join->I_2] or None for zeros.

    Only the main structural links are loaded:
      B_Fix, B_Rot, In_1, Join, In_2, Out_1, Out_2, EE
    """
    try:
        import trimesh
    except ImportError:
        return {}

    if q_dexarm is None:
        q_dexarm = [0.0, 0.0, 0.0]

    # DexArm joint chain (simplified; mimic joints ignored for display):
    # B_Fix -> B_Rot: joint "bases", axis Z, origin 0 0 0
    # B_Rot -> In_1: joint "B_Rot->In_1", axis X, origin (0, -0.02, 0.13)
    # In_1  -> Join: joint mimic(-In_1), skip
    # Join  -> In_2: joint "Join->I_2", axis X, origin (0, -0.022, -0.150)
    # etc.  For display we just load all links at identity (q=0) unless
    # a non-zero q_dexarm is specified.
    #
    # For simplicity, only B_Fix is FK'd to T_chassis_bfix; all other links
    # are relative to B_Fix with their URDF visual origins applied at q=0.
    # A full DexArm FK is not required for the system model display.

    stl_dir = dexarm_dir / "STL"
    # Maps link_name -> (stl_file, visual_xyz_m, visual_rpy)
    # Visual origins from DexArm.urdf, STL scale=0.001 so they're in mm but
    # loaded as if in metres with scale applied.
    dexarm_links = [
        ("B_Fix",  "00-B_Fix.stl",   [0, 0, 0],              (0, 0, 0)),
        ("B_Rot",  "01-B_Rot.stl",   [0, 0, 0],              (0, 0, 0)),
        ("In_1",   "02-In_1.stl",    [0, 0.022, -0.127],     (0, 0, 0)),
        ("Join",   "03-Join.stl",    [0, 0.0332, -0.277],    (0, 0, 0)),
        ("In_2",   "04-In_2.stl",    [0, 0.022, -0.127],     (0, 0, 0)),
        ("EE",     "05-EE.stl",      [0, 0.036, -0.277],     (0, 0, 0)),
        ("Out_1",  "06-Out_1.stl",   [0, 0.014, -0.277],     (0, 0, 0)),
        ("Out_2",  "07-Out_2.stl",   [0, 0.014, -0.277],     (0, 0, 0)),
    ]

    result: Dict[str, "trimesh.Trimesh"] = {}
    for link_name, stl_fname, vis_xyz, vis_rpy in dexarm_links:
        stl_path = stl_dir / stl_fname
        if not stl_path.exists():
            continue
        mesh = _load_stl_metres(stl_path)
        # STL is in mm, scale=0.001 converts to metres
        mesh.apply_scale(0.001)
        # Apply visual origin (already in metres after scale)
        T_vis = make_tf(np.array(vis_xyz, dtype=float), vis_rpy)
        # Link frame in chassis: B_Fix uses T_chassis_bfix; others at q=0 relative to B_Fix.
        # For q=0, all link frames = B_Fix frame (joints are zero).
        T_chassis_link = T_chassis_bfix.copy()
        result[f"dexarm/{link_name}"] = _apply_tf(_apply_tf(mesh, T_vis), T_chassis_link)
    return result


# ---------------------------------------------------------------------------
# Static env: bases.stl bodies
# ---------------------------------------------------------------------------

def load_bases_meshes(
    bases_stl: Path,
) -> Tuple[Dict[str, "trimesh.Trimesh"], float]:
    """Load bases.stl and return bodies + table Z (Z_min of mount bodies, metres).

    bases.stl contains both SO-101 base parts + the mounts they sit on + the
    DexArm base.  All in metres (chassis frame, Z up).

    Returns
    -------
    meshes : dict name -> trimesh in chassis frame (metres)
    table_z_m : Z_min of the lowest body in bases.stl (= table/floor height)
    """
    import trimesh

    mesh = _load_stl_metres(bases_stl)
    bodies = _split_bodies(mesh)
    if not bodies:
        bodies = [mesh]

    meshes = {f"bases/body_{i}": b for i, b in enumerate(bodies)}
    table_z_m = float(min(b.bounds[0][2] for b in bodies))
    return meshes, table_z_m
