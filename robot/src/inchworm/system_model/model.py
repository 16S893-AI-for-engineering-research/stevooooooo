"""
model.py
========
SystemModel: assembles the full inchworm robot scene from calibrated placements.

Usage
-----
    from inchworm.system_model.model import SystemModel
    m = SystemModel()
    scene = m.scene(q_left=[0]*5, q_right=[0]*5)
    # scene is a list of (name, trimesh_in_chassis_frame)

All geometry is in the **chassis frame** (= STEP assembly frame = Onshape
assembly frame; Z up, metres).

Coordinate frame
----------------
- Origin: Onshape assembly origin (confirmed by calibration data).
- Z up: confirmed by Z_min ≈ 0 mm for base plates.
- Y: runs along the arm axis (left arm Y ≈ 235 mm, right arm Y ≈ 418 mm).
- X: across the chassis (both arm bases at X ≈ −115 mm).

Tube placement (from cad_collision_report_2026-10-25.md §3.3)
-------------------------------------------------------------
Three tube instances shown in the CAD (three inchworm advance steps).
Mid position (default): centre at (−109.8, 176.8, 315.7) mm.
Tube axis parallel to chassis Y (owner: "tube runs parallel to the table").
Tube OD from Part-13 X-span; tube radius = OD/2.
Table-to-far-side-of-tube ≈ 360 mm → tube axis at Z ≈ 320 mm
(consistent with report Part-13 Z centre ≈ 315 mm).
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .assets import (
    JOINT_NAMES,
    forward_kinematics,
    invert_tf,
    load_bases_meshes,
    load_dexarm_meshes,
    load_raw_export,
    load_urdf_arm_meshes,
    load_wrist_camera_meshes,
    load_wrist_gripper_meshes,
    make_tf,
)

# ---------------------------------------------------------------------------
# Repository-relative paths
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve()
_ROBOT = _HERE.parents[3]          # robot/
_REPO = _ROBOT.parent              # repo root

_CALIB_DIR = _ROBOT / "calibration_data"
_BASE_FIT   = _CALIB_DIR / "aa40_base_fit_2026-10-25.json"
_FIT_ALL    = _CALIB_DIR / "aa40_fit_all_2026-10-25.json"

_VENDOR_SO101  = _ROBOT / "vendor" / "so101"
_VENDOR_DEXARM = _ROBOT / "vendor" / "dexarm"
_EXPORTS       = _REPO  / "CAD" / "exports"

# ---------------------------------------------------------------------------
# Tube geometry constants (from §3.3 of cad_collision_report)
# ---------------------------------------------------------------------------

# Tube mid-position centre in chassis frame (metres)
_TUBE_CENTRE_M  = np.array([-0.1098, 0.1768, 0.3157])
# Tube dimensions (OD from Part-13 bbox, height from Z span)
_TUBE_RADIUS_M  = 0.040   # metres; OD from Part-13 X-span in collision report
_TUBE_HEIGHT_M  = 0.081   # metres; Z span from Part-13 bbox

# Tube axis is chassis Y; create a rotation so that trimesh cylinder axis (Z)
# points along chassis Y.
_T_TUBE: np.ndarray = np.eye(4)
_T_TUBE[:3, :3] = np.array([
    [1, 0, 0],
    [0, 0, -1],
    [0, 1, 0],
])
_T_TUBE[:3, 3] = _TUBE_CENTRE_M


# ---------------------------------------------------------------------------
# Calibration loading
# ---------------------------------------------------------------------------

def _load_base_fit() -> Tuple[np.ndarray, np.ndarray, float]:
    """Load arm base placements.

    Returns
    -------
    T_chassis_left  : 4×4, left arm base_link in chassis frame (metres)
    T_chassis_right : 4×4, right arm base_link in chassis frame (metres)
    base_spacing_mm : scalar (mm)
    """
    with open(_BASE_FIT) as f:
        data = json.load(f)
    T_left  = np.array(data["left"]["T_chassis_baselink"],  dtype=float)
    T_right = np.array(data["right"]["T_chassis_baselink"], dtype=float)
    spacing = data["base_spacing_mm"]
    return T_left, T_right, spacing


def _load_wrist_fit() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load wrist/dexarm calibration.

    Returns
    -------
    T_export_wristlink_cam  : from wrist_camera.stl fit (4×4, metres)
    T_export_wristlink_grip : from wrist_gripper.stl fit (4×4, metres)
    T_chassis_dexarm        : B_Fix link pose (4×4, metres)
    """
    with open(_FIT_ALL) as f:
        data = json.load(f)
    T_cam  = np.array(data["wrist_camera.stl:wrist#0"]["T_export_wristlink"],  dtype=float)
    T_grip = np.array(data["wrist_gripper.stl:wrist#0"]["T_export_wristlink"], dtype=float)
    T_dex  = np.array(data["dexarm_base#0"]["T_chassis_dexarm"],               dtype=float)
    return T_cam, T_grip, T_dex


# ---------------------------------------------------------------------------
# SystemModel
# ---------------------------------------------------------------------------

class SystemModel:
    """Inchworm system model.

    Attributes
    ----------
    T_left   : 4×4 base_link of left arm in chassis frame (metres)
    T_right  : 4×4 base_link of right arm in chassis frame (metres)
    T_dexarm : 4×4 B_Fix in chassis frame (metres)
    T_export_wristlink : averaged 4×4 (metres)
    base_spacing_mm : float

    Table Z (metres, Z_min of bases.stl)
    tube_centre_m : np.ndarray (3,)
    """

    def __init__(
        self,
        base_fit_path: Optional[Path] = None,
        fit_all_path: Optional[Path] = None,
        vendor_so101: Optional[Path] = None,
        vendor_dexarm: Optional[Path] = None,
        exports_dir: Optional[Path] = None,
    ) -> None:
        self._base_fit_path = base_fit_path or _BASE_FIT
        self._fit_all_path  = fit_all_path  or _FIT_ALL
        self._vendor_so101  = vendor_so101  or _VENDOR_SO101
        self._vendor_dexarm = vendor_dexarm or _VENDOR_DEXARM
        self._exports_dir   = exports_dir   or _EXPORTS

        # Load calibration
        self.T_left, self.T_right, self.base_spacing_mm = _load_base_fit()
        T_cam, T_grip, self.T_dexarm = _load_wrist_fit()

        # Average the two wrist fit transforms (< 0.05 mm apart)
        self.T_export_wristlink = self._avg_wrist(T_cam, T_grip)

        # Compute wrist_link → gripper_link at wrist_roll = 0 (needed for gripper frames)
        self._T_wristlink_gripperlink_q0 = self._compute_T_wristlink_gripperlink(0.0)

        # Table Z (lazy; computed once when bases.stl is loaded)
        self._table_z_m: Optional[float] = None
        self.tube_centre_m = _TUBE_CENTRE_M.copy()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _avg_wrist(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
        """Average two 4×4 wrist transforms (translation midpoint, R from T1)."""
        T_avg = T1.copy()
        T_avg[:3, 3] = 0.5 * (T1[:3, 3] + T2[:3, 3])
        return T_avg

    def _compute_T_wristlink_gripperlink(self, q_wrist_roll: float) -> np.ndarray:
        """Compute T_wristlink_gripperlink at a given wrist_roll angle (rad)."""
        from .assets import _URDF_JOINTS, axis_angle_tf
        jd = _URDF_JOINTS["wrist_roll"]
        T_joint_origin = make_tf(jd["xyz"], jd["rpy"])
        T_rot = axis_angle_tf(jd["axis"], q_wrist_roll)
        return T_joint_origin @ T_rot

    def _wrist_link_in_chassis(self, T_chassis_base: np.ndarray, q: List[float]) -> np.ndarray:
        """Return T_chassis_wristlink at a given arm config q."""
        frames = forward_kinematics(T_chassis_base, q)
        return frames["wrist_link"]

    def _gripper_link_in_chassis(self, T_chassis_base: np.ndarray, q: List[float]) -> np.ndarray:
        frames = forward_kinematics(T_chassis_base, q)
        return frames["gripper_link"]

    # ------------------------------------------------------------------
    # Scene builder
    # ------------------------------------------------------------------

    def scene(
        self,
        q_left: List[float],
        q_right: List[float],
        dexarm_q: Optional[List[float]] = None,
        include_custom_wrist: bool = True,
        include_raw_exports: bool = False,
        tube_position: str = "mid",  # "mid", "rear", "front"
    ) -> List[Tuple[str, "trimesh.Trimesh"]]:
        """Build the full system scene.

        Parameters
        ----------
        q_left, q_right : 5-element joint angle vectors (rad)
        dexarm_q : DexArm joint config (default zeros)
        include_custom_wrist : include owner's custom wrist parts (requires exports)
        include_raw_exports : include raw export meshes in chassis frame (for self-check)
        tube_position : 'mid', 'rear', 'front' or 'none'

        Returns
        -------
        list of (name: str, mesh: trimesh.Trimesh) in chassis frame (metres)
        """
        result: List[Tuple[str, "trimesh.Trimesh"]] = []
        assets_dir = self._vendor_so101 / "assets"

        # --- Left arm (structural meshes) ---
        left_meshes = load_urdf_arm_meshes(self.T_left, q_left, assets_dir)
        for name, mesh in left_meshes.items():
            result.append((f"arm_left/{name}", mesh))

        # --- Right arm (structural meshes) ---
        right_meshes = load_urdf_arm_meshes(self.T_right, q_right, assets_dir)
        for name, mesh in right_meshes.items():
            result.append((f"arm_right/{name}", mesh))

        # --- Custom wrist parts (left arm only – both arms have the same wrist) ---
        # The exports are from one arm.  We apply them to both arms.
        if include_custom_wrist and (self._exports_dir / "wrist_camera.stl").exists():
            for arm_name, T_base, q in [("left", self.T_left, q_left), ("right", self.T_right, q_right)]:
                T_wristlink = self._wrist_link_in_chassis(T_base, q)
                T_wl_gl_q0 = self._T_wristlink_gripperlink_q0

                cam_meshes = load_wrist_camera_meshes(
                    self._exports_dir / "wrist_camera.stl",
                    self.T_export_wristlink,
                    T_wristlink,
                )
                for name, mesh in cam_meshes.items():
                    result.append((f"arm_{arm_name}/custom_wrist_cam/{name}", mesh))

                if (self._exports_dir / "wrist_gripper.stl").exists():
                    grip_meshes = load_wrist_gripper_meshes(
                        self._exports_dir / "wrist_gripper.stl",
                        self.T_export_wristlink,
                        T_wristlink,
                        T_wl_gl_q0,
                        min_vol_m3=0.5e-6,
                    )
                    for name, mesh in grip_meshes.items():
                        result.append((f"arm_{arm_name}/custom_wrist_grip/{name}", mesh))

        # --- Raw exports (for self-check) ---
        if include_raw_exports and self._exports_dir.exists():
            for fname in ["wrist_camera.stl", "wrist_gripper.stl"]:
                stl = self._exports_dir / fname
                if stl.exists():
                    min_v = 0.5e-6 if "gripper" in fname else 0.0
                    raw = load_raw_export(stl, min_vol_m3=min_v)
                    for name, mesh in raw.items():
                        result.append((f"export_raw/{fname}/{name}", mesh))

        # --- Static env: bases.stl ---
        bases_stl = self._exports_dir / "bases.stl"
        if bases_stl.exists():
            try:
                bases_meshes, table_z = load_bases_meshes(bases_stl)
                self._table_z_m = table_z
                for name, mesh in bases_meshes.items():
                    result.append((f"static/{name}", mesh))
            except Exception:
                pass

        # --- DexArm ---
        try:
            dexarm_meshes = load_dexarm_meshes(
                self.T_dexarm,
                dexarm_q,
                self._vendor_dexarm,
            )
            for name, mesh in dexarm_meshes.items():
                result.append((name, mesh))
        except Exception:
            pass

        # --- Tube ---
        if tube_position != "none":
            try:
                result.append(("tube", self._make_tube(tube_position)))
            except Exception:
                pass

        return result

    def _make_tube(self, position: str = "mid") -> "trimesh.Trimesh":
        """Create a trimesh cylinder for the tube."""
        import trimesh
        # Tube centre positions from collision report (metres)
        centres = {
            "rear":  np.array([-0.1130, -0.0068, 0.3140]),
            "mid":   np.array([-0.1098,  0.1768, 0.3157]),
            "front": np.array([-0.1066,  0.3605, 0.3173]),
        }
        centre = centres.get(position, centres["mid"])

        # Cylinder with axis along Z, height = tube height, then rotate to Y
        cyl = trimesh.creation.cylinder(
            radius=_TUBE_RADIUS_M,
            height=_TUBE_HEIGHT_M,
            sections=32,
        )
        # Rotate so cylinder axis (Z) → chassis Y
        T = np.eye(4)
        T[:3, :3] = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0],
        ])
        T[:3, 3] = centre
        cyl.apply_transform(T)
        return cyl

    @property
    def table_z_m(self) -> Optional[float]:
        """Z_min of bases.stl (metres).  None if bases.stl not yet loaded."""
        return self._table_z_m

    def load_table_z(self) -> float:
        """Eagerly load bases.stl to determine table Z."""
        if self._table_z_m is not None:
            return self._table_z_m
        bases_stl = self._exports_dir / "bases.stl"
        if not bases_stl.exists():
            return 0.0
        _, table_z = load_bases_meshes(bases_stl)
        self._table_z_m = table_z
        return table_z

    # ------------------------------------------------------------------
    # CAD self-check
    # ------------------------------------------------------------------

    def self_check_wrist_frame(
        self,
        q_left_cad: List[float],
        verbose: bool = True,
    ) -> Dict[str, float]:
        """Self-check: wrist frame bookkeeping roundtrip must be < 1 mm.

        The export bodies in the raw export (chassis frame) are transformed
        to wrist_link frame via inv(T_export_wristlink), then back to chassis
        frame using T_export_wristlink itself.  This is an identity roundtrip
        that tests that the inv/fwd pair is computed correctly.

        A second check: at the CAD-drawn fold (q_left_cad), the FK wrist_link
        position should be within 5 mm of T_export_wristlink (fit residual from
        the CAD configuration used for the export).

        Returns dict with 'max_dist_mm', 'rms_dist_mm', 'wrist_fk_offset_mm'.
        """
        import trimesh
        from scipy.spatial import cKDTree

        # --- Part 1: roundtrip identity check using T_export_wristlink ---
        # Transform: raw_export -> wrist_link frame -> chassis frame
        # Using T_export_wristlink as the chassis<-wristlink transform.
        # This must be an identity: p_out = T_export_wristlink @ inv(T_export_wristlink) @ p_raw = p_raw
        T_wristlink_export = invert_tf(self.T_export_wristlink)

        cam_stl = self._exports_dir / "wrist_camera.stl"
        if not cam_stl.exists():
            return {"max_dist_mm": float("nan"), "rms_dist_mm": float("nan")}

        from .assets import _load_stl_metres, _split_bodies, _apply_tf
        mesh = _load_stl_metres(cam_stl)
        bodies = _split_bodies(mesh)
        if not bodies:
            return {"max_dist_mm": float("nan"), "rms_dist_mm": float("nan")}

        # Pick the largest body (wrist_roll_pitch)
        body = max(bodies, key=lambda b: abs(b.volume))
        raw_verts = body.vertices

        # Roundtrip: chassis -> wrist_link -> chassis (using T_export_wristlink)
        T_total = self.T_export_wristlink @ T_wristlink_export  # should be ~identity
        transformed = _apply_tf(body, T_total)
        xformed_verts = transformed.vertices

        # Downsample
        n = min(2000, len(raw_verts))
        idx = np.random.default_rng(42).choice(len(raw_verts), n, replace=False)
        pts_raw = raw_verts[idx]
        pts_xf  = xformed_verts[idx]

        dists_mm = np.linalg.norm(pts_raw - pts_xf, axis=1) * 1000.0

        result = {
            "max_dist_mm": float(np.max(dists_mm)),
            "rms_dist_mm": float(np.sqrt(np.mean(dists_mm**2))),
            "n_samples": n,
        }

        # --- Part 2: FK wrist_link vs. T_export_wristlink offset ---
        frames = forward_kinematics(self.T_left, q_left_cad)
        T_wl_fk = frames["wrist_link"]
        wrist_offset_mm = float(np.linalg.norm(T_wl_fk[:3, 3] - self.T_export_wristlink[:3, 3]) * 1000.0)
        result["wrist_fk_offset_mm"] = wrist_offset_mm

        if verbose:
            print(
                f"[self_check] roundtrip max={result['max_dist_mm']:.4f} mm  "
                f"rms={result['rms_dist_mm']:.4f} mm  "
                f"wrist_fk_offset={wrist_offset_mm:.2f} mm  "
                f"n={n}"
            )
        return result
