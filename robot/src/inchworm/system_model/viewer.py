"""
viewer.py
=========
Meshcat-based pose viewer for the inchworm robot.

Usage (interactive):
    uv run python robot/scripts/view_pose.py

Usage (offscreen PNG):
    uv run python robot/scripts/view_pose.py --png out.png

All requires the ``model`` optional extra:
    uv sync --extra model
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Colour palette per group
# ---------------------------------------------------------------------------

_GROUP_COLOURS = {
    "arm_left":   [0.2, 0.6, 1.0, 0.9],   # blue
    "arm_right":  [1.0, 0.4, 0.2, 0.9],   # orange
    "tube":       [0.9, 0.9, 0.2, 1.0],   # yellow
    "dexarm":     [0.3, 0.8, 0.3, 0.7],   # green
    "chassis":    [0.6, 0.6, 0.6, 0.5],   # grey
    "other":      [0.5, 0.5, 0.5, 0.3],   # light grey
}


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------

def _rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _make_tf(xyz_m: np.ndarray, rpy: Tuple[float, float, float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz_m
    return T


# ---------------------------------------------------------------------------
# Meshcat viewer
# ---------------------------------------------------------------------------

def build_scene(
    T_world_left_mm: np.ndarray,
    T_world_right_mm: np.ndarray,
    q_left: List[float],
    q_right: List[float],
    vendor_dir: Path,
    tube_y_mm: Optional[float] = None,
) -> Dict[str, object]:
    """Build a dict of {name: trimesh_mesh} for the full scene in STEP world (mm).

    Parameters
    ----------
    T_world_left_mm / T_world_right_mm : 4×4 with translation in mm
    q_left / q_right : 5-element joint angle vectors
    vendor_dir : robot/vendor/so101
    tube_y_mm : If provided, add a tube mesh at this Y position
    """
    import trimesh
    from .model_builder import load_arm_meshes

    scene: Dict[str, object] = {}

    # Left arm
    left_set = load_arm_meshes(T_world_left_mm, q_left, vendor_dir, "left")
    for lname, mesh in left_set.link_meshes.items():
        scene[f"arm_left/{lname}"] = mesh

    # Right arm
    right_set = load_arm_meshes(T_world_right_mm, q_right, vendor_dir, "right")
    for lname, mesh in right_set.link_meshes.items():
        scene[f"arm_right/{lname}"] = mesh

    # Tube (simple cylinder)
    if tube_y_mm is not None:
        import trimesh
        _TUBE_R = 4 * 10  # OD ~8*10 mm from Part 13 bbox
        cyl = trimesh.creation.cylinder(radius=_TUBE_R, height=81.0, sections=32)
        # Place tube at STEP world position:
        # X ≈ -110mm (from report), Z ≈ 315mm above base
        T_tube = np.eye(4)
        T_tube[0, 3] = -110.0
        T_tube[1, 3] = tube_y_mm
        T_tube[2, 3] = 315.0
        cyl.apply_transform(T_tube)
        scene["tube"] = cyl

    return scene


def render_png_with_trimesh(
    scene: Dict[str, object],
    out_png: Path,
    resolution: Tuple[int, int] = (1280, 720),
) -> bool:
    """Render a scene as a PNG using trimesh's built-in renderer.

    Returns True on success, False on failure.
    """
    try:
        import trimesh
        ts = trimesh.Scene()

        colours = {
            "arm_left":  [51, 153, 255, 230],
            "arm_right": [255, 102, 51, 230],
            "tube":      [230, 230, 51, 255],
            "dexarm":    [77, 204, 77, 179],
            "chassis":   [153, 153, 153, 128],
        }

        for name, mesh in scene.items():
            prefix = name.split("/")[0]
            rgba = colours.get(prefix, [180, 180, 180, 100])
            # Convert from mm to metres for trimesh scene
            m = mesh.copy()
            m.apply_scale(0.001)
            m.visual.face_colors = rgba
            ts.add_geometry(m, geom_name=name)

        # Set camera to look at the scene
        ts.set_camera(angles=[1.05, 0, 0.785])
        img = ts.save_image(resolution=resolution, visible=False)
        if img is not None:
            with open(out_png, "wb") as f:
                f.write(img)
            return True
    except Exception as exc:
        print(f"  render_png failed: {exc}")

    # Fallback: matplotlib 3D scatter
    return _render_png_matplotlib(scene, out_png)


def _render_png_matplotlib(
    scene: Dict[str, object],
    out_png: Path,
) -> bool:
    """Matplotlib 3D scatter fallback renderer (headless-safe)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        fig = plt.figure(figsize=(14, 8))
        ax = fig.add_subplot(111, projection="3d")

        colours = {
            "arm_left":  "royalblue",
            "arm_right": "orangered",
            "tube":      "gold",
            "dexarm":    "mediumseagreen",
            "chassis":   "lightgrey",
        }
        alpha_map = {
            "arm_left": 0.8, "arm_right": 0.8, "tube": 1.0,
            "dexarm": 0.5, "chassis": 0.3,
        }

        for name, mesh in scene.items():
            prefix = name.split("/")[0]
            colour = colours.get(prefix, "grey")
            alpha = alpha_map.get(prefix, 0.3)

            # Sample some vertices for scatter
            verts = mesh.vertices
            if len(verts) > 500:
                idx = np.random.choice(len(verts), 500, replace=False)
                verts = verts[idx]

            ax.scatter(verts[:, 0], verts[:, 1], verts[:, 2],
                       c=colour, alpha=alpha, s=1)

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title("Inchworm robot - STEP world frame")
        plt.tight_layout()
        plt.savefig(str(out_png), dpi=120)
        plt.close()
        return True
    except Exception as exc:
        print(f"  matplotlib fallback failed: {exc}")
        return False


def launch_meshcat_viewer(
    T_world_left_mm: np.ndarray,
    T_world_right_mm: np.ndarray,
    q_left: List[float],
    q_right: List[float],
    vendor_dir: Path,
    tube_y_mm: Optional[float] = None,
    port: int = 7000,
) -> None:
    """Launch a meshcat browser viewer. Blocks until Ctrl+C."""
    import meshcat
    import meshcat.geometry as g

    vis = meshcat.Visualizer(zmq_url=f"tcp://127.0.0.1:{port}")
    vis.open()

    print(f"Meshcat viewer open at: {vis.url()}")
    print("Building scene...")

    scene = build_scene(
        T_world_left_mm, T_world_right_mm,
        q_left, q_right,
        vendor_dir, tube_y_mm,
    )

    import trimesh
    for name, mesh in scene.items():
        prefix = name.split("/")[0]
        colours_rgba = {
            "arm_left":  [0.2, 0.6, 1.0, 0.9],
            "arm_right": [1.0, 0.4, 0.2, 0.9],
            "tube":      [0.9, 0.9, 0.2, 1.0],
            "dexarm":    [0.3, 0.8, 0.3, 0.7],
            "chassis":   [0.6, 0.6, 0.6, 0.5],
        }
        rgba = colours_rgba.get(prefix, [0.5, 0.5, 0.5, 0.5])
        colour_hex = "0x{:02x}{:02x}{:02x}".format(
            int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255)
        )

        # Convert mm → m for meshcat
        mc = mesh.copy()
        mc.apply_scale(0.001)

        mc_geom = g.TriangularMeshGeometry(
            mc.vertices,
            mc.faces,
        )
        mat = g.MeshLambertMaterial(color=int(colour_hex, 16), opacity=float(rgba[3]))
        vis[name.replace("/", "_")].set_object(mc_geom, mat)

    print("Scene loaded. Open browser at the URL above.")
    print("Press Ctrl+C to stop.")
    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Viewer stopped.")
