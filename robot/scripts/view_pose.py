#!/usr/bin/env python3
"""
view_pose.py
============
Pose viewer for the inchworm robot.  Uses the calibrated system model to
render both arms at specified joint configurations.

Usage
-----
# Render preset PNGs (fold, zero, cad_check, measured_fold):
    uv run --extra model python robot/scripts/view_pose.py --png-dir robot/out/system_model

# Render a single pose to PNG:
    uv run --extra model python robot/scripts/view_pose.py \\
        --q-left 0 -1.25 1.64 -1.31 1.36 \\
        --q-right 0 -1.25 1.64 -1.31 1.36 \\
        --png robot/out/system_model/my_pose.png

# Load a pose from the log and render:
    uv run --extra model python robot/scripts/view_pose.py \\
        --pose-log robot/calibration_data/poses_2026-10-25.jsonl \\
        --label folded_transit \\
        --png robot/out/system_model/measured_fold.png

# Interactive meshcat viewer (browser opens at localhost:7000):
    uv run --extra model python robot/scripts/view_pose.py --meshcat

Requires: uv sync --extra model
No hardware access; no serial; no cameras.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# Make package importable when run directly
_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / "robot" / "src"))

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

_COLOURS: Dict[str, List[int]] = {
    "arm_left":       [51,  153, 255, 230],  # blue
    "arm_right":      [255, 102,  51, 230],  # orange
    "static":         [160, 160, 160, 180],  # grey  (bases)
    "dexarm":         [77,  204,  77, 180],  # green
    "tube":           [230, 230,  51, 255],  # yellow
    "custom_wrist":   [255, 200,  50, 220],  # gold
    "export_raw":     [255,  0x50,  0x50, 180],  # red (raw exports for comparison)
    "cad_check_arm":  [ 50, 220, 220, 220],  # cyan (CAD-drawn arm)
    "default":        [180, 180, 180, 120],
}


def _rgba_for_name(name: str, override: Optional[List[int]] = None) -> List[int]:
    if override:
        return override
    if "arm_left" in name:
        return _COLOURS["arm_left"]
    if "arm_right" in name:
        return _COLOURS["arm_right"]
    if name.startswith("static"):
        return _COLOURS["static"]
    if name.startswith("dexarm"):
        return _COLOURS["dexarm"]
    if name == "tube":
        return _COLOURS["tube"]
    if "custom_wrist" in name:
        return _COLOURS["custom_wrist"]
    if "export_raw" in name:
        return _COLOURS["export_raw"]
    if "cad_check" in name:
        return _COLOURS["cad_check_arm"]
    return _COLOURS["default"]


# ---------------------------------------------------------------------------
# Pose log helpers
# ---------------------------------------------------------------------------

def _load_pose_log(
    log_path: Path,
    label: str,
    pan_override: Optional[float] = None,
) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """Load q_left and q_right for a given label from a JSON-lines pose log.

    Skips SUPERSEDED records.  Returns the SECOND (non-superseded) reading.

    Parameters
    ----------
    pan_override : if given, overrides shoulder_pan (index 0) in both arms.
    """
    with open(log_path) as f:
        records = [json.loads(line) for line in f if line.strip()]

    matching = [
        r for r in records
        if r.get("label") == label and "q_rad" in r
    ]
    # Group by arm
    left_records  = [r for r in matching if r.get("arm") == "left"]
    right_records = [r for r in matching if r.get("arm") == "right"]

    q_left = left_records[0]["q_rad"]  if left_records  else None
    q_right = right_records[0]["q_rad"] if right_records else None

    if pan_override is not None:
        if q_left  is not None:
            q_left  = [pan_override] + q_left[1:]
        if q_right is not None:
            q_right = [pan_override] + q_right[1:]

    return q_left, q_right


# ---------------------------------------------------------------------------
# Renderer
# ---------------------------------------------------------------------------

def _build_trimesh_scene(
    entries: List[Tuple[str, "trimesh.Trimesh"]],
) -> "trimesh.Scene":
    """Build a trimesh.Scene from (name, mesh) pairs.  Meshes are in metres."""
    import trimesh

    ts = trimesh.Scene()
    for name, mesh in entries:
        rgba = _rgba_for_name(name)
        m = mesh.copy()
        m.visual.face_colors = rgba
        ts.add_geometry(m, geom_name=name)
    return ts


def _set_camera_side(ts: "trimesh.Scene", view: str = "oblique") -> None:
    """Set camera viewpoint."""
    import trimesh
    # Compute scene bounding sphere for camera placement
    try:
        centre = ts.centroid
        radius = float(np.linalg.norm(ts.extents)) * 0.6
    except Exception:
        centre = np.array([0.0, 0.3, 0.2])
        radius = 0.5

    if view == "side":
        # Looking along chassis X (camera at negative X, looking toward +X)
        eye = centre + np.array([-radius * 2, 0, 0])
        up  = np.array([0, 0, 1])
    elif view == "top":
        eye = centre + np.array([0, 0, radius * 2])
        up  = np.array([0, 1, 0])
    elif view == "oblique":
        # Oblique: slightly from above and to the side
        eye = centre + np.array([-radius * 1.2, -radius * 0.8, radius * 0.9])
        up  = np.array([0, 0, 1])
    elif view == "front":
        # Looking along chassis Y
        eye = centre + np.array([0, -radius * 2, 0])
        up  = np.array([0, 0, 1])
    else:
        return

    try:
        from trimesh.transformations import look_at
        T = look_at(eye, centre, up)
        ts.camera_transform = T
    except Exception:
        # Fallback: set camera angles (trimesh < 4.3)
        try:
            ts.set_camera(
                angles=[1.05, 0.0, 0.4 if view == "oblique" else 0.0],
                distance=radius * 3,
                center=centre,
            )
        except Exception:
            pass


def _render_views(
    entries: List[Tuple[str, "trimesh.Trimesh"]],
    out_png: Path,
    views: List[str] = ("side", "oblique"),
    resolution: Tuple[int, int] = (960, 540),
) -> bool:
    """Render multiple views tiled into a single PNG.

    Returns True on success.
    """
    import trimesh
    import numpy as np

    tiles = []
    for view in views:
        ts = _build_trimesh_scene(entries)
        _set_camera_side(ts, view)
        try:
            img_bytes = ts.save_image(resolution=resolution, visible=False)
        except Exception:
            try:
                # Trimesh >= 4.x API
                img_bytes = ts.save_image(resolution=list(resolution), visible=False)
            except Exception as exc:
                print(f"  Warning: render failed for view {view}: {exc}")
                img_bytes = None
        if img_bytes is not None:
            from PIL import Image
            import io
            img = Image.open(io.BytesIO(img_bytes))
            tiles.append(np.array(img))

    if not tiles:
        return _render_matplotlib_fallback(entries, out_png)

    # Tile horizontally
    from PIL import Image
    if len(tiles) == 1:
        combined = Image.fromarray(tiles[0])
    else:
        total_w = sum(t.shape[1] for t in tiles)
        max_h = max(t.shape[0] for t in tiles)
        canvas = np.zeros((max_h, total_w, 4), dtype=np.uint8)
        x = 0
        for t in tiles:
            h, w = t.shape[:2]
            if t.shape[2] == 3:
                t_rgba = np.concatenate([t, np.full((h, w, 1), 255, np.uint8)], axis=2)
            else:
                t_rgba = t
            canvas[:h, x:x+w] = t_rgba
            x += w
        combined = Image.fromarray(canvas)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    combined.save(str(out_png))
    print(f"  Saved: {out_png}")
    return True


def _render_matplotlib_fallback(
    entries: List[Tuple[str, "trimesh.Trimesh"]],
    out_png: Path,
) -> bool:
    """Fallback renderer using matplotlib Poly3DCollection."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        fig, axes = plt.subplots(1, 2, figsize=(16, 8), subplot_kw={"projection": "3d"})

        for ax, (azim, elev) in zip(axes, [(0, 15), (45, 30)]):
            for name, mesh in entries:
                rgba_int = _rgba_for_name(name)
                fc = [c / 255.0 for c in rgba_int[:3]] + [rgba_int[3] / 255.0]
                # Downsample triangles for speed
                verts = mesh.vertices
                faces = mesh.faces
                if len(faces) > 5000:
                    idx = np.random.default_rng(0).choice(len(faces), 5000, replace=False)
                    faces = faces[idx]
                tris = verts[faces]
                poly = Poly3DCollection(tris, alpha=fc[3] * 0.8, linewidth=0)
                poly.set_facecolor(fc[:3])
                ax.add_collection3d(poly)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.view_init(elev=elev, azim=azim)
            ax.set_aspect("equal", adjustable="box")
            # Set axis limits from all meshes
            all_v = np.vstack([m.vertices for _, m in entries if len(m.vertices) > 0])
            margin = 0.05
            ax.set_xlim(all_v[:, 0].min() - margin, all_v[:, 0].max() + margin)
            ax.set_ylim(all_v[:, 1].min() - margin, all_v[:, 1].max() + margin)
            ax.set_zlim(all_v[:, 2].min() - margin, all_v[:, 2].max() + margin)

        plt.tight_layout()
        out_png.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(str(out_png), dpi=120)
        plt.close()
        print(f"  Saved (matplotlib): {out_png}")
        return True
    except Exception as exc:
        print(f"  Matplotlib fallback also failed: {exc}")
        return False


# ---------------------------------------------------------------------------
# Meshcat viewer
# ---------------------------------------------------------------------------

def _launch_meshcat(
    entries: List[Tuple[str, "trimesh.Trimesh"]],
    port: int = 7000,
) -> None:
    """Launch a meshcat server and send all meshes."""
    try:
        import meshcat
        import meshcat.geometry as mg
        import meshcat.transformations as mt
    except ImportError:
        print("meshcat not installed; skipping browser viewer")
        return

    vis = meshcat.Visualizer().open()
    print(f"Meshcat URL: {vis.url()}")

    for name, mesh in entries:
        rgba_int = _rgba_for_name(name)
        color = int(rgba_int[0]) << 16 | int(rgba_int[1]) << 8 | int(rgba_int[2])
        opacity = rgba_int[3] / 255.0
        mat = mg.MeshLambertMaterial(color=color, opacity=opacity, transparent=(opacity < 1.0))
        path = name.replace("/", "_")
        obj = mg.TriangularMeshGeometry(
            vertices=mesh.vertices,
            faces=mesh.faces,
        )
        vis[path].set_object(obj, mat)

    print("Meshcat viewer running.  Press Ctrl+C to exit.")
    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Preset renders
# ---------------------------------------------------------------------------

def _run_presets(model: "SystemModel", out_dir: Path, verbose: bool = True) -> Dict[str, Path]:
    """Render all preset PNGs into out_dir.

    Returns dict of {label: path}.
    """
    from inchworm.system_model.model import SystemModel

    out_dir.mkdir(parents=True, exist_ok=True)
    results: Dict[str, Path] = {}

    # Canonical fold pose (task spec)
    Q_FOLD = [0.0, -1.25, 1.64, -1.31, 1.36]
    # Zero pose
    Q_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0]
    # CAD-drawn fold (from wrist part analysis)
    Q_CAD = [-0.0506, -1.578, 1.574, -1.561, 0.0]

    # ------------------------------------------------------------------
    # 1. fold.png
    # ------------------------------------------------------------------
    print("Rendering fold.png ...")
    sc = model.scene(Q_FOLD, Q_FOLD)
    p = out_dir / "fold.png"
    _render_views(sc, p)
    results["fold"] = p

    # ------------------------------------------------------------------
    # 2. measured_fold.png  (pose log, label=folded_transit, 2nd reading, pan=0)
    # ------------------------------------------------------------------
    print("Rendering measured_fold.png ...")
    pose_log = _REPO / "robot" / "calibration_data" / "poses_2026-10-25.jsonl"
    if pose_log.exists():
        q_left_m, q_right_m = _load_pose_log(pose_log, "folded_transit", pan_override=0.0)
        if q_left_m and q_right_m:
            sc = model.scene(q_left_m, q_right_m)
            p = out_dir / "measured_fold.png"
            _render_views(sc, p)
            results["measured_fold"] = p
        else:
            print("  Warning: folded_transit not found in pose log; skipping.")
    else:
        print(f"  Warning: pose log not found at {pose_log}; skipping measured_fold.")

    # ------------------------------------------------------------------
    # 3. zero.png
    # ------------------------------------------------------------------
    print("Rendering zero.png ...")
    sc = model.scene(Q_ZERO, Q_ZERO)
    p = out_dir / "zero.png"
    _render_views(sc, p)
    results["zero"] = p

    # ------------------------------------------------------------------
    # 4. cad_check.png – left arm at CAD fold in cyan, with export meshes in red
    # ------------------------------------------------------------------
    print("Rendering cad_check.png ...")
    sc_cad = model.scene(Q_CAD, Q_ZERO, include_custom_wrist=False, include_raw_exports=False)
    # Override colours: left arm gets cad_check colour
    sc_cad_colored = [(f"cad_check/{n}", m) for n, m in sc_cad if n.startswith("arm_left")]
    sc_right = [(n, m) for n, m in sc_cad if n.startswith("arm_right")]
    sc_static = [(n, m) for n, m in sc_cad if n.startswith("static") or n.startswith("dexarm") or n == "tube"]

    # Raw export meshes (red)
    from inchworm.system_model.assets import load_raw_export
    sc_exports: List[Tuple[str, "trimesh.Trimesh"]] = []
    for fname in ["wrist_camera.stl", "wrist_gripper.stl"]:
        stl = model._exports_dir / fname
        if stl.exists():
            min_v = 0.5e-6 if "gripper" in fname else 0.0
            raw = load_raw_export(stl, min_vol_m3=min_v)
            for name, m in raw.items():
                sc_exports.append((f"export_raw/{fname}/{name}", m))

    combined = sc_cad_colored + sc_right + sc_static + sc_exports
    p = out_dir / "cad_check.png"
    _render_views(combined, p)
    results["cad_check"] = p

    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inchworm robot pose viewer (no hardware access)"
    )
    parser.add_argument(
        "--q-left", nargs=5, type=float, default=None, metavar="Q",
        help="Left arm joint angles [rad] (shoulder_pan .. wrist_roll)"
    )
    parser.add_argument(
        "--q-right", nargs=5, type=float, default=None, metavar="Q",
        help="Right arm joint angles [rad]"
    )
    parser.add_argument(
        "--pose-log", default=None,
        help="JSON-lines pose log path"
    )
    parser.add_argument(
        "--label", default=None,
        help="Label to load from pose log (requires --pose-log)"
    )
    parser.add_argument(
        "--pan-zero", action="store_true",
        help="Force shoulder_pan = 0 in both arms"
    )
    parser.add_argument(
        "--png", default=None,
        help="Save PNG to this path (headless)"
    )
    parser.add_argument(
        "--png-dir", default=None,
        help="Render all preset PNGs to this directory"
    )
    parser.add_argument(
        "--meshcat", action="store_true",
        help="Launch interactive meshcat browser viewer"
    )
    parser.add_argument(
        "--port", type=int, default=7000,
        help="Meshcat port (default: 7000)"
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    # Check model extra
    try:
        import trimesh
    except ImportError:
        print(
            "ERROR: model optional extra not installed.\n"
            "Run: cd robot && uv sync --extra model"
        )
        sys.exit(1)

    from inchworm.system_model.model import SystemModel
    from inchworm.system_model.assets import load_raw_export

    model = SystemModel()

    # ------------------------------------------------------------------
    # Preset mode
    # ------------------------------------------------------------------
    if args.png_dir:
        out_dir = Path(args.png_dir)
        results = _run_presets(model, out_dir)
        print("\nPreset PNGs:")
        for label, p in results.items():
            print(f"  {label}: {p.resolve()}")

        # Self-check
        Q_CAD = [-0.0506, -1.578, 1.574, -1.561, 0.0]
        if (model._exports_dir / "wrist_camera.stl").exists():
            print("\nRunning self-check (wrist frame bookkeeping) ...")
            check = model.self_check_wrist_frame(Q_CAD)
            check_path = out_dir / "self_check.json"
            import json as _json
            with open(check_path, "w") as f:
                _json.dump(check, f, indent=2)
            print(f"  Self-check result saved to: {check_path.resolve()}")
            status = "PASS" if check.get("max_dist_mm", 999) < 1.0 else "FAIL"
            print(f"  RESULT: {status}  max={check.get('max_dist_mm', 'nan'):.3f} mm")
        return

    # ------------------------------------------------------------------
    # Single pose mode
    # ------------------------------------------------------------------
    q_left = args.q_left or [0.0] * 5
    q_right = args.q_right or [0.0] * 5

    if args.pose_log and args.label:
        ql, qr = _load_pose_log(
            Path(args.pose_log),
            args.label,
            pan_override=0.0 if args.pan_zero else None,
        )
        if ql:
            q_left = ql
        if qr:
            q_right = qr

    if args.pan_zero:
        q_left[0] = 0.0
        q_right[0] = 0.0

    scene = model.scene(q_left, q_right)

    if args.png:
        p = Path(args.png)
        _render_views(scene, p)

    if args.meshcat:
        _launch_meshcat(scene, args.port)

    if not args.png and not args.meshcat:
        print("No output requested.  Use --png PATH or --meshcat or --png-dir DIR.")
        print("Dry-run OK: model loaded, scene built.")


if __name__ == "__main__":
    main()
