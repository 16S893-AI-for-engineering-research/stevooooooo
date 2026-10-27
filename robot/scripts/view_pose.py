#!/usr/bin/env python3
"""
view_pose.py
============
Pose viewer for the inchworm robot.

Loads the full-system model from CAD/Assembly 1.step and renders joint poses
for both SO-101 arms in a meshcat browser viewer or as offscreen PNG snapshots.

Usage
-----
# Interactive meshcat viewer (open browser at localhost:7000):
uv run python robot/scripts/view_pose.py

# Render PNG snapshots of CAD pose, folded pose, and zero pose:
uv run python robot/scripts/view_pose.py --png-dir robot/out/system_model

# Render a specific pose from CLI:
uv run python robot/scripts/view_pose.py --q-left 0 0 0 0 0 --q-right 0 0 0 0 0 --png out.png

# Load pose log (JSON-lines with q_rad fields):
uv run python robot/scripts/view_pose.py --pose-log robot/calibration_data/poses_2026-10-25.jsonl

Requires: uv sync --extra model
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

# Ensure the package is importable
_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / "robot" / "src"))


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Inchworm robot pose viewer")
    p.add_argument(
        "--step",
        default=str(_REPO / "CAD" / "Assembly 1.step"),
        help="Path to STEP file",
    )
    p.add_argument(
        "--vendor",
        default=str(_REPO / "robot" / "vendor" / "so101"),
        help="Path to vendor/so101/ directory",
    )
    p.add_argument(
        "--q-left", nargs=5, type=float, default=None,
        metavar=("Q0", "Q1", "Q2", "Q3", "Q4"),
        help="Left arm joint angles [rad] (shoulder_pan .. wrist_roll)",
    )
    p.add_argument(
        "--q-right", nargs=5, type=float, default=None,
        metavar=("Q0", "Q1", "Q2", "Q3", "Q4"),
        help="Right arm joint angles [rad]",
    )
    p.add_argument("--png", default=None, help="Save a single PNG to this path")
    p.add_argument(
        "--png-dir", default=None,
        help="Directory to save all preset PNG snapshots (CAD, fold, zero)",
    )
    p.add_argument(
        "--pose-log", default=None,
        help="Path to JSON-lines pose log (each line has q_rad field)",
    )
    p.add_argument("--port", type=int, default=7000, help="Meshcat port")
    p.add_argument("--tube-y", type=float, default=None,
                   help="Tube axial position in STEP Y (mm)")
    return p.parse_args()


def _check_extra() -> None:
    try:
        import trimesh  # noqa: F401
    except ImportError:
        print(
            "ERROR: trimesh not installed. Run:\n"
            "  cd robot && uv sync --extra model\n"
        )
        sys.exit(1)


def main() -> None:
    args = _parse_args()
    _check_extra()

    from inchworm.system_model.arm_placer import extract_arm_placements
    from inchworm.system_model.viewer import build_scene, render_png_with_trimesh

    step = Path(args.step)
    vendor = Path(args.vendor)
    out_dir = Path(args.png_dir) if args.png_dir else None

    if not step.exists():
        print(f"ERROR: STEP file not found: {step}")
        sys.exit(1)

    print(f"Loading placements from {step.name}...")
    arms = extract_arm_placements(step, vendor)

    T_left = arms.T_world_left_mm
    T_right = arms.T_world_right_mm

    # Print summary
    def _fmt(T):
        t = T[:3, 3]
        R = T[:3, :3]
        # ZYX Euler (yaw-pitch-roll) in degrees
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
        roll  = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        yaw   = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        return (
            f"xyz=({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm  "
            f"rpy=({roll:.0f}, {pitch:.0f}, {yaw:.0f}) deg"
        )

    print(f"Left arm  base_link: {_fmt(T_left)}")
    print(f"Right arm base_link: {_fmt(T_right)}")
    sep = np.linalg.norm(T_left[:3, 3] - T_right[:3, 3])
    print(f"Arm separation: {sep:.1f} mm (expected 182.5 mm)")

    # Determine poses to render
    FOLD_Q = [0.0, -1.25, 1.64, -1.31, 1.36]  # owner-measured folded pose
    ZERO_Q = [0.0, 0.0, 0.0, 0.0, 0.0]

    if args.pose_log:
        # Load last pose from pose log
        q_left = FOLD_Q
        q_right = FOLD_Q
        try:
            log_path = Path(args.pose_log)
            with open(log_path) as fh:
                for line in fh:
                    entry = json.loads(line.strip())
                    if "q_rad" in entry:
                        q_left = entry["q_rad"][:5]
                        if len(entry["q_rad"]) >= 10:
                            q_right = entry["q_rad"][5:10]
                        else:
                            q_right = entry["q_rad"][:5]
            print(f"Loaded pose from {args.pose_log}: q_left={[round(v,3) for v in q_left]}")
        except Exception as exc:
            print(f"Warning: could not load pose log: {exc}")
    elif args.q_left is not None:
        q_left = list(args.q_left)
        q_right = list(args.q_right) if args.q_right is not None else FOLD_Q
    else:
        q_left = FOLD_Q
        q_right = FOLD_Q

    # PNG output mode
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        _render_presets(T_left, T_right, vendor, out_dir, args.tube_y)
        return

    if args.png is not None:
        out_png = Path(args.png)
        out_png.parent.mkdir(parents=True, exist_ok=True)
        print(f"Rendering PNG to {out_png}...")
        scene = build_scene(T_left, T_right, q_left, q_right, vendor, args.tube_y)
        ok = render_png_with_trimesh(scene, out_png)
        if ok:
            print(f"Saved: {out_png}")
        else:
            print("PNG render failed.")
        return

    # Interactive meshcat viewer
    # When run without --png / --png-dir in a non-interactive environment,
    # detect this and exit cleanly instead of blocking.
    if not sys.stdin.isatty() or os.environ.get('CI') or os.environ.get('INCHWORM_BACKEND') == 'mock':
        print(
            "view_pose.py: no output path given and non-interactive environment detected.\n"
            "Use --png <path> or --png-dir <dir> for headless rendering.\n"
            "Use --port <n> for interactive meshcat in a terminal."
        )
        return
    from inchworm.system_model.viewer import launch_meshcat_viewer
    launch_meshcat_viewer(
        T_left, T_right, q_left, q_right, vendor, args.tube_y, args.port
    )


def _render_presets(
    T_left: np.ndarray,
    T_right: np.ndarray,
    vendor: Path,
    out_dir: Path,
    tube_y: float | None,
) -> None:
    """Render the three preset PNGs: CAD pose, folded pose, zero pose."""
    from inchworm.system_model.viewer import build_scene, render_png_with_trimesh

    FOLD_Q = [0.0, -1.25, 1.64, -1.31, 1.36]
    ZERO_Q = [0.0, 0.0, 0.0, 0.0, 0.0]

    # The "CAD pose" is the configuration shown in the STEP file.
    # Since all arm part origins in STEP are at the base placement (identity rotation),
    # the CAD configuration is frozen - we use FOLD_Q as the best known approximation.
    presets = [
        ("cad_approx",   FOLD_Q,   FOLD_Q,   "CAD configuration (approx. folded pose)"),
        ("fold",         FOLD_Q,   FOLD_Q,   "Owner-measured folded pose q=[0,-1.25,1.64,-1.31,1.36]"),
        ("zero",         ZERO_Q,   ZERO_Q,   "All joints at zero"),
    ]

    for name, q_l, q_r, desc in presets:
        out_png = out_dir / f"robot_{name}.png"
        print(f"Rendering '{desc}' -> {out_png.name}...")
        scene = build_scene(T_left, T_right, q_l, q_r, vendor, tube_y)
        ok = render_png_with_trimesh(scene, out_png)
        if ok:
            print(f"  Saved: {out_png}")
        else:
            print(f"  FAILED: {out_png}")


if __name__ == "__main__":
    main()
