#!/usr/bin/env python3
"""Render ONE arm alone at the URDF zero pose (all joints 0) from four sides, and print the servo
ticks the current hardware.yaml calibration expects there. No hardware.

Use it to match the real arm by eye (other arm unmounted), then read the real ticks with
read_pose.py --live and compare with the table. The difference is the zero-offset correction.

    uv run python scripts/view_zero_pose.py --arm left --png out/zero_left.png
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
VIEWS = [("from the CABLE side (+X), DexArm on your left", 0, 0), ("from the OPPOSITE side (-X), DexArm on your right", 0, 180),
         ("from the FRONT (DexArm end, looking along +Y)", 0, -90), ("from ABOVE (DexArm toward the left of the image)", 89, 0)]


def expected_zero_ticks(cfg, arm: str) -> dict:
    blk = (cfg.motion_guard.servo_calibration.left_arm if arm == "left" else cfg.motion_guard.servo_calibration.right_arm)
    lim = cfg.motion_guard.left_arm_limits if arm == "left" else cfg.motion_guard.right_arm_limits
    out = {}
    for n in JOINTS:
        t = getattr(blk, f"{n}_zero_offset_deg") * 4096 / 360
        lo, hi = lim.servo_tick_limits[n] if isinstance(lim.servo_tick_limits, dict) else getattr(lim.servo_tick_limits, n)
        out[n] = (round(t), lo, hi, getattr(blk, f"{n}_sign"))
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--png", type=Path, default=None)
    a = ap.parse_args(argv)

    from inchworm.config import load_config
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.system_model.collision import CellState

    cfg = load_config()
    print(f"{a.arm} arm: servo ticks the CURRENT calibration expects at URDF zero (joint, ticks, EEPROM limits, sign):")
    for n, (t, lo, hi, s) in expected_zero_ticks(cfg, a.arm).items():
        print(f"  {n:14s} {t:5d}   [{lo}..{hi}]   sign {s:+d}{'' if lo <= t <= hi else '   <-- OUTSIDE limits'}")
    if a.png is None:
        return 0
    g = build_mesh_collision_guard(cfg, "left")
    if g is None or g.checker is None:
        print(f"mesh collision model unavailable: {g and g.load_error}", file=sys.stderr)
        return 2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    z = np.zeros(5)
    meshes = g.checker.world_meshes(CellState(z, z))
    show = [(n, m) for n, m in meshes if n.startswith(a.arm)]
    ws = np.vstack([m.vertices for _, m in show])
    c, half = (ws.min(0) + ws.max(0)) / 2, (ws.max(0) - ws.min(0)).max() / 2
    rng = np.random.default_rng(0)
    fig = plt.figure(figsize=(14, 14))
    for k, (name, elev, azim) in enumerate(VIEWS):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        for link, m in show:
            f = m.faces
            if len(f) > 3000:
                f = f[rng.choice(len(f), 3000, replace=False)]
            pc = Poly3DCollection(m.vertices[f], linewidth=0, alpha=0.8)
            pc.set_facecolor((0.20, 0.50, 0.95) if "gripper" not in link else (0.05, 0.15, 0.55))
            ax.add_collection3d(pc)
        ax.set_xlim(c[0] - half, c[0] + half); ax.set_ylim(c[1] - half, c[1] + half); ax.set_zlim(c[2] - half, c[2] + half)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=elev, azim=azim); ax.set_axis_off(); ax.set_title(name, fontsize=10)
    fig.suptitle(f"{a.arm} arm alone at URDF zero (all joints 0). Dark = gripper.", fontsize=12)
    a.png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.png, dpi=80, bbox_inches="tight")
    print(f"saved {a.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
