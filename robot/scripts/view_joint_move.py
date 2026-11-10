#!/usr/bin/env python3
"""Show what the cell model predicts for ONE joint move, from both sides. No hardware.

Rows: the measured rest pose, then the joint moved by -DELTA and +DELTA degrees of servo
ticks (ticks DOWN / ticks UP). Columns: seen from the CABLE side (+X; DexArm on the viewer's
left) and from the OPPOSITE side (-X; DexArm on the viewer's right). The moved link is drawn
dark, the rest of that arm blue (left) or orange (right), static parts grey.

Geometry (cell frame): +Z up, -Y toward the DexArm, +X toward the cable exits of joints 2-4.

    uv run python scripts/view_joint_move.py --arm left --joint wrist_flex --delta-deg 20 --png out/wrist_flex.png
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
# Measured at rest 2026-11-09 after the rebuild (read_pose.py --label post_move).
REST_TICKS = {"left": [1948, 1425, 3119, 1006, 2035], "right": [2089, 1490, 3078, 920, 1719]}
# Link that the joint moves (and everything downstream of it is drawn dark too).
MOVED = {
    "wrist_flex": ("wrist", "camera_mount", "gripper"),
    "wrist_roll": ("gripper", "camera_mount"),
    "elbow_flex": ("lower_arm", "wrist", "camera_mount", "gripper"),
    "shoulder_lift": ("upper_arm", "lower_arm", "wrist", "camera_mount", "gripper"),
    "shoulder_pan": ("shoulder", "upper_arm", "lower_arm", "wrist", "camera_mount", "gripper"),
}


def q_from_ticks(cfg, arm: str, ticks: List[int]) -> np.ndarray:
    sc = cfg.motion_guard.servo_calibration
    blk = sc.left_arm if arm == "left" else sc.right_arm
    return np.array([getattr(blk, f"{n}_sign") * math.radians(t * 360 / 4096 - getattr(blk, f"{n}_zero_offset_deg"))
                     for n, t in zip(JOINTS, ticks)])


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--joint", choices=JOINTS, default="wrist_flex")
    ap.add_argument("--delta-deg", type=float, default=20.0)
    ap.add_argument("--png", type=Path, default=_ROBOT / "out" / "joint_move.png")
    a = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    from inchworm.config import load_config
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.system_model.collision import CellState

    cfg = load_config()
    g = build_mesh_collision_guard(cfg, "left")
    if g is None or g.checker is None:
        print(f"mesh collision model unavailable: {g and g.load_error}", file=sys.stderr)
        return 2
    chk = g.checker
    other = "right" if a.arm == "left" else "left"
    j = JOINTS.index(a.joint)
    d_ticks = round(a.delta_deg * 4096 / 360)
    dark = {f"{a.arm}_{s}" for s in MOVED[a.joint]}

    def state(delta):
        t = list(REST_TICKS[a.arm]); t[j] += delta
        qs = {a.arm: q_from_ticks(cfg, a.arm, t), other: q_from_ticks(cfg, other, REST_TICKS[other])}
        return CellState(qs["left"], qs["right"])

    cases = [("REST (as measured)", 0), (f"ticks DOWN {a.delta_deg:g} deg  (-{d_ticks})", -d_ticks),
             (f"ticks UP {a.delta_deg:g} deg  (+{d_ticks})", +d_ticks)]
    views = [("from the CABLE side (+X)\nDexArm on your LEFT", 5, 0), ("from the OPPOSITE side (-X)\nDexArm on your RIGHT", 5, 180)]

    tip_rest = None
    fig = plt.figure(figsize=(12, 15))
    rng = np.random.default_rng(0)
    centre = None
    for r, (title, delta) in enumerate(cases):
        meshes = chk.world_meshes(state(delta))
        gm = dict(meshes)
        gv = np.asarray(gm[f"{a.arm}_gripper"].vertices).mean(0)
        if tip_rest is None:
            tip_rest = gv
        move_mm = (gv - tip_rest) * 1000
        if centre is None:
            ws = np.vstack([m.vertices for n, m in meshes if n.startswith(a.arm)])
            centre, half = (ws.min(0) + ws.max(0)) / 2, (ws.max(0) - ws.min(0)).max() / 2 * 0.9
        for c, (vname, elev, azim) in enumerate(views):
            ax = fig.add_subplot(3, 2, r * 2 + c + 1, projection="3d")
            for link, m in meshes:
                if link in dark:
                    col, al = (0.05, 0.15, 0.55) if a.arm == "left" else (0.65, 0.2, 0.0), 0.95
                elif link.startswith(a.arm):
                    col, al = ((0.45, 0.70, 1.0) if a.arm == "left" else (1.0, 0.65, 0.4)), 0.55
                elif link.startswith(other):
                    col, al = (0.8, 0.8, 0.8), 0.25
                else:
                    col, al = (0.6, 0.6, 0.6), 0.3
                f = m.faces
                if len(f) > 2000:
                    f = f[rng.choice(len(f), 2000, replace=False)]
                pc = Poly3DCollection(m.vertices[f], linewidth=0, alpha=al)
                pc.set_facecolor(col)
                ax.add_collection3d(pc)
            ax.set_xlim(centre[0] - half, centre[0] + half)
            ax.set_ylim(centre[1] - half, centre[1] + half)
            ax.set_zlim(centre[2] - half, centre[2] + half)
            ax.set_box_aspect((1, 1, 1))
            ax.view_init(elev=elev, azim=azim)
            ax.set_axis_off()
            ax.set_title(f"{title}\n{vname}" if c == 0 else vname, fontsize=9)
        fig.text(0.5, 0.96 - r * 0.325, f"gripper centroid vs rest: {move_mm[2]:+.1f} mm up, {move_mm[1]:+.1f} mm away from DexArm (+Y)",
                 ha="center", fontsize=9)
    # Rotation sense of the model for ticks DOWN, from the gripper's motion about the joint origin.
    first = {"wrist_flex": "wrist", "elbow_flex": "lower_arm", "shoulder_lift": "upper_arm"}.get(a.joint)
    if first:
        P0, P1 = chk.link_poses(state(0)), chk.link_poses(state(-d_ticks))
        o = P0[f"{a.arm}_{first}"][:3, 3]
        w = np.cross(P0[f"{a.arm}_gripper"][:3, 3] - o, P1[f"{a.arm}_gripper"][:3, 3] - P0[f"{a.arm}_gripper"][:3, 3])
        wx = w[0] / np.linalg.norm(w)
        print(f"model: {a.joint} ticks DOWN = {'CLOCKWISE' if wx < 0 else 'COUNTER-clockwise'} seen from the cable side (+X), "
              f"{'COUNTER-clockwise' if wx < 0 else 'CLOCKWISE'} seen from the opposite side (-X); ticks UP is the reverse.")
    fig.suptitle(f"{a.arm} arm, {a.joint}: model prediction (exaggerated to {a.delta_deg:g} deg; the live test used 5)\n"
                 "dark = parts the joint moves", fontsize=11)
    a.png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.png, dpi=80, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {a.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
