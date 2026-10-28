#!/usr/bin/env python3
"""
view_pose.py: render the cell (owner URDF CAD/robot_urdf_new) at any arm poses.

AA-40 pose viewer. No hardware, no serial, no cameras. Renders the collision
meshes the motion guard checks, so what you see is what is checked. Parts in
contact (or within the guard margin) are drawn RED and listed.

Examples (run from robot/):
    # presets: measured fold, pan-zero reading, working pose -> PNGs
    uv run python scripts/view_pose.py --presets --out out/poses

    # one pose (rad, shoulder_pan..wrist_roll)
    uv run python scripts/view_pose.py --q-left 0 -1.4 1.4 -1.4 0 --q-right 0 -1.4 1.4 -1.4 0 --png out/p.png

    # a logged pose (poses jsonl label; joints recomputed from servo ticks)
    uv run python scripts/view_pose.py --pose-log calibration_data/poses_2026-10-25.jsonl --label folded_transit --png out/fold.png

    # a planned trajectory: JSONL with {"left": [5], "right": [5]} per line
    uv run python scripts/view_pose.py --trajectory plan.jsonl --out out/traj --every 5

    # tube in the cell: advance in attachment pitches (0 = CAD placement)
    ... --tube-pitches 1
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

_COLOUR = {
    "left": (0.20, 0.60, 1.00),
    "right": (1.00, 0.40, 0.20),
    "dexarm": (0.30, 0.80, 0.30),
    "static": (0.60, 0.60, 0.60),
    "tube": (0.90, 0.85, 0.20),
    "contact": (0.95, 0.05, 0.05),
}


def _group(link: str, driver: Dict[str, Optional[str]]) -> str:
    from inchworm.system_model.collision import TUBE_LINK
    if link == TUBE_LINK:
        return "tube"
    arm = driver.get(link)
    if arm:
        return arm
    if link in ("baseforreal", "left_base", "right_base"):
        return "static"
    return "dexarm"


def pose_from_log(path: Path, label: str, cfg) -> Tuple[np.ndarray, np.ndarray]:
    """(left_q, right_q) for a pose-log label, recomputed from servo ticks with the current calibration."""
    sc = cfg.motion_guard.servo_calibration
    found: Dict[str, np.ndarray] = {}
    for line in path.read_text().splitlines():
        d = json.loads(line)
        if d.get("label") != label:
            continue
        ticks = d.get("ticks") or d.get("all_ticks")
        blk = sc.left_arm if d["arm"] == "left" else sc.right_arm
        found[d["arm"]] = np.array([
            math.radians(getattr(blk, f"{n}_sign") * (ticks[i] * 360.0 / 4096.0 - getattr(blk, f"{n}_zero_offset_deg")))
            for i, n in enumerate(JOINTS)
        ])
    if set(found) != {"left", "right"}:
        raise SystemExit(f"label {label!r}: need both arms in {path}, found {sorted(found)}")
    return found["left"], found["right"]


def guard_contacts(checker, state):
    """Exactly what the motion guard checks: pairs involving a moving SO-101 link."""
    seen, out = set(), []
    for arm in ("left", "right"):
        for c in checker.check(state, arm=arm, first_only=False):
            k = frozenset((c.link_a, c.link_b))
            if k not in seen:
                seen.add(k)
                out.append(c)
    return out


def render(checker, state, out_png: Path, title: str = "", max_faces: int = 2500) -> List[str]:
    """Render two views (side from -X, oblique) to out_png. Returns contact strings."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    contacts = guard_contacts(checker, state)
    hit = {c.link_a for c in contacts} | {c.link_b for c in contacts}
    meshes = checker.world_meshes(state)
    rng = np.random.default_rng(0)
    allv = np.vstack([m.vertices for _, m in meshes])
    lo, hi = allv.min(0), allv.max(0)
    centre, half = (lo + hi) / 2, (hi - lo).max() / 2

    fig = plt.figure(figsize=(16, 8))
    for k, (elev, azim, name) in enumerate([(5, 180, "side (from -X)"), (25, 225, "oblique")]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        for link, m in meshes:
            g = _group(link, checker._driver)
            col = _COLOUR["contact"] if link in hit else _COLOUR[g]
            f = m.faces
            if len(f) > max_faces:
                f = f[rng.choice(len(f), max_faces, replace=False)]
            pc = Poly3DCollection(m.vertices[f], linewidth=0, alpha=0.9 if link in hit else 0.55)
            pc.set_facecolor(col)
            ax.add_collection3d(pc)
        for i, setlim in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            setlim(centre[i] - half, centre[i] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("X"); ax.set_ylabel("Y (DexArm at -Y)"); ax.set_zlabel("Z")
        ax.set_title(name)
    txt = title + ("\nCONTACTS: " + "; ".join(str(c) for c in contacts[:6]) if contacts else "\nno contacts")
    fig.suptitle(txt, fontsize=10)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=90)
    plt.close(fig)
    return [str(c) for c in contacts]


def _fmt(q: Sequence[float]) -> str:
    return "[" + ", ".join(f"{math.degrees(v):.1f}" for v in q) + "] deg"


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--q-left", nargs=5, type=float)
    p.add_argument("--q-right", nargs=5, type=float)
    p.add_argument("--pose-log", type=Path)
    p.add_argument("--label")
    p.add_argument("--trajectory", type=Path, help="JSONL: one {left:[5], right:[5]} per line (rad)")
    p.add_argument("--every", type=int, default=1, help="render every Nth trajectory sample")
    p.add_argument("--presets", action="store_true")
    p.add_argument("--tube-pitches", type=float, default=None, help="tube advance in attachment pitches (omit: no tube)")
    p.add_argument("--dexarm", default=None, help='DexArm joints as JSON, e.g. \'{"bases": 1.2}\'')
    p.add_argument("--png", type=Path)
    p.add_argument("--out", type=Path, default=_ROBOT / "out" / "poses")
    p.add_argument("--check-only", action="store_true", help="print contacts, no rendering")
    args = p.parse_args(argv)

    from inchworm.config import load_config
    from inchworm.constants import ATTACHMENT_PITCH_MM
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.system_model.collision import CellState

    cfg = load_config()
    g = build_mesh_collision_guard(cfg, "left")
    if g is None or g.checker is None:
        print(f"mesh collision model unavailable: {g and g.load_error}", file=sys.stderr)
        return 2
    checker = g.checker
    tube = None if args.tube_pitches is None else args.tube_pitches * float(ATTACHMENT_PITCH_MM) / 1000.0
    dex = json.loads(args.dexarm) if args.dexarm else {}

    jobs: List[Tuple[str, np.ndarray, np.ndarray]] = []
    if args.presets:
        log = _ROBOT / "calibration_data" / "poses_2026-10-25.jsonl"
        jobs.append(("measured_fold", *pose_from_log(log, "folded_transit", cfg)))
        jobs.append(("pan_zero_reading", *pose_from_log(log, "pan_zero_pointing_at_dexarm", cfg)))
        w = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
        jobs.append(("owner_working_pose", w, w))
        for np_ in cfg.motion_guard.named_poses:
            q = np.array(np_.q_rad)
            jobs.append((f"named_{np_.name}", q, q))
    if args.pose_log and args.label:
        jobs.append((args.label, *pose_from_log(args.pose_log, args.label, cfg)))
    if args.q_left is not None or args.q_right is not None:
        jobs.append(("pose", np.array(args.q_left or [0.0] * 5), np.array(args.q_right or [0.0] * 5)))
    if args.trajectory:
        lines = [json.loads(l) for l in args.trajectory.read_text().splitlines() if l.strip()]
        for i, d in enumerate(lines):
            if i % max(1, args.every) == 0 or i == len(lines) - 1:
                jobs.append((f"traj_{i:04d}", np.array(d["left"]), np.array(d["right"])))
    if not jobs:
        p.print_help()
        print("\nnothing to show: give --presets, --q-left/--q-right, --pose-log+--label or --trajectory")
        return 0

    worst = 0
    for name, ql, qr in jobs:
        st = CellState(ql, qr, dexarm_q=dex, tube_advance_m=tube)
        title = f"{name}  L {_fmt(ql)}  R {_fmt(qr)}  tube={'none' if tube is None else f'{args.tube_pitches:g} pitch'}"
        if args.check_only:
            contacts = [str(c) for c in guard_contacts(checker, st)]
        else:
            out = args.png if (args.png and len(jobs) == 1) else args.out / f"{name}.png"
            contacts = render(checker, st, out, title)
            print(f"saved {out}")
        print(f"{name}: L {_fmt(ql)} R {_fmt(qr)} -> {'; '.join(contacts) if contacts else 'no contacts'}")
        worst = max(worst, len(contacts))
    return 1 if worst else 0


if __name__ == "__main__":
    sys.exit(main())
