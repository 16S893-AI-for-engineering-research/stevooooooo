#!/usr/bin/env python3
"""AA-45: plan and render the inchworm print-crawl cycle from the cell URDF.

No hardware, no serial, no cameras, no laser. Builds the owner-specified crawl
cycle (see :mod:`inchworm.inchworm_plan`) as a frame-by-frame timeline on the
owner cell model (``CAD/robot_urdf_new``), checks every frame against the mesh
collision model, joint limits, the gripper invariants and the rigid-tube
constraint, and writes:

* ``out/inchworm/timeline.jsonl`` — per frame: both arms' joint angles,
  gripper lock states, tube pose, step name;
* ``out/inchworm/report.txt``    — leads with the PLACEHOLDER finding, then the
  stored-vs-IK printing-pose joint values, the cycle and the validation result;
* ``out/inchworm/cycle.gif`` (``--animate``) — one side view from +X looking
  -X (DexArm at -Y on the left, left arm, right arm, tube), with the step name,
  per-gripper lock state and tube advance overlaid, contacting parts drawn red,
  and the placeholder printing pose labelled.

Run from robot/:
    uv run python scripts/inchworm_animation.py
    uv run python scripts/inchworm_animation.py --animate --every 2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))

_COLOUR = {
    "left": (0.20, 0.60, 1.00),
    "right": (1.00, 0.40, 0.20),
    "dexarm": (0.30, 0.80, 0.30),
    "static": (0.60, 0.60, 0.60),
    "tube": (0.90, 0.85, 0.20),
    "contact": (0.95, 0.05, 0.05),
}


def _group(link: str, driver) -> str:
    from inchworm.system_model.collision import TUBE_LINK
    if link == TUBE_LINK:
        return "tube"
    arm = driver.get(link)
    if arm:
        return arm
    if link in ("baseforreal", "left_base", "right_base"):
        return "static"
    return "dexarm"


def _frame_contacts(checker, state) -> List[str]:
    out, seen = [], set()
    for arm in ("left", "right"):
        for c in checker.check(state, arm=arm, first_only=False):
            k = frozenset((c.link_a, c.link_b))
            if k not in seen:
                seen.add(k)
                out.append(c)
    return out


def animate(plan, checker, out_path: Path, every: int = 1, max_faces: int = 2000) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    from inchworm.system_model.collision import CellState

    frames = plan.frames[::max(1, every)]

    # Fixed bounds from the start frame (grippers/tube do not travel far).
    st0 = CellState(frames[0].left_q, frames[0].right_q, dexarm_q={},
                    tube_advance_m=frames[0].tube_advance_m)
    allv = np.vstack([m.vertices for _, m in checker.world_meshes(st0)])
    lo, hi = allv.min(0), allv.max(0)
    centre, half = (lo + hi) / 2, (hi - lo).max() / 2

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    rng = np.random.default_rng(0)

    def draw(idx):
        ax.clear()
        fr = frames[idx]
        st = CellState(fr.left_q, fr.right_q, dexarm_q={}, tube_advance_m=fr.tube_advance_m)
        contacts = _frame_contacts(checker, st)
        hit = {c.link_a for c in contacts} | {c.link_b for c in contacts}
        for link, m in checker.world_meshes(st):
            f = m.faces
            if len(f) > max_faces:
                f = f[rng.choice(len(f), max_faces, replace=False)]
            col = _COLOUR["contact"] if link in hit else _COLOUR[_group(link, checker._driver)]
            pc = Poly3DCollection(m.vertices[f], linewidth=0,
                                  alpha=0.9 if link in hit else 0.55)
            pc.set_facecolor(col)
            ax.add_collection3d(pc)
        for i, sl in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            sl(centre[i] - half, centre[i] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=5, azim=0)  # +X looking -X: DexArm (-Y) on the left
        ax.set_axis_off()
        lk = lambda b: "LOCKED" if b else "unlocked"
        overlay = (
            f"PLACEHOLDER printing pose (IK, seeded from folded_transit)\n"
            f"step: {fr.step}\n"
            f"left gripper: {lk(fr.left_locked)}   right gripper: {lk(fr.right_locked)}\n"
            f"tube advance: {fr.tube_advance_m * 1000:.1f} mm "
            f"({fr.tube_advance_m / (0.183676):.2f} pitch)"
        )
        if contacts:
            overlay += "\nCONTACT (red): " + "; ".join(str(c) for c in contacts[:3])
        ax.set_title(overlay, fontsize=8, loc="left")

    anim = FuncAnimation(fig, draw, frames=len(frames), interval=200)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out_path, writer=PillowWriter(fps=5), dpi=80)
    plt.close(fig)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=_ROBOT / "out" / "inchworm")
    ap.add_argument("--animate", action="store_true", help="also render cycle.gif")
    ap.add_argument("--every", type=int, default=1, help="animate every Nth frame")
    args = ap.parse_args(argv)

    from inchworm.config import load_config
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.inchworm_plan import PlanStopped, plan_cycle, write_timeline_jsonl

    cfg = load_config()
    g = build_mesh_collision_guard(cfg, "left")
    if g is None or g.checker is None:
        print(f"mesh collision model unavailable: {g and g.load_error}", file=sys.stderr)
        return 2
    checker = g.checker

    args.out.mkdir(parents=True, exist_ok=True)
    try:
        plan = plan_cycle(cfg, checker)
    except PlanStopped as exc:
        report = args.out / "report.txt"
        report.write_text(
            "AA-45 inchworm cycle plan — STOPPED\n" + "=" * 60 + "\n\n" + str(exc) + "\n"
        )
        print(f"STOPPED: {exc}")
        print(f"wrote {report}")
        return 3

    timeline = args.out / "timeline.jsonl"
    write_timeline_jsonl(plan, timeline)
    report = args.out / "report.txt"
    report.write_text(plan.report + "\n")
    print(f"wrote {timeline} ({len(plan.frames)} frames)")
    print(f"wrote {report}")
    print()
    print(plan.report)

    if args.animate:
        gif = args.out / "cycle.gif"
        animate(plan, checker, gif, every=args.every)
        print(f"wrote {gif}")

    # Exit non-zero if the cycle has violations, so callers notice the
    # placeholder is not physically achievable.
    return 0 if plan.ok else 1


if __name__ == "__main__":
    sys.exit(main())
