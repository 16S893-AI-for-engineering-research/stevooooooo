#!/usr/bin/env python3
"""Pose snapshot: read both arms (READ-ONLY), render that pose on the cell model,
and save the image next to the readings, for comparing the model with the real robot.

Owner 2026-11-11: check that poses match between the animation and real life.
Step 1 of the verification plan: you put the arms in a pose by hand with torque
OFF; this reads joints 1-5 of both arms (never writes, refuses if any torque is
on), converts with the verified servo calibration, and renders the same pose in
the cell URDF (side view from +X like the AA-45 animation, plus oblique). You
compare the render with the real arms (a photo from the same side helps) and
answer y/n; the answer is logged with the ticks.

Optionally compares the reading with a plan's start pose (``--plan-start``,
timeline JSONL), which is the same check a runner makes before moving.

    uv run python scripts/pose_snapshot.py --label fold                    # dry-run
    uv run python scripts/pose_snapshot.py --label fold --live
    uv run python scripts/pose_snapshot.py --label start --live --plan-start out/inchworm/timeline.jsonl
    # offline: re-render a logged snapshot
    uv run python scripts/pose_snapshot.py --from-log calibration_data/pose_snapshots.jsonl --label fold

Outputs: out/pose_snapshots/<label>_<time>.png (gitignored) and one JSON line in
calibration_data/pose_snapshots.jsonl (ticks, radians, contacts, owner answer).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))
sys.path.insert(0, str(_ROBOT / "scripts"))

ARMS = ("left", "right")
DEFAULT_LOG = _ROBOT / "calibration_data" / "pose_snapshots.jsonl"


def snapshot(
    cfg,
    read_ticks: Callable[[str], List[int]],
    label: str,
    *,
    out_dir: Path,
    log_path: Optional[Path],
    tube_pitches: Optional[float] = 0.0,
    plan_start: Optional[Dict[str, List[float]]] = None,
    tol_deg: float = 3.0,
    ask: Optional[Callable[[str], str]] = None,
    render_fn=None,
    checker=None,
) -> Dict[str, object]:
    """Read both arms, render, compare, log. Returns the logged record."""
    import numpy as np

    from inchworm.constants import ATTACHMENT_PITCH_MM
    from inchworm.so101_pose_reader import ticks_to_q
    from inchworm.start_pose_check import compare_start_pose
    from inchworm.system_model.collision import CellState

    ticks = {arm: [int(t) for t in read_ticks(arm)] for arm in ARMS}
    q = {arm: ticks_to_q(cfg, arm, ticks[arm]) for arm in ARMS}
    stamp = _dt.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    rec: Dict[str, object] = {"label": label, "time": stamp, "ticks": ticks,
                              "q_rad": q, "q_deg": {a: [round(math.degrees(v), 2) for v in q[a]] for a in ARMS},
                              "tube_pitches": tube_pitches}

    if checker is None:
        from inchworm.motion_guard import build_mesh_collision_guard
        g = build_mesh_collision_guard(cfg, "left")
        if g is None or g.checker is None:
            raise RuntimeError(f"mesh collision model unavailable: {g and g.load_error}")
        checker = g.checker
    if render_fn is None:
        from view_pose import render as render_fn
    tube = None if tube_pitches is None else tube_pitches * float(ATTACHMENT_PITCH_MM) / 1000.0
    st = CellState(np.array(q["left"]), np.array(q["right"]), tube_advance_m=tube)
    png = Path(out_dir) / f"{label}_{stamp}.png"
    fmt = lambda v: "[" + ", ".join(f"{math.degrees(x):.1f}" for x in v) + "] deg"
    title = f"SNAPSHOT {label}  L {fmt(q['left'])}  R {fmt(q['right'])}"
    rec["contacts"] = list(render_fn(checker, st, png, title))
    rec["png"] = str(png)

    if plan_start is not None:
        res = compare_start_pose(q, plan_start, tol_deg)
        rec["plan_start_ok"] = res.ok
        rec["plan_start_diff_deg"] = res.diff_deg
        print(res.summary())

    print(f"saved {png}")
    print(f"left  {rec['q_deg']['left']} deg\nright {rec['q_deg']['right']} deg")
    print("contacts in the model: " + ("; ".join(rec["contacts"]) if rec["contacts"] else "none"))
    if ask is not None:
        ans = ask("Open the PNG. Does the render match the real arms (pose, sides, bends)? [y/n/notes]: ").strip()
        rec["owner_match"] = ans
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with Path(log_path).open("a") as f:
            f.write(json.dumps(rec) + "\n")
        print(f"logged to {log_path}")
    return rec


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="snapshot", help="pose name, e.g. fold, start, rest")
    ap.add_argument("--live", action="store_true", help="read the arms (torque must be OFF)")
    ap.add_argument("--from-log", type=Path, default=None, help="re-render the last snapshot with --label from this log")
    ap.add_argument("--plan-start", type=Path, default=None, help="timeline JSONL; compare with its first frame")
    ap.add_argument("--tol-deg", type=float, default=3.0)
    ap.add_argument("--tube-pitches", type=float, default=0.0, help="tube advance to draw (default 0 = CAD placement); -1 = no tube")
    ap.add_argument("--out", type=Path, default=_ROBOT / "out" / "pose_snapshots")
    ap.add_argument("--log", type=Path, default=DEFAULT_LOG)
    a = ap.parse_args(argv)

    from inchworm.config import load_config
    from inchworm.start_pose_check import plan_start_from_timeline

    cfg = load_config()
    tube = None if a.tube_pitches < 0 else a.tube_pitches
    plan = plan_start_from_timeline(a.plan_start) if a.plan_start else None

    if a.from_log is not None:
        recs = [json.loads(l) for l in a.from_log.read_text().splitlines() if l.strip()]
        recs = [r for r in recs if r.get("label") == a.label]
        if not recs:
            print(f"no snapshot labelled {a.label!r} in {a.from_log}"); return 2
        last = recs[-1]
        snapshot(cfg, lambda arm: last["ticks"][arm], a.label, out_dir=a.out, log_path=None,
                 tube_pitches=tube, plan_start=plan, tol_deg=a.tol_deg)
        return 0

    print(f"Plan: READ-ONLY snapshot '{a.label}' of both arms (joints 1-5 present position); "
          f"nothing is written; refuses if any joint has torque on.")
    if not a.live:
        print("DRY-RUN: nothing opened. Add --live (torque OFF on both arms).")
        return 0

    from inchworm.backends import live_backend
    from inchworm.so101_pose_reader import PoseReadError, read_arm_ticks

    be = live_backend(cfg)
    try:
        snapshot(cfg, lambda arm: read_arm_ticks(cfg, be, arm), a.label, out_dir=a.out,
                 log_path=a.log, tube_pitches=tube, plan_start=plan, tol_deg=a.tol_deg, ask=input)
    except PoseReadError as exc:
        print(f"REFUSED: {exc}")
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
