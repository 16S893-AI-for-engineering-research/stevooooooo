#!/usr/bin/env python3
"""AA-43 #4: READ-ONLY joint direction check. You move a joint by hand; the script reads ticks.

Torque must be OFF on the arm. Nothing is ever written to the bus. For each joint:
1. You move it CLOCKWISE as you see it from where you stand, and say which side you stand on
   (c = the side the servo's cable comes out of, o = the opposite side).
2. The script reports whether your clockwise was ticks UP or DOWN and whether that matches the
   doc-1 convention (higher ticks = clockwise with the cable pointing AWAY from the viewer, i.e.
   viewed from the side opposite the cable).
3. It also prints what the cell model (sign +1 from hardware.yaml) says the gripper tip does for
   that tick change (up/down, toward/away from the DexArm), and you answer y/n whether the real
   tip did that. That is the side-independent confirmation.

    uv run python scripts/direction_check.py --arm left                 # dry-run
    uv run python scripts/direction_check.py --arm left --live --out calibration_data/direction_check_2026-11-09.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MIN_MOVE_TICKS = 40          # about 3.5 deg: smaller is treated as "did not move"
_REG_TORQUE_ENABLE = 40
_REG_POSITION = 56


def convention_matches(delta_ticks: int, stood_on: str) -> bool:
    """doc-1: higher ticks = clockwise when the cable points AWAY from the viewer (viewer on the side opposite the cable)."""
    cw_is_ticks_up = (stood_on == "o")
    return (delta_ticks > 0) == cw_is_ticks_up


def tip_words(dxyz_mm) -> str:
    """Cell frame: +Z up, +Y away from the DexArm."""
    dx, dy, dz = dxyz_mm
    parts = []
    if abs(dz) >= 1.0:
        parts.append("UP" if dz > 0 else "DOWN")
    if abs(dy) >= 1.0:
        parts.append("AWAY from the DexArm" if dy > 0 else "TOWARD the DexArm")
    if abs(dx) >= 1.0:
        parts.append(f"sideways (cell x {dx:+.0f} mm)")
    return " and ".join(parts) if parts else "essentially stays in place (rotation about its own axis)"


def model_tip_motion(cfg, arm: str, joint: str, ticks: List[int], other_ticks: List[int], delta_ticks: int):
    """Gripper-centroid displacement (mm, cell frame) predicted by the cell model for ``delta_ticks`` on ``joint``."""
    import numpy as np
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.system_model.collision import CellState

    sc = cfg.motion_guard.servo_calibration

    def q_of(a, tk):
        blk = sc.left_arm if a == "left" else sc.right_arm
        return np.array([getattr(blk, f"{n}_sign") * math.radians(t * 360 / 4096 - getattr(blk, f"{n}_zero_offset_deg"))
                         for n, t in zip(JOINTS, tk[:5])])

    chk = build_mesh_collision_guard(cfg, "left").checker
    other = "right" if arm == "left" else "left"
    moved = list(ticks[:5])
    moved[JOINTS.index(joint)] += delta_ticks

    def tip(tk):
        qs = {arm: q_of(arm, tk), other: q_of(other, other_ticks)}
        P = chk.link_poses(CellState(qs["left"], qs["right"]))
        k = f"{arm}_gripper"
        v = np.asarray(chk._mesh[k].vertices)
        T = P[k]
        return (T[:3, :3] @ v.T).T.mean(0) + T[:3, 3]

    return tuple((tip(moved) - tip(ticks)) * 1000.0)


def read_ticks(raw) -> List[int]:
    return [struct.unpack("<H", raw.read_bytes(s, _REG_POSITION, 2))[0] for s in range(1, 7)]


def run(raw_arm, raw_other, cfg, arm: str, joints, *, prompt: Callable[[str], str] = input, out=None) -> List[dict]:
    recs = []
    for j in joints:
        before = read_ticks(raw_arm)
        other = read_ticks(raw_other)
        side = ""
        while side not in ("c", "o", "q"):
            side = prompt(f"\n[{arm} {j}] Stand where you like. Which side are you viewing the joint from?  "
                          "c = the side the cable comes out of, o = the opposite side, q = stop: ").strip().lower()
        if side == "q":
            break
        prompt(f"Now move {j} CLOCKWISE as you see it from there (a clear 10-30 degrees), hold it, then press Enter: ")
        after = read_ticks(raw_arm)
        delta = after[JOINTS.index(j)] - before[JOINTS.index(j)]
        rec = {"arm": arm, "joint": j, "viewed_from": "cable side" if side == "c" else "opposite the cable",
               "ticks_before": before[:5], "ticks_after": after[:5], "delta_ticks": delta,
               "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        if abs(delta) < MIN_MOVE_TICKS:
            rec["result"] = "NO_MOVEMENT_DETECTED"
            print(f"  ticks {before[JOINTS.index(j)]} -> {after[JOINTS.index(j)]}: no clear movement, skipped.")
        else:
            rec["your_clockwise_is"] = "ticks UP" if delta > 0 else "ticks DOWN"
            rec["matches_doc1_convention"] = convention_matches(delta, side)
            if j != "wrist_roll":
                dxyz = model_tip_motion(cfg, arm, j, before, other, delta)
                rec["model_tip_mm"] = [round(x, 1) for x in dxyz]
                rec["model_says_tip_goes"] = tip_words(dxyz)
            print(f"  ticks {before[JOINTS.index(j)]} -> {after[JOINTS.index(j)]} ({delta:+d}): your clockwise = "
                  f"{rec['your_clockwise_is']}. doc-1 convention: {'MATCH' if rec['matches_doc1_convention'] else 'MISMATCH'}.")
            if "model_says_tip_goes" in rec:
                print(f"  The model says that tick change moves the gripper tip: {rec['model_says_tip_goes']}.")
                ans = ""
                while ans not in ("y", "n"):
                    ans = prompt("  Did the real gripper tip go that way? (y/n): ").strip().lower()
                rec["real_tip_matches_model"] = ans == "y"
            else:
                print("  (wrist_roll: the model has no tip translation; the doc-1 rule is clockwise seen from the gripper tip looking back at the wrist = ticks UP.)")
        recs.append(rec)
        print(json.dumps(rec))
        if out:
            with open(out, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
    return recs


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--joints", default=",".join(JOINTS[1:]), help="default: lift, elbow, wrist_flex, wrist_roll")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    joints = [j for j in a.joints.split(",") if j]
    if any(j not in JOINTS for j in joints):
        print(f"REFUSED: joints must be from {JOINTS}"); return 2
    print(f"Plan: READ-ONLY on arm_{a.arm}: for {joints} you move each joint clockwise by hand; ticks are read, never written.")
    if not a.live:
        print("DRY-RUN: nothing opened. Add --live (torque must be off).")
        return 0
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus, JOINT_NAMES

    cfg = load_config()
    be = live_backend(cfg)
    other = "right" if a.arm == "left" else "left"
    ts = {r: be.open_serial(f"arm_{r}") for r in (a.arm, other)}
    try:
        raws = {r: ArmServoBus(ts[r], role=f"arm_{r}", joint_id_map={n: i + 1 for i, n in enumerate(JOINT_NAMES)})._bus for r in ts}
        for r, raw in raws.items():
            if any(raw.read_bytes(s, _REG_TORQUE_ENABLE, 1)[0] for s in range(1, 7)):
                print(f"REFUSED: arm_{r} has torque ON; this check is read-only and needs it off."); return 4
        run(raws[a.arm], raws[other], cfg, a.arm, joints, out=a.out)
    finally:
        for t in ts.values():
            t.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
