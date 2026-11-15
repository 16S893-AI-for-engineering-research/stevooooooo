#!/usr/bin/env python3
"""AA-23: read-only readiness check for the demo start (``replay_cycle.py --stop-after-frame 42
--tube-in-grippers``). Opens both arms, reads only; nothing is written.

Ready = all 12 servos torque OFF, both grippers in their open band, and every joint within
10 deg of the (EEPROM/fold-clamped) timeline frame 0 (the replay adjusts the rest itself).
Exit 0 = ready, 1 = not ready (each problem printed), 2+ = could not read.

    uv run python scripts/start_check.py
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DEG = 360.0 / 4096


def problems(states: Dict[str, dict], start: Dict[str, List[int]], max_deg: float) -> List[str]:
    """``states`` = hold_tube.read_state() per arm; ``start`` = frame-0 ticks per arm."""
    out = []
    for arm, st in states.items():
        on = [s for s, v in st["torque"].items() if v]
        if on:
            out.append(f"{arm}: torque ON on servos {on}")
        if st["gripper_band"] != "open":
            out.append(f"{arm}: gripper is {st['gripper_band']} ({st['ticks'][6]} ticks; open band "
                       f"{st['open_band_ticks'][0]}..{st['open_band_ticks'][1]})")
        for k, j in enumerate(J5):
            d = (st["ticks"][k + 1] - start[arm][k]) * DEG
            if abs(d) > max_deg:
                out.append(f"{arm} {j}: {d:+.1f} deg from the start pose (max {max_deg:g}); move it by hand "
                           f"(reads {st['ticks'][k + 1]} ticks, start {start[arm][k]})")
    return out


def main() -> int:
    from inchworm import cycle_replay as cr
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus
    from hold_tube import read_state
    cfg = load_config()
    frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(ROOT / "out" / "inchworm" / "timeline.jsonl"),
                                   elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    start = {a: cr.q_to_ticks(cfg, a, frames[0].q[a]) for a in cr.ARMS}
    states = {}
    for arm in cr.ARMS:
        t = live_backend(cfg).open_serial(f"arm_{arm}")
        try:
            b = ArmServoBus(t, role=f"arm_{arm}", joint_id_map={n: i + 1 for i, n in enumerate((*J5, "gripper"))})
            states[arm] = read_state(b._bus.read_bytes, getattr(cfg.gripper_lock, f"arm_{arm}"))
        finally:
            t.close()
    for arm in cr.ARMS:
        st = states[arm]
        print(f"{arm:5s} torque {[st['torque'][s] for s in range(1, 7)]}  gripper {st['gripper_band']} "
              f"({st['ticks'][6]})  joints {[st['ticks'][s] for s in range(1, 6)]}  start {start[arm]}")
    probs = problems(states, start, cr.APPROACH_MAX_DEG)
    if probs:
        print("NOT READY:")
        for p in probs:
            print("  " + p)
        return 1
    print("READY: put the tube keys in both open grippers, then run replay_cycle.py --stop-after-frame 42 "
          "--tube-in-grippers --live ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
