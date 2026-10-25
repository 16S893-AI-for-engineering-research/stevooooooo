#!/usr/bin/env python3
"""Read-only joint pose reader (both arms). NEVER writes to the bus.

Reads present position of servo IDs 1-5 (arm joints) and 6 (gripper) on each
arm, converts joints 1-5 to URDF radians using the VERIFIED
motion_guard.servo_calibration (zero offset + sign), and checks the result
against the motion_guard soft limits. Refuses if any arm servo has torque on.

    uv run python scripts/read_pose.py                       # dry-run
    uv run python scripts/read_pose.py --live --label folded \\
        --out calibration_data/poses_2026-10-25.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

J5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--label", default="pose")
    ap.add_argument("--arms", default="left,right")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    if not a.live:
        print("DRY-RUN: would read both arms' joint positions (read-only) and print URDF radians")
        return 0
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus, JOINT_NAMES

    cfg = load_config()
    be = live_backend(cfg)
    rc = 0
    for arm in a.arms.split(","):
        cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
        if cal.unverified:
            print(f"REFUSED: {arm} servo_calibration is unverified"); return 4
        lim = getattr(cfg.motion_guard, f"{arm}_arm_limits")
        t = be.open_serial(f"arm_{arm}")
        bus = ArmServoBus(t, role=f"arm_{arm}", joint_id_map={n: i + 1 for i, n in enumerate(JOINT_NAMES)})
        raw = bus._bus
        try:
            if any(raw.read_bytes(s, 40, 1)[0] for s in range(1, 7)):
                print(f"REFUSED: arm_{arm} has a servo with torque ON; this reader is read-only"); return 4
            ticks = [struct.unpack("<H", raw.read_bytes(s, 56, 2))[0] for s in range(1, 7)]
        finally:
            t.close()
        q, flags = [], []
        for j, tk in zip(J5, ticks[:5]):
            z = getattr(cal, f"{j}_zero_offset_deg"); sg = getattr(cal, f"{j}_sign")
            r = sg * math.radians(tk * 360 / 4096 - z)
            q.append(round(r, 4))
            lo = getattr(lim, f"{j}_lower_rad"); hi = getattr(lim, f"{j}_upper_rad")
            if lo is not None and hi is not None and not (lo <= r <= hi):
                flags.append(f"{j} {r:.3f} outside [{lo}, {hi}]")
        rec = {"arm": arm, "label": a.label, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "ticks": ticks, "q_rad": q, "q_deg": [round(math.degrees(x), 1) for x in q],
               "gripper_ticks": ticks[5], "soft_limit_violations": flags}
        print(json.dumps(rec))
        if flags:
            rc = 3
        if a.out:
            a.out.parent.mkdir(parents=True, exist_ok=True)
            with open(a.out, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())
