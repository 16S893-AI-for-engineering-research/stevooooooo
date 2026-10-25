#!/usr/bin/env python3
"""Read-only gripper position sampler for AA-17 band measurement.

Takes N readings of one arm's gripper servo (ID from config) and prints the
mean, min and max position in servo degrees and raw ticks, plus load and
current. NEVER writes to the bus. Refuses if gripper torque is on.

Used by the orchestrator in an owner-supervised session: the owner holds the
gripper in a named state (fully open / engaged-but-unlocked / locked) and the
sample is labelled with that state. Results append to a JSON-lines file.

    uv run python scripts/gripper_sample.py --arm left --label open            # dry-run
    uv run python scripts/gripper_sample.py --arm left --label open --live \\
        --out calibration_data/gripper_samples_2026-10-25.jsonl
"""
from __future__ import annotations

import argparse
import json
import statistics
import struct
import sys
import time
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    if not a.live:
        print(f"DRY-RUN: would sample arm_{a.arm} gripper x{a.n} labelled {a.label!r} (read-only)")
        return 0
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus, JOINT_NAMES, _decode_load_sign_magnitude
    cfg = load_config()
    sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
    sid = sj.gripper
    if sid is None:
        print("REFUSED: gripper servo ID is null"); return 4
    t = live_backend(cfg).open_serial(f"arm_{a.arm}")
    bus = ArmServoBus(t, role=f"arm_{a.arm}", joint_id_map={n: i + 1 for i, n in enumerate(JOINT_NAMES)})
    raw = bus._bus
    try:
        if raw.read_bytes(sid, 40, 1)[0] != 0:
            print("REFUSED: gripper torque is ON; this sampler is read-only and needs torque off"); return 4
        ticks, loads, cur = [], [], []
        for _ in range(a.n):
            blk = raw.read_bytes(sid, 56, 15)
            ticks.append(struct.unpack_from("<H", blk, 0)[0])
            loads.append(_decode_load_sign_magnitude(struct.unpack_from("<H", blk, 4)[0]))
            cur.append(struct.unpack_from("<H", blk, 13)[0])
            time.sleep(0.02)
    finally:
        t.close()
    deg = lambda x: round(x * 360 / 4096, 2)  # noqa: E731
    rec = {
        "arm": f"arm_{a.arm}", "servo_id": sid, "label": a.label,
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "n": a.n,
        "ticks_mean": round(statistics.mean(ticks), 1), "ticks_min": min(ticks), "ticks_max": max(ticks),
        "deg_mean": deg(statistics.mean(ticks)), "deg_min": deg(min(ticks)), "deg_max": deg(max(ticks)),
        "load_raw_mean": round(statistics.mean(loads), 1), "current_raw_mean": round(statistics.mean(cur), 1),
    }
    print(json.dumps(rec))
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        with open(a.out, "a") as fh:
            fh.write(json.dumps(rec) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
