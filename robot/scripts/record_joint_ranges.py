#!/usr/bin/env python3
"""Record raw per-joint servo travel ranges, READ-ONLY (AA-9 AC#1 raw data).

Written 2026-10-24 for the owner-supervised hardware session. Unlike
``measure_limits.py`` this script **never writes to the bus**: no torque
changes, no goal positions. It requires torque to already be OFF on every
servo it records (it checks register 40 and refuses otherwise), then polls
``present_position`` at ~20 Hz and tracks the min/max raw ticks per joint while
the operator sweeps each joint by hand to its mechanical/safe limits.

Ranges are recorded in RAW servo units (ticks 0-4095 and servo degrees), which
are independent of the not-yet-measured joint zero calibration. Converting them
to URDF joint radians needs ``zero_offset_deg`` and ``sign`` (AA-9
servo_calibration) and is a separate, explicit step.

Stopping: the recording ends when the file given by ``--stop-file`` appears, or
after ``--max-seconds``. This lets a supervising agent stop it on the owner's
word without an interactive terminal.

Wrap-around: STS3215 positions wrap at 4095->0. If consecutive samples of a
joint jump by more than half a turn, the joint is flagged ``wrapped`` and its
range must not be trusted.

Usage::

    uv run python scripts/record_joint_ranges.py --arm left            # dry-run
    uv run python scripts/record_joint_ranges.py --arm left --live \\
        --stop-file /tmp/stop_ranges --out out/ranges_left.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TICKS = 4096
HALF_TURN = TICKS // 2


@dataclass
class JointRange:
    name: str
    servo_id: int
    start: int
    lo: int
    hi: int
    last: int
    samples: int = 1
    wrapped: bool = False

    # Unwrapped (continuous) position in ticks relative to the start, so a
    # joint whose travel crosses the 4095->0 seam gets a correct range.
    # Orchestrator 2026-10-24: first left-arm sweep crossed the seam on 4 joints.
    cont: int = 0
    cont_lo: int = 0
    cont_hi: int = 0

    def update(self, ticks: int) -> None:
        d = ticks - self.last
        if d > HALF_TURN:
            d -= TICKS
            self.wrapped = True
        elif d < -HALF_TURN:
            d += TICKS
            self.wrapped = True
        self.cont += d
        self.cont_lo = min(self.cont_lo, self.cont)
        self.cont_hi = max(self.cont_hi, self.cont)
        self.last = ticks
        self.lo = min(self.lo, ticks)
        self.hi = max(self.hi, ticks)
        self.samples += 1

    def to_dict(self) -> dict:
        deg = lambda t: round(t * 360.0 / TICKS, 2)  # noqa: E731
        lo_abs = (self.start + self.cont_lo) % TICKS
        hi_abs = (self.start + self.cont_hi) % TICKS
        return {
            "servo_id": self.servo_id, "start_ticks": self.start,
            # Wrap-aware range: travel from the start position, unwrapped.
            "travel_lo_ticks_rel_start": self.cont_lo,
            "travel_hi_ticks_rel_start": self.cont_hi,
            "travel_span_deg": deg(self.cont_hi - self.cont_lo),
            "range_lo_servo_deg": deg(lo_abs), "range_hi_servo_deg": deg(hi_abs),
            "crosses_seam": (self.start + self.cont_lo) < 0 or (self.start + self.cont_hi) >= TICKS,
            "raw_min_max_INVALID_if_wrapped": True,
            "min_ticks": self.lo, "max_ticks": self.hi,
            "min_servo_deg": deg(self.lo), "max_servo_deg": deg(self.hi),
            "span_deg": deg(self.hi - self.lo), "samples": self.samples,
            "wrapped": self.wrapped,
        }


@dataclass
class Recorder:
    read_ticks: Callable[[int], int]
    ids: Dict[str, int]
    ranges: Dict[str, JointRange] = field(default_factory=dict)

    def start(self) -> None:
        for name, sid in self.ids.items():
            t = self.read_ticks(sid)
            self.ranges[name] = JointRange(name, sid, t, t, t, t)

    def poll(self) -> None:
        for name, sid in self.ids.items():
            self.ranges[name].update(self.read_ticks(sid))


def _status_line(rec: Recorder) -> str:
    return " | ".join(
        f"{n[:8]}:{r.last * 360 / TICKS:6.1f} [{r.lo * 360 / TICKS:5.1f},{r.hi * 360 / TICKS:5.1f}]"
        for n, r in rec.ranges.items()
    )


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--live", action="store_true", help="read the real arm (read-only)")
    ap.add_argument("--stop-file", type=Path, default=Path("/tmp/inchworm_stop_ranges"))
    ap.add_argument("--max-seconds", type=float, default=600.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    if not args.live:
        print(f"DRY-RUN: would record raw ranges for arm_{args.arm} joints {JOINTS} "
              f"(read-only, torque must already be off), stop on {args.stop_file} "
              f"or after {args.max_seconds:.0f} s.")
        return 0

    import struct
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus

    cfg = load_config()
    sj = (cfg.arms.left if args.arm == "left" else cfg.arms.right).servo_joints
    ids = {j: getattr(sj, j) for j in JOINTS}
    if any(v is None for v in ids.values()):
        print(f"REFUSED: null servo IDs in config: {[k for k, v in ids.items() if v is None]}")
        return 4
    t = live_backend(cfg).open_serial(f"arm_{args.arm}")
    bus = ArmServoBus(t, role=f"arm_{args.arm}", joint_id_map=ids)
    raw = bus._bus  # read-only register access; NOTHING in this script writes
    try:
        torque_on = {j: raw.read_bytes(s, 40, 1)[0] for j, s in ids.items()}
        if any(torque_on.values()):
            print(f"REFUSED: torque is ON for {[j for j, v in torque_on.items() if v]}; "
                  "this read-only recorder requires torque OFF on every joint.")
            return 4
        rec = Recorder(lambda sid: struct.unpack("<H", raw.read_bytes(sid, 56, 2))[0], ids)
        rec.start()
        args.stop_file.unlink(missing_ok=True)
        print(f"RECORDING arm_{args.arm} (read-only). Stop: touch {args.stop_file}", flush=True)
        t0 = time.monotonic(); last_print = 0.0
        while time.monotonic() - t0 < args.max_seconds and not args.stop_file.exists():
            rec.poll()
            if time.monotonic() - last_print > 2.0:
                print(f"[{time.monotonic() - t0:5.0f}s] {_status_line(rec)}", flush=True)
                last_print = time.monotonic()
            time.sleep(1.0 / args.hz)
    finally:
        t.close()
    result = {
        "arm": f"arm_{args.arm}", "recorded_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "method": "torque-off manual sweep, read-only present_position polling (record_joint_ranges.py)",
        "units": "raw STS3215 ticks (0-4095) and servo degrees; NOT joint radians",
        "joints": {n: r.to_dict() for n, r in rec.ranges.items()},
    }
    out = args.out or Path(__file__).resolve().parents[1] / "out" / f"joint_ranges_{args.arm}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
