#!/usr/bin/env python3
"""AA-43: half-turn homing offsets and position limits for selected servos (EEPROM, owner-supervised).

Same procedure as the 2026-10-24 calibration (robot/calibration_data/README.md),
packaged as a script for the replacement servos 4-6.

Subcommands
-----------
``homing``: the owner holds each target joint at mid-travel; for each target servo
  the homing offset (reg 31, sign-magnitude bit 11) is first set to 0, the raw
  position read, and the offset set so that the pose reads 2047.
``limits``: from a read-only sweep (``record_joint_ranges.py --live --out ...``),
  Min/Max_Position_Limit (regs 9/11) = swept range -/+ margin (default 30 ticks),
  refused if the sweep wrapped, crossed the seam, or would exceed 0..4095.

Safety
------
* Dry-run by default: prints the plan, opens nothing. ``--live`` is explicit.
* Refuses unless torque is OFF on every servo of the arm. Nothing moves.
* Full register dump (0-70) of all 6 servos saved before any write; another after.
* EEPROM unlock (reg 55 = 0) per servo, re-lock (reg 55 = 1) in ``finally``.
* Every value read back; any change outside the written registers is reported.
* Never writes ID, baud, mode, phase, drive or torque registers.

Usage (from robot/)::

    uv run python scripts/home_servos.py homing --arm left --ids 4,5,6            # dry-run
    uv run python scripts/home_servos.py homing --arm left --ids 4,5,6 --live
    uv run python scripts/record_joint_ranges.py --arm left --live --out out/ranges_left.json
    uv run python scripts/home_servos.py limits --arm left --ids 4,5,6 --ranges out/ranges_left.json --live
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Tuple

ReadFn = Callable[[int, int, int], bytes]
WriteFn = Callable[[int, int, bytes], None]

REG_MIN_LIMIT = 9
REG_MAX_LIMIT = 11
REG_HOMING_OFFSET = 31
REG_TORQUE_ENABLE = 40
REG_LOCK = 55
REG_PRESENT_POSITION = 56
DUMP_LEN = 71
HALF_TURN = 2047
MAX_OFFSET = 2047          # sign-magnitude, 11-bit magnitude
FEEDBACK_REGS = range(56, 71)
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def encode_offset(v: int) -> int:
    """Signed homing offset -> STS3215 sign-magnitude (bit 11 = negative)."""
    if abs(v) > MAX_OFFSET:
        raise ValueError(f"homing offset {v} out of range +/-{MAX_OFFSET}")
    return (abs(v) | 0x800) if v < 0 else v


def decode_offset(raw: int) -> int:
    return -(raw & 0x7FF) if raw & 0x800 else raw & 0x7FF


def u16(read: ReadFn, sid: int, addr: int) -> int:
    r = read(sid, addr, 2)
    return r[0] | (r[1] << 8)


def le16(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def dump(read: ReadFn, ids: Iterable[int]) -> Dict[int, List[int]]:
    return {sid: list(read(sid, 0, 40) + read(sid, 40, DUMP_LEN - 40)) for sid in ids}


def _eeprom_write(write: WriteFn, sid: int, addr: int, data: bytes, settle_s: float) -> None:
    write(sid, REG_LOCK, b"\x00")
    try:
        write(sid, addr, data)
        time.sleep(settle_s)
    finally:
        write(sid, REG_LOCK, b"\x01")


def plan_limits(ranges: dict, ids_by_joint: Dict[str, int], targets: Iterable[int], margin: int) -> Dict[int, Tuple[int, int]]:
    """{sid: (min, max)} from a record_joint_ranges.py result. Refuses unsafe sweeps."""
    out: Dict[int, Tuple[int, int]] = {}
    by_id = {sid: j for j, sid in ids_by_joint.items()}
    for sid in targets:
        j = by_id[sid]
        r = ranges["joints"][j]
        if r["servo_id"] != sid:
            raise SystemExit(f"REFUSED: sweep has {j} at ID {r['servo_id']}, config says {sid}")
        if r["wrapped"] or r["crosses_seam"]:
            raise SystemExit(f"REFUSED: {j} sweep wrapped/crossed the 4095->0 seam; re-home first")
        lo = r["start_ticks"] + r["travel_lo_ticks_rel_start"] - margin
        hi = r["start_ticks"] + r["travel_hi_ticks_rel_start"] + margin
        if lo < 0 or hi > 4095:
            raise SystemExit(f"REFUSED: {j} limits {lo}..{hi} with margin {margin} leave 0..4095")
        if hi - lo < 2 * margin + 50:
            raise SystemExit(f"REFUSED: {j} swept span {hi - lo - 2 * margin} ticks; joint not swept?")
        out[sid] = (lo, hi)
    return out


def do_homing(read: ReadFn, write: WriteFn, targets: Iterable[int], settle_s: float = 0.05) -> Dict[int, dict]:
    res: Dict[int, dict] = {}
    for sid in targets:
        _eeprom_write(write, sid, REG_HOMING_OFFSET, le16(0), settle_s)
        if decode_offset(u16(read, sid, REG_HOMING_OFFSET)) != 0:
            raise SystemExit(f"READ-BACK MISMATCH: ID {sid} homing offset did not clear to 0")
        raw = u16(read, sid, REG_PRESENT_POSITION)
        off = raw - HALF_TURN
        _eeprom_write(write, sid, REG_HOMING_OFFSET, le16(encode_offset(off)), settle_s)
        got = decode_offset(u16(read, sid, REG_HOMING_OFFSET))
        pos = u16(read, sid, REG_PRESENT_POSITION)
        res[sid] = {"raw_position": raw, "offset": off, "offset_read_back": got, "position_after": pos}
        if got != off or abs(pos - HALF_TURN) > 3:
            raise SystemExit(f"READ-BACK MISMATCH: ID {sid} {res[sid]}")
    return res


def do_limits(read: ReadFn, write: WriteFn, limits: Dict[int, Tuple[int, int]], settle_s: float = 0.05) -> Dict[int, dict]:
    res: Dict[int, dict] = {}
    for sid, (lo, hi) in limits.items():
        _eeprom_write(write, sid, REG_MIN_LIMIT, le16(lo), settle_s)
        _eeprom_write(write, sid, REG_MAX_LIMIT, le16(hi), settle_s)
        got = (u16(read, sid, REG_MIN_LIMIT), u16(read, sid, REG_MAX_LIMIT))
        res[sid] = {"min": lo, "max": hi, "read_back": list(got)}
        if got != (lo, hi):
            raise SystemExit(f"READ-BACK MISMATCH: ID {sid} limits {got} != {(lo, hi)}")
    return res


def unexpected_changes(before: Dict[int, List[int]], after: Dict[int, List[int]], written: Dict[int, set]) -> List[Tuple[int, int]]:
    return [(sid, i) for sid in before for i in range(DUMP_LEN)
            if before[sid][i] != after[sid][i] and i not in written.get(sid, set()) and i not in FEEDBACK_REGS]


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("homing", "limits"))
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--ids", default="4,5,6", help="comma-separated servo IDs (default 4,5,6)")
    ap.add_argument("--ranges", type=Path, help="record_joint_ranges.py JSON (limits only)")
    ap.add_argument("--margin", type=int, default=30, help="limit margin in ticks (default 30)")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--backup-dir", type=Path, default=Path(__file__).resolve().parents[1] / "calibration_data")
    a = ap.parse_args(argv)
    targets = [int(x) for x in a.ids.split(",")]
    if any(t not in range(1, 7) for t in targets):
        print(f"REFUSED: IDs must be within 1-6, got {targets}"); return 4
    if a.action == "limits" and a.ranges is None:
        print("REFUSED: limits needs --ranges <record_joint_ranges.py JSON>"); return 4

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from inchworm.config import load_config
    cfg = load_config()
    sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
    jm = {n: getattr(sj, n) for n in JOINTS}
    if [jm[n] for n in JOINTS] != [1, 2, 3, 4, 5, 6]:
        print(f"REFUSED: expected servo IDs 1-6 in joint order, got {jm}"); return 4
    limits = plan_limits(json.loads(a.ranges.read_text()), jm, targets, a.margin) if a.action == "limits" else None

    if not a.live:
        names = {sid: j for j, sid in jm.items()}
        if a.action == "homing":
            print(f"DRY-RUN: arm_{a.arm}, hold {[names[t] for t in targets]} at MID-TRAVEL. Would set each "
                  "homing offset so that pose reads 2047. Torque must be off; nothing moves. Add --live.")
        else:
            print(f"DRY-RUN: arm_{a.arm} would write Min/Max_Position_Limit "
                  + ", ".join(f"{names[s]} (ID {s}) {lo}..{hi}" for s, (lo, hi) in limits.items()) + ". Add --live.")
        return 0

    from inchworm.backends import live_backend
    from inchworm.servo_bus import ArmServoBus
    role = f"arm_{a.arm}"
    t = live_backend(cfg).open_serial(role)
    bus = ArmServoBus(t, role=role, joint_id_map=jm)
    raw = bus._bus
    read, write = raw.read_bytes, raw.write_bytes
    try:
        on = [sid for sid in range(1, 7) if read(sid, REG_TORQUE_ENABLE, 1)[0] != 0]
        if on:
            print(f"REFUSED: torque is ON for servo(s) {on}; switch it off first"); return 4
        before = dump(read, range(1, 7))
        stamp = time.strftime("%Y-%m-%dT%H-%M-%S")
        bpath = a.backup_dir / f"eeprom_backup_{a.arm}_{stamp}_pre_{a.action}.json"
        bpath.write_text(json.dumps({"arm": a.arm, "utc": stamp, "registers_0_70": before,
                                     "targets": targets, "limits": limits}, indent=1))
        print(f"backup: {bpath}")
        if a.action == "homing":
            res = do_homing(read, write, targets)
            written = {sid: {REG_HOMING_OFFSET, REG_HOMING_OFFSET + 1} for sid in targets}
        else:
            res = do_limits(read, write, limits)
            written = {sid: {9, 10, 11, 12} for sid in targets}
        after = dump(read, range(1, 7))
        (a.backup_dir / f"eeprom_after_{a.arm}_{stamp}_{a.action}.json").write_text(
            json.dumps({"arm": a.arm, "utc": stamp, "registers_0_70": after, "result": res}, indent=1))
        for sid, r in res.items():
            print(f"ID {sid}: {r}")
        bad = unexpected_changes(before, after, written)
        if bad:
            print(f"UNEXPECTED CHANGE outside the written registers: {bad}"); return 3
        print("OK: all values read back as written; nothing else changed.")
        return 0
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
