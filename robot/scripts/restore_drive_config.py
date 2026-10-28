#!/usr/bin/env python3
"""AA-41: restore the zeroed EEPROM drive configuration of servos 4-6 (owner-approved 2026-10-27).

Finding (AA-41 diagnosis): on both arms the firmware-3.10 servos (IDs 4 wrist_flex,
5 wrist_roll, 6 gripper) read 0 for Max_Torque_Limit, P/D, Protection_Current,
Overload_Torque, Max_Temperature, ... so they cannot produce torque.

Owner-approved plan:
* IDs 4 and 5: copy the drive registers from that arm's ID 3 (IDs 1-3 are normal
  and identical); the script checks IDs 1-3 agree and refuses otherwise.
* ID 6 (gripper): the same copy, then the LeRobot SO-101 gripper caps:
  Max_Torque_Limit 500, Protection_Current 250, Overload_Torque 25, P 16, D 32.
* Torque must be OFF on every servo of the arm (refuses otherwise). Nothing moves.
* A full register dump (0-70) of all 6 servos is saved BEFORE any write.
* EEPROM unlock (reg 55 = 0), write, re-lock (reg 55 = 1); every value read back.

Not written: ID, baud, position limits, homing offset, phase/direction, mode.

Usage (from robot/)::

    uv run python scripts/restore_drive_config.py --arm left            # dry-run: shows the diff
    uv run python scripts/restore_drive_config.py --arm left --live
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

ReadFn = Callable[[int, int, int], bytes]
WriteFn = Callable[[int, int, bytes], None]

# (name, address, length) of the drive-config registers copied from ID 3.
DRIVE_REGS: List[Tuple[str, int, int]] = [
    ("max_temperature", 13, 1),
    ("max_torque_limit", 16, 2),
    ("unloading_condition", 19, 1),
    ("led_alarm_condition", 20, 1),
    ("p_coefficient", 21, 1),
    ("d_coefficient", 22, 1),
    ("i_coefficient", 23, 1),
    ("minimum_startup_force", 24, 2),
    ("cw_dead_zone", 26, 1),
    ("ccw_dead_zone", 27, 1),
    ("protection_current", 28, 2),
    ("protective_torque", 34, 1),
    ("protection_time", 35, 1),
    ("overload_torque", 36, 1),
    ("speed_p_coefficient", 37, 1),
    ("overcurrent_protection_time", 38, 1),
    ("speed_i_coefficient", 39, 1),
]
GRIPPER_CAPS: Dict[str, int] = {  # LeRobot so_follower.configure() gripper values + its P/D defaults
    "max_torque_limit": 500,
    "protection_current": 250,
    "overload_torque": 25,
    "p_coefficient": 16,
    "d_coefficient": 32,
}
TARGET_IDS = (4, 5, 6)
REFERENCE_IDS = (1, 2, 3)
GRIPPER_ID = 6
REG_TORQUE_ENABLE = 40
REG_LOCK = 55
DUMP_LEN = 71  # registers 0..70


def _get(read: ReadFn, sid: int, addr: int, n: int) -> int:
    raw = read(sid, addr, n)
    return raw[0] if n == 1 else raw[0] | (raw[1] << 8)


def _enc(val: int, n: int) -> bytes:
    return bytes([val & 0xFF]) if n == 1 else bytes([val & 0xFF, (val >> 8) & 0xFF])


def drive_values(read: ReadFn, sid: int) -> Dict[str, int]:
    return {name: _get(read, sid, a, n) for name, a, n in DRIVE_REGS}


def plan(read: ReadFn) -> Dict[int, Dict[str, int]]:
    """Target values per servo. Raises if the reference servos disagree or look zeroed."""
    refs = {sid: drive_values(read, sid) for sid in REFERENCE_IDS}
    base = refs[REFERENCE_IDS[-1]]
    for sid, v in refs.items():
        if v != base:
            diff = {k: (v[k], base[k]) for k in v if v[k] != base[k]}
            raise SystemExit(f"REFUSED: reference servos disagree (ID {sid} vs {REFERENCE_IDS[-1]}): {diff}")
    if base["max_torque_limit"] == 0 or base["p_coefficient"] == 0:
        raise SystemExit("REFUSED: the reference servos are zeroed too; no known-good values to copy")
    out = {}
    for sid in TARGET_IDS:
        t = dict(base)
        if sid == GRIPPER_ID:
            t.update(GRIPPER_CAPS)
        out[sid] = t
    return out


def dump(read: ReadFn, ids) -> Dict[int, List[int]]:
    return {sid: list(read(sid, 0, 40) + read(sid, 40, DUMP_LEN - 40)) for sid in ids}


def apply(read: ReadFn, write: WriteFn, targets: Dict[int, Dict[str, int]], settle_s: float = 0.02) -> Dict[int, Dict[str, Tuple[int, int]]]:
    """Write targets (EEPROM unlock/relock per servo); return {sid: {reg: (wanted, read_back)}} mismatches."""
    regs = {name: (a, n) for name, a, n in DRIVE_REGS}
    bad: Dict[int, Dict[str, Tuple[int, int]]] = {}
    for sid, vals in targets.items():
        write(sid, REG_LOCK, b"\x00")
        try:
            for name, v in vals.items():
                a, n = regs[name]
                write(sid, a, _enc(v, n))
                time.sleep(settle_s)
        finally:
            write(sid, REG_LOCK, b"\x01")
        got = drive_values(read, sid)
        mism = {k: (vals[k], got[k]) for k in vals if got[k] != vals[k]}
        if _get(read, sid, REG_LOCK, 1) != 1:
            mism["eeprom_lock"] = (1, _get(read, sid, REG_LOCK, 1))
        if mism:
            bad[sid] = mism
    return bad


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--backup-dir", type=Path, default=Path(__file__).resolve().parents[1] / "calibration_data")
    a = ap.parse_args(argv)
    if not a.live:
        print(f"DRY-RUN (default): would back up all 6 servos of arm_{a.arm}, copy the drive registers "
              f"of ID 3 to IDs 4-5, and the same plus gripper caps {GRIPPER_CAPS} to ID 6. "
              "Add --live (torque must be off; nothing moves).")
        return 0

    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus

    cfg = load_config()
    role = f"arm_{a.arm}"
    sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
    jm = {n: getattr(sj, n) for n in names}
    if [jm[n] for n in names] != [1, 2, 3, 4, 5, 6]:
        print(f"REFUSED: expected servo IDs 1-6 in joint order, got {jm}"); return 4
    t = live_backend(cfg).open_serial(role)
    bus = ArmServoBus(t, role=role, joint_id_map=jm)
    rd, wr = bus._bus.read_bytes, bus._bus.write_bytes
    try:
        on = [sid for sid in range(1, 7) if _get(rd, sid, REG_TORQUE_ENABLE, 1) != 0]
        if on:
            print(f"REFUSED: torque is ON for servo(s) {on}; switch it off first"); return 4
        targets = plan(rd)
        before = dump(rd, range(1, 7))
        stamp = time.strftime("%Y-%m-%dT%H-%M-%S")
        bpath = a.backup_dir / f"eeprom_backup_{a.arm}_{stamp}_pre_drive_restore.json"
        bpath.write_text(json.dumps({"arm": a.arm, "utc": stamp, "registers_0_70": before,
                                     "targets": targets}, indent=1))
        print(f"backup: {bpath}")
        for sid, vals in targets.items():
            cur = drive_values(rd, sid)
            print(f"ID {sid}: " + ", ".join(f"{k} {cur[k]}->{v}" for k, v in vals.items() if cur[k] != v))
        bad = apply(rd, wr, targets)
        after = dump(rd, range(1, 7))
        (a.backup_dir / f"eeprom_after_{a.arm}_{stamp}_drive_restore.json").write_text(
            json.dumps({"arm": a.arm, "utc": stamp, "registers_0_70": after}, indent=1))
        untouched = [(sid, i) for sid in range(1, 7) for i in range(DUMP_LEN)
                     if before[sid][i] != after[sid][i]
                     and not (sid in targets and any(a0 <= i < a0 + n for _, a0, n in DRIVE_REGS))
                     and i not in range(56, 71)]  # 56-70 = live feedback (position, load, volt, temp, current)
        if bad:
            print(f"READ-BACK MISMATCH: {bad}"); return 3
        if untouched:
            print(f"UNEXPECTED CHANGE outside the drive registers: {untouched}"); return 3
        print("OK: all drive registers read back as written; nothing else changed.")
        return 0
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
