#!/usr/bin/env python3
"""AA-23 / AA-44: lock the RIGHT (or left) arm on the tube and leave it holding, for teleop demos.

Owner decision 2026-11-13: during demos the other arm holds the tube, gripper LOCKED, torque
on (``tube_support = locked_arm``). This tool does only that arm. It does not plan or move to
a pose: it holds the arm exactly where you placed it by hand.

Actions (dry-run unless ``--live``)::

    uv run python scripts/hold_tube.py status --arm right --live      # read-only report
    uv run python scripts/hold_tube.py lock   --arm right --tube-in-gripper --live
    uv run python scripts/hold_tube.py lock   --arm right --no-tube --live        # rehearsal
    uv run python scripts/hold_tube.py release --arm right --live [--gripper-off]

lock:
  refuses unless all six servos are torque OFF, the drive registers are sane, every joint is
  inside its EEPROM limits and the gripper reads inside its open band (open it first with
  ``scripts/gripper_ends.py --arm right --action open --no-tube``, then put the tube's
  attachment key into the open gripper). After YES: joints 1-5 torque ON holding the present
  pose (goal := present, Goal_Speed 5, accel 20, torque limit 500), then the AA-17 position lock
  (``GripperLockSM``: slow drive, torque 300, LOCKED within 12 ticks of the lock goal). On a lock
  failure the gripper goes torque-off (FAULT) and the joints keep holding until you support the
  arm and press Enter.
  On success the script EXITS and the servos keep holding by themselves (the STS3215 has no
  host watchdog), so ``lerobot-record --robot.tube_support=locked_arm`` can open this arm's
  port to check the lock. Emergency stop = the arm power switch.
release:
  joints 1-5 torque OFF after you support the arm (YES). The gripper keeps torque on (still
  locked) unless ``--gripper-off``; the gripper cannot be back-driven, so it stays closed on the
  key either way. Unlocking with the tube attached is NOT offered here (both grippers would be
  unlocked: AGENTS.md invariant 4).

Log: calibration_data/hold_tube_<date>.jsonl.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
REG_TORQUE, REG_POS, REG_MIN, REG_MAX = 0x28, 56, 9, 11
HOLD_SPEED, HOLD_ACCEL, HOLD_TORQUE = 5, 20, 500        # as the AA-50 replay energise
T = lambda d: round(d * 4096 / 360)                      # noqa: E731


class Refused(RuntimeError):
    code = 4


def _u16(b: bytes) -> int:
    return b[0] | (b[1] << 8)


def read_state(read_bytes, bands) -> dict:
    """Read-only snapshot: torque flags, ticks, EEPROM limits, gripper band verdict."""
    torque = {s: read_bytes(s, REG_TORQUE, 1)[0] for s in range(1, 7)}
    ticks = {s: _u16(read_bytes(s, REG_POS, 2)) for s in range(1, 7)}
    lim = {s: (_u16(read_bytes(s, REG_MIN, 2)), _u16(read_bytes(s, REG_MAX, 2))) for s in range(1, 7)}
    g = ticks[6]
    if T(bands.locked_low_deg) <= g <= T(bands.locked_high_deg):
        band = "locked"
    elif T(bands.unlocked_low_deg) <= g <= T(bands.unlocked_high_deg):
        band = "open"
    else:
        band = "between"
    return {"torque": torque, "ticks": ticks, "eeprom": lim, "gripper_band": band,
            "locked_band_ticks": [T(bands.locked_low_deg), T(bands.locked_high_deg)],
            "open_band_ticks": [T(bands.unlocked_low_deg), T(bands.unlocked_high_deg)]}


def lock_preflight(read_bytes, bands) -> dict:
    from inchworm.servo_checks import drive_config_faults
    st = read_state(read_bytes, bands)
    on = [s for s, v in st["torque"].items() if v]
    if on:
        raise Refused(f"torque is ON on servos {on}; switch it off first (this tool starts from a limp arm)")
    bad = [f"servo {s}: {f}" for s in range(1, 7) for f in drive_config_faults(read_bytes, s)
           if not f.startswith("torque_limit")]
    if bad:
        e = Refused(f"drive config: {bad}"); e.code = 5; raise e
    out = [f"servo {s} at {st['ticks'][s]} outside EEPROM {st['eeprom'][s]}" for s in range(1, 7)
           if not st["eeprom"][s][0] <= st["ticks"][s] <= st["eeprom"][s][1]]
    if out:
        raise Refused("; ".join(out))
    if st["gripper_band"] != "open":
        raise Refused(f"gripper at {st['ticks'][6]} ticks is not in the open band {st['open_band_ticks']}; "
                      "open it first (scripts/gripper_ends.py --action open --no-tube), then insert the key")
    return st


def energise_hold(bus) -> None:
    """Joints 1-5 torque ON at their present position (set_torque_enable writes goal := present)."""
    for s in range(1, 6):
        bus.set_goal_speed(s, HOLD_SPEED)
        bus.set_acceleration(s, HOLD_ACCEL)
        bus.set_torque_limit(s, HOLD_TORQUE)
        bus.set_torque_enable(s, True)


def lock_gripper(bus, cfg, role: str, *, clock=None, sleep=None) -> dict:
    """AA-17 position lock through GripperLockSM (the only gripper writer). Raises on failure."""
    from inchworm.gripper_lock import GripperLockSM, GripperState
    sm = GripperLockSM(bus=bus, role=role, config=cfg.gripper_lock, opposite_arm_locked_fn=lambda: False)
    if clock is not None:
        sm._clock, sm._sleep = clock, sleep
    sm._state = GripperState.UNLOCKED          # verified by lock_preflight: gripper in the open band
    sm.energise_slow()
    sm.set_approach_ready()
    verdict = sm.lock()                        # FAULT + gripper torque off on failure
    return {"verdict": verdict.name, "state": sm.state.name,
            "gripper_ticks": round(bus.read_position_deg(6) * 4096 / 360)}


def joints_off(read_bytes, write_bytes) -> List[int]:
    off = []
    for s in range(1, 6):
        if read_bytes(s, REG_TORQUE, 1)[0]:
            write_bytes(s, REG_TORQUE, b"\x00")
            off.append(s)
    return off


def run(a, *, open_bus: Callable, cfg, prompt=input, log=print, clock=None, sleep=None) -> int:
    role = f"arm_{a.arm}"
    bands = getattr(cfg.gripper_lock, role)
    bus, close = open_bus(role)
    raw_r, raw_w = bus._bus.read_bytes, bus._bus.write_bytes
    try:
        if a.action == "status":
            st = read_state(raw_r, bands)
            log({"event": "status", "arm": a.arm, **st})
            return 0
        if a.action == "release":
            st = read_state(raw_r, bands)
            log({"event": "release_start", "arm": a.arm, **st})
            if prompt(f"Support the {a.arm} arm (and the tube), then type YES to switch joints 1-5 torque OFF: ").strip() != "YES":
                log({"event": "release_not_confirmed"}); return 0
            off = joints_off(raw_r, raw_w)
            if a.gripper_off and raw_r(6, REG_TORQUE, 1)[0]:
                bus.set_torque_enable(6, False)
            log({"event": "released", "joints_off": off, "gripper_torque_off": bool(a.gripper_off),
                 **read_state(raw_r, bands)})
            return 0
        # lock
        try:
            st = lock_preflight(raw_r, bands)
        except Refused as e:
            log({"event": "refused", "reason": str(e)}); print(f"REFUSED: {e}"); return e.code
        tube = "in_gripper" if a.tube_in_gripper else "none"
        log({"event": "lock_start", "arm": a.arm, "tube": tube, **st})
        print(f"\nCONFIRM: the {a.arm} arm will power up and HOLD where it is, then the gripper locks "
              f"(slow, ~3 s).\n  - tube: {'attachment key IN the open gripper' if a.tube_in_gripper else 'NO tube (rehearsal)'}\n"
              "  - hands clear of the gripper; you are at the power switch")
        if prompt("Type YES to lock: ").strip() != "YES":
            log({"event": "lock_not_confirmed"}); print("not confirmed; nothing energised"); return 0
        energise_hold(bus)
        log({"event": "joints_holding"})
        try:
            res = lock_gripper(bus, cfg, role, clock=clock, sleep=sleep)
        except BaseException as exc:  # noqa: BLE001  (Ctrl-C too: never leave the gripper driving)
            try:
                bus.set_torque_enable(6, False)
            except Exception:  # noqa: BLE001
                pass
            log({"event": "lock_failed", "error": f"{type(exc).__name__}: {exc}", **read_state(raw_r, bands)})
            print(f"\nLOCK FAILED ({exc}). Gripper torque OFF; joints 1-5 still HOLDING.")
            try:
                prompt("Support the arm, then press Enter to switch joints 1-5 off: ")
            except (KeyboardInterrupt, EOFError):
                pass
            joints_off(raw_r, raw_w)
            log({"event": "released_after_failure"})
            return 7
        st = read_state(raw_r, bands)
        log({"event": "locked", **res, **st})
        print(f"\nLOCKED at {res['gripper_ticks']} ticks; joints 1-5 holding. The {a.arm} arm keeps holding after "
              f"this script exits. Release later with: uv run python scripts/hold_tube.py release --arm {a.arm} --live")
        return 0
    finally:
        close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", choices=("status", "lock", "release"))
    ap.add_argument("--arm", choices=("left", "right"), default="right")
    ap.add_argument("--live", action="store_true")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--tube-in-gripper", action="store_true", help="you state the tube's key is in the open gripper")
    g.add_argument("--no-tube", action="store_true", help="rehearsal: you state there is no tube")
    ap.add_argument("--gripper-off", action="store_true", help="release: also switch the gripper torque off")
    a = ap.parse_args(argv)
    if a.action is None:
        print("DRY-RUN: no action given (status | lock | release); nothing opened. See --help.")
        return 0
    if a.action == "lock" and not (a.tube_in_gripper or a.no_tube):
        print("REFUSED: lock needs --tube-in-gripper or --no-tube (your statement, logged)"); return 2
    if not a.live:
        print(f"DRY-RUN: nothing opened. '{a.action}' on arm_{a.arm} would run with --live "
              "(see --help for the exact sequence and refusals).")
        return 0
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus
    cfg = load_config()
    out = ROOT / "calibration_data" / f"hold_tube_{time.strftime('%Y-%m-%d')}.jsonl"
    fh = open(out, "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **rec}
        fh.write(json.dumps(rec, default=str) + "\n"); fh.flush()
        print(json.dumps(rec, default=str), flush=True)

    def open_bus(role):
        t = live_backend(cfg).open_serial(role)
        return ArmServoBus(t, role=role, joint_id_map={n: i + 1 for i, n in enumerate((*J5, "gripper"))}), t.close

    try:
        return run(a, open_bus=open_bus, cfg=cfg, log=log)
    finally:
        fh.close()


if __name__ == "__main__":
    sys.exit(main())
