#!/usr/bin/env python3
"""AA-43 #5: first supervised small powered move of one joint of IDs 4-6, with full logging.

ONE joint at a time, everything else on the arm stays torque-OFF. Dry-run by default.

* wrist_flex / wrist_roll: move by +/- ``--delta-deg`` (default 5, max 10) from where it
  is and back, at low speed and a reduced torque limit, and report goal vs reached.
* gripper: commanded ONLY through the AA-17 state machine (``GripperLockSM.reset()`` =
  drive to the unlocked/open band). Verifies the open end of the gripper after the
  rebuild (the bands were assumed from the old servos). The gripper is left open,
  torque off.

Why this does not use the AA-9 GuardedArm: live SO-101 motion is blocked until the owner's
render comparison (AA-40 #6). This script is a narrow, owner-approved bring-up exception:
one joint, a few degrees, inside the EEPROM position limits (checked here with a margin,
and enforced by the servo itself), no tube, no other joint energised. It does NOT replace the
guard for any real motion.

Safety
------
* Refuses unless torque is OFF on all six servos of the arm, and the drive registers are
  non-zero. ``set_torque_enable`` sets goal := present before torque on.
* RAM limits during the move: goal speed, acceleration, torque limit (restored after).
  Nothing in EEPROM is written.
* Aborts to torque OFF on: stop file ``/tmp/inchworm_stop_move``, a stall (not at goal and
  not moving for 1 s), timeout, any servo error, Ctrl-C. Torque OFF on every exit.

    uv run python scripts/powered_move_check.py --arm left --joint wrist_flex            # dry-run
    uv run python scripts/powered_move_check.py --arm left --joint wrist_flex --live --out calibration_data/powered_move_2026-11-09.jsonl
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MOVE_JOINTS = ("wrist_flex", "wrist_roll")
STOP_FILE = Path("/tmp/inchworm_stop_move")
MAX_DELTA_DEG = 10.0
LIMIT_MARGIN_TICKS = 40
SPEED_TICKS_S = 150          # about 13 deg/s
ACCEL = 20
TORQUE_LIMIT_RAW = 300
TOL_TICKS = 6                # about 0.5 deg
TIMEOUT_S = 6.0
STALL_S = 1.0
_REG_LIMITS = (9, 11)
_REG_TORQUE_ENABLE = 40
_REG_TORQUE_LIMIT = 48


def _u16(b: bytes) -> int:
    return b[0] | (b[1] << 8)


def direction_hint(joint: str, delta_ticks: int) -> str:
    """Plain-language expectation for a move, with a side-independent check.

    Convention (doc-1, owner 2026-10-24): HIGHER ticks = clockwise when the servo's cable
    exits AWAY from the viewer. wrist_flex cables exit to the robot's left (toward the
    DexArm), so the cable-away viewer stands on the robot's right; a viewer on the cable side
    sees the opposite sense. The cell model (sign +1) gives the side-independent version:
    fewer ticks on wrist_flex lifts the gripper tip and swings it away from the DexArm.
    wrist_roll: viewed from the gripper tip looking back toward the wrist, clockwise = ticks up.
    """
    up = delta_ticks > 0
    if joint == "wrist_flex":
        return (f"ticks {'UP' if up else 'DOWN'}: {'clockwise' if up else 'counter-clockwise'} seen with the cable "
                f"pointing AWAY from you (cable-side viewer sees the opposite); gripper tip "
                f"{'DROPS and swings TOWARD the DexArm' if up else 'RISES and swings AWAY from the DexArm'}")
    if joint == "wrist_roll":
        return (f"ticks {'UP' if up else 'DOWN'}: seen from the gripper tip looking back toward the wrist, "
                f"{'clockwise' if up else 'counter-clockwise'}")
    return "gripper: opens toward its open band"


def wait_for_goal(bus, sid: int, goal_ticks: int, *, tol: int = TOL_TICKS, timeout_s: float = TIMEOUT_S,
                  stall_s: float = STALL_S, stop_file: Path = STOP_FILE, sleep=time.sleep, clock=time.monotonic):
    """Poll until within ``tol`` of goal. Returns (status, final_ticks, trace)."""
    t0 = last_move = clock()
    last = None
    trace: List[dict] = []
    while True:
        st = bus.read_joint_state(sid)
        ticks = round(st.position_deg * 4096 / 360)
        now = clock()
        trace.append({"t": round(now - t0, 3), "ticks": ticks, "speed": st.speed_raw, "load": st.load_raw,
                      "current": st.current_raw, "temp_c": st.temperature_c})
        if stop_file.exists():
            return "STOP_FILE", ticks, trace
        if abs(ticks - goal_ticks) <= tol:
            return "REACHED", ticks, trace
        if last is None or abs(ticks - last) > 1:
            last_move = now
        last = ticks
        if now - last_move > stall_s:
            return "STALLED", ticks, trace
        if now - t0 > timeout_s:
            return "TIMEOUT", ticks, trace
        sleep(0.02)


def plan_targets(start: int, delta_deg: float, lo: int, hi: int):
    """Target ticks = start +/- delta, validated inside [lo+margin, hi-margin]. Returns (out, back) or raises."""
    if not 0.5 <= abs(delta_deg) <= MAX_DELTA_DEG:
        raise ValueError(f"--delta-deg must be 0.5..{MAX_DELTA_DEG}")
    d = round(delta_deg * 4096 / 360)
    target = start + d
    if not (lo + LIMIT_MARGIN_TICKS <= start <= hi - LIMIT_MARGIN_TICKS):
        raise ValueError(f"start {start} is within {LIMIT_MARGIN_TICKS} ticks of the EEPROM limits {lo}..{hi}")
    if not (lo + LIMIT_MARGIN_TICKS <= target <= hi - LIMIT_MARGIN_TICKS):
        raise ValueError(f"target {target} would leave {lo + LIMIT_MARGIN_TICKS}..{hi - LIMIT_MARGIN_TICKS}; use the other sign")
    return target, start


def move_joint(bus, joint: str, sid: int, delta_deg: float, *, role: str, limits, stop_file: Path = STOP_FILE) -> List[dict]:
    """Out-and-back move of one non-gripper joint. Caller guarantees torque is off for all servos."""
    raw = bus._bus
    st0 = bus.read_joint_state(sid)
    start = round(st0.position_deg * 4096 / 360)
    target, back = plan_targets(start, delta_deg, *limits)
    prev_limit = _u16(raw.read_bytes(sid, _REG_TORQUE_LIMIT, 2))
    records = []
    try:
        bus.set_goal_speed(sid, SPEED_TICKS_S)
        bus.set_acceleration(sid, ACCEL)
        bus.set_torque_limit(sid, TORQUE_LIMIT_RAW)
        bus.set_torque_enable(sid, True)               # goal := present first
        for leg, goal in (("out", target), ("back", back)):
            print(f"[{leg}] expect: {direction_hint(joint, goal - (start if leg == 'out' else target))}")
            bus.write_position_deg(joint, goal * 360.0 / 4096)
            status, final, trace = wait_for_goal(bus, sid, goal, stop_file=stop_file)
            stt = bus.read_joint_state(sid)
            rec = {"arm": role, "joint": joint, "servo_id": sid, "leg": leg, "start_ticks": start,
                   "goal_ticks": goal, "final_ticks": final, "error_ticks": final - goal, "status": status,
                   "tolerance_ticks": TOL_TICKS, "speed_limit": SPEED_TICKS_S, "torque_limit_raw": TORQUE_LIMIT_RAW,
                   "peak_current": max(p["current"] for p in trace), "peak_abs_load": max(abs(p["load"]) for p in trace),
                   "temp_c": stt.temperature_c, "direction_expected": direction_hint(joint, goal - (start if leg == "out" else target)),
                   "samples": len(trace), "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            records.append(rec)
            print(json.dumps(rec))
            if status != "REACHED":
                break
            time.sleep(0.4)
    finally:
        for step in (lambda: bus.set_torque_enable(sid, False), lambda: bus.set_torque_limit(sid, prev_limit),
                     lambda: bus.set_goal_speed(sid, 0), lambda: bus.set_acceleration(sid, 0)):
            try:
                step()
            except Exception as exc:  # noqa: BLE001
                print(f"WARNING: cleanup step failed: {exc} -- switch the arm power off if still energised")
    return records


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--joint", choices=(*MOVE_JOINTS, "gripper"), required=True)
    ap.add_argument("--delta-deg", type=float, default=5.0, help=f"wrist joints only; signed, 0.5..{MAX_DELTA_DEG}")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    try:
        if a.joint != "gripper" and not 0.5 <= abs(a.delta_deg) <= MAX_DELTA_DEG:
            raise ValueError(f"--delta-deg must be 0.5..{MAX_DELTA_DEG}")
    except ValueError as exc:
        print(f"REFUSED: {exc}"); return 2
    role = f"arm_{a.arm}"
    if a.joint == "gripper":
        what = "drive the gripper to the unlocked/open band via the AA-17 state machine (reset()), leave it open, torque off"
    else:
        what = (f"move {a.joint} {a.delta_deg:+.1f} deg and back at {SPEED_TICKS_S} ticks/s, torque limit {TORQUE_LIMIT_RAW}, "
                f"tolerance {TOL_TICKS} ticks")
    if a.joint != "gripper":
        print(f"Direction: {direction_hint(a.joint, round(a.delta_deg * 4096 / 360))}")
    print(f"Plan: {role}: {what}. Everything else on the arm stays torque-OFF. Abort: touch {STOP_FILE}")
    if not a.live:
        print("DRY-RUN: nothing opened or moved. Add --live with the owner present.")
        return 0

    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus
    from gripper_lock_probe import drive_config_faults

    STOP_FILE.unlink(missing_ok=True)
    cfg = load_config()
    sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
    jm = {n: getattr(sj, n) for n in (*ARM_JOINTS, "gripper")}
    if [jm[n] for n in (*ARM_JOINTS, "gripper")] != [1, 2, 3, 4, 5, 6]:
        print(f"REFUSED: expected servo IDs 1-6 in joint order, got {jm}"); return 4
    sid = jm[a.joint]
    t = live_backend(cfg).open_serial(role)
    bus = ArmServoBus(t, role=role, joint_id_map=jm)
    raw = bus._bus
    records: List[dict] = []
    try:
        on = [n for n, i in jm.items() if raw.read_bytes(i, _REG_TORQUE_ENABLE, 1)[0] != 0]
        if on:
            print(f"REFUSED: torque is ON for {on}; switch it off first"); return 4
        bad = [b for b in drive_config_faults(raw.read_bytes, sid) if not b.startswith("torque_limit")]
        if bad:
            print(f"REFUSED: servo {sid} drive config is zeroed: {bad}"); return 5
        if a.joint == "gripper":
            from inchworm.gripper_lock import GripperLockSM
            gl = cfg.gripper_lock
            bands = getattr(gl, role)
            lo, hi = _u16(raw.read_bytes(sid, 9, 2)), _u16(raw.read_bytes(sid, 11, 2))
            prev_limit = _u16(raw.read_bytes(sid, _REG_TORQUE_LIMIT, 2))
            start = round(bus.read_joint_state(sid).position_deg * 4096 / 360)
            goal_deg = (bands.unlocked_low_deg + bands.unlocked_high_deg) / 2
            goal = round(goal_deg * 4096 / 360)
            print(f"gripper at {start} ticks, EEPROM limits {lo}..{hi}, open band goal {goal} ticks ({goal_deg:.2f} deg)")
            try:
                bus.set_goal_speed(sid, SPEED_TICKS_S); bus.set_acceleration(sid, ACCEL)
                bus.set_torque_limit(sid, TORQUE_LIMIT_RAW)
                bus.set_torque_enable(sid, True)
                sm = GripperLockSM(bus=bus, role=role, config=gl, opposite_arm_locked_fn=lambda: False, dry_run=False)
                sm.reset()
                status, final, trace = wait_for_goal(bus, sid, goal, tol=12, timeout_s=20.0)
                stt = bus.read_joint_state(sid)
                rec = {"arm": role, "joint": "gripper", "servo_id": sid, "leg": "open", "start_ticks": start,
                       "goal_ticks": goal, "final_ticks": final, "error_ticks": final - goal, "status": status,
                       "eeprom_limits": [lo, hi], "unlocked_band_ticks": [round(bands.unlocked_low_deg * 4096 / 360),
                                                                         round(bands.unlocked_high_deg * 4096 / 360)],
                       "peak_current": max(p["current"] for p in trace), "peak_abs_load": max(abs(p["load"]) for p in trace),
                       "temp_c": stt.temperature_c, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
                records.append(rec); print(json.dumps(rec))
            finally:
                for step in (lambda: bus.set_torque_enable(sid, False), lambda: bus.set_torque_limit(sid, prev_limit),
                             lambda: bus.set_goal_speed(sid, 0), lambda: bus.set_acceleration(sid, 0)):
                    try:
                        step()
                    except Exception as exc:  # noqa: BLE001
                        print(f"WARNING: cleanup step failed: {exc}")
        else:
            lim = (_u16(raw.read_bytes(sid, _REG_LIMITS[0], 2)), _u16(raw.read_bytes(sid, _REG_LIMITS[1], 2)))
            records = move_joint(bus, a.joint, sid, a.delta_deg, role=role, limits=lim)
    finally:
        try:
            for i in jm.values():
                if raw.read_bytes(i, _REG_TORQUE_ENABLE, 1)[0] != 0:
                    raw.write_bytes(i, _REG_TORQUE_ENABLE, b"\x00")
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: final torque-off sweep failed: {exc}")
        t.close()
    if a.out and records:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        with open(a.out, "a") as fh:
            for r in records:
                fh.write(json.dumps(r) + "\n")
    ok = bool(records) and all(r["status"] == "REACHED" for r in records)
    print("RESULT:", "REACHED goal within tolerance" if ok else f"NOT OK: {[r['status'] for r in records]}")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
