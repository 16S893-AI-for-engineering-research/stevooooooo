#!/usr/bin/env python3
"""Measure the gripper ends and close an EMPTY gripper (AA-43, owner-approved 2026-11-12).

One arm, the gripper only, through the AA-17 state machine (GripperLockSM). Dry-run by default.

Actions (``--action``):
  open-end    drive slowly to the servo's EEPROM limit on the open side; report where it stopped
  closed-end  same, on the closed side
  close       close the empty gripper to the configured closed_empty position (stops with FAULT
              if the jaws stop > 20 ticks early: something is between them)
  open        open to the middle of the open band (reset())

Every move: 150 ticks/s, acceleration 20, torque limit 300 (the values that opened both grippers
2026-11-12); torque OFF afterwards on every exit path. Abort: ``touch /tmp/inchworm_stop_move``
or the arm power switch.

Refuses unless ``--no-tube`` is given (you state that the tube is NOT in either gripper), torque
is OFF on all six servos of the arm, and the gripper drive registers are non-zero.

Usage (from robot/)::

    uv run python scripts/gripper_ends.py --arm left --action open-end --no-tube            # dry-run
    uv run python scripts/gripper_ends.py --arm left --action open-end --no-tube --live \\
        --out calibration_data/gripper_ends_2026-11-12.jsonl
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

STOP_FILE = Path("/tmp/inchworm_stop_move")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ACTIONS = ("open-end", "closed-end", "close", "open")
_REG_TORQUE_ENABLE = 40


def describe(action: str) -> str:
    return {
        "open-end": "drive the gripper OPEN to the servo's EEPROM limit, report where it stops",
        "closed-end": "drive the gripper CLOSED to the servo's EEPROM limit, report where it stops",
        "close": "close the EMPTY gripper to closed_empty (FAULT if it stops > 20 ticks early)",
        "open": "open the gripper to the middle of the open band (reset())",
    }[action]


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--action", choices=ACTIONS, required=True)
    ap.add_argument("--no-tube", action="store_true", help="you state the tube is NOT in either gripper")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    role = f"arm_{a.arm}"
    if not a.no_tube:
        print("REFUSED: give --no-tube to state that the tube is NOT in either gripper"); return 2
    print(f"Plan: {role}: {describe(a.action)}. 150 ticks/s, torque limit 300, torque OFF afterwards. "
          f"Abort: touch {STOP_FILE} or the power switch.")

    from inchworm.config import load_config
    from inchworm.gripper_lock import GripperLockSM, GripperState
    cfg = load_config()
    bands = getattr(cfg.gripper_lock, role)
    T = lambda d: round(d * 4096 / 360)  # noqa: E731
    print(f"Configured: open band {T(bands.unlocked_low_deg)}..{T(bands.unlocked_high_deg)} ticks, "
          f"closed_empty {T(bands.closed_empty_deg)} ticks")
    if not a.live:
        from inchworm.backends import MockSerialTransport
        from inchworm.servo_bus import ArmServoBus
        sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
        bus = ArmServoBus(MockSerialTransport(role), role=role,
                          joint_id_map={n: getattr(sj, n) for n in JOINTS}, dry_run=True)
        sm = GripperLockSM(bus=bus, role=role, config=cfg.gripper_lock, dry_run=True)
        sm.reset()
        if a.action == "close":
            print("DRY-RUN:", sm.close_empty(tube_attached=False))
        elif a.action in ("open-end", "closed-end"):
            print("DRY-RUN:", sm.measure_end(a.action.split("-")[0], tube_attached=False))
        print("DRY-RUN: nothing opened or moved. Add --live with the owner present.")
        return 0

    from inchworm.backends import live_backend
    from inchworm.servo_bus import ArmServoBus
    from gripper_lock_probe import drive_config_faults

    STOP_FILE.unlink(missing_ok=True)
    sj = (cfg.arms.left if a.arm == "left" else cfg.arms.right).servo_joints
    jm = {n: getattr(sj, n) for n in JOINTS}
    if [jm[n] for n in JOINTS] != [1, 2, 3, 4, 5, 6]:
        print(f"REFUSED: expected servo IDs 1-6 in joint order, got {jm}"); return 4
    t = live_backend(cfg).open_serial(role)
    bus = ArmServoBus(t, role=role, joint_id_map=jm)
    raw = bus._bus
    rec: dict = {}
    try:
        on = [n for n, i in jm.items() if raw.read_bytes(i, _REG_TORQUE_ENABLE, 1)[0] != 0]
        if on:
            print(f"REFUSED: torque is ON for {on}; switch it off first"); return 4
        bad = [b for b in drive_config_faults(raw.read_bytes, 6) if not b.startswith("torque_limit")]
        if bad:
            print(f"REFUSED: gripper drive config is zeroed: {bad}"); return 5
        sm = GripperLockSM(bus=bus, role=role, config=cfg.gripper_lock, opposite_arm_locked_fn=lambda: False)
        stop = STOP_FILE.exists
        if a.action == "open":
            from powered_move_check import wait_for_goal
            start = round(bus.read_position_deg(6) * 4096 / 360)
            goal = round((bands.unlocked_low_deg + bands.unlocked_high_deg) / 2 * 4096 / 360)
            prev = raw.read_bytes(6, 48, 2)
            try:
                bus.set_goal_speed(6, 150); bus.set_acceleration(6, 20); bus.set_torque_limit(6, 300)
                bus.set_torque_enable(6, True)
                sm.reset()
                status, final, trace = wait_for_goal(bus, 6, goal, tol=12, timeout_s=20.0)
            finally:
                for step in (lambda: bus.set_torque_enable(6, False), lambda: bus.set_torque_limit(6, prev[0] | (prev[1] << 8)),
                             lambda: bus.set_goal_speed(6, 0), lambda: bus.set_acceleration(6, 0)):
                    try:
                        step()
                    except Exception as exc:  # noqa: BLE001
                        print(f"WARNING: cleanup step failed: {exc} -- switch the arm power off")
            rec = {"role": role, "context": "open", "status": status, "start_ticks": start, "goal_ticks": goal,
                   "final_ticks": final, "error_ticks": final - goal,
                   "peak_abs_load": max(abs(p["load"]) for p in trace)}
        elif a.action == "close":
            sm._state = GripperState.UNLOCKED if _in_open_band(bus, bands) else GripperState.UNKNOWN
            if sm.state != GripperState.UNLOCKED:
                print("REFUSED: close needs the gripper in the open band first (run --action open)"); return 6
            try:
                rec = sm.close_empty(tube_attached=False, should_abort=stop)
            except Exception as exc:  # noqa: BLE001
                rec = {"role": role, "context": "close_empty", "status": "FAULT", "error": str(exc)}
        else:
            rec = sm.measure_end(a.action.split("-")[0], tube_attached=False, should_abort=stop)
        rec["action"] = a.action
        rec["state_after"] = sm.state.name
    finally:
        try:
            for i in jm.values():
                if raw.read_bytes(i, _REG_TORQUE_ENABLE, 1)[0] != 0:
                    raw.write_bytes(i, _REG_TORQUE_ENABLE, b"\x00")
            rec["torque_after"] = [raw.read_bytes(i, _REG_TORQUE_ENABLE, 1)[0] for i in jm.values()]
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: final torque-off sweep failed: {exc} -- switch the arm power off")
        t.close()
    rec["utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    print(json.dumps(rec))
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        with open(a.out, "a") as fh:
            fh.write(json.dumps(rec) + "\n")
    ok = rec.get("status") in ("SETTLED", "REACHED") and not any(rec.get("torque_after", [1]))
    print("RESULT:", "OK" if ok else f"NOT OK: {rec.get('status')}")
    if "stopped_short" in rec:
        print(f"  stopped at {rec['final_ticks']} (EEPROM limit {rec['goal_ticks']}); "
              f"{'STOPPED SHORT: a mechanical stop came first' if rec['stopped_short'] else 'reached the servo limit'}")
    return 0 if ok else 1


def _in_open_band(bus, bands) -> bool:
    deg = bus.read_position_deg(6)
    return bands.unlocked_low_deg <= deg <= bands.unlocked_high_deg


if __name__ == "__main__":
    sys.exit(main())
