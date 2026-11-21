#!/usr/bin/env python3
"""AA-54 (owner plan 2026-11-21): put the tube in the HOLDING gripper at a frozen training position.

Start AFTER ``scripts/replay_cycle.py --stop-after-frame <frame of train_<arm>>`` WITHOUT the tube
(``record_train.sh`` runs both). Both arms then hold at the frozen training position; the replay
left the holding gripper LOCKED (empty) and the inserting gripper open.

1. Read-only: both arms within POSE_TOL_TICKS of the frozen ticks (config/training_poses.json),
   joints torque ON, inserting gripper open, holding gripper LOCKED (adopted by read-back).
2. You confirm the tube is NOT attached to anything (typed YES). Only then the empty holding
   gripper opens (AA-17 state machine reset, slow); the inserting gripper is never commanded.
3. You hold the tube, put its attachment key in the open holding gripper, type YES; the holding
   gripper locks (GripperLockSM, 45-degree turn); you look: y = key in and locked.
4. The handoff state is written (tube in the grippers, tube pose = the frozen one) for the
   LeRobot follower (tube_support=handoff). Both arms keep holding when this exits.

Any stop holds both arms; the grippers stay as they are. ``touch /tmp/inchworm_stop_move`` = stop.

    uv run python scripts/place_training_tube.py --arm left --dry-run    # simulated servos
    uv run python scripts/place_training_tube.py --arm left --live
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

ROOT = Path(__file__).resolve().parents[1]
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
STATE = ROOT / "out" / "handoff_state.json"
STOP_FILE = Path("/tmp/inchworm_stop_move")
POSE_TOL_TICKS = 34                       # = inchworm.handoff.POSE_TOL_TICKS


class Stop(RuntimeError):
    pass


def _digest(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def pose_problems(r, pos: dict, tol: int = POSE_TOL_TICKS) -> list:
    out = []
    for a, want in pos["joint_ticks"].items():
        now = r._ticks(a)
        d = [n - w for n, w in zip(now, want)]
        if max(abs(x) for x in d) > tol:
            out.append(f"{a} reads {now}, frozen {want} (diff {d}; max {tol})")
    return out


def place(cfg, r, arm: str, pos: dict, ask, say=print, log=lambda rec: None) -> None:
    """Steps 1-3 on an already built runner (real or simulated buses). Raises Stop."""
    from inchworm.gripper_lock import GripperState
    hold = pos["holding_arm"]
    probs = pose_problems(r, pos)
    if probs:
        raise Stop("not at the frozen training position: " + "; ".join(probs))
    states = r.adopt_holding(require={arm: "UNLOCKED"})
    log({"event": "adopted", "grippers": states})
    say(f"\nBoth arms hold at train_{arm}. The {hold} gripper reads {states[hold]}, the {arm} gripper is open.")
    sm = r.arms[hold].sm
    if sm.state != GripperState.UNLOCKED:
        if ask(f"Is the tube NOT attached to anything (not in either gripper)? Type YES to open the EMPTY "
               f"{hold} gripper (slow): ").strip() != "YES":
            raise Stop("tube state not confirmed; no gripper moved")
        log({"event": "owner_confirmed", "what": "tube not attached"})
        sm.reset()                               # AA-17 state machine, empty gripper
        r._wait_gripper_open(hold)
        log({"event": "gripper_open", "arm": hold})
    if ask(f"Hold the tube and put its attachment key in the open {hold} gripper (the {arm} gripper stays "
           f"empty, under its point). Type YES to LOCK the {hold} gripper: ").strip() != "YES":
        raise Stop("tube not confirmed in the gripper; nothing locked")
    if r.arms[arm].sm.state != GripperState.UNLOCKED:
        raise Stop(f"the {arm} gripper is not open")
    say(f"locking the {hold} gripper (45-degree turn) ...")
    r._lock(hold)
    if ask(f"LOOK at the {hold} gripper: is the key fully IN and LOCKED? y/n: ").strip().lower() != "y":
        raise Stop(f"owner: the {hold} key is not in and locked")
    log({"event": "owner_observation", "arm": hold, "key_in_and_locked": "y"})
    probs = pose_problems(r, pos)
    if probs:
        raise Stop("the arms moved while the tube went in: " + "; ".join(probs))


def write_state(cfg, r, pos: dict, path: Path, timeline: Path = TIMELINE) -> dict:
    from inchworm import cycle_replay as cr, handoff as ho
    from inchworm.so101_pose_reader import ticks_to_q
    hold, arm = pos["holding_arm"], pos["inserting_arm"]
    return ho.write_state(path, stop_frame=pos["timeline_frame"], step=f"training position train_{arm}",
                          timeline_digest=_digest(timeline), tube_in_grippers=True,
                          goal_ticks=pos["joint_ticks"], present_ticks={a: r._ticks(a) for a in cr.ARMS},
                          gripper_ticks={a: round(r.arms[a].bus.read_position_deg(6) * 4096 / 360) for a in cr.ARMS},
                          locked={arm: False, hold: True},
                          q_rad={a: ticks_to_q(cfg, a, pos["joint_ticks"][a]) for a in cr.ARMS},
                          tube_pose=pos["tube_pose"])


def sim_buses(cfg, state: dict):
    """Both arms holding at the (simulated) replay stop state: joints and grippers torque on."""
    import replay_sim
    from inchworm import cycle_replay as cr
    clock = replay_sim.SimClock()
    buses = {}
    for a in cr.ARMS:
        lim = getattr(cfg.motion_guard, f"{a}_arm_limits").servo_tick_limits
        b = replay_sim.SimArmBus(f"arm_{a}", list(state["present_ticks"][a]) + [state["gripper_ticks"][a]], clock,
                                 limits={i + 1: tuple(lim[j]) for i, j in enumerate((*cr.JOINTS, "gripper"))})
        for s in range(1, 7):
            b.set_torque_enable(s, True)
        buses[a] = b
    return buses, clock


def main(argv=None, ask=input) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), default=None, help="the INSERTING arm (required)")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="(default) simulated servos from the sim state")
    ap.add_argument("--state", type=Path, default=None, help="handoff state to write (default out/handoff_state.json)")
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-21 - AA-54 training recording")
    a = ap.parse_args(argv)
    if a.arm is None:
        if a.live:
            print("REFUSED: --live needs --arm left|right"); return 2
        print("DRY-RUN: no --arm given; nothing opened or moved (see --help)"); return 0
    from inchworm import cycle_replay as cr, handoff as ho, training_poses as tp
    from inchworm.config import load_config
    cfg = load_config()
    drift = tp.check_against_timeline(cfg, TIMELINE)
    if drift:
        print("REFUSED: the frozen training positions no longer match the timeline + calibration:\n  "
              + "\n  ".join(drift)); return 2
    pos = tp.for_arm(a.arm)
    path = a.state or (STATE if a.live else STATE.with_name("sim_" + STATE.name))
    replay_state = ho.load_state(path) if path.exists() else None
    if replay_state is None or replay_state["stop_frame"] != pos["timeline_frame"]:
        print(f"REFUSED: {path} is not the replay stop at frame {pos['timeline_frame']} "
              f"(run scripts/replay_cycle.py --stop-after-frame {pos['timeline_frame']} first)"); return 2
    if replay_state.get("tube_in_grippers"):
        print(f"REFUSED: {path} already has the tube in the grippers"); return 2
    frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    logf = (ROOT / "calibration_data" if a.live else ROOT / "out") / f"place_training_tube_{time.strftime('%Y-%m-%d')}.jsonl"
    fh = open(logf, "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "arm": a.arm, "live": a.live, **rec}
        fh.write(json.dumps(rec, default=str) + "\n"); fh.flush()

    STOP_FILE.unlink(missing_ok=True)
    close = lambda: None  # noqa: E731
    if not a.live and ask is input:
        def ask(m):                              # dry run: scripted owner answers
            ans = "y" if "y/n" in m else "YES"
            print(m + ans + " (dry run)")
            return ans
    try:
        if a.live:
            buses, close = _open_live(cfg)
            kw = {}
        else:
            buses, clock = sim_buses(cfg, replay_state)
            kw = {"clock": clock, "sleep": clock.sleep}
        r = cr.build_runner(cfg, [frames[pos["timeline_frame"]]], buses, live=a.live,
                            soft_limits_override=a.allow_unverified_soft_limits,
                            should_abort=STOP_FILE.exists, log=log, abort_on_faults=True, **kw)
        try:
            place(cfg, r, a.arm, pos, ask, log=log)
            st = write_state(cfg, r, pos, path)
            log({"event": "state_written", "path": str(path), "present_ticks": st["present_ticks"]})
            print(f"\nTUBE IN: the {pos['holding_arm']} gripper LOCKED, both arms HOLDING (they keep holding after "
                  f"this exits).\nState: {path}")
            return 0
        except (Stop, cr.ReplayAborted, KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
            what = "Ctrl-C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
            print(f"\nSTOPPED: {what}\nHolding both arms in place (torque on); the grippers stay as they are.")
            log({"event": "stopped", "reason": what, "hold": {k: str(v) for k, v in r.hold().items()}})
            print("To end: support the arms (and the tube if it is in), then\n"
                  "  uv run python scripts/hold_tube.py release --arm left --live  (and --arm right)")
            return 1
    finally:
        close()
        fh.close()


def _open_live(cfg):
    """Both arm ports; refuses unless joints 1-5 of both arms are torque ON (holding after the replay)."""
    from inchworm.backends import live_backend
    from inchworm.servo_bus import ArmServoBus
    from inchworm.servo_checks import drive_config_faults
    from inchworm import cycle_replay as cr
    be = live_backend(cfg)
    ts, buses = [], {}
    try:
        for arm in cr.ARMS:
            t = be.open_serial(f"arm_{arm}")
            ts.append(t)
            buses[arm] = ArmServoBus(t, role=f"arm_{arm}", joint_id_map={n: i + 1 for i, n in enumerate((*cr.JOINTS, "gripper"))})
        for arm, b in buses.items():
            off = [s for s in range(1, 6) if not b._bus.read_bytes(s, 0x28, 1)[0]]
            if off:
                raise Stop(f"arm_{arm} joints {off} are torque OFF (the replay must leave both arms holding)")
            bad = [f"servo {s}: {f}" for s in range(1, 7) for f in drive_config_faults(b._bus.read_bytes, s)
                   if not f.startswith("torque_limit")]
            if bad:
                raise Stop(f"arm_{arm} drive config: {bad}")
    except BaseException:
        for t in ts:
            t.close()
        raise
    return buses, lambda: [t.close() for t in ts]


if __name__ == "__main__":
    sys.exit(main())
