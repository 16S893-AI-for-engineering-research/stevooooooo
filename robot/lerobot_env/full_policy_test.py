"""AA-39 FULL policy test (owner 2026-11-20): left policy -> lock -> cycle continues -> right policy.

Starts AFTER ``scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers`` (the launcher
``full_policy_test.sh`` runs both). Phases (plan F2-F7 in AA-39):

F2  LEFT policy (inchworm_follower, tube_support=handoff: mesh gate with the tube on the start
    catch-up only (policy steps unchecked, owner 2026-11-24, AA-57), speed cap,
    jump guard, time cap, finish rule). After each attempt the owner chooses (decision 2a):
    l = lock (anyway), r = return to the stop pose + retry, q = quit (release prompt).
F3  The follower DETACHES HOLDING (no torque cut). Both ports reopen in the cycle runner, which
    ADOPTS both arms as they are (grippers by read-back: left must be open, right LOCKED). The
    left gripper locks through the AA-17 state machine. Then the owner looks (decision 1a):
    y = key IN and LOCKED; anything else = stop, nothing unlocks.
F4  One owner YES for F4 + F5. Left joint line (~5 deg/s) from where the policy locked to the
    measured demo-end pose (median of the 29 demos), mesh-checked with the tube first, refused
    if any joint is more than FINISH_NEAR_DEG away.
F5  Timeline frames 49..67 with the left at the demo-end pose (decision 3: the right aims at the
    model's point): right unlock (refused unless the left is LOCKED), right carry to its stop.
    The whole path was preflighted with the tube at startup.
F6  RIGHT policy: the LEFT-trained checkpoint through the per-arm joint offset map, same prompts.
F7  Right lock (AA-17) + owner look. End: both arms hold; support the tube and both arms, Enter:
    joints 1-5 of both arms torque OFF, grippers untouched (both stay LOCKED).

Any stop (q, Ctrl-C, ``touch robot/out/STOP_TELEOP`` or /tmp/inchworm_stop_move, an error) holds
both arms, then the same release prompt. The gripper is commanded only by GripperLockSM.

    bash ~/893/robot/lerobot_env/full_policy_test.sh --dry-run    # no hardware
    bash ~/893/robot/lerobot_env/full_policy_test.sh              # live, owner supervising
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import queue
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT = HERE.parent
sys.path.insert(0, str(HERE))
import policy_handoff as ph  # noqa: E402

TIMELINE = ROBOT / "out" / "inchworm" / "timeline.jsonl"
RESUME_FROM, RESUME_TO = 49, 67              # left relocked .. right stop pose (timeline frames)
STATE_67 = ROBOT / "out" / "full_test_state_67.json"
OUT_DIR = ROBOT / "out" / "rollout"
RUNNER_STOP = Path("/tmp/inchworm_stop_move")
J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DEG_PER_TICK = 360.0 / 4096
POSE_TOL_TICKS = 34                          # = inchworm.handoff.POSE_TOL_TICKS


RELEASE_HINT = ("Both arms may still be HOLDING (torque on). Support the tube and both arms, then:\n"
                "  cd ~/893/robot && uv run python scripts/hold_tube.py release --arm left --live\n"
                "  cd ~/893/robot && uv run python scripts/hold_tube.py release --arm right --live")


SIM: dict = {}                               # dry run: the simulated buses (tests inspect them)


class Stop(RuntimeError):
    """Stop the test: both arms hold, then the release prompt."""


def _digest(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


# ---------------------------------------------------------------------------
# Owner interaction
# ---------------------------------------------------------------------------
class UI:
    """Typed prompts + the g/s keyboard listener during attempts. Dry run: scripted answers."""

    def __init__(self, dry: bool, auto=None):
        self.dry, self.keys = dry, queue.Queue()
        self.auto = auto or {}
        self._listener = None

    def start_keys(self):
        if self.dry:
            return
        from lerobot.utils.keyboard_input import create_key_listener
        self._listener = create_key_listener(lambda k: self.keys.put(k), controls_help="s = stop the attempt")
        if self._listener is None:
            raise Stop("no keyboard listener (run in an interactive terminal)")

    def stop_keys(self):
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    def text(self, msg: str) -> str:
        if self.dry:
            ans = "" if "Enter" in msg else "YES"
            print(msg + f"{ans} (dry run)")
            return ans
        restart = self._listener is not None
        self.stop_keys()
        try:
            return input(msg)
        finally:
            while not self.keys.empty():
                self.keys.get_nowait()
            if restart:
                self.start_keys()

    def choose(self, msg: str, choices: str, auto: str, tag: str = "") -> str:
        if self.dry:
            auto = self.auto.get(tag, auto)              # tests script the owner's answers
            print(f"{msg} [{'/'.join(choices)}]: {auto} (dry run)")
            return auto
        while True:
            a = self.text(f"{msg} [{'/'.join(choices)}]: ").strip().lower()
            if a in choices:
                return a


# ---------------------------------------------------------------------------
# Phase pieces (all hardware injected: real ports or simulated buses)
# ---------------------------------------------------------------------------
def demo_end_q(cfg):
    from inchworm.so101_pose_reader import ticks_to_q
    return np.asarray(ticks_to_q(cfg, "left", list(ph.DEMO_END_LEFT_TICKS)), dtype=float)


def plan_resume(cfg, checker, timeline: Path = TIMELINE):
    """F5 frames + fluid segments with the left at the demo-end pose, fully preflighted with the
    tube (raises Stop on any problem). Returns (frames, segments)."""
    from inchworm import cycle_replay as cr
    frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(timeline), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    sub = cr.resume_frames(frames, RESUME_FROM, RESUME_TO, "left", demo_end_q(cfg))
    probs = cr.preflight(cfg, sub, checker, tube=True)
    segs = cr.build_segments(sub, cr.DEFAULT_TEMPO)
    probs += cr.stream_preflight(cfg, segs, checker, frames=sub)
    if probs:
        raise Stop("resumed cycle 49..67 fails its preflight: " + "; ".join(probs[:5]))
    if sub[-1].locked["right"] or not sub[-1].locked["left"]:
        raise Stop(f"frame {RESUME_TO}: want left LOCKED, right open (got {sub[-1].locked})")
    return sub, segs


def line_problem(cfg, checker, q_from, q_to, right_q, tube, step_deg: float = 1.0):
    """Mesh check of the left joint line q_from -> q_to (right static, tube where it is)."""
    from inchworm import cycle_replay as cr
    a, b = np.asarray(q_from, float), np.asarray(q_to, float)
    n = max(1, int(math.ceil(math.degrees(float(np.abs(b - a).max())) / step_deg)))
    for k in range(n + 1):
        q = a + (b - a) * k / n
        try:
            hits = [c for arm in cr.ARMS for c in checker.check(cr.cell_state(q, right_q, tube), arm=arm)]
        except Exception as exc:  # noqa: BLE001 - fail closed
            return f"collision check failed: {exc}"
        if hits:
            return f"step {k}/{n}: {hits[0]}"
    return None


def policy_phase(robot, policy, ui, trace, kw, log, warn_far: bool = True) -> bool:
    """Attempts until the owner chooses. True = lock here (robot still connected), False = quit.
    ``warn_far``: AA-39 warning that a lock far from the demo end stops the cycle (not in AA-55)."""
    robot.connect()                                              # handoff checks + typed YES
    ui.start_keys()
    arm = robot.config.follower
    n = 0
    while True:
        print(f"{arm.upper()} ATTEMPT {n}: policy driving (s = stop + hold)")
        trace.write(json.dumps({"arm": arm, "attempt": n}) + "\n")
        res = ph.PolicyAttempt(robot, policy, keys=ui.keys, trace=trace, **kw).run()
        print(f"{arm} attempt {n} ended ({res['end']}): {res['steps']} steps, {res['s']} s, holds {res['holds']}, "
              f"overruns {res['overruns']}. Arm HOLDS.")
        n += 1
        if res["end"] == "jump_guard":
            ui.choose("Jump guard (the policy is off its data): q = quit and report", "q", "q", f"{arm}_choice")
            choice = "q"
        elif res["end"] == "finished":
            choice = ui.choose(f"FINISHED by the rule ({res.get('finish_deg_from_ref')} deg from the demo end). "
                               "l = lock, r = return + retry, q = quit", "lrq", "l", f"{arm}_choice")
        else:
            far = float(np.abs(np.asarray(res["present_deg"]) - kw["finish_ref_deg"]).max())
            note = (f" WARNING: {far:.1f} deg from the demo end (> {ph.FINISH_NEAR_DEG:g}): after a lock the "
                    "cycle will NOT continue (stop + release)." if far > ph.FINISH_NEAR_DEG and warn_far else "")
            choice = ui.choose(f"NOT finished ({res['end']}).{note} l = lock anyway, r = return + retry, q = quit",
                               "lrq", "l", f"{arm}_choice")
        log({"event": "owner_choice", "arm": arm, "attempt": n - 1, "end": res["end"], "choice": choice})
        if choice == "l":
            ui.stop_keys()
            return True
        if choice == "q":
            ui.stop_keys()
            return False
        print("RETURN: both arms back to the stop pose")
        robot.handoff_return(confirm=robot.returns_done == 0)


def lock_and_look(r, arm: str, ui, log) -> None:
    """F3/F7: adopt both arms holding (this arm open, the other LOCKED), lock this gripper through
    the AA-17 state machine, then the owner's look. Raises Stop on anything but y."""
    other = "right" if arm == "left" else "left"
    r.adopt_holding(require={arm: "UNLOCKED", other: "LOCKED"})
    print(f"locking the {arm} gripper (45-degree turn) ...")
    r._lock(arm)
    ans = ui.choose(f"LOOK at the {arm} gripper: is the key fully IN and LOCKED? y = yes, n = no", "yn", "y", f"{arm}_look")
    log({"event": "owner_observation", "arm": arm, "key_in_and_locked": ans})
    if ans != "y":
        raise Stop(f"owner: the {arm} key is not in and locked. Nothing unlocks")


def move_left_and_resume(cfg, r, checker, sub, segs, ui, log) -> None:
    """F4 + F5 with one YES. ``r`` = runner over ``sub`` (both arms adopted, both grippers LOCKED)."""
    from inchworm import cycle_replay as cr
    now = r.present_q()
    qd = sub[0].q["left"]
    d_left = np.degrees(np.abs(now["left"] - qd))
    if d_left.max() > ph.FINISH_NEAR_DEG:
        raise Stop(f"the left arm locked {d_left.max():.1f} deg from the demo-end pose (max "
                   f"{ph.FINISH_NEAR_DEG:g}); not moving it with the tube locked")
    want_r = cr.q_to_ticks(cfg, "right", sub[0].q["right"])
    off = [f"{j} {t - w:+d}" for j, t, w in zip(J5, r._ticks("right"), want_r) if abs(t - w) > POSE_TOL_TICKS]
    if off:
        raise Stop(f"the right arm is off its frame-{RESUME_FROM} pose (ticks {', '.join(off)}); the tube moved?")
    p = line_problem(cfg, checker, now["left"], qd, sub[0].q["right"], sub[0].tube)
    if p is not None:
        raise Stop(f"left line to the demo-end pose hits the cell model ({p}); ask the agent (no allowance "
                   "is added without the owner)")
    print(f"\nNEXT (one YES):\n  F4 left (gripper LOCKED) to the demo-end pose: max {d_left.max():.1f} deg, "
          f"~{cr.APPROACH_TICKS_S * DEG_PER_TICK:.0f} deg/s; the tube may be pushed a little\n"
          f"  F5 cycle frames {RESUME_FROM}..{RESUME_TO}: right gripper UNLOCKS (left LOCKED), right moves "
          f"under its next point ({sum(s.duration_s for s in segs):.0f} s)\n  then the right policy")
    if ui.text("Type YES to move: ").strip() != "YES":
        raise Stop("not confirmed")
    log({"event": "left_to_demo_end", "from_deg": np.degrees(now["left"]).round(2).tolist(),
         "max_deg": round(float(d_left.max()), 2)})
    start = cr.Frame(-1, "left at its policy lock", {"left": now["left"], "right": sub[0].q["right"]},
                     dict(sub[0].locked))
    r.abort_on_faults = True                       # stall/fault = hold (the left pushes on the tube)
    r._move(start, sub[0], vmax_ticks_s=cr.APPROACH_TICKS_S)
    r.abort_on_faults = False                      # fluid part: as replay_cycle (logged, never acted on)
    print(f"cycle frames {RESUME_FROM}..{RESUME_TO} ...")
    r.run_fluid(segs)


def write_state_67(cfg, r, sub, path: Path, timeline: Path = TIMELINE) -> dict:
    from inchworm import cycle_replay as cr
    from inchworm import handoff as ho
    f = sub[-1]
    return ho.write_state(path, stop_frame=f.index, step=f.step, timeline_digest=_digest(timeline),
                          tube_in_grippers=True, goal_ticks={a: cr.q_to_ticks(cfg, a, f.q[a]) for a in cr.ARMS},
                          present_ticks={a: r._ticks(a) for a in cr.ARMS},
                          gripper_ticks={a: round(r.arms[a].bus.read_position_deg(6) * 4096 / 360) for a in cr.ARMS},
                          locked=f.locked, q_rad={a: f.q[a] for a in cr.ARMS}, tube_pose=f.tube)


def open_live_buses(cfg):
    """Both arm ports, ArmServoBus; refuses unless joints 1-5 of both arms are torque ON (holding)."""
    from inchworm.backends import live_backend
    from inchworm.servo_bus import ArmServoBus
    from inchworm.servo_checks import drive_config_faults
    be = live_backend(cfg)
    ts, buses = [], {}
    try:
        for arm in ("left", "right"):
            t = be.open_serial(f"arm_{arm}")
            ts.append(t)
            buses[arm] = ArmServoBus(t, role=f"arm_{arm}", joint_id_map={n: i + 1 for i, n in enumerate((*J5, "gripper"))})
        for arm, b in buses.items():
            off = [s for s in range(1, 6) if not b._bus.read_bytes(s, 0x28, 1)[0]]
            if off:
                raise Stop(f"arm_{arm} joints {off} are torque OFF (both arms must still be holding)")
            bad = [f"servo {s}: {f}" for s in range(1, 7) for f in drive_config_faults(b._bus.read_bytes, s)
                   if not f.startswith("torque_limit")]
            if bad:
                raise Stop(f"arm_{arm} drive config: {bad}")
    except BaseException:
        for t in ts:
            t.close()
        raise
    return buses, lambda: [t.close() for t in ts]


def hold_and_release(r, ui, log, why: str, done: bool = False) -> None:
    """Hold both arms, the owner supports them, joints 1-5 of both arms torque OFF, grippers untouched."""
    print(f"\n{'END' if done else 'STOPPED'}: {why}\nHolding both arms in place (torque on).")
    log({"event": "stopped", "reason": why, "hold": {k: str(v) for k, v in r.hold().items()}})
    try:
        ui.text("Support the TUBE and BOTH arms, then press Enter: joints 1-5 of both arms go torque OFF, "
                "the grippers stay as they are: ")
    except (KeyboardInterrupt, EOFError):
        pass
    errs = r.release(grippers=False)
    log({"event": "released", "errors": {k: str(v) for k, v in errs.items() if v}})


# ---------------------------------------------------------------------------
def load_policy(checkpoint: Path, device: str):
    """(policy, sha256) of the left-trained checkpoint, warmed up. Tests replace it."""
    from inchworm.policy_contract import Observation
    from inchworm.policy_runner import CheckpointLoader, CheckpointPolicy, LeRobotBackend
    meta = CheckpointLoader(checkpoint).load()                 # validates: 5 joints, no gripper
    base = CheckpointPolicy(meta, LeRobotBackend(device=device, follower="left"))
    base.step(Observation(frame=np.zeros((480, 640), np.uint8), joint_positions_deg=np.full(5, 180.0, np.float32),
                          step_index=0))
    base.reset()
    return base, meta.sha256


def main(argv=None, ui=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--handoff-state", type=Path, required=True, help="frame-42 state from replay_cycle")
    ap.add_argument("--checkpoint", type=Path, default=ph.DEFAULT_CKPT)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--max-deg-s", type=float, default=5.0)
    ap.add_argument("--max-jump-deg", type=float, default=10.0)
    ap.add_argument("--max-s", type=float, default=20.0)
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-20 - AA-39 full policy test")
    ap.add_argument("--dry-run", action="store_true", help="simulated servos, real checkpoint + cell mesh")
    ap.add_argument("--replay-dataset", type=Path, default=ph.DEFAULT_REPLAY)
    ap.add_argument("--replay-episode", type=int, default=26)
    a = ap.parse_args(argv)
    if not 0.5 <= a.max_deg_s <= 10.0 or not 0 < a.max_jump_deg <= 20.0:
        print("REFUSED: --max-deg-s 0.5..10, --max-jump-deg 0..20"); return 2
    logging.basicConfig(level=logging.WARNING)
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from inchworm import cycle_replay as cr
    from inchworm import handoff as ho
    from inchworm.arm_transfer import JointOffsetMap, MappedPolicy
    from inchworm.config import load_config
    from inchworm.motion_guard import build_mesh_collision_guard
    from lerobot_robot_inchworm import InchwormFollower, InchwormFollowerConfig

    cfg = load_config()
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    tdir = OUT_DIR
    tdir.mkdir(parents=True, exist_ok=True)
    logf = open(tdir / f"full_test_{'dry_' if a.dry_run else ''}{utc}.jsonl", "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": "full_policy_test", **rec}
        logf.write(json.dumps(rec, default=str) + "\n"); logf.flush()
        if rec.get("event") not in ("stream", "frame"):
            print(json.dumps(rec, default=str), flush=True)

    # --- everything checked before anything moves ----------------------------------------
    st42 = ho.load_state(a.handoff_state)
    if st42["stop_frame"] != 42 or st42["locked"] != {"left": False, "right": True} or not st42["tube_in_grippers"]:
        print(f"REFUSED: {a.handoff_state}: want frame 42, left open, right LOCKED, tube in "
              f"(got frame {st42['stop_frame']}, {st42['locked']}, tube {st42['tube_in_grippers']})"); return 2
    if st42["timeline_sha256"] != _digest(TIMELINE):
        print("REFUSED: the handoff state was written from a different timeline"); return 2
    print("loading the cell model and preflighting the resumed cycle 49..67 (left at the demo-end pose) ...")
    checker = build_mesh_collision_guard(cfg, "left").checker
    try:
        sub, segs = plan_resume(cfg, checker)
    except Stop as exc:
        print(f"REFUSED: {exc}"); return 3
    print(f"resumed cycle preflight OK: {len(sub)} frames, {sum(s.duration_s for s in segs):.1f} s of motion")
    amap = JointOffsetMap(cfg, source="left", target="right")
    base, sha = load_policy(a.checkpoint, a.device)
    print(f"checkpoint {a.checkpoint}  sha256 {sha[:16]}")
    ref_left = np.array(ph.DEMO_END_LEFT_TICKS, dtype=np.float64) * DEG_PER_TICK
    phases = {"left": (base, ref_left), "right": (MappedPolicy(base, amap), amap.to_target(ref_left))}
    kw = dict(fps=a.fps, max_deg_s=a.max_deg_s, max_jump_deg=a.max_jump_deg, max_s=a.max_s)
    log({"event": "full_test_start", "dry_run": a.dry_run, "checkpoint": str(a.checkpoint), "sha256": sha,
         "handoff_state": str(a.handoff_state), "state_utc": st42["utc"], "joint_offset_map_deg": amap.summary(),
         "resume_frames": [RESUME_FROM, RESUME_TO], "demo_end_left_ticks": list(ph.DEMO_END_LEFT_TICKS), **kw})

    ui = ui or UI(a.dry_run)
    state67 = STATE_67.with_name("sim_" + STATE_67.name) if a.dry_run else STATE_67
    state67.unlink(missing_ok=True)                    # never start the right phase from an old file
    sim_buses = None
    if a.dry_run:
        sim_buses = ph.sim_buses(cfg, st42)
        SIM["buses"] = sim_buses
        frames_img = ph._replay_frames(a.replay_dataset, a.replay_episode, int(a.max_s * a.fps) + 5)

    def make_robot(arm, state_path):
        r = InchwormFollower(InchwormFollowerConfig(
            id=arm, follower=arm, camera=True, tube_support="handoff", catchup_deg_s=a.max_deg_s,
            policy_no_mesh_check=ph.POLICY_NO_MESH_CHECK,
            handoff_state=str(state_path), allow_unverified_soft_limits=a.allow_unverified_soft_limits,
            control_hz=float(a.fps), log_dir=str(tdir) if a.dry_run else ""))
        if a.dry_run:
            st = ho.load_state(state_path)
            r._open_hardware = lambda c: ph._SimHW(cfg, st, frames_img, follower=arm, buses=sim_buses)
        r.prompt = ui.text
        return r

    def runner(frames):
        if a.dry_run:
            buses, close = sim_buses, (lambda: None)
        else:
            buses, close = open_live_buses(cfg)
        r = cr.build_runner(cfg, frames, buses, live=not a.dry_run,
                            soft_limits_override=a.allow_unverified_soft_limits,
                            should_abort=lambda: RUNNER_STOP.exists() or ph_stop_file().exists(), log=log,
                            abort_on_faults=True)
        return r, close

    RUNNER_STOP.unlink(missing_ok=True)
    trace = open(tdir / f"policy_full_{'dry_' if a.dry_run else ''}{utc}.jsonl", "w")
    rc = 0
    try:
        for arm in ("left", "right"):
            policy, ref = phases[arm]
            state_path = a.handoff_state if arm == "left" else state67
            print(f"\n=== {arm.upper()} POLICY ({'left-trained, joint-offset map' if arm == 'right' else 'left-trained'})"
                  f" from {state_path.name}")
            robot = make_robot(arm, state_path)
            try:
                lock = policy_phase(robot, policy, ui, trace, {**kw, "finish_ref_deg": ref}, log)
            except BaseException:
                ui.stop_keys()
                if robot.is_connected:
                    robot.disconnect()                          # hold, release prompt, joints of both arms off
                raise
            if not lock:
                robot.disconnect()
                log({"event": "quit", "where": f"{arm} policy"})
                return 0
            robot.detach_holding()                              # no torque cut; ports closed
            try:
                r, close = runner(sub if arm == "left" else sub[-1:])
            except BaseException as exc:
                log({"event": "runner_open_failed", "reason": f"{type(exc).__name__}: {exc}"})
                print(f"\nSTOPPED before the {arm} lock: {exc}\n" + RELEASE_HINT)
                return 7
            try:
                lock_and_look(r, arm, ui, log)
                if arm == "left":
                    move_left_and_resume(cfg, r, checker, sub, segs, ui, log)
                    st = write_state_67(cfg, r, sub, state67)
                    log({"event": "resume_stop", "state": str(state67), "present_ticks": st["present_ticks"],
                         "gripper_ticks": st["gripper_ticks"]})
                else:
                    log({"event": "full_test_done"})
                    print("\nFULL TEST DONE: both grippers LOCKED, both arms holding.")
                    hold_and_release(r, ui, log, "end of the test", done=True)
            except (Stop, cr.ReplayAborted, KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
                what = "Ctrl-C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
                hold_and_release(r, ui, log, what)
                rc = 0 if isinstance(exc, Stop) else 7
                return rc
            finally:
                close()                                         # never a torque-off here (holding)
    except KeyboardInterrupt:
        print("\nCtrl-C.")
        rc = 7
    except Exception as exc:  # noqa: BLE001
        print(f"\nSTOPPED: {type(exc).__name__}: {exc}\n" + RELEASE_HINT)
        rc = 1
    finally:
        trace.close()
        logf.close()
        print(f"log: {logf.name}\ntrace: {trace.name}")
    return rc


def ph_stop_file() -> Path:
    import lerobot_robot_inchworm.inchworm_follower as fmod
    return fmod.STOP_FILE


if __name__ == "__main__":
    sys.exit(main())
