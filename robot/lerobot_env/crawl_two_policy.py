"""AA-55 two-policy crawl sequence (owner plan 2026-11-21).

Starts AFTER ``scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers`` (step 1; the
launcher ``crawl_two_policy.sh`` runs both). Phases = ``inchworm.two_policy.PHASES``:

2 left_policy           LEFT checkpoint (inchworm_follower, tube_support=handoff), only if both arms
                        read within POSE_TOL_TICKS of the frozen ``train_left`` ticks. Owner l/r/q.
3 left_lock             adopt both arms holding, left gripper LOCKS (GripperLockSM) + owner look y/n.
                        Its distance from the finish reference is LOGGED only (owner 2026-11-21: no 8 deg stop).
   Steps 4-6 are planned from the present pose and preflighted (limits + cell mesh with the tube
   carried by the left) BEFORE the right unlocks; then one typed YES per motion phase.
4 right_out             right gripper UNLOCKS (GripperLockSM; refused unless the left reads LOCKED by
                        read-back and the owner said y), straight out 50 mm, then clear (frame 55).
5 left_to_train_right   left (LOCKED, tube) joint line to its frozen train_right ticks, right static.
6 right_to_train_right  right along timeline 55..train_right frame, left static.
7 right_policy          RIGHT checkpoint (no joint-offset map), from a train_right handoff state with
                        the carried tube; only if both arms read within tolerance of train_right.
8 right_lock            right gripper LOCKS + owner look y/n.
9 home                  both arms (both LOCKED) to timeline frame 86 (fade-out plan, else a joint line).
   End: hold, owner supports, joints 1-5 of both arms OFF, grippers untouched (both stay LOCKED).

AA-56 ``--autonomous``: after the start replay's two typed YES nothing is asked until the end.
Each policy LOCKS by itself when finished by the rule or at ``--lock-s`` (25 s) if within 8 deg of
its finish reference (else return + ONE automatic retry, then stop + hold); the lock is read back.
OWNER 2026-11-24 02:05Z: the order is the ORIGINAL AA-55 one (``inchworm.two_policy.AUTO_PHASES``):
steps 4-6 planned from the actual lock pose and mesh-checked with the carried tube before the
right unlocks (left reads LOCKED), streamed without YES; then the right policy, lock, home. The
end holds and asks Enter before joints off.
``--autonomous --regrip-first`` = the 2026-11-23 order the owner reversed (left to its regrip pose
with BOTH grippers LOCKED, no cell-mesh check, refused > 8 deg off its finish reference, before
the right comes out; ``REGRIP_FIRST_PHASES``). Only used if asked for explicitly.

Any stop (q, Ctrl-C, ``touch robot/out/STOP_TELEOP`` or /tmp/inchworm_stop_move, an error,
a refused check) holds both arms, then the usual release prompt. Grippers only via GripperLockSM.

    bash ~/893/robot/lerobot_env/crawl_two_policy.sh --dry-run    # no hardware
    bash ~/893/robot/lerobot_env/crawl_two_policy.sh              # live, owner supervising
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT = HERE.parent
sys.path.insert(0, str(HERE))
import full_policy_test as fpt  # noqa: E402  (UI, Stop, policy_phase, lock_and_look, hold_and_release, ...)
import policy_handoff as ph  # noqa: E402

TIMELINE = fpt.TIMELINE
OUT_DIR = ROBOT / "out" / "rollout"
STATE_TRAIN_RIGHT = ROBOT / "out" / "crawl_state_train_right.json"
RUNNER_STOP = Path("/tmp/inchworm_stop_move")
DEG_PER_TICK = 360.0 / 4096
J5 = fpt.J5
LEFT_DATASET = ROBOT / "datasets/insert_left_v3"
RIGHT_DATASET = ROBOT / "datasets/insert_right_v3"
#: The policies of the first successful live crawl (2026-11-23 14:04Z, AA-55): 30 frozen-pose demos
#: per arm recorded from r incl. the catch-up, ACT 16000 steps, brightness/contrast augmentation.
LEFT_CKPT = ROBOT / "out/train/insert_left_v5nom/checkpoints/016000/pretrained_model"
RIGHT_CKPT = ROBOT / "out/train/insert_right_v5nom/checkpoints/016000/pretrained_model"
#: v3 (2026-11-22, owner: night + daylight sets, brightness/contrast augmentation). Finish references
#: = median last-frame observation.state of each v3 dataset's episodes (left 92, right 80), ticks;
#: test_crawl_two_policy recomputes them. v2 values were left (1982, 2203, 1919, 1388, 1986),
#: right (2062, 2482, 1646, 1395, 2007).
LEFT_FINISH_REF_TICKS = (1977, 2194, 1952, 1380, 1988)
RIGHT_FINISH_REF_TICKS = (2066, 2470, 1671, 1382, 2002)

SIM: dict = {}                               # dry run: the simulated buses (tests inspect them)
Stop = fpt.Stop


def demo_end_median_ticks(dataset: Path) -> list:
    """Median of each episode's last observation.state (raw servo deg) -> ticks."""
    import pyarrow.parquet as pq
    rows = {}
    for f in sorted((Path(dataset) / "data").glob("chunk-*/file-*.parquet")):
        t = pq.read_table(f, columns=["observation.state", "episode_index", "frame_index"]).to_pydict()
        for s, e, i in zip(t["observation.state"], t["episode_index"], t["frame_index"]):
            if e not in rows or i > rows[e][0]:
                rows[e] = (i, s)
    med = np.median(np.array([s for _, s in rows.values()], dtype=float), axis=0)
    return [int(round(v / DEG_PER_TICK)) for v in med]


def read_ticks(buses) -> dict:
    return {a: [round(buses[a].read_position_deg(i) * 4096 / 360) for i in range(1, 6)] for a in ("left", "right")}


def load_policy(checkpoint: Path, device: str, follower: str):
    """(policy, sha256) of ``follower``'s checkpoint, warmed up. Tests replace it."""
    from inchworm.policy_contract import Observation
    from inchworm.policy_runner import CheckpointLoader, CheckpointPolicy, LeRobotBackend
    meta = CheckpointLoader(checkpoint).load()                 # validates: 5 joints, no gripper
    base = CheckpointPolicy(meta, LeRobotBackend(device=device, follower=follower))
    base.step(Observation(frame=np.zeros((480, 640), np.uint8), joint_positions_deg=np.full(5, 180.0, np.float32),
                          step_index=0))
    base.reset()
    return base, meta.sha256


def locked_readback(r, arm: str, what: str) -> float:
    """The ``arm`` gripper must read inside its LOCKED band AND its state machine must be LOCKED."""
    from inchworm.gripper_lock import GripperState
    band = getattr(r.cfg.gripper_lock, f"arm_{arm}")
    deg = r.arms[arm].bus.read_position_deg(6)
    if not band.locked_low_deg <= deg <= band.locked_high_deg or r.arms[arm].sm.state != GripperState.LOCKED:
        raise Stop(f"the {arm} gripper reads {deg:.1f} deg / {r.arms[arm].sm.state.name} (locked band "
                   f"{band.locked_low_deg}..{band.locked_high_deg}); {what}")
    return deg


def unlock_right(r, look: str, log) -> None:
    """AC #3: the right unlocks only if the owner said y (AA-56 ``--autonomous``: "auto", no owner
    look) AND the left gripper reads inside its LOCKED band (read-back) AND the state machine
    agrees (GripperLockSM refuses otherwise)."""
    if look not in ("y", "auto"):
        raise Stop("the owner did not confirm the left key is in and locked; the right does not unlock")
    deg = locked_readback(r, "left", "the right does not unlock")
    log({"event": "left_lock_readback", "deg": round(deg, 2), "look": look})
    r._unlock("right")


def auto_policy_phase(robot, policy, ui, trace, kw, log, retries: int = 1) -> None:
    """AA-56: no owner choice. Lock (return) when finished by the rule, or at the lock timer
    (kw max_s) if within FINISH_NEAR_DEG of the finish reference; otherwise return + retry up to
    ``retries`` times, then Stop (the caller holds). Jump guard / owner stop key = Stop."""
    robot.connect()                                          # config confirm=False: no typed YES
    ui.start_keys()
    arm = robot.config.follower
    try:
        for n in range(retries + 1):
            print(f"{arm.upper()} ATTEMPT {n} (autonomous; s = stop + hold)")
            trace.write(json.dumps({"arm": arm, "attempt": n}) + "\n")
            res = ph.PolicyAttempt(robot, policy, keys=ui.keys, trace=trace, **kw).run()
            far = float(np.abs(np.asarray(res["present_deg"]) - kw["finish_ref_deg"]).max())
            if res["end"] == "finished" or (res["end"] == "time_cap" and far <= ph.FINISH_NEAR_DEG):
                decision = "lock"
            elif res["end"] in ("jump_guard", "owner_stop"):
                decision = "stop"
            else:
                decision = "retry" if n < retries else "stop"
            log({"event": "auto_choice", "arm": arm, "attempt": n, "end": res["end"], "s": res["s"],
                 "deg_from_finish_ref": round(far, 2), "decision": decision})
            if decision == "lock":
                return
            if decision == "stop":
                raise Stop(f"{arm} policy: {res['end']}, {far:.1f} deg from the finish reference after "
                           f"{n + 1} attempt(s); holding, nothing locks")
            print(f"RETURN (automatic): {far:.1f} deg from the finish reference at {res['s']} s")
            robot.handoff_return(confirm=False)
    finally:
        ui.stop_keys()


def auto_lock(r, arm: str, log) -> None:
    """AA-56: adopt both arms holding (this arm open, the other LOCKED), lock through GripperLockSM,
    read it back (no owner look)."""
    other = "right" if arm == "left" else "left"
    r.adopt_holding(require={arm: "UNLOCKED", other: "LOCKED"})
    print(f"locking the {arm} gripper (45-degree turn) ...")
    r._lock(arm)
    deg = locked_readback(r, arm, "stopping (holding)")
    log({"event": "auto_lock_readback", "arm": arm, "deg": round(deg, 2)})


def stream(r, frames, segs, ui, log, name: str, what: str, confirm: bool = True) -> None:
    """One preflighted motion phase: typed YES (not with ``confirm`` False, AA-56 autonomous),
    then the fluid stream (as replay_cycle)."""
    print(f"\nNEXT {name}: {what} ({sum(s.duration_s for s in segs):.1f} s). Ctrl-C = hold.")
    if confirm and ui.text("Type YES to move: ").strip() != "YES":
        raise Stop(f"{name} not confirmed")
    log({"event": "phase", "name": name, "status": "start", "frames": [frames[0].index, frames[-1].index]})
    r.frames = frames
    r.abort_on_faults = False                       # owner 2026-11-13: confirmed motion, readings logged
    r.run_fluid(segs)
    log({"event": "phase", "name": name, "status": "done", "ticks": {a: r._ticks(a) for a in ("left", "right")}})


def log_lock_offset(arm: str, ticks, ref_ticks, log) -> None:
    """OWNER 2026-11-21: the 8 deg lock-pose check is dropped (it guarded the AA-39 flow, where the
    left moved with the tube between two locked grippers). The distance is logged only; every
    following move is planned from the actual lock pose and mesh-checked with the tube."""
    d = np.abs(np.asarray(ticks, float) - np.asarray(ref_ticks, float)) * DEG_PER_TICK
    log({"event": "lock_offset_from_finish_ref", "arm": arm, "max_deg": round(float(d.max()), 2),
         "joint": J5[int(d.argmax())], "ticks": list(ticks)})


def checked(cfg, checker, frames, label: str):
    from inchworm import two_policy as tw
    probs = tw.frames_problems(cfg, checker, frames)
    segs, sp = tw.fluid_problems(cfg, checker, frames) if not probs else ([], [])
    if probs or sp:
        raise Stop(f"{label} fails its preflight: " + "; ".join((probs + sp)[:5]))
    return segs


def main(argv=None, ui=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--handoff-state", type=Path, required=True, help="frame-42 state from replay_cycle")
    ap.add_argument("--left-checkpoint", type=Path, default=LEFT_CKPT)
    ap.add_argument("--right-checkpoint", type=Path, default=RIGHT_CKPT)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--max-deg-s", type=float, default=5.0)
    ap.add_argument("--max-jump-deg", type=float, default=10.0)
    ap.add_argument("--max-s", type=float, default=20.0)
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-21 - AA-55 two-policy crawl")
    ap.add_argument("--dry-run", action="store_true", help="simulated servos, real checkpoints + cell mesh")
    ap.add_argument("--replay-episode", type=int, default=0, help="dry run: camera frames from this episode")
    ap.add_argument("--preview-port", type=int, default=8765, help="browser live view of the policy's frames (0 = off)")
    ap.add_argument("--autonomous", action="store_true",
                    help="AA-56: no prompts until the end, timed locks; AA-55 order (right out first)")
    ap.add_argument("--regrip-first", action="store_true",
                    help="with --autonomous only: the REVERSED 2026-11-23 order (left to the regrip pose with both "
                         "LOCKED, no mesh check, before right out). Off unless asked for")
    ap.add_argument("--lock-s", type=float, default=25.0, help="--autonomous: lock timer per policy attempt (s)")
    a = ap.parse_args(argv)
    if a.regrip_first and not a.autonomous:
        print("REFUSED: --regrip-first needs --autonomous"); return 2
    if a.autonomous and not 5.0 <= a.lock_s <= 60.0:
        print("REFUSED: --lock-s 5..60"); return 2
    if not 0.5 <= a.max_deg_s <= 10.0 or not 0 < a.max_jump_deg <= 20.0:
        print("REFUSED: --max-deg-s 0.5..10, --max-jump-deg 0..20"); return 2
    logging.basicConfig(level=logging.WARNING)
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from inchworm import cycle_replay as cr
    from inchworm import handoff as ho
    from inchworm import training_poses as tp
    from inchworm import two_policy as tw
    from inchworm.config import load_config
    from inchworm.kinematics import build_chain, with_soft_limits
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.so101_pose_reader import ticks_to_q
    from lerobot_robot_inchworm import InchwormFollower, InchwormFollowerConfig

    cfg = load_config()
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    logf = open(OUT_DIR / f"crawl_two_policy_{'dry_' if a.dry_run else ''}{utc}.jsonl", "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": "crawl_two_policy", **rec}
        logf.write(json.dumps(rec, default=str) + "\n"); logf.flush()
        if rec.get("event") not in ("stream", "frame"):
            print(json.dumps(rec, default=str), flush=True)

    # --- everything checked before anything moves ----------------------------------------
    st42 = ho.load_state(a.handoff_state)
    f42 = tp.position("train_left")["timeline_frame"]
    if st42["stop_frame"] != f42 or st42["locked"] != {"left": False, "right": True} or not st42["tube_in_grippers"]:
        print(f"REFUSED: {a.handoff_state}: want frame {f42}, left open, right LOCKED, tube in (got frame "
              f"{st42['stop_frame']}, {st42['locked']}, tube {st42['tube_in_grippers']})"); logf.close(); return 2
    if st42["timeline_sha256"] != fpt._digest(TIMELINE):
        print("REFUSED: the handoff state was written from a different timeline"); logf.close(); return 2
    drift = tp.check_against_timeline(cfg, TIMELINE)
    if drift:
        print("REFUSED: frozen training positions drifted: " + "; ".join(drift)); logf.close(); return 2
    for p in (a.left_checkpoint, a.right_checkpoint):
        if not Path(p).exists():
            print(f"REFUSED: no checkpoint {p}"); logf.close(); return 2
    print("loading the cell model ...")
    checker = build_mesh_collision_guard(cfg, "left").checker
    chain_r = with_soft_limits(build_chain(cfg.so101_kinematics), cfg.motion_guard.right_arm_limits)
    timeline, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    refs = {"left": np.array(LEFT_FINISH_REF_TICKS), "right": np.array(RIGHT_FINISH_REF_TICKS)}
    policies = {}
    for arm, ck in (("left", a.left_checkpoint), ("right", a.right_checkpoint)):
        policies[arm] = load_policy(ck, a.device, arm)
        print(f"{arm} checkpoint {ck}  sha256 {policies[arm][1][:16]}")
    kw = dict(fps=a.fps, max_deg_s=a.max_deg_s, max_jump_deg=a.max_jump_deg,
              max_s=a.lock_s if a.autonomous else a.max_s)
    log({"event": "crawl_start", "dry_run": a.dry_run, "autonomous": a.autonomous,
         "phases": list((tw.REGRIP_FIRST_PHASE_NAMES if a.regrip_first else tw.AUTO_PHASE_NAMES)
                       if a.autonomous else tw.PHASE_NAMES), "regrip_first": a.regrip_first,
         "checkpoints": {k: str(c) for k, c in (("left", a.left_checkpoint), ("right", a.right_checkpoint))},
         "sha256": {k: v[1] for k, v in policies.items()}, "handoff_state": str(a.handoff_state),
         "state_utc": st42["utc"], "finish_ref_ticks": {k: v.tolist() for k, v in refs.items()}, **kw})

    ui = ui or fpt.UI(a.dry_run)
    st_tr = STATE_TRAIN_RIGHT.with_name("sim_" + STATE_TRAIN_RIGHT.name) if a.dry_run else STATE_TRAIN_RIGHT
    st_tr.unlink(missing_ok=True)                      # never start the right phase from an old file
    sim_buses = None
    if a.dry_run:
        sim_buses = ph.sim_buses(cfg, st42)
        SIM["buses"] = sim_buses

    def make_robot(arm, state_path):
        r = InchwormFollower(InchwormFollowerConfig(
            id=arm, follower=arm, camera=True, tube_support="handoff", catchup_deg_s=a.max_deg_s,
            confirm=not a.autonomous,
            handoff_state=str(state_path), allow_unverified_soft_limits=a.allow_unverified_soft_limits,
            control_hz=float(a.fps), log_dir=str(OUT_DIR) if a.dry_run else ""))
        if a.dry_run:
            st = ho.load_state(state_path)
            imgs = ph._replay_frames(LEFT_DATASET if arm == "left" else RIGHT_DATASET, a.replay_episode,
                                     int(kw["max_s"] * a.fps) + 5)
            r._open_hardware = lambda c: ph._SimHW(cfg, st, imgs, follower=arm, buses=sim_buses)
        r.prompt = ui.text
        current["robot"] = r
        return r

    def open_buses():
        return (sim_buses, (lambda: None)) if a.dry_run else fpt.open_live_buses(cfg)

    def runner(buses):
        return cr.build_runner(cfg, [], buses, live=not a.dry_run, soft_limits_override=a.allow_unverified_soft_limits,
                               should_abort=lambda: RUNNER_STOP.exists() or fpt.ph_stop_file().exists(), log=log,
                               abort_on_faults=True)

    def pose_gate(name, ticks):
        probs = tw.training_pose_problems(name, ticks)
        log({"event": "training_pose_gate", "pose": name, "ticks": ticks, "problems": probs})
        if probs:
            raise Stop(f"{name}: " + "; ".join(probs) + ". The policy does not start")

    def run_policy(arm, state_path) -> bool:
        print(f"\n=== {arm.upper()} POLICY ({arm} checkpoint) from {state_path.name}")
        robot = make_robot(arm, state_path)
        try:
            if a.autonomous:
                auto_policy_phase(robot, policies[arm][0], ui, trace,
                                  {**kw, "finish_ref_deg": refs[arm] * DEG_PER_TICK}, log)
                robot.detach_holding()
                return True
            lock = fpt.policy_phase(robot, policies[arm][0], ui, trace,
                                    {**kw, "finish_ref_deg": refs[arm] * DEG_PER_TICK}, log, warn_far=False)
        except BaseException:
            ui.stop_keys()
            if robot.is_connected:
                robot.disconnect()                     # hold, release prompt, joints of both arms off
            raise
        if not lock:
            robot.disconnect()
            log({"event": "quit", "where": f"{arm} policy"})
            return False
        robot.detach_holding()                         # no torque cut; ports closed
        return True

    current = {"robot": None}
    preview = None if a.dry_run else ph.start_preview(lambda: current["robot"], a.preview_port)
    RUNNER_STOP.unlink(missing_ok=True)
    trace = open(OUT_DIR / f"policy_crawl_{'dry_' if a.dry_run else ''}{utc}.jsonl", "w")
    rc, r, close = 0, None, None
    try:
        # ---- 2 left policy (gate: frozen train_left) -------------------------------------
        buses, close = open_buses()
        try:
            probs = tw.training_pose_problems("train_left", read_ticks(buses))
        finally:
            close(); close = None
        log({"event": "training_pose_gate", "pose": "train_left", "problems": probs})
        if probs:
            print("REFUSED (nothing moved): " + "; ".join(probs) + "\n" + fpt.RELEASE_HINT)
            return 4
        if not run_policy("left", a.handoff_state):
            return 0
        buses, close = open_buses()
        r = runner(buses)
        if a.autonomous:
            if a.regrip_first:
                auto_crawl(cfg, r, checker, chain_r, timeline, refs, st42, st_tr, log, ui)
            else:
                auto_lock(r, "left", log)
                now_t = {s: r._ticks(s) for s in ("left", "right")}
                log_lock_offset("left", now_t["left"], refs["left"], log)
                original_order(cfg, r, checker, chain_r, timeline, now_t, st42, st_tr, ui, log, look="auto")
            close(); close, r = None, None
            run_policy("right", st_tr)
            buses, close = open_buses()
            r = runner(buses)
            auto_lock(r, "right", log)
            now_t = {s: r._ticks(s) for s in ("left", "right")}
            log_lock_offset("right", now_t["right"], refs["right"], log)
            home(cfg, r, checker, timeline, now_t, st_tr, ui, log, confirm=False)
            return 0
        # ---- 3 left lock + look -------------------------------------------------------------
        r.adopt_holding(require={"left": "UNLOCKED", "right": "LOCKED"})
        print("locking the left gripper (45-degree turn) ...")
        r._lock("left")
        look = ui.choose("LOOK at the left gripper: is the key fully IN and LOCKED? y = yes, n = no", "yn", "y",
                         "left_look")
        log({"event": "owner_observation", "arm": "left", "key_in_and_locked": look})
        if look != "y":
            raise Stop("owner: the left key is not in and locked. Nothing unlocks")
        now_t = {s: r._ticks(s) for s in ("left", "right")}
        log_lock_offset("left", now_t["left"], refs["left"], log)     # owner 2026-11-21: logged, no 8 deg stop
        tube = original_order(cfg, r, checker, chain_r, timeline, now_t, st42, st_tr, ui, log, look=look)
        close(); close, r = None, None
        if not run_policy("right", st_tr):
            return 0
        buses, close = open_buses()
        r = runner(buses)
        # ---- 8 right lock + look ------------------------------------------------------------
        fpt.lock_and_look(r, "right", ui, log)
        now_t = {s: r._ticks(s) for s in ("left", "right")}
        log_lock_offset("right", now_t["right"], refs["right"], log)     # owner 2026-11-21: logged, no 8 deg stop
        # ---- 9 home ---------------------------------------------------------------------------
        home(cfg, r, checker, timeline, now_t, st_tr, ui, log, tube=tube)
    except KeyboardInterrupt as exc:
        rc = _stopped(r, ui, log, "Ctrl-C", exc)
    except (Stop, Exception) as exc:  # noqa: BLE001
        rc = _stopped(r, ui, log, f"{type(exc).__name__}: {exc}", exc)
    finally:
        if close is not None:
            close()                                    # never a torque-off here (holding)
        if preview is not None:
            preview.stop()
        trace.close()
        logf.close()
        print(f"log: {logf.name}\ntrace: {trace.name}")
    return rc


def home(cfg, r, checker, timeline, now_t, st_tr, ui, log, confirm: bool = True, tube=None) -> None:
    """Step 9: both arms (both LOCKED) home, then hold + the Enter before joints off. ``tube``
    None = the tube carried by the left from the train_right handoff state (autonomous)."""
    from inchworm import handoff as ho
    from inchworm import two_policy as tw
    from inchworm.so101_pose_reader import ticks_to_q
    now = {s: np.asarray(ticks_to_q(cfg, s, now_t[s])) for s in ("left", "right")}
    if tube is None:
        st = ho.load_state(st_tr)
        tube = tw.CarriedTube(checker, "left", {s: np.asarray(ticks_to_q(cfg, s, st["present_ticks"][s]))
                                                for s in ("left", "right")}, np.asarray(st["tube_pose"]))
    try:
        plan9 = tw.plan_home_timeline(timeline, now, tube)
        segs9 = checked(cfg, checker, plan9, "home (timeline)")
    except (Stop, tw.PlanRefused) as exc:
        log({"event": "home_fallback", "reason": str(exc)})
        plan9 = tw.plan_home_line(timeline, now, tube)
        segs9 = checked(cfg, checker, plan9, "home (joint line)")
    stream(r, plan9, segs9, ui, log, "home", f"both arms (both LOCKED) home: {plan9[0].step}", confirm=confirm)
    log({"event": "crawl_done"})
    print("\nCRAWL DONE: both grippers LOCKED, both arms at home, holding.")
    fpt.hold_and_release(r, ui, log, "end of the crawl", done=True)


def original_order(cfg, r, checker, chain_r, timeline, now_t, st42, st_tr, ui, log, look: str):
    """AA-55 steps 4-7 (the left already LOCKED at ``now_t``): plan + preflight right_out,
    left_to_train_right and right_to_train_right from the actual pose (limits + cell mesh, tube
    carried by the left) BEFORE the right unlocks; unlock (left read back); stream the three;
    train_right gate; handoff state. ``look`` "y" = supervised (a typed YES per motion phase),
    "auto" = AA-56 --autonomous (no prompt). Returns the carried-tube model."""
    from inchworm import handoff as ho
    from inchworm import training_poses as tp
    from inchworm import two_policy as tw
    from inchworm.so101_pose_reader import ticks_to_q
    confirm = look != "auto"
    now = {s: np.asarray(ticks_to_q(cfg, s, now_t[s])) for s in ("left", "right")}
    tube = tw.CarriedTube(checker, "left", now, np.asarray(st42["tube_pose"]))
    plan4 = tw.plan_right_out(chain_r, timeline, now, tube)
    plan5 = tw.plan_left_to_train_right(cfg, plan4[-1].q, tube)
    plan6 = tw.plan_right_to_train_right(timeline, plan5[-1].q["left"], tube)
    segs4 = checked(cfg, checker, plan4, "right_out")
    segs5 = checked(cfg, checker, plan5, "left_to_train_right")
    segs6 = checked(cfg, checker, plan6, "right_to_train_right")
    mm, deg = tw.tube_offset(plan6[-1].tube, tp.position("train_right")["tube_pose"])
    log({"event": "plans_ok", "frames": [len(plan4), len(plan5), len(plan6)],
         "carried_tube_vs_frozen_train_right": {"mm": mm, "deg": deg}})
    unlock_right(r, look, log)
    stream(r, plan4, segs4, ui, log, "right_out", "right straight out of its key 50 mm, then clear", confirm=confirm)
    stream(r, plan5, segs5, ui, log, "left_to_train_right",
           f"left (LOCKED, tube) to its train_right pose (tube {mm} mm / {deg} deg off the frozen one)",
           confirm=confirm)
    stream(r, plan6, segs6, ui, log, "right_to_train_right", "right to its train_right start pose", confirm=confirm)
    ticks = read_ticks({s: r.arms[s].bus for s in ("left", "right")})
    probs = tw.training_pose_problems("train_right", ticks)
    log({"event": "training_pose_gate", "pose": "train_right", "ticks": ticks, "problems": probs})
    if probs:
        raise Stop("train_right: " + "; ".join(probs) + ". The policy does not start")
    goal = tp.joint_ticks("train_right")
    q = {s: np.asarray(ticks_to_q(cfg, s, ticks[s])) for s in ("left", "right")}
    ho.write_state(st_tr, stop_frame=tp.position("train_right")["timeline_frame"],
                   step="train_right (AA-56 autonomous)" if not confirm else "train_right (AA-55)",
                   timeline_digest=fpt._digest(TIMELINE), tube_in_grippers=True, goal_ticks=goal, present_ticks=ticks,
                   gripper_ticks={s: round(r.arms[s].bus.read_position_deg(6) * 4096 / 360) for s in ("left", "right")},
                   locked={"left": True, "right": False},
                   q_rad={s: ticks_to_q(cfg, s, goal[s]) for s in ("left", "right")},
                   tube_pose=tube(q["left"], q["right"]))
    return tube


def auto_crawl(cfg, r, checker, chain_r, timeline, refs, st42, st_tr, log, ui) -> None:
    """``--autonomous --regrip-first`` only (owner reversed this order 2026-11-24): left lock, left to the regrip pose (both LOCKED, no mesh
    check), right unlock + out from where it is, right to its train_right start, then the
    train_right gate and handoff state. Raises Stop (the caller holds) on any refusal."""
    from inchworm import handoff as ho
    from inchworm import training_poses as tp
    from inchworm import two_policy as tw
    from inchworm.so101_pose_reader import ticks_to_q
    q_of = lambda t: {s: np.asarray(ticks_to_q(cfg, s, t[s])) for s in ("left", "right")}  # noqa: E731
    # ---- 3 left lock (read back) + 8 deg guard for the both-LOCKED move ----------------------
    auto_lock(r, "left", log)
    now_t = {s: r._ticks(s) for s in ("left", "right")}
    log_lock_offset("left", now_t["left"], refs["left"], log)
    p = tw.lock_pose_problem(now_t["left"], refs["left"])
    if p:
        raise Stop(p)
    now = q_of(now_t)
    tube = tw.CarriedTube(checker, "left", now, np.asarray(st42["tube_pose"]))
    # ---- 4 left to the regrip pose: both LOCKED, limits + speed only (owner: no collision check)
    plan_f = tw.plan_left_to_train_right(cfg, now, tube, right_locked=True)
    segs_f, probs = tw.forced_line_problems(cfg, plan_f)
    if probs:
        raise Stop("left_to_regrip fails its limits check: " + "; ".join(probs[:5]))
    locked_readback(r, "left", "the left does not move")
    locked_readback(r, "right", "the left does not move")
    log({"event": "forced_move", "name": "left_to_regrip", "mesh_check": "skipped (owner 2026-11-23, AA-56)",
         "frames": len(plan_f)})
    stream(r, plan_f, segs_f, ui, log, "left_to_regrip",
           "left (LOCKED) to its train_right pose, right LOCKED: no collision check (owner)", confirm=False)
    # ---- 5 / 6 planned from where the arms actually are now ------------------------------------
    now_t = {s: r._ticks(s) for s in ("left", "right")}
    now = q_of(now_t)
    plan_o = tw.plan_right_out(chain_r, timeline, now, tube)
    plan_a = tw.plan_right_to_train_right(timeline, plan_o[-1].q["left"], tube)
    segs_o = checked(cfg, checker, plan_o, "right_out")
    segs_a = checked(cfg, checker, plan_a, "right_to_train_right")
    mm, deg = tw.tube_offset(plan_a[-1].tube, tp.position("train_right")["tube_pose"])
    log({"event": "plans_ok", "frames": [len(plan_o), len(plan_a)], "after_regrip_ticks": now_t,
         "carried_tube_vs_frozen_train_right": {"mm": mm, "deg": deg}})
    unlock_right(r, "auto", log)
    stream(r, plan_o, segs_o, ui, log, "right_out", "right straight out of its key 50 mm, then clear", confirm=False)
    stream(r, plan_a, segs_a, ui, log, "right_to_train_right", "right to its train_right start pose", confirm=False)
    # ---- 7 gate + handoff state for the right policy ----------------------------------------
    ticks = read_ticks({s: r.arms[s].bus for s in ("left", "right")})
    probs = tw.training_pose_problems("train_right", ticks)
    log({"event": "training_pose_gate", "pose": "train_right", "ticks": ticks, "problems": probs})
    if probs:
        raise Stop("train_right: " + "; ".join(probs) + ". The policy does not start")
    goal = tp.joint_ticks("train_right")
    q = q_of(ticks)
    ho.write_state(st_tr, stop_frame=tp.position("train_right")["timeline_frame"], step="train_right (AA-56)",
                   timeline_digest=fpt._digest(TIMELINE), tube_in_grippers=True, goal_ticks=goal, present_ticks=ticks,
                   gripper_ticks={s: round(r.arms[s].bus.read_position_deg(6) * 4096 / 360) for s in ("left", "right")},
                   locked={"left": True, "right": False},
                   q_rad={s: ticks_to_q(cfg, s, goal[s]) for s in ("left", "right")},
                   tube_pose=tube(q["left"], q["right"]))


def _stopped(r, ui, log, why, exc) -> int:
    if r is not None:
        fpt.hold_and_release(r, ui, log, why)
    else:
        print(f"\nSTOPPED: {why}\n" + fpt.RELEASE_HINT)
        log({"event": "stopped", "reason": why, "hold": "no runner open"})
    return 0 if isinstance(exc, Stop) else 7


if __name__ == "__main__":
    sys.exit(main())
