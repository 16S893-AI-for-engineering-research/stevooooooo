"""AA-25/AA-26 first physical policy test from the demo-start handoff (owner 2026-11-19).

The trained ACT checkpoint drives the LEFT arm instead of the leader, through the same
``inchworm_follower`` plugin in ``tube_support=handoff`` mode that records the demos, so every
existing safety layer stays in force: handoff checks + typed YES at connect, the mesh
collision check of every goal (tube where the cycle left it; a hit = hold), soft / EEPROM /
hardware.yaml limits, the stop file, joints 1-5 only (the gripper is never in the action
space and is never commanded), and the release prompt at the end.

Added here for the policy (all logged; agent proposals, UNVERIFIED, owner to confirm):
* speed cap ``--max-deg-s`` (default 5 deg/s = the approved catch-up speed) on every joint;
* jump guard: a policy target more than ``--max-jump-deg`` (10) from the present pose on any
  joint stops the attempt and HOLDS (the policy is off its training data);
* time cap ``--max-s`` (20 s; demos were 9-18 s);
* finish rule (owner 2026-11-20, replaces the AA-24 seating detector, which is not needed): the
  attempt ends "finished" when every joint has stayed within ``FINISH_STILL_DEG`` (2 deg) for
  ``FINISH_STILL_S`` (2 s) AND every joint is within ``FINISH_NEAR_DEG`` (8 deg) of the median
  demo-end pose. The arm then HOLDS; locking after the finish is not automated yet (AA-26).
  Not finished by the time cap = a failed attempt.

Start AFTER  scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live ...
(or use the single launcher ``policy_test.sh --arm left|right``, AA-39).

AA-39 (owner 2026-11-20): ``--arm right`` runs the LEFT-trained policy on the RIGHT arm without
retraining (stop frame 67: left LOCKED, right open). Joints are mapped through the per-arm zero
offsets (``inchworm.arm_transfer``; refused if any joint sign differs). The right wrist camera
is used as is.

    bash ~/893/robot/lerobot_env/policy_handoff.sh --dry-run    # no hardware (simulated servos)
    bash ~/893/robot/lerobot_env/policy_handoff.sh              # live, owner supervising

Keys (terminal focused): g = go (one attempt), s = stop the attempt (arms hold),
r = automatic return to the stop pose (mesh-checked; the first one asks YES), q = quit
(release prompt). Ctrl-C or ``touch robot/out/STOP_TELEOP`` = hold, then the release prompt.
"""
from __future__ import annotations

import argparse
import json
import logging
import queue
import sys
import time
from pathlib import Path

import numpy as np

ROBOT = Path(__file__).resolve().parents[1]
DEFAULT_CKPT = ROBOT / "out/train/insert_left_20261119T221003Z/checkpoints/020000/pretrained_model"
DEFAULT_REPLAY = ROBOT / "datasets/insert_left_20261119T221003Z"
#: Median final pose of the 29 episodes of insert_left_20261119T221003Z (left arm, ticks),
#: measured 2026-11-20 (AA-39 notes). The finish rule compares against it.
DEMO_END_LEFT_TICKS = (1977, 2195, 1930, 1396, 1988)
#: Owner 2026-11-20 (agent recommendation from the three live attempts of 2026-11-20 03:01Z).
FINISH_STILL_S = 2.0
FINISH_STILL_DEG = 2.0
FINISH_NEAR_DEG = 8.0


class PolicyAttempt:
    """One policy attempt: observation -> policy -> guards -> robot.send_action. Injected
    robot/policy/clock so the dry-run and tests use the same loop as the live run."""

    def __init__(self, robot, policy, *, fps=30, max_deg_s=5.0, max_jump_deg=10.0, max_s=20.0,
                 keys=None, clock=time.perf_counter, sleep=time.sleep, say=print, trace=None,
                 finish_ref_deg=None, finish_still_s=FINISH_STILL_S, finish_still_deg=FINISH_STILL_DEG,
                 finish_near_deg=FINISH_NEAR_DEG):
        self.robot, self.policy, self.fps = robot, policy, fps
        self.max_deg_s, self.max_jump_deg, self.max_s = max_deg_s, max_jump_deg, max_s
        self.keys = keys or queue.Queue()
        self.clock, self.sleep, self.say, self.trace = clock, sleep, say, trace
        #: follower degrees of the demo-end pose; None = no finish rule (time cap only)
        self.finish_ref = None if finish_ref_deg is None else np.asarray(finish_ref_deg, dtype=np.float64)
        self.finish_still_s, self.finish_still_deg = finish_still_s, finish_still_deg
        self.finish_near_deg = finish_near_deg

    def _finished(self, hist, now, present) -> bool:
        """Owner finish rule: still within finish_still_deg over the last finish_still_s, and near the
        demo-end pose. ``hist`` = [(t, present)] of this attempt."""
        if self.finish_ref is None or now - hist[0][0] < self.finish_still_s:
            return False
        if np.abs(present - self.finish_ref).max() > self.finish_near_deg:
            return False
        w = np.array([p for t, p in hist if t >= now - self.finish_still_s])
        return float((w.max(0) - w.min(0)).max()) <= self.finish_still_deg

    def _key(self):
        try:
            return self.keys.get_nowait().lower()
        except queue.Empty:
            return None

    def run(self) -> dict:
        from inchworm import teleop_live as tl
        from inchworm.policy_contract import JOINT_ORDER, Observation
        assert tuple(JOINT_ORDER) == tuple(tl.J5)
        r = self.robot
        self.policy.reset()
        step_max = self.max_deg_s / self.fps                 # deg per control step
        period = 1.0 / self.fps
        stats = {"steps": 0, "overruns": 0, "rate_clamped_steps": 0, "holds": 0, "infer_ms": [],
                 "max_target_minus_present_deg": 0.0, "end": "time_cap"}
        hist = []
        r._log({"event": "policy_attempt_start", "max_deg_s": self.max_deg_s,
                "max_jump_deg": self.max_jump_deg, "max_s": self.max_s,
                "finish_rule": None if self.finish_ref is None else {
                    "still_s": self.finish_still_s, "still_deg": self.finish_still_deg,
                    "near_deg": self.finish_near_deg, "ref_deg": self.finish_ref.round(2).tolist()},
                "follower_ticks": r.core.follower_ticks()})
        t0 = self.clock()
        while True:
            ts = self.clock()
            if self._key() in ("s", "q"):
                stats["end"] = "owner_stop"
                break
            if ts - t0 >= self.max_s:
                break
            obs = r.get_observation()
            present = np.array([obs[f"{j}.pos"] for j in tl.J5], dtype=np.float64)
            hist.append((ts, present))
            while hist and hist[0][0] < ts - self.finish_still_s - 1.0:
                hist.pop(0)
            if stats["steps"] and ts - t0 >= self.finish_still_s and self._finished(hist, ts, present):
                stats["end"] = "finished"
                stats["finish_deg_from_ref"] = round(float(np.abs(present - self.finish_ref).max()), 2)
                self.say(f"FINISHED: still {self.finish_still_s:g} s, {stats['finish_deg_from_ref']} deg from the "
                         "demo end; holding")
                break
            frame = obs["wrist"][..., 0] if "wrist" in obs else np.zeros((480, 640), np.uint8)
            ti = time.perf_counter()
            act = self.policy.step(Observation(frame=frame, joint_positions_deg=present.astype(np.float32),
                                               step_index=stats["steps"]))
            stats["infer_ms"].append((time.perf_counter() - ti) * 1000)
            if set(act) != set(tl.J5):
                raise RuntimeError(f"policy action keys {sorted(act)} are not joints 1-5")
            target = np.array([act[j] for j in tl.J5], dtype=np.float64)
            # policy output is in the state's turn (the training actions were unwrapped to it)
            d = target - present
            jump = float(np.abs(d).max())
            stats["max_target_minus_present_deg"] = max(stats["max_target_minus_present_deg"], jump)
            if jump > self.max_jump_deg:
                stats["end"] = "jump_guard"
                r._log({"event": "policy_jump_guard", "target_deg": target.round(2).tolist(),
                        "present_deg": present.round(2).tolist(), "max_jump_deg": self.max_jump_deg})
                self.say(f"STOP: policy target {jump:.1f} deg from the arm (> {self.max_jump_deg:g}); holding")
                break
            last = np.array(r.core.last_goal, dtype=np.float64) * tl.DEG_PER_TICK
            # unwrap the last goal to the present turn, then limit the step from it
            last = last - np.round((last - present) / 360.0) * 360.0
            goal = last + np.clip(target - last, -step_max, step_max)
            if np.any(np.abs(goal - target) > 1e-9):
                stats["rate_clamped_steps"] += 1
            held_before = r._handoff["held"] if r._handoff else 0
            sent = r.send_action({f"{j}.pos": float(g) for j, g in zip(tl.J5, goal)})
            if r._handoff and r._handoff["held"] > held_before:
                stats["holds"] += 1
            if self.trace is not None:
                self.trace.write(json.dumps({"t": round(ts - t0, 3), "present": present.round(2).tolist(),
                                             "target": target.round(2).tolist(),
                                             "sent": [round(sent[f"{j}.pos"], 2) for j in tl.J5]}) + "\n")
            stats["steps"] += 1
            rest = period - (self.clock() - ts)
            if rest > 0:
                self.sleep(rest)
            else:
                stats["overruns"] += 1
        ms = np.array(stats.pop("infer_ms") or [0.0])
        stats.update({"s": round(self.clock() - t0, 2), "infer_ms_p50": round(float(np.median(ms)), 1),
                      "infer_ms_p95": round(float(np.percentile(ms, 95)), 1),
                      "infer_ms_max": round(float(ms.max()), 1),
                      "max_target_minus_present_deg": round(stats["max_target_minus_present_deg"], 2),
                      "follower_ticks": r.core.follower_ticks(),
                      "present_deg": [round(t * tl.DEG_PER_TICK, 2) for t in r.core.follower_ticks()]})
        r._log({"event": "policy_attempt_end", **stats})
        return stats


# ---------------------------------------------------------------------------
# Dry run: simulated servos at the handoff stop pose, frames from a recorded episode
# ---------------------------------------------------------------------------
def sim_buses(cfg, state, clock=time.monotonic) -> dict:
    """Both arms as simulated servos (scripts/replay_sim.SimArmBus), holding at the handoff
    state's present pose (joints and grippers torque on); drive registers read sane."""
    import importlib.util
    from inchworm import teleop_live as tl
    s = importlib.util.spec_from_file_location("replay_sim", ROBOT / "scripts" / "replay_sim.py")
    sim = importlib.util.module_from_spec(s); s.loader.exec_module(sim)

    class Bus(sim.SimArmBus):
        def _read_bytes(self, sid, reg, n):
            if reg in (16, 21, 28, 48):           # drive registers: sane, non-zero
                return bytes([100, 0])[:n]
            return super()._read_bytes(sid, reg, n)

    def bus(arm):
        lim = getattr(cfg.motion_guard, f"{arm}_arm_limits").servo_tick_limits
        b = Bus(f"arm_{arm}", list(state["present_ticks"][arm]) + [state["gripper_ticks"][arm]],
                clock, limits={i + 1: tuple(lim[j]) for i, j in enumerate((*tl.J5, "gripper"))})
        b._bus.read_bytes = b._read_bytes
        for sid in range(1, 7):
            b.torque[sid] = True
        return b
    return {a: bus(a) for a in ("left", "right")}


class _SimHW:
    """FollowerHardware surface on simulated servos (scripts/replay_sim.SimArmBus), both arms
    holding at the handoff state's present pose. No port, no camera."""

    def __init__(self, cfg, state, frames, follower="left", clock=time.monotonic, buses=None):
        other = "right" if follower == "left" else "left"
        buses = buses or sim_buses(cfg, state, clock)        # AA-39 full test: shared across phases
        self.bus, self.obus = buses[follower], buses[other]
        self.role = f"arm_{follower}"
        self.read_bytes, self.write_bytes = self.bus._read_bytes, self.bus._write_bytes
        self._frames, self._i = frames, 0

    def other_arm_read_bytes(self, other):
        return self.obus._read_bytes, lambda: None

    def other_arm_bus(self, other):
        return self.obus._read_bytes, self.obus._write_bytes, lambda: None

    def other_arm_servo_bus(self, other):
        return self.obus, self.obus._read_bytes, lambda: None

    def open_camera(self, follower, w, h, fps):
        from types import SimpleNamespace

        def grab():
            f = self._frames[min(self._i, len(self._frames) - 1)]
            self._i += 1
            return SimpleNamespace(data=f, corrupt=False)
        return grab

    def close_camera(self):
        pass

    def close(self):
        pass


def _replay_frames(root: Path, episode: int, n: int):
    from inchworm.policy_runner import DatasetReplaySource
    src = DatasetReplaySource(root, episode)
    return [src.item(i)[0] for i in range(min(n, src.length))]


def start_preview(get_robot, port: int):
    """Browser live view of the frames the POLICY gets (robot._last_frame, after format_frame),
    with the frame mean / std written on it. Never opens a camera (one stream at a time). The
    view only updates while the policy runs (frames are grabbed per control step). None if off."""
    if not port:
        return None
    import record_handoff as rh

    def frame():
        r = get_robot()
        f = None if r is None else r._last_frame
        if f is None:
            return None
        import cv2
        g = f[:, :, 0] if f.ndim == 3 else f
        out = np.ascontiguousarray(f).copy()
        cv2.putText(out, f"{getattr(r.config, 'follower', '')} cam  mean {g.mean():.0f}  std {g.std():.0f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return out
    try:
        pv = rh.Preview(frame, port)
    except OSError as exc:
        print(f"(no live view: port {port}: {exc})")
        return None
    print(f"LIVE VIEW (what the policy sees): open {pv.url} in a browser")
    return pv


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, default=DEFAULT_CKPT)
    ap.add_argument("--arm", choices=("left", "right"), default="left", help="the arm the policy moves")
    ap.add_argument("--policy-arm", choices=("left", "right"), default="left",
                    help="the arm the checkpoint was trained on; differs from --arm = AA-39 joint-offset map")
    ap.add_argument("--handoff-state", type=Path, default=None,
                    help="handoff state (default out/handoff_state.json; --dry-run: out/sim_handoff_state.json "
                         "if it matches --arm, written by replay_cycle.py's dry run)")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--max-deg-s", type=float, default=5.0, help="speed cap per joint (0.5..10 deg/s)")
    ap.add_argument("--max-jump-deg", type=float, default=10.0,
                    help="stop + hold if a policy target is this far from the present pose")
    ap.add_argument("--max-s", type=float, default=20.0, help="attempt time cap (s)")
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-19 - AA-25 first policy test")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware: simulated servos at the handoff stop pose, real checkpoint + cell mesh, "
                         "wrist frames replayed from a recorded episode; runs one attempt")
    ap.add_argument("--replay-dataset", type=Path, default=DEFAULT_REPLAY)
    ap.add_argument("--replay-episode", type=int, default=26)
    ap.add_argument("--preview-port", type=int, default=8765, help="browser live view of the policy's frames (0 = off)")
    a = ap.parse_args(argv)
    if not 0.5 <= a.max_deg_s <= 10.0:
        print("REFUSED: --max-deg-s must be 0.5..10"); return 2
    if not 0 < a.max_jump_deg <= 20.0:
        print("REFUSED: --max-jump-deg must be 0..20"); return 2
    logging.basicConfig(level=logging.INFO)
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from inchworm import handoff as ho
    from inchworm.arm_transfer import JointOffsetMap, MappedPolicy
    from inchworm.config import load_config
    from inchworm.policy_runner import CheckpointLoader, CheckpointPolicy, LeRobotBackend
    from lerobot_robot_inchworm import InchwormFollower, InchwormFollowerConfig

    cfg = load_config()
    amap = None
    if a.arm != a.policy_arm:                               # refuses on a sign mismatch, before any load
        amap = JointOffsetMap(cfg, source=a.policy_arm, target=a.arm)
        print(f"AA-39 joint-offset map: {a.policy_arm}-trained policy on the {a.arm} arm; "
              f"{a.policy_arm} deg = {a.arm} deg + {amap.summary()}")
    state_path = a.handoff_state
    if state_path is None:
        state_path = ho.STATE_PATH
        sim_path = ho.STATE_PATH.with_name("sim_" + ho.STATE_PATH.name)
        if a.dry_run and sim_path.exists() and not ho.load_state(sim_path)["locked"][a.arm]:
            state_path = sim_path
    meta = CheckpointLoader(a.checkpoint).load()            # validates: 5 joints, no gripper
    policy = CheckpointPolicy(meta, LeRobotBackend(device=a.device, follower=a.policy_arm))
    print(f"checkpoint {a.checkpoint}  sha256 {meta.sha256[:16]}")
    from inchworm.policy_contract import Observation
    t_w = time.perf_counter()                               # warm-up (first CUDA call ~2 s): before connect
    policy.step(Observation(frame=np.zeros((480, 640), np.uint8),
                            joint_positions_deg=np.full(5, 180.0, np.float32), step_index=0))
    policy.reset()
    if amap is not None:
        policy = MappedPolicy(policy, amap)
    print(f"policy warm-up {time.perf_counter() - t_w:.1f} s (nothing connected yet)")

    robot = InchwormFollower(InchwormFollowerConfig(
        id=a.arm, follower=a.arm, camera=True, tube_support="handoff", catchup_deg_s=a.max_deg_s,
        handoff_state=str(state_path),
        allow_unverified_soft_limits=a.allow_unverified_soft_limits, control_hz=float(a.fps),
        log_dir=str(ROBOT / "out" / "rollout") if a.dry_run else ""))
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    tdir = ROBOT / "out" / "rollout"
    tdir.mkdir(parents=True, exist_ok=True)
    trace_path = tdir / f"policy_{'dry_' if a.dry_run else ''}{a.arm}_{utc}.jsonl"
    ref = np.array(DEMO_END_LEFT_TICKS, dtype=np.float64) * 360.0 / 4096
    if a.policy_arm != "left":
        print("REFUSED: the finish rule's demo-end pose is for the left-trained checkpoint"); return 2
    if amap is not None:
        ref = amap.to_target(ref)                           # same URDF pose on the target arm
    kw = dict(fps=a.fps, max_deg_s=a.max_deg_s, max_jump_deg=a.max_jump_deg, max_s=a.max_s, finish_ref_deg=ref)

    if a.dry_run:
        st = ho.load_state(state_path)
        frames = _replay_frames(a.replay_dataset, a.replay_episode, int(a.max_s * a.fps) + 5)
        robot._open_hardware = lambda c: _SimHW(cfg, st, frames, follower=a.arm)
        robot.prompt = lambda msg: print(msg + "YES (dry run)") or "YES"
        print(f"DRY RUN ({a.arm} arm): simulated servos at the handoff stop frame {st['stop_frame']} "
              f"({state_path.name}, {st['utc']}), real cell mesh, frames of {a.replay_dataset.name} "
              f"episode {a.replay_episode}" + ("" if amap is None else
                                               " (a LEFT episode: mapping check only)"))
        robot.connect()
        _log_map(robot, a, amap)
        try:
            with trace_path.open("w") as tr:
                res = PolicyAttempt(robot, policy, trace=tr, **kw).run()
        finally:
            robot.prompt = lambda msg: ""
            robot.disconnect()
        print("dry run result:", json.dumps(res))
        print(f"trace: {trace_path}")
        ok = res["end"] in ("time_cap", "finished") and res["steps"] > 0
        return 0 if ok else 1

    from lerobot.utils.keyboard_input import create_key_listener
    keys: "queue.Queue[str]" = queue.Queue()
    listener = None
    closing = False

    def start_listener():
        return create_key_listener(lambda k: keys.put(k), controls_help="g go, s stop, r return, q quit")

    def prompt(msg: str) -> str:
        nonlocal listener
        if listener is not None:
            listener.stop(); listener = None
        try:
            return input(msg)
        finally:
            if not closing and robot.is_connected:
                listener = start_listener()
                while not keys.empty():
                    keys.get_nowait()

    robot.prompt = prompt
    rc, n = 0, 0
    preview = start_preview(lambda: robot, a.preview_port)
    try:
        robot.connect()                                     # handoff checks + typed YES
        _log_map(robot, a, amap)
        listener = start_listener()
        if listener is None:
            print("REFUSED: no keyboard listener (run in an interactive terminal)"); return 2
        print(f"POLICY TEST: speed cap {a.max_deg_s:g} deg/s, jump guard {a.max_jump_deg:g} deg, "
              f"time cap {a.max_s:g} s. Success is NOT judged by software: watch the arm.")
        print("IDLE: g = go, r = return to the stop pose, q = quit")
        with trace_path.open("w") as tr:
            while True:
                try:
                    k = keys.get(timeout=0.05).lower()
                except queue.Empty:
                    continue
                if k == "q":
                    break
                if k == "g":
                    print(f"ATTEMPT {n}: policy driving (s = stop + hold)")
                    tr.write(json.dumps({"attempt": n}) + "\n")
                    res = PolicyAttempt(robot, policy, keys=keys, trace=tr, **kw).run()
                    print(f"ATTEMPT {n} ended ({res['end']}): {res['steps']} steps, {res['s']} s, "
                          f"holds {res['holds']}, overruns {res['overruns']}, "
                          f"inference p95 {res['infer_ms_p95']} ms. Arms HOLD.")
                    n += 1
                    if res["end"] == "finished":
                        print("FINISHED by the rule (still 2 s near the demo end). The lock is not automated yet: "
                              "look, then r = return or q = quit.")
                    elif res["end"] == "time_cap":
                        print("Time cap without the finish rule: count it as a failed attempt.")
                    if res["end"] == "jump_guard":
                        print("Jump guard: quit (q) and report; do not retry without the agent.")
                    print("IDLE: g = go again from here, r = return to the stop pose, q = quit")
                elif k == "r":
                    print("RETURN: both arms back to the stop pose")
                    robot.handoff_return(confirm=robot.returns_done == 0)
                    print("IDLE: g = go, r = return, q = quit")
    except KeyboardInterrupt:
        print("\nCtrl-C: holding.")
    except Exception as exc:  # noqa: BLE001
        print(f"\nSTOPPED: {type(exc).__name__}: {exc}\nBoth arms hold where they are.")
        rc = 1
    finally:
        closing = True
        if preview is not None:
            preview.stop()
        if listener is not None:
            listener.stop()
        if robot.is_connected:
            robot.disconnect()                              # support the tube + both arms, Enter
        print(f"trace: {trace_path}")
    return rc


def _log_map(robot, a, amap) -> None:
    robot._log({"event": "policy_arm", "arm": a.arm, "policy_arm": a.policy_arm, "checkpoint": str(a.checkpoint),
                "joint_offset_map_deg": None if amap is None else amap.summary()})


if __name__ == "__main__":
    sys.exit(main())
