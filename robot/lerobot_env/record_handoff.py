"""AA-23 rapid-fire demo recording from the demo-start handoff (owner spec 2026-11-15).

Start AFTER  scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live ...
(both arms holding, right gripper LOCKED on the tube, left gripper open under the key).

    bash ~/893/robot/lerobot_env/record_handoff.sh            # or: uv run python record_handoff.py --help

Keys (terminal focused):
    r  start recording (the left arm catches up slowly to the leader, then follows)
    s  stop + SAVE the episode, then the automatic return
    d  stop + DISCARD the episode, then the automatic return
    q  quit. While recording: SAVE the episode and quit (no return; the arms hold, then the
       normal release prompt). Ctrl-C = hold (an unfinished episode is discarded), then release.

Return after s/d (not recorded): left arm straight out of the key 50 mm (-z of its base
frame, 10 mm/s), joint line to its stop pose (10 deg/s), then the right arm (tube carried)
to its stop pose. Planned and mesh-checked before anything moves; the FIRST return of a
session asks for a typed YES. Episodes stop and save by themselves after 3 min.
Move the leader to the new start pose before pressing r again.

Grippers are never commanded. Nothing is uploaded; the dataset is local and gitignored.
"""
from __future__ import annotations

import argparse
import logging
import queue
import sys
import time
from pathlib import Path

ROBOT = Path(__file__).resolve().parents[1]
EPISODE_MAX_S = 180.0          # owner 2026-11-15


class Session:
    """Idle -> recording -> (save | discard) -> return -> idle. Hardware/dataset injected."""

    def __init__(self, robot, teleop, dataset, task: str, fps: int = 30, keys=None,
                 clock=time.perf_counter, sleep=time.sleep, say=print, max_s: float = EPISODE_MAX_S):
        self.robot, self.teleop, self.dataset, self.task = robot, teleop, dataset, task
        self.fps, self.keys, self.clock, self.sleep, self.say = fps, keys or queue.Queue(), clock, sleep, say
        self.max_s = max_s
        self.saved = self.discarded = 0
        self.quit = False

    def _key(self):
        try:
            return self.keys.get_nowait().lower()
        except queue.Empty:
            return None

    def _frame(self, obs, act) -> dict:
        from lerobot.utils.constants import ACTION, OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame
        return {**build_dataset_frame(self.dataset.features, obs, prefix=OBS_STR),
                **build_dataset_frame(self.dataset.features, act, prefix=ACTION), "task": self.task}

    def _tick(self, record: bool) -> None:
        """One control step. Idle = the arm holds (no action sent); recording = follow the leader."""
        obs = self.robot.get_observation()
        if record:
            act = self.teleop.get_action()
            self.robot.send_action(act)
            self.dataset.add_frame(self._frame(obs, act))

    def record_episode(self) -> str:
        """Returns 'save' or 'discard'."""
        n = self.dataset.num_episodes
        self.robot.episode = n
        self.robot._log({"event": "episode_start", "episode": n})
        self.say(f"RECORDING episode {n}  (s = save, d = discard, q = save + quit; auto-save after {self.max_s:g} s)")
        t0 = self.clock()
        period = 1.0 / self.fps
        why = "save"
        while True:
            ts = self.clock()
            k = self._key()
            if k in ("s", "d", "q"):
                why = "discard" if k == "d" else "save"
                self.quit = k == "q"
                break
            if ts - t0 >= self.max_s:
                self.say(f"{self.max_s:g} s cap reached: saving")
                why = "save"
                break
            self._tick(record=True)
            rest = period - (self.clock() - ts)
            if rest > 0:
                self.sleep(rest)
        frames = self.dataset.writer.episode_buffer["size"] if getattr(self.dataset, "writer", None) else None
        if why == "save" and frames:
            self.say("saving (the arms hold) ...")
            self.dataset.save_episode()
            self.saved += 1
        else:
            self.dataset.clear_episode_buffer()
            self.discarded += 1
            why = "discard"
        self.robot._log({"event": "episode_end", "episode": n, "result": why, "frames": frames,
                         "s": round(self.clock() - t0, 2)})
        self.robot.episode = None
        return why

    def run(self) -> None:
        self.say("IDLE: move the leader to the start pose, then r = record, q = quit")
        while True:
            k = self._key()
            if k == "q":
                return
            if k == "r":
                self.record_episode()
                if self.quit:
                    self.say(f"QUIT ({self.saved} saved, {self.discarded} discarded): the arms hold")
                    return
                self.say("RETURN: both arms go back to the stop pose")
                self.robot.handoff_return(confirm=self.robot.returns_done == 0, clock=self.clock, sleep=self.sleep)
                self.say(f"IDLE ({self.saved} saved, {self.discarded} discarded): move the leader, "
                         "then r = record, q = quit")
            self.sleep(0.02)


def make_dataset(robot, root: Path, repo_id: str, fps: int):
    from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
    from lerobot.processor import make_default_processors
    from lerobot.utils.feature_utils import combine_feature_dicts
    t, _, o = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(pipeline=t, initial_features=create_initial_features(
            action=robot.action_features), use_videos=True),
        aggregate_pipeline_dataset_features(pipeline=o, initial_features=create_initial_features(
            observation=robot.observation_features), use_videos=True))
    n_cam = len(robot.cameras)
    return LeRobotDataset.create(repo_id, fps, root=root, robot_type=robot.name, features=features,
                                 use_videos=True, image_writer_threads=4 * n_cam)


def dry_run(frames=range(42, 50)) -> int:
    """Plan the return from the stop pose and from each planned insertion frame, with the real cell
    model and the handoff state. Nothing opens a port."""
    import numpy as np
    from inchworm import cycle_replay as cr, handoff as ho, handoff_return as hr, teleop_live as tl
    from inchworm.config import load_config
    from inchworm.kinematics import build_chain, with_soft_limits
    from inchworm.motion_guard import build_mesh_collision_guard
    cfg = load_config()
    st = ho.load_state()
    checker = build_mesh_collision_guard(cfg, "left").checker
    model = hr.CarriedTubeModel(cfg, checker, st["goal_ticks"], np.asarray(st["tube_pose"]))
    chain = with_soft_limits(build_chain(cfg.so101_kinematics), cfg.motion_guard.left_arm_limits)
    lim = {}
    for arm in ("left", "right"):
        soft = tl.soft_limit_ticks(cfg, arm)
        y = getattr(cfg.motion_guard, f"{arm}_arm_limits").servo_tick_limits
        lim[arm] = {j: (max(soft[j][0], y[j][0] + 10), min(soft[j][1], y[j][1] - 10)) for j in tl.J5}
    fr = cr.load_timeline(ROBOT / "out" / "inchworm" / "timeline.jsonl")
    bad = 0
    print(f"DRY RUN (no hardware): handoff state {st['utc']}, stop frame {st['stop_frame']}")
    for i in frames:
        now = {"left": cr.q_to_ticks(cfg, "left", fr[i].q["left"]), "right": st["goal_ticks"]["right"]}
        try:
            p = hr.plan_return(cfg, model, chain, lim, now)
            print(f"  from frame {i} ({fr[i].step}): OK, {p.checks} mesh checks; " + "; ".join(p.summary))
        except hr.ReturnRefused as exc:
            bad += 1
            print(f"  from frame {i}: REFUSED: {exc}")
    print("dry run: " + ("all returns plan clean" if not bad else f"{bad} refused"))
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", default="insert_left")
    ap.add_argument("--task", default="insert the left gripper into the next attachment point (before lock)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-15 - AA-23 handoff recording")
    ap.add_argument("--no-camera", action="store_true", help="joints only (rehearsal)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware: plan + mesh-check the return from the planned insertion (timeline frames 42-49)")
    a = ap.parse_args(argv)
    if a.dry_run:
        return dry_run()
    logging.basicConfig(level=logging.INFO)
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from lerobot.utils.keyboard_input import create_key_listener
    from lerobot_robot_inchworm import (InchwormFollower, InchwormFollowerConfig, InchwormLeader,
                                        InchwormLeaderConfig)
    robot = InchwormFollower(InchwormFollowerConfig(
        id="left", follower="left", camera=not a.no_camera, tube_support="handoff",
        allow_unverified_soft_limits=a.allow_unverified_soft_limits, control_hz=float(a.fps)))
    teleop = InchwormLeader(InchwormLeaderConfig(id="leader", follower="left"))
    root = ROBOT / "datasets" / f"{a.dataset}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    keys: "queue.Queue[str]" = queue.Queue()
    listener = None
    dataset = None
    closing = False
    rc = 0

    def start_listener():
        return create_key_listener(lambda k: keys.put(k), controls_help="r record, s save, d discard, q quit")

    def prompt(msg: str) -> str:
        """The key listener holds the terminal raw: pause it around every typed prompt."""
        nonlocal listener
        if listener is not None:
            listener.stop()
            listener = None
        try:
            return input(msg)
        finally:
            if not closing and dataset is not None and robot.is_connected:
                listener = start_listener()
                while not keys.empty():          # keys typed into the prompt are not commands
                    keys.get_nowait()

    robot.prompt = prompt
    teleop.connect()
    try:
        robot.connect()                                    # handoff checks + typed YES
        dataset = make_dataset(robot, root, f"local/{a.dataset}", a.fps)
        listener = start_listener()
        if listener is None:
            print("REFUSED: no keyboard listener (run in an interactive terminal)")
            return 2
        print(f"dataset: {root}")
        Session(robot, teleop, dataset, a.task, a.fps, keys).run()
    except KeyboardInterrupt:
        print("\nCtrl-C: holding.")
    except Exception as exc:  # noqa: BLE001 - ReturnRefused (nothing moved) / ReturnAborted (holding)
        print(f"\nSTOPPED: {type(exc).__name__}: {exc}\nBoth arms hold where they are.")
        rc = 1
    finally:
        closing = True
        if listener is not None:
            listener.stop()
            listener = None
        if dataset is not None:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer()
            dataset.finalize()
            print(f"dataset: {root}  episodes: {dataset.num_episodes}")
        if robot.is_connected:
            robot.disconnect()                             # release prompt: support the tube + both arms
        if teleop.is_connected:
            teleop.disconnect()
    return rc


if __name__ == "__main__":
    sys.exit(main())
