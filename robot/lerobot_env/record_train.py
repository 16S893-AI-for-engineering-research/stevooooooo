"""AA-54 per-arm insertion recording from the FROZEN training positions (owner plan 2026-11-21).

Start AFTER ``record_train.sh`` steps 1-2 (replay to the training position without the tube,
then ``scripts/place_training_tube.py``): both arms hold at ``train_<arm>`` (config/
training_poses.json), the HOLDING gripper is LOCKED on the tube, the inserting gripper is open.

    bash ~/893/robot/lerobot_env/record_train.sh --arm left            # whole sequence, live
    bash ~/893/robot/lerobot_env/record_train.sh --arm right --dry-run # no hardware

Per arm: N_EPISODES saved episodes (owner: 50), N_NOMINAL of them at the exact frozen position
(owner: 10), the others with the attachment point perturbed (<= 2 deg rotation about any axis,
<= 20 mm per x/y/z) by the HOLDING arm moving the tube (inchworm.perturb; NOT a per-joint
jitter). The schedule is planned and mesh-checked up front (seeded, saved). Before each episode
the holding arm moves the tube to that episode's placement (mesh-checked, slow; the first move of
the session asks for a typed YES); the inserting arm starts at its frozen pose. Between two perturbed placements the tube goes back
to the frozen placement first (AA-54 AC). A DISCARDED
episode keeps its placement (the next take repeats it), so the saved episodes hold exactly the
schedule. Keys as record_handoff.py: r record, s save, d discard, q quit.

Per episode the requested + achieved (FK) perturbation and the measured holding ticks go to
``<dataset>/perturbations.json`` (and the robot log). Grippers are never commanded.
"""
from __future__ import annotations

import argparse
import json
import logging
import queue
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT = HERE.parent
sys.path.insert(0, str(HERE))
import record_handoff as rh  # noqa: E402

TIMELINE = ROBOT / "out" / "inchworm" / "timeline.jsonl"
STATE = ROBOT / "out" / "handoff_state.json"


class Refused(RuntimeError):
    pass


def leader_map_path(arm: str) -> Path:
    return ROBOT / "calibration_data" / f"leader_map_{arm}.json"


def preflight(cfg, arm: str, state: dict, timeline: Path = TIMELINE) -> dict:
    """Refusals before anything connects. Returns the frozen position."""
    from inchworm import training_poses as tp
    drift = tp.check_against_timeline(cfg, timeline)
    if drift:
        raise Refused("the frozen training positions no longer match the timeline + calibration "
                      "(owner approval needed):\n  " + "\n  ".join(drift))
    pos = tp.for_arm(arm)
    if not leader_map_path(arm).exists():
        raise Refused(f"no leader map {leader_map_path(arm).name}: fit it first "
                      f"(scripts/leader_mapping.py capture/fit --follower {arm}, owner at the arms)")
    want = {pos["inserting_arm"]: False, pos["holding_arm"]: True}
    if (state["stop_frame"] != pos["timeline_frame"] or not state["tube_in_grippers"]
            or state["locked"] != want or state["goal_ticks"] != pos["joint_ticks"]):
        raise Refused(f"handoff state: frame {state['stop_frame']}, locked {state['locked']}, tube "
                      f"{state['tube_in_grippers']} - want the frozen train_{arm} pose, {want}, tube in "
                      "(run scripts/place_training_tube.py first)")
    return pos


class TrainSession(rh.Session):
    """record_handoff.Session + the perturbation schedule: before each take the holding arm moves
    the tube to the placement of the next SAVED episode."""

    def __init__(self, robot, teleop, dataset, task, schedule, fps=30, keys=None, out=None, **kw):
        super().__init__(robot, teleop, dataset, task, fps, keys, **kw)
        self.schedule, self.out = list(schedule), out
        self.records = []
        self.placed = None                     # schedule index the tube is at now (None = frozen)

    def frozen(self):
        """(holding ticks, tube pose) at the frozen training position (the first nominal entry)."""
        e = next((x for x in self.schedule if x.nominal), None)
        if e is None:
            st = self.robot._handoff["state"]
            return st["goal_ticks"][self.robot._handoff["other"]], st["tube_pose"]
        return e.holding_ticks, e.tube_pose

    def place_next(self) -> bool:
        """Move the tube to the placement of episode ``saved``. False = schedule complete."""
        i = self.saved
        if i >= len(self.schedule):
            return False
        if self.placed == i:
            return True
        e = self.schedule[i]
        frozen = self.frozen()
        h = self.robot._handoff or {}
        if h.get("hold_ticks") is not None and list(h["hold_ticks"]) != list(frozen[0]):
            # AA-54 AC: back to the frozen holding pose before the next perturbation
            self.say("TUBE: back to the frozen placement first")
            self.robot.move_holding(frozen[0], frozen[1], confirm=getattr(self.robot, "holding_moves_done", 0) == 0,
                                    clock=self.clock, sleep=self.sleep)
        if e.nominal and list((self.robot._handoff or {}).get("hold_ticks") or frozen[0]) == list(frozen[0]):
            self.placed = i
            self.say(f"TUBE: episode {i} placement NOMINAL (frozen)")
            return True
        self.say(f"TUBE: episode {i} placement " + ("NOMINAL (frozen)" if e.nominal else
                 f"rotated {e.achieved['rot_deg']:.2f} deg, moved {e.achieved['trans_mm']} mm"))
        self.robot.move_holding(e.holding_ticks, e.tube_pose,
                                confirm=getattr(self.robot, "holding_moves_done", 0) == 0,
                                clock=self.clock, sleep=self.sleep)
        self.placed = i
        return True

    def run(self) -> None:
        if not self.place_next():
            self.say("schedule already complete"); return
        self.say("IDLE: move the leader to the start pose, then r = record, q = quit")
        while True:
            k = self._key()
            if k == "q":
                return
            if k == "r":
                i = self.saved
                why = self.record_episode()
                self._note(i, why)
                if self.quit:
                    self.say(f"QUIT ({self.saved} saved, {self.discarded} discarded): the arms hold")
                    return
                self.say("RETURN: the inserting arm goes back to its start, the tube stays placed")
                self.robot.handoff_return(confirm=self.robot.returns_done == 0, clock=self.clock, sleep=self.sleep)
                if not self.place_next():
                    self.say(f"SCHEDULE COMPLETE ({self.saved} saved, {self.discarded} discarded)")
                    return
                self.say(f"IDLE ({self.saved} saved / {len(self.schedule)}, {self.discarded} discarded): "
                         "move the leader, then r = record, q = quit")
            self.sleep(0.02)

    def _note(self, i: int, why: str) -> None:
        from dataclasses import asdict
        h = self.robot._handoff or {}
        rec = {"schedule_index": i, "result": why,
               "dataset_episode": self.dataset.num_episodes - 1 if why == "save" else None,
               "holding_ticks_target": h.get("hold_ticks"), "episode": asdict(self.schedule[i])}
        self.records.append(rec)
        self.robot._log({"event": "perturbation_episode", **{k: v for k, v in rec.items() if k != "episode"},
                         "nominal": self.schedule[i].nominal, "achieved": self.schedule[i].achieved})
        if self.out is not None:
            d = json.loads(self.out.read_text()) if self.out.exists() else {}
            d["takes"] = self.records
            self.out.write_text(json.dumps(d, indent=1))


def plan(cfg, arm: str, seed: int, n: int, n_nominal: int, timeline: Path = TIMELINE, say=print):
    from inchworm import perturb as pb
    t0 = time.time()
    say(f"planning {n} placements for {arm} ({n_nominal} nominal, seed {seed}) with the cell mesh ...")
    pl = pb.build_planner(cfg, arm, timeline)
    eps = pb.plan_schedule(pl, seed, n, n_nominal)
    say(f"  done in {time.time() - t0:.0f} s: {pb.summary(eps)}")
    return pl, eps


def dry_run(arm: str, seed: int, n: int, n_nominal: int, out: Path = None) -> int:
    """No hardware: plan the schedule, then plan + mesh-check, per placement, the tube move to it
    from the frozen placement (and the same line back), and the inserting arm's return from the
    perturbed inserted pose."""
    import numpy as np
    from inchworm import handoff_return as hr, perturb as pb
    from inchworm.config import load_config
    from inchworm.kinematics import build_chain, with_soft_limits
    cfg = load_config()
    pl, eps = plan(cfg, arm, seed, n, n_nominal)
    chain = with_soft_limits(build_chain(cfg.so101_kinematics), getattr(cfg.motion_guard, f"{arm}_arm_limits"))
    bad = 0
    for e in eps:
        home = {arm: pl.ticks[arm], pl.hold: e.holding_ticks}
        model = hr.CarriedTubeModel(cfg, pl.checker, home, np.asarray(e.tube_pose), follower=arm)
        try:
            frozen = list(pl.ticks[pl.hold])
            for w in hr._line(frozen, e.holding_ticks)[1:]:          # always from the frozen placement (AC)
                p = model.problem(pl.ticks[arm], w)
                if p is not None:
                    raise hr.ReturnRefused(f"tube move: {p}")
            p = hr.plan_return(cfg, model, chain, pl.limits, {arm: e.insert_ticks, pl.hold: e.holding_ticks})
            print(f"  episode {e.index:2d} {'nominal  ' if e.nominal else 'perturbed'} rot {e.achieved['rot_deg']:.2f} "
                  f"deg trans {e.achieved['trans_mm']} mm: move + return OK ({p.checks} return checks)")
        except hr.ReturnRefused as exc:
            bad += 1
            print(f"  episode {e.index:2d}: REFUSED: {exc}")
    if out is not None:
        pb.write(out, pl, seed, eps, {"dry_run": True})
        print(f"schedule: {out}")
    print(f"dry run {arm}: " + ("all placements, tube moves and returns plan clean" if not bad else f"{bad} refused"))
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("left", "right"), required=True, help="the INSERTING arm (its policy)")
    ap.add_argument("--seed", type=int, default=None, help="schedule seed (default: from the clock, saved)")
    ap.add_argument("--episodes", type=int, default=None, help="saved episodes (default 50)")
    ap.add_argument("--nominal", type=int, default=None, help="of them at the frozen pose (default 10)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--allow-unverified-soft-limits", default="owner 2026-11-21 - AA-54 training recording")
    ap.add_argument("--no-camera", action="store_true", help="joints only (rehearsal)")
    ap.add_argument("--preview-port", type=int, default=8765)
    ap.add_argument("--state", type=Path, default=STATE)
    ap.add_argument("--dry-run", action="store_true", help="no hardware: plan + check every placement")
    a = ap.parse_args(argv)
    from inchworm import perturb as pb
    n = pb.N_EPISODES if a.episodes is None else a.episodes
    n_nom = pb.N_NOMINAL if a.nominal is None else a.nominal
    seed = int(time.time()) % 100000 if a.seed is None else a.seed
    if a.dry_run:
        return dry_run(a.arm, seed, n, n_nom, ROBOT / "out" / f"sim_perturbations_{a.arm}.json")
    logging.basicConfig(level=logging.INFO)
    from inchworm import handoff as ho
    from inchworm.config import load_config
    cfg = load_config()
    try:
        pos = preflight(cfg, a.arm, ho.load_state(a.state))
    except (Refused, ho.HandoffRefused) as exc:
        print(f"REFUSED: {exc}"); return 2
    pl, eps = plan(cfg, a.arm, seed, n, n_nom)
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from lerobot.utils.keyboard_input import create_key_listener
    from lerobot_robot_inchworm import (InchwormFollower, InchwormFollowerConfig, InchwormLeader,
                                        InchwormLeaderConfig)
    robot = InchwormFollower(InchwormFollowerConfig(
        id=a.arm, follower=a.arm, camera=not a.no_camera, tube_support="handoff", handoff_state=str(a.state),
        allow_unverified_soft_limits=a.allow_unverified_soft_limits, control_hz=float(a.fps)))
    teleop = InchwormLeader(InchwormLeaderConfig(id="leader", follower=a.arm))
    name = f"insert_{a.arm}_v2"
    root = ROBOT / "datasets" / f"{name}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    task = f"insert the {a.arm} gripper into the next attachment point (before lock)"
    keys: "queue.Queue[str]" = queue.Queue()
    listener = dataset = preview = None
    closing = False
    rc = 0

    def start_listener():
        return create_key_listener(lambda k: keys.put(k), controls_help="r record, s save, d discard, q quit")

    def prompt(msg: str) -> str:
        nonlocal listener
        if listener is not None:
            listener.stop(); listener = None
        try:
            return input(msg)
        finally:
            if not closing and dataset is not None and robot.is_connected:
                listener = start_listener()
                while not keys.empty():
                    keys.get_nowait()

    robot.prompt = prompt
    teleop.connect()
    try:
        robot.connect()
        dataset = rh.make_dataset(robot, root, f"local/{name}", a.fps)
        out = root / "perturbations.json"
        pb.write(out, pl, seed, eps, {"dataset": str(root), "training_position": pos})
        listener = start_listener()
        if listener is None:
            print("REFUSED: no keyboard listener (run in an interactive terminal)"); return 2
        print(f"dataset: {root}\nschedule: {out}")
        if not a.no_camera and a.preview_port:
            try:
                preview = rh.Preview(lambda: robot._last_frame, a.preview_port)
                print(f"LIVE VIEW: open {preview.url} in a browser")
            except OSError as exc:
                print(f"live view not started ({exc}); recording is not affected")
        TrainSession(robot, teleop, dataset, task, eps, a.fps, keys, out=out).run()
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
        if dataset is not None:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer()
            dataset.finalize()
            print(f"dataset: {root}  episodes: {dataset.num_episodes}")
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
    return rc


if __name__ == "__main__":
    sys.exit(main())
