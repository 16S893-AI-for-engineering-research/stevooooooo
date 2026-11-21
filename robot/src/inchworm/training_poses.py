"""Owner plan 2026-11-21: the two FROZEN policy training positions.

``config/training_poses.json`` holds, per position, the joint ticks of BOTH arms, which arm
inserts, which arm holds (LOCKED) and the tube pose. Owner rule: these positions must NOT
change after training unless the owner says so. They are saved as raw servo ticks (what the
policies see), not as timeline radians, so a later zero-offset or timeline change cannot move
them silently.

* ``train_left``  = cycle frame 42: right LOCKED holding, left open ~50 mm under its point.
* ``train_right`` = cycle frame 67: left LOCKED holding (gait inserted pose), right open
  ~50 mm under its point.

``check_against_timeline`` reports any drift between the frozen ticks and what the current
timeline + calibration would give (callers decide; recording and deployment REFUSE on drift).
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import numpy as np

ROBOT_ROOT = Path(__file__).resolve().parents[2]
PATH = ROBOT_ROOT / "config" / "training_poses.json"
SCHEMA = "inchworm.training_poses/1"
NAMES = ("train_left", "train_right")
ARMS = ("left", "right")


class TrainingPoseError(RuntimeError):
    pass


def load(path: Path = PATH) -> dict:
    d = json.loads(Path(path).read_text())
    if d.get("schema") != SCHEMA or not d.get("frozen"):
        raise TrainingPoseError(f"{path}: not a frozen {SCHEMA} file")
    for n in NAMES:
        p = d["positions"][n]
        for a in ARMS:
            if len(p["joint_ticks"][a]) != 5:
                raise TrainingPoseError(f"{path}: {n} {a} needs 5 joint ticks")
        if p["locked"][p["inserting_arm"]] or not p["locked"][p["holding_arm"]]:
            raise TrainingPoseError(f"{path}: {n} must have the holding arm LOCKED and the inserting arm open")
        T = np.asarray(p["tube_pose"], dtype=float)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)):
            raise TrainingPoseError(f"{path}: {n} tube pose is not a finite 4x4")
    return d


def position(name: str, path: Path = PATH) -> dict:
    if name not in NAMES:
        raise TrainingPoseError(f"unknown training position {name!r} (want one of {NAMES})")
    return load(path)["positions"][name]


def for_arm(inserting_arm: str, path: Path = PATH) -> dict:
    """The position the given arm's policy is trained / run from."""
    return position(f"train_{inserting_arm}", path)


def check_against_timeline(cfg, timeline: Path, path: Path = PATH, tol_ticks: int = 0) -> List[str]:
    """Problems if the current timeline + calibration no longer reproduce the frozen ticks."""
    from inchworm import cycle_replay as cr
    d = load(path)
    frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(timeline), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    out = []
    for n in NAMES:
        p = d["positions"][n]
        f = frames[p["timeline_frame"]]
        for a in ARMS:
            now = cr.q_to_ticks(cfg, a, f.q[a])
            diff = [int(x) - int(y) for x, y in zip(now, p["joint_ticks"][a])]
            if max(abs(x) for x in diff) > tol_ticks:
                out.append(f"{n} {a}: timeline frame {p['timeline_frame']} now gives {now}, frozen "
                           f"{p['joint_ticks'][a]} (diff {diff}); owner approval needed to change")
    return out


def joint_ticks(name: str, path: Path = PATH) -> Dict[str, List[int]]:
    return {a: [int(t) for t in v] for a, v in position(name, path)["joint_ticks"].items()}
