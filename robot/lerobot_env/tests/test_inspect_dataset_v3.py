"""scripts/inspect_dataset.py on a LeRobot v3.0 dataset (run in robot/lerobot_env: pandas + PyAV).

A tiny video dataset is recorded with the follower features: 30 fps, every camera image
repeated 3 times (10 new images/s), episode 1 saved with a 360-degree action offset.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(sys.version_info < (3, 12), reason="lerobot_env only")

from inchworm.policy_contract import JOINT_ORDER  # noqa: E402

H, W = 480, 640
SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "inspect_dataset.py"


def _load_inspector():
    spec = importlib.util.spec_from_file_location("inspect_dataset", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def dataset(tmp_path_factory) -> Path:
    from lerobot.datasets import LeRobotDataset
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
    from lerobot_robot_inchworm.config_inchworm import InchwormFollowerConfig
    from lerobot_robot_inchworm.inchworm_follower import follower_features, format_frame

    root = tmp_path_factory.mktemp("inspect") / "ds"
    obs_ft, act_ft = follower_features(InchwormFollowerConfig(follower="left"))
    features = {**hw_to_dataset_features(act_ft, ACTION), **hw_to_dataset_features(obs_ft, OBS_STR)}
    ds = LeRobotDataset.create("local/inspect", 30, root=root, robot_type="inchworm_follower",
                               features=features, use_videos=True)
    for ep, offset_deg in ((0, 0.0), (1, 360.0)):
        for i in range(30):
            if i % 3 == 0:          # smooth image (AV1 repeats it within codec noise), new level each time
                level = 40 + 15 * (i // 3) + 100 * ep
                grey = np.clip(level + np.linspace(0, 60, W)[None, :] + np.zeros((H, 1)), 0, 255).astype(np.uint8)
            state = np.array([176.0, 187.0, 191.0, 109.0, 179.0]) + 0.1 * i
            act = state + 0.5
            act[1] += offset_deg                              # elbow-style turn error on shoulder_lift
            obs = {f"{j}.pos": float(v) for j, v in zip(JOINT_ORDER, state)}
            obs["wrist"] = format_frame(grey, H, W, 3)
            ds.add_frame({**build_dataset_frame(ds.features, obs, prefix=OBS_STR),
                          **build_dataset_frame(ds.features,
                                                {f"{j}.pos": float(v) for j, v in zip(JOINT_ORDER, act)},
                                                prefix=ACTION),
                          "task": "insert"})
        ds.save_episode()
    ds.finalize()
    return root


def test_lerobot_v3_report(dataset):
    mod = _load_inspector()
    assert mod.is_lerobot_v3(dataset)
    rep = mod.lerobot_v3_report(dataset)
    assert (rep["episodes"], rep["frames"], rep["fps"]) == (2, 60, 30)
    assert rep["features"]["observation.images.wrist"]["shape"] == [H, W, 3]
    e0, e1 = rep["per_episode"]
    assert e0["length"] == 30 and e0["wrist_frames_decoded"] == 30
    assert e0["wrist_new_images_per_s"] == pytest.approx(10.0, abs=1.5)
    assert e1["wrist_new_images_per_s"] == pytest.approx(10.0, abs=1.5)
    assert e0["action_offset_joints"] == []
    assert e1["action_offset_joints"] == ["shoulder_lift.pos"]
    assert e0["start_state"]["shoulder_pan.pos"] == pytest.approx(176.0, abs=0.01)


def test_cli_prints_report_and_writes_labels(dataset, capsys, monkeypatch):
    mod = _load_inspector()
    monkeypatch.setattr(sys, "argv", ["inspect_dataset.py", str(dataset), "--label", "0=success",
                                      "--label", "1=failure"])
    assert mod.main() == 0
    out = capsys.readouterr().out
    assert "Episode count   : 2" in out
    assert "360-degree action-offset check: FAIL on episodes [1]" in out
    assert "Labels: 1 success / 2 labelled of 2" in out
    assert json.loads((dataset / "meta" / "episode_labels.json").read_text()) == {"0": "success", "1": "failure"}
    monkeypatch.setattr(sys, "argv", ["inspect_dataset.py", str(dataset), "--label", "5=success"])
    assert mod.main() == 1
