"""AA-26 policy runner on the real LeRobot 0.6.1 API (run in robot/lerobot_env).

A tiny ACT checkpoint and a tiny recorded dataset (follower features, grey frame in 3
channels) are generated in tmp_path; the checkpoint goes through CheckpointLoader,
LeRobotBackend (pre/post processors) and the unchanged PolicySupervisor in dry-run with
observations replayed by DatasetReplaySource. No hardware is opened.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(sys.version_info < (3, 12), reason="lerobot_env only")

import torch  # noqa: E402

from inchworm.config import PolicySupervisorConfig, load_config  # noqa: E402
from inchworm.kinematics import build_chain  # noqa: E402
from inchworm.policy_contract import JOINT_ORDER, N_JOINTS, Observation  # noqa: E402
from inchworm.policy_runner import (  # noqa: E402
    CheckpointIncompatible,
    CheckpointLoader,
    CheckpointPolicy,
    DatasetReplaySource,
    LeRobotBackend,
    PolicyRunner,
    raw_servo_deg_to_q_rad,
)
from inchworm.supervisor import SupervisorOutcome  # noqa: E402

H, W = 480, 640
STOP_POSE_DEG = np.array([176.8, 187.4, 191.1, 109.3, 178.9], dtype=np.float32)  # recorded frame 0


def _make_dataset(root: Path, n_frames: int = 6) -> None:
    from lerobot.datasets import LeRobotDataset
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
    from lerobot_robot_inchworm.config_inchworm import InchwormFollowerConfig
    from lerobot_robot_inchworm.inchworm_follower import follower_features, format_frame

    obs_ft, act_ft = follower_features(InchwormFollowerConfig(follower="left"))
    features = {**hw_to_dataset_features(act_ft, ACTION, use_video=False),
                **hw_to_dataset_features(obs_ft, OBS_STR, use_video=False)}
    ds = LeRobotDataset.create("local/tiny", 30, root=root, robot_type="inchworm_follower",
                               features=features, use_videos=False)
    rng = np.random.default_rng(0)
    for i in range(n_frames):
        state = STOP_POSE_DEG + 0.1 * i
        obs = {f"{j}.pos": float(v) for j, v in zip(JOINT_ORDER, state)}
        obs["wrist"] = format_frame(rng.integers(0, 255, (H, W), dtype=np.uint8), H, W, 3)
        act = {f"{j}.pos": float(v) + 0.5 for j, v in zip(JOINT_ORDER, state)}
        ds.add_frame({**build_dataset_frame(ds.features, obs, prefix=OBS_STR),
                      **build_dataset_frame(ds.features, act, prefix=ACTION), "task": "insert"})
    ds.save_episode()
    ds.finalize()


def _make_checkpoint(ckpt: Path, dataset_root: Path) -> None:
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    cfg = ACTConfig(
        input_features={"observation.state": PolicyFeature(FeatureType.STATE, (N_JOINTS,)),
                        "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, H, W))},
        output_features={"action": PolicyFeature(FeatureType.ACTION, (N_JOINTS,))},
        chunk_size=4, n_action_steps=4, dim_model=32, n_heads=2, dim_feedforward=64,
        n_encoder_layers=1, n_decoder_layers=1, use_vae=False,
        pretrained_backbone_weights=None, device="cpu", push_to_hub=False,
    )
    policy = ACTPolicy(cfg)
    policy.save_pretrained(ckpt)
    stats = LeRobotDataset("local/tiny", root=dataset_root).meta.stats
    pre, post = make_pre_post_processors(cfg, dataset_stats=stats)
    pre.save_pretrained(ckpt)
    post.save_pretrained(ckpt)
    (ckpt / "train_config.json").write_text(json.dumps({"dataset": {"root": str(dataset_root)}}))


@pytest.fixture(scope="module")
def tiny(tmp_path_factory):
    base = tmp_path_factory.mktemp("aa26")
    root, ckpt = base / "ds", base / "pretrained_model"
    _make_dataset(root)
    _make_checkpoint(ckpt, root)
    return root, ckpt


def test_loader_accepts_lerobot_061_config(tiny):
    _, ckpt = tiny
    meta = CheckpointLoader(ckpt).load()
    assert meta.action_dim == N_JOINTS
    assert set(meta.observation_keys) == {"observation.state", "observation.images.wrist"}


def test_loader_rejects_gripper_in_training_dataset_actions(tiny, tmp_path):
    root, ckpt = tiny
    bad_root = tmp_path / "ds"
    (bad_root / "meta").mkdir(parents=True)
    info = json.loads((root / "meta" / "info.json").read_text())
    info["features"]["action"]["names"] = [f"{j}.pos" for j in JOINT_ORDER[:4]] + ["gripper.pos"]
    (bad_root / "meta" / "info.json").write_text(json.dumps(info))
    bad = tmp_path / "ckpt"
    bad.mkdir()
    for f in ("config.json", "model.safetensors"):
        (bad / f).write_bytes((ckpt / f).read_bytes())
    (bad / "train_config.json").write_text(json.dumps({"dataset": {"root": str(bad_root)}}))
    with pytest.raises(CheckpointIncompatible, match="gripper"):
        CheckpointLoader(bad).load()


def test_backend_recorded_frame_gives_five_joint_targets(tiny):
    root, ckpt = tiny
    backend = LeRobotBackend(device="cpu")
    CheckpointPolicy(CheckpointLoader(ckpt).load(), backend)
    replay = DatasetReplaySource(root, 0)
    frame, joints = replay()
    assert frame.shape == (H, W) and frame.dtype == np.uint8
    raw = backend.raw_observation(Observation(frame=frame, joint_positions_deg=joints, step_index=0))
    assert raw["wrist"].shape == (H, W, 3)
    assert np.array_equal(raw["wrist"][..., 0], raw["wrist"][..., 2])          # grey in 3 channels
    action = backend.step(Observation(frame=frame, joint_positions_deg=joints, step_index=0))
    assert list(action) == list(JOINT_ORDER)
    assert all(np.isfinite(v) for v in action.values())
    # MEAN_STD unnormalised: an untrained head lands near the recorded action scale (~180 deg),
    # not near 0 as it would without the postprocessor.
    assert all(abs(v - 180.0) < 120.0 for v in action.values())


def test_supervisor_dry_run_replay_never_commands_gripper(tiny):
    root, ckpt = tiny
    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    to_rad = raw_servo_deg_to_q_rad(cfg, "left")
    replay = DatasetReplaySource(root, 0)
    target = chain.fk(to_rad(replay.item(replay.length - 1)[1]))[:3, 3]
    policy = CheckpointPolicy(CheckpointLoader(ckpt).load(), LeRobotBackend(device="cpu"))
    seen, commanded, locks = [], [], []
    inner = policy.step
    policy.step = lambda obs: seen.append(inner(obs)) or seen[-1]
    runner = PolicyRunner(
        policy=policy,
        supervisor_kwargs={
            "config": PolicySupervisorConfig(max_steps=5, max_wall_time_s=60.0, workspace_radius_m=0.15,
                                             max_action_magnitude_deg=5.0, max_retries=0,
                                             retract_pose_q_rad=[0.0] * N_JOINTS),
            "chain": chain, "target_attachment_point_m": target, "frame_source": replay,
            "motion_executor": commanded.append, "approach_callable": lambda: True,
            "joint_deg_to_rad": to_rad, "dry_run": True,
        },
        lock_sm=None, dry_run=True,
    )
    result = runner.run_insertion()
    assert result.outcome == SupervisorOutcome.HALT            # never-seated stub: step limit
    assert result.supervisor_result.total_steps == 5
    assert len(seen) == 5
    for a in seen:
        assert set(a) == set(JOINT_ORDER) and "gripper" not in a
    assert commanded == []                                       # dry-run: nothing sent


_AUDIT_CKPT = Path(os.environ.get("AA26_REAL_CKPT", "/tmp/lerobot-audit/train_act/checkpoints/last/pretrained_model"))


@pytest.mark.skipif(not (_AUDIT_CKPT / "config.json").exists(), reason="no real 0.6.1 ACT checkpoint on this host")
def test_real_trained_checkpoint_loads_and_steps():
    meta = CheckpointLoader(_AUDIT_CKPT).load()
    backend = LeRobotBackend(device="cuda" if torch.cuda.is_available() else "cpu")
    CheckpointPolicy(meta, backend)
    obs = Observation(frame=np.full((H, W), 90, np.uint8), joint_positions_deg=STOP_POSE_DEG, step_index=0)
    action = backend.step(obs)
    assert list(action) == list(JOINT_ORDER) and all(np.isfinite(v) for v in action.values())
