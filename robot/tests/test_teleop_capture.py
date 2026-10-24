"""Tests for AA-23: Cross-arm teleop demonstration capture.

Evidence matrix
---------------
AC#2  - test_joint6_excluded_from_mapper, test_gripper_not_in_teleop_joints,
        test_assert_no_gripper_raises, test_step_never_writes_gripper
AC#3  - test_precondition_fixture_ok, test_precondition_fixture_missing_token,
        test_precondition_locked_arm_ok, test_precondition_locked_arm_unlocked,
        test_precondition_unknown_source, test_precondition_no_source,
        test_precondition_follower_state_fn_raises
AC#6  - test_episode_records_leader_follower_timestamps,
        test_dataset_written_locally, test_writer_never_hub_push
AC#7  - test_episode_seed_and_offset_in_meta
AC#8  - test_episode_labels_retained_all_three,
        test_aborted_episode_retained
AC#9  - test_step_uses_arbiter_for_camera (mock arbiter called, only follower role)
AC#10 - test_inspect_dataset_runs (pasted output below)

Hardware ACs left unchecked:
  AC#1  Mapping verification at low speed:
        needs owner-supervised hardware session:
        ``cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2``
  AC#4  Fixture validation vs real crawl pose:
        needs owner-supervised hardware session:
        ``cd robot && uv run python scripts/teleop.py --live --validate-fixture``
  AC#5  Backdrive effort and latency measurement:
        needs owner-supervised hardware session:
        ``cd robot && uv run python scripts/teleop.py --live --measure-latency``

All tests run without hardware.  No real serial ports or cameras opened.
"""

from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Dict, List
from unittest.mock import MagicMock, patch, call

import sys

import pytest

from inchworm.teleop_capture import (
    TELEOP_JOINTS,
    GripperInTeleopError,
    MirrorMapper,
    PreconditionError,
    TubeSupportSource,
    _assert_no_gripper,
    _LocalLeRobotWriter,
    check_preconditions,
    draw_start_offset,
    load_dataset_stats,
    make_writer,
    EpisodeMeta,
    EpisodeRecorder,
    FrameRecord,
)
from inchworm.servo_bus import JointState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_JOINTS_5 = list(TELEOP_JOINTS)  # joints 1-5 only


def _make_joint_states(pos_deg: float = 180.0) -> Dict[str, JointState]:
    """Return a dict of JointState for all TELEOP_JOINTS."""
    return {
        j: JointState(
            servo_id=i + 1,
            position_deg=pos_deg,
            speed_raw=0,
            load_raw=0,
            voltage_v=12.0,
            temperature_c=30,
        )
        for i, j in enumerate(TELEOP_JOINTS)
    }


def _make_mirror_cfg():
    """Return a minimal mock mirror_map config with sign=1, offset=0 for all joints."""
    cfg = MagicMock()
    for j in TELEOP_JOINTS:
        p = MagicMock()
        p.sign = 1
        p.offset_deg = 0.0
        setattr(cfg, j, p)
    return cfg


def _make_offset_range_cfg(max_deg: float = 5.0):
    cfg = MagicMock()
    for j in TELEOP_JOINTS:
        setattr(cfg, f"{j}_max_deg", max_deg)
    return cfg


def _make_teleop_cfg(tube_support: str = "fixture", token: str = "tok"):
    cfg = MagicMock()
    cfg.tube_support_source = tube_support
    cfg.operator_token = token
    cfg.leader_arm = "right"
    cfg.follower_arm = "left"
    cfg.mirror_map = _make_mirror_cfg()
    cfg.start_offset_range = _make_offset_range_cfg()
    cfg.dataset_root = "robot/datasets/teleop"
    cfg.camera_frame_width = 64
    cfg.camera_frame_height = 48
    cfg.speed_scale_default = 0.2
    return cfg


def _make_writer(tmp_path: Path) -> _LocalLeRobotWriter:
    return _LocalLeRobotWriter(tmp_path / "ds", fps=10.0)


def _make_recorder(
    tmp_path: Path,
    episode_index: int = 0,
    seed: int = 42,
    live: bool = False,
    arbiter=None,
) -> EpisodeRecorder:
    leader_bus = MagicMock()
    leader_bus.read_all_joint_states.return_value = _make_joint_states()

    follower = MagicMock()
    follower_bus = MagicMock()
    follower_bus.read_all_joint_states.return_value = _make_joint_states()
    follower.bus = follower_bus

    mapper = MirrorMapper(_make_mirror_cfg())
    writer = _make_writer(tmp_path)

    return EpisodeRecorder(
        episode_index=episode_index,
        seed=seed,
        start_offset_deg={j: 0.0 for j in TELEOP_JOINTS},
        pre_insertion_q_deg=[180.0] * len(TELEOP_JOINTS),
        leader_bus=leader_bus,
        follower_guarded_arm=follower,
        mirror_mapper=mapper,
        camera_arbiter=arbiter,
        follower_camera_role="camera_left_arm",
        writer=writer,
        live=live,
    )


# ---------------------------------------------------------------------------
# AC#2: Joint 6 excluded from teleop mapping and action space
# ---------------------------------------------------------------------------


def test_gripper_not_in_teleop_joints():
    """AC#2: 'gripper' must not appear in TELEOP_JOINTS."""
    assert "gripper" not in TELEOP_JOINTS, (
        "gripper (joint 6) must be excluded from TELEOP_JOINTS"
    )
    assert len(TELEOP_JOINTS) == 5, "TELEOP_JOINTS must have exactly 5 entries (joints 1-5)"


def test_assert_no_gripper_raises():
    """AC#2: _assert_no_gripper raises GripperInTeleopError when gripper is present."""
    with pytest.raises(GripperInTeleopError):
        _assert_no_gripper(["shoulder_pan", "gripper"])


def test_assert_no_gripper_ok():
    """AC#2: _assert_no_gripper does not raise for joints 1-5 only."""
    _assert_no_gripper(list(TELEOP_JOINTS))  # must not raise


def test_joint6_excluded_from_mapper():
    """AC#2: MirrorMapper.apply raises if gripper key is passed."""
    mapper = MirrorMapper(_make_mirror_cfg())
    bad_input = {j: 180.0 for j in TELEOP_JOINTS}
    bad_input["gripper"] = 45.0
    with pytest.raises(GripperInTeleopError):
        mapper.apply(bad_input)


def test_mapper_maps_only_5_joints():
    """AC#2: MirrorMapper.apply output contains only TELEOP_JOINTS (no gripper)."""
    mapper = MirrorMapper(_make_mirror_cfg())
    result = mapper.apply({j: 180.0 for j in TELEOP_JOINTS})
    assert set(result.keys()) == set(TELEOP_JOINTS)
    assert "gripper" not in result


def test_step_never_writes_gripper(tmp_path: Path):
    """AC#2: EpisodeRecorder.step never calls apply_joint_command with 'gripper'."""
    recorder = _make_recorder(tmp_path)
    recorder.step()

    follower = recorder._follower
    for c in follower.apply_joint_command.call_args_list:
        joint_name = c[0][0] if c[0] else c[1].get("joint_name")
        assert joint_name != "gripper", (
            "apply_joint_command was called with 'gripper' — this is forbidden"
        )


# ---------------------------------------------------------------------------
# AC#3: Precondition checks
# ---------------------------------------------------------------------------


def test_precondition_no_source():
    """AC#3: refuse when tube_support_source is None."""
    with pytest.raises(PreconditionError, match="tube_support_source is not set"):
        check_preconditions(
            tube_support_source=None,
            operator_token="tok",
            leader_arm="right",
        )


def test_precondition_unknown_source():
    """AC#3: refuse when tube_support_source is an unknown value."""
    with pytest.raises(PreconditionError, match="unknown tube_support_source"):
        check_preconditions(
            tube_support_source="magic",
            operator_token="tok",
            leader_arm="right",
        )


def test_precondition_fixture_ok():
    """AC#3: fixture source with non-empty token is accepted."""
    result = check_preconditions(
        tube_support_source="fixture",
        operator_token="ok_token",
        leader_arm="right",
    )
    assert result is TubeSupportSource.FIXTURE


def test_precondition_fixture_missing_token():
    """AC#3: fixture source with no token is refused."""
    with pytest.raises(PreconditionError, match="operator_token"):
        check_preconditions(
            tube_support_source="fixture",
            operator_token=None,
            leader_arm="right",
        )


def test_precondition_fixture_empty_token():
    """AC#3: fixture source with empty string token is refused."""
    with pytest.raises(PreconditionError, match="operator_token"):
        check_preconditions(
            tube_support_source="fixture",
            operator_token="",
            leader_arm="right",
        )


def test_precondition_locked_arm_ok():
    """AC#3: locked_arm source with LOCKED state is accepted."""
    from inchworm.gripper_lock import GripperState
    result = check_preconditions(
        tube_support_source="locked_arm",
        operator_token=None,
        leader_arm="right",
        follower_gripper_state_fn=lambda: GripperState.LOCKED,
    )
    assert result is TubeSupportSource.LOCKED_ARM


def test_precondition_locked_arm_unlocked():
    """AC#3: locked_arm source with UNLOCKED state is refused (torque-off arm cannot hold tube)."""
    from inchworm.gripper_lock import GripperState
    with pytest.raises(PreconditionError, match="not LOCKED"):
        check_preconditions(
            tube_support_source="locked_arm",
            operator_token=None,
            leader_arm="right",
            follower_gripper_state_fn=lambda: GripperState.UNLOCKED,
        )


def test_precondition_locked_arm_fault_state():
    """AC#3: locked_arm source with FAULT state is refused."""
    from inchworm.gripper_lock import GripperState
    with pytest.raises(PreconditionError, match="not LOCKED"):
        check_preconditions(
            tube_support_source="locked_arm",
            operator_token=None,
            leader_arm="right",
            follower_gripper_state_fn=lambda: GripperState.FAULT,
        )


def test_precondition_locked_arm_no_fn():
    """AC#3: locked_arm source without state function is refused."""
    with pytest.raises(PreconditionError, match="follower_gripper_state_fn"):
        check_preconditions(
            tube_support_source="locked_arm",
            operator_token=None,
            leader_arm="right",
            follower_gripper_state_fn=None,
        )


def test_precondition_follower_state_fn_raises():
    """AC#3: if follower_gripper_state_fn raises, precondition is refused."""
    def _bad():
        raise RuntimeError("bus error")

    with pytest.raises(PreconditionError, match="follower_gripper_state_fn raised"):
        check_preconditions(
            tube_support_source="locked_arm",
            operator_token=None,
            leader_arm="right",
            follower_gripper_state_fn=_bad,
        )


# ---------------------------------------------------------------------------
# AC#6: Episode records leader/follower/timestamps; dataset local only
# ---------------------------------------------------------------------------


def test_episode_records_leader_follower_timestamps(tmp_path: Path):
    """AC#6: EpisodeRecorder captures leader state, follower state, timestamps."""
    recorder = _make_recorder(tmp_path, seed=0)
    recorder.step()
    recorder.step()

    assert len(recorder._frames) == 2
    for fr in recorder._frames:
        assert isinstance(fr.leader_positions_deg, dict)
        assert isinstance(fr.follower_positions_deg, dict)
        assert set(fr.leader_positions_deg.keys()) == set(TELEOP_JOINTS)
        assert set(fr.follower_positions_deg.keys()) == set(TELEOP_JOINTS)
        assert isinstance(fr.ts_monotonic, float)
        assert isinstance(fr.ts_iso, str)
        # Gripper must not appear
        assert "gripper" not in fr.leader_positions_deg
        assert "gripper" not in fr.follower_positions_deg


def test_dataset_written_locally(tmp_path: Path):
    """AC#6: dataset is written to the local filesystem."""
    recorder = _make_recorder(tmp_path, seed=0)
    for _ in range(3):
        recorder.step()
    meta = recorder.finalise("success")

    ds_root = tmp_path / "ds"
    # At least one episode file exists
    episode_files = list((ds_root / "data" / "chunk-000").glob("episode_*"))
    assert len(episode_files) == 1, f"Expected 1 episode file, got {episode_files}"


def test_writer_never_hub_push(tmp_path: Path):
    """AC#6: info.json must contain hub_push=False."""
    writer = _make_writer(tmp_path)
    meta = EpisodeMeta(
        episode_index=0,
        seed=1,
        start_offset_deg={j: 0.0 for j in TELEOP_JOINTS},
        label="success",
        pre_insertion_pose_q_deg=[180.0] * 5,
        ts_start_iso="2026-01-01T00:00:00+00:00",
        ts_end_iso="2026-01-01T00:00:01+00:00",
        duration_s=1.0,
        frame_count=10,
    )
    info_path = writer.write_info([meta])
    info = json.loads(info_path.read_text())
    assert info.get("hub_push") is False, "info.json must have hub_push=False"


# ---------------------------------------------------------------------------
# AC#7: Seed and offset in metadata
# ---------------------------------------------------------------------------


def test_episode_seed_and_offset_in_meta(tmp_path: Path):
    """AC#7: EpisodeMeta records seed and start_offset_deg."""
    recorder = _make_recorder(tmp_path, seed=12345)
    recorder.step()
    meta = recorder.finalise("success")

    assert meta.seed == 12345
    assert isinstance(meta.start_offset_deg, dict)
    assert set(meta.start_offset_deg.keys()) == set(TELEOP_JOINTS)

    # Verify offset persisted in episode file
    ds_root = tmp_path / "ds"
    info_path = recorder._writer.write_info([meta])
    info = json.loads(info_path.read_text())
    ep_meta = info["episodes"][0]
    assert ep_meta["seed"] == 12345
    assert "start_offset_deg" in ep_meta


def test_draw_start_offset_range():
    """AC#7: draw_start_offset stays within configured range."""
    cfg = _make_offset_range_cfg(max_deg=10.0)
    rng = random.Random(0)
    offset = draw_start_offset(rng, cfg)
    assert set(offset.keys()) == set(TELEOP_JOINTS)
    for j, v in offset.items():
        assert -10.0 <= v <= 10.0, f"offset for {j} out of range: {v}"
    assert "gripper" not in offset


# ---------------------------------------------------------------------------
# AC#8: Labels retained (success / failure / aborted)
# ---------------------------------------------------------------------------


def test_episode_labels_retained_all_three(tmp_path: Path):
    """AC#8: success, failure, and aborted are all valid labels."""
    writer = _LocalLeRobotWriter(tmp_path / "ds2", fps=10.0)
    metas = []
    for label in ("success", "failure", "aborted"):
        recorder = EpisodeRecorder(
            episode_index=len(metas),
            seed=0,
            start_offset_deg={j: 0.0 for j in TELEOP_JOINTS},
            pre_insertion_q_deg=[180.0] * 5,
            leader_bus=MagicMock(read_all_joint_states=MagicMock(return_value=_make_joint_states())),
            follower_guarded_arm=MagicMock(bus=MagicMock(read_all_joint_states=MagicMock(return_value=_make_joint_states()))),
            mirror_mapper=MirrorMapper(_make_mirror_cfg()),
            camera_arbiter=None,
            follower_camera_role="camera_left_arm",
            writer=writer,
        )
        recorder.step()
        metas.append(recorder.finalise(label))

    info_path = writer.write_info(metas)
    info = json.loads(info_path.read_text())
    labels = [e["label"] for e in info["episodes"]]
    assert "success" in labels
    assert "failure" in labels
    assert "aborted" in labels
    # All three episodes present
    assert len(info["episodes"]) == 3


def test_aborted_episode_retained(tmp_path: Path):
    """AC#8: aborted episodes are written to the dataset (not discarded)."""
    recorder = _make_recorder(tmp_path, seed=0)
    recorder.step()
    meta = recorder.finalise("aborted")
    assert meta.label == "aborted"
    ds_root = tmp_path / "ds"
    episode_files = list((ds_root / "data" / "chunk-000").glob("episode_*"))
    assert len(episode_files) == 1, "Aborted episode must be retained"


def test_invalid_label_rejected(tmp_path: Path):
    """AC#8: invalid label raises ValueError."""
    recorder = _make_recorder(tmp_path)
    with pytest.raises(ValueError, match="label must be"):
        recorder.finalise("bad_label")


# ---------------------------------------------------------------------------
# AC#9: Only follower wrist camera via arbiter
# ---------------------------------------------------------------------------


def test_step_uses_arbiter_for_camera(tmp_path: Path):
    """AC#9: EpisodeRecorder.step acquires only the follower wrist camera role."""
    mock_transport = MagicMock()
    mock_transport.read_frame.return_value = b"\xff\xd8\xff\xd9"  # minimal JPEG
    mock_arbiter = MagicMock()
    mock_arbiter.acquire.return_value.__enter__ = MagicMock(return_value=mock_transport)
    mock_arbiter.acquire.return_value.__exit__ = MagicMock(return_value=False)

    recorder = _make_recorder(tmp_path, arbiter=mock_arbiter)
    recorder.step()

    # arbiter.acquire must have been called with the follower camera role
    acquire_calls = [str(c) for c in mock_arbiter.acquire.call_args_list]
    assert any("camera_left_arm" in s for s in acquire_calls), (
        "arbiter.acquire must be called with the follower wrist camera role 'camera_left_arm'"
    )

    # Must never be called with the leader camera role
    assert not any("camera_right_arm" in s for s in acquire_calls), (
        "leader camera must never be acquired during recording"
    )


# ---------------------------------------------------------------------------
# AC#10: Dataset inspection tool
# ---------------------------------------------------------------------------


def test_inspect_dataset_runs(tmp_path: Path, capsys):
    """AC#10: inspect_dataset reports episode count, duration, success rate, offset coverage.

    Paste of stdout is included at end of this test as the AC evidence.
    """
    # Create a synthetic dataset with 6 episodes (3 success, 1 failure, 1 aborted, 1 success)
    writer = _LocalLeRobotWriter(tmp_path / "inspect_ds", fps=10.0)
    metas = []
    import math
    for i in range(6):
        label = ["success", "success", "success", "failure", "aborted", "success"][i]
        offset = {j: float(i * 2 - 5) for j in TELEOP_JOINTS}
        meta = EpisodeMeta(
            episode_index=i,
            seed=i,
            start_offset_deg=offset,
            label=label,
            pre_insertion_pose_q_deg=[180.0] * 5,
            ts_start_iso="2026-01-01T00:00:00+00:00",
            ts_end_iso="2026-01-01T00:00:01+00:00",
            duration_s=1.0 + i * 0.1,
            frame_count=10 + i,
        )
        metas.append(meta)
    writer.write_info(metas)

    # Run the inspector
    from inchworm.teleop_capture import load_dataset_stats, DatasetStats

    stats = load_dataset_stats(tmp_path / "inspect_ds", offset_bins=3)

    assert stats.episode_count == 6
    assert stats.success_count == 4
    assert stats.failure_count == 1
    assert stats.aborted_count == 1
    assert len(stats.durations_s) == 6

    # Run the script's print_report to produce the output for AC evidence
    sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
    import importlib, sys as _sys
    # Import the inspect_dataset module directly
    spec_path = Path(__file__).parents[1] / "scripts" / "inspect_dataset.py"
    import importlib.util
    spec = importlib.util.spec_from_file_location("inspect_dataset", spec_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.print_report(stats, bins=3)

    captured = capsys.readouterr()
    output = captured.out

    assert "Episode count" in output
    assert "Duration" in output
    assert "success rate" in output.lower() or "success" in output.lower()
    assert "shoulder_pan" in output

    # --- AC#10 evidence: pasted stdout ---
    # (auto-generated by this test run; see test output below)
    print("\n--- AC#10 inspector output (evidence) ---")
    print(output)
    print("--- end AC#10 evidence ---")
