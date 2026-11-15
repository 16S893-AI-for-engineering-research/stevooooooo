"""AA-44: the LeRobot plugin (inchworm_follower / inchworm_leader) against simulated servos.

Run: cd robot/lerobot_env && uv run pytest tests/ -v
No hardware: the plugin's hardware hooks are replaced by scripts/replay_sim.SimArmBus.
"""
from __future__ import annotations

import importlib.util
import json
import pathlib
import time

import numpy as np
import pytest

from inchworm import teleop_live as tl
from inchworm.config import load_config
from lerobot_robot_inchworm import (InchwormFollower, InchwormFollowerConfig, InchwormLeader,
                                    InchwormLeaderConfig, InchwormRefused)
from lerobot_robot_inchworm import inchworm_follower as fmod

ROBOT = pathlib.Path(__file__).resolve().parents[2]
_s = importlib.util.spec_from_file_location("replay_sim", ROBOT / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)

CFG = load_config()
F_TICKS = [1948, 1390, 3122, 1032, 990]          # inside the left soft/EEPROM limits
GRIP = 2034
MAP_PATH = ROBOT / "calibration_data" / "leader_map_left.json"
MAP = tl.load_leader_map(MAP_PATH, "left")
H, W = 48, 64


class Bus(sim.SimArmBus):
    def _read_bytes(self, sid, reg, n):
        if reg in (16, 21, 28, 48):               # drive registers: sane, non-zero
            return bytes([100, 0])[:n]
        return super()._read_bytes(sid, reg, n)


class Cam:
    def __init__(self, corrupt_every=0):
        self.n, self.corrupt_every, self.closed = 0, corrupt_every, False

    def grab(self):
        from types import SimpleNamespace
        self.n += 1
        if self.corrupt_every and self.n % self.corrupt_every == 0:
            return SimpleNamespace(data=np.zeros((0, 0), np.uint8), corrupt=True)
        return SimpleNamespace(data=np.full((H, W), self.n % 256, np.uint8), corrupt=False)


class FakeHW:
    """Same surface as inchworm_follower.FollowerHardware."""

    def __init__(self, clock=time.monotonic, other=None, cam=None):
        lim = CFG.motion_guard.left_arm_limits.servo_tick_limits
        self.bus = Bus("arm_left", F_TICKS + [GRIP], clock,
                       limits={i + 1: tuple(lim[j]) for i, j in enumerate((*tl.J5, "gripper"))})
        self.bus._bus.read_bytes = self.bus._read_bytes
        self.role = "arm_left"
        self.read_bytes = self.bus._read_bytes
        self.write_bytes = self.bus._write_bytes
        self.other = other
        self.cam = cam or Cam()
        self.camera_open = False
        self.closed = False

    def other_arm_read_bytes(self, other):
        assert other == "right"
        return self.other, lambda: None

    def open_camera(self, follower, w, h, fps):
        self.camera_open = True
        return self.cam.grab

    def close_camera(self):
        self.camera_open = False

    def close(self):
        self.close_camera(); self.closed = True


class Leader:
    def __init__(self, ticks):
        self.ticks = list(ticks)

    def read(self, sid, reg, n):
        assert reg == tl.REG_PRESENT_POSITION and 1 <= sid <= 5    # read-only, joints 1-5 only
        return int(self.ticks[sid - 1]).to_bytes(2, "little")


def _leader_ticks_for(f):
    return [round((t - MAP["joints"][j]["offset_ticks"]) % 4096) for t, j in zip(f, tl.J5)]


def _robot(tmp_path, monkeypatch, hw=None, answers=("YES", ""), **kw):
    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    base = dict(id="test", calibration_dir=tmp_path / "cal", allow_unverified_soft_limits="test",
                no_collision_check="test", tube_support="no_tube", camera_width=W, camera_height=H,
                log_dir=str(tmp_path / "log"))
    base.update(kw)
    r = InchwormFollower(InchwormFollowerConfig(**base))
    hw = hw or FakeHW()
    r._open_hardware = lambda cfg: hw
    it = iter(answers)
    r.prompt = lambda *_: next(it)
    return r, hw


def _leader(tmp_path, ticks):
    t = InchwormLeader(InchwormLeaderConfig(id="test", calibration_dir=tmp_path / "cal", map_path=str(MAP_PATH)))
    lead = Leader(ticks)
    t._open_leader = lambda: (lead.read, lambda: None)
    return t, lead


def _log(tmp_path):
    return [json.loads(l) for l in next((tmp_path / "log").glob("teleop_lerobot_*.jsonl")).read_text().splitlines()]


def test_features_have_no_gripper_and_three_channel_camera(tmp_path, monkeypatch):
    r, _ = _robot(tmp_path, monkeypatch)
    assert list(r.action_features) == [f"{j}.pos" for j in tl.J5]
    assert r.observation_features["wrist"] == (H, W, 3)
    assert not any("gripper" in k for k in r.observation_features)
    r2, _ = _robot(tmp_path, monkeypatch, camera=False)
    assert "wrist" not in r2.observation_features


@pytest.mark.parametrize("kw, needle", [
    (dict(no_collision_check=""), "no_collision_check"),
    (dict(allow_unverified_soft_limits=""), "allow_unverified_soft_limits"),
    (dict(tube_support=""), "tube_support"),
    (dict(tube_support="fixture"), "tube_support"),
])
def test_connect_refuses_without_owner_statements(tmp_path, monkeypatch, kw, needle):
    if "allow_unverified_soft_limits" in kw and not CFG.motion_guard.left_arm_limits.unverified:
        pytest.skip("soft limits verified")
    r, hw = _robot(tmp_path, monkeypatch, **kw)
    opened = []
    r._open_hardware = lambda cfg: opened.append(1) or hw
    with pytest.raises(InchwormRefused, match=needle):
        r.connect()
    assert not opened and not r.is_connected


def test_connect_refuses_torque_on_and_not_confirmed(tmp_path, monkeypatch):
    hw = FakeHW()
    hw.bus.torque[3] = True
    r, _ = _robot(tmp_path, monkeypatch, hw=hw)
    with pytest.raises(InchwormRefused, match="torque is ON"):
        r.connect()
    assert hw.closed and not r.is_connected
    r2, hw2 = _robot(tmp_path, monkeypatch, answers=("no",))
    with pytest.raises(InchwormRefused, match="not confirmed"):
        r2.connect()
    assert hw2.closed and not any(hw2.bus.torque.values())


def _other_arm(torque=(1, 1, 1, 1, 1, 1), grip_ticks=2038):
    def read(sid, reg, n):
        if reg == tl.REG_TORQUE_ENABLE:
            return bytes([torque[sid - 1]])
        assert reg == tl.REG_PRESENT_POSITION and sid == 6
        return int(grip_ticks).to_bytes(2, "little")
    return read


@pytest.mark.parametrize("other, ok", [
    (_other_arm(), True),
    (_other_arm(grip_ticks=1550), False),                       # unlocked
    (_other_arm(torque=(1, 1, 1, 1, 1, 0)), False),             # gripper torque off
    (_other_arm(torque=(1, 0, 1, 1, 1, 1)), False),             # holding arm limp
])
def test_locked_arm_check(tmp_path, monkeypatch, other, ok):
    hw = FakeHW(other=other)
    r, _ = _robot(tmp_path, monkeypatch, hw=hw, tube_support="locked_arm")
    if ok:
        r.connect()
        assert r.is_connected
        r.disconnect()
    else:
        with pytest.raises(InchwormRefused, match="locked_arm"):
            r.connect()
        assert hw.closed


def test_start_gate_refuses_far_action_and_energises_nothing(tmp_path, monkeypatch):
    r, hw = _robot(tmp_path, monkeypatch)
    r.connect()
    obs = r.get_observation()
    act = {k: obs[k] for k in r.action_features}
    act["shoulder_lift.pos"] += 20.0
    with pytest.raises(InchwormRefused, match="start gate"):
        r.send_action(act)
    assert not any(hw.bus.torque.values())
    r.disconnect()
    assert hw.closed and not r.is_connected
    r.disconnect()                                   # idempotent


def test_gripper_action_rejected(tmp_path, monkeypatch):
    r, hw = _robot(tmp_path, monkeypatch)
    r.connect()
    obs = r.get_observation()
    act = {k: obs[k] for k in r.action_features}
    with pytest.raises(ValueError, match="gripper"):
        r.send_action({**act, "gripper.pos": 100.0})
    r.disconnect()


def test_follows_clamps_holds_and_releases(tmp_path, monkeypatch):
    clock = sim.SimClock()
    hw = FakeHW(clock=clock)
    r, _ = _robot(tmp_path, monkeypatch, hw=hw)
    r.connect()
    t, lead = _leader(tmp_path, _leader_ticks_for(F_TICKS))
    t.connect()
    lead.ticks[0] = (lead.ticks[0] + 100) % 4096
    for _ in range(90):
        obs = r.get_observation()
        sent = r.send_action(t.get_action()); clock.sleep(1 / 30)
    assert abs(hw.bus.pos[1] - (F_TICKS[0] + 100)) <= 3
    assert abs(obs["shoulder_pan.pos"] - (F_TICKS[0] + 100) * tl.DEG_PER_TICK) < 1.0
    hi = r.core.limits["elbow_flex"][1]
    lead.ticks[2] = (lead.ticks[2] + 2000) % 4096                # far past the elbow limit
    for _ in range(300):
        sent = r.send_action(t.get_action()); clock.sleep(1 / 30)
    assert sent["elbow_flex.pos"] <= hi * tl.DEG_PER_TICK + 1e-6
    assert hw.bus.pos[3] <= hi + 1 and r.core.clamps["elbow_flex"] > 0
    assert hw.bus.goal[6] == GRIP and not hw.bus.torque[6]         # gripper never touched
    r.disconnect(); t.disconnect()
    ev = [e["event"] for e in _log(tmp_path)]
    assert ev[:2] == ["connect", "energised"] and ev[-2:] == ["hold", "released"]
    assert not any(hw.bus.torque[s] for s in range(1, 6)) and hw.closed


def test_stop_file_raises_keyboard_interrupt(tmp_path, monkeypatch):
    r, hw = _robot(tmp_path, monkeypatch)
    r.connect()
    obs = r.get_observation()
    act = {k: obs[k] for k in r.action_features}
    r.send_action(act)
    (tmp_path / "STOP_TELEOP").touch()
    with pytest.raises(KeyboardInterrupt):
        r.send_action(act)
    r.disconnect()
    assert not any(hw.bus.torque[s] for s in range(1, 6))


def test_corrupt_frames_are_skipped_never_raised(tmp_path, monkeypatch):
    hw = FakeHW(cam=Cam(corrupt_every=2))
    r, _ = _robot(tmp_path, monkeypatch, hw=hw)
    r.connect()
    frames = [r.get_observation()["wrist"] for _ in range(6)]
    assert all(f.shape == (H, W, 3) and f.dtype == np.uint8 for f in frames)
    assert all((f[..., 0] == f[..., 1]).all() for f in frames)   # grey copied into 3 channels
    r.disconnect()
    assert not hw.camera_open


def test_lerobot_teleop_loop(tmp_path, monkeypatch):
    """LeRobot's own teleop loop (lerobot-teleoperate) drives the plugin."""
    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_teleoperate import teleop_loop

    r, hw = _robot(tmp_path, monkeypatch)
    t, lead = _leader(tmp_path, _leader_ticks_for(F_TICKS))
    t.connect(); r.connect()
    lead.ticks[0] = (lead.ticks[0] + 30) % 4096
    a, b, c = make_default_processors()
    teleop_loop(teleop=t, robot=r, fps=30, display_data=False, duration=1.0, teleop_action_processor=a,
                robot_action_processor=b, robot_observation_processor=c)
    assert r.core.energised and abs(hw.bus.pos[1] - (F_TICKS[0] + 30)) <= 15
    r.disconnect(); t.disconnect()
    assert not any(hw.bus.torque[s] for s in range(1, 6))


def test_lerobot_record_end_to_end(tmp_path, monkeypatch):
    """LeRobot's real record() (lerobot-record) with the plugin: one local episode, video, no hub."""
    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts import lerobot_record as rec

    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    hw = FakeHW()
    monkeypatch.setattr(InchwormFollower, "_open_hardware", lambda self, cfg: hw)
    answers = iter(["YES", ""])
    monkeypatch.setattr(InchwormFollower, "prompt", lambda self, *_: next(answers))
    lead = Leader(_leader_ticks_for(F_TICKS))
    monkeypatch.setattr(InchwormLeader, "_open_leader", lambda self: (lead.read, lambda: None))
    monkeypatch.setattr(rec, "init_keyboard_listener",
                        lambda: (None, {"exit_early": False, "rerecord_episode": False, "stop_recording": False}))
    cfg = rec.RecordConfig(
        robot=InchwormFollowerConfig(id="test", calibration_dir=tmp_path / "cal", allow_unverified_soft_limits="test",
                                     no_collision_check="test", tube_support="no_tube", camera_width=W,
                                     camera_height=H, log_dir=str(tmp_path / "log")),
        teleop=InchwormLeaderConfig(id="test", calibration_dir=tmp_path / "cal", map_path=str(MAP_PATH)),
        dataset=DatasetRecordConfig(repo_id="local/inchworm_test", single_task="test", root=tmp_path / "ds",
                                    fps=30, episode_time_s=1, reset_time_s=0, num_episodes=1, push_to_hub=False,
                                    no_stamp=True),
        play_sounds=False)
    ds = rec.record(cfg)
    ds = LeRobotDataset("local/inchworm_test", root=tmp_path / "ds")
    assert ds.meta.total_episodes == 1 and 15 <= ds.meta.total_frames <= 31
    item = ds[0]
    assert tuple(item["observation.images.wrist"].shape) == (3, H, W)
    assert item["action"].shape[-1] == 5 and item["observation.state"].shape[-1] == 5
    assert ds.meta.robot_type == "inchworm_follower"
    ev = [e["event"] for e in _log(tmp_path)]
    assert "energised" in ev and ev[-1] == "released"
    assert not any(hw.bus.torque[s] for s in range(1, 6)) and hw.bus.goal[6] == GRIP


def test_leader_is_read_only_and_maps(tmp_path):
    t, lead = _leader(tmp_path, _leader_ticks_for(F_TICKS))
    t.connect()
    act = t.get_action()
    for k, j in enumerate(tl.J5):
        d = act[f"{j}.pos"] / tl.DEG_PER_TICK - F_TICKS[k]
        assert abs(d - 4096 * round(d / 4096)) < 1.0
    t.disconnect()
    assert not t.is_connected
    bad = InchwormLeader(InchwormLeaderConfig(id="x", calibration_dir=tmp_path / "cal", follower="right",
                                              map_path=str(MAP_PATH)))
    with pytest.raises(tl.TeleopRefused):
        bad.connect()
