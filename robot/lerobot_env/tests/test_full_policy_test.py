"""AA-39 full test (full_policy_test.py) end to end on simulated servos: real cell mesh, real
timeline, a stub policy that drives to the demo-end pose (no checkpoint, no GPU)."""
from __future__ import annotations

import importlib.util
import json

import numpy as np
import pytest

from tests.test_inchworm_plugin import ROBOT

_s = importlib.util.spec_from_file_location("full_policy_test", ROBOT / "lerobot_env" / "full_policy_test.py")
fpt = importlib.util.module_from_spec(_s); _s.loader.exec_module(fpt)
ph = fpt.ph
pytestmark = pytest.mark.skipif(not fpt.TIMELINE.exists() or not ph.DEFAULT_REPLAY.exists(),
                                reason="needs the timeline and the replay dataset")


class ToDemoEnd:
    """Left-frame policy: always the median demo-end pose (the speed cap makes it a slow move)."""

    def reset(self):
        pass

    def step(self, obs):                     # at most 5 deg ahead of the arm (inside the 10 deg jump guard)
        p = np.asarray(obs.joint_positions_deg, float)
        t = np.array(ph.DEMO_END_LEFT_TICKS) * fpt.DEG_PER_TICK
        return {j: float(v) for j, v in zip(fpt.J5, p + np.clip(t - p, -5.0, 5.0))}


def _state42(tmp_path):
    from inchworm import cycle_replay as cr
    from inchworm import handoff as ho
    from inchworm.config import load_config
    cfg = load_config()
    fr, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(fpt.TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    f = fr[42]
    t = {a: cr.q_to_ticks(cfg, a, f.q[a]) for a in cr.ARMS}
    p = tmp_path / "h42.json"
    ho.write_state(p, stop_frame=42, step=f.step, timeline_digest=fpt._digest(fpt.TIMELINE), tube_in_grippers=True,
                   goal_ticks=t, present_ticks=t, gripper_ticks={"left": 1547, "right": 2038},
                   locked=f.locked, q_rad=f.q, tube_pose=f.tube)
    return p


def _run(tmp_path, monkeypatch, answers=None, max_s="12"):
    import lerobot_robot_inchworm.inchworm_follower as fmod
    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    monkeypatch.setattr(fpt, "RUNNER_STOP", tmp_path / "stop_move")
    monkeypatch.setattr(fpt, "OUT_DIR", tmp_path / "out")
    monkeypatch.setattr(fpt, "STATE_67", tmp_path / "state67.json")
    monkeypatch.setattr(fpt, "load_policy", lambda ckpt, dev: (ToDemoEnd(), "stub"))
    ui = fpt.UI(True, auto=answers or {})
    rc = fpt.main(["--handoff-state", str(_state42(tmp_path)), "--dry-run", "--max-deg-s", "10",
                   "--max-s", max_s], ui=ui)
    log = [json.loads(l) for p in (tmp_path / "out").glob("full_test_dry_*.jsonl") for l in p.read_text().splitlines()]
    return rc, log, fpt.SIM["buses"]


def _grip(buses, arm):
    return buses[arm].pos[6]


def test_full_test_left_policy_lock_cycle_right_policy_lock(tmp_path, monkeypatch):
    rc, log, buses = _run(tmp_path, monkeypatch)
    ev = [e["event"] for e in log]
    assert rc == 0 and "full_test_done" in ev
    g = [(e["event"], e["arm"]) for e in log if e["event"] in ("gripper_lock", "gripper_unlock")]
    assert g == [("gripper_lock", "left"), ("gripper_unlock", "right"), ("gripper_lock", "right")]
    seg = next(e for e in log if e["event"] == "segment")
    assert seg["frames"] == [49, 67]
    # end: both grippers LOCKED and still powered, joints 1-5 of both arms off (after the Enter)
    assert all(2013 <= _grip(buses, a) <= 2066 and buses[a].torque[6] for a in ("left", "right"))
    assert not any(buses[a].torque[s] for a in ("left", "right") for s in range(1, 6))
    st = json.loads((tmp_path / "sim_state67.json").read_text())
    assert st["stop_frame"] == 67 and st["locked"] == {"left": True, "right": False}
    assert np.abs(np.array(st["present_ticks"]["left"]) - ph.DEMO_END_LEFT_TICKS).max() <= 20


def test_owner_says_the_left_key_is_not_in_nothing_unlocks(tmp_path, monkeypatch):
    rc, log, buses = _run(tmp_path, monkeypatch, answers={"left_look": "n"})
    ev = [e["event"] for e in log]
    assert rc == 0 and "gripper_unlock" not in ev and "left_to_demo_end" not in ev
    assert 2013 <= _grip(buses, "right") <= 2066 and buses["right"].torque[6]      # right still LOCKED
    assert not (tmp_path / "sim_state67.json").exists()
    assert not any(buses[a].torque[s] for a in ("left", "right") for s in range(1, 6))


def test_quit_after_the_left_attempt_locks_nothing(tmp_path, monkeypatch):
    rc, log, buses = _run(tmp_path, monkeypatch, answers={"left_choice": "q"})
    assert rc == 0 and [e["event"] for e in log if e["event"].startswith("gripper")] == []
    assert _grip(buses, "left") < 1700                                              # left still open
    assert not any(buses[a].torque[s] for a in ("left", "right") for s in range(1, 6))


def test_lock_far_from_the_demo_end_stops_before_the_right_unlocks(tmp_path, monkeypatch):
    # 1 s cap: the left is still far from the demo end; the owner locks anyway -> no continuation
    rc, log, buses = _run(tmp_path, monkeypatch, max_s="1")
    stop = next(e for e in log if e["event"] == "stopped")
    assert "from the demo-end pose" in stop["reason"] and "gripper_unlock" not in [e["event"] for e in log]
    assert all(2013 <= _grip(buses, a) <= 2066 for a in ("left", "right"))           # both LOCKED


def test_refuses_a_state_that_is_not_frame_42(tmp_path, monkeypatch):
    from inchworm import handoff as ho
    p = _state42(tmp_path)
    st = json.loads(p.read_text()); st["stop_frame"] = 67
    p.write_text(json.dumps(st))
    assert fpt.main(["--handoff-state", str(p), "--dry-run"]) == 2
    assert ho.load_state(p)["stop_frame"] == 67
