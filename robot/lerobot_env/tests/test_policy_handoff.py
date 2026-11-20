"""AA-25 first policy test loop (policy_handoff.PolicyAttempt) on simulated servos via the plugin
in tube_support=handoff (fixtures from test_inchworm_plugin)."""
from __future__ import annotations

import importlib.util
import queue

import numpy as np
import pytest

from inchworm import teleop_live as tl
from tests.test_inchworm_plugin import (F_TICKS, ROBOT, Gate, HandoffHW, _handoff_robot, _log, sim)

_s = importlib.util.spec_from_file_location("policy_handoff", ROBOT / "lerobot_env" / "policy_handoff.py")
ph = importlib.util.module_from_spec(_s); _s.loader.exec_module(ph)

F_DEG = np.array(F_TICKS) * tl.DEG_PER_TICK


class Policy:
    """Returns ``fn(present_deg, step)`` as joint targets; records resets/observations."""

    def __init__(self, fn):
        self.fn, self.resets, self.obs = fn, 0, []

    def reset(self):
        self.resets += 1

    def step(self, obs):
        self.obs.append(obs)
        return {j: float(v) for j, v in zip(tl.J5, self.fn(np.asarray(obs.joint_positions_deg, float), obs.step_index))}


def _attempt(tmp_path, monkeypatch, fn, gate=None, keys=None, **kw):
    clock = sim.SimClock()
    hw = HandoffHW(clock)
    r, g = _handoff_robot(tmp_path, monkeypatch, hw, gate=gate)
    r.connect()
    grip = (hw.bus.goal[6], hw.bus.torque[6], hw.other_bus.goal[6], hw.other_bus.torque[6])
    pol = Policy(fn)
    res = ph.PolicyAttempt(r, pol, keys=keys, clock=clock, sleep=clock.sleep, say=lambda *_: None, **kw).run()
    return r, hw, pol, res, grip


def test_speed_cap_and_grippers_untouched(tmp_path, monkeypatch):
    goal = F_DEG + np.array([0, 4.0, -6.0, 3.0, 0])               # inside the 10 deg jump guard
    r, hw, pol, res, grip = _attempt(tmp_path, monkeypatch, lambda p, i: goal, max_deg_s=5.0, max_s=3.0)
    assert res["end"] == "time_cap" and pol.resets == 1 and res["steps"] >= 85
    assert res["rate_clamped_steps"] > 0
    # 6 deg at 5 deg/s takes 1.2 s; after 3 s the arm is there
    assert max(abs(hw.bus.pos[i + 1] - g / tl.DEG_PER_TICK) for i, g in enumerate(goal)) <= 3
    assert (hw.bus.goal[6], hw.bus.torque[6], hw.other_bus.goal[6], hw.other_bus.torque[6]) == grip
    assert pol.obs[0].frame.shape == (48, 64)                      # grey wrist frame reached the policy
    r.disconnect()


def test_speed_cap_never_exceeded(tmp_path, monkeypatch):
    goal = F_DEG + np.array([0, 9.0, 0, 0, 0])
    clock = sim.SimClock()
    hw = HandoffHW(clock)
    r, _ = _handoff_robot(tmp_path, monkeypatch, hw)
    r.connect()
    sent = []
    orig = r.send_action
    r.send_action = lambda a: sent.append(a) or orig(a)
    ph.PolicyAttempt(r, Policy(lambda p, i: goal), clock=clock, sleep=clock.sleep, say=lambda *_: None,
                     max_deg_s=5.0, max_s=2.5).run()
    lift = [a["shoulder_lift.pos"] for a in sent]
    steps = np.abs(np.diff([F_DEG[1]] + lift))
    assert steps.max() <= 5.0 / 30 + 1e-6
    r.disconnect()


def test_jump_guard_stops_and_holds_without_sending(tmp_path, monkeypatch):
    far = F_DEG + np.array([0, 0, 15.0, 0, 0])
    r, hw, pol, res, grip = _attempt(tmp_path, monkeypatch, lambda p, i: far if i >= 5 else p, max_s=3.0)
    assert res["end"] == "jump_guard" and res["steps"] == 5
    assert abs(hw.bus.goal[3] - F_TICKS[2]) <= 1                   # the far target was never commanded
    assert all(hw.bus.torque[s] for s in range(1, 6))             # holding
    assert "policy_jump_guard" in [e["event"] for e in _log(tmp_path)]
    r.disconnect()


def test_mesh_hit_holds(tmp_path, monkeypatch):
    goal = F_DEG + np.array([0, 8.0, 0, 0, 0])
    limit = F_TICKS[1] + 30
    r, hw, pol, res, _ = _attempt(tmp_path, monkeypatch, lambda p, i: goal, gate=Gate(hit_above=limit), max_s=4.0)
    assert hw.bus.goal[2] <= limit and res["holds"] >= 1
    r.disconnect()


def test_owner_stop_key(tmp_path, monkeypatch):
    class Keys:
        def __init__(self):
            self.n = 0

        def get_nowait(self):
            self.n += 1
            if self.n > 10:
                return "s"
            raise queue.Empty
    r, hw, pol, res, _ = _attempt(tmp_path, monkeypatch, lambda p, i: p, keys=Keys(), max_s=10.0)
    assert res["end"] == "owner_stop" and res["steps"] == 10
    r.disconnect()


def test_policy_with_gripper_key_is_refused(tmp_path, monkeypatch):
    clock = sim.SimClock()
    hw = HandoffHW(clock)
    r, _ = _handoff_robot(tmp_path, monkeypatch, hw)
    r.connect()

    class Bad(Policy):
        def step(self, obs):
            return {**super().step(obs), "gripper": 10.0}
    g6 = (hw.bus.goal[6], hw.bus.torque[6])
    with pytest.raises(RuntimeError, match="joints 1-5"):
        ph.PolicyAttempt(r, Bad(lambda p, i: p), clock=clock, sleep=clock.sleep, say=lambda *_: None).run()
    assert (hw.bus.goal[6], hw.bus.torque[6]) == g6
    r.disconnect()


# --- AA-39: the LEFT policy on the RIGHT arm (stop frame 67: left LOCKED, right open) -----------------
def _right_state(tmp_path):
    from inchworm import handoff as ho
    from inchworm.config import load_config
    cfg = load_config()
    left = [1972, 2337, 1775, 1438, 1999]          # replay_cycle dry run, frame 67 (sim_handoff_state.json)
    right = [2063, 2261, 2140, 1135, 1992]
    st = ho.write_state(tmp_path / "h67.json", stop_frame=67, step="right regrip middle point", timeline_digest="x",
                        tube_in_grippers=True, goal_ticks={"left": left, "right": right},
                        present_ticks={"left": left, "right": right}, gripper_ticks={"left": 2038, "right": 1547},
                        locked={"left": True, "right": False}, q_rad={"left": [0.0] * 5, "right": [0.0] * 5},
                        tube_pose=np.eye(4))
    return cfg, st


def test_left_policy_drives_the_right_arm_through_the_offset_map(tmp_path, monkeypatch):
    from inchworm.arm_transfer import JointOffsetMap, MappedPolicy
    from lerobot_robot_inchworm import InchwormFollower, InchwormFollowerConfig
    import lerobot_robot_inchworm.inchworm_follower as fmod
    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    cfg, st = _right_state(tmp_path)
    clock = sim.SimClock()
    hw = ph._SimHW(cfg, st, [np.zeros((48, 64), np.uint8)], follower="right", clock=clock)
    r = InchwormFollower(InchwormFollowerConfig(
        id="right", follower="right", tube_support="handoff", allow_unverified_soft_limits="test",
        handoff_state=str(tmp_path / "h67.json"), camera_width=64, camera_height=48, log_dir=str(tmp_path / "log")))
    r._open_hardware = lambda c: hw
    r.prompt = lambda *_: "YES"
    r._make_gate = lambda *a: Gate()
    r.connect()
    m = JointOffsetMap(cfg, source="left", target="right")
    R0 = np.array(st["goal_ticks"]["right"]) * tl.DEG_PER_TICK
    L0 = m.to_source(R0)
    seen = []

    class LeftPolicy:                               # wants +3 deg of lift relative to the LEFT stop pose
        def reset(self):
            pass

        def step(self, obs):
            seen.append(np.array(obs.joint_positions_deg, float))
            return {j: float(v) for j, v in zip(tl.J5, L0 + np.array([0, 3.0, 0, 0, 0]))}
    g6 = (hw.bus.goal[6], hw.bus.torque[6], hw.obus.goal[6], hw.obus.torque[6])
    res = ph.PolicyAttempt(r, MappedPolicy(LeftPolicy(), m), clock=clock, sleep=clock.sleep,
                           say=lambda *_: None, max_s=2.0).run()
    assert res["end"] == "time_cap" and res["steps"] > 0
    assert np.abs(seen[0] - L0).max() < 0.1          # the policy saw LEFT-equivalent degrees of the right arm
    want = (R0 + np.array([0, 3.0, 0, 0, 0])) / tl.DEG_PER_TICK
    assert max(abs(hw.bus.pos[i + 1] - want[i]) for i in range(5)) <= 3    # the RIGHT arm went +3 deg lift
    assert all(abs(hw.obus.pos[i + 1] - st["goal_ticks"]["left"][i]) <= 1 for i in range(5))  # left untouched
    assert (hw.bus.goal[6], hw.bus.torque[6], hw.obus.goal[6], hw.obus.torque[6]) == g6        # grippers untouched
    r.prompt = lambda *_: ""
    r.disconnect()
    assert not any(hw.bus.torque[s] or hw.obus.torque[s] for s in range(1, 6))
    assert hw.obus.torque[6]                         # the LOCKED left gripper stays on


def test_right_follower_refuses_the_left_stop_state(tmp_path, monkeypatch):
    """A stop-frame-42 state (right LOCKED) must refuse --arm right (it would pull on the tube)."""
    from lerobot_robot_inchworm import InchwormFollower, InchwormFollowerConfig
    from lerobot_robot_inchworm.inchworm_follower import InchwormRefused
    import lerobot_robot_inchworm.inchworm_follower as fmod
    from inchworm import handoff as ho
    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    cfg, st = _right_state(tmp_path)
    st["locked"] = {"left": False, "right": True}
    (tmp_path / "h67.json").write_text(__import__("json").dumps(st))
    hw = ph._SimHW(cfg, st, [np.zeros((48, 64), np.uint8)], follower="right")
    r = InchwormFollower(InchwormFollowerConfig(
        id="right", follower="right", tube_support="handoff", allow_unverified_soft_limits="test",
        handoff_state=str(tmp_path / "h67.json"), camera_width=64, camera_height=48, log_dir=str(tmp_path / "log")))
    r._open_hardware = lambda c: hw
    r.prompt = lambda *_: "YES"
    r._make_gate = lambda *a: Gate()
    with pytest.raises(InchwormRefused, match="right gripper is LOCKED"):
        r.connect()
    assert ho.load_state(tmp_path / "h67.json")["locked"]["right"]


# --- owner 2026-11-20 finish rule: still 2 deg for 2 s AND within 8 deg of the demo-end pose ----------
def test_finish_rule_fires_after_2_s_still_near_the_demo_end(tmp_path, monkeypatch):
    goal = F_DEG + np.array([0, 3.0, 0, 0, 0])                 # reached in 0.6 s at 5 deg/s, then still
    r, hw, pol, res, grip = _attempt(tmp_path, monkeypatch, lambda p, i: goal, max_s=10.0,
                                     finish_ref_deg=goal + np.array([0, 0, 7.0, 0, 0]))
    assert res["end"] == "finished" and 2.0 <= res["s"] <= 3.5, res
    assert res["finish_deg_from_ref"] <= 8.0
    assert all(hw.bus.torque[s] for s in range(1, 6))         # holding; the grippers were not touched
    assert (hw.bus.goal[6], hw.bus.torque[6], hw.other_bus.goal[6], hw.other_bus.torque[6]) == grip
    r.disconnect()


def test_finish_rule_does_not_fire_when_still_but_far_from_the_demo_end(tmp_path, monkeypatch):
    r, hw, pol, res, _ = _attempt(tmp_path, monkeypatch, lambda p, i: F_DEG, max_s=4.0,
                                  finish_ref_deg=F_DEG + np.array([0, 9.0, 0, 0, 0]))
    assert res["end"] == "time_cap"
    r.disconnect()


def test_finish_rule_does_not_fire_while_the_arm_creeps(tmp_path, monkeypatch):
    """Creeping 2.65 deg/s (the live median) near the demo end: more than 2 deg in any 2 s window."""
    r, hw, pol, res, _ = _attempt(tmp_path, monkeypatch, lambda p, i: F_DEG + np.array([0, 2.65 * i / 30, 0, 0, 0]),
                                  max_s=3.0, finish_ref_deg=F_DEG)
    assert res["end"] == "time_cap"
    r.disconnect()


def test_no_finish_before_2_s_even_when_still(tmp_path, monkeypatch):
    r, hw, pol, res, _ = _attempt(tmp_path, monkeypatch, lambda p, i: F_DEG, max_s=1.9, finish_ref_deg=F_DEG)
    assert res["end"] == "time_cap"
    r.disconnect()


def test_finish_rule_on_the_three_live_attempts():
    """Replay of the logged 2026-11-20 03:01Z live attempts: the rule fires at 13.7 s / 12.4 s and never
    on the third (its end is 18 deg from the demo end)."""
    import json
    log = ROBOT / "out" / "rollout" / "policy_20261120T030055Z.jsonl"
    if not log.exists():
        pytest.skip("live trace not present")
    att = []
    for line in log.read_text().splitlines():
        d = json.loads(line)
        if "attempt" in d:
            att.append([])
        else:
            att[-1].append((d["t"], np.array(d["present"])))
    a = ph.PolicyAttempt(None, None, finish_ref_deg=np.array(ph.DEMO_END_LEFT_TICKS) * tl.DEG_PER_TICK)
    fired = []
    for rows in att:
        hist, hit = [], None
        for t, p in rows:
            hist.append((t, p))
            if t >= 2.0 and a._finished(hist, t, p):
                hit = round(t, 1); break
        fired.append(hit)
    assert fired == [13.7, 12.4, None]
