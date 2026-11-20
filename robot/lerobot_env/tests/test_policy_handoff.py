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
