"""Cycle replay (owner 2026-11-12: full movement test, no tube, grippers moving). No hardware."""
import json
import logging
import math

import numpy as np
import pytest

from inchworm import cycle_replay as cr
import importlib.util, pathlib
_s = importlib.util.spec_from_file_location("replay_sim", pathlib.Path(__file__).parents[1] / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)
from inchworm.config import load_config


def _write(tmp_path, frames):
    p = tmp_path / "t.jsonl"
    p.write_text("\n".join(json.dumps(f) for f in frames))
    return p


def _f(q, ll=True, rl=True, step="s"):
    return {"step": step, "left": q, "right": q, "left_locked": ll, "right_locked": rl}


Q0 = [0.0, -1.5, 1.5, -1.5, 0.0]


def test_load_refuses_both_unlocked_and_unlocked_start(tmp_path):
    with pytest.raises(ValueError):
        cr.load_timeline(_write(tmp_path, [_f(Q0), _f(Q0, False, False)]))
    with pytest.raises(ValueError):
        cr.load_timeline(_write(tmp_path, [_f(Q0, False, True)]))
    with pytest.raises(ValueError):
        cr.load_timeline(_write(tmp_path, [_f([0.0] * 4)]))


def test_gripper_events_follow_the_plan(tmp_path):
    fr = cr.load_timeline(_write(tmp_path, [_f(Q0), _f(Q0, False, True), _f(Q0), _f(Q0, True, False), _f(Q0)]))
    assert cr.gripper_events(fr) == {1: [("left", "unlock")], 2: [("left", "lock")],
                                     3: [("right", "unlock")], 4: [("right", "lock")]}


def test_frame_speeds_arrive_together_and_never_zero():
    a = np.zeros(5); b = np.array([0.1, 0.05, 0.0, -0.1, 0.0])
    sp, t = cr.frame_speeds(a, b, 150)
    assert max(sp) == 150 and min(sp) == cr.MIN_SPEED_TICKS_S
    assert t == pytest.approx(0.1 * cr.TICKS_PER_RAD / 150)


def test_clamp_pulls_the_fold_elbow_inside_the_eeprom_limit():
    cfg = load_config()
    lim = cfg.motion_guard.left_arm_limits.servo_tick_limits["elbow_flex"]
    cal = cfg.motion_guard.servo_calibration.left_arm
    q = list(Q0)
    q[2] = math.radians((lim[1] + 30) * 360 / 4096 - cal.elbow_flex_zero_offset_deg)
    fr = [cr.Frame(0, "s", {"left": np.array(q), "right": np.array(Q0)}, {"left": True, "right": True})]
    out, notes = cr.clamp_to_eeprom(cfg, fr)
    assert cr.q_to_ticks(cfg, "left", out[0].q["left"])[2] == lim[1] - cr.EEPROM_INSIDE_TICKS
    assert len(notes) == 1 and "elbow_flex" in notes[0]


def _short_plan(cfg):
    """Start pose of the real timeline plus two small moves and one regrip of the left arm."""
    from inchworm.so101_pose_reader import ticks_to_q
    q = {a: np.array(ticks_to_q(cfg, a, [2048, 1500, 2800, 1100, 2048])) for a in cr.ARMS}
    q2 = {a: v + np.array([0, 0.1, -0.1, 0.05, 0]) for a, v in q.items()}
    F = cr.Frame
    return [F(0, "start", q, {"left": True, "right": True}),
            F(1, "move", q2, {"left": True, "right": True}),
            F(2, "left regrip", q2, {"left": False, "right": True}),
            F(3, "left regrip", q, {"left": False, "right": True}),
            F(4, "relock", q, {"left": True, "right": True})]


def test_simulated_replay_runs_the_sequence_slowly(caplog):
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        recs = []
        r, buses = sim.simulate(cfg, _short_plan(cfg), log=recs.append)
    finally:
        logging.disable(logging.NOTSET)
    ev = [(x["event"], x.get("arm")) for x in recs if x["event"] != "frame"]
    assert ev == [("gripper_lock", "left"), ("gripper_lock", "right"),
                  ("gripper_unlock", "left"), ("gripper_lock", "left")]
    assert all(max(abs(e) for e in x["error_ticks"][a]) <= cr.REACH_TOL_TICKS
               for x in recs if x["event"] == "frame" for a in x["error_ticks"])
    for b in buses.values():                       # never full speed (Goal_Speed 0)
        assert 0 < max(b.max_speed_seen.values()) <= 150


def test_obstruction_aborts():
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        lift = cr.q_to_ticks(cfg, "left", plan[1].q["left"])[1]
        start = cr.q_to_ticks(cfg, "left", plan[0].q["left"])[1]
        with pytest.raises(cr.ReplayAborted):
            sim.simulate(cfg, plan, stall={"left": {2: (start + lift) // 2}})
    finally:
        logging.disable(logging.NOTSET)
