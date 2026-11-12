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


def test_soft_limit_waiver_lifts_only_the_unverified_gate():
    from inchworm.motion_guard import LiveMotionBlockedError, build_joint_limit_guard
    cfg = load_config()
    assert cfg.motion_guard.left_arm_limits.unverified        # premise of the waiver
    g = build_joint_limit_guard(cfg, "left")
    with pytest.raises(LiveMotionBlockedError):
        g.check([0.0, 0.0, 0.0, 0.0, 0.0], live=True)
    w = build_joint_limit_guard(cfg, "left", unverified_live_override="owner test")
    assert w.check([0.0, 0.0, 0.0, 0.0, 0.0], live=True) is None
    hi = cfg.motion_guard.left_arm_limits.wrist_flex_upper_rad
    assert w.check([0.0, 0.0, 0.0, hi + 0.01, 0.0], live=True) is not None   # values still enforced


def test_cli_live_refused_without_owner_waiver(capsys):
    _s2 = importlib.util.spec_from_file_location(
        "replay_cycle", pathlib.Path(__file__).parents[1] / "scripts" / "replay_cycle.py")
    m = importlib.util.module_from_spec(_s2); _s2.loader.exec_module(m)
    assert m.main(["--live"]) == 2
    assert "allow-unverified-soft-limits" in capsys.readouterr().out
    assert m.main(["--speed", "1000"]) == 2


def test_stop_request_aborts_then_hold_and_release():
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        clock = sim.SimClock()
        buses = {a: sim.SimArmBus(a, cr.q_to_ticks(cfg, a, plan[0].q[a]) + [2500], clock) for a in cr.ARMS}
        stop = {"armed": False, "n": 0}

        def should_abort():
            if stop["armed"]:
                stop["n"] += 1
            return stop["n"] > 5           # a few polls into the first move

        r = cr.build_runner(cfg, plan, buses, live=True, soft_limits_override="test",
                            clock=clock, sleep=clock.sleep, should_abort=should_abort)
        r.energise(); r.lock_both()
        stop["armed"] = True
        with pytest.raises(cr.ReplayAborted, match="stop requested"):
            r.run()
        assert all(v is None for v in r.hold().values())
        assert all(b.torque[s] for b in buses.values() for s in range(1, 6))       # held
        assert not any(v for v in r.release().values())
        assert not any(b.torque[s] for b in buses.values() for s in range(1, 7))   # released
    finally:
        logging.disable(logging.NOTSET)


class _Checker:
    def __init__(self, hit=False):
        self.hit = hit

    def check(self, state, arm):
        return ["left_wrist vs chassis"] if self.hit else []


def test_approach_adjusts_when_close_refuses_when_far_or_colliding():
    cfg = load_config()
    f0 = _short_plan(cfg)[0]
    near = {a: f0.q[a] + np.radians([0.5, -0.3, 3.5, 0.0, -2.7]) for a in cr.ARMS}   # the measured fold offsets
    assert cr.approach_problems(cfg, near, f0, _Checker()) == []
    far = {a: f0.q[a] + np.radians([0, 0, 12.0, 0, 0]) for a in cr.ARMS}
    p = cr.approach_problems(cfg, far, f0, _Checker())
    assert p and "elbow_flex" in p[0] and "closer by hand" in p[0]
    assert cr.approach_problems(cfg, near, f0, _Checker(hit=True))
    assert cr.approach_problems(cfg, near, f0, None) == ["mesh collision model unavailable"]
    assert cr.approach_problems(cfg, {"left": near["left"]}, f0, _Checker())       # missing arm


def test_real_fold_is_close_enough_to_adjust():
    """The last measured fold (2026-11-11) passes the approach check on the real timeline."""
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.so101_pose_reader import ticks_to_q
    cfg = load_config()
    path = pathlib.Path(__file__).parents[1] / "out" / "inchworm" / "timeline.jsonl"
    if not path.exists():
        pytest.skip("no planned timeline")
    fr, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(path)[:1])
    ticks = {"left": [1978, 1423, 3121, 1003, 2031], "right": [2072, 1493, 3075, 968, 2023]}
    m = {a: ticks_to_q(cfg, a, ticks[a]) for a in cr.ARMS}
    assert cr.approach_problems(cfg, m, fr[0], build_mesh_collision_guard(cfg, "left").checker) == []
