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

    def check(self, state, arm=None):
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


def test_lower_arm_may_rest_on_its_own_shoulder_anywhere():
    """Owner 2026-11-13: the lower arm touching its own shoulder is wanted (stability)."""
    from inchworm.system_model.collision import PoseScopedAllowance, CellState
    al = PoseScopedAllowance("always", frozenset({frozenset(("right_lower_arm", "right_shoulder"))}),
                             {"left": [0.0] * 5, "right": [0.0] * 5}, float("inf"))
    drv = {"right_lower_arm": "right", "right_shoulder": "right", "left_wrist": "left"}
    st = CellState(left_q=[0.0] * 5, right_q=[1.0, -1.0, 1.5, 0.5, 0.0])
    assert al.covers("right_shoulder", "right_lower_arm", drv, st)
    assert not al.covers("right_shoulder", "left_wrist", drv, st)       # other pairs still checked
    cfg = load_config()
    pairs = {frozenset(p) for p in cfg.motion_guard.collision.mesh.always_allowed_pairs}
    assert pairs == {frozenset(("left_lower_arm", "left_shoulder")),
                     frozenset(("right_lower_arm", "right_shoulder"))}


# --- fluid motion (owner 2026-11-13: both arms together, no stopping, half animation speed) ---

def test_segments_split_only_at_gripper_actions_and_rest_at_the_ends():
    cfg = load_config()
    plan = _short_plan(cfg)
    segs = cr.build_segments(plan, tempo=0.5)
    assert [(s.first, s.last, s.before) for s in segs] == [
        (0, 1, []), (1, 3, [("left", "unlock")]), (3, 4, [("left", "lock")])]
    s = segs[0]
    assert s.duration_s == pytest.approx(2 * cr.ANIM_FRAME_S / 0.5)
    for a in cr.ARMS:                       # passes through the frames, starts/ends on them
        assert np.allclose(s.q(0.0)[a], plan[0].q[a]) and np.allclose(s.q(s.duration_s)[a], plan[1].q[a])
        e = 1e-3                            # at rest at both ends (eased in and out)
        assert np.abs(s.q(e)[a] - s.q(0)[a]).max() / e < 0.05
        assert np.abs(s.q(s.duration_s)[a] - s.q(s.duration_s - e)[a]).max() / e < 0.05
    assert segs[2].duration_s == 0.0        # relock only, no motion
    with pytest.raises(ValueError):
        cr.build_segments(plan, tempo=1.5)


def test_full_speed_is_twice_half_speed():
    cfg = load_config()
    plan = _short_plan(cfg)
    assert cr.peak_speed_ticks_s(cr.build_segments(plan, 1.0)) == pytest.approx(
        2 * cr.peak_speed_ticks_s(cr.build_segments(plan, 0.5)), rel=0.02)


class _HitAt:
    def __init__(self, n):
        self.n, self.k = n, 0

    def check(self, state, arm=None):
        self.k += 1
        return ["left_wrist vs right_wrist"] if self.k == self.n else []


def test_stream_preflight_refuses_any_contact_and_speed():
    cfg = load_config()
    segs = cr.build_segments(_short_plan(cfg), 0.5)
    assert cr.stream_preflight(cfg, segs, _Checker()) == []
    p = cr.stream_preflight(cfg, segs, _HitAt(7))
    assert len(p) == 1 and "left_wrist vs right_wrist" in p[0]
    assert cr.stream_preflight(cfg, segs, None) == ["mesh collision model unavailable"]
    fast = cr.build_segments(_short_plan(cfg), 1.0)
    old = cr.MAX_STREAM_SPEED_TICKS_S
    try:
        cr.MAX_STREAM_SPEED_TICKS_S = cr.peak_speed_ticks_s(fast) - 1
        assert any("ceiling" in x for x in cr.stream_preflight(cfg, fast, _Checker()))
    finally:
        cr.MAX_STREAM_SPEED_TICKS_S = old


def test_fluid_simulation_never_stops_between_frames():
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        recs = []
        r, buses = sim.simulate(cfg, plan, log=recs.append, segments=cr.build_segments(plan, 0.5))
    finally:
        logging.disable(logging.NOTSET)
    ev = [(x["event"], x.get("arm")) for x in recs if x["event"] not in ("stream", "frame")]
    assert ev == [("gripper_lock", "left"), ("gripper_lock", "right"), ("segment", None),
                  ("gripper_unlock", "left"), ("segment", None), ("gripper_lock", "left")]
    segs = [x for x in recs if x["event"] == "segment"]
    assert all(max(x["worst_path_error_ticks"].values()) < 40 for x in segs)
    assert all(x["took_s"] < x["planned_s"] + 1.0 for x in segs)        # no per-frame stops


def test_fluid_run_never_aborts_on_readings_only_on_the_owner():
    """Owner 2026-11-13: after the confirmed path, readings are logged, never acted on."""
    from inchworm.servo_bus import JointState
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        lift = cr.q_to_ticks(cfg, "left", plan[1].q["left"])[1]
        start = cr.q_to_ticks(cfg, "left", plan[0].q["left"])[1]
        segs = cr.build_segments(plan, 0.5)
        clock = sim.SimClock()
        buses = {}
        for a in cr.ARMS:
            buses[a] = sim.SimArmBus(f"arm_{a}", cr.q_to_ticks(cfg, a, plan[0].q[a]) + [1600], clock,
                                     stall={2: (start + lift) // 2} if a == "left" else None)
        hot = buses["right"].read_joint_state
        buses["right"].read_joint_state = lambda sid: JointState(sid, hot(sid).position_deg, 0, 40, 12.2, 84, 2)
        recs = []
        r = cr.build_runner(cfg, plan, buses, live=False, clock=clock, sleep=clock.sleep, log=recs.append,
                            abort_on_faults=False)
        r.energise(); r.lock_both(); r.move_to_start(); r.run_fluid(segs)      # no exception
        assert [x["event"] for x in recs].count("segment") == 2
        assert any(x["event"] == "reading_warning" and "84 C" in x["what"] for x in recs)
        stop = {"n": 0}

        def flag():
            stop["n"] += 1
            return stop["n"] > 5
        r2 = cr.build_runner(cfg, plan, buses, live=False, clock=clock, sleep=clock.sleep,
                             abort_on_faults=False, should_abort=flag)
        with pytest.raises(cr.ReplayAborted, match="stop requested"):
            r2._stream_segment(segs[0])                                        # the owner still stops it
    finally:
        logging.disable(logging.NOTSET)


def test_step_by_step_mode_still_aborts_on_obstruction():
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        lift = cr.q_to_ticks(cfg, "left", plan[1].q["left"])[1]
        start = cr.q_to_ticks(cfg, "left", plan[0].q["left"])[1]
        with pytest.raises(cr.ReplayAborted, match="stopped short"):
            sim.simulate(cfg, plan, stall={"left": {2: (start + lift) // 2}})
    finally:
        logging.disable(logging.NOTSET)


def test_preflighted_stream_uses_one_sync_write_and_never_the_gripper():
    import struct
    from inchworm.servo_bus import ArmServoBus, GripperWriteNotAllowed

    class T:
        def __init__(self):
            self.out = []

        def write(self, b):
            self.out.append(bytes(b))

        def read(self, n):
            return b""
    t = T()
    b = ArmServoBus(t, role="arm_left", joint_id_map={n: i + 1 for i, n in enumerate((*cr.JOINTS, "gripper"))})
    b.sync_write_goal_ticks({"shoulder_pan": 2000, "elbow_flex": 3000})
    pkt = t.out[-1]
    assert pkt[:3] == b"\xff\xff\xfe" and pkt[4] == 0x83 and pkt[5:7] == bytes([0x2A, 2])
    assert pkt[7:10] == bytes([1]) + struct.pack("<H", 2000) and pkt[10:13] == bytes([3]) + struct.pack("<H", 3000)
    assert (~sum(pkt[2:-1])) & 0xFF == pkt[-1]
    b.sync_write_goal_ticks({"shoulder_pan": 2000}, {"shoulder_pan": 250})
    assert t.out[-1][5:7] == bytes([0x2A, 6]) and t.out[-1][7:14] == bytes([1]) + struct.pack("<HHH", 2000, 0, 250)
    with pytest.raises(Exception):
        b.sync_write_goal_ticks({"shoulder_pan": 2000}, {"shoulder_pan": 0})   # 0 = full speed: refused
    with pytest.raises(GripperWriteNotAllowed):
        b.sync_write_goal_ticks({"gripper": 2000})


def test_stream_point_is_pose_checked_and_must_be_near_the_present_pose():
    from inchworm.motion_guard import MotionViolationError, build_guarded_arm
    cfg = load_config()
    plan = _short_plan(cfg)
    clock = sim.SimClock()
    bus = sim.SimArmBus("arm_left", cr.q_to_ticks(cfg, "left", plan[0].q["left"]) + [1600], clock)
    g = build_guarded_arm(bus, cfg, "left", live=False)
    g.guard.mesh.other_arm_pose_source = lambda: plan[0].q["right"]
    q = plan[0].q["left"].copy()
    g.write_stream_point_rad(q + 0.05, 0.2)                  # small step: written
    far = q.copy(); far[1] += 0.3
    with pytest.raises(MotionViolationError, match="present pose"):
        g.write_stream_point_rad(far, 0.2)
    bad = q.copy(); bad[3] = cfg.motion_guard.left_arm_limits.wrist_flex_upper_rad + 0.01
    with pytest.raises(MotionViolationError):
        g.write_stream_point_rad(bad, 0.2)
    with pytest.raises(ValueError):
        g.write_stream_point_rad(q, 1.0)                     # step ceiling


def test_one_garbled_reply_is_retried_once():
    from inchworm.servo_bus import ServoChecksumError
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        plan = _short_plan(cfg)
        clock = sim.SimClock()
        buses = {a: sim.SimArmBus(f"arm_{a}", cr.q_to_ticks(cfg, a, plan[0].q[a]) + [1600], clock) for a in cr.ARMS}
        recs = []
        r = cr.build_runner(cfg, plan, buses, live=False, clock=clock, sleep=clock.sleep, log=recs.append)
        real, n = buses["left"].read_joint_state, {"k": 0}

        def flaky(sid):
            n["k"] += 1
            if n["k"] in (1, 4, 5):
                raise ServoChecksumError(sid, 0xF9, 0xFB)
            return real(sid)
        buses["left"].read_joint_state = flaky
        assert r._read_state("left", 2) is not None              # 1st fails, retried
        assert recs[-1]["event"] == "read_retry"
        r._read_state("left", 2)                                  # call 3 ok
        with pytest.raises(ServoChecksumError):
            r._read_state("left", 2)                              # 4 and its retry 5 fail: abort
    finally:
        logging.disable(logging.NOTSET)


def test_fold_elbow_stays_where_it_was_until_the_owner_decides_the_contact():
    cfg = load_config()
    for a, lim in (("left", 3151), ("right", 3106)):
        assert cfg.motion_guard.__dict__[f"{a}_arm_limits"].servo_tick_limits["elbow_flex"][1] == lim
    path = pathlib.Path(__file__).parents[1] / "out" / "inchworm" / "timeline.jsonl"
    if not path.exists():
        pytest.skip("no planned timeline")
    fr, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(path)[:1], elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    assert [cr.q_to_ticks(cfg, a, fr[0].q[a])[2] for a in cr.ARMS] == [3081, 3036]
