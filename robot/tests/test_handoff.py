"""AA-23 / AA-44 demo start: the cycle replay stops at the handoff frame holding, with the tube
modelled, and the handoff helpers (state file, servo checks, catch-up, mesh gate).
Simulated servos only (scripts/replay_sim.py)."""
from __future__ import annotations

import importlib.util
import json
import logging
import pathlib

import numpy as np
import pytest

from inchworm import cycle_replay as cr
from inchworm import handoff as ho
from inchworm.config import load_config

ROOT = pathlib.Path(__file__).parents[1]
_s = importlib.util.spec_from_file_location("replay_sim", ROOT / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
STOP = 42          # owner 2026-11-15


def _replay_module():
    s = importlib.util.spec_from_file_location("replay_cycle", ROOT / "scripts" / "replay_cycle.py")
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m


def _write_timeline(path, rows):
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    return path


def _row(left, right, ll=True, rl=True, lift=0.0):
    M = np.eye(4); M[2, 3] = lift
    return {"step": "s", "left": left, "right": right, "left_locked": ll, "right_locked": rl,
            "tube_motion": M.tolist(), "tube_base": np.eye(4).tolist(), "tube_advance_m": 0.0}


def test_timeline_keeps_the_tube_pose(tmp_path):
    z = [0.0] * 5
    fr = cr.load_timeline(_write_timeline(tmp_path / "t.jsonl", [_row(z, z), _row(z, z, lift=0.1)]))
    assert fr[0].tube is not None and fr[1].tube[2, 3] == pytest.approx(0.1)
    mid = cr.tube_at(fr, 0.5)
    assert mid[2, 3] == pytest.approx(0.05) and np.allclose(mid[:3, :3], np.eye(3))
    clamped, _ = cr.clamp_to_eeprom(load_config(), fr)
    assert clamped[1].tube is not None                 # clamping keeps the tube


def test_tube_preflight_refuses_a_timeline_without_tube_pose(tmp_path):
    z = [0.0] * 5
    rows = [{"step": "s", "left": z, "right": z, "left_locked": True, "right_locked": True}] * 2
    fr = cr.load_timeline(_write_timeline(tmp_path / "t.jsonl", rows))

    class Ok:
        def check(self, *a, **k):
            return []
    assert any("no tube pose" in p for p in cr.preflight(load_config(), fr, Ok(), tube=True))


def test_tube_states_reach_the_checker(tmp_path):
    z = [0.0] * 5
    fr = cr.load_timeline(_write_timeline(tmp_path / "t.jsonl", [_row(z, z), _row(z, z, lift=0.1)]))
    seen = []

    class Rec:
        def check(self, st, arm=None, first_only=True):
            seen.append(st)
            return []
    cr.preflight(load_config(), fr, Rec(), samples=4, tube=True)
    assert seen and all(s.tube_advance_m == 0.0 and s.tube_transform is not None for s in seen)
    assert max(np.asarray(s.tube_transform)[2, 3] for s in seen) == pytest.approx(0.1)
    seen.clear()
    cr.preflight(load_config(), fr, Rec(), samples=4)
    assert seen and all(s.tube_advance_m is None for s in seen)     # default: no tube


@pytest.mark.skipif(not TIMELINE.exists(), reason="needs out/inchworm/timeline.jsonl (scripts/inchworm_animation.py)")
def test_stop_frame_42_is_the_left_handoff():
    m = _replay_module()
    frames = cr.load_timeline(TIMELINE)
    assert m.stop_frame_problems(frames, STOP) == []
    f = frames[STOP]
    assert f.locked == {"left": False, "right": True}
    assert f.locked == frames[STOP - 1].locked                     # inside the move, not at a gripper action
    assert m.stop_frame_problems(frames, 0) and m.stop_frame_problems(frames, len(frames) - 1)
    assert m.stop_frame_problems(frames, 10)                       # both locked there: refused


def test_cli_refuses_tube_without_stop_and_tube_step_by_step(capsys):
    m = _replay_module()
    assert m.main(["--tube-in-grippers"]) == 2
    assert m.main(["--tube-in-grippers", "--stop-after-frame", "42", "--step-by-step"]) == 2


@pytest.mark.skipif(not TIMELINE.exists(), reason="needs out/inchworm/timeline.jsonl")
def test_simulated_stop_leaves_both_arms_holding_right_locked(tmp_path):
    logging.disable(logging.WARNING)
    try:
        cfg = load_config()
        frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
        frames = frames[:STOP + 1]
        segs = cr.build_segments(frames)
        r, buses = sim.simulate(cfg, frames, segments=segs, end_press=False)
    finally:
        logging.disable(logging.NOTSET)
    from inchworm.gripper_lock import GripperState
    assert r.arms["right"].sm.state == GripperState.LOCKED
    assert r.arms["left"].sm.state != GripperState.LOCKED
    for a in cr.ARMS:
        assert all(buses[a].torque[s] for s in range(1, 6))         # both arms hold
        goal = cr.q_to_ticks(cfg, a, frames[-1].q[a])
        assert all(abs(buses[a].pos[s + 1] - g) <= cr.REACH_TOL_TICKS for s, g in enumerate(goal))
    # the state file written from it is accepted by the handoff checks
    a = type("A", (), {})()
    a.handoff_state, a.timeline, a.tube_in_grippers = tmp_path / "h.json", TIMELINE, True
    m = _replay_module()
    rec = m.handoff_record(cfg, r, frames, a)
    st = ho.load_state(tmp_path / "h.json")
    assert rec["stop_frame"] == STOP and st["locked"] == {"left": False, "right": True}
    assert ho.state_problems(st, "left") == []
    for x in cr.ARMS:
        assert ho.arm_problems(x, buses[x]._read_bytes, st) == []
    # release with the tube never touches the grippers
    r.release(grippers=False)
    assert buses["right"].torque[6] and not any(buses[x].torque[s] for x in cr.ARMS for s in range(1, 6))


def _state(tmp_path, **kw):
    base = dict(stop_frame=42, step="s", timeline_digest="x", tube_in_grippers=True,
                goal_ticks={"left": [2000] * 5, "right": [2100] * 5},
                present_ticks={"left": [2000] * 5, "right": [2100] * 5},
                gripper_ticks={"left": 1558, "right": 2041}, locked={"left": False, "right": True},
                q_rad={"left": [0.0] * 5, "right": [0.0] * 5}, tube_pose=np.eye(4))
    base.update(kw)
    ho.write_state(tmp_path / "h.json", **base)
    return ho.load_state(tmp_path / "h.json")


def test_state_file_refusals(tmp_path):
    with pytest.raises(ho.HandoffRefused, match="no handoff state"):
        ho.load_state(tmp_path / "missing.json")
    with pytest.raises(ho.HandoffRefused, match="no tube pose"):
        _state(tmp_path, tube_pose=None)
    st = _state(tmp_path, locked={"left": True, "right": True})
    assert any("LOCKED" in p for p in ho.state_problems(st, "left"))
    st = _state(tmp_path, locked={"left": False, "right": False})
    assert any("not locked" in p for p in ho.state_problems(st, "left"))


def test_arm_problems_torque_off_and_moved(tmp_path):
    st = _state(tmp_path)
    clock = sim.SimClock()
    b = sim.SimArmBus("arm_left", [2000] * 5 + [1558], clock)
    assert any("torque OFF" in p for p in ho.arm_problems("left", b._read_bytes, st))
    for s in range(1, 6):
        b.set_torque_enable(s, True)
    assert ho.arm_problems("left", b._read_bytes, st) == []
    b.pos[3] = 2000 + ho.POSE_TOL_TICKS + 1
    assert any("elbow_flex" in p for p in ho.arm_problems("left", b._read_bytes, st))


def test_catch_up_is_rate_limited_and_finishes():
    goal, target, step = [0.0, 0.0], [100.0, -5.0], 10.0
    n = 0
    done = False
    while not done:
        new, done = ho.catch_up_target(goal, target, step)
        assert all(abs(a - b) <= step + 1e-9 for a, b in zip(new, goal))
        goal, n = new, n + 1
    assert n == 10 and goal == pytest.approx(target)


def test_collision_gate_checks_with_tube_and_fails_closed():
    cfg = load_config()

    class Hit:
        def __init__(self):
            self.states = []

        def check(self, st, arm=None):
            self.states.append(st)
            return ["left_wrist vs tube_1"] if st.left_q[1] > 0.5 else []
    h = Hit()
    T = np.eye(4); T[2, 3] = 0.17
    g = ho.CollisionGate(cfg, "left", [0.1] * 5, T, checker=h)
    cal = cfg.motion_guard.servo_calibration.left_arm
    z = [round(getattr(cal, f"{j}_zero_offset_deg") * 4096 / 360) for j in ho.J5]
    assert g.pose_problem(z) is None
    st = h.states[-1]
    assert st.tube_advance_m == 0.0 and np.asarray(st.tube_transform)[2, 3] == pytest.approx(0.17)
    assert np.allclose(st.right_q, 0.1)
    far = list(z); far[1] += cal.shoulder_lift_sign * 0.8 * 4096 / (2 * np.pi)
    assert "mesh collision" in g.step_problem(z, far)      # caught on the way, not only at the end
    assert len(h.states) > 3

    class Boom:
        def check(self, *a, **k):
            raise RuntimeError("model error")
    assert "fail closed" in ho.CollisionGate(cfg, "left", [0.0] * 5, T, checker=Boom()).pose_problem(z)


@pytest.mark.skipif(not TIMELINE.exists(), reason="needs out/inchworm/timeline.jsonl")
def test_real_model_stop_pose_is_clear_with_the_tube():
    """Frame 42 with the tube where the cycle left it: no contact for the left arm (real mesh)."""
    cfg = load_config()
    f = cr.load_timeline(TIMELINE)[STOP]
    g = ho.CollisionGate(cfg, "left", f.q["right"], f.tube)
    assert g.pose_problem(cr.q_to_ticks(cfg, "left", f.q["left"])) is None


def test_joints_off_never_touches_the_gripper():
    clock = sim.SimClock()
    b = sim.SimArmBus("arm_right", [2000] * 6, clock)
    for s in range(1, 7):
        b.set_torque_enable(s, True)
    assert ho.joints_off(b._read_bytes, b._write_bytes) == [1, 2, 3, 4, 5]
    assert b.torque[6] and not any(b.torque[s] for s in range(1, 6))
