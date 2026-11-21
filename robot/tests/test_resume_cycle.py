"""AA-39 full test pieces in the main env: resumed timeline frames, gripper adoption by read-back,
segments of a resumed piece, adopt_holding (simulated servos, no hardware)."""
from __future__ import annotations

import importlib.util
import pathlib

import numpy as np
import pytest

from inchworm import cycle_replay as cr
from inchworm.config import load_config
from inchworm.gripper_lock import GripperLockError, GripperLockSM, GripperState, UnlockRefused

ROOT = pathlib.Path(__file__).parents[1]
_s = importlib.util.spec_from_file_location("replay_sim", ROOT / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
CFG = load_config()
QD = np.array([0.008, -0.384, -0.232, -0.97, -0.017])          # ~ the demo-end pose (rad)
needs_tl = pytest.mark.skipif(not TIMELINE.exists(), reason="no timeline")


def _frames():
    fr, _ = cr.clamp_to_eeprom(CFG, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    return fr


@needs_tl
def test_resume_frames_replace_only_the_held_arm():
    fr = _frames()
    sub = cr.resume_frames(fr, 49, 67, "left", QD)
    assert [f.index for f in sub] == list(range(49, 68))
    assert all(np.allclose(f.q["left"], QD) for f in sub)
    assert all(np.allclose(f.q["right"], fr[f.index].q["right"]) for f in sub)
    assert all(np.allclose(f.tube, fr[f.index].tube) for f in sub)
    assert sub[0].locked == {"left": True, "right": True} and sub[-1].locked == {"left": True, "right": False}


@needs_tl
@pytest.mark.parametrize("start, stop, arm, msg", [
    (42, 67, "left", "not LOCKED"),        # frame 42: the left is open
    (49, 67, "right", "not LOCKED"),        # the right unlocks at 50
    (74, 80, "left", "moves the left"),     # both locked from 74, the left moves in the lowering
    (49, 200, "left", "outside"),
])
def test_resume_frames_refusals(start, stop, arm, msg):
    with pytest.raises(ValueError, match=msg):
        cr.resume_frames(_frames(), start, stop, arm, QD)


@needs_tl
def test_segments_of_a_resumed_piece_keep_timeline_indices():
    sub = cr.resume_frames(_frames(), 49, 67, "left", QD)
    segs = cr.build_segments(sub)
    assert [(s.first, s.last, s.before) for s in segs] == [(49, 49, []), (49, 67, [("right", "unlock")])]
    assert np.allclose(segs[-1].q(segs[-1].duration_s)["right"], sub[-1].q["right"])
    assert segs[-1].s_at(segs[-1].duration_s) == pytest.approx(67)


def _sm(grip_ticks, clock=None):
    clock = clock or sim.SimClock()
    b = sim.SimArmBus("arm_left", [2048] * 5 + [grip_ticks], clock)
    return GripperLockSM(bus=b, role="arm_left", config=CFG.gripper_lock), b


@pytest.mark.parametrize("ticks, state", [(2030, GripperState.LOCKED), (1560, GripperState.UNLOCKED)])
def test_adopt_readback_reads_and_never_writes(ticks, state):
    sm, b = _sm(ticks)
    goal, torque = b.goal[6], b.torque[6]
    assert sm.adopt_readback() == state
    assert (b.goal[6], b.torque[6]) == (goal, torque)


def test_adopt_readback_between_bands_is_a_fault():
    sm, b = _sm(1800)
    with pytest.raises(GripperLockError, match="neither"):
        sm.adopt_readback()
    assert sm.state == GripperState.FAULT


def _runner(left_grip, right_grip):
    clock = sim.SimClock()
    q = {"left": QD, "right": QD}
    fr = [cr.Frame(49, "s", {a: v.copy() for a, v in q.items()}, {"left": True, "right": True})]
    buses = {a: sim.SimArmBus(f"arm_{a}", cr.q_to_ticks(CFG, a, QD) + [g], clock)
             for a, g in (("left", left_grip), ("right", right_grip))}
    for b in buses.values():
        for s in range(1, 7):
            b.set_torque_enable(s, True)
    return cr.build_runner(CFG, fr, buses, live=False, clock=clock, sleep=clock.sleep), buses


def test_adopt_holding_requires_the_gripper_states_and_moves_nothing():
    r, buses = _runner(1560, 2035)
    before = {a: dict(b.goal) for a, b in buses.items()}
    with pytest.raises(cr.ReplayAborted, match="required"):
        r.adopt_holding(require={"left": "LOCKED", "right": "LOCKED"})
    assert {a: dict(b.goal) for a, b in buses.items()} == before       # nothing written
    r, buses = _runner(1560, 2035)
    assert r.adopt_holding(require={"left": "UNLOCKED", "right": "LOCKED"}) == {"left": "UNLOCKED", "right": "LOCKED"}
    assert all(b.torque[s] for b in buses.values() for s in range(1, 7))
    assert all(abs(buses[a].pos[s] - before[a][s]) <= 1 for a in buses for s in range(1, 7))


def test_adopted_runner_never_unlocks_with_the_other_open():
    r, _ = _runner(1560, 2035)
    r.adopt_holding()
    with pytest.raises((UnlockRefused, Exception)):
        r._unlock("right")                                              # left is open
    assert r.arms["right"].sm.state == GripperState.LOCKED
    r._lock("left")
    r._unlock("right")                                                  # now allowed
    assert r.arms["left"].sm.state == GripperState.LOCKED
