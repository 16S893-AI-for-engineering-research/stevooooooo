"""AA-54 (owner plan 2026-11-21): per-arm recording from the frozen training positions.

The holding-arm tube move (follower.move_holding), the schedule-driven recording session, the
refusals, and the tube placement at the training position (simulated servos)."""
from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import numpy as np
import pytest

ROBOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import test_inchworm_plugin as tp_  # noqa: E402  (shared fakes: sim buses, leader, handoff robot)

_rt = importlib.util.spec_from_file_location("record_train", ROBOT / "lerobot_env" / "record_train.py")
rt = importlib.util.module_from_spec(_rt); _rt.loader.exec_module(rt)
_pt = importlib.util.spec_from_file_location("place_training_tube", ROBOT / "scripts" / "place_training_tube.py")
pt = importlib.util.module_from_spec(_pt); _pt.loader.exec_module(pt)

from inchworm import perturb as pb  # noqa: E402
from inchworm import training_poses as tpo  # noqa: E402

sim = tp_.sim


class HoldModel(tp_.FreeModel):
    """Return/move model stand-in that reports the current holding home (like CarriedTubeModel)."""


def _robot(tmp_path, monkeypatch, clock, answers, hit=None):
    hw = tp_.ReturnHW(clock)
    r, g = tp_._handoff_robot(tmp_path, monkeypatch, hw, answers=answers)
    g.other_q, g.tube = None, None

    def model(cfg):
        h = r._handoff
        home = {"left": tp_.F_TICKS, "right": list(h.get("hold_ticks") or tp_.OTHER_TICKS)}
        return HoldModel(home, hit)
    r._make_return_model = model
    r._make_chain = lambda cfg: tp_.LinChain()
    return r, hw, g


def _ep(i, nominal, ticks):
    return pb.Episode(i, nominal, list(ticks), np.eye(4).tolist(), achieved={"rot_deg": 0.0 if nominal else 1.0,
                      "trans_mm": [0.0, 0.0, 0.0] if nominal else [5.0, -3.0, 2.0]})


def test_move_holding_moves_only_the_holding_arm_and_updates_the_gate(tmp_path, monkeypatch):
    clock = sim.SimClock()
    r, hw, g = _robot(tmp_path, monkeypatch, clock, answers=("YES", "YES", ""))
    r.connect()
    tgt = list(tp_.OTHER_TICKS); tgt[1] += 30; tgt[2] -= 20
    grips = (hw.bus.goal[6], hw.bus.torque[6], hw.other_bus.goal[6], hw.other_bus.torque[6])
    fgoal = {s: hw.bus.goal[s] for s in range(1, 6)}
    T = np.eye(4); T[0, 3] = 0.01
    r.move_holding(tgt, T, confirm=True, clock=clock, sleep=clock.sleep)
    assert max(abs(hw.other_bus.pos[i + 1] - v) for i, v in enumerate(tgt)) <= 10
    assert {s: hw.bus.goal[s] for s in range(1, 6)} == fgoal                 # the inserting arm held still
    assert (hw.bus.goal[6], hw.bus.torque[6], hw.other_bus.goal[6], hw.other_bus.torque[6]) == grips
    assert r._handoff["hold_ticks"] == tgt and np.allclose(g.tube, T) and r._handoff["catching_up"]
    # the next return uses the perturbed holding pose as the holding arm's home (no move back)
    rep = r.handoff_return(confirm=False, clock=clock, sleep=clock.sleep)
    assert all(not l["leg"].startswith("right") for l in rep["legs"])
    r.disconnect()


@pytest.mark.parametrize("answers, hit, needle", [
    (("YES", "no", ""), None, "not confirmed"),
    (("YES", "YES", ""), "mesh collision: right_gripper intersects baseforreal", "mesh collision"),
])
def test_move_holding_refusal_moves_nothing(tmp_path, monkeypatch, answers, hit, needle):
    from inchworm.handoff_return import ReturnRefused
    clock = sim.SimClock()
    r, hw, _ = _robot(tmp_path, monkeypatch, clock, answers=answers, hit=hit)
    r.connect()
    before = (dict(hw.bus.goal), dict(hw.other_bus.goal))
    tgt = list(tp_.OTHER_TICKS); tgt[1] += 30
    with pytest.raises(ReturnRefused, match=needle):
        r.move_holding(tgt, np.eye(4), confirm=True, clock=clock, sleep=clock.sleep)
    assert (dict(hw.bus.goal), dict(hw.other_bus.goal)) == before and r._handoff.get("hold_ticks") is None
    r.disconnect()


def test_move_holding_refuses_a_drifted_holding_arm(tmp_path, monkeypatch):
    from inchworm.handoff_return import ReturnRefused
    clock = sim.SimClock()
    r, hw, _ = _robot(tmp_path, monkeypatch, clock, answers=("YES", ""))
    r.connect()
    hw.other_bus.pos[2] += 80; hw.other_bus.goal[2] += 80                  # ~7 deg: the tube was moved
    with pytest.raises(ReturnRefused, match="probably moved"):
        r.move_holding(tp_.OTHER_TICKS, np.eye(4), confirm=False, clock=clock, sleep=clock.sleep)
    r.disconnect()


def test_train_session_follows_the_schedule_and_repeats_a_discarded_placement(tmp_path, monkeypatch):
    clock = sim.SimClock()
    r, hw, _ = _robot(tmp_path, monkeypatch, clock, answers=("YES", "YES", "YES", ""))
    r.connect()
    t, lead = tp_._leader(tmp_path, tp_._leader_ticks_for(tp_.F_TICKS)); t.connect()
    ds = tp_.rh.make_dataset(r, tmp_path / "ds", "local/train_test", 30)
    a = list(tp_.OTHER_TICKS); a[1] += 25
    sched = [_ep(0, False, a), _ep(1, True, tp_.OTHER_TICKS)]
    out = tmp_path / "ds" / "perturbations.json"
    s = rt.TrainSession(r, t, ds, "insert", sched, 30,
                        tp_.Keys(["r", 30, "d", 3, "r", 30, "s", 3, "r", 30, "s", 3]), out=out,
                        clock=clock, sleep=clock.sleep, say=lambda *_: None, max_s=2.0)
    s.run()
    assert (s.saved, s.discarded) == (2, 1)
    takes = json.loads(out.read_text())["takes"]
    assert [(x["schedule_index"], x["result"]) for x in takes] == [(0, "discard"), (0, "save"), (1, "save")]
    assert [x["dataset_episode"] for x in takes] == [None, 0, 1]
    assert max(abs(hw.other_bus.pos[i + 1] - v) for i, v in enumerate(tp_.OTHER_TICKS)) <= 10
    assert getattr(r, "holding_moves_done", 0) == 2                       # to episode 0, then back to nominal
    r.disconnect(); t.disconnect()


def _state(pos, **over):
    st = {"stop_frame": pos["timeline_frame"], "tube_in_grippers": True,
          "locked": {pos["inserting_arm"]: False, pos["holding_arm"]: True}, "goal_ticks": pos["joint_ticks"]}
    st.update(over)
    return st


def test_preflight_refusals(monkeypatch, tmp_path):
    cfg = tp_.CFG
    pos = tpo.for_arm("left")
    monkeypatch.setattr(tpo, "check_against_timeline", lambda *a, **k: [])
    assert rt.preflight(cfg, "left", _state(pos)) == pos
    with pytest.raises(rt.Refused, match="handoff state"):
        rt.preflight(cfg, "left", _state(pos, tube_in_grippers=False))
    with pytest.raises(rt.Refused, match="handoff state"):
        rt.preflight(cfg, "left", _state(pos, locked={"left": True, "right": True}))
    monkeypatch.setattr(rt, "leader_map_path", lambda arm: tmp_path / f"leader_map_{arm}.json")
    with pytest.raises(rt.Refused, match="no leader map"):
        rt.preflight(cfg, "left", _state(pos))
    monkeypatch.setattr(tpo, "check_against_timeline", lambda *a, **k: ["train_left left: drift"])
    with pytest.raises(rt.Refused, match="no longer match"):
        rt.preflight(cfg, "left", _state(pos))


# --- tube placement at the training position (simulated servos, real GripperLockSM) ----------
def _place_runner(pos):
    from inchworm import cycle_replay as cr
    cfg = tp_.CFG
    lock = {"left": 2026, "right": 2029}
    st = {"present_ticks": pos["joint_ticks"],
          "gripper_ticks": {pos["inserting_arm"]: 1550, pos["holding_arm"]: lock[pos["holding_arm"]]}}
    buses, clock = pt.sim_buses(cfg, st)
    f = cr.Frame(pos["timeline_frame"], "t", {a: np.zeros(5) for a in cr.ARMS},
                 {pos["inserting_arm"]: False, pos["holding_arm"]: True})
    r = cr.build_runner(cfg, [f], buses, live=False, clock=clock, sleep=clock.sleep)
    return r, buses


@pytest.mark.parametrize("arm", ["left", "right"])
def test_place_locks_only_the_holding_gripper_never_both_open_with_tube(arm):
    from inchworm.gripper_lock import GripperState
    pos = tpo.for_arm(arm)
    r, buses = _place_runner(pos)
    asked = []
    ans = iter(["YES", "YES", "y"])

    def ask(m):
        asked.append(m); return next(ans)
    ins_goal = buses[arm].goal[6]
    pt.place(tp_.CFG, r, arm, pos, ask, say=lambda *_: None)
    assert r.arms[pos["holding_arm"]].sm.state == GripperState.LOCKED
    assert r.arms[arm].sm.state == GripperState.UNLOCKED and buses[arm].goal[6] == ins_goal   # never commanded
    assert "NOT attached" in asked[0]          # both grippers open only after the owner says no tube is attached


@pytest.mark.parametrize("answers, needle", [(["no"], "tube state not confirmed"),
                                              (["YES", "no"], "nothing locked"),
                                              (["YES", "YES", "n"], "not in and locked")])
def test_place_stops_without_confirmation(answers, needle):
    pos = tpo.for_arm("left")
    r, buses = _place_runner(pos)
    it = iter(answers)
    with pytest.raises(pt.Stop, match=needle):
        pt.place(tp_.CFG, r, "left", pos, lambda m: next(it), say=lambda *_: None)


def test_place_refuses_off_the_frozen_pose():
    pos = tpo.for_arm("right")
    r, buses = _place_runner(pos)
    buses["right"].pos[2] += 60
    with pytest.raises(pt.Stop, match="not at the frozen training position"):
        pt.place(tp_.CFG, r, "right", pos, lambda m: "YES", say=lambda *_: None)
