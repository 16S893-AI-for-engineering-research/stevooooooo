"""AA-23 rapid-fire recording: the automatic return between demos (owner spec 2026-11-15).
Real cell mesh model + the real timeline; simulated servos (scripts/replay_sim.py)."""
from __future__ import annotations

import importlib.util
import pathlib

import numpy as np
import pytest

from inchworm import cycle_replay as cr
from inchworm import handoff_return as hr
from inchworm import teleop_live as tl
from inchworm.config import load_config

ROOT = pathlib.Path(__file__).parents[1]
_s = importlib.util.spec_from_file_location("replay_sim", ROOT / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
CFG = load_config()
pytestmark = pytest.mark.skipif(not TIMELINE.exists(), reason="no timeline")


@pytest.fixture(scope="module")
def cell():
    from inchworm.kinematics import build_chain, with_soft_limits
    from inchworm.motion_guard import build_mesh_collision_guard
    fr = cr.load_timeline(TIMELINE)
    home = {a: cr.q_to_ticks(CFG, a, fr[42].q[a]) for a in ("left", "right")}
    checker = build_mesh_collision_guard(CFG, "left").checker
    model = hr.CarriedTubeModel(CFG, checker, home, fr[42].tube)
    chain = with_soft_limits(build_chain(CFG.so101_kinematics), CFG.motion_guard.left_arm_limits)
    lim = {}
    for a in ("left", "right"):
        soft = tl.soft_limit_ticks(CFG, a)
        y = getattr(CFG.motion_guard, f"{a}_arm_limits").servo_tick_limits
        lim[a] = {j: (max(soft[j][0], y[j][0] + 10), min(soft[j][1], y[j][1] - 10)) for j in tl.J5}
    return dict(fr=fr, home=home, model=model, chain=chain, lim=lim)


def test_carried_tube_is_the_handoff_tube_at_the_stop_pose_and_moves_with_the_right_gripper(cell):
    m, home = cell["model"], cell["home"]
    assert np.allclose(m.tube(home["right"]), cell["fr"][42].tube, atol=1e-9)
    moved = list(home["right"]); moved[0] += 40                       # pan ~3.5 deg
    assert not np.allclose(m.tube(moved)[:3, 3], m.tube(home["right"])[:3, 3], atol=1e-3)


@pytest.mark.parametrize("frame", [48, 49])                        # inserted (planned insertion end)
def test_plan_from_the_inserted_pose_pulls_straight_out_then_returns(cell, frame):
    fr, m, chain = cell["fr"], cell["model"], cell["chain"]
    now = {"left": cr.q_to_ticks(CFG, "left", fr[frame].q["left"]), "right": [t + 15 for t in cell["home"]["right"]]}
    plan = hr.plan_return(CFG, m, chain, cell["lim"], now)
    out, line, right = plan.legs
    assert out.arm == "left" and abs(out.duration_s - 5.0) < 1e-9        # 50 mm at 10 mm/s
    T = [chain.fk(np.asarray(hr.ticks_to_q(CFG, "left", w))) for w in out.waypoints]
    dz = [(t[2, 3] - T[0][2, 3]) * 1000 for t in T]
    assert abs(dz[-1] + 50.0) < 0.6                                       # 50 mm down (tick rounding aside)
    assert all(b < a + 0.05 for a, b in zip(dz, dz[1:]))                  # monotonic out of the key
    xy = max(np.linalg.norm(t[:2, 3] - T[0][:2, 3]) for t in T) * 1000
    assert xy < 0.5                                                        # straight: no sideways drift
    ang = max(np.degrees(np.arccos(np.clip((np.trace(T[0][:3, :3].T @ t[:3, :3]) - 1) / 2, -1, 1))) for t in T)
    assert ang < 0.6                                                       # orientation held
    assert line.waypoints[-1] == pytest.approx(cell["home"]["left"])
    assert right.arm == "right" and right.waypoints[-1] == pytest.approx(cell["home"]["right"])
    assert plan.checks == sum(len(l.waypoints) - 1 for l in plan.legs)    # every step mesh-checked


def test_retract_stops_at_the_stop_pose_height(cell):
    fr, chain = cell["fr"], cell["chain"]
    now = {"left": cr.q_to_ticks(CFG, "left", fr[45].q["left"]), "right": cell["home"]["right"]}
    out = hr.plan_return(CFG, cell["model"], chain, cell["lim"], now).legs[0]
    z = lambda w: chain.fk(np.asarray(hr.ticks_to_q(CFG, "left", w)))[2, 3] * 1000
    assert abs(z(out.waypoints[-1]) - z(cell["home"]["left"])) < 0.6       # pulled out to the stop height only
    assert 20 < z(out.waypoints[0]) - z(out.waypoints[-1]) < 35            # frame 45: ~25 mm in
    at_home = {"left": cell["home"]["left"], "right": cell["home"]["right"]}
    assert hr.plan_return(CFG, cell["model"], chain, cell["lim"], at_home).legs[0].name == "left to its stop pose"


def test_right_drift_over_5_deg_refuses(cell):
    now = {"left": cell["home"]["left"], "right": list(cell["home"]["right"])}
    now["right"][2] += 60                                                  # 5.3 deg
    with pytest.raises(hr.ReturnRefused, match="drifted"):
        hr.plan_return(CFG, cell["model"], cell["chain"], cell["lim"], now)


def test_a_contact_on_the_path_refuses_before_anything_moves(cell):
    class Hit:
        checks = 0

        def problem(self, f, o):
            self.checks += 1
            return "mesh collision: left_gripper intersects tube_1" if self.checks > 10 else None

    m = cell["model"]
    hit = Hit()
    hit.follower, hit.other, hit.home = m.follower, m.other, m.home
    now = {"left": cr.q_to_ticks(CFG, "left", cell["fr"][48].q["left"]), "right": cell["home"]["right"]}
    with pytest.raises(hr.ReturnRefused, match="Nothing moved"):
        hr.plan_return(CFG, hit, cell["chain"], cell["lim"], now)


class Bus(sim.SimArmBus):
    def _read_bytes(self, sid, reg, n):
        if reg in (16, 21, 28, 48):
            return bytes([100, 0])[:n]
        return super()._read_bytes(sid, reg, n)


def _core(clock, start, stall=None):
    b = Bus("arm_left", list(start) + [1560], clock, stall=stall)
    b._bus.read_bytes = b._read_bytes
    for s in range(1, 6):
        b.set_torque_enable(s, True)
    lim = {j: (0, 4095) for j in tl.J5}
    core = tl.TeleopCore(leader_read=hr._no_leader, follower_bus=b, mapping={}, limits=lim)
    core.adopt_holding()
    return core, b


def test_run_leg_follows_the_plan_at_the_planned_pace(cell):
    clock = sim.SimClock()
    now = {"left": cr.q_to_ticks(CFG, "left", cell["fr"][48].q["left"]), "right": cell["home"]["right"]}
    plan = hr.plan_return(CFG, cell["model"], cell["chain"], cell["lim"], now)
    core, b = _core(clock, now["left"])
    g6 = (b.goal[6], b.torque[6])
    for leg in plan.legs[:2]:
        rep = hr.run_leg(core, leg, clock=clock, sleep=clock.sleep)
        assert rep["worst_track_ticks"] <= hr.TRACK_TOL_TICKS
    assert max(abs(b.pos[i + 1] - t) for i, t in enumerate(cell["home"]["left"])) <= hr.SETTLE_TICKS
    assert max(b.max_speed_seen[s] for s in range(1, 6)) * tl.DEG_PER_TICK < 3 * hr.RETURN_DEG_S
    assert (b.goal[6], b.torque[6]) == g6                                 # gripper never written


def test_run_leg_stops_and_holds_when_something_resists():
    clock = sim.SimClock()
    start = [2000, 2000, 2000, 2000, 2000]
    core, b = _core(clock, start, stall={2: 2030})                        # shoulder_lift blocked
    leg = hr.Leg("left", "test", hr._line(start, [2000, 2300, 2000, 2000, 2000]), 3.0)
    with pytest.raises(hr.ReturnAborted, match="resists"):
        hr.run_leg(core, leg, clock=clock, sleep=clock.sleep)
    assert abs(b.goal[2] - b.pos[2]) <= 2 and all(b.torque[s] for s in range(1, 6))   # holding there


# --- AA-39: the same return with the RIGHT arm as the follower (stop frame 67, left LOCKED) ---------
@pytest.fixture(scope="module")
def right_cell(cell):
    from inchworm.kinematics import build_chain, with_soft_limits
    fr = cell["fr"]
    assert fr[67].locked == {"left": True, "right": False}
    home = {a: cr.q_to_ticks(CFG, a, fr[67].q[a]) for a in ("left", "right")}
    model = hr.CarriedTubeModel(CFG, cell["model"].checker, home, fr[67].tube, follower="right")
    chain = with_soft_limits(build_chain(CFG.so101_kinematics), CFG.motion_guard.right_arm_limits)
    return dict(fr=fr, home=home, model=model, chain=chain, lim=cell["lim"])


def test_right_follower_carried_tube_is_the_handoff_tube(right_cell):
    m, home = right_cell["model"], right_cell["home"]
    assert m.other == "left"
    assert np.allclose(m.tube(home["left"]), right_cell["fr"][67].tube, atol=1e-9)


def test_right_follower_return_from_the_inserted_pose(right_cell):
    fr, m, chain = right_cell["fr"], right_cell["model"], right_cell["chain"]
    now = {"right": cr.q_to_ticks(CFG, "right", fr[73].q["right"]), "left": [t + 15 for t in right_cell["home"]["left"]]}
    plan = hr.plan_return(CFG, m, chain, right_cell["lim"], now)
    out, line, other = plan.legs
    assert out.arm == "right" and abs(out.duration_s - 5.0) < 1e-9
    T = [chain.fk(np.asarray(hr.ticks_to_q(CFG, "right", w))) for w in out.waypoints]
    assert abs((T[-1][2, 3] - T[0][2, 3]) * 1000 + 50.0) < 0.6
    assert line.waypoints[-1] == pytest.approx(right_cell["home"]["right"])
    assert other.arm == "left" and other.waypoints[-1] == pytest.approx(right_cell["home"]["left"])
    assert plan.checks == sum(len(l.waypoints) - 1 for l in plan.legs)
