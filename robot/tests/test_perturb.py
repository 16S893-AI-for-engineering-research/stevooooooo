"""AA-54: perturbed tube placement (owner plan 2026-11-21)."""
from pathlib import Path

import numpy as np
import pytest

from inchworm import perturb as pb

ROBOT = Path(__file__).resolve().parents[1]
TIMELINE = ROBOT / "out" / "inchworm" / "timeline.jsonl"


def test_schedule_counts_and_seed():
    f = pb.schedule_flags(7)
    assert len(f) == pb.N_EPISODES == 50 and sum(f) == pb.N_NOMINAL == 10
    assert f == pb.schedule_flags(7) and f != pb.schedule_flags(8)
    with pytest.raises(pb.PerturbError):
        pb.schedule_flags(1, n=5, n_nominal=6)


def test_about_point_and_measure():
    P = np.eye(4); P[:3, 3] = [0.3, -0.1, 0.4]
    R = pb._axis_angle([0, 0, 1], np.radians(1.5))
    D = pb.about_point(P, R, [0.01, -0.02, 0.005])
    m = pb.measure(D, P)
    assert m["rot_deg"] == pytest.approx(1.5, abs=1e-6)
    assert m["trans_mm"] == pytest.approx([10.0, -20.0, 5.0], abs=1e-6)
    assert pb.within_bounds(m)
    assert not pb.within_bounds({"rot_deg": 2.01, "trans_mm": [0, 0, 0]})
    assert not pb.within_bounds({"rot_deg": 0.0, "trans_mm": [0, 20.5, 0]})


@pytest.fixture(scope="module")
def planners():
    if not TIMELINE.exists():
        pytest.skip("no timeline")
    from inchworm.config import load_config
    cfg = load_config()
    pl = pb.build_planner(cfg, "left", TIMELINE)
    return {"left": pl, "right": pb.Planner(cfg, pl.checker, "right", *_frames(cfg), pb.teleop_limits(cfg))}


def _frames(cfg):
    from inchworm import cycle_replay as cr
    return (cr.clamp_to_eeprom(cfg, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)[0],)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_samples_within_bounds_and_clean(planners, arm):
    pl = planners[arm]
    assert pl.hold != arm
    rng = np.random.default_rng(3)
    for i in range(3):
        e = pl.sample(i, rng)
        assert not e.nominal and pb.within_bounds(e.achieved)
        assert e.insert_reach["pos_mm"] <= pb.REACH_POS_MM
        assert pl.line_problem(e.holding_ticks) is None
        # the achieved perturbation re-measured from the holding ticks
        D = pl._grip(pl.hold, e.holding_ticks) @ np.linalg.inv(pl.G0)
        assert pb.measure(D, pl.P0) == e.achieved


def test_nominal_is_the_frozen_pose(planners):
    pl = planners["left"]
    e = pl.nominal(0)
    assert e.holding_ticks == pl.ticks[pl.hold] and e.achieved["rot_deg"] == 0.0
    assert np.allclose(e.tube_pose, pl.tube0)


def test_plan_schedule_small(planners, tmp_path):
    pl = planners["right"]
    eps = pb.plan_schedule(pl, seed=5, n=4, n_nominal=1)
    assert [e.nominal for e in eps] == pb.schedule_flags(5, 4, 1)
    rec = pb.write(tmp_path / "p.json", pl, 5, eps)
    assert rec["summary"]["nominal"] == 1 and rec["summary"]["perturbed"] == 3
    assert rec["summary"]["max_rot_deg"] <= pb.ROT_MAX_DEG


def test_unreachable_bounds_raise(planners, monkeypatch):
    pl = planners["left"]
    monkeypatch.setattr(pb, "MAX_TRIES", 3)
    monkeypatch.setattr(pb, "REACH_POS_MM", -1.0)            # nothing can pass
    with pytest.raises(pb.PerturbError, match="no usable perturbation"):
        pl.sample(0, np.random.default_rng(0))


# --- AA-58 profile v6 (owner 2026-11-24): holding wrist_roll +-1 deg, attachment point +-5 mm ------
def test_v6_draw_uniform_over_the_whole_range():
    rng = np.random.default_rng(58)
    d = [pb.draw_v6(rng) for _ in range(pb.N_EPISODES)]
    roll = np.array([r for r, _ in d])
    t = np.array([x for _, x in d])
    assert np.all(np.abs(roll) <= pb.V6_ROLL_MAX_DEG) and np.all(np.abs(t) <= pb.V6_TRANS_MAX_MM)
    # spread: values in the inner AND outer half of each bound, both signs (not only the extremes)
    for v, b in [(roll, pb.V6_ROLL_MAX_DEG)] + [(t[:, k], pb.V6_TRANS_MAX_MM) for k in range(3)]:
        assert np.any(np.abs(v) < b / 2) and np.any(np.abs(v) > b / 2)
        assert np.any(v > 0) and np.any(v < 0)
        assert 0.25 < np.mean(np.abs(v) < b / 2) < 0.75
    assert len(set(np.round(roll, 6))) == len(roll)
    rng2 = np.random.default_rng(58)
    assert all(r == pb.draw_v6(rng2)[0] for r, _ in d)


def test_v6_bounds_and_profile_defaults():
    assert pb.within_bounds_v6({"roll_deg": -1.0, "trans_mm": [5.0, -5.0, 0.0]})
    assert not pb.within_bounds_v6({"roll_deg": 1.01, "trans_mm": [0, 0, 0]})
    assert not pb.within_bounds_v6({"roll_deg": 0.0, "trans_mm": [0, 0, -5.01]})
    assert pb.default_nominal("aa54") == pb.N_NOMINAL == 10 and pb.default_nominal("v6") == pb.V6_N_NOMINAL == 0
    assert pb.bounds("aa54") == {"rot_max_deg": 2.0, "trans_max_mm_per_axis": 20.0}
    assert pb.bounds("v6") == {"roll_max_deg": 1.0, "trans_max_mm_per_axis": 5.0}
    f = pb.schedule_flags(7, n=50, n_nominal=pb.V6_N_NOMINAL)
    assert len(f) == 50 and not any(f)
    with pytest.raises(pb.PerturbError, match="unknown perturbation profile"):
        pb.default_nominal("v5")


@pytest.mark.parametrize("arm", ["left", "right"])
def test_v6_samples_hold_the_servo_roll_offset_and_pass_the_checks(planners, arm):
    pl = planners[arm]
    rng = np.random.default_rng(11)
    for i in range(3):
        e = pl.sample_v6(i, rng)
        assert not e.nominal and pb.within_bounds_v6(e.achieved)
        # servo 5 (wrist_roll) of the HOLDING arm = frozen + the tick-rounded requested offset
        want = pl.ticks[pl.hold][pb.ROLL_JOINT] + round(e.requested["roll_offset_deg"] / pb.DEG_PER_TICK)
        assert e.holding_ticks[pb.ROLL_JOINT] == want
        assert e.achieved["roll_offset_deg"] == pytest.approx(
            (want - pl.ticks[pl.hold][pb.ROLL_JOINT]) * pb.DEG_PER_TICK, abs=1e-4)
        lim = pl.limits[pl.hold]
        assert all(lim[j][0] <= t <= lim[j][1] for j, t in zip(lim, e.holding_ticks))
        assert e.insert_reach["pos_mm"] <= pb.REACH_POS_MM and e.insert_reach["rot_deg"] <= pb.REACH_ROT_DEG
        assert pl.line_problem(e.holding_ticks) is None
        D = pl._grip(pl.hold, e.holding_ticks) @ np.linalg.inv(pl.G0)
        assert pl.measure_v6(e.holding_ticks, D) == e.achieved


def test_v6_schedule_deterministic_for_a_seed(planners, tmp_path):
    pl = planners["left"]
    a = pb.plan_schedule(pl, seed=9, n=3, n_nominal=0, profile="v6")
    b = pb.plan_schedule(pl, seed=9, n=3, n_nominal=0, profile="v6")
    assert [(e.holding_ticks, e.requested, e.achieved) for e in a] == [(e.holding_ticks, e.requested, e.achieved) for e in b]
    rec = pb.write(tmp_path / "p.json", pl, 9, a, profile="v6")
    assert rec["profile"] == "v6" and rec["bounds"] == pb.bounds("v6")
    assert all("roll_offset_deg" in e["requested"] and "roll_deg" in e["achieved"] for e in rec["episodes"])
    assert pb.write(tmp_path / "q.json", pl, 9, a)["profile"] == "aa54"          # default unchanged


def test_v6_full_schedule_counts_variation_and_spread(planners):
    """The whole 50-episode v6 schedule for one arm (right, the faster plan): all perturbed, inside
    the bounds, every requested roll different, and requested + achieved values in both the inner
    and outer half of each bound (not all near the limits)."""
    pl = planners["right"]
    eps = pb.plan_schedule(pl, seed=58, n=pb.N_EPISODES, n_nominal=pb.default_nominal("v6"), profile="v6")
    assert len(eps) == 50 and not any(e.nominal for e in eps)
    assert all(pb.within_bounds_v6(e.achieved) for e in eps)
    assert len({e.requested["roll_offset_deg"] for e in eps}) == 50
    assert len({tuple(e.holding_ticks) for e in eps}) == 50
    req = [("roll", [e.requested["roll_offset_deg"] for e in eps], pb.V6_ROLL_MAX_DEG)] + \
          [(k, [e.requested["trans_mm"][i] for e in eps], pb.V6_TRANS_MAX_MM) for i, k in enumerate("xyz")]
    ach = [("roll", [e.achieved["roll_deg"] for e in eps], pb.V6_ROLL_MAX_DEG)] + \
          [(k, [e.achieved["trans_mm"][i] for e in eps], pb.V6_TRANS_MAX_MM) for i, k in enumerate("yz", 1)]
    for name, v, b in req + ach:
        v = np.abs(v)
        assert np.any(v < b / 2) and np.any(v > b / 2), name
