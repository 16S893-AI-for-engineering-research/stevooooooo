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
