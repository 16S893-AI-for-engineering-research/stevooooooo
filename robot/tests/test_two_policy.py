"""AA-55 two-policy sequence: the pure planning helpers (inchworm.two_policy) on the real cell
mesh and the real timeline (no servos)."""
from __future__ import annotations

import pathlib

import numpy as np
import pytest

from inchworm import cycle_replay as cr
from inchworm import training_poses as tp
from inchworm import two_policy as tw
from inchworm.config import load_config

ROOT = pathlib.Path(__file__).parents[1]
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
CFG = load_config()
DEMO_END_LEFT_TICKS = (1977, 2195, 1930, 1396, 1988)          # = lerobot_env/policy_handoff.py


def test_phase_order():
    assert tw.PHASE_NAMES == ("replay_0_42", "left_policy", "left_lock", "right_out", "left_to_train_right",
                              "right_to_train_right", "right_policy", "right_lock", "home")
    # the left carries the tube to train_right only after the right is unlocked and out of its key
    assert tw.PHASE_NAMES.index("left_to_train_right") > tw.PHASE_NAMES.index("right_out")


def test_training_pose_gate():
    t = tp.joint_ticks("train_left")
    assert tw.training_pose_problems("train_left", t) == []
    off = {"left": list(t["left"]), "right": list(t["right"])}
    off["right"][1] += tw.POSE_TOL_TICKS + 1
    p = tw.training_pose_problems("train_left", off)
    assert len(p) == 1 and "right arm off the frozen train_left pose" in p[0] and "shoulder_lift +35" in p[0]


def test_lock_pose_problem():
    assert tw.lock_pose_problem(DEMO_END_LEFT_TICKS, DEMO_END_LEFT_TICKS) is None
    far = list(DEMO_END_LEFT_TICKS); far[2] += 92                         # 8.1 deg
    assert "elbow_flex" in tw.lock_pose_problem(far, DEMO_END_LEFT_TICKS)


def test_joint_line_one_point_per_degree():
    a = {"left": np.zeros(5), "right": np.zeros(5)}
    b = {"left": np.radians([0, 10.5, 0, 0, 0]), "right": np.zeros(5)}
    line = tw.joint_line(a, b)
    assert len(line) == 12 and np.allclose(line[-1]["left"], b["left"]) and np.allclose(line[0]["left"], 0)


@pytest.fixture(scope="module")
def cell():
    if not TIMELINE.exists():
        pytest.skip("no timeline")
    from inchworm.kinematics import build_chain, with_soft_limits
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.so101_pose_reader import ticks_to_q
    fr, _ = cr.clamp_to_eeprom(CFG, cr.load_timeline(TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    checker = build_mesh_collision_guard(CFG, "left").checker
    chain = with_soft_limits(build_chain(CFG.so101_kinematics), CFG.motion_guard.right_arm_limits)
    lock = {"left": np.asarray(ticks_to_q(CFG, "left", list(DEMO_END_LEFT_TICKS))), "right": fr[42].q["right"]}
    tube = tw.CarriedTube(checker, "left", lock, fr[42].tube)
    return dict(fr=fr, checker=checker, chain=chain, lock=lock, tube=tube)


def test_carried_tube_is_the_frame_42_tube_at_the_lock_pose(cell):
    T = cell["tube"](cell["lock"]["left"], cell["lock"]["right"])
    assert tw.tube_offset(T, cell["fr"][42].tube) == (0.0, 0.0)


def test_right_out_is_straight_down_then_clear_and_passes_the_checks(cell):
    fr, chain = cell["fr"], cell["chain"]
    plan = tw.plan_right_out(chain, fr, cell["lock"], cell["tube"])
    z = [chain.fk(f.q["right"])[2, 3] for f in plan[:26]]
    assert abs((z[25] - z[0]) * 1000 + 50.0) < 0.1 and all(b < a for a, b in zip(z, z[1:]))
    assert np.allclose(plan[-1].q["right"], fr[tw.RIGHT_CLEAR_FRAME].q["right"])
    assert all(np.allclose(f.q["left"], cell["lock"]["left"]) for f in plan)          # left static
    assert all(f.locked == {"left": True, "right": False} for f in plan)
    assert tw.frames_problems(CFG, cell["checker"], plan, samples=2) == []


def test_left_and_right_to_train_right_pass_the_checks(cell):
    fr = cell["fr"]
    now = {"left": cell["lock"]["left"], "right": fr[tw.RIGHT_CLEAR_FRAME].q["right"]}
    left = tw.plan_left_to_train_right(CFG, now, cell["tube"])
    assert cr.q_to_ticks(CFG, "left", left[-1].q["left"]) == tp.joint_ticks("train_right")["left"]
    assert all(np.allclose(f.q["right"], now["right"]) for f in left)
    assert tw.frames_problems(CFG, cell["checker"], left, samples=2) == []
    right = tw.plan_right_to_train_right(fr, left[-1].q["left"], cell["tube"])
    assert [f.index for f in right] == list(range(55, 68))
    assert cr.q_to_ticks(CFG, "right", right[-1].q["right"]) == tp.joint_ticks("train_right")["right"]
    segs, probs = tw.fluid_problems(CFG, cell["checker"], right)
    assert probs == [] and len(segs) == 1
    # the carried tube is not the frozen one (the left locked off the model's insert pose): logged only
    mm, deg = tw.tube_offset(right[-1].tube, tp.position("train_right")["tube_pose"])
    assert mm > 1.0


def test_home_timeline_plan_starts_where_the_arms_are_and_ends_at_frame_86(cell):
    fr = cell["fr"]
    now = {"left": np.asarray(fr[74].q["left"]) + np.radians([0, 2, -3, 1, 0]),
           "right": np.asarray(fr[74].q["right"]) + np.radians([0, -1, 2, 0, 0])}
    plan = tw.plan_home_timeline(fr, now, cell["tube"])
    assert [f.index for f in plan] == list(range(74, 87))
    assert all(np.allclose(plan[0].q[a], now[a]) for a in ("left", "right"))
    assert all(np.allclose(plan[-1].q[a], fr[86].q[a]) for a in ("left", "right"))
    assert all(f.locked == {"left": True, "right": True} for f in plan)
    line = tw.plan_home_line(fr, now, cell["tube"])
    assert all(np.allclose(line[-1].q[a], fr[86].q[a]) for a in ("left", "right"))


def test_home_plan_refuses_a_timeline_piece_with_an_open_gripper(cell):
    fr = list(cell["fr"])
    fr[80] = cr.Frame(80, "x", fr[80].q, {"left": True, "right": False}, fr[80].tube)
    with pytest.raises(tw.PlanRefused):
        tw.plan_home_timeline(fr, {a: fr[74].q[a] for a in ("left", "right")}, cell["tube"])
