"""
test_inchworm_plan.py
=====================
AA-45: the inchworm cycle planner (inchworm.inchworm_plan) on the owner cell
URDF. These tests prove the per-frame guard rejects the three failure modes the
owner cares about — a colliding frame, a both-unlocked frame, and a rigid-tube
violation each fail — and that the planner produces a labelled, validated
timeline from the placeholder printing pose.

The checker loads once per module (meshes take about 11 s).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

fcl = pytest.importorskip("fcl")
pytest.importorskip("trimesh")

from inchworm.config import load_config  # noqa: E402
from inchworm.inchworm_plan import (  # noqa: E402
    FOLDED_TRANSIT_PLACEHOLDER,
    CycleFrame,
    PlanStopped,
    build_attachment_model,
    build_cell_kinematics,
    plan_cycle,
    validate_frame,
    write_timeline_jsonl,
)
from inchworm.motion_guard import build_mesh_collision_guard  # noqa: E402

_ROBOT = Path(__file__).resolve().parents[1]

WORKING = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
PARK = np.array([1.5, 0.0, 0.0, 0.0, 0.0])


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def checker():
    g = build_mesh_collision_guard(load_config(), "left")
    assert g is not None and g.checker is not None, g and g.load_error
    return g.checker


@pytest.fixture(scope="module")
def kin(cfg, checker):
    return build_cell_kinematics(cfg, checker)


# ---------------------------------------------------------------------------
# The three required failure modes
# ---------------------------------------------------------------------------


def test_colliding_frame_fails(kin, checker):
    """A pose that drives the left arm into the DexArm base is rejected."""
    # Pan swung toward the DexArm with the arm extended: known arm-DexArm hit
    # (mirrors test_cell_collision arm-DexArm case).
    colliding = CycleFrame(
        step="test collision",
        left_q=np.zeros(5),
        right_q=PARK,
        left_locked=True,
        right_locked=False,
        tube_advance_m=None,
    )
    reasons = validate_frame(colliding, kin, reference_relpose=None)
    assert any("mesh collision" in r for r in reasons), reasons


def test_both_unlocked_frame_fails(kin):
    """Both grippers unlocked while the tube is attached violates the invariant."""
    frame = CycleFrame(
        step="test both unlocked",
        left_q=WORKING,
        right_q=WORKING,
        left_locked=False,
        right_locked=False,
        tube_advance_m=0.0,
    )
    reasons = validate_frame(frame, kin, reference_relpose=None)
    assert any("both grippers unlocked" in r for r in reasons), reasons


def test_rigid_tube_violation_fails(kin, checker):
    """Both locked but the relative gripper pose drifted: rigid-tube failure."""
    att = build_attachment_model(kin, WORKING)
    qL, _, _ = kin.ik("left", att.point_pose(0, 0.0), FOLDED_TRANSIT_PLACEHOLDER)
    qR, _, _ = kin.ik("right", att.point_pose(+1, 0.0), FOLDED_TRANSIT_PLACEHOLDER)
    ref_rel = np.linalg.inv(kin.gripper_pose("left", qL)) @ kin.gripper_pose("right", qR)

    # Move only the left arm's pan a few degrees: both still "locked", but the
    # relative gripper pose is no longer the rigid-tube reference.
    drifted_left = qL + np.array([0.08, 0.0, 0.0, 0.0, 0.0])
    frame = CycleFrame(
        step="test rigid drift",
        left_q=drifted_left,
        right_q=qR,
        left_locked=True,
        right_locked=True,
        tube_advance_m=0.0,
    )
    reasons = validate_frame(frame, kin, reference_relpose=ref_rel)
    assert any("rigid-tube constraint" in r for r in reasons), reasons

    # The same drift is NOT a rigid-tube failure when one gripper is unlocked
    # (only the locked-locked segment pins the relative pose).
    frame_unlocked = CycleFrame(
        step="test one unlocked",
        left_q=drifted_left,
        right_q=qR,
        left_locked=False,
        right_locked=True,
        tube_advance_m=0.0,
    )
    reasons2 = validate_frame(frame_unlocked, kin, reference_relpose=ref_rel)
    assert not any("rigid-tube constraint" in r for r in reasons2), reasons2


# ---------------------------------------------------------------------------
# A valid frame passes
# ---------------------------------------------------------------------------


def test_printing_pose_start_has_no_invariant_or_limit_failure(kin, checker):
    """The start frame keeps both grippers locked and stays within joint limits."""
    att = build_attachment_model(kin, WORKING)
    qL, _, _ = kin.ik("left", att.point_pose(0, 0.0), FOLDED_TRANSIT_PLACEHOLDER)
    qR, _, _ = kin.ik("right", att.point_pose(+1, 0.0), FOLDED_TRANSIT_PLACEHOLDER)
    ref_rel = np.linalg.inv(kin.gripper_pose("left", qL)) @ kin.gripper_pose("right", qR)
    frame = CycleFrame("printing pose (start)", qL, qR, True, True, 0.0)
    reasons = validate_frame(frame, kin, reference_relpose=ref_rel)
    assert not any("gripper invariant" in r for r in reasons), reasons
    assert not any("joint limit" in r for r in reasons), reasons
    assert not any("rigid-tube" in r for r in reasons), reasons


# ---------------------------------------------------------------------------
# Full plan
# ---------------------------------------------------------------------------


def test_plan_cycle_produces_labelled_timeline(cfg, checker, tmp_path):
    plan = plan_cycle(cfg, checker)
    # Printing pose is reachable (decision: stop if it is not).
    assert plan.printing_pose_ik_residual_m[0] < 1e-3
    assert plan.printing_pose_ik_residual_m[1] < 1e-3

    # Timeline: every frame carries both arms, lock states, tube pose, step.
    assert len(plan.frames) > 10
    out = tmp_path / "timeline.jsonl"
    write_timeline_jsonl(plan, out)
    rows = [json.loads(l) for l in out.read_text().splitlines()]
    assert len(rows) == len(plan.frames)
    for r in rows:
        assert len(r["left"]) == 5 and len(r["right"]) == 5
        assert isinstance(r["left_locked"], bool) and isinstance(r["right_locked"], bool)
        assert "tube_advance_m" in r and "step" in r

    # The gripper invariant holds at every frame of the owner cycle (never
    # both unlocked, at most one unlocked).
    for fr in plan.frames:
        assert fr.left_locked or fr.right_locked

    # Steps follow the owner-specified order.
    steps = [fr.step for fr in plan.frames]
    assert steps[0] == "printing pose (start)"
    assert any("reposition" in s for s in steps)
    assert any("left regrip" in s for s in steps)
    assert any("right regrip" in s for s in steps)
    assert steps[-1] == "fold back to printing pose"

    # Inserted repositions are logged with a reason.
    assert plan.repositions and all(rp.reason for rp in plan.repositions)

    # Report leads with the PLACEHOLDER finding.
    assert "PLACEHOLDER FINDING (read first)" in plan.report
    assert "never written to hardware.yaml" in plan.report


def test_regrip_target_absent_stops_instead_of_inventing(cfg, checker, monkeypatch):
    """If a regrip target is off the modelled tube, planning stops (never invents)."""
    import inchworm.inchworm_plan as ip

    real_build = ip.build_attachment_model

    def shrunk(kin, working):
        att = real_build(kin, working)
        # Shrink the tube so the near point (index -1) no longer exists on it.
        att.axis_extent_m = (0.10, att.axis_extent_m[1])
        return att

    monkeypatch.setattr(ip, "build_attachment_model", shrunk)
    with pytest.raises(PlanStopped) as exc:
        ip.plan_cycle(cfg, checker)
    assert "does not exist on the modelled tube" in str(exc.value)


def test_printing_pose_unreachable_stops(cfg, checker, monkeypatch):
    """If the printing-pose IK cannot reach, planning stops and reports."""
    import inchworm.inchworm_plan as ip

    real_build = ip.build_attachment_model

    def far_away(kin, working):
        att = real_build(kin, working)
        # Push the middle point 1 m off along the tube axis: unreachable.
        att.T0_middle = att.T0_middle.copy()
        att.T0_middle[:3, 3] = att.T0_middle[:3, 3] + att.tube_axis * 1.0
        att.axis_extent_m = (-1.0, 3.0)
        return att

    monkeypatch.setattr(ip, "build_attachment_model", far_away)
    with pytest.raises(PlanStopped) as exc:
        ip.plan_cycle(cfg, checker)
    assert "did not reach" in str(exc.value)
