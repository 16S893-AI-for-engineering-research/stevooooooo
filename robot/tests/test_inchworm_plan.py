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
    # Printing pose is reachable (decision: stop if it is not). With the
    # owner's matching start (2026-11-11) the shared pose is off each point by
    # the arm-spacing/pitch mismatch; the per-arm 1 mm check is kept in
    # test_per_arm_start_reaches_each_point.
    assert plan.printing_pose_ik_residual_m[0] < 3e-3
    assert plan.printing_pose_ik_residual_m[1] < 3e-3

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

    # Steps follow the owner-specified order (2026-11-11): lift, carry right,
    # left regrip, right regrip, lower + return.
    order = ["printing pose (start)", "lift tube (both locked)",
             "carry tube right (both locked)", "left regrip near point",
             "right regrip middle point",
             "lower tube, arms return to start (both locked)"]
    steps = [fr.step for fr in plan.frames]
    seen = [s for i, s in enumerate(steps) if i == 0 or s != steps[i - 1]]
    assert seen == order, seen

    # The tube moves only while both arms are locked (it cannot move by itself).
    for a, b in zip(plan.frames, plan.frames[1:]):
        moved = (abs(b.tube_advance_m - a.tube_advance_m) > 1e-9
                 or abs(b.tube_lift_m - a.tube_lift_m) > 1e-9)
        if moved:
            assert b.left_locked and b.right_locked, b.step

    # Lift goes up first, before any advance; the cycle ends one pitch on, lowered.
    lift_frames = [f for f in plan.frames if f.step == "lift tube (both locked)"]
    assert lift_frames[-1].tube_lift_m == pytest.approx(plan.lift_m, abs=1e-9)
    # Pure vertical lift: no horizontal tube motion yet.
    assert all(np.linalg.norm(f.tube_motion[:2, 3]) < 1e-9 for f in lift_frames)
    from inchworm.constants import ATTACHMENT_PITCH_MM
    assert plan.frames[-1].tube_advance_m == pytest.approx(ATTACHMENT_PITCH_MM / 1000.0, abs=1e-6)
    # Back down: the end displacement is exactly one pitch along the tube's own
    # (CAD-tilted) axis, i.e. the lift is fully undone.
    end = plan.frames[-1]
    np.testing.assert_allclose(end.tube_motion[:3, 3],
                               end.tube_axis * ATTACHMENT_PITCH_MM / 1000.0, atol=1e-9)

    # Planar crawl (owner): the elbow is used and joint 1 stays small for the
    # left arm (the right arm's pan is reported, see the report JOINT USE).
    L = np.array([f.left_q for f in plan.frames])
    assert np.degrees(np.ptp(L[:, 2])) > 30.0
    assert np.degrees(np.ptp(L[:, 0])) < 5.0
    assert "JOINT USE" in plan.report

    # Both-arm moves are logged with a reason.
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


# ---------------------------------------------------------------------------
# Owner 2026-11-11: matching start pose and the configured soft limits
# ---------------------------------------------------------------------------


def test_matching_start_pose_and_return(cfg, checker):
    """Both arms start in ONE shared pose, and the cycle returns them to it."""
    plan = plan_cycle(cfg, checker)
    assert "matching" in plan.start_kind
    np.testing.assert_allclose(plan.printing_pose_left, plan.printing_pose_right)
    first, last = plan.frames[0], plan.frames[-1]
    np.testing.assert_allclose(first.left_q, first.right_q)
    # Back at the start pose within the IK tolerance (~0.1 deg).
    np.testing.assert_allclose(last.left_q, plan.printing_pose_left, atol=2e-3)
    np.testing.assert_allclose(last.right_q, plan.printing_pose_right, atol=2e-3)
    # The shared pose sits off the attachment points only by the arm-spacing /
    # pitch mismatch (a few mm), reported for the insertion policy to absorb.
    assert max(plan.printing_pose_ik_residual_m) < 3e-3
    assert "matching" in plan.report


def test_soft_limit_violation_is_reported(kin):
    """A frame inside the URDF range but outside the configured soft limit fails."""
    assert kin.soft_limits is not None
    lo, _ = kin.soft_limits["left"]
    q = WORKING.copy()
    q[1] = lo[1] - 0.05                  # shoulder_lift just below the soft limit
    assert q[1] > kin.lower_rad[1]       # ...but still inside the URDF range
    frame = CycleFrame("test soft", q, PARK, True, False, 0.0)
    reasons = validate_frame(frame, kin, reference_relpose=None)
    assert any(r.startswith("soft limit: left shoulder_lift") for r in reasons), reasons


def test_per_arm_start_reaches_each_point(cfg, checker):
    """Per-arm IK (matching_start=False) still puts each gripper on its point within 1 mm."""
    plan = plan_cycle(cfg, checker, matching_start=False, steps=2)
    assert plan.start_kind == "per-arm IK"
    assert plan.printing_pose_ik_residual_m[0] < 1e-3
    assert plan.printing_pose_ik_residual_m[1] < 1e-3
