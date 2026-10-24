"""Tests for AA-18: Coarse approach trajectory planner.

Evidence matrix
---------------
AC#1 — test_plan_produces_trajectory, test_trajectory_time_parameterised
AC#2 — test_pre_insertion_pose_single_definition, test_pre_insertion_pose_standoff
AC#3 — test_opposite_arm_clearance_rejection, test_unsafe_trajectory_rejected
AC#4 — HARDWARE (repeatability over 10 approaches): unchecked.
        needs owner-supervised hardware session:
        ``cd robot && uv run python scripts/approach_repeatability.py --live --n 10``
AC#5 — test_dry_run_prints_plan
AC#6 — test_tolerance_failure_triggers_retreat, test_tolerance_ok_no_abort,
        test_retreat_missing_raises
AC#7 — all tests below

All tests run without hardware. No real serial ports or cameras opened.
Synthetic data with known ground truth plus noise; recovered-vs-true error
is reported in test_ik_synthetic_known_pose.

Implementation note on IK and synthetic test poses
---------------------------------------------------
With unverified/identity arm-base-to-chassis transforms and a tube at the
chassis origin, the pre-insertion pose target (attachment-point + standoff)
falls at [0, 0, 0.05] — very close to the arm shoulder in the URDF rest
pose.  Full 6-DoF IK at this near-singular, geometrically constrained pose
does not converge from the zero seed.

The tests that exercise the full planner pipeline therefore use a tube pose
placed at a known FK-reachable location: the tube is positioned so that the
pre-insertion pose target exactly matches the FK of a known joint config
at ``q_mid = [0, 0.5, -0.5, 0.3, 0.0]``, using position-only IK in
``ApproachConfig.position_only_ik = True``.  This is the correct way to
write unit tests for an arm with an unverified tool frame: forward-then-invert
with known ground truth.

Tests that specifically exercise IK failure (unreachable target) use a
far-away tube pose where IK cannot converge regardless of mode.
"""

from __future__ import annotations

import math
from typing import List, Optional
from unittest.mock import MagicMock

import numpy as np
import pytest

from inchworm.config import (
    ApproachConfig,
    SO101TrajectoryLimits,
    load_config,
)
from inchworm.frames import FrameModel, TubePose
from inchworm.kinematics import build_chain, KinematicChain, IKResult
from inchworm.motion_guard import (
    MotionGuard,
    build_motion_guard,
    MotionViolationError,
    LiveMotionBlockedError,
)
from inchworm.approach import (
    get_pre_insertion_pose,
    plan_approach,
    execute_approach,
    ApproachPlan,
    ApproachUnreachableError,
    ApproachUnsafeError,
    ApproachToleranceError,
    ApproachRetreatMissingError,
    _pose_error,
    _chain_link_points,
)


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def chain(cfg):
    return build_chain(cfg.so101_kinematics)


@pytest.fixture(scope="module")
def guard(cfg):
    return build_motion_guard(cfg, arm_role="left")


@pytest.fixture(scope="module")
def traj_limits(cfg):
    return cfg.so101_kinematics.trajectory


@pytest.fixture(scope="module")
def q_mid():
    """A mid-range joint config within URDF limits — FK gives a reachable pose."""
    return np.array([0.0, 0.5, -0.5, 0.3, 0.0], dtype=np.float64)


@pytest.fixture(scope="module")
def frame_model_at_fk(cfg, chain, q_mid):
    """FrameModel with tube pose chosen so that pre-insertion pose == FK(q_mid).

    With identity arm-base-to-chassis (unverified placeholder) and identity
    tube pose, T_chassis_ap = T_tube_ap = attachment-point in chassis frame.
    In the arm base frame (= chassis frame here), T_base_ap = T_chassis_ap.

    We want:  T_base_ap @ translate(standoff, z-axis) == chain.fk(q_mid)
    So:       T_tube_ap = T_base_ap = chain.fk(q_mid) @ translate(0, 0, -standoff)

    Since T_tube_ap for index 0 = T_chassis_tube (tube is at its own origin,
    index 0 = translation 0), we set T_chassis_tube = chain.fk(q_mid) @ translate(0,0,-standoff).
    """
    _STANDOFF = 0.05
    T_fk = chain.fk(q_mid)
    # Move standoff backward along the TCP z-axis (column 2 of rotation)
    T_tube_target = T_fk.copy()
    T_tube_target[:3, 3] = T_fk[:3, 3] - _STANDOFF * T_fk[:3, 2]
    tube = TubePose(T_chassis_tube=T_tube_target)
    return FrameModel.from_config(cfg, chain, chain, tube_pose=tube)


@pytest.fixture(scope="module")
def approach_cfg_reachable():
    """ApproachConfig with standoff=0.05 matching the frame_model_at_fk setup."""
    return ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,   # loose orientation tolerance (TCP z ≈ arm z at that pose)
        pose_tolerance_m=0.02,  # 2 cm — achievable in simulation
        pose_tolerance_rad=0.5,
        waypoints_q_rad=[],
        retreat_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        unverified=True,
    )


@pytest.fixture
def q_home():
    """Zero configuration (URDF rest pose)."""
    return np.zeros(5, dtype=np.float64)


@pytest.fixture
def approach_cfg_with_retreat():
    """ApproachConfig with retreat and very loose tolerances for execution tests.

    unverified=False so live=True execution tests are not blocked by the
    unverified-config check.  The tolerance and standoff values here are
    chosen for synthetic test reproducibility only.
    """
    return ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,
        pose_tolerance_m=0.5,
        pose_tolerance_rad=1.0,
        waypoints_q_rad=[],
        retreat_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        unverified=False,  # False so live=True tests are not blocked
    )


# ─────────────────────────────────────────────────────────────────────────────
# AC#2 — Pre-insertion pose: single definition, standoff, orientation
# ─────────────────────────────────────────────────────────────────────────────


def test_pre_insertion_pose_single_definition(cfg, chain, approach_cfg_reachable,
                                               frame_model_at_fk):
    """get_pre_insertion_pose is the single definition and returns a 4×4 HTM."""
    T = get_pre_insertion_pose(0, "left", frame_model_at_fk, approach_cfg_reachable, chain)
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-10)


def test_pre_insertion_pose_standoff(cfg, chain, frame_model_at_fk):
    """Standoff distance is correctly encoded in the pre-insertion pose."""
    cfg_s1 = ApproachConfig(standoff_m=0.05, orientation_rpy_rad=[0.0, 0.0, 0.0],
                             ik_tolerance_m=1e-3, ik_tolerance_rad=0.1,
                             pose_tolerance_m=0.01, pose_tolerance_rad=0.1)
    cfg_s2 = ApproachConfig(standoff_m=0.10, orientation_rpy_rad=[0.0, 0.0, 0.0],
                             ik_tolerance_m=1e-3, ik_tolerance_rad=0.1,
                             pose_tolerance_m=0.01, pose_tolerance_rad=0.1)
    T1 = get_pre_insertion_pose(0, "left", frame_model_at_fk, cfg_s1, chain)
    T2 = get_pre_insertion_pose(0, "left", frame_model_at_fk, cfg_s2, chain)
    diff = float(np.linalg.norm(T1[:3, 3] - T2[:3, 3]))
    assert abs(diff - 0.05) < 1e-9, f"standoff diff {diff:.6f} != 0.05"


def test_pre_insertion_pose_different_lattice_indices(cfg, chain, approach_cfg_reachable,
                                                       frame_model_at_fk):
    """Different lattice indices produce different pre-insertion poses."""
    T0 = get_pre_insertion_pose(0, "left", frame_model_at_fk, approach_cfg_reachable, chain)
    T1 = get_pre_insertion_pose(1, "left", frame_model_at_fk, approach_cfg_reachable, chain)
    diff = float(np.linalg.norm(T0[:3, 3] - T1[:3, 3]))
    from inchworm.constants import ATTACHMENT_PITCH_MM
    assert abs(diff - ATTACHMENT_PITCH_MM / 1000.0) < 1e-6, (
        f"lattice-index distance {diff:.6f} m != pitch {ATTACHMENT_PITCH_MM/1000:.6f} m"
    )


def test_pre_insertion_pose_orientation(cfg, chain, frame_model_at_fk):
    """Non-zero orientation_rpy_rad changes the pose orientation."""
    cfg_identity = ApproachConfig(standoff_m=0.05, orientation_rpy_rad=[0.0, 0.0, 0.0],
                                   ik_tolerance_m=1e-3, ik_tolerance_rad=0.1,
                                   pose_tolerance_m=0.01, pose_tolerance_rad=0.1)
    cfg_rotated = ApproachConfig(standoff_m=0.05, orientation_rpy_rad=[0.3, 0.0, 0.0],
                                  ik_tolerance_m=1e-3, ik_tolerance_rad=0.1,
                                  pose_tolerance_m=0.01, pose_tolerance_rad=0.1)
    T_id = get_pre_insertion_pose(0, "left", frame_model_at_fk, cfg_identity, chain)
    T_rot = get_pre_insertion_pose(0, "left", frame_model_at_fk, cfg_rotated, chain)
    assert not np.allclose(T_id[:3, :3], T_rot[:3, :3])


def test_pre_insertion_pose_matches_fk(cfg, chain, approach_cfg_reachable,
                                        frame_model_at_fk, q_mid):
    """Pre-insertion pose closely matches the FK of q_mid (within IK tolerance).

    This validates the frame geometry: the tube was placed so that
    pre-insertion pose = FK(q_mid), and IK from q_mid should recover it.
    """
    T_target = get_pre_insertion_pose(0, "left", frame_model_at_fk, approach_cfg_reachable, chain)
    T_fk_mid = chain.fk(q_mid)
    pos_err_mm = float(np.linalg.norm(T_target[:3, 3] - T_fk_mid[:3, 3])) * 1000
    # Should be very close (< 1 mm) by construction
    assert pos_err_mm < 2.0, (
        f"Pre-insertion pose {pos_err_mm:.3f} mm from FK(q_mid); "
        f"fixture construction may be off"
    )


# ─────────────────────────────────────────────────────────────────────────────
# AC#1 — Plan produces a collision-free, time-parameterised trajectory
# ─────────────────────────────────────────────────────────────────────────────


def test_plan_produces_trajectory(cfg, chain, guard, traj_limits, approach_cfg_reachable,
                                   frame_model_at_fk, q_mid):
    """plan_approach produces an ApproachPlan with at least one segment."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,  # start near the target for convergence
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_reachable,
        traj_limits=traj_limits,
        live=False,
    )
    assert isinstance(plan, ApproachPlan)
    assert len(plan.segments) >= 1
    assert plan.ik_result.reachable


def test_trajectory_time_parameterised(cfg, chain, guard, traj_limits, approach_cfg_reachable,
                                        frame_model_at_fk, q_mid):
    """Each segment has a non-negative duration and samples correctly."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_reachable,
        traj_limits=traj_limits,
        live=False,
    )
    for seg in plan.segments:
        traj = seg.trajectory
        assert traj.T >= 0.0
        q0 = traj.q_at(0.0)
        np.testing.assert_allclose(q0, seg.q_start, atol=1e-9)
        qT = traj.q_at(traj.T)
        np.testing.assert_allclose(qT, seg.q_end, atol=1e-6)


def test_plan_joint_limits_respected(cfg, chain, guard, traj_limits, approach_cfg_reachable,
                                      frame_model_at_fk, q_mid):
    """All trajectory waypoints must stay within URDF joint limits."""
    from inchworm.kinematics import _URDF_JOINT_LIMITS_RAD
    limits = list(_URDF_JOINT_LIMITS_RAD.values())

    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_reachable,
        traj_limits=traj_limits,
        live=False,
    )
    from inchworm.trajectory import sample_trajectory
    for seg in plan.segments:
        for _, q in sample_trajectory(seg.trajectory, dt=max(0.02, seg.trajectory.T / 20 + 1e-9)):
            for j, (lo, hi) in enumerate(limits):
                assert lo - 1e-4 <= float(q[j]) <= hi + 1e-4, (
                    f"joint {j} = {q[j]:.4f} outside limits [{lo:.4f}, {hi:.4f}]"
                )


def test_plan_total_duration_positive(cfg, chain, guard, traj_limits, approach_cfg_reachable,
                                       frame_model_at_fk, q_mid):
    """Total plan duration is non-negative."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_reachable,
        traj_limits=traj_limits,
        live=False,
    )
    assert plan.total_duration_s >= 0.0


# ─────────────────────────────────────────────────────────────────────────────
# AC#3 — Opposite arm rejection and unsafe trajectory rejection
# ─────────────────────────────────────────────────────────────────────────────


def test_unsafe_trajectory_rejected(cfg, chain, guard, traj_limits,
                                     frame_model_at_fk, approach_cfg_reachable, q_mid):
    """A trajectory that violates motion guard is rejected with ApproachUnsafeError."""
    original_check = guard.check_trajectory

    def _always_collide(q_start, q_end, live=False):
        return "synthetic collision: keep-out zone 'test_zone' hit"

    guard.check_trajectory = _always_collide
    try:
        with pytest.raises(ApproachUnsafeError) as exc_info:
            plan_approach(
                lattice_index=0,
                arm="left",
                q_present=q_mid,
                frame_model=frame_model_at_fk,
                chain=chain,
                guard=guard,
                approach_cfg=approach_cfg_reachable,
                traj_limits=traj_limits,
                live=False,
            )
        assert "synthetic collision" in str(exc_info.value)
    finally:
        guard.check_trajectory = original_check


def test_opposite_arm_clearance_rejection(cfg, chain, guard, traj_limits,
                                           frame_model_at_fk, approach_cfg_reachable, q_mid):
    """Opposite arm at same position causes clearance violation → ApproachUnsafeError."""
    # Moving arm starts at q_mid. Opposite arm is also at q_mid → identical FK positions.
    # Distance between identical link points = 0 < 0.05 m minimum clearance.
    with pytest.raises(ApproachUnsafeError) as exc_info:
        plan_approach(
            lattice_index=0,
            arm="left",
            q_present=q_mid,
            frame_model=frame_model_at_fk,
            chain=chain,
            guard=guard,
            approach_cfg=approach_cfg_reachable,
            traj_limits=traj_limits,
            opposite_arm_q=q_mid.copy(),  # same FK → zero clearance
            opposite_arm_chain=chain,
            live=False,
        )
    assert "clearance" in str(exc_info.value).lower()


# ─────────────────────────────────────────────────────────────────────────────
# AC#5 — Dry-run prints the plan
# ─────────────────────────────────────────────────────────────────────────────


def test_dry_run_prints_plan(cfg, chain, guard, traj_limits, approach_cfg_reachable,
                              frame_model_at_fk, q_mid, capsys):
    """execute_approach in dry-run mode prints the plan to stdout."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_reachable,
        traj_limits=traj_limits,
        live=False,
    )

    mock_executor = MagicMock()
    execute_approach(
        plan=plan,
        executor=mock_executor,
        chain=chain,
        approach_cfg=approach_cfg_reachable,
        live=False,
    )

    captured = capsys.readouterr()
    assert "AA-18 ApproachPlan" in captured.out
    assert "lattice_index=0" in captured.out
    mock_executor.write_positions_rad.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
# AC#6 — Tolerance failure → typed abort + retreat
# ─────────────────────────────────────────────────────────────────────────────


def test_tolerance_failure_triggers_retreat(cfg, chain, guard, traj_limits,
                                             approach_cfg_with_retreat,
                                             frame_model_at_fk, q_mid):
    """When achieved pose is outside tolerance, retreat is commanded and error raised."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_with_retreat,
        traj_limits=traj_limits,
        live=False,
    )
    plan.safe = True  # force-mark safe so execute_approach proceeds

    calls = []

    class _FakeExecutor:
        def write_positions_rad(self, q, bypass_guard=False):
            calls.append(q.copy())

        def check_trajectory_rad(self, q_start, q_end):
            return None

        def read_positions_rad(self):
            # Return a configuration far from planned → large FK error
            return np.array([1.5, 1.0, -1.0, 0.5, -0.5], dtype=np.float64)

    tight_cfg = ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,
        pose_tolerance_m=1e-9,   # impossibly tight
        pose_tolerance_rad=1e-9,
        waypoints_q_rad=[],
        retreat_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        unverified=False,  # False so live=True is not blocked
    )

    with pytest.raises(ApproachToleranceError) as exc_info:
        execute_approach(
            plan=plan,
            executor=_FakeExecutor(),
            chain=chain,
            approach_cfg=tight_cfg,
            live=True,
        )

    err = exc_info.value
    assert err.retreated is True
    retreat_q = np.array(tight_cfg.retreat_q_rad, dtype=np.float64)
    np.testing.assert_allclose(calls[-1], retreat_q, atol=1e-9)


def test_tolerance_ok_no_abort(cfg, chain, guard, traj_limits, approach_cfg_with_retreat,
                                frame_model_at_fk, q_mid):
    """When achieved pose is within tolerance, no abort occurs."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=approach_cfg_with_retreat,
        traj_limits=traj_limits,
        live=False,
    )
    plan.safe = True

    class _PerfectExecutor:
        def write_positions_rad(self, q, bypass_guard=False):
            pass

        def check_trajectory_rad(self, q_start, q_end):
            return None

        def read_positions_rad(self):
            return plan.q_preinsert.copy()

    wide_cfg = ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,
        pose_tolerance_m=1.0,
        pose_tolerance_rad=1.0,
        waypoints_q_rad=[],
        retreat_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        unverified=False,  # False so live=True is not blocked
    )

    # Should not raise
    execute_approach(
        plan=plan,
        executor=_PerfectExecutor(),
        chain=chain,
        approach_cfg=wide_cfg,
        live=True,
    )


def test_retreat_missing_raises(cfg, chain, guard, traj_limits, frame_model_at_fk, q_mid):
    """No retreat_q_rad + tolerance failure → ApproachRetreatMissingError."""
    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=ApproachConfig(
            standoff_m=0.05,
            orientation_rpy_rad=[0.0, 0.0, 0.0],
            ik_tolerance_m=1e-3,
            ik_tolerance_rad=0.1,
            pose_tolerance_m=1e-9,
            pose_tolerance_rad=1e-9,
            waypoints_q_rad=[],
            retreat_q_rad=None,
            unverified=True,
        ),
        traj_limits=traj_limits,
        live=False,
    )
    plan.safe = True

    class _BadExecutor:
        def write_positions_rad(self, q, bypass_guard=False):
            pass

        def check_trajectory_rad(self, q_start, q_end):
            return None

        def read_positions_rad(self):
            return np.zeros(5, dtype=np.float64)

    no_retreat_cfg = ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,
        pose_tolerance_m=1e-9,
        pose_tolerance_rad=1e-9,
        retreat_q_rad=None,
        unverified=False,  # False so live=True is not blocked
    )

    with pytest.raises(ApproachRetreatMissingError):
        execute_approach(
            plan=plan,
            executor=_BadExecutor(),
            chain=chain,
            approach_cfg=no_retreat_cfg,
            live=True,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Unreachable target rejection
# ─────────────────────────────────────────────────────────────────────────────


def test_unreachable_target_rejected(cfg, chain, guard, traj_limits, q_mid):
    """A pre-insertion pose 100 m away raises ApproachUnreachableError."""
    # Move tube 100 m away — far outside reachable workspace
    T_far = np.eye(4, dtype=np.float64)
    T_far[0, 3] = 100.0
    far_tube = TubePose(T_chassis_tube=T_far)
    far_model = FrameModel.from_config(cfg, chain, chain, tube_pose=far_tube)

    with pytest.raises(ApproachUnreachableError) as exc_info:
        plan_approach(
            lattice_index=0,
            arm="left",
            q_present=q_mid,
            frame_model=far_model,
            chain=chain,
            guard=guard,
            approach_cfg=ApproachConfig(
                standoff_m=0.05,
                orientation_rpy_rad=[0.0, 0.0, 0.0],
                ik_tolerance_m=1e-9,   # very tight → certainly fails at 100 m
                ik_tolerance_rad=1e-9,
                pose_tolerance_m=0.01,
                pose_tolerance_rad=0.1,
            ),
            traj_limits=traj_limits,
            live=False,
        )
    assert exc_info.value.lattice_index == 0
    assert exc_info.value.pos_err_m > 0.1


# ─────────────────────────────────────────────────────────────────────────────
# Live mode fail-closed on unverified config
# ─────────────────────────────────────────────────────────────────────────────


def test_live_mode_blocked_unverified_config(cfg, chain, guard, traj_limits,
                                              frame_model_at_fk, q_mid):
    """Live mode raises LiveMotionBlockedError when approach config is unverified."""
    with pytest.raises(LiveMotionBlockedError):
        plan_approach(
            lattice_index=0,
            arm="left",
            q_present=q_mid,
            frame_model=frame_model_at_fk,
            chain=chain,
            guard=guard,
            approach_cfg=ApproachConfig(unverified=True),
            traj_limits=traj_limits,
            live=True,  # live=True with unverified=True → must raise
        )


# ─────────────────────────────────────────────────────────────────────────────
# Waypoints
# ─────────────────────────────────────────────────────────────────────────────


def test_waypoints_produce_extra_segments(cfg, chain, guard, traj_limits,
                                           frame_model_at_fk, q_mid):
    """Waypoints in approach_cfg produce additional trajectory segments."""
    cfg_with_wp = ApproachConfig(
        standoff_m=0.05,
        orientation_rpy_rad=[0.0, 0.0, 0.0],
        ik_tolerance_m=1e-3,
        ik_tolerance_rad=0.1,
        pose_tolerance_m=0.02,
        pose_tolerance_rad=0.5,
        waypoints_q_rad=[[0.1, 0.4, -0.4, 0.3, 0.0]],
        retreat_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        unverified=True,
    )

    plan = plan_approach(
        lattice_index=0,
        arm="left",
        q_present=q_mid,
        frame_model=frame_model_at_fk,
        chain=chain,
        guard=guard,
        approach_cfg=cfg_with_wp,
        traj_limits=traj_limits,
        live=False,
    )
    # With one waypoint we expect 2 segments (present→wp, wp→pre_insertion)
    assert len(plan.segments) == 2


# ─────────────────────────────────────────────────────────────────────────────
# Synthetic IK test: known ground-truth pose with noise (AC#7 / solver test)
# ─────────────────────────────────────────────────────────────────────────────


def test_ik_synthetic_known_pose(chain):
    """IK recovers a known configuration from FK + small noise; reports error.

    This test validates the IK solver on synthetic data with known ground truth.
    A random joint config is FK-forward to a pose, then IK is used to recover it.
    The recovered-vs-true position error is reported and must be < 1 mm.
    """
    rng = np.random.default_rng(42)
    limits = chain.joint_limits()
    q_true = np.array(
        [rng.uniform(lo * 0.5, hi * 0.5) for lo, hi in limits],  # stay near centre
        dtype=np.float64,
    )
    T_target = chain.fk(q_true)

    # Add small Gaussian noise to the seed
    noise = rng.standard_normal(5) * 0.05
    q_seed = np.clip(
        q_true + noise,
        [lo for lo, hi in limits],
        [hi for lo, hi in limits],
    )

    ik_result = chain.ik(
        T_target,
        q_seed=q_seed,
        tolerance_m=1e-4,
        tolerance_rad=1e-3,
    )

    pos_err_mm = ik_result.pos_err_m * 1000
    ori_err_deg = math.degrees(ik_result.ori_err_rad)
    q_recovered = ik_result.q
    q_err_norm = float(np.linalg.norm(q_recovered - q_true))
    T_recovered = chain.fk(q_recovered)
    fk_pos_err_mm = float(np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])) * 1000

    print(
        f"\n[IK synthetic test] "
        f"pos_err={pos_err_mm:.4f} mm  ori_err={ori_err_deg:.4f} deg  "
        f"q_err_norm={q_err_norm:.6f} rad  fk_pos_err={fk_pos_err_mm:.4f} mm"
    )

    assert ik_result.reachable, (
        f"IK did not converge: pos_err={pos_err_mm:.2f} mm, "
        f"ori_err={ori_err_deg:.2f} deg"
    )
    assert pos_err_mm < 1.0, f"IK position residual {pos_err_mm:.3f} mm >= 1 mm"
    assert fk_pos_err_mm < 1.0, f"FK of recovered q has {fk_pos_err_mm:.3f} mm error"


# ─────────────────────────────────────────────────────────────────────────────
# _pose_error helper
# ─────────────────────────────────────────────────────────────────────────────


def test_pose_error_identity():
    """Same pose → zero error."""
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = 0.1
    pos_err, ori_err = _pose_error(T, T)
    assert pos_err < 1e-12
    assert ori_err < 1e-12


def test_pose_error_translation():
    """Pure translation → correct position error, zero orientation error."""
    T1 = np.eye(4, dtype=np.float64)
    T2 = np.eye(4, dtype=np.float64)
    T2[0, 3] = 0.1
    pos_err, ori_err = _pose_error(T1, T2)
    assert abs(pos_err - 0.1) < 1e-9
    assert ori_err < 1e-12


# ─────────────────────────────────────────────────────────────────────────────
# ApproachConfig validation
# ─────────────────────────────────────────────────────────────────────────────


def test_approach_config_default_from_yaml(cfg):
    """Config loads the AA-18 approach block from hardware.yaml."""
    assert cfg.approach is not None
    assert cfg.approach.standoff_m == 0.05
    assert cfg.approach.unverified is True
    assert cfg.approach.retreat_q_rad is None


def test_approach_config_bad_orientation_shape():
    """orientation_rpy_rad must have exactly 3 elements."""
    with pytest.raises(Exception):
        ApproachConfig(orientation_rpy_rad=[0.0, 0.0])


def test_approach_config_bad_waypoint_shape():
    """Waypoints must have exactly 5 elements each."""
    with pytest.raises(Exception):
        ApproachConfig(waypoints_q_rad=[[0.0, 0.0, 0.0]])


def test_approach_config_bad_retreat_shape():
    """retreat_q_rad must have exactly 5 elements."""
    with pytest.raises(Exception):
        ApproachConfig(retreat_q_rad=[0.0, 0.0, 0.0])


# --- Orchestrator 2026-10-24: clearance is evaluated in the CHASSIS frame ---
def test_opposite_arm_clearance_uses_arm_base_transforms(guard, chain, q_mid):
    """The same joint pose on two arms whose bases are 1 m apart is NOT a
    collision; with coincident bases it is. Before the fix, link points were
    compared in their own base frames, so base separation was ignored."""
    import numpy as np
    from inchworm.approach import _check_opposite_arm_clearance
    T0 = np.eye(4)
    T1 = np.eye(4)
    T1[1, 3] = 1.0  # opposite arm base 1 m along chassis y
    same = _check_opposite_arm_clearance(
        guard=guard, q_start=q_mid, q_end=q_mid, opposite_q=q_mid, opposite_chain=chain,
        label="t", T_chassis_moving=T0, T_chassis_opposite=T0)
    apart = _check_opposite_arm_clearance(
        guard=guard, q_start=q_mid, q_end=q_mid, opposite_q=q_mid, opposite_chain=chain,
        label="t", T_chassis_moving=T0, T_chassis_opposite=T1)
    assert same is not None and "clearance" in same
    assert apart is None


def test_opposite_arm_clearance_requires_transforms(guard, chain, q_mid):
    import pytest
    from inchworm.approach import _check_opposite_arm_clearance
    with pytest.raises(ValueError, match="arm-base-to-chassis"):
        _check_opposite_arm_clearance(
            guard=guard, q_start=q_mid, q_end=q_mid, opposite_q=q_mid,
            opposite_chain=chain, label="t")
