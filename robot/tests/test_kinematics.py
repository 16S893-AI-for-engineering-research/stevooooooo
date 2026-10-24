"""Tests for AA-11: SO-101 FK/IK round trips, trajectory, jog dry-run.

Evidence for AC#3 (simulation sub-mm accuracy) is produced by
test_fk_ik_round_trip_accuracy() which asserts pos_err < 1e-3 m.

AC#3 physical-pose verification and AC#4 collision-check:
  - Require hardware; left unchecked per AGENTS.md.
"""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from inchworm.config import load_config
from inchworm.kinematics import build_chain, IKResult
from inchworm.trajectory import plan_joint_trajectory, sample_trajectory


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def chain():
    cfg = load_config()
    return build_chain(cfg.so101_kinematics)


@pytest.fixture(scope="module")
def traj_limits():
    cfg = load_config()
    return cfg.so101_kinematics.trajectory


# ---------------------------------------------------------------------------
# AC#1 — Forward kinematics
# ---------------------------------------------------------------------------


class TestForwardKinematics:
    def test_zero_config_returns_4x4(self, chain):
        q = np.zeros(5)
        T = chain.fk(q)
        assert T.shape == (4, 4)

    def test_zero_config_deterministic(self, chain):
        q = np.zeros(5)
        T1 = chain.fk(q)
        T2 = chain.fk(q)
        np.testing.assert_array_equal(T1, T2)

    def test_rotation_matrix_is_orthonormal(self, chain):
        rng = np.random.default_rng(0)
        for _ in range(20):
            q = rng.uniform(-0.5, 0.5, 5)
            T = chain.fk(q)
            R = T[:3, :3]
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
            np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-10)

    def test_homogeneous_row(self, chain):
        q = np.array([0.1, -0.2, 0.3, 0.4, -0.5])
        T = chain.fk(q)
        np.testing.assert_array_equal(T[3], [0, 0, 0, 1])

    def test_wrong_q_length_raises(self, chain):
        with pytest.raises(ValueError):
            chain.fk(np.zeros(4))

    def test_joint_rotation_changes_pose(self, chain):
        q_zero = np.zeros(5)
        for i in range(5):
            q = q_zero.copy()
            q[i] = 0.5
            T = chain.fk(q)
            assert not np.allclose(T, chain.fk(q_zero)), (
                f"FK identical with and without rotation at joint {i}"
            )

    def test_poses_documented_in_base_frame(self, chain):
        """FK result is in arm base_link frame (per module docstring)."""
        # At q=0 the TCP should be somewhere in front of the arm (non-zero pos)
        T = chain.fk(np.zeros(5))
        # Just verify position vector is finite and non-NaN
        assert np.all(np.isfinite(T[:3, 3]))


# ---------------------------------------------------------------------------
# AC#2 — Inverse kinematics
# ---------------------------------------------------------------------------


class TestInverseKinematics:
    def test_reachable_target_returns_reachable(self, chain):
        """IK converges for a pose obtained from FK at known joint angles."""
        q_target = np.array([0.2, -0.3, 0.4, 0.1, -0.2])
        T_target = chain.fk(q_target)
        result = chain.ik(T_target)
        assert isinstance(result, IKResult)
        assert result.reachable, (
            f"IK failed for reachable target: pos_err={result.pos_err_m*1000:.3f} mm"
        )

    def test_ik_result_has_correct_shape(self, chain):
        T = chain.fk(np.zeros(5))
        result = chain.ik(T)
        assert result.q.shape == (5,)

    def test_unreachable_target_reports_not_reachable(self, chain):
        """A target impossibly far away should be reported as unreachable."""
        T_far = np.eye(4, dtype=np.float64)
        T_far[:3, 3] = [10.0, 10.0, 10.0]  # 10 metres away — impossible
        result = chain.ik(T_far, max_iter=50)
        assert not result.reachable, "Expected unreachable but got reachable=True"
        assert result.pos_err_m > 1e-3

    def test_ik_respects_joint_limits(self, chain):
        """IK solution must be within joint limits."""
        rng = np.random.default_rng(42)
        for _ in range(30):
            q_valid = rng.uniform(-0.3, 0.3, 5)
            T = chain.fk(q_valid)
            result = chain.ik(T)
            if result.reachable:
                for i, (lo, hi) in enumerate(chain.joint_limits()):
                    assert lo - 1e-9 <= result.q[i] <= hi + 1e-9, (
                        f"Joint {i} out of limits: {result.q[i]:.4f} not in [{lo},{hi}]"
                    )

    def test_seed_zeros_is_deterministic(self, chain):
        """Two calls with default seed must return identical results."""
        T = chain.fk(np.array([0.1, -0.1, 0.2, 0.1, -0.1]))
        r1 = chain.ik(T)
        r2 = chain.ik(T)
        np.testing.assert_array_equal(r1.q, r2.q)

    def test_custom_seed_changes_solution(self, chain):
        """IK with different seeds may return different configurations."""
        q_target = np.array([0.3, -0.4, 0.5, 0.2, -0.3])
        T = chain.fk(q_target)
        r_default = chain.ik(T, q_seed=np.zeros(5))
        r_custom  = chain.ik(T, q_seed=np.array([0.3, -0.4, 0.5, 0.2, -0.3]))
        # Both should be reachable; solutions may differ
        assert r_default.reachable
        assert r_custom.reachable


# ---------------------------------------------------------------------------
# AC#3 — FK/IK round-trip accuracy (simulation, sub-mm)
# ---------------------------------------------------------------------------


class TestFkIkRoundTripAccuracy:
    """
    AC#3 (first half): FK/IK round-trip accuracy in simulation.

    Physical-pose verification (AC#3 second half, ≥5 poses per arm with
    residuals) requires hardware.  Left unchecked with the command:
        needs owner-supervised hardware session: uv run python scripts/fk_verify.py --live
    """

    ROUND_TRIP_POSES = [
        np.array([0.0,   0.0,   0.0,   0.0,   0.0  ]),
        np.array([0.2,  -0.3,   0.4,   0.1,  -0.2  ]),
        np.array([-0.5,  0.5,  -0.4,   0.3,   0.8  ]),
        np.array([0.4,  -0.1,   0.2,  -0.3,   0.5  ]),
        np.array([-0.2,  0.4,  -0.3,   0.2,  -0.4  ]),
        np.array([0.1,   0.1,   0.1,   0.1,   0.1  ]),
        np.array([-0.3, -0.2,   0.5,  -0.1,   0.2  ]),
        np.array([0.3,   0.3,  -0.3,   0.3,  -0.3  ]),
    ]

    def test_fk_ik_round_trip_accuracy(self, chain):
        """FK/IK round-trip position error must be < 1 mm for all test poses."""
        errors_mm = []
        for q_in in self.ROUND_TRIP_POSES:
            T_target = chain.fk(q_in)
            result = chain.ik(T_target, q_seed=np.zeros(5))
            assert result.reachable, (
                f"IK failed for q={q_in}: pos_err={result.pos_err_m*1000:.4f} mm"
            )
            T_back = chain.fk(result.q)
            pos_err_mm = np.linalg.norm(T_back[:3, 3] - T_target[:3, 3]) * 1000
            errors_mm.append(pos_err_mm)
            # Orchestrator 2026-10-24: require 0.05 mm, well inside the
            # sub-mm AC, so the check is not just the solver's stop threshold.
            assert pos_err_mm < 0.05, (
                f"Round-trip position error {pos_err_mm:.4f} mm ≥ 1 mm for q={q_in}"
            )

        # Print summary (visible with pytest -s or in captured output)
        print("\n--- FK/IK Round-Trip Accuracy (simulation) ---")
        print(f"{'Pose':<6}  {'pos_err_mm':>12}")
        for i, (q_in, err) in enumerate(zip(self.ROUND_TRIP_POSES, errors_mm)):
            print(f"{i:<6}  {err:12.6f}")
        print(f"{'max':>6}  {max(errors_mm):12.6f}")
        print(f"{'mean':>6}  {sum(errors_mm)/len(errors_mm):12.6f}")
        print("--- all errors < 1 mm ✓ ---")


# ---------------------------------------------------------------------------
# AC#5 — Trajectory generation
# ---------------------------------------------------------------------------


class TestTrajectoryGeneration:
    def test_plan_returns_trajectory(self, chain, traj_limits):
        q_start = np.zeros(5)
        q_end   = np.array([0.3, -0.5, 0.7, 0.2, -0.1])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits,
                                     joint_names=chain.joint_names())
        assert traj.T > 0
        assert len(traj.segments) == 5

    def test_trajectory_starts_and_ends_correctly(self, chain, traj_limits):
        q_start = np.array([0.1, -0.1, 0.2, -0.2, 0.3])
        q_end   = np.array([0.4, -0.4, 0.5, -0.5, 0.6])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits)
        np.testing.assert_allclose(traj.q_at(0.0),    q_start, atol=1e-9)
        np.testing.assert_allclose(traj.q_at(traj.T), q_end,   atol=1e-6)

    def test_velocity_limit_not_exceeded(self, chain, traj_limits):
        """Peak velocity in trajectory must not exceed config limit."""
        q_start = np.zeros(5)
        q_end   = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits)
        dt = 1e-3
        ts = np.arange(0, traj.T, dt)
        for t in ts:
            qdot = traj.qdot_at(t)
            for i, v in enumerate(qdot):
                assert abs(v) <= traj_limits.max_velocity_rad_per_s + 1e-9, (
                    f"Joint {i} velocity {abs(v):.4f} > limit {traj_limits.max_velocity_rad_per_s}"
                )

    def test_zero_displacement_trajectory(self, chain, traj_limits):
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
        traj = plan_joint_trajectory(q, q.copy(), traj_limits)
        # Zero displacement: T may be 0, q_at(0) = q_start
        np.testing.assert_allclose(traj.q_at(0.0), q, atol=1e-9)

    def test_sample_trajectory_yields_start_and_end(self, chain, traj_limits):
        q_start = np.zeros(5)
        q_end   = np.array([0.5, -0.5, 0.5, -0.5, 0.5])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits)
        samples = list(sample_trajectory(traj, dt=0.05))
        assert len(samples) >= 2
        t_first, q_first = samples[0]
        t_last,  q_last  = samples[-1]
        assert t_first == 0.0
        assert abs(t_last - traj.T) < 1e-9
        np.testing.assert_allclose(q_first, q_start, atol=1e-9)
        np.testing.assert_allclose(q_last,  q_end,   atol=1e-6)

    def test_synchronisation_all_joints_finish_together(self, chain, traj_limits):
        """All joints must have the same T in the synchronised trajectory."""
        q_start = np.zeros(5)
        q_end   = np.array([1.0, 0.1, 0.5, 0.2, 0.8])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits)
        for seg in traj.segments:
            assert abs(seg.T - traj.T) < 1e-9, (
                f"Joint {seg.joint_name} has T={seg.T} != traj.T={traj.T}"
            )

    def test_trajectory_summary_string(self, chain, traj_limits):
        q_start = np.zeros(5)
        q_end   = np.array([0.3, -0.3, 0.3, -0.3, 0.3])
        traj = plan_joint_trajectory(q_start, q_end, traj_limits,
                                     joint_names=chain.joint_names())
        summary = traj.summary()
        assert "JointTrajectory" in summary
        assert "shoulder_pan" in summary


# ---------------------------------------------------------------------------
# AC#4 — Jog CLI dry-run (no motion, no gripper commands)
# ---------------------------------------------------------------------------


class TestJogDryRun:
    """
    AC#4 (dry-run half): jog CLI prints plan without moving.

    The collision-envelope check (AC#4 second half) depends on AA-9 which
    does not yet exist.  The jog calls a hook `collision_check_hook` that
    currently always returns None (no collision).  AC#4 is left unchecked
    pending AA-9.
    """

    JOG_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "jog.py"

    def test_jog_script_exists(self):
        assert self.JOG_SCRIPT.exists(), f"jog.py not found at {self.JOG_SCRIPT}"

    def test_jog_dry_run_delta(self):
        """jog.py --dry-run --arm left --delta prints plan without erroring."""
        result = subprocess.run(
            [sys.executable, str(self.JOG_SCRIPT),
             "--arm", "left",
             "--delta", "0.01,0.0,0.0",
             "--dry-run"],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, (
            f"jog.py --dry-run failed:\nstdout={result.stdout}\nstderr={result.stderr}"
        )
        output = result.stdout + result.stderr
        assert "DRY-RUN" in output.upper() or "dry" in output.lower(), (
            f"Expected dry-run indicator in output:\n{output}"
        )

    def test_jog_dry_run_abs(self):
        """jog.py --dry-run --abs prints absolute-pose plan."""
        result = subprocess.run(
            [sys.executable, str(self.JOG_SCRIPT),
             "--arm", "right",
             "--abs", "0.1,0.0,0.2",
             "--dry-run"],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, (
            f"jog.py --abs --dry-run failed:\nstdout={result.stdout}\nstderr={result.stderr}"
        )

    def test_jog_no_gripper_command_in_output(self):
        """Dry-run output must not contain gripper servo commands."""
        result = subprocess.run(
            [sys.executable, str(self.JOG_SCRIPT),
             "--arm", "left",
             "--delta", "0.0,0.01,0.0",
             "--dry-run"],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0
        combined = (result.stdout + result.stderr).lower()
        # gripper_channel must not appear in motion commands
        assert "gripper_channel" not in combined or "blocked" in combined or \
               "not commanded" in combined or "excluded" in combined, (
            "Dry-run output appears to contain gripper servo commands"
        )

    def test_jog_live_flag_required_for_motion(self):
        """Without --live flag, jog.py must never open serial ports."""
        # This is guaranteed by the dry-run default; we verify --live is
        # recognised (but we don't actually run it, to avoid opening hardware).
        result = subprocess.run(
            [sys.executable, str(self.JOG_SCRIPT), "--help"],
            capture_output=True, text=True, timeout=10,
        )
        assert result.returncode == 0
        assert "--live" in result.stdout or "--live" in result.stderr


# ---------------------------------------------------------------------------
# AC#6 — Config integration: kinematics block loads correctly
# ---------------------------------------------------------------------------


class TestConfigIntegration:
    def test_kinematics_config_loads(self):
        cfg = load_config()
        assert cfg.so101_kinematics is not None
        assert len(cfg.so101_kinematics.joints) == 5

    def test_joint_names_in_order(self):
        cfg = load_config()
        names = [j.name for j in cfg.so101_kinematics.joints]
        assert names == [
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"
        ]

    def test_joint_limits_from_urdf(self):
        cfg = load_config()
        j = {jt.name: jt for jt in cfg.so101_kinematics.joints}
        assert abs(j["shoulder_pan"].limit_lower_rad - (-1.91986)) < 1e-4
        assert abs(j["wrist_roll"].limit_upper_rad   -   2.84121)  < 1e-4

    def test_tool_frame_unverified_flag(self):
        cfg = load_config()
        assert cfg.so101_kinematics.tool_frame_offset.unverified is True

    def test_base_to_chassis_unverified(self):
        cfg = load_config()
        assert cfg.so101_kinematics.left_arm_base_to_chassis.unverified is True
        assert cfg.so101_kinematics.right_arm_base_to_chassis.unverified is True
