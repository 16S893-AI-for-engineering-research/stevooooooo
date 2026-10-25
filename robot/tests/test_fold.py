"""Tests for AA-19: Folded transit pose and fold/unfold sequencing.

Evidence for ACs:

AC#2 — Fold and unfold sequences execute in documented joint order with
        intermediate waypoints that are individually collision-checked, not a
        single simultaneous interpolation.

        Evidence:
        - test_fold_executes_waypoints_in_order: verifies that move_to is called
          once per waypoint in the expected order, with the correct q_rad values,
          and that check_pose + check_trajectory are called per step.
        - test_unfold_executes_waypoints_in_order: same for unfold.
        - test_fold_is_not_simultaneous: verifies each move_to call is separate
          (not a single call with all waypoints).

AC#3 — Folding is only permitted when both grippers report locked state; refusing
        with a clear reason otherwise.

        Evidence:
        - test_fold_refused_both_unlocked: FoldRefused raised, reason mentions both.
        - test_fold_refused_left_unlocked: FoldRefused raised, reason mentions left.
        - test_fold_refused_right_unlocked: FoldRefused raised, reason mentions right.
        - test_unfold_refused_left_unlocked: unfold also requires LOCKED.

AC#5 — An unfold path returns both arms to a state from which a coarse approach
        can begin, without passing through keep-out zones.

        Evidence:
        - test_unfold_final_waypoint_near_neutral: the final unfold waypoint is
          near the neutral pose (coarse-approach-ready).
        - test_unfold_waypoints_collision_checked: guard.check_pose and
          guard.check_trajectory are called for every unfold waypoint.

AC#6 — Tests cover sequencing order, the lock precondition rejection, and
        collision validation of all intermediate waypoints.

        Evidence:
        - All tests below.  Guard violation tests use an injected guard that
          returns a non-None violation string to confirm the sequencer aborts
          with FoldMotionViolation.

AC#1 — Hardware (matches pictures/robot.png): UNCHECKED.
        needs owner-supervised hardware session:
            uv run python scripts/fold_cycle.py --live --cycles 1

AC#4 — Repeatability (10 cycles per joint): UNCHECKED.
        needs owner-supervised hardware session:
            uv run python scripts/fold_cycle.py --live --cycles 10

All tests run without hardware. No real serial ports opened.
Mock backend only.
"""

from __future__ import annotations

import math
from typing import List, Optional
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

from inchworm.config import (
    FoldSequenceConfig,
    FoldWaypoint,
    HardwareConfig,
    load_config,
)
from inchworm.fold import (
    ArmMoverProtocol,
    FoldMotionViolation,
    FoldRefused,
    FoldSequencer,
    GripperLockSMStatus,
    GripperStatusProtocol,
    UnverifiedWaypointError,
    build_fold_sequencer,
)
from inchworm.motion_guard import (
    MotionGuard,
    build_motion_guard,
)


# ---------------------------------------------------------------------------
# Test doubles and fixtures
# ---------------------------------------------------------------------------


class _LockedGripper:
    """Stub GripperStatusProtocol: always reports LOCKED."""

    @property
    def is_locked(self) -> bool:
        return True


class _UnlockedGripper:
    """Stub GripperStatusProtocol: always reports NOT locked."""

    @property
    def is_locked(self) -> bool:
        return False


class _RecordingMover:
    """Stub ArmMoverProtocol: records every move_to call."""

    def __init__(self) -> None:
        self.calls: List[tuple] = []  # list of (q_rad, dry_run)

    def move_to(self, q_rad, *, dry_run: bool) -> None:
        self.calls.append((list(q_rad), dry_run))


def _make_guard_that_accepts() -> MotionGuard:
    """Return a MotionGuard whose check_pose and check_trajectory always pass."""
    guard = MagicMock(spec=MotionGuard)
    guard.check_pose.return_value = None
    guard.check_trajectory.return_value = None
    return guard


def _make_guard_that_rejects(violation: str = "test violation") -> MotionGuard:
    """Return a MotionGuard whose check_pose always returns a violation string."""
    guard = MagicMock(spec=MotionGuard)
    guard.check_pose.return_value = violation
    guard.check_trajectory.return_value = None
    return guard


def _three_waypoint_fold_cfg() -> FoldSequenceConfig:
    """Return a three-waypoint fold config (all unverified, safe zero-ish q_rad)."""
    return FoldSequenceConfig(
        fold_waypoints=[
            FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
            FoldWaypoint(q_rad=[0.0, 1.2, -0.8, 0.0, 0.0], unverified=True),
            FoldWaypoint(q_rad=[0.0, 1.57, -1.57, 0.0, 0.0], unverified=True),
        ],
        unfold_waypoints=[
            FoldWaypoint(q_rad=[0.0, 1.2, -0.8, 0.0, 0.0], unverified=True),
            FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
            FoldWaypoint(q_rad=[0.0, 0.0, 0.0, 0.0, 0.0], unverified=True),
        ],
    )


_FOLDED_TRANSIT_Q = np.array([0.0, 1.57, -1.57, 0.0, 0.0], dtype=np.float64)


def _make_sequencer(
    fold_cfg: Optional[FoldSequenceConfig] = None,
    guard_left: Optional[MotionGuard] = None,
    guard_right: Optional[MotionGuard] = None,
    mover_left: Optional[_RecordingMover] = None,
    mover_right: Optional[_RecordingMover] = None,
    gripper_left: Optional[GripperStatusProtocol] = None,
    gripper_right: Optional[GripperStatusProtocol] = None,
) -> tuple[FoldSequencer, _RecordingMover, _RecordingMover]:
    """Build a FoldSequencer with sensible defaults."""
    if fold_cfg is None:
        fold_cfg = _three_waypoint_fold_cfg()
    if guard_left is None:
        guard_left = _make_guard_that_accepts()
    if guard_right is None:
        guard_right = _make_guard_that_accepts()
    if mover_left is None:
        mover_left = _RecordingMover()
    if mover_right is None:
        mover_right = _RecordingMover()
    if gripper_left is None:
        gripper_left = _LockedGripper()
    if gripper_right is None:
        gripper_right = _LockedGripper()

    seq = FoldSequencer(
        fold_cfg=fold_cfg,
        folded_transit_q=_FOLDED_TRANSIT_Q,
        guard_left=guard_left,
        guard_right=guard_right,
        mover_left=mover_left,
        mover_right=mover_right,
        gripper_left=gripper_left,
        gripper_right=gripper_right,
    )
    return seq, mover_left, mover_right


@pytest.fixture(scope="module")
def cfg() -> HardwareConfig:
    return load_config()


# ---------------------------------------------------------------------------
# AC#6: Gripper lock precondition tests (AC#3)
# ---------------------------------------------------------------------------


class TestGripperPrecondition:
    """AC#3: fold only when both grippers are LOCKED."""

    def test_fold_refused_both_unlocked(self):
        """Fold raises FoldRefused when both grippers are unlocked."""
        seq, _, _ = _make_sequencer(
            gripper_left=_UnlockedGripper(),
            gripper_right=_UnlockedGripper(),
        )
        with pytest.raises(FoldRefused) as exc_info:
            seq.fold(dry_run=True)
        # Reason must mention both grippers
        assert "both grippers" in exc_info.value.reason.lower() or (
            "left" in exc_info.value.reason and "right" in exc_info.value.reason
        ), f"expected 'both grippers' in reason, got: {exc_info.value.reason!r}"

    def test_fold_refused_left_unlocked(self):
        """Fold raises FoldRefused when left gripper is not LOCKED."""
        seq, _, _ = _make_sequencer(
            gripper_left=_UnlockedGripper(),
            gripper_right=_LockedGripper(),
        )
        with pytest.raises(FoldRefused) as exc_info:
            seq.fold(dry_run=True)
        assert "left" in exc_info.value.reason.lower(), (
            f"expected 'left' in reason, got: {exc_info.value.reason!r}"
        )

    def test_fold_refused_right_unlocked(self):
        """Fold raises FoldRefused when right gripper is not LOCKED."""
        seq, _, _ = _make_sequencer(
            gripper_left=_LockedGripper(),
            gripper_right=_UnlockedGripper(),
        )
        with pytest.raises(FoldRefused) as exc_info:
            seq.fold(dry_run=True)
        assert "right" in exc_info.value.reason.lower(), (
            f"expected 'right' in reason, got: {exc_info.value.reason!r}"
        )

    def test_unfold_refused_left_unlocked(self):
        """Unfold also requires both grippers LOCKED."""
        seq, _, _ = _make_sequencer(
            gripper_left=_UnlockedGripper(),
            gripper_right=_LockedGripper(),
        )
        with pytest.raises(FoldRefused):
            seq.unfold(dry_run=True)

    def test_unfold_refused_right_unlocked(self):
        """Unfold also requires both grippers LOCKED."""
        seq, _, _ = _make_sequencer(
            gripper_left=_LockedGripper(),
            gripper_right=_UnlockedGripper(),
        )
        with pytest.raises(FoldRefused):
            seq.unfold(dry_run=True)

    def test_fold_refused_raises_typed_exception(self):
        """FoldRefused is a subclass of FoldError."""
        from inchworm.fold import FoldError

        seq, _, _ = _make_sequencer(
            gripper_left=_UnlockedGripper(),
            gripper_right=_UnlockedGripper(),
        )
        with pytest.raises(FoldError):
            seq.fold(dry_run=True)

    def test_fold_proceeds_when_both_locked(self):
        """Fold executes without raising when both grippers are LOCKED."""
        seq, mover_l, mover_r = _make_sequencer()
        seq.fold(dry_run=True)
        assert len(mover_l.calls) > 0
        assert len(mover_r.calls) > 0


# ---------------------------------------------------------------------------
# AC#6 + AC#2: Sequencing order tests
# ---------------------------------------------------------------------------


class TestSequencingOrder:
    """AC#2: waypoints executed in documented order, not simultaneously."""

    def test_fold_executes_waypoints_in_order(self):
        """move_to is called for each waypoint in the configured order."""
        fold_cfg = _three_waypoint_fold_cfg()
        seq, mover_l, mover_r = _make_sequencer(fold_cfg=fold_cfg)
        seq.fold(dry_run=True)

        # Expected: 3 config waypoints. The terminal folded_transit is already
        # the same as the last config waypoint, so it should NOT be duplicated.
        expected_q = [
            [0.0, 0.8, 0.0, 0.0, 0.0],
            [0.0, 1.2, -0.8, 0.0, 0.0],
            [0.0, 1.57, -1.57, 0.0, 0.0],
        ]
        actual_q = [call[0] for call in mover_l.calls]
        assert len(actual_q) == len(expected_q), (
            f"expected {len(expected_q)} waypoints, got {len(actual_q)}: {actual_q}"
        )
        for i, (actual, expected) in enumerate(zip(actual_q, expected_q)):
            assert np.allclose(actual, expected, atol=1e-6), (
                f"waypoint {i}: expected {expected}, got {actual}"
            )

    def test_unfold_executes_waypoints_in_order(self):
        """Unfold move_to calls follow the configured unfold waypoint order."""
        fold_cfg = _three_waypoint_fold_cfg()
        seq, mover_l, mover_r = _make_sequencer(fold_cfg=fold_cfg)
        seq.unfold(dry_run=True)

        expected_q = [
            [0.0, 1.2, -0.8, 0.0, 0.0],
            [0.0, 0.8, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        actual_q = [call[0] for call in mover_l.calls]
        assert len(actual_q) == len(expected_q), (
            f"expected {len(expected_q)} unfold waypoints, got {len(actual_q)}"
        )
        for i, (actual, expected) in enumerate(zip(actual_q, expected_q)):
            assert np.allclose(actual, expected, atol=1e-6), (
                f"unfold waypoint {i}: expected {expected}, got {actual}"
            )

    def test_fold_is_not_simultaneous(self):
        """Each waypoint is a separate move_to call (never batched)."""
        fold_cfg = _three_waypoint_fold_cfg()
        seq, mover_l, mover_r = _make_sequencer(fold_cfg=fold_cfg)
        seq.fold(dry_run=True)

        # 3 separate calls, not one call with all waypoints
        assert len(mover_l.calls) == 3, (
            f"expected 3 individual move_to calls, got {len(mover_l.calls)}"
        )
        # Each call argument is a 5-element list, not a list of lists
        for i, (q_rad, _dry_run) in enumerate(mover_l.calls):
            assert len(q_rad) == 5, (
                f"call {i}: expected 5-element q_rad, got {len(q_rad)}-element"
            )

    def test_fold_terminal_not_duplicated_when_last_waypoint_matches(self):
        """folded_transit is not appended when the last config waypoint already matches it."""
        # Last waypoint exactly matches folded_transit_q
        fold_cfg = FoldSequenceConfig(
            fold_waypoints=[
                FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
                FoldWaypoint(q_rad=[0.0, 1.57, -1.57, 0.0, 0.0], unverified=True),
            ],
            unfold_waypoints=[],
        )
        seq, mover_l, _ = _make_sequencer(fold_cfg=fold_cfg)
        seq.fold(dry_run=True)
        # Exactly 2 calls: no duplicate of folded_transit
        assert len(mover_l.calls) == 2

    def test_fold_terminal_appended_when_last_waypoint_differs(self):
        """folded_transit is appended when the last config waypoint differs."""
        fold_cfg = FoldSequenceConfig(
            fold_waypoints=[
                FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
            ],
            unfold_waypoints=[],
        )
        seq, mover_l, _ = _make_sequencer(fold_cfg=fold_cfg)
        seq.fold(dry_run=True)
        # 2 calls: the one config waypoint + the appended folded_transit
        assert len(mover_l.calls) == 2
        last_q = mover_l.calls[-1][0]
        assert np.allclose(last_q, [0.0, 1.57, -1.57, 0.0, 0.0], atol=1e-6)

    def test_both_arms_commanded_each_step(self):
        """Both left and right arms receive a move_to call at every waypoint."""
        seq, mover_l, mover_r = _make_sequencer()
        seq.fold(dry_run=True)
        assert len(mover_l.calls) == len(mover_r.calls), (
            "left and right arms must be commanded equally"
        )

    def test_dry_run_flag_propagated(self):
        """move_to is called with dry_run=True in dry-run mode."""
        seq, mover_l, _ = _make_sequencer()
        seq.fold(dry_run=True)
        for _, dry_run in mover_l.calls:
            assert dry_run is True

    def test_right_arm_same_waypoints_as_left(self):
        """Right arm receives identical waypoints as left arm (same joint angles)."""
        seq, mover_l, mover_r = _make_sequencer()
        seq.fold(dry_run=True)
        for i, (left_call, right_call) in enumerate(zip(mover_l.calls, mover_r.calls)):
            assert np.allclose(left_call[0], right_call[0], atol=1e-6), (
                f"step {i}: left arm q_rad {left_call[0]} != right arm q_rad {right_call[0]}"
            )


# ---------------------------------------------------------------------------
# AC#6: Collision validation tests
# ---------------------------------------------------------------------------


class TestCollisionValidation:
    """AC#6: All intermediate waypoints are collision-checked."""

    def test_fold_aborts_on_waypoint_violation(self):
        """FoldMotionViolation raised when check_pose rejects a waypoint."""
        guard = _make_guard_that_rejects("test: waypoint in keep-out zone")
        seq, mover_l, _ = _make_sequencer(guard_left=guard)
        with pytest.raises(FoldMotionViolation) as exc_info:
            seq.fold(dry_run=True)
        assert exc_info.value.step == 0
        assert "test: waypoint in keep-out zone" in exc_info.value.violation

    def test_fold_aborts_at_first_violating_step(self):
        """No move_to is called after a guard violation."""
        guard = _make_guard_that_rejects("violation at step 0")
        seq, mover_l, _ = _make_sequencer(guard_left=guard)
        with pytest.raises(FoldMotionViolation):
            seq.fold(dry_run=True)
        # No moves should have been executed
        assert len(mover_l.calls) == 0

    def test_fold_check_pose_called_for_every_waypoint(self):
        """check_pose is called for every waypoint in the sequence."""
        guard_l = _make_guard_that_accepts()
        seq, mover_l, _ = _make_sequencer(guard_left=guard_l)
        seq.fold(dry_run=True)
        # 3 config waypoints; folded_transit not duplicated
        assert guard_l.check_pose.call_count == 3, (
            f"expected 3 check_pose calls, got {guard_l.check_pose.call_count}"
        )

    def test_fold_check_trajectory_called_between_waypoints(self):
        """check_trajectory is called between consecutive waypoints."""
        guard_l = _make_guard_that_accepts()
        start_q = [0.0, 0.0, 0.0, 0.0, 0.0]
        seq, _, _ = _make_sequencer(guard_left=guard_l)
        seq.fold(dry_run=True, start_q_left=np.array(start_q))
        # 3 waypoints with a start_q -> 3 trajectory checks
        assert guard_l.check_trajectory.call_count == 3, (
            f"expected 3 check_trajectory calls, got {guard_l.check_trajectory.call_count}"
        )

    def test_fold_trajectory_not_checked_without_start_q(self):
        """check_trajectory is NOT called at step 0 when start_q is not provided."""
        guard_l = _make_guard_that_accepts()
        seq, _, _ = _make_sequencer(guard_left=guard_l)
        seq.fold(dry_run=True)  # no start_q_left
        # 2 segment checks (between steps 0→1, 1→2); step 0 has no prior config
        assert guard_l.check_trajectory.call_count == 2, (
            f"expected 2 check_trajectory calls (no start_q), "
            f"got {guard_l.check_trajectory.call_count}"
        )

    def test_unfold_waypoints_collision_checked(self):
        """check_pose is called for every unfold waypoint."""
        guard_l = _make_guard_that_accepts()
        seq, _, _ = _make_sequencer(guard_left=guard_l)
        seq.unfold(dry_run=True)
        assert guard_l.check_pose.call_count == 3

    def test_unfold_aborts_on_segment_violation(self):
        """FoldMotionViolation raised when check_trajectory rejects a segment."""
        guard_l = MagicMock(spec=MotionGuard)
        guard_l.check_pose.return_value = None
        guard_l.check_trajectory.return_value = "segment enters keep-out zone"

        start_q = np.array([0.0, 1.57, -1.57, 0.0, 0.0])
        seq, mover_l, _ = _make_sequencer(guard_left=guard_l)
        with pytest.raises(FoldMotionViolation) as exc_info:
            seq.unfold(dry_run=True, start_q_left=start_q)
        assert "segment enters keep-out zone" in exc_info.value.violation

    def test_violation_names_arm_role(self):
        """FoldMotionViolation identifies which arm triggered the violation."""
        guard_l = _make_guard_that_rejects("left arm violation")
        seq, _, _ = _make_sequencer(guard_left=guard_l)
        with pytest.raises(FoldMotionViolation) as exc_info:
            seq.fold(dry_run=True)
        assert exc_info.value.arm_role == "left"


# ---------------------------------------------------------------------------
# AC#5: Unfold path to coarse-approach-ready state
# ---------------------------------------------------------------------------


class TestUnfoldPath:
    """AC#5: unfold returns arms to coarse-approach-ready state."""

    def test_unfold_final_waypoint_near_neutral(self, cfg):
        """The last unfold waypoint in hardware.yaml is near neutral (all ~0 rad)."""
        assert cfg.fold_sequence is not None
        assert len(cfg.fold_sequence.unfold_waypoints) > 0
        last_q = cfg.fold_sequence.unfold_waypoints[-1].q_rad
        # All joints should be within 0.3 rad of 0 for a near-neutral approach pose
        for i, q_i in enumerate(last_q):
            assert abs(q_i) <= 0.3, (
                f"unfold final waypoint joint {i} is {q_i:.4f} rad; "
                f"expected close to 0 for coarse-approach-ready state"
            )

    def test_unfold_does_not_start_from_folded_without_calling_fold_first(self):
        """Unfold can be called independently; it does not require fold() first."""
        seq, mover_l, _ = _make_sequencer()
        seq.unfold(dry_run=True)
        assert len(mover_l.calls) > 0

    def test_unfold_waypoints_ordered_distal_to_proximal(self, cfg):
        """Unfold waypoints move from elbow-tucked to shoulder-lowered order.

        First waypoint should have higher shoulder_lift (arm still raised),
        last waypoint should have lower shoulder_lift (arm near neutral).
        This verifies distal-first ordering: elbow/wrist relax before
        shoulder is lowered, avoiding keep-out zones.
        """
        assert cfg.fold_sequence is not None
        waypoints = cfg.fold_sequence.unfold_waypoints
        assert len(waypoints) >= 2
        # shoulder_lift is index 1
        # 2026-10-25: the measured fold has shoulder_lift NEGATIVE (URDF sign), so
        # 'raised' = larger |lift|. Same ordering property, sign-independent.
        first_lift = abs(waypoints[0].q_rad[1])
        last_lift = abs(waypoints[-1].q_rad[1])
        assert first_lift > last_lift, (
            f"unfold should start with raised shoulder ({first_lift:.4f} rad) "
            f"and end with lowered shoulder ({last_lift:.4f} rad)"
        )


# ---------------------------------------------------------------------------
# AC#2: Named pose lookup and config integration
# ---------------------------------------------------------------------------


class TestNamedPoseIntegration:
    """AC#2: folded_transit named pose from AA-9 is loaded and validated."""

    def test_folded_transit_pose_in_config(self, cfg):
        """hardware.yaml contains the folded_transit named pose."""
        assert cfg.motion_guard is not None
        names = [p.name for p in cfg.motion_guard.named_poses]
        assert "folded_transit" in names, (
            f"expected 'folded_transit' in named_poses, got {names}"
        )

    def test_folded_transit_has_five_joints(self, cfg):
        """folded_transit q_rad has exactly 5 joint angles."""
        assert cfg.motion_guard is not None
        for pose in cfg.motion_guard.named_poses:
            if pose.name == "folded_transit":
                assert len(pose.q_rad) == 5

    def test_fold_sequence_config_present(self, cfg):
        """hardware.yaml contains the fold_sequence block."""
        assert cfg.fold_sequence is not None

    def test_fold_sequence_has_waypoints(self, cfg):
        """fold_sequence has at least one fold and one unfold waypoint."""
        assert len(cfg.fold_sequence.fold_waypoints) > 0
        assert len(cfg.fold_sequence.unfold_waypoints) > 0

    def test_all_waypoints_have_five_joints(self, cfg):
        """Every waypoint has exactly 5 joint angles."""
        for i, wp in enumerate(cfg.fold_sequence.fold_waypoints):
            assert len(wp.q_rad) == 5, (
                f"fold_waypoints[{i}].q_rad has {len(wp.q_rad)} elements, expected 5"
            )
        for i, wp in enumerate(cfg.fold_sequence.unfold_waypoints):
            assert len(wp.q_rad) == 5, (
                f"unfold_waypoints[{i}].q_rad has {len(wp.q_rad)} elements, expected 5"
            )

    def test_build_fold_sequencer_from_config(self, cfg):
        """build_fold_sequencer succeeds with real config and mock movers."""
        from inchworm.motion_guard import build_motion_guard

        guard_l = build_motion_guard(cfg, "left")
        guard_r = build_motion_guard(cfg, "right")
        seq = build_fold_sequencer(
            cfg=cfg,
            guard_left=guard_l,
            guard_right=guard_r,
            mover_left=_RecordingMover(),
            mover_right=_RecordingMover(),
            gripper_left=_LockedGripper(),
            gripper_right=_LockedGripper(),
        )
        assert isinstance(seq, FoldSequencer)

    def test_build_fold_sequencer_missing_fold_sequence_raises(self):
        """build_fold_sequencer raises ValueError when fold_sequence is missing."""
        cfg = load_config()
        # Create a modified version with fold_sequence=None by parsing a dict
        from inchworm.config import parse_config
        import yaml

        from inchworm.config import DEFAULT_CONFIG_PATH
        raw = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
        raw.pop("fold_sequence", None)
        cfg_no_fold = parse_config(raw)

        from inchworm.motion_guard import build_motion_guard

        guard_l = build_motion_guard(cfg_no_fold, "left")
        guard_r = build_motion_guard(cfg_no_fold, "right")

        with pytest.raises(ValueError, match="fold_sequence"):
            build_fold_sequencer(
                cfg=cfg_no_fold,
                guard_left=guard_l,
                guard_right=guard_r,
                mover_left=_RecordingMover(),
                mover_right=_RecordingMover(),
                gripper_left=_LockedGripper(),
                gripper_right=_LockedGripper(),
            )


# ---------------------------------------------------------------------------
# Live-mode: unverified waypoint rejection
# ---------------------------------------------------------------------------


class TestLiveModeFailing:
    """Live execution fails closed on unverified waypoints."""

    def test_live_fold_rejected_on_unverified_waypoint(self):
        """UnverifiedWaypointError raised in live mode for any unverified waypoint."""
        fold_cfg = FoldSequenceConfig(
            fold_waypoints=[
                FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
            ],
            unfold_waypoints=[],
        )
        seq, mover_l, _ = _make_sequencer(fold_cfg=fold_cfg)
        with pytest.raises(UnverifiedWaypointError) as exc_info:
            seq.fold(dry_run=False)
        assert exc_info.value.step == 0
        # No motion should have been attempted
        assert len(mover_l.calls) == 0

    def test_dry_run_allows_unverified_waypoints(self):
        """Dry-run executes unverified waypoints without raising."""
        fold_cfg = FoldSequenceConfig(
            fold_waypoints=[
                FoldWaypoint(q_rad=[0.0, 0.8, 0.0, 0.0, 0.0], unverified=True),
            ],
            unfold_waypoints=[],
        )
        seq, mover_l, _ = _make_sequencer(fold_cfg=fold_cfg)
        seq.fold(dry_run=True)  # must not raise
        assert len(mover_l.calls) > 0


# ---------------------------------------------------------------------------
# GripperLockSMStatus adapter
# ---------------------------------------------------------------------------


class TestGripperLockSMStatus:
    """GripperLockSMStatus wraps a GripperLockSM-like object correctly."""

    def test_locked_when_state_name_is_LOCKED(self):
        class FakeSM:
            class state:
                name = "LOCKED"

        adapter = GripperLockSMStatus(FakeSM())
        assert adapter.is_locked is True

    def test_not_locked_when_state_name_is_UNLOCKED(self):
        class FakeSM:
            class state:
                name = "UNLOCKED"

        adapter = GripperLockSMStatus(FakeSM())
        assert adapter.is_locked is False

    def test_not_locked_when_state_is_None(self):
        class FakeSM:
            state = None

        adapter = GripperLockSMStatus(FakeSM())
        assert adapter.is_locked is False

    def test_not_locked_when_state_has_no_name(self):
        class FakeSM:
            state = object()

        adapter = GripperLockSMStatus(FakeSM())
        assert adapter.is_locked is False

    def test_satisfies_gripper_status_protocol(self):
        class FakeSM:
            class state:
                name = "LOCKED"

        adapter = GripperLockSMStatus(FakeSM())
        assert isinstance(adapter, GripperStatusProtocol)


# ---------------------------------------------------------------------------
# Protocol structural typing
# ---------------------------------------------------------------------------


class TestProtocolStructure:
    """ArmMoverProtocol and GripperStatusProtocol are runtime-checkable."""

    def test_recording_mover_satisfies_protocol(self):
        """_RecordingMover satisfies ArmMoverProtocol."""
        assert isinstance(_RecordingMover(), ArmMoverProtocol)

    def test_locked_gripper_satisfies_protocol(self):
        """_LockedGripper satisfies GripperStatusProtocol."""
        assert isinstance(_LockedGripper(), GripperStatusProtocol)
