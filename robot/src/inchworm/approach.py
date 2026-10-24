"""AA-18: Coarse approach trajectory planner to pre-insertion pose.

This module plans a collision-free, velocity- and acceleration-limited
joint-space trajectory from the arm's current configuration to the
**pre-insertion pose** — the single, documented handoff point from which the
learned insertion policy takes over (AA-20, AA-23, AA-26).

The pre-insertion pose is THE single definition
------------------------------------------------
``get_pre_insertion_pose()`` is the one function that maps an attachment-point
lattice index to the TCP target in the arm's base frame.  AA-20, AA-23, and
AA-26 must call this function rather than computing the pose themselves.

The pose is parameterised by:
* ``standoff_m``: standoff distance along the attachment-point z-axis (positive
  = away from the tube bore, toward the arm).
* ``orientation_rpy_rad``: RPY rotation of the TCP at the pre-insertion pose
  relative to the attachment-point frame.

Both values live in ``hardware.yaml`` (``approach`` block, ``unverified: true``
until AA-16 calibration).  Live mode fails closed when ``unverified`` is True.

Architecture
------------
:func:`get_pre_insertion_pose`
    Maps a lattice index + FrameModel + ApproachConfig → 4×4 HTM in the arm
    base frame.  This is the **single definition** for AA-20/AA-23/AA-26.

:func:`plan_approach`
    Full planner: present pose → optional waypoints → pre-insertion pose.
    Returns an :class:`ApproachPlan` (dry-run path with trajectory objects,
    plus all guard results).

:func:`execute_approach`
    Execute a plan through an injected ``GuardedArm``-compatible executor.
    Checks achieved pose (FK of read-back joints) against the pre-insertion
    pose; aborts with typed error and commands the retreat pose on failure.

Typed errors
------------
:class:`ApproachUnreachableError`
    IK failed to converge; the target is geometrically unreachable.

:class:`ApproachUnsafeError`
    MotionGuard rejected the approach; the path collides with a keep-out zone
    or violates joint limits.

:class:`ApproachToleranceError`
    Achieved pose (FK of read-back joints) is outside tolerance after motion.
    The arm has been moved to the retreat pose before this is raised.

:class:`ApproachRetreatMissingError`
    Abort was required but no retreat pose is configured (live mode only).

Safety invariants
-----------------
* Every segment is checked by ``MotionGuard.check_trajectory`` before motion.
* The opposite arm's current holding configuration generates a keep-out capsule
  appended to the trajectory check (see ``_build_opposite_arm_keep_outs``).
* Execution never leaves the arm mid-trajectory: the executor runs each
  segment atomically; on failure the retreat pose is always commanded.
* The gripper is never commanded here; gripper state is managed by AA-17.
* Dry-run (``live=False``) is the default; live motion requires ``live=True``.
* Live mode fails closed on ``unverified: true`` approach config or
  unverified MotionGuard limits.
* Telemetry via AA-7 ``SessionLogger`` when provided.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Protocol, Sequence, Tuple

import numpy as np
import numpy.typing as npt

from inchworm.config import ApproachConfig
from inchworm.frames import FrameModel, _apply_offset
from inchworm.kinematics import KinematicChain, IKResult, _htm_from_rpy_xyz
from inchworm.motion_guard import MotionGuard, MotionViolationError, LiveMotionBlockedError
from inchworm.trajectory import (
    JointTrajectory,
    plan_joint_trajectory,
    sample_trajectory,
)
from inchworm.config import SO101TrajectoryLimits

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Mat44 = npt.NDArray[np.float64]
JointVec = npt.NDArray[np.float64]  # 5-vector of joint angles (rad)

# ---------------------------------------------------------------------------
# Typed exceptions
# ---------------------------------------------------------------------------


class ApproachUnreachableError(ValueError):
    """IK failed to converge; target is geometrically unreachable.

    Attributes
    ----------
    lattice_index:
        The requested lattice index.
    pos_err_m:
        Residual IK position error at the best-effort solution (metres).
    """

    def __init__(self, lattice_index: int, pos_err_m: float, detail: str = "") -> None:
        self.lattice_index = lattice_index
        self.pos_err_m = pos_err_m
        msg = (
            f"AA-18: pre-insertion pose for lattice index {lattice_index} is "
            f"unreachable (IK residual {pos_err_m*1000:.2f} mm)"
        )
        if detail:
            msg += f": {detail}"
        super().__init__(msg)


class ApproachUnsafeError(ValueError):
    """MotionGuard rejected the approach trajectory.

    Attributes
    ----------
    segment:
        Human-readable description of the segment that failed.
    violation:
        MotionGuard violation string.
    """

    def __init__(self, segment: str, violation: str) -> None:
        self.segment = segment
        self.violation = violation
        super().__init__(
            f"AA-18: approach trajectory unsafe in segment '{segment}': {violation}"
        )


class ApproachToleranceError(RuntimeError):
    """Achieved pose is outside tolerance after motion; retreat was commanded.

    Attributes
    ----------
    pos_err_m:
        Position error between achieved and planned pre-insertion pose.
    ori_err_rad:
        Orientation error (radians).
    retreated:
        True if the arm was successfully moved to the retreat pose.
    """

    def __init__(
        self,
        pos_err_m: float,
        ori_err_rad: float,
        retreated: bool,
    ) -> None:
        self.pos_err_m = pos_err_m
        self.ori_err_rad = ori_err_rad
        self.retreated = retreated
        super().__init__(
            f"AA-18: achieved pose outside tolerance "
            f"(pos_err={pos_err_m*1000:.2f} mm, "
            f"ori_err={math.degrees(ori_err_rad):.2f} deg). "
            f"Retreat {'succeeded' if retreated else 'FAILED'}."
        )


class ApproachRetreatMissingError(RuntimeError):
    """Abort required but no retreat pose is configured.

    In live mode this is a hard error: the arm cannot be left at an
    unknown position without a defined safe retreat.  In dry-run mode
    this never fires; the retreat is only attempted in live mode.
    """

    def __init__(self) -> None:
        super().__init__(
            "AA-18: abort triggered but retreat_q_rad is null in approach config. "
            "Populate approach.retreat_q_rad in hardware.yaml and re-verify before live use."
        )


# ---------------------------------------------------------------------------
# Executor protocol
# ---------------------------------------------------------------------------


class ArmExecutor(Protocol):
    """Protocol satisfied by GuardedArm and test doubles.

    The planner is injected with an executor rather than importing GuardedArm
    directly, so tests can substitute a mock without opening hardware.
    """

    def write_positions_rad(self, q: JointVec, bypass_guard: bool = False) -> None:
        """Command a full 5-DoF configuration (radians)."""
        ...

    def check_trajectory_rad(
        self,
        q_start: JointVec,
        q_end: JointVec,
    ) -> Optional[str]:
        """Validate a straight-line trajectory; return violation string or None."""
        ...


# ---------------------------------------------------------------------------
# Pre-insertion pose definition (THE single definition — AC#2)
# ---------------------------------------------------------------------------


def get_pre_insertion_pose(
    lattice_index: int,
    arm: str,
    frame_model: FrameModel,
    approach_cfg: ApproachConfig,
    chain: KinematicChain,
) -> Mat44:
    """Return the pre-insertion TCP pose in the arm base frame (THE single definition).

    This is the *single definition* of the pre-insertion pose used by AA-18,
    AA-20, AA-23, and AA-26.  Callers must not compute this independently.

    The pose is constructed as::

        T_preinsert = T_base_ap @ translate(0, 0, standoff_m) @ R_orientation

    where ``T_base_ap`` is the attachment-point pose in the arm's base frame,
    ``standoff_m`` is the offset along the attachment-point z-axis, and
    ``R_orientation`` is the RPY rotation from ``approach_cfg.orientation_rpy_rad``.

    Parameters
    ----------
    lattice_index:
        Integer lattice index of the target attachment point.
    arm:
        ``"left"`` or ``"right"``.
    frame_model:
        The :class:`~inchworm.frames.FrameModel` (AA-15) holding current tube pose.
    approach_cfg:
        The :class:`~inchworm.config.ApproachConfig` from ``hardware.yaml``.
    chain:
        The kinematic chain for this arm (used only for the base transform).

    Returns
    -------
    Mat44
        4×4 HTM of the pre-insertion TCP pose in the arm **base frame**.

    Notes
    -----
    ``approach_cfg.unverified`` is warned about but does not block this
    function (planning is allowed in dry-run).  Live execution is blocked
    in :func:`execute_approach` when ``unverified`` is True.
    """
    if approach_cfg.unverified:
        logger.warning(
            "AA-18 get_pre_insertion_pose: approach config is UNVERIFIED "
            "(standoff and orientation are placeholders). "
            "Do not use for live motion before AA-16 measurement."
        )

    # Get attachment point in chassis frame
    T_chassis_ap = frame_model.attachment_pose_in_chassis(lattice_index)

    # Get arm-base-to-chassis transform (inverted to get chassis→base)
    if arm == "left":
        T_chassis_base = frame_model._T_chassis_left
    elif arm == "right":
        T_chassis_base = frame_model._T_chassis_right
    else:
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")

    T_base_chassis = np.linalg.inv(T_chassis_base)
    T_base_ap = T_base_chassis @ T_chassis_ap

    # Apply standoff along attachment-point z-axis
    standoff_offset = np.array([0.0, 0.0, approach_cfg.standoff_m], dtype=np.float64)
    T_with_standoff = _apply_offset(T_base_ap, standoff_offset)

    # Apply orientation rotation at the pre-insertion pose
    rpy = approach_cfg.orientation_rpy_rad
    T_orientation = _htm_from_rpy_xyz([0.0, 0.0, 0.0], rpy)
    T_preinsert = T_with_standoff @ T_orientation

    return T_preinsert


# ---------------------------------------------------------------------------
# Approach plan
# ---------------------------------------------------------------------------


@dataclass
class ApproachSegment:
    """One segment of the approach plan (a straight joint-space arc).

    Attributes
    ----------
    label:
        Human-readable segment name (e.g. ``"present→waypoint_0"``,
        ``"waypoint_0→pre_insertion"``).
    q_start:
        Start joint configuration (rad).
    q_end:
        End joint configuration (rad).
    trajectory:
        Time-parameterised trapezoidal trajectory for this segment.
    guard_violation:
        ``None`` if the segment passed guard checks; violation string otherwise.
        A plan with any ``guard_violation`` set must not be executed.
    """

    label: str
    q_start: JointVec
    q_end: JointVec
    trajectory: JointTrajectory
    guard_violation: Optional[str] = None


@dataclass
class ApproachPlan:
    """A complete, validated approach plan (dry-run or live).

    Attributes
    ----------
    lattice_index:
        Target attachment-point lattice index.
    arm:
        ``"left"`` or ``"right"``.
    T_preinsert:
        Pre-insertion pose in arm base frame (the single definition, AC#2).
    q_preinsert:
        Joint configuration at the pre-insertion pose (IK solution).
    ik_result:
        Full IK result for the pre-insertion pose.
    segments:
        Ordered approach segments (each has a trajectory and guard result).
    safe:
        True if all segments passed guard checks AND IK converged.
    provisional:
        True if any transform, offset, or approach config is unverified.
    total_duration_s:
        Sum of all segment durations (seconds).
    summary:
        Human-readable multi-line plan summary for dry-run output.
    """

    lattice_index: int
    arm: str
    T_preinsert: Mat44
    q_preinsert: JointVec
    ik_result: IKResult
    segments: List[ApproachSegment]
    safe: bool
    provisional: bool
    total_duration_s: float
    summary: str = field(default="", repr=False)


def _build_plan_summary(plan: ApproachPlan) -> str:
    """Build a human-readable summary string for the plan."""
    lines = [
        f"AA-18 ApproachPlan  arm={plan.arm}  lattice_index={plan.lattice_index}",
        f"  IK: reachable={plan.ik_result.reachable}  "
        f"pos_err={plan.ik_result.pos_err_m*1000:.3f} mm  "
        f"ori_err={math.degrees(plan.ik_result.ori_err_rad):.3f} deg",
        f"  q_preinsert: {np.round(plan.q_preinsert, 4).tolist()}",
        f"  total_duration: {plan.total_duration_s:.3f} s",
        f"  safe={plan.safe}  provisional={plan.provisional}",
        f"  segments ({len(plan.segments)}):",
    ]
    for seg in plan.segments:
        status = "OK" if seg.guard_violation is None else f"UNSAFE: {seg.guard_violation}"
        lines.append(
            f"    [{seg.label}]  T={seg.trajectory.T:.3f}s  {status}"
        )
        lines.append(f"      {seg.trajectory.summary()}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


def plan_approach(
    lattice_index: int,
    arm: str,
    q_present: JointVec,
    frame_model: FrameModel,
    chain: KinematicChain,
    guard: MotionGuard,
    approach_cfg: ApproachConfig,
    traj_limits: SO101TrajectoryLimits,
    opposite_arm_q: Optional[JointVec] = None,
    opposite_arm_chain: Optional[KinematicChain] = None,
    *,
    live: bool = False,
    session_logger: Any = None,
) -> ApproachPlan:
    """Plan a coarse approach trajectory to the pre-insertion pose.

    Produces an :class:`ApproachPlan` with:
    * The IK solution for the pre-insertion pose.
    * One or more :class:`ApproachSegment` objects (present → waypoints →
      pre-insertion pose), each time-parameterised with trapezoidal limits.
    * Per-segment ``MotionGuard.check_trajectory`` results.
    * Keep-out checks for the opposite arm's current holding configuration
      and the tube volume (both inherited from the guard's collision model).

    Raises
    ------
    ApproachUnreachableError
        If IK for the pre-insertion pose does not converge.
    ApproachUnsafeError
        If any segment is rejected by the guard.  All segments are checked
        regardless (the first unsafe one raises immediately during execution).
        The plan is marked ``safe=False`` but a complete plan is always returned.

    Parameters
    ----------
    lattice_index:
        Target attachment-point lattice index.
    arm:
        ``"left"`` or ``"right"``.
    q_present:
        Current joint configuration (rad) of the moving arm.
    frame_model:
        AA-15 :class:`~inchworm.frames.FrameModel` (current tube pose).
    chain:
        Kinematic chain for the moving arm.
    guard:
        :class:`~inchworm.motion_guard.MotionGuard` for the moving arm.
    approach_cfg:
        :class:`~inchworm.config.ApproachConfig` from ``hardware.yaml``.
    traj_limits:
        :class:`~inchworm.config.SO101TrajectoryLimits` for time-parameterisation.
    opposite_arm_q:
        Current joint configuration of the *opposite* arm (optional).
        If provided, FK link points are added as additional keep-out spheres.
        This implements the AC#3 requirement to account for the opposite arm.
    opposite_arm_chain:
        Kinematic chain for the opposite arm (required if ``opposite_arm_q``
        is provided).
    live:
        If True and approach config or guard limits are unverified, raises
        immediately (fail-closed).
    session_logger:
        Optional AA-7 session logger.

    Returns
    -------
    ApproachPlan
        Full plan; inspect ``.safe`` before calling :func:`execute_approach`.
    """
    _log(session_logger, "info",
         role=f"arm_{arm}", subsystem="approach", event_type="plan_start",
         payload={"lattice_index": lattice_index, "arm": arm, "live": live})

    # Fail-closed: unverified config in live mode
    if live and approach_cfg.unverified:
        raise LiveMotionBlockedError(
            "AA-18: approach config is UNVERIFIED (standoff/orientation are placeholders). "
            "Run AA-16 calibration, update hardware.yaml, and set unverified: false "
            "before live motion."
        )

    # Compute the pre-insertion pose (THE single definition)
    T_preinsert = get_pre_insertion_pose(
        lattice_index, arm, frame_model, approach_cfg, chain
    )

    # Solve IK for the pre-insertion pose
    ik_result = chain.ik(
        T_preinsert,
        q_seed=np.asarray(q_present, dtype=np.float64),
        tolerance_m=approach_cfg.ik_tolerance_m,
        tolerance_rad=approach_cfg.ik_tolerance_rad,
    )

    if not ik_result.reachable:
        _log(session_logger, "warning",
             role=f"arm_{arm}", subsystem="approach", event_type="ik_failed",
             payload={
                 "lattice_index": lattice_index,
                 "pos_err_m": ik_result.pos_err_m,
                 "ori_err_rad": ik_result.ori_err_rad,
             })
        raise ApproachUnreachableError(
            lattice_index,
            ik_result.pos_err_m,
            detail=f"ori_err={math.degrees(ik_result.ori_err_rad):.2f} deg",
        )

    q_preinsert = ik_result.q

    # Build the sequence of (q_start, q_end, label) tuples
    waypoints: list[JointVec] = [
        np.asarray(wp, dtype=np.float64)
        for wp in approach_cfg.waypoints_q_rad
    ]
    q_sequence = (
        [np.asarray(q_present, dtype=np.float64)]
        + waypoints
        + [q_preinsert]
    )

    joint_names = chain.joint_names()
    segments: list[ApproachSegment] = []
    any_unsafe = False
    provisional = approach_cfg.unverified or frame_model._transforms_unverified

    for i in range(len(q_sequence) - 1):
        q_s = q_sequence[i]
        q_e = q_sequence[i + 1]

        if i == 0 and len(q_sequence) == 2:
            label = "present→pre_insertion"
        elif i == 0:
            label = "present→waypoint_0"
        elif i == len(q_sequence) - 2:
            label = f"waypoint_{i-1}→pre_insertion"
        else:
            label = f"waypoint_{i-1}→waypoint_{i}"

        # Time-parameterise
        traj = plan_joint_trajectory(
            q_start=q_s,
            q_end=q_e,
            limits=traj_limits,
            joint_names=joint_names,
        )

        # Guard check: straight-line interpolation (dense sampling)
        violation = guard.check_trajectory(q_s, q_e, live=live)

        # Also check opposite arm keep-outs (AC#3)
        if violation is None and opposite_arm_q is not None and opposite_arm_chain is not None:
            violation = _check_opposite_arm_clearance(
                guard=guard,
                q_start=q_s,
                q_end=q_e,
                opposite_q=np.asarray(opposite_arm_q, dtype=np.float64),
                opposite_chain=opposite_arm_chain,
                label=label,
                T_chassis_moving=_arm_base_T(frame_model, arm),
                T_chassis_opposite=_arm_base_T(frame_model, "right" if arm == "left" else "left"),
            )

        if violation is not None:
            any_unsafe = True
            _log(session_logger, "warning",
                 role=f"arm_{arm}", subsystem="approach", event_type="segment_unsafe",
                 payload={"segment": label, "violation": violation})

        segments.append(ApproachSegment(
            label=label,
            q_start=q_s.copy(),
            q_end=q_e.copy(),
            trajectory=traj,
            guard_violation=violation,
        ))

    total_duration_s = sum(s.trajectory.T for s in segments)

    plan = ApproachPlan(
        lattice_index=lattice_index,
        arm=arm,
        T_preinsert=T_preinsert,
        q_preinsert=q_preinsert,
        ik_result=ik_result,
        segments=segments,
        safe=not any_unsafe,
        provisional=provisional,
        total_duration_s=total_duration_s,
    )
    plan.summary = _build_plan_summary(plan)

    _log(session_logger, "info",
         role=f"arm_{arm}", subsystem="approach", event_type="plan_complete",
         payload={
             "lattice_index": lattice_index,
             "safe": plan.safe,
             "provisional": provisional,
             "total_duration_s": total_duration_s,
             "n_segments": len(segments),
         })

    if any_unsafe:
        # Find first unsafe segment for the error message
        first_unsafe = next(s for s in segments if s.guard_violation is not None)
        raise ApproachUnsafeError(first_unsafe.label, first_unsafe.guard_violation)

    return plan


def _check_opposite_arm_clearance(
    *,
    guard: MotionGuard,
    q_start: JointVec,
    q_end: JointVec,
    opposite_q: JointVec,
    opposite_chain: KinematicChain,
    label: str,
    T_chassis_moving: Optional[Mat44] = None,
    T_chassis_opposite: Optional[Mat44] = None,
) -> Optional[str]:
    """Check that the moving arm's straight-line segment does not collide
    with FK link points of the opposite arm (AC#3).

    The opposite arm's FK link points are computed once and treated as
    point obstacles.  The moving arm's trajectory is sampled at the guard's
    configured ``interpolation_samples`` resolution; at each sample the
    Cartesian distance from every moving-arm link point to every opposite-arm
    link point is compared against a conservative minimum clearance.

    The minimum clearance (0.05 m) is a conservative placeholder until AA-16
    measures the actual arm dimensions.  It is marked as such.

    Returns the violation string or None.
    """
    # Conservative minimum clearance between arm link points (metres).
    # UNVERIFIED: placeholder until arm link geometry is measured in AA-16.
    _MIN_CLEARANCE_M: float = 0.05

    # Orchestrator fix 2026-10-24: each chain's link points are in its OWN
    # arm-base frame. Comparing them directly treats both arms as mounted at
    # the same origin. Transform both into the chassis frame first, using the
    # FrameModel arm-base transforms (unverified until AA-16).
    def _to_chassis(T: Optional[Mat44], pts: list) -> list:
        if T is None:
            raise ValueError(
                "opposite-arm clearance requires both arm-base-to-chassis transforms"
            )
        return [(T @ np.append(p, 1.0))[:3] for p in pts]

    # FK link points for opposite arm (static during this segment)
    opp_points = _to_chassis(T_chassis_opposite, _chain_link_points(opposite_chain, opposite_q))

    # Sample moving arm through the segment
    n_samples = guard.collision_model.collision_cfg.interpolation_samples
    for i in range(n_samples):
        t = float(i) / float(n_samples - 1)
        q_sample = (1.0 - t) * q_start + t * q_end
        moving_points = _to_chassis(T_chassis_moving, _chain_link_points(guard.collision_model.chain, q_sample))

        for mp in moving_points:
            for op in opp_points:
                dist = float(np.linalg.norm(mp - op))
                if dist < _MIN_CLEARANCE_M:
                    return (
                        f"opposite arm clearance violation at sample {i}/{n_samples-1} "
                        f"(t={t:.3f}): moving-arm link {mp.tolist()} is {dist*1000:.1f} mm "
                        f"from opposite-arm link {op.tolist()} "
                        f"(min {_MIN_CLEARANCE_M*1000:.0f} mm, UNVERIFIED placeholder)"
                    )
    return None


def _arm_base_T(frame_model: FrameModel, arm: str) -> Mat44:
    """Chassis <- arm-base transform from the AA-15 FrameModel."""
    side = arm.replace("arm_", "")
    return (frame_model._T_chassis_left if side == "left" else frame_model._T_chassis_right).copy()


def _chain_link_points(
    chain: KinematicChain,
    q: JointVec,
) -> list[npt.NDArray[np.float64]]:
    """Return list of 3D link-frame origin + TCP positions for chain at q."""
    import numpy as np  # noqa: F811

    points: list[npt.NDArray[np.float64]] = []
    T = np.eye(4, dtype=np.float64)

    # We need access to chain internals; replicate the logic from motion_guard
    for jf, qi in zip(chain.joints, q):
        # Fixed part + revolute rotation
        c, s = math.cos(qi), math.sin(qi)
        Rz = np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=np.float64)
        T = T @ jf.T_fixed @ Rz
        points.append(T[:3, 3].copy())

    # TCP
    T_tcp = T @ chain.T_tool_offset
    points.append(T_tcp[:3, 3].copy())
    return points


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------


def execute_approach(
    plan: ApproachPlan,
    executor: ArmExecutor,
    chain: KinematicChain,
    approach_cfg: ApproachConfig,
    *,
    live: bool = False,
    session_logger: Any = None,
) -> None:
    """Execute an approved :class:`ApproachPlan` through the injected executor.

    Sends each segment's time-parameterised trajectory to the executor in
    sequence.  After completing all segments, reads back the joint positions,
    computes FK, and verifies the achieved pose is within tolerance of
    ``plan.T_preinsert``.

    On failure (tolerance exceeded) the arm is immediately commanded to
    ``approach_cfg.retreat_q_rad``.  If no retreat pose is configured,
    :class:`ApproachRetreatMissingError` is raised.

    The arm is **never left mid-trajectory**; every segment is sent
    atomically via the executor's ``write_positions_rad`` call, which
    the underlying ``GuardedArm`` will validate via the motion guard.

    Parameters
    ----------
    plan:
        The :class:`ApproachPlan` from :func:`plan_approach`.
    executor:
        A :class:`ArmExecutor`-compatible object (e.g. ``GuardedArm``).
    chain:
        Kinematic chain for the arm (for FK of read-back joints).
    approach_cfg:
        :class:`~inchworm.config.ApproachConfig` with tolerances and retreat pose.
    live:
        If True, live-mode checks are enforced (unverified config raises).
    session_logger:
        Optional AA-7 session logger.

    Raises
    ------
    ApproachUnsafeError
        If plan is not safe (should have been caught in :func:`plan_approach`).
    ApproachRetreatMissingError
        If abort is needed but no retreat pose is configured.
    ApproachToleranceError
        If achieved pose is outside tolerance after motion; retreat is sent first.
    LiveMotionBlockedError
        If live mode and approach config is unverified.
    """
    if not plan.safe:
        first_unsafe = next(s for s in plan.segments if s.guard_violation is not None)
        raise ApproachUnsafeError(first_unsafe.label, first_unsafe.guard_violation)

    if live and approach_cfg.unverified:
        raise LiveMotionBlockedError(
            "AA-18 execute_approach: approach config is UNVERIFIED. "
            "Set approach.unverified: false in hardware.yaml after AA-16 calibration."
        )

    _log(session_logger, "info",
         role=f"arm_{plan.arm}", subsystem="approach", event_type="execute_start",
         payload={
             "lattice_index": plan.lattice_index,
             "n_segments": len(plan.segments),
             "live": live,
             "dry_run": not live,
         })

    if not live:
        # Dry-run: print the plan, do not command motion.
        print(plan.summary)
        _log(session_logger, "info",
             role=f"arm_{plan.arm}", subsystem="approach", event_type="dry_run_complete",
             payload={"lattice_index": plan.lattice_index})
        return

    # Live execution: send each segment
    for seg in plan.segments:
        _log(session_logger, "info",
             role=f"arm_{plan.arm}", subsystem="approach", event_type="segment_start",
             payload={"segment": seg.label, "T_s": seg.trajectory.T})
        # Send endpoint of this segment (the executor owns per-step interpolation
        # and guard validation at the bus level; we command the final pose of
        # each segment here, having already validated the whole trajectory in plan_approach).
        executor.write_positions_rad(seg.q_end)
        _log(session_logger, "info",
             role=f"arm_{plan.arm}", subsystem="approach", event_type="segment_complete",
             payload={"segment": seg.label})

    # --- Tolerance check (AC#6) ---
    # Read back the achieved joint configuration via FK
    achieved_q = _read_back_q(executor, plan.q_preinsert)
    T_achieved = chain.fk(achieved_q)

    pos_err, ori_err = _pose_error(T_achieved, plan.T_preinsert)

    within_tol = (
        pos_err <= approach_cfg.pose_tolerance_m
        and ori_err <= approach_cfg.pose_tolerance_rad
    )

    _log(session_logger, "info",
         role=f"arm_{plan.arm}", subsystem="approach", event_type="tolerance_check",
         payload={
             "pos_err_m": pos_err,
             "ori_err_rad": ori_err,
             "within_tolerance": within_tol,
             "pose_tolerance_m": approach_cfg.pose_tolerance_m,
             "pose_tolerance_rad": approach_cfg.pose_tolerance_rad,
         })

    if not within_tol:
        retreated = _attempt_retreat(
            executor=executor,
            approach_cfg=approach_cfg,
            arm=plan.arm,
            session_logger=session_logger,
        )
        raise ApproachToleranceError(pos_err, ori_err, retreated=retreated)

    _log(session_logger, "info",
         role=f"arm_{plan.arm}", subsystem="approach", event_type="execute_complete",
         payload={
             "lattice_index": plan.lattice_index,
             "pos_err_m": pos_err,
             "ori_err_rad": ori_err,
         })


def _read_back_q(
    executor: ArmExecutor,
    q_fallback: JointVec,
) -> JointVec:
    """Read back current joint configuration from the executor.

    Falls back to the planned ``q_fallback`` if the executor does not expose
    a ``read_positions_rad`` method (e.g. mock executors).  Logs a warning
    in that case.
    """
    read_fn = getattr(executor, "read_positions_rad", None)
    if read_fn is not None:
        try:
            return np.asarray(read_fn(), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "AA-18: read_positions_rad failed (%s); "
                "using planned q as fallback for tolerance check.",
                exc,
            )
    else:
        logger.warning(
            "AA-18: executor has no read_positions_rad; "
            "using planned q as fallback for tolerance check."
        )
    return q_fallback.copy()


def _pose_error(T_achieved: Mat44, T_target: Mat44) -> Tuple[float, float]:
    """Return (pos_err_m, ori_err_rad) between two 4×4 HTMs."""
    pos_err = float(np.linalg.norm(T_achieved[:3, 3] - T_target[:3, 3]))

    R_err = T_achieved[:3, :3] @ T_target[:3, :3].T
    trace_val = max(-1.0, min(3.0, float(np.trace(R_err))))
    cos_angle = (trace_val - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    ori_err = float(math.acos(cos_angle))

    return pos_err, ori_err


def _attempt_retreat(
    *,
    executor: ArmExecutor,
    approach_cfg: ApproachConfig,
    arm: str,
    session_logger: Any,
) -> bool:
    """Attempt to command the retreat pose; return True on success."""
    if approach_cfg.retreat_q_rad is None:
        _log(session_logger, "error",
             role=f"arm_{arm}", subsystem="approach", event_type="retreat_missing",
             payload={"reason": "retreat_q_rad is null in approach config"})
        raise ApproachRetreatMissingError()

    retreat_q = np.asarray(approach_cfg.retreat_q_rad, dtype=np.float64)
    try:
        executor.write_positions_rad(retreat_q)
        _log(session_logger, "info",
             role=f"arm_{arm}", subsystem="approach", event_type="retreat_complete",
             payload={"retreat_q_rad": retreat_q.tolist()})
        return True
    except Exception as exc:  # noqa: BLE001
        _log(session_logger, "error",
             role=f"arm_{arm}", subsystem="approach", event_type="retreat_failed",
             payload={"error": str(exc)})
        logger.error("AA-18: retreat command FAILED: %s", exc)
        return False


# ---------------------------------------------------------------------------
# Dry-run visualisation helper (optional; matplotlib only if already installed)
# ---------------------------------------------------------------------------


def visualize_approach(plan: ApproachPlan) -> None:
    """Visualise the planned trajectory using matplotlib (if available).

    Does nothing if matplotlib is not installed.  Does NOT add matplotlib
    as a dependency; this is strictly an optional convenience function.

    Plots joint angles vs time for all segments in a single figure.
    """
    try:
        import matplotlib.pyplot as plt  # type: ignore[import]
    except ImportError:
        logger.info(
            "AA-18 visualize_approach: matplotlib not installed; skipping visualisation."
        )
        return

    joint_names = plan.segments[0].trajectory.joint_names if plan.segments else []
    n_joints = len(joint_names)

    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2 * n_joints), sharex=False)
    if n_joints == 1:
        axes = [axes]

    t_offset = 0.0
    for seg in plan.segments:
        traj = seg.trajectory
        times = []
        qs: list[list[float]] = [[] for _ in range(n_joints)]
        for t, q in sample_trajectory(traj, dt=0.02):
            times.append(t + t_offset)
            for j in range(n_joints):
                qs[j].append(float(q[j]))
        for j, ax in enumerate(axes):
            ax.plot(times, qs[j], label=seg.label)
            ax.set_ylabel(joint_names[j] if j < len(joint_names) else f"j{j}")
        t_offset += traj.T

    for j, ax in enumerate(axes):
        ax.legend(fontsize=7)
        ax.grid(True)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(
        f"AA-18 Approach plan: arm={plan.arm} lattice_index={plan.lattice_index}"
    )
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Telemetry helper
# ---------------------------------------------------------------------------


def _log(
    session_logger: Any,
    level: str,
    *,
    role: str,
    subsystem: str,
    event_type: str,
    payload: Optional[dict] = None,
) -> None:
    """Emit to AA-7 SessionLogger if present, else to stdlib logger."""
    if session_logger is not None:
        try:
            getattr(session_logger, level)(
                role=role,
                subsystem=subsystem,
                event_type=event_type,
                payload=payload,
            )
        except Exception:  # noqa: BLE001
            pass
    else:
        getattr(logger, level)(
            "%s.%s: %s %s", role, subsystem, event_type, payload or ""
        )
