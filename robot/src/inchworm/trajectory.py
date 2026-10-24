"""Joint-space trajectory generation for the SO-101 arm.

This module produces **time-parameterized, velocity- and
acceleration-limited** joint-space trajectories.  The planner uses a
**trapezoidal velocity profile** per joint, which is the simplest profile
that respects both velocity and acceleration constraints while remaining
easy to verify analytically.

Trapezoidal velocity profile (per joint)
-----------------------------------------
For each joint independently the planner computes:

1. Constant-acceleration ramp-up from zero to ``v_peak`` (or the maximum
   velocity if the move is long enough).
2. Constant-velocity cruise (may be zero for short moves — triangular profile).
3. Constant-deceleration ramp-down to zero.

The joint that requires the **longest duration** sets the synchronisation
time ``T``.  All other joints are re-scaled to finish at the same time
while respecting their individual limits (time-scaled trapezoidal).

Usage::

    from inchworm.config import load_config
    from inchworm.trajectory import plan_joint_trajectory, sample_trajectory

    cfg = load_config()
    traj = plan_joint_trajectory(
        q_start=np.zeros(5),
        q_end=np.array([0.3, -0.5, 0.7, 0.2, -0.1]),
        limits=cfg.so101_kinematics.trajectory,
    )
    for t, q in sample_trajectory(traj, dt=0.02):
        # send q to servo driver
        ...

Dry-run / inspection::

    for segment in traj.segments:
        print(segment)

"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np
import numpy.typing as npt

from inchworm.config import SO101TrajectoryLimits

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

JointVec = npt.NDArray[np.float64]


# ---------------------------------------------------------------------------
# Per-joint trapezoidal segment
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TrapezoidSegment:
    """Trapezoidal (or triangular) motion profile for one joint.

    Phases
    ------
    ``[0, t_acc]``           : constant-acceleration ramp-up
    ``[t_acc, t_acc+t_flat]``: constant-velocity cruise (may be zero)
    ``[t_acc+t_flat, T]``    : constant-deceleration ramp-down
    """

    joint_index: int
    joint_name: str
    q_start: float        # rad
    q_end: float          # rad
    direction: float      # +1.0 or -1.0
    v_peak: float         # peak velocity (rad/s) ≤ v_max
    a_use: float          # acceleration magnitude (rad/s²) ≤ a_max
    t_acc: float          # ramp duration (s)
    t_flat: float         # cruise duration (s), may be 0
    T: float              # total segment duration (s)

    def position(self, t: float) -> float:
        """Joint position at time ``t`` (clamped to [0, T])."""
        t = max(0.0, min(t, self.T))
        delta = abs(self.q_end - self.q_start)
        if delta < 1e-12:
            return self.q_start
        d = self.direction
        a = self.a_use
        v = self.v_peak
        t_a = self.t_acc
        t_c = self.t_flat
        if t <= t_a:
            # Ramp up
            q = 0.5 * a * t * t
        elif t <= t_a + t_c:
            # Cruise
            q = 0.5 * a * t_a * t_a + v * (t - t_a)
        else:
            # Ramp down
            dt = t - t_a - t_c
            q = 0.5 * a * t_a * t_a + v * t_c + v * dt - 0.5 * a * dt * dt
        return self.q_start + d * min(q, delta)

    def velocity(self, t: float) -> float:
        """Joint velocity at time ``t`` (clamped to [0, T])."""
        t = max(0.0, min(t, self.T))
        delta = abs(self.q_end - self.q_start)
        if delta < 1e-12:
            return 0.0
        d = self.direction
        a = self.a_use
        v = self.v_peak
        t_a = self.t_acc
        t_c = self.t_flat
        if t <= t_a:
            return d * a * t
        elif t <= t_a + t_c:
            return d * v
        else:
            dt = t - t_a - t_c
            return d * max(0.0, v - a * dt)


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------


@dataclass
class JointTrajectory:
    """A synchronised multi-joint trapezoidal trajectory.

    All joints share the same total duration ``T`` (set by the slowest
    joint).  Joints with no motion have ``T_segment == 0`` but are still
    represented for consistent indexing.

    Attributes
    ----------
    q_start, q_end:
        Start and end configurations (rad).
    segments:
        One :class:`TrapezoidSegment` per joint (5 for SO-101 5-DoF).
    T:
        Total trajectory duration (seconds).
    joint_names:
        Names of joints in chain order.
    """

    q_start: JointVec
    q_end: JointVec
    segments: list[TrapezoidSegment]
    T: float
    joint_names: list[str]

    def q_at(self, t: float) -> JointVec:
        """Interpolated joint configuration at time ``t`` (seconds)."""
        return np.array([seg.position(t) for seg in self.segments], dtype=np.float64)

    def qdot_at(self, t: float) -> JointVec:
        """Interpolated joint velocity at time ``t`` (rad/s)."""
        return np.array([seg.velocity(t) for seg in self.segments], dtype=np.float64)

    def summary(self) -> str:
        """Human-readable one-liner per joint for dry-run output."""
        lines = [f"JointTrajectory  T={self.T:.3f}s  n_joints={len(self.segments)}"]
        for seg in self.segments:
            lines.append(
                f"  {seg.joint_name:<14s}  "
                f"Δq={seg.q_end - seg.q_start:+.4f} rad  "
                f"v_peak={seg.v_peak:.3f} rad/s  "
                f"t_acc={seg.t_acc:.3f}s  t_flat={seg.t_flat:.3f}s  T={seg.T:.3f}s"
            )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


def plan_joint_trajectory(
    q_start: JointVec,
    q_end: JointVec,
    limits: SO101TrajectoryLimits,
    joint_names: list[str] | None = None,
) -> JointTrajectory:
    """Plan a synchronised trapezoidal joint-space trajectory.

    Each joint is first planned independently at its own maximum velocity and
    acceleration.  The joint that needs the most time sets the synchronisation
    duration ``T_sync``.  Every other joint is then re-scaled: its peak
    velocity is reduced so that it also completes in ``T_sync``, while its
    acceleration is allowed up to the configured maximum.

    Parameters
    ----------
    q_start:
        Starting joint configuration (rad).
    q_end:
        Target joint configuration (rad).
    limits:
        Velocity and acceleration limits from ``so101_kinematics.trajectory``
        in ``hardware.yaml``.
    joint_names:
        Optional joint name labels (for logging and ``summary()``).  Defaults
        to ``["j0", "j1", ..., "j{n-1}"]``.

    Returns
    -------
    JointTrajectory
        Trajectory object; call ``.q_at(t)`` to sample.
    """
    q_start = np.asarray(q_start, dtype=np.float64)
    q_end   = np.asarray(q_end,   dtype=np.float64)
    n = len(q_start)
    if len(q_end) != n:
        raise ValueError(f"q_start length {n} != q_end length {len(q_end)}")

    if joint_names is None:
        joint_names = [f"j{i}" for i in range(n)]

    v_max = limits.max_velocity_rad_per_s
    a_max = limits.max_acceleration_rad_per_s2

    # --- Phase 1: per-joint unconstrained planning ---
    raw_T: list[float] = []
    for i in range(n):
        delta = abs(q_end[i] - q_start[i])
        if delta < 1e-12:
            raw_T.append(0.0)
        else:
            t_raw = _trap_duration(delta, v_max, a_max)
            raw_T.append(t_raw)

    T_sync = max(raw_T) if raw_T else 0.0

    # --- Phase 2: synchronise all joints to T_sync ---
    segments: list[TrapezoidSegment] = []
    for i in range(n):
        delta = abs(q_end[i] - q_start[i])
        direction = 1.0 if q_end[i] >= q_start[i] else -1.0
        if delta < 1e-12 or T_sync < 1e-12:
            # No motion for this joint (or zero-length trajectory)
            segments.append(TrapezoidSegment(
                joint_index=i,
                joint_name=joint_names[i],
                q_start=float(q_start[i]),
                q_end=float(q_end[i]),
                direction=direction,
                v_peak=0.0,
                a_use=a_max,
                t_acc=0.0,
                t_flat=0.0,
                T=T_sync,
            ))
            continue

        v_peak, a_use, t_acc, t_flat = _fit_trap(delta, T_sync, v_max, a_max)
        segments.append(TrapezoidSegment(
            joint_index=i,
            joint_name=joint_names[i],
            q_start=float(q_start[i]),
            q_end=float(q_end[i]),
            direction=direction,
            v_peak=v_peak,
            a_use=a_use,
            t_acc=t_acc,
            t_flat=t_flat,
            T=T_sync,
        ))

    return JointTrajectory(
        q_start=q_start,
        q_end=q_end,
        segments=segments,
        T=T_sync,
        joint_names=joint_names,
    )


# ---------------------------------------------------------------------------
# Sampling helper
# ---------------------------------------------------------------------------


def sample_trajectory(
    traj: JointTrajectory,
    dt: float = 0.02,
) -> Iterator[tuple[float, JointVec]]:
    """Yield ``(t, q)`` pairs sampled at interval ``dt`` seconds.

    The first sample is at ``t=0``; the last is always at ``t=T`` regardless
    of whether ``T`` is an exact multiple of ``dt``.

    Parameters
    ----------
    traj:
        Planned trajectory.
    dt:
        Sample interval (seconds).  Default 20 ms (50 Hz).

    Yields
    ------
    (t, q)
        Time (seconds) and joint configuration (rad array of length n_joints).
    """
    if traj.T < 1e-12:
        yield 0.0, traj.q_start.copy()
        return

    t = 0.0
    while t < traj.T - 1e-9:
        yield t, traj.q_at(t)
        t += dt
    # Always include endpoint exactly
    yield traj.T, traj.q_at(traj.T)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _trap_duration(delta: float, v_max: float, a_max: float) -> float:
    """Minimum time to travel ``delta`` rad with trapezoidal profile."""
    # Time to accelerate to v_max and back
    t_ramp = v_max / a_max
    d_ramp = 0.5 * a_max * t_ramp * t_ramp
    if 2 * d_ramp >= delta:
        # Triangular profile: no cruise phase
        t_acc = math.sqrt(delta / a_max)
        return 2 * t_acc
    else:
        t_flat = (delta - 2 * d_ramp) / v_max
        return 2 * t_ramp + t_flat


def _fit_trap(
    delta: float,
    T: float,
    v_max: float,
    a_max: float,
) -> tuple[float, float, float, float]:
    """Fit a trapezoidal profile to travel ``delta`` in exactly ``T`` seconds.

    Returns ``(v_peak, a_use, t_acc, t_flat)``.

    Derivation
    ~~~~~~~~~~
    With acceleration ``a``, peak velocity ``v``, and ``t_acc = v/a``:

        delta = v * t_acc + v * t_flat
              = v * (T - t_acc)
              = v * (T - v/a)

    Rearranging: ``v² - a*T*v + a*delta = 0``

    Roots: ``v = (a*T ± sqrt(a²*T² - 4*a*delta)) / 2``

    The smaller root gives the longer ramp (more acceleration time), which
    is the physically consistent trapezoidal solution.  The larger root
    corresponds to a near-triangular profile.  We take the **larger** root
    because for a given T longer than the minimum, there is excess time, and
    the desired solution has a flat cruise phase:
    ``v = (a*T - sqrt(a²T² - 4*a*delta)) / 2`` produces the shortest ramp
    (least acceleration effort) solution.

    If ``a²T² < 4*a*delta`` the motion cannot complete in time T at this
    acceleration; we fall back to the triangular peak (``v = a*T/2``).
    """
    a = a_max
    disc = a * a * T * T - 4.0 * a * delta
    if disc < 0.0:
        # Cannot cover delta in time T at a_max — saturate (triangular peak)
        v_peak = a * T / 2.0
    else:
        # Smaller v_peak root → longer ramp, less cruise time.
        # We want the root that gives t_flat >= 0, i.e. v <= a*T/2.
        # v = (aT - sqrt(disc)) / 2  satisfies v <= aT/2 always.
        v_peak = (a * T - math.sqrt(disc)) / 2.0

    # Cap at v_max (re-solve with the effective velocity)
    v_peak = min(v_peak, v_max)
    a_use  = a_max

    if v_peak < 1e-12:
        return 0.0, a_use, 0.0, 0.0

    t_acc  = v_peak / a_use
    t_flat = max(0.0, T - 2.0 * t_acc)
    return v_peak, a_use, t_acc, t_flat
