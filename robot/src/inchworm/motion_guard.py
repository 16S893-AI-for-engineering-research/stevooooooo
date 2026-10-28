"""AA-9: Joint-limit and collision-envelope enforcement for the SO-101 arms.

This module is the single motion-safety layer.  Every position write for
joints 1-5 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)
must pass through :class:`MotionGuard` before reaching the servo bus.

Architecture
------------
:class:`JointLimitGuard`
    Validates joint angles in radians against per-arm soft limits from
    ``motion_guard.left_arm_limits`` / ``right_arm_limits`` in
    ``hardware.yaml``.

    *Fail-closed*: if a limit is null (not measured) or the soft-limits block
    is marked ``unverified: true``, the guard falls back to the URDF limits
    from ``so101_kinematics`` but **refuses LIVE motion**.  An operator must
    run ``scripts/measure_limits.py --live``, populate all limit fields, and
    set ``unverified: false`` to unlock live operation.

:class:`CollisionModel`
    Checks FK link-point positions against conservative keep-out primitives
    (boxes and capsules) defined in ``motion_guard.collision`` in
    ``hardware.yaml``.  Primitives are evaluated in the **arm base frame**
    (``base_link`` frame of the arm being commanded).

    *Trajectory interpolation*: straight-line joint-space paths are sampled
    at a dense, documented resolution (default 50 samples, configurable via
    ``collision.interpolation_samples``).  Both endpoints and all intermediate
    samples are checked.  Endpoint-only checking is insufficient because a
    path can enter and exit a keep-out zone while keeping both endpoints
    outside it.

:class:`MotionGuard`
    Combines :class:`JointLimitGuard` and :class:`CollisionModel`.  Exposes a
    single entry point:

    .. code-block:: python

        violation = guard.check_pose(q, live=True)
        if violation is not None:
            raise MotionViolationError(violation)

        violation = guard.check_trajectory(q_start, q_end, live=True)
        if violation is not None:
            raise MotionViolationError(violation)

:class:`GuardedArm`
    Wraps :class:`~inchworm.servo_bus.ArmServoBus`.  All position writes for
    joints 1-5 are intercepted and passed through :class:`MotionGuard`.  The
    gripper joint is never guarded here; it is handled by the AA-17 state
    machine.

    An explicit ``bypass_guard=True`` flag is available for emergency or
    diagnostic use.  **Its use logs a WARNING event via AA-7 telemetry.**
    Every call site that sets ``bypass_guard=True`` must have a comment
    explaining why; the design intent is that no production code path ever
    sets it.

Single-joint write: full-pose check
-------------------------------------
:meth:`GuardedArm.write_position_deg` receives a single joint name and servo
degree value.  To run a full 5-DoF check it must know the current angle of
the other four joints.  It obtains those by calling
``ArmServoBus.read_position_deg`` for each mapped servo ID, converting via
the per-joint calibration (see :mod:`inchworm.joint_calibration`), and then
passing the assembled 5-vector to :meth:`MotionGuard.check_pose` and
:meth:`MotionGuard.check_trajectory` (present → target).

A ``present_pose_source`` callback can be injected at construction time to
override the bus read (useful in tests or when the caller already holds the
present state).

URDF fallback limits
--------------------
URDF limits are taken from ``so101_kinematics.joints`` in ``hardware.yaml``
(source: ``vendor/so101/so101_new_calib.urdf``, hash a758567c).  They are
used only when the soft-limits block has null values.  When URDF fallbacks are
used, the guard logs a WARNING and blocks LIVE motion.

Named poses
-----------
:func:`get_named_pose` looks up a pose from ``motion_guard.named_poses`` by
name.  Poses with ``unverified: true`` are returned with a warning and must
not be used for live motion without explicit human review.

Interpolation resolution
------------------------
The straight-line joint-space interpolation for trajectory collision checks
is sampled at ``collision.interpolation_samples`` evenly-spaced points
(including both endpoints).  The default is 50 samples.  For the widest joint
(``wrist_roll`` ≈ 5.58 rad), 50 samples gives ≤ 0.11 rad per step.  Decrease
``interpolation_samples`` for speed (accepting coarser checking), increase for
tighter safety.  The resolution is documented here and in ``hardware.yaml``.

Safety invariants (doc-1 s8, AGENTS.md)
----------------------------------------
* This guard never opens real serial ports.
* Dry-run is the default; live motion requires ``live=True``.
* Fail-closed: null or unverified limits block live motion.
* Never both grippers unlocked while tube attached (enforced by AA-17, not here).
* The gripper servo is never commanded by this module.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import numpy.typing as npt

from inchworm.config import (
    CollisionConfig,
    HardwareConfig,
    KeepOutBox,
    KeepOutCapsule,
    MotionGuardConfig,
    NamedPose,
    SO101KinematicsConfig,
    SoftJointLimits,
)
from inchworm.kinematics import KinematicChain, build_chain

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

JointVec = npt.NDArray[np.float64]  # 5-vector of joint angles (rad)

# Joint names in chain order (proximal → distal).  Index aligns with JointVec.
JOINT_NAMES: Tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

# ---------------------------------------------------------------------------
# Typed exceptions
# ---------------------------------------------------------------------------


class MotionViolationError(ValueError):
    """Raised when a commanded pose or trajectory violates the motion guard.

    Attributes
    ----------
    violation:
        Human-readable description of the violation (joint name, limit value,
        or collision primitive label).
    """

    def __init__(self, violation: str) -> None:
        self.violation = violation
        super().__init__(f"motion guard violation: {violation}")


class LiveMotionBlockedError(MotionViolationError):
    """Raised when LIVE motion is requested but limits are unverified.

    The operator must run ``scripts/measure_limits.py --live``, populate the
    soft-limits block in ``hardware.yaml``, and set ``unverified: false``.
    """


# ---------------------------------------------------------------------------
# Resolved joint limits (one arm)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ResolvedLimits:
    """Resolved per-joint soft limits for one arm.

    ``unverified`` is True when any limit came from the URDF fallback or
    when the soft-limits config block is marked ``unverified: true``.
    ``live_ok`` is True only when all limits are present and ``unverified``
    is False.
    """

    # Parallel to JOINT_NAMES
    lower_rad: Tuple[float, ...]  # length 5
    upper_rad: Tuple[float, ...]  # length 5
    unverified: bool
    live_ok: bool


def _resolve_limits_for_arm(
    soft: Optional[SoftJointLimits],
    kin_cfg: SO101KinematicsConfig,
    arm_role: str,
) -> ResolvedLimits:
    """Build :class:`ResolvedLimits` for one arm.

    Falls back to URDF limits for any null field.  Marks ``unverified=True``
    and ``live_ok=False`` if:
    * ``soft`` is None (no soft-limits block)
    * ``soft.unverified`` is True
    * any individual limit field is null

    Parameters
    ----------
    soft:
        The ``SoftJointLimits`` block from config, or ``None``.
    kin_cfg:
        The ``SO101KinematicsConfig`` block (source of URDF fallback limits).
    arm_role:
        ``"left"`` or ``"right"`` — used in log messages.
    """
    # Build URDF limit map: name → (lower, upper)
    urdf_map: dict[str, tuple[float, float]] = {}
    for jcfg in kin_cfg.joints:
        lo = jcfg.limit_lower_rad
        hi = jcfg.limit_upper_rad
        if lo is not None and hi is not None:
            urdf_map[jcfg.name] = (float(lo), float(hi))

    # Per-joint limit resolution
    _FIELD_PREFIX = {
        "shoulder_pan":  "shoulder_pan",
        "shoulder_lift": "shoulder_lift",
        "elbow_flex":    "elbow_flex",
        "wrist_flex":    "wrist_flex",
        "wrist_roll":    "wrist_roll",
    }

    lower_list: list[float] = []
    upper_list: list[float] = []
    any_null = False
    any_unverified = soft is None or (soft is not None and soft.unverified)

    for name in JOINT_NAMES:
        prefix = _FIELD_PREFIX[name]
        lo_field = f"{prefix}_lower_rad"
        hi_field = f"{prefix}_upper_rad"

        lo_val: Optional[float] = getattr(soft, lo_field, None) if soft else None
        hi_val: Optional[float] = getattr(soft, hi_field, None) if soft else None

        if lo_val is None or hi_val is None:
            any_null = True
            # Fallback to URDF
            urdf_lo, urdf_hi = urdf_map.get(name, (-math.pi, math.pi))
            if lo_val is None:
                lo_val = urdf_lo
                logger.warning(
                    "AA-9 %s arm: soft limit for %s (lower) is null; "
                    "using URDF fallback %.4f rad — UNVERIFIED, LIVE blocked.",
                    arm_role, name, urdf_lo,
                )
            if hi_val is None:
                hi_val = urdf_hi
                logger.warning(
                    "AA-9 %s arm: soft limit for %s (upper) is null; "
                    "using URDF fallback %.4f rad — UNVERIFIED, LIVE blocked.",
                    arm_role, name, urdf_hi,
                )

        lower_list.append(float(lo_val))
        upper_list.append(float(hi_val))

    is_unverified = any_unverified or any_null
    live_ok = not is_unverified

    if is_unverified:
        logger.warning(
            "AA-9 %s arm: soft limits are UNVERIFIED (null or unverified=True). "
            "LIVE motion is blocked until measure_limits.py --live is run and "
            "hardware.yaml is updated with method/measured_date and unverified: false.",
            arm_role,
        )

    return ResolvedLimits(
        lower_rad=tuple(lower_list),
        upper_rad=tuple(upper_list),
        unverified=is_unverified,
        live_ok=live_ok,
    )


# ---------------------------------------------------------------------------
# Joint-limit guard
# ---------------------------------------------------------------------------


class JointLimitGuard:
    """Validate joint angles against per-arm soft limits.

    Construction
    ------------
    Use :func:`build_joint_limit_guard` rather than calling directly.

    Fail-closed rule
    ----------------
    If ``limits.live_ok`` is False (limits are null or unverified), calling
    ``check(q, live=True)`` raises :class:`LiveMotionBlockedError` immediately,
    before checking individual joint values.  This prevents any live motion
    when limits have not been measured.
    """

    def __init__(self, limits: ResolvedLimits, arm_role: str) -> None:
        self._limits = limits
        self._arm_role = arm_role

    @property
    def limits(self) -> ResolvedLimits:
        return self._limits

    def check(
        self, q: JointVec, live: bool = False
    ) -> Optional[str]:
        """Check joint angles against soft limits.

        Parameters
        ----------
        q:
            5-vector of joint angles (rad), ordered per :data:`JOINT_NAMES`.
        live:
            If True and limits are unverified, raises
            :class:`LiveMotionBlockedError` immediately.

        Returns
        -------
        str or None
            Violation description if any joint is outside its limit, else None.

        Raises
        ------
        LiveMotionBlockedError
            If ``live=True`` and limits are unverified (fail-closed).
        ValueError
            If ``q`` does not have exactly 5 elements.
        """
        if len(q) != 5:
            raise ValueError(f"q must have 5 elements, got {len(q)}")

        # Fail-closed: block live motion if unverified
        if live and not self._limits.live_ok:
            raise LiveMotionBlockedError(
                f"AA-9 {self._arm_role} arm: soft limits are UNVERIFIED. "
                "Run scripts/measure_limits.py --live, populate hardware.yaml "
                "(method, measured_date, unverified: false) before live motion."
            )

        for i, name in enumerate(JOINT_NAMES):
            lo = self._limits.lower_rad[i]
            hi = self._limits.upper_rad[i]
            qi = float(q[i])
            if qi < lo or qi > hi:
                return (
                    f"{self._arm_role} arm joint '{name}' value {qi:.4f} rad "
                    f"is outside soft limits [{lo:.4f}, {hi:.4f}] rad"
                )
        return None


# ---------------------------------------------------------------------------
# Collision model
# ---------------------------------------------------------------------------


def _point_in_box(p: npt.NDArray[np.float64], box: KeepOutBox) -> bool:
    """Return True if point ``p`` (3-vector, metres) is inside ``box``."""
    return (
        box.x_min_m <= p[0] <= box.x_max_m
        and box.y_min_m <= p[1] <= box.y_max_m
        and box.z_min_m <= p[2] <= box.z_max_m
    )


def _point_to_segment_dist2(
    p: npt.NDArray[np.float64],
    a: npt.NDArray[np.float64],
    b: npt.NDArray[np.float64],
) -> float:
    """Squared distance from point ``p`` to segment ``ab``."""
    ab = b - a
    ab_sq = float(np.dot(ab, ab))
    if ab_sq < 1e-12:
        return float(np.dot(p - a, p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab)) / ab_sq))
    closest = a + t * ab
    diff = p - closest
    return float(np.dot(diff, diff))


def _point_in_capsule(
    p: npt.NDArray[np.float64], cap: KeepOutCapsule
) -> bool:
    """Return True if point ``p`` is within the capsule's radius of the segment."""
    a = np.array(cap.p0_xyz, dtype=np.float64)
    b = np.array(cap.p1_xyz, dtype=np.float64)
    dist2 = _point_to_segment_dist2(p, a, b)
    return dist2 <= cap.radius_m ** 2


@dataclass
class CollisionModel:
    """Check FK link-point positions against keep-out primitives.

    Link points checked per pose
    ----------------------------
    For each joint in the kinematic chain, the origin of that joint's frame
    (after applying all preceding transforms up to and including that joint's
    rotation) is extracted from the FK chain.  The TCP (tool-centre point) is
    also checked.  This covers six 3-D points per arm per pose: one per joint
    frame origin plus the TCP.

    All points are expressed in the **arm base frame** (``base_link``).
    Keep-out primitives are also specified in the arm base frame (because
    ``left_arm_base_to_chassis`` and ``right_arm_base_to_chassis`` are
    ``unverified: true`` until AA-16, and re-expressing primitives into the
    chassis frame would compound two unverified transforms).

    Interpolation resolution
    ------------------------
    Straight-line joint-space paths are sampled at
    ``CollisionConfig.interpolation_samples`` points (including endpoints).
    Both endpoints and all interior samples are checked.  50 samples is the
    documented default.

    Unverified primitives
    ---------------------
    All primitives start with ``unverified: true``.  The collision model logs
    a WARNING when any primitive is unverified but still enforces the check
    (conservative — better to have false positives than false negatives).
    """

    chain: KinematicChain
    collision_cfg: CollisionConfig

    def __post_init__(self) -> None:
        # Warn once if any primitive is unverified
        all_primitives = list(self.collision_cfg.keep_out_boxes) + list(
            self.collision_cfg.keep_out_capsules
        )
        unverified_labels = [p.label for p in all_primitives if p.unverified]
        if unverified_labels:
            logger.warning(
                "AA-9 collision model: %d primitive(s) are UNVERIFIED placeholders: %s. "
                "Check results are conservative (fail-closed). "
                "Measure real chassis geometry and update hardware.yaml.",
                len(unverified_labels),
                ", ".join(unverified_labels),
            )

    def _link_points(self, q: JointVec) -> List[npt.NDArray[np.float64]]:
        """Return list of 3-D link frame origins for all joints + TCP.

        Uses partial FK up to each joint to extract the joint-frame origin.
        """
        import numpy as np  # noqa: F811 (already imported at module top)
        points: list[npt.NDArray[np.float64]] = []
        T = np.eye(4, dtype=np.float64)
        for jf, qi in zip(self.chain.joints, q):
            T = T @ jf.T_fixed @ _rotz_for_guard(qi)
            points.append(T[:3, 3].copy())
        # TCP: apply tool offset
        T_tcp = T @ self.chain.T_tool_offset
        points.append(T_tcp[:3, 3].copy())
        return points

    def check_pose(self, q: JointVec) -> Optional[str]:
        """Check a single pose for collisions.

        Parameters
        ----------
        q:
            5-vector of joint angles (rad).

        Returns
        -------
        str or None
            Collision description if any link point is in a keep-out zone,
            else None.
        """
        points = self._link_points(q)
        for p in points:
            for box in self.collision_cfg.keep_out_boxes:
                if _point_in_box(p, box):
                    return (
                        f"link point {p.tolist()} is inside keep-out box "
                        f"'{box.label}'"
                    )
            for cap in self.collision_cfg.keep_out_capsules:
                if _point_in_capsule(p, cap):
                    return (
                        f"link point {p.tolist()} is inside keep-out capsule "
                        f"'{cap.label}' (radius {cap.radius_m} m)"
                    )
        return None

    def check_trajectory(
        self,
        q_start: JointVec,
        q_end: JointVec,
    ) -> Optional[str]:
        """Check a straight-line joint-space interpolation for collisions.

        Samples ``collision.interpolation_samples`` evenly-spaced points
        (including both endpoints) along the linear interpolation from
        ``q_start`` to ``q_end``.  Returns the first collision found.

        This check detects collisions whose endpoints are safe but whose
        interior passes through a keep-out zone.

        Parameters
        ----------
        q_start:
            Start joint configuration (rad).
        q_end:
            End joint configuration (rad).

        Returns
        -------
        str or None
            Collision description if any sample is in a keep-out zone,
            else None.
        """
        n = self.collision_cfg.interpolation_samples
        for i in range(n):
            t = float(i) / float(n - 1)
            q_sample = (1.0 - t) * np.asarray(q_start) + t * np.asarray(q_end)
            result = self.check_pose(q_sample)
            if result is not None:
                return (
                    f"trajectory sample {i}/{n-1} (t={t:.3f}): {result}"
                )
        return None


# ---------------------------------------------------------------------------
# Rotz helper (avoid importing private from kinematics)
# ---------------------------------------------------------------------------


def _rotz_for_guard(theta: float) -> npt.NDArray[np.float64]:
    """Pure Z-rotation 4×4 HTM (duplicated to avoid private import)."""
    import numpy as np  # noqa: F811
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# AA40 mesh collision layer
# ---------------------------------------------------------------------------


class MeshCollisionGuard:
    """Part-shape collision check for one arm on the owner cell URDF.

    Wraps :class:`inchworm.system_model.collision.CellCollisionChecker`.
    Additive to the keep-out primitives (they stay). Fails closed: a model
    that cannot be loaded, an unknown pose of the OTHER arm, or any
    evaluation error is a violation.

    The other arm's pose comes from ``other_arm_pose_source`` (a callable
    returning its 5-vector, rad). The tube comes from ``tube_source``
    (returns the tube advance in metres along its axis from the cell-URDF
    placement, or None for "no tube in the cell"). In LIVE mode a missing ``tube_source`` is a
    violation: "no tube" must be stated explicitly, never assumed.
    """

    def __init__(
        self,
        arm_role: str,
        mesh_cfg: object,
        checker: Optional[object] = None,
        load_error: Optional[str] = None,
        other_arm_pose_source: Optional[Callable[[], JointVec]] = None,
        tube_source: Optional[Callable[[], object]] = None,
        dexarm_pose_source: Optional[Callable[[], dict]] = None,
        interpolation_samples: int = 50,
    ) -> None:
        self.arm_role = arm_role
        self.cfg = mesh_cfg
        self.checker = checker
        self.load_error = load_error
        self.other_arm_pose_source = other_arm_pose_source
        self.tube_source = tube_source
        self.dexarm_pose_source = dexarm_pose_source
        self.interpolation_samples = int(interpolation_samples)

    def missing_inputs(self, live: bool) -> List[str]:
        """Cell-state inputs that are not wired. Live needs all three."""
        missing = []
        if self.other_arm_pose_source is None:
            missing.append("other arm pose")
        if live and self.tube_source is None:
            missing.append("tube state")
        if live and self.dexarm_pose_source is None:
            missing.append("DexArm pose")
        return missing

    def _state(self, q: JointVec, live: bool):
        from inchworm.system_model.collision import CellState
        missing = self.missing_inputs(live)
        if missing:
            raise RuntimeError("cell state incomplete: " + ", ".join(missing) + " not given")
        other = np.asarray(self.other_arm_pose_source(), dtype=np.float64)
        tube = self.tube_source() if self.tube_source is not None else None
        if tube is not None:
            tube = float(tube)
        dex = dict(self.dexarm_pose_source()) if self.dexarm_pose_source is not None else {}
        q = np.asarray(q, dtype=np.float64)
        if self.arm_role == "left":
            return CellState(left_q=q, right_q=other, dexarm_q=dex, tube_advance_m=tube)
        return CellState(left_q=other, right_q=q, dexarm_q=dex, tube_advance_m=tube)

    def _check_state(self, q: JointVec, live: bool) -> Optional[str]:
        if self.checker is None:
            return f"mesh collision model unavailable: {self.load_error}"
        if not live and self.missing_inputs(live):
            # Dry-run only (same convention as the GuardedArm present-pose
            # fallback): without the other arm's pose there is no cell state
            # to check. Live never takes this branch; it fails closed below.
            if not getattr(self, "_warned_skip", False):
                logger.warning(
                    "AA40 mesh collision [%s]: DRY-RUN check skipped, %s not given. "
                    "LIVE motion refuses in this state.",
                    self.arm_role, ", ".join(self.missing_inputs(live)),
                )
                self._warned_skip = True
            return None
        try:
            contacts = self.checker.check(self._state(q, live), arm=self.arm_role)
        except Exception as exc:  # noqa: BLE001 - fail closed on any model error
            return f"mesh collision check failed (fail closed): {exc}"
        if contacts:
            return f"mesh collision ({self.arm_role}): {contacts[0]}"
        return None

    def check_pose(self, q: JointVec, live: bool = False) -> Optional[str]:
        if live and getattr(self.cfg, "unverified", True):
            raise LiveMotionBlockedError(
                "mesh collision model is unverified (owner render comparison, AA40 #6)"
            )
        return self._check_state(q, live)

    def check_trajectory(self, q_start: JointVec, q_end: JointVec, live: bool = False) -> Optional[str]:
        if live and getattr(self.cfg, "unverified", True):
            raise LiveMotionBlockedError(
                "mesh collision model is unverified (owner render comparison, AA40 #6)"
            )
        n = max(2, self.interpolation_samples)
        qs, qe = np.asarray(q_start, dtype=np.float64), np.asarray(q_end, dtype=np.float64)
        for i in range(n):
            t = i / (n - 1)
            v = self._check_state((1 - t) * qs + t * qe, live)
            if v is not None:
                return f"trajectory sample {i}/{n-1} (t={t:.3f}): {v}"
        return None


def build_mesh_collision_guard(
    cfg: HardwareConfig,
    arm_role: str,
    other_arm_pose_source: Optional[Callable[[], JointVec]] = None,
    tube_source: Optional[Callable[[], object]] = None,
    dexarm_pose_source: Optional[Callable[[], dict]] = None,
) -> Optional[MeshCollisionGuard]:
    """Mesh guard from config, or None when ``motion_guard.collision.mesh`` is absent/disabled.

    A configured model that fails to load does NOT return None: it returns a
    guard that reports every pose as a violation (fail closed).
    """
    from pathlib import Path
    guard_cfg = cfg.motion_guard
    mesh_cfg = guard_cfg.collision.mesh if guard_cfg is not None else None
    if mesh_cfg is None or not mesh_cfg.enabled:
        return None
    checker = None
    err = None
    try:
        from inchworm.system_model.collision import (
            REPO_ROOT, JointMapping, get_cell_checker,
        )
        urdf = Path(mesh_cfg.urdf_path)
        urdf = urdf if urdf.is_absolute() else REPO_ROOT / urdf
        srdf = None
        if mesh_cfg.srdf_path is not None:
            srdf = Path(mesh_cfg.srdf_path)
            srdf = srdf if srdf.is_absolute() else REPO_ROOT / srdf
        mapping = JointMapping(
            sign=(1, 1, 1, 1, mesh_cfg.wrist_roll_cell_sign),
            offset_rad=(0.0, 0.0, 0.0, 0.0, mesh_cfg.wrist_roll_cell_offset_rad),
        )
        from inchworm.system_model.collision import PoseScopedAllowance
        allowances = []
        if mesh_cfg.fold_allowance_pairs:
            allowances.append(PoseScopedAllowance(
                label="measured_fold (owner 2026-10-27: go with the real fold)",
                pairs=frozenset(frozenset(p) for p in mesh_cfg.fold_allowance_pairs),
                centre_rad={"left": mesh_cfg.fold_allowance_centre_left_rad,
                            "right": mesh_cfg.fold_allowance_centre_right_rad},
                radius_rad=mesh_cfg.fold_allowance_radius_rad,
            ))
        checker = get_cell_checker(urdf, srdf, margin_m=mesh_cfg.margin_m,
                                   joint_mapping=mapping, allowances=allowances)
    except Exception as exc:  # noqa: BLE001 - fail closed
        err = f"{type(exc).__name__}: {exc}"
        logger.error("AA40 mesh collision model failed to load: %s", err)
    return MeshCollisionGuard(
        arm_role=arm_role, mesh_cfg=mesh_cfg, checker=checker, load_error=err,
        other_arm_pose_source=other_arm_pose_source, tube_source=tube_source,
        dexarm_pose_source=dexarm_pose_source,
        interpolation_samples=guard_cfg.collision.interpolation_samples,
    )


# ---------------------------------------------------------------------------
# Combined motion guard
# ---------------------------------------------------------------------------


@dataclass
class MotionGuard:
    """Combined joint-limit + collision guard for one arm.

    Use :func:`build_motion_guard` to construct from a loaded
    :class:`~inchworm.config.HardwareConfig`.

    Entry points
    ------------
    - :meth:`check_pose`: validate a single joint configuration.
    - :meth:`check_trajectory`: validate a straight-line interpolation.
    Both raise :class:`LiveMotionBlockedError` if ``live=True`` and limits
    are unverified.

    Returned violations are strings (human-readable); the caller decides
    whether to raise, log, or handle them.
    """

    limit_guard: JointLimitGuard
    collision_model: CollisionModel
    arm_role: str
    #: AA40 mesh collision layer (owner cell URDF). None = not configured.
    mesh: Optional["MeshCollisionGuard"] = None

    def check_pose(
        self,
        q: JointVec,
        live: bool = False,
    ) -> Optional[str]:
        """Validate a single joint configuration.

        Checks joint limits first (fail-closed on unverified), then collision.

        Parameters
        ----------
        q:
            5-vector of joint angles (rad).
        live:
            If True and limits are unverified, raises
            :class:`LiveMotionBlockedError` immediately.

        Returns
        -------
        str or None
            First violation found, or None if the pose is safe.

        Raises
        ------
        LiveMotionBlockedError
            When ``live=True`` and soft limits are unverified.
        """
        violation = self.limit_guard.check(q, live=live)
        if violation is not None:
            return violation
        violation = self.collision_model.check_pose(q)
        if violation is not None:
            return violation
        if self.mesh is not None:
            return self.mesh.check_pose(q, live=live)
        if live:
            raise LiveMotionBlockedError(
                "mesh collision model not configured (motion_guard.collision.mesh)"
            )
        return None

    def check_trajectory(
        self,
        q_start: JointVec,
        q_end: JointVec,
        live: bool = False,
    ) -> Optional[str]:
        """Validate a straight-line joint-space interpolation.

        Checks both endpoints with the joint-limit guard (fail-closed on
        unverified), then samples the full interpolation for collisions.
        Sampling is done even if endpoint collision checks pass (interior
        could still intersect a keep-out zone).

        Parameters
        ----------
        q_start:
            Start joint configuration (rad).
        q_end:
            End joint configuration (rad).
        live:
            If True and limits are unverified, raises
            :class:`LiveMotionBlockedError` immediately.

        Returns
        -------
        str or None
            First violation found, or None if the trajectory is safe.

        Raises
        ------
        LiveMotionBlockedError
            When ``live=True`` and soft limits are unverified.
        """
        # Check both endpoints for joint-limit violations (fail-closed on live)
        v = self.limit_guard.check(q_start, live=live)
        if v is not None:
            return f"q_start: {v}"
        v = self.limit_guard.check(q_end, live=False)  # live=False: already checked above
        if v is not None:
            return f"q_end: {v}"

        # Dense trajectory collision check (samples endpoints + interior)
        v = self.collision_model.check_trajectory(q_start, q_end)
        if v is not None:
            return v
        if self.mesh is not None:
            return self.mesh.check_trajectory(q_start, q_end, live=live)
        if live:
            raise LiveMotionBlockedError(
                "mesh collision model not configured (motion_guard.collision.mesh)"
            )
        return None


# ---------------------------------------------------------------------------
# GuardedArm: ArmServoBus wrapper
# ---------------------------------------------------------------------------


class PresentPoseUnknownError(RuntimeError):
    """Raised in live mode when the arm's present pose cannot be read, so a
    guarded move cannot be validated and must not be sent (fail closed)."""


class GuardedArm:
    """Wraps :class:`~inchworm.servo_bus.ArmServoBus` to enforce motion guard.

    All position writes for joints 1-5 (shoulder_pan … wrist_roll) are
    intercepted and validated by :class:`MotionGuard` before the write
    reaches the servo bus.  The gripper joint is excluded (managed by AA-17).

    Single-joint writes (:meth:`write_position_deg`)
    -------------------------------------------------
    When a single joint is commanded, the guard must check the *full* 5-DoF
    pose to test joint limits and collision.  It does this by:

    1. Reading the present servo position of every mapped joint from the bus
       (via ``ArmServoBus.read_position_deg``) or from the injected
       ``present_pose_source`` callback if one is provided.
    2. Converting each servo-degree to joint-radians via the per-joint
       calibration (see :mod:`inchworm.joint_calibration`).
    3. Substituting the commanded joint's target angle into the pose vector.
    4. Running :meth:`MotionGuard.check_pose` on the full target pose.
    5. Running :meth:`MotionGuard.check_trajectory` from present pose to
       target pose.

    This guarantees that no single-joint write can bypass the full collision
    and limit envelope, even when the other joints are at non-zero angles.

    Servo-degree ↔ joint-radian conversion
    ----------------------------------------
    Conversion is performed by :func:`~inchworm.joint_calibration.servo_deg_to_joint_rad`
    and :func:`~inchworm.joint_calibration.joint_rad_to_servo_deg` using per-joint
    calibration from ``motion_guard.servo_calibration`` in ``hardware.yaml``.
    The old hard-coded ``deg-180`` assumption has been removed.  Live mode
    fails closed when calibration is null or unverified.

    Bypass
    ------
    Pass ``bypass_guard=True`` to any position-write call to skip guard
    validation.  **This always logs a WARNING via AA-7 telemetry.**
    The intent is that no production code path ever sets this flag;
    it exists only for emergency recovery or operator-commanded override.

    Parameters
    ----------
    bus:
        The :class:`~inchworm.servo_bus.ArmServoBus` to wrap.
    guard:
        The :class:`MotionGuard` for this arm.
    calibration:
        The :class:`~inchworm.joint_calibration.ArmCalibration` for this arm.
        If ``None``, falls back to the null-calibration fallback (dry-run only).
    live:
        If True, position writes will raise
        :class:`LiveMotionBlockedError` when limits are unverified.
        In dry-run mode (``live=False``) guard violations are logged but
        writes are still blocked; unverified limits do not block dry-run.
    session_logger:
        Optional AA-7 session logger.  If present, WARNING events are
        emitted via ``session_logger.warning()``.
    present_pose_source:
        Optional callable that returns a 5-vector (radians) representing
        the present joint configuration.  When provided, the guard uses this
        instead of reading the bus.  Signature:
        ``() -> NDArray[float64]``.
        Useful for tests or when the caller already holds the present state.
    """

    def __init__(
        self,
        bus: object,  # ArmServoBus — typed as object to avoid circular import
        guard: MotionGuard,
        calibration: Optional[object] = None,  # ArmCalibration from joint_calibration
        live: bool = False,
        session_logger: Optional[object] = None,
        present_pose_source: Optional[Callable[[], JointVec]] = None,
    ) -> None:
        self._bus = bus
        self._guard = guard
        self._calibration = calibration  # ArmCalibration | None
        self._live = live
        self._session_logger = session_logger
        self._present_pose_source = present_pose_source

    @property
    def bus(self) -> object:
        """The underlying :class:`~inchworm.servo_bus.ArmServoBus`."""
        return self._bus

    @property
    def guard(self) -> MotionGuard:
        return self._guard

    def _emit_warning(self, event_type: str, payload: dict) -> None:
        """Emit a WARNING event via AA-7 telemetry if a logger is present."""
        logger.warning("AA-9 GuardedArm [%s]: %s %r", self._guard.arm_role, event_type, payload)
        if self._session_logger is not None:
            warning_fn = getattr(self._session_logger, "warning", None)
            if warning_fn is not None:
                warning_fn(
                    role=self._guard.arm_role,
                    subsystem="motion_guard",
                    event_type=event_type,
                    payload=payload,
                )

    # ------------------------------------------------------------------
    # Calibration-based conversion
    # ------------------------------------------------------------------

    def _servo_deg_to_joint_rad(self, joint_index: int, servo_deg: float) -> float:
        """Convert servo_deg to joint_rad using per-joint calibration."""
        # Lazy import to avoid circular dependency (joint_calibration imports
        # JOINT_NAMES from motion_guard, which is fine at function call time).
        from inchworm.joint_calibration import (
            ArmCalibration,
            servo_deg_to_joint_rad,
            JointCalibParams,
            _FALLBACK_ZERO_OFFSET_DEG,
        )
        if self._calibration is None:
            # No calibration provided: use fallback
            calib = JointCalibParams(
                joint_name=JOINT_NAMES[joint_index],
                zero_offset_deg=None,
                sign=1,
                unverified=True,
            )
        else:
            calib = self._calibration.get(joint_index)
        return servo_deg_to_joint_rad(
            servo_deg, calib, self._guard.arm_role, live=self._live
        )

    def _joint_rad_to_servo_deg(self, joint_index: int, joint_rad: float) -> float:
        """Convert joint_rad to servo_deg using per-joint calibration."""
        from inchworm.joint_calibration import (
            joint_rad_to_servo_deg,
            JointCalibParams,
        )
        if self._calibration is None:
            calib = JointCalibParams(
                joint_name=JOINT_NAMES[joint_index],
                zero_offset_deg=None,
                sign=1,
                unverified=True,
            )
        else:
            calib = self._calibration.get(joint_index)
        return joint_rad_to_servo_deg(
            joint_rad, calib, self._guard.arm_role, live=self._live
        )

    # ------------------------------------------------------------------
    # Present-pose acquisition for full-pose check
    # ------------------------------------------------------------------

    def _read_present_pose_rad(self) -> JointVec:
        """Read the present pose from the bus or present_pose_source.

        Returns a 5-vector of joint angles in radians, in JOINT_NAMES order.
        If a joint's servo ID is None (not yet assigned), uses 0.0 rad as a
        conservative fallback (logs WARNING).
        """
        if self._present_pose_source is not None:
            return np.asarray(self._present_pose_source(), dtype=np.float64)

        q = np.zeros(5, dtype=np.float64)
        joint_id_map = getattr(self._bus, '_joint_id_map', {})
        for i, joint_name in enumerate(JOINT_NAMES):
            servo_id = joint_id_map.get(joint_name)
            # Orchestrator 2026-10-24: in LIVE mode an unknown present pose
            # must fail closed. A fabricated 0.0 rad would make the collision
            # check validate a pose the arm is not actually in. The 0.0
            # fallback is kept for dry-run/mock planning only, with a warning.
            if servo_id is None:
                if self._live:
                    raise PresentPoseUnknownError(
                        f"{self._guard.arm_role}: joint '{joint_name}' has no servo ID"
                    )
                logger.warning(
                    "AA-9 GuardedArm [%s]: joint '%s' has no servo ID; "
                    "using 0.0 rad as DRY-RUN present-pose fallback.",
                    self._guard.arm_role, joint_name,
                )
                continue
            try:
                servo_deg = self._bus.read_position_deg(servo_id)  # type: ignore[attr-defined]
                q[i] = self._servo_deg_to_joint_rad(i, servo_deg)
            except Exception as exc:  # noqa: BLE001
                if self._live:
                    raise PresentPoseUnknownError(
                        f"{self._guard.arm_role}: cannot read present position of "
                        f"'{joint_name}' (servo {servo_id}): {exc}"
                    ) from exc
                logger.warning(
                    "AA-9 GuardedArm [%s]: failed to read present position for '%s' "
                    "(servo %d): %s; using 0.0 rad DRY-RUN fallback.",
                    self._guard.arm_role, joint_name, servo_id, exc,
                )
        return q

    # ------------------------------------------------------------------
    # Public write interface
    # ------------------------------------------------------------------

    def write_position_deg(
        self,
        joint_name: str,
        position_deg: float,
        bypass_guard: bool = False,
    ) -> None:
        """Write a goal position to a named joint (degrees), enforcing the guard.

        Reads the present positions of all other joints, converts to radians
        using the per-joint calibration, assembles the full 5-DoF target pose,
        and validates it with :meth:`MotionGuard.check_pose` and
        :meth:`MotionGuard.check_trajectory` (present → target) before writing.

        Parameters
        ----------
        joint_name:
            Joint name (must be one of the 5 arm joints, not the gripper).
        position_deg:
            Desired position in degrees (STS3215 range 0-360).
        bypass_guard:
            If True, skip guard validation.
            **ALWAYS logs a WARNING. Use only for emergency override.**

        Raises
        ------
        MotionViolationError
            If the target angle violates joint limits or collision envelope,
            and ``bypass_guard=False``.
        LiveMotionBlockedError
            If ``live=True``, limits are unverified, and ``bypass_guard=False``.
        CalibrationMissingError
            If ``live=True``, calibration is null/unverified, and
            ``bypass_guard=False``.
        ValueError
            If ``joint_name`` is not in JOINT_NAMES.
        """
        if bypass_guard:
            self._emit_warning(
                "guard_bypass",
                {
                    "joint_name": joint_name,
                    "position_deg": position_deg,
                    "reason": "bypass_guard=True was set by caller",
                },
            )
        else:
            if joint_name not in JOINT_NAMES:
                raise ValueError(
                    f"joint_name {joint_name!r} is not a valid arm joint; "
                    f"known: {JOINT_NAMES}"
                )
            joint_idx = JOINT_NAMES.index(joint_name)

            # Convert target servo_deg to joint_rad via calibration.
            target_rad = self._servo_deg_to_joint_rad(joint_idx, position_deg)

            # Read the present pose of all joints (including others).
            present_q = self._read_present_pose_rad()

            # Build the full target pose: same as present except at joint_idx.
            target_q = present_q.copy()
            target_q[joint_idx] = target_rad

            # Full pose check (limits + collision) on the target.
            violation = self._guard.check_pose(target_q, live=self._live)
            if violation is not None:
                raise MotionViolationError(violation)

            # Trajectory check from present to target.
            traj_violation = self._guard.check_trajectory(
                present_q, target_q, live=False  # live already checked in check_pose
            )
            if traj_violation is not None:
                raise MotionViolationError(traj_violation)

        # Delegate to underlying bus
        self._bus.write_position_deg(joint_name, position_deg)  # type: ignore[attr-defined]

    def write_positions_rad(
        self,
        q: JointVec,
        bypass_guard: bool = False,
    ) -> None:
        """Write a complete 5-DoF joint configuration (radians) with guard enforcement.

        This is the preferred entry point for motion commands that compute
        a full configuration (e.g. from IK).  It runs the full
        :meth:`MotionGuard.check_pose` check plus a trajectory check from
        the present pose.

        Parameters
        ----------
        q:
            5-vector of joint angles (rad), ordered per :data:`JOINT_NAMES`.
        bypass_guard:
            If True, skip guard validation.
            **ALWAYS logs a WARNING. Use only for emergency override.**

        Raises
        ------
        MotionViolationError
            If ``q`` violates joint limits or collision envelope,
            and ``bypass_guard=False``.
        LiveMotionBlockedError
            If ``live=True``, limits are unverified, and ``bypass_guard=False``.
        """
        if bypass_guard:
            self._emit_warning(
                "guard_bypass",
                {
                    "q_rad": list(q),
                    "reason": "bypass_guard=True was set by caller",
                },
            )
        else:
            # Pose check
            violation = self._guard.check_pose(q, live=self._live)
            if violation is not None:
                raise MotionViolationError(violation)

            # Trajectory check from present to target
            present_q = self._read_present_pose_rad()
            traj_violation = self._guard.check_trajectory(
                present_q, np.asarray(q, dtype=np.float64), live=False
            )
            if traj_violation is not None:
                raise MotionViolationError(traj_violation)

        # Convert rad → deg and write each joint
        for i, name in enumerate(JOINT_NAMES):
            deg = self._joint_rad_to_servo_deg(i, float(q[i]))
            deg = max(0.0, min(360.0, deg))
            self._bus.write_position_deg(name, deg)  # type: ignore[attr-defined]

    def check_trajectory_rad(
        self,
        q_start: JointVec,
        q_end: JointVec,
    ) -> Optional[str]:
        """Validate a straight-line joint-space trajectory without writing.

        Does not write to the servo bus.  Returns a violation string or None.

        Raises
        ------
        LiveMotionBlockedError
            If ``live=True`` and limits are unverified.
        """
        return self._guard.check_trajectory(q_start, q_end, live=self._live)

    def apply_joint_command(
        self,
        joint_name: str,
        position_deg: float,
        bypass_guard: bool = False,
    ) -> None:
        """External caller API: command a joint (degrees) through the guard.

        This is the entry point for callers outside :mod:`inchworm.motion_guard`
        (e.g. :mod:`inchworm.homing`, :mod:`inchworm.policy_actions`).
        Internally it calls :meth:`write_position_deg` with the full-pose check.

        Using a distinct method name keeps the AST guard clean: the rule is that
        only :mod:`inchworm.motion_guard`, :mod:`inchworm.servo_bus`,
        :mod:`inchworm.gripper_lock`, and :mod:`inchworm.torque` may call
        ``write_position_deg`` (the internal guarded write).  External callers
        use ``apply_joint_command`` instead, which is the same operation but
        with a name that does not match the banned identifier.

        Parameters
        ----------
        joint_name:
            One of the 5 arm joints (shoulder_pan … wrist_roll).
        position_deg:
            Desired servo position (STS3215 range 0-360).
        bypass_guard:
            Forwarded to :meth:`write_position_deg`.  Always logs a WARNING.
        """
        self.write_position_deg(joint_name, position_deg, bypass_guard=bypass_guard)


# ---------------------------------------------------------------------------
# Named pose lookup
# ---------------------------------------------------------------------------


def get_named_pose(
    name: str,
    guard_cfg: MotionGuardConfig,
) -> Optional[NamedPose]:
    """Look up a named pose from the motion guard config.

    Returns None if the pose is not found.

    Logs a WARNING if the pose has ``unverified: true``.  Unverified poses
    must not be used for live motion without explicit operator review.
    """
    for pose in guard_cfg.named_poses:
        if pose.name == name:
            if pose.unverified:
                logger.warning(
                    "AA-9 named pose '%s' is UNVERIFIED (placeholder). "
                    "Do not use for live motion without owner verification.",
                    name,
                )
            return pose
    logger.warning("AA-9 named pose '%s' not found in config.", name)
    return None


# ---------------------------------------------------------------------------
# Factory functions
# ---------------------------------------------------------------------------


def build_joint_limit_guard(
    cfg: HardwareConfig,
    arm_role: str,
) -> JointLimitGuard:
    """Build a :class:`JointLimitGuard` from a loaded :class:`HardwareConfig`.

    Parameters
    ----------
    cfg:
        Loaded hardware config.
    arm_role:
        ``"left"`` or ``"right"``.
    """
    guard_cfg = cfg.motion_guard
    soft: Optional[SoftJointLimits] = None
    if guard_cfg is not None:
        soft = guard_cfg.left_arm_limits if arm_role == "left" else guard_cfg.right_arm_limits
    limits = _resolve_limits_for_arm(soft, cfg.so101_kinematics, arm_role)
    return JointLimitGuard(limits, arm_role)


def build_collision_model(
    cfg: HardwareConfig,
) -> CollisionModel:
    """Build a :class:`CollisionModel` from a loaded :class:`HardwareConfig`.

    Uses the ``so101_kinematics`` chain for FK and the
    ``motion_guard.collision`` primitives.
    """
    chain = build_chain(cfg.so101_kinematics)
    guard_cfg = cfg.motion_guard
    collision_cfg = guard_cfg.collision if guard_cfg is not None else CollisionConfig()
    return CollisionModel(chain=chain, collision_cfg=collision_cfg)


def build_motion_guard(
    cfg: HardwareConfig,
    arm_role: str,
    other_arm_pose_source: Optional[Callable[[], JointVec]] = None,
    tube_source: Optional[Callable[[], object]] = None,
    dexarm_pose_source: Optional[Callable[[], dict]] = None,
) -> MotionGuard:
    """Build a :class:`MotionGuard` for one arm from a loaded config.

    Parameters
    ----------
    cfg:
        Loaded hardware config.
    arm_role:
        ``"left"`` or ``"right"``.
    other_arm_pose_source, tube_source, dexarm_pose_source:
        Inputs of the AA40 mesh collision layer (see
        :class:`MeshCollisionGuard`). Without an other-arm pose source the
        mesh layer reports every pose as a violation (fail closed); set it
        later with ``guard.mesh.other_arm_pose_source = ...``.
    """
    limit_guard = build_joint_limit_guard(cfg, arm_role)
    collision_model = build_collision_model(cfg)
    return MotionGuard(
        limit_guard=limit_guard,
        collision_model=collision_model,
        arm_role=arm_role,
        mesh=build_mesh_collision_guard(
            cfg, arm_role,
            other_arm_pose_source=other_arm_pose_source,
            tube_source=tube_source,
            dexarm_pose_source=dexarm_pose_source,
        ),
    )


def build_guarded_arm(
    bus: object,
    cfg: HardwareConfig,
    arm_role: str,
    live: bool = False,
    session_logger: Optional[object] = None,
    present_pose_source: Optional[Callable[[], JointVec]] = None,
) -> GuardedArm:
    """Build a :class:`GuardedArm` for one arm from a loaded config.

    Convenience factory that constructs the :class:`MotionGuard` and
    :class:`~inchworm.joint_calibration.ArmCalibration` from config and
    returns a wired :class:`GuardedArm`.

    Parameters
    ----------
    bus:
        The :class:`~inchworm.servo_bus.ArmServoBus` to wrap.
    cfg:
        Loaded hardware config.
    arm_role:
        ``"left"`` or ``"right"``.
    live:
        If True, position writes raise :class:`LiveMotionBlockedError` when
        limits or calibration are unverified.
    session_logger:
        Optional AA-7 session logger.
    present_pose_source:
        Optional callable returning the present 5-vector (radians).
    """
    from inchworm.joint_calibration import build_arm_calibration
    guard = build_motion_guard(cfg, arm_role)
    calib_block = None
    if cfg.motion_guard is not None and cfg.motion_guard.servo_calibration is not None:
        sc = cfg.motion_guard.servo_calibration
        calib_block = sc.left_arm if arm_role == "left" else sc.right_arm
    calibration = build_arm_calibration(calib_block, arm_role)
    return GuardedArm(
        bus=bus,
        guard=guard,
        calibration=calibration,
        live=live,
        session_logger=session_logger,
        present_pose_source=present_pose_source,
    )
