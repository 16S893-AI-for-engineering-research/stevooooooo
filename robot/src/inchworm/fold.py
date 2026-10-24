"""AA-19: Folded transit pose and fold/unfold sequencing.

After each regrip both arms must return to the folded pose visible in
``pictures/robot.png``.  The fold serves two purposes:

1. It brings the part as close as possible to the DexArm print head.
2. It is the pose from which the system expects repeatable geometry for
   levelling and printing.

Because the folded configuration is tight, the order in which joints move
matters.  A naive simultaneous interpolation can collide.  This module
enforces the documented joint ORDER with individually collision-checked
intermediate waypoints.

Design principles
-----------------
* **Injected protocols** — :class:`ArmMoverProtocol` and
  :class:`GripperStatusProtocol` are structural ``Protocol`` classes so
  callers can pass any compatible object (a real :class:`GuardedArm`, a stub,
  a mock).  This avoids importing ``GuardedArm`` directly and insulates the
  implementation from parallel-lane changes to GuardedArm internals (AA-9).

* **Dry-run by default** — every motion entry point takes a ``dry_run=True``
  default.  Live motion requires ``dry_run=False`` explicitly.

* **Fail-closed preconditions** — folding is refused unless both grippers
  report ``LOCKED`` via the injected :class:`GripperStatusProtocol`.
  Attempting to fold with an unlocked gripper raises :class:`FoldRefused`
  with a typed reason.

* **Per-waypoint + per-segment guard** — before each ``move_to`` call:
  1. :meth:`MotionGuard.check_pose` validates the target waypoint.
  2. :meth:`MotionGuard.check_trajectory` validates the straight-line
     interpolation from the previous waypoint to the next.
  A violation at any step aborts the sequence and raises
  :class:`FoldMotionViolation`.

* **Never simultaneous interpolation** — the waypoint list is executed
  sequentially; each waypoint completes before the next begins.

* **AA-7 telemetry** — structured events are emitted via an optional
  injected :class:`~inchworm.telemetry.SessionLogger`.

Joint ORDER (documented)
------------------------
The fold sequence visits configurations in proximal-to-distal order to keep
link clearance:

1. Raise ``shoulder_lift`` first — lifts the upper arm clear of the chassis
   base before any forearm movement.
2. Tuck ``elbow_flex`` — folds the forearm toward the upper arm.
3. Adjust ``wrist_flex`` / ``wrist_roll`` — fine-tune the wrist for compact
   transit.
4. Move ``shoulder_pan`` last — lateral alignment once the arm is tucked.

The exact intermediate configurations are stored as an ordered list in
``hardware.yaml`` under the ``fold_sequence`` key (all ``unverified: true``
until a hardware session confirms them).  See ``config.FoldSequenceConfig``.

The unfold sequence reverses the intent: distal joints relax first, then the
shoulder is lowered to a coarse-approach-ready state.

Safety invariants (doc-1, AGENTS.md)
-------------------------------------
* Dry-run by default.  Pass ``dry_run=False`` only after owner supervision.
* The gripper servo is never commanded here (AA-17 invariant).
* All motion goes through :class:`MotionGuard` (AA-9 invariant).
* Live execution fails closed on unverified waypoints.

Usage::

    from inchworm.fold import FoldSequencer, build_fold_sequencer
    from inchworm.config import load_config
    from inchworm.motion_guard import build_motion_guard

    cfg = load_config()
    guard_left = build_motion_guard(cfg, "left")
    guard_right = build_motion_guard(cfg, "right")

    sequencer = build_fold_sequencer(cfg, guard_left, guard_right,
                                     mover_left=my_left_arm,
                                     mover_right=my_right_arm,
                                     gripper_left=left_sm,
                                     gripper_right=right_sm)
    sequencer.fold(dry_run=True)   # dry-run: guard-checked, no servo writes
    sequencer.unfold(dry_run=True)
"""

from __future__ import annotations

import logging
from typing import List, Optional, Protocol, Sequence, runtime_checkable

import numpy as np
import numpy.typing as npt

from inchworm.config import FoldSequenceConfig, FoldWaypoint, HardwareConfig
from inchworm.motion_guard import (
    MotionGuard,
    MotionViolationError,
    get_named_pose,
)

logger = logging.getLogger(__name__)

# Maximum characters of waypoint description logged per event (avoids spamming
# telemetry with long strings). Not a design value; purely a logging concern.
_DESC_TRUNCATE: int = 20 * 4  # 80 chars

# ---------------------------------------------------------------------------
# Type alias
# ---------------------------------------------------------------------------

JointVec = npt.NDArray[np.float64]  # 5-vector of joint angles (rad)

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class FoldError(Exception):
    """Base class for all fold/unfold errors."""


class FoldRefused(FoldError):
    """Raised when folding is refused because a precondition is not met.

    Attributes
    ----------
    reason:
        Human-readable description of why the fold was refused.
    """

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"fold refused: {reason}")


class FoldMotionViolation(FoldError):
    """Raised when a fold/unfold waypoint or segment violates the motion guard.

    Attributes
    ----------
    step:
        Zero-based waypoint index where the violation occurred.
    violation:
        The violation string from :class:`~inchworm.motion_guard.MotionGuard`.
    arm_role:
        Which arm triggered the violation (``"left"`` or ``"right"``).
    """

    def __init__(self, step: int, violation: str, arm_role: str) -> None:
        self.step = step
        self.violation = violation
        self.arm_role = arm_role
        super().__init__(
            f"fold motion violation on {arm_role} arm at step {step}: {violation}"
        )


class UnverifiedWaypointError(FoldError):
    """Raised when a live fold is attempted with an unverified waypoint.

    Unverified waypoints must not be used for live motion without an
    owner-supervised hardware session that confirms their safety.  Set
    ``unverified: false`` in hardware.yaml after confirmation.
    """

    def __init__(self, step: int, arm_role: str) -> None:
        self.step = step
        self.arm_role = arm_role
        super().__init__(
            f"fold waypoint {step} on {arm_role} arm is UNVERIFIED. "
            "Verify on hardware and set unverified: false in fold_sequence config "
            "before running live."
        )


# ---------------------------------------------------------------------------
# Protocols (structural typing — no import of GuardedArm needed)
# ---------------------------------------------------------------------------


@runtime_checkable
class ArmMoverProtocol(Protocol):
    """Minimal interface for commanding an arm to a joint configuration.

    Any object that implements ``move_to(q_rad, *, dry_run)`` satisfies this
    protocol.  The AA-9 :class:`~inchworm.motion_guard.GuardedArm` satisfies
    it via its ``write_positions_rad`` method, but any stub or mock that
    provides the signature also works.

    Note: AA-19 wraps guard checks explicitly before calling ``move_to``, so
    the mover does NOT need to perform its own guard checks.  A plain mock
    that records calls is sufficient for testing.
    """

    def move_to(self, q_rad: JointVec, *, dry_run: bool) -> None:
        """Command the arm to joint configuration ``q_rad``.

        Parameters
        ----------
        q_rad:
            5-vector of joint angles (rad), ordered
            [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll].
        dry_run:
            If True, log the command but do not write to hardware.
        """
        ...


@runtime_checkable
class GripperStatusProtocol(Protocol):
    """Minimal interface for reading gripper lock state.

    Any object with an ``is_locked`` property (or callable) returning a
    ``bool`` satisfies this protocol.  The AA-17
    :class:`~inchworm.gripper_lock.GripperLockSM` satisfies it via its
    ``state`` property (callers should wrap it as shown in
    :func:`gripper_sm_to_status`).

    Why a separate protocol instead of importing ``GripperLockSM``?
    Avoids the AA-8 gripper capability token import chain in this module,
    and lets tests inject simple stubs without the full state machine.
    """

    @property
    def is_locked(self) -> bool:
        """Return True when the gripper is in the LOCKED state."""
        ...


# ---------------------------------------------------------------------------
# Adapter: wrap GripperLockSM for use as GripperStatusProtocol
# ---------------------------------------------------------------------------


class GripperLockSMStatus:
    """Thin adapter that exposes :attr:`is_locked` from a
    :class:`~inchworm.gripper_lock.GripperLockSM`.

    This avoids importing ``GripperState`` in fold.py (that would pull in the
    gripper capability token chain).

    Parameters
    ----------
    sm:
        Any object with a ``state`` attribute whose ``.name`` attribute is
        ``"LOCKED"`` when the gripper is locked.  The AA-17 GripperLockSM
        satisfies this without a hard type dependency.
    """

    def __init__(self, sm: object) -> None:
        self._sm = sm

    @property
    def is_locked(self) -> bool:
        """True when ``sm.state.name == "LOCKED"``."""
        state = getattr(self._sm, "state", None)
        if state is None:
            return False
        return getattr(state, "name", None) == "LOCKED"


# ---------------------------------------------------------------------------
# Fold sequencer
# ---------------------------------------------------------------------------


class FoldSequencer:
    """Execute fold and unfold sequences for a pair of SO-101 arms.

    Parameters
    ----------
    fold_cfg:
        :class:`~inchworm.config.FoldSequenceConfig` loaded from
        ``hardware.yaml``.  Contains the ordered waypoint lists.
    folded_transit_q:
        The ``folded_transit`` named pose as a 5-vector (rad).  Loaded from
        the AA-9 ``named_poses`` config.
    guard_left:
        :class:`~inchworm.motion_guard.MotionGuard` for the left arm.
    guard_right:
        :class:`~inchworm.motion_guard.MotionGuard` for the right arm.
    mover_left:
        Any object satisfying :class:`ArmMoverProtocol` for the left arm.
    mover_right:
        Any object satisfying :class:`ArmMoverProtocol` for the right arm.
    gripper_left:
        Any object satisfying :class:`GripperStatusProtocol` for the left arm.
    gripper_right:
        Any object satisfying :class:`GripperStatusProtocol` for the right arm.
    session_logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    """

    def __init__(
        self,
        fold_cfg: FoldSequenceConfig,
        folded_transit_q: JointVec,
        guard_left: MotionGuard,
        guard_right: MotionGuard,
        mover_left: ArmMoverProtocol,
        mover_right: ArmMoverProtocol,
        gripper_left: GripperStatusProtocol,
        gripper_right: GripperStatusProtocol,
        session_logger: Optional[object] = None,
    ) -> None:
        self._fold_cfg = fold_cfg
        self._folded_transit_q: JointVec = np.asarray(
            folded_transit_q, dtype=np.float64
        )
        self._guard_left = guard_left
        self._guard_right = guard_right
        self._mover_left = mover_left
        self._mover_right = mover_right
        self._gripper_left = gripper_left
        self._gripper_right = gripper_right
        self._session_logger = session_logger

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------

    def fold(
        self,
        *,
        dry_run: bool = True,
        start_q_left: Optional[JointVec] = None,
        start_q_right: Optional[JointVec] = None,
    ) -> None:
        """Move both arms to the folded transit pose.

        Precondition: both grippers must report LOCKED.  If either reports
        unlocked, :class:`FoldRefused` is raised before any motion.

        Each waypoint in ``fold_sequence.fold_waypoints`` is individually
        collision-checked (per-waypoint ``check_pose`` and per-segment
        ``check_trajectory`` from the previous configuration).  After the
        configured waypoints the ``folded_transit`` named pose from AA-9 is
        executed as the final step.

        Live execution (``dry_run=False``) fails closed on any unverified
        waypoint: :class:`UnverifiedWaypointError` is raised before motion.

        Parameters
        ----------
        dry_run:
            Default ``True``.  Pass ``False`` only after owner supervision
            and hardware verification of all waypoints.
        start_q_left:
            Current joint configuration of the left arm (rad), used for the
            first segment's collision check.  If ``None``, the first segment
            check is skipped (only per-waypoint checks run).
        start_q_right:
            Current joint configuration of the right arm (rad).  Same as
            ``start_q_left`` but for the right arm.

        Raises
        ------
        :class:`FoldRefused`
            Either gripper is not LOCKED.
        :class:`UnverifiedWaypointError`
            Any waypoint is unverified and ``dry_run=False``.
        :class:`FoldMotionViolation`
            Any waypoint or segment fails the motion guard.
        """
        self._check_gripper_precondition()
        self._emit_info(
            "fold_start",
            {"dry_run": dry_run, "waypoints": len(self._fold_cfg.fold_waypoints)},
        )

        # Build the full waypoint sequence: config waypoints + folded_transit
        waypoints = list(self._fold_cfg.fold_waypoints)
        # NOTE: folded_transit is appended as the terminal step only if the
        # last config waypoint doesn't already match it (avoids duplicate move).
        if not waypoints or not self._waypoints_equal(
            waypoints[-1].q_rad, self._folded_transit_q
        ):
            from inchworm.config import FoldWaypoint as _FW

            terminal = _FW(
                q_rad=list(self._folded_transit_q),
                description="folded_transit named pose (AA-9) — terminal fold step",
                unverified=True,  # inherits unverified flag from named_pose
            )
            waypoints = waypoints + [terminal]

        self._execute_sequence(
            waypoints=waypoints,
            start_q_left=start_q_left,
            start_q_right=start_q_right,
            dry_run=dry_run,
            direction="fold",
        )
        self._emit_info("fold_complete", {"dry_run": dry_run})

    def unfold(
        self,
        *,
        dry_run: bool = True,
        start_q_left: Optional[JointVec] = None,
        start_q_right: Optional[JointVec] = None,
    ) -> None:
        """Move both arms from the folded transit pose to a coarse-approach-ready state.

        Uses ``fold_sequence.unfold_waypoints`` from config.  The sequence
        is designed to move distal joints first (relax wrist/elbow) before
        lowering the shoulder, avoiding collision with keep-out zones that
        might be entered if shoulder were lowered first.

        The same per-waypoint and per-segment guard checks apply as in
        :meth:`fold`.

        Parameters
        ----------
        dry_run:
            Default ``True``.
        start_q_left:
            Current joint configuration for the left arm (should be near
            folded_transit).  If ``None``, the first segment check is skipped.
        start_q_right:
            Current joint configuration for the right arm.

        Raises
        ------
        :class:`FoldRefused`
            Either gripper is not LOCKED.
        :class:`UnverifiedWaypointError`
            Any waypoint is unverified and ``dry_run=False``.
        :class:`FoldMotionViolation`
            Any waypoint or segment fails the motion guard.
        """
        self._check_gripper_precondition()
        self._emit_info(
            "unfold_start",
            {"dry_run": dry_run, "waypoints": len(self._fold_cfg.unfold_waypoints)},
        )

        self._execute_sequence(
            waypoints=self._fold_cfg.unfold_waypoints,
            start_q_left=start_q_left,
            start_q_right=start_q_right,
            dry_run=dry_run,
            direction="unfold",
        )
        self._emit_info("unfold_complete", {"dry_run": dry_run})

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check_gripper_precondition(self) -> None:
        """Raise FoldRefused if either gripper is not LOCKED."""
        left_locked = self._gripper_left.is_locked
        right_locked = self._gripper_right.is_locked
        if not left_locked and not right_locked:
            raise FoldRefused(
                "both grippers are not LOCKED (left: unlocked, right: unlocked). "
                "Both grippers must be LOCKED before folding."
            )
        if not left_locked:
            raise FoldRefused(
                "left gripper is not LOCKED. "
                "Both grippers must be LOCKED before folding."
            )
        if not right_locked:
            raise FoldRefused(
                "right gripper is not LOCKED. "
                "Both grippers must be LOCKED before folding."
            )

    def _execute_sequence(
        self,
        waypoints: Sequence[FoldWaypoint],
        start_q_left: Optional[JointVec],
        start_q_right: Optional[JointVec],
        dry_run: bool,
        direction: str,
    ) -> None:
        """Execute a sequence of waypoints on both arms.

        For each waypoint:
        1. Live-mode: reject if unverified.
        2. Per-waypoint collision check (left + right arms).
        3. Per-segment trajectory check from previous config (left + right).
        4. If all checks pass: command both arms to the waypoint.

        Parameters are the same as :meth:`fold` / :meth:`unfold`.
        """
        prev_q_left = (
            np.asarray(start_q_left, dtype=np.float64)
            if start_q_left is not None
            else None
        )
        prev_q_right = (
            np.asarray(start_q_right, dtype=np.float64)
            if start_q_right is not None
            else None
        )

        for step_idx, waypoint in enumerate(waypoints):
            q = np.asarray(waypoint.q_rad, dtype=np.float64)

            # Live: fail closed on unverified waypoints
            if not dry_run and waypoint.unverified:
                raise UnverifiedWaypointError(step_idx, "left")  # both arms same wp

            # --- Per-waypoint check_pose (both arms) ---
            for arm_role, guard in (("left", self._guard_left), ("right", self._guard_right)):
                violation = guard.check_pose(q, live=not dry_run)
                if violation is not None:
                    self._emit_error(
                        f"{direction}_waypoint_violation",
                        {
                            "step": step_idx,
                            "arm": arm_role,
                            "violation": violation,
                            "dry_run": dry_run,
                        },
                    )
                    raise FoldMotionViolation(step_idx, violation, arm_role)

            # --- Per-segment check_trajectory (both arms) ---
            for arm_role, guard, prev_q in (
                ("left", self._guard_left, prev_q_left),
                ("right", self._guard_right, prev_q_right),
            ):
                if prev_q is not None:
                    violation = guard.check_trajectory(prev_q, q, live=not dry_run)
                    if violation is not None:
                        self._emit_error(
                            f"{direction}_segment_violation",
                            {
                                "step": step_idx,
                                "arm": arm_role,
                                "violation": violation,
                                "dry_run": dry_run,
                            },
                        )
                        raise FoldMotionViolation(step_idx, violation, arm_role)

            # --- Execute the move ---
            self._emit_info(
                f"{direction}_waypoint",
                {
                    "step": step_idx,
                    "q_rad": list(q),
                    "description": (waypoint.description or "")[:_DESC_TRUNCATE],
                    "unverified": waypoint.unverified,
                    "dry_run": dry_run,
                },
            )
            self._mover_left.move_to(q, dry_run=dry_run)
            self._mover_right.move_to(q, dry_run=dry_run)

            prev_q_left = q.copy()
            prev_q_right = q.copy()

    @staticmethod
    def _waypoints_equal(
        q_list: List[float], q_arr: npt.NDArray[np.float64], tol: float = 1e-6
    ) -> bool:
        """Return True when q_list and q_arr are within ``tol`` rad."""
        return bool(np.allclose(np.asarray(q_list), q_arr, atol=tol))

    # ------------------------------------------------------------------
    # Telemetry helpers
    # ------------------------------------------------------------------

    def _emit_info(self, event_type: str, payload: dict) -> None:
        logger.info("AA-19 FoldSequencer [%s]: %s", event_type, payload)
        if self._session_logger is not None:
            emit = getattr(self._session_logger, "info", None)
            if emit is not None:
                emit(
                    role="fold_sequencer",
                    subsystem="fold",
                    event_type=event_type,
                    payload=payload,
                )

    def _emit_error(self, event_type: str, payload: dict) -> None:
        logger.error("AA-19 FoldSequencer [%s]: %s", event_type, payload)
        if self._session_logger is not None:
            emit = getattr(self._session_logger, "error", None)
            if emit is not None:
                emit(
                    role="fold_sequencer",
                    subsystem="fold",
                    event_type=event_type,
                    payload=payload,
                )


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_fold_sequencer(
    cfg: HardwareConfig,
    guard_left: MotionGuard,
    guard_right: MotionGuard,
    mover_left: ArmMoverProtocol,
    mover_right: ArmMoverProtocol,
    gripper_left: GripperStatusProtocol,
    gripper_right: GripperStatusProtocol,
    session_logger: Optional[object] = None,
) -> FoldSequencer:
    """Build a :class:`FoldSequencer` from a loaded :class:`HardwareConfig`.

    Parameters
    ----------
    cfg:
        Loaded hardware config (from ``load_config()``).
    guard_left:
        :class:`~inchworm.motion_guard.MotionGuard` for the left arm.
    guard_right:
        :class:`~inchworm.motion_guard.MotionGuard` for the right arm.
    mover_left:
        Left-arm mover satisfying :class:`ArmMoverProtocol`.
    mover_right:
        Right-arm mover satisfying :class:`ArmMoverProtocol`.
    gripper_left:
        Left-arm gripper status satisfying :class:`GripperStatusProtocol`.
    gripper_right:
        Right-arm gripper status satisfying :class:`GripperStatusProtocol`.
    session_logger:
        Optional AA-7 session logger.

    Returns
    -------
    :class:`FoldSequencer`

    Raises
    ------
    ValueError
        If ``fold_sequence`` or ``motion_guard.named_poses`` is missing from
        the config, or if the ``folded_transit`` named pose is absent.
    """
    if cfg.fold_sequence is None:
        raise ValueError(
            "hardware.yaml is missing the 'fold_sequence' block. "
            "Add it under the # --- AA-19 --- marker."
        )

    if cfg.motion_guard is None:
        raise ValueError(
            "hardware.yaml is missing the 'motion_guard' block. "
            "The folded_transit named pose is required."
        )

    named_pose = get_named_pose("folded_transit", cfg.motion_guard)
    if named_pose is None:
        raise ValueError(
            "Named pose 'folded_transit' not found in motion_guard.named_poses. "
            "Add it to hardware.yaml under the AA-9 motion_guard block."
        )

    if named_pose.unverified:
        logger.warning(
            "AA-19: 'folded_transit' named pose is UNVERIFIED. "
            "Do not use for live motion without owner verification."
        )

    folded_transit_q = np.asarray(named_pose.q_rad, dtype=np.float64)

    return FoldSequencer(
        fold_cfg=cfg.fold_sequence,
        folded_transit_q=folded_transit_q,
        guard_left=guard_left,
        guard_right=guard_right,
        mover_left=mover_left,
        mover_right=mover_right,
        gripper_left=gripper_left,
        gripper_right=gripper_right,
        session_logger=session_logger,
    )
