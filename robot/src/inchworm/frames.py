"""AA-15: Chassis–tube frame model and attachment-point lattice.

CRITICAL CONVENTION — READ FIRST
---------------------------------
In this machine the **chassis is fixed to the ground** and the **tube
translates** along its own axis.  This inverts the usual mobile-robot
convention.  Every function in this module uses that convention explicitly:

* ``T_chassis_tube`` — pose of the tube *origin* in the chassis-fixed frame.
* Attachment-point poses are computed from the *current* tube pose, not from
  accumulated deltas, so errors do not compound across crawl cycles.
* Arm base-to-chassis transforms (``T_chassis_armbase``) are read from
  ``so101_kinematics.{left,right}_arm_base_to_chassis`` in config; both are
  marked ``unverified: true`` until AA-16 calibration.

Frame hierarchy
---------------
::

    [chassis] (fixed, world frame)
        ├─ [left arm base]   T_chassis_left   (unverified until AA-16)
        ├─ [right arm base]  T_chassis_right  (unverified until AA-16)
        └─ [tube origin]     T_chassis_tube   (updated by advance / correction)
               └─ [attachment point k]  T_tube_ap(k)
                                         = translation(k * ATTACHMENT_PITCH_MM, 0, 0)
                      (z-axis of ap-frame points into the tube bore)

The tube translates only along one axis in the chassis frame; rotation of the
tube is not modelled at this stage (orientation is fixed by the fixture).

Attachment-point lattice
------------------------
Attachment points are numbered by integer **lattice index**.  Point ``k`` in
the *tube* frame is at::

    T_tube_ap(k) = translate_x(k * ATTACHMENT_PITCH_MM / 1000)  # metres

The pitch constant is imported from ``inchworm.constants`` and must never be
redefined here.

IK / reachability
-----------------
Reachability calls the AA-11 IK solver (``inchworm.kinematics``) on the target
attachment pose expressed in the relevant arm's *base* frame.  Offsets are
taken from ``config.frames_model.{left,right}``; if they are marked
``unverified`` the results are labelled provisional.  IK failure or
exceeding joint limits → unreachable.

Results are **provisional** until AA-16 supplies verified transforms and
offsets.

Telemetry
---------
All state mutations log a structured event via the AA-7 ``SessionLogger``
(if one is supplied at construction time).
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np
import numpy.typing as npt

from inchworm.constants import ATTACHMENT_PITCH_MM
from inchworm.kinematics import KinematicChain, IKResult, _htm_from_rpy_xyz  # noqa: F401

# ─────────────────────────────────────────────────────────────────────────────
# Module-level logger (Python stdlib, not AA-7 — AA-7 SessionLogger is optional)
# ─────────────────────────────────────────────────────────────────────────────

logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# Type aliases
# ─────────────────────────────────────────────────────────────────────────────

Mat44 = npt.NDArray[np.float64]   # 4×4 homogeneous transform
Vec3  = npt.NDArray[np.float64]   # 3-vector (metres)

# Schema version for JSON serialisation – increment when the layout changes.
_SCHEMA_VERSION: int = 1

# ─────────────────────────────────────────────────────────────────────────────
# Tube pose
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class TubePose:
    """Pose of the tube origin in the chassis-fixed frame.

    The tube translates along ``axis_in_chassis`` (default: x-axis).
    Rotation of the tube is not modelled; ``T_chassis_tube`` encodes only the
    translational component plus the fixed orientation of the tube w.r.t the
    chassis (set at construction, updated only by corrections).

    Parameters
    ----------
    T_chassis_tube:
        4×4 HTM expressing the tube origin in the chassis frame.
    advance_count:
        Strictly monotonically increasing integer; incremented on every
        *advance* update (not on corrections).  Used by the consistency
        check when resuming from serialised state.
    """

    T_chassis_tube: Mat44
    advance_count: int = 0

    # ------------------------------------------------------------------
    # Constructors
    # ------------------------------------------------------------------

    @classmethod
    def identity(cls) -> "TubePose":
        """Tube origin coincident with chassis origin, zero advance count."""
        return cls(T_chassis_tube=np.eye(4, dtype=np.float64))

    @classmethod
    def from_translation(cls, tx: float, ty: float, tz: float) -> "TubePose":
        """Tube translated by (tx, ty, tz) metres from the chassis origin."""
        T = np.eye(4, dtype=np.float64)
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz
        return cls(T_chassis_tube=T)

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def translation_m(self) -> Vec3:
        """Translation component of T_chassis_tube (metres)."""
        return self.T_chassis_tube[:3, 3].copy()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "T_chassis_tube": self.T_chassis_tube.tolist(),
            "advance_count": self.advance_count,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TubePose":
        return cls(
            T_chassis_tube=np.array(d["T_chassis_tube"], dtype=np.float64),
            advance_count=int(d["advance_count"]),
        )


# ─────────────────────────────────────────────────────────────────────────────
# Update kind: advance vs measured correction
# ─────────────────────────────────────────────────────────────────────────────


class UpdateKind(str, Enum):
    """Distinguishes the two paths by which tube pose may be updated.

    ``ADVANCE``
        The orchestrator commanded a move and the tube was pushed forward by
        one lattice step.  The advance counter is incremented.

    ``MEASURED_CORRECTION``
        A sensor measurement (e.g. laser scan, vision) produced a better
        estimate of the true tube pose.  The advance counter is **not**
        changed; the pose is replaced with the measurement.
    """

    ADVANCE = "advance"
    MEASURED_CORRECTION = "measured_correction"


# ─────────────────────────────────────────────────────────────────────────────
# Attachment-point pose helper
# ─────────────────────────────────────────────────────────────────────────────


def _attachment_point_in_tube_frame(index: int) -> Mat44:
    """4×4 HTM of attachment point ``index`` in the *tube* frame.

    The tube is a 1-D lattice: point ``k`` is at
    ``x = k * ATTACHMENT_PITCH_MM / 1000`` metres along the tube x-axis.
    The attachment-point z-axis points into the bore (same as tube frame).

    Parameters
    ----------
    index:
        Integer lattice index (may be negative).
    """
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = index * (ATTACHMENT_PITCH_MM / 1000.0)  # convert mm → m
    return T


def attachment_point_in_chassis_frame(tube_pose: TubePose, index: int) -> Mat44:
    """4×4 HTM of lattice point ``index`` in the *chassis* frame.

    Computed from the current tube pose (never from accumulated deltas).

    Parameters
    ----------
    tube_pose:
        Current :class:`TubePose`.
    index:
        Integer lattice index.
    """
    T_tube_ap = _attachment_point_in_tube_frame(index)
    return tube_pose.T_chassis_tube @ T_tube_ap


# ─────────────────────────────────────────────────────────────────────────────
# Attachment-point descriptor
# ─────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class AttachmentPoint:
    """A single attachment point on the lattice with its chassis-frame pose.

    Attributes
    ----------
    index:
        Integer lattice index.
    T_chassis_ap:
        Pose of this attachment point in the chassis-fixed frame.
    """

    index: int
    T_chassis_ap: Mat44


# ─────────────────────────────────────────────────────────────────────────────
# Per-arm reachability result
# ─────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ArmReachability:
    """Reachability of one attachment point for one arm.

    All fields are from the arm's perspective in its own base frame.

    Attributes
    ----------
    index:
        Lattice index.
    arm:
        ``"left"`` or ``"right"``.
    coarse_approach_reachable:
        True if IK converged on the coarse-approach pose.
    insertion_reachable:
        True if IK converged on the insertion pose.
    coarse_ik:
        IK result for coarse approach (best-effort even if not reachable).
    insertion_ik:
        IK result for insertion (best-effort even if not reachable).
    provisional:
        True when the arm-base or offset transforms are unverified (AA-16
        not yet run).  Results should not be used to command live motion.
    note:
        Human-readable reason string when not reachable or provisional.
    """

    index: int
    arm: str
    coarse_approach_reachable: bool
    insertion_reachable: bool
    coarse_ik: Optional[IKResult]
    insertion_ik: Optional[IKResult]
    provisional: bool
    note: str = ""


# ─────────────────────────────────────────────────────────────────────────────
# Reachability report
# ─────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ReachabilityReport:
    """Reachability of a range of lattice indices for both arms.

    Attributes
    ----------
    left:
        List of :class:`ArmReachability` for the left arm (one per index).
    right:
        List of :class:`ArmReachability` for the right arm (one per index).
    provisional:
        True if any result is provisional.
    """

    left: Tuple[ArmReachability, ...]
    right: Tuple[ArmReachability, ...]
    provisional: bool


# ─────────────────────────────────────────────────────────────────────────────
# Lattice state — per-arm held index
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class LatticeState:
    """Which attachment-point index each arm currently holds.

    Attributes
    ----------
    left_index:
        Lattice index currently held by the left arm (``None`` = not engaged).
    right_index:
        Lattice index currently held by the right arm (``None`` = not engaged).
    left_target:
        Next target index for the left arm (set by the orchestrator).
    right_target:
        Next target index for the right arm (set by the orchestrator).
    """

    left_index: Optional[int] = None
    right_index: Optional[int] = None
    left_target: Optional[int] = None
    right_target: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "left_index": self.left_index,
            "right_index": self.right_index,
            "left_target": self.left_target,
            "right_target": self.right_target,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "LatticeState":
        return cls(
            left_index=d.get("left_index"),
            right_index=d.get("right_index"),
            left_target=d.get("left_target"),
            right_target=d.get("right_target"),
        )


# ─────────────────────────────────────────────────────────────────────────────
# Config hash for consistency check
# ─────────────────────────────────────────────────────────────────────────────


def _config_hash(cfg: Any) -> str:
    """Stable SHA-256 hash of the serialised ``HardwareConfig``.

    Uses the model's JSON representation for determinism.  Two configs that
    differ in any field will produce different hashes; stale snapshots are
    therefore rejected at resume time.
    """
    try:
        raw = cfg.model_dump_json()
    except AttributeError:
        raw = json.dumps(cfg, sort_keys=True, default=str)
    return hashlib.sha256(raw.encode()).hexdigest()


# ─────────────────────────────────────────────────────────────────────────────
# FrameModel — the core object
# ─────────────────────────────────────────────────────────────────────────────


class FrameModel:
    """Chassis–tube frame model and attachment-point lattice bookkeeper.

    IMPORTANT: The chassis is **fixed**; the tube **translates**.

    This class tracks:

    * The current tube pose (``T_chassis_tube``) — updated via
      :meth:`update_tube_pose`.
    * Which lattice indices each arm holds — updated via
      :meth:`set_held_index` / :meth:`release_index`.
    * Reachability — queried via :meth:`reachability_report`.
    * JSON serialisation for interrupted-run resume —
      :meth:`to_json` / :meth:`from_json`.

    Parameters
    ----------
    left_chain:
        Kinematic chain for the left arm (built by ``kinematics.build_chain``).
    right_chain:
        Kinematic chain for the right arm.
    T_chassis_left:
        4×4 HTM: left arm base in chassis frame (unverified until AA-16).
    T_chassis_right:
        4×4 HTM: right arm base in chassis frame (unverified until AA-16).
    left_offsets_unverified:
        Whether the left-arm approach offsets are unverified placeholders.
    right_offsets_unverified:
        Whether the right-arm approach offsets are unverified placeholders.
    coarse_offset_left_m:
        3-vector offset (attachment-point frame) for coarse approach, left arm.
    insertion_offset_left_m:
        3-vector offset (attachment-point frame) for insertion, left arm.
    coarse_offset_right_m:
        3-vector offset (attachment-point frame) for coarse approach, right arm.
    insertion_offset_right_m:
        3-vector offset (attachment-point frame) for insertion, right arm.
    transforms_unverified:
        True when ``T_chassis_left`` / ``T_chassis_right`` are unverified.
    tube_pose:
        Initial tube pose (default: identity — tube at chassis origin).
    lattice_state:
        Initial lattice state (default: both arms unengaged).
    config_hash:
        SHA-256 hash of the hardware config used to create this model.
        Required for consistency check at resume.
    session_logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    """

    def __init__(
        self,
        *,
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        T_chassis_left: Mat44,
        T_chassis_right: Mat44,
        coarse_offset_left_m: Sequence[float],
        insertion_offset_left_m: Sequence[float],
        coarse_offset_right_m: Sequence[float],
        insertion_offset_right_m: Sequence[float],
        left_offsets_unverified: bool = True,
        right_offsets_unverified: bool = True,
        transforms_unverified: bool = True,
        tube_pose: Optional[TubePose] = None,
        lattice_state: Optional[LatticeState] = None,
        config_hash: str = "",
        session_logger: Any = None,
    ) -> None:
        self._left_chain = left_chain
        self._right_chain = right_chain
        self._T_chassis_left = T_chassis_left.copy()
        self._T_chassis_right = T_chassis_right.copy()
        self._coarse_left = np.array(coarse_offset_left_m, dtype=np.float64)
        self._ins_left    = np.array(insertion_offset_left_m, dtype=np.float64)
        self._coarse_right = np.array(coarse_offset_right_m, dtype=np.float64)
        self._ins_right    = np.array(insertion_offset_right_m, dtype=np.float64)
        self._left_offsets_unverified  = left_offsets_unverified
        self._right_offsets_unverified = right_offsets_unverified
        self._transforms_unverified    = transforms_unverified
        self._tube_pose   = tube_pose if tube_pose is not None else TubePose.identity()
        self._lattice     = lattice_state if lattice_state is not None else LatticeState()
        self._config_hash = config_hash
        self._slog        = session_logger

        if transforms_unverified:
            logger.warning(
                "AA-15 FrameModel: arm-base-to-chassis transforms are UNVERIFIED "
                "(AA-16 calibration not yet run). Reachability results are provisional."
            )
        if left_offsets_unverified or right_offsets_unverified:
            logger.warning(
                "AA-15 FrameModel: one or more approach/insertion offsets are "
                "UNVERIFIED. Reachability results are provisional."
            )

    # ------------------------------------------------------------------
    # Factory from config
    # ------------------------------------------------------------------

    @classmethod
    def from_config(
        cls,
        cfg: Any,
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        *,
        tube_pose: Optional[TubePose] = None,
        lattice_state: Optional[LatticeState] = None,
        session_logger: Any = None,
    ) -> "FrameModel":
        """Build a :class:`FrameModel` from a loaded ``HardwareConfig``.

        ``cfg.so101_kinematics.left_arm_base_to_chassis`` and
        ``right_arm_base_to_chassis`` supply the arm-base transforms.
        ``cfg.frames_model`` supplies the approach/insertion offsets.

        Parameters
        ----------
        cfg:
            Fully validated :class:`~inchworm.config.HardwareConfig`.
        left_chain / right_chain:
            Pre-built kinematic chains (use ``kinematics.build_chain``).
        tube_pose:
            Initial tube pose (default: identity).
        lattice_state:
            Initial lattice state (default: both arms unengaged).
        session_logger:
            Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
        """
        kin = cfg.so101_kinematics
        T_l = _htm_from_rpy_xyz(
            kin.left_arm_base_to_chassis.xyz,
            kin.left_arm_base_to_chassis.rpy,
        )
        T_r = _htm_from_rpy_xyz(
            kin.right_arm_base_to_chassis.xyz,
            kin.right_arm_base_to_chassis.rpy,
        )
        transforms_unverified = (
            kin.left_arm_base_to_chassis.unverified
            or kin.right_arm_base_to_chassis.unverified
        )

        fm = cfg.frames_model
        if fm is not None:
            coarse_l   = fm.left.coarse_approach_offset_m
            ins_l      = fm.left.insertion_offset_m
            coarse_r   = fm.right.coarse_approach_offset_m
            ins_r      = fm.right.insertion_offset_m
            unverified_l = fm.left.unverified
            unverified_r = fm.right.unverified
        else:
            # No frames_model block → use zero offsets, mark unverified.
            coarse_l = ins_l = coarse_r = ins_r = [0.0, 0.0, 0.0]
            unverified_l = unverified_r = True
            logger.warning(
                "AA-15 FrameModel: no frames_model block in hardware.yaml; "
                "using zero approach offsets (unverified)."
            )

        return cls(
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_l,
            T_chassis_right=T_r,
            coarse_offset_left_m=coarse_l,
            insertion_offset_left_m=ins_l,
            coarse_offset_right_m=coarse_r,
            insertion_offset_right_m=ins_r,
            left_offsets_unverified=unverified_l,
            right_offsets_unverified=unverified_r,
            transforms_unverified=transforms_unverified,
            tube_pose=tube_pose,
            lattice_state=lattice_state,
            config_hash=_config_hash(cfg),
            session_logger=session_logger,
        )

    # ------------------------------------------------------------------
    # Tube pose
    # ------------------------------------------------------------------

    @property
    def tube_pose(self) -> TubePose:
        """Current tube pose (read-only snapshot)."""
        return self._tube_pose

    def update_tube_pose(
        self,
        new_T_chassis_tube: Mat44,
        kind: UpdateKind,
    ) -> None:
        """Update the tube pose, distinguishing advance from measurement.

        Parameters
        ----------
        new_T_chassis_tube:
            New 4×4 HTM for tube origin in chassis frame.
        kind:
            :class:`UpdateKind` — ``ADVANCE`` increments the advance counter;
            ``MEASURED_CORRECTION`` does not.
        """
        prev = self._tube_pose
        if kind is UpdateKind.ADVANCE:
            new_count = prev.advance_count + 1
        else:
            new_count = prev.advance_count

        self._tube_pose = TubePose(
            T_chassis_tube=new_T_chassis_tube.copy(),
            advance_count=new_count,
        )

        self._log(
            "info",
            role="orchestrator",
            subsystem="frames",
            event_type="tube_pose_updated",
            payload={
                "kind": kind.value,
                "advance_count": new_count,
                "translation_m": self._tube_pose.translation_m.tolist(),
            },
        )

    def complete_advance(self, *, direction: int = +1) -> None:
        """Record one completed crawl advance of exactly one pitch.

        Orchestrator addition 2026-10-24 (AA-15 AC#2/AC#4). The new tube pose is
        derived from the lattice, ``T_chassis_tube`` translated by
        ``direction * ATTACHMENT_PITCH_MM`` along the tube x-axis in the tube
        frame, so callers cannot inject an accumulated or rounded delta. The
        tube moves; the chassis does not. ``direction`` is +1 (advance) or -1
        (retreat). Which physical way +1 points along the chassis is set by the
        tube-frame orientation in config (unverified until AA-16).
        """
        if direction not in (+1, -1):
            raise ValueError("direction must be +1 or -1")
        step = np.eye(4)
        step[0, 3] = direction * (ATTACHMENT_PITCH_MM / 1000.0)
        self.update_tube_pose(self._tube_pose.T_chassis_tube @ step, UpdateKind.ADVANCE)

    # ------------------------------------------------------------------
    # Lattice state
    # ------------------------------------------------------------------

    @property
    def lattice_state(self) -> LatticeState:
        """Current lattice state (held / target indices)."""
        return self._lattice

    def set_held_index(self, arm: str, index: int) -> None:
        """Record that ``arm`` (``"left"`` or ``"right"``) now holds ``index``."""
        _require_arm(arm)
        if arm == "left":
            self._lattice.left_index = index
        else:
            self._lattice.right_index = index
        self._log(
            "info",
            role=f"arm_{arm}",
            subsystem="frames",
            event_type="arm_index_held",
            payload={"arm": arm, "index": index},
        )

    def release_index(self, arm: str) -> None:
        """Record that ``arm`` has released its current attachment point."""
        _require_arm(arm)
        if arm == "left":
            self._lattice.left_index = None
        else:
            self._lattice.right_index = None
        self._log(
            "info",
            role=f"arm_{arm}",
            subsystem="frames",
            event_type="arm_index_released",
            payload={"arm": arm},
        )

    def set_target_index(self, arm: str, index: int) -> None:
        """Set the next target attachment index for ``arm``."""
        _require_arm(arm)
        if arm == "left":
            self._lattice.left_target = index
        else:
            self._lattice.right_target = index

    # ------------------------------------------------------------------
    # Attachment-point query
    # ------------------------------------------------------------------

    def attachment_pose_in_chassis(self, index: int) -> Mat44:
        """Chassis-frame pose of lattice point ``index``.

        Computed from the *current* tube pose — never from accumulated deltas.
        """
        return attachment_point_in_chassis_frame(self._tube_pose, index)

    def held_indices(self) -> Dict[str, Optional[int]]:
        """Return ``{"left": left_index, "right": right_index}``."""
        return {
            "left": self._lattice.left_index,
            "right": self._lattice.right_index,
        }

    def next_targets(self) -> Dict[str, Optional[int]]:
        """Return ``{"left": left_target, "right": right_target}``."""
        return {
            "left": self._lattice.left_target,
            "right": self._lattice.right_target,
        }

    # ------------------------------------------------------------------
    # Reachability
    # ------------------------------------------------------------------

    def reachability_report(
        self,
        indices: Sequence[int],
    ) -> ReachabilityReport:
        """Check whether each arm can reach the listed lattice indices.

        For each index, IK is attempted on the coarse-approach pose and the
        insertion pose, both expressed in the arm's *base* frame.

        Results are **provisional** when any transform or offset is unverified.

        Parameters
        ----------
        indices:
            Integer lattice indices to check.

        Returns
        -------
        ReachabilityReport
            One :class:`ArmReachability` per (arm, index) pair.
        """
        provisional = (
            self._transforms_unverified
            or self._left_offsets_unverified
            or self._right_offsets_unverified
        )

        left_results: list[ArmReachability] = []
        right_results: list[ArmReachability] = []

        for idx in indices:
            T_ap = self.attachment_pose_in_chassis(idx)

            left_results.append(
                self._check_arm(
                    idx=idx,
                    arm="left",
                    chain=self._left_chain,
                    T_chassis_base=self._T_chassis_left,
                    T_chassis_ap=T_ap,
                    coarse_offset=self._coarse_left,
                    insertion_offset=self._ins_left,
                    offsets_unverified=self._left_offsets_unverified,
                    provisional=provisional,
                )
            )
            right_results.append(
                self._check_arm(
                    idx=idx,
                    arm="right",
                    chain=self._right_chain,
                    T_chassis_base=self._T_chassis_right,
                    T_chassis_ap=T_ap,
                    coarse_offset=self._coarse_right,
                    insertion_offset=self._ins_right,
                    offsets_unverified=self._right_offsets_unverified,
                    provisional=provisional,
                )
            )

        report = ReachabilityReport(
            left=tuple(left_results),
            right=tuple(right_results),
            provisional=provisional,
        )
        self._log(
            "info",
            role="orchestrator",
            subsystem="frames",
            event_type="reachability_report",
            payload={
                "indices": list(indices),
                "provisional": provisional,
                "left_reachable_coarse": [r.coarse_approach_reachable for r in left_results],
                "right_reachable_coarse": [r.coarse_approach_reachable for r in right_results],
            },
        )
        return report

    def _check_arm(
        self,
        *,
        idx: int,
        arm: str,
        chain: KinematicChain,
        T_chassis_base: Mat44,
        T_chassis_ap: Mat44,
        coarse_offset: Vec3,
        insertion_offset: Vec3,
        offsets_unverified: bool,
        provisional: bool,
    ) -> ArmReachability:
        """Internal: IK-check one (arm, index) pair for coarse and insertion."""
        # T_base_ap: attachment point in arm base frame
        T_base_chassis = np.linalg.inv(T_chassis_base)
        T_base_ap = T_base_chassis @ T_chassis_ap

        # Build coarse-approach target in arm base frame
        T_coarse = _apply_offset(T_base_ap, coarse_offset)
        # Build insertion target in arm base frame
        T_ins = _apply_offset(T_base_ap, insertion_offset)

        ik_coarse = chain.ik(T_coarse)
        ik_ins    = chain.ik(T_ins)

        note_parts: list[str] = []
        if provisional:
            note_parts.append("provisional (unverified transforms/offsets)")
        if offsets_unverified:
            note_parts.append("offsets unverified")
        if not ik_coarse.reachable:
            note_parts.append(
                f"coarse IK failed: pos_err={ik_coarse.pos_err_m*1000:.2f} mm"
            )
        if not ik_ins.reachable:
            note_parts.append(
                f"insertion IK failed: pos_err={ik_ins.pos_err_m*1000:.2f} mm"
            )

        return ArmReachability(
            index=idx,
            arm=arm,
            coarse_approach_reachable=ik_coarse.reachable,
            insertion_reachable=ik_ins.reachable,
            coarse_ik=ik_coarse,
            insertion_ik=ik_ins,
            provisional=provisional,
            note="; ".join(note_parts),
        )

    # ------------------------------------------------------------------
    # Serialisation (JSON) — AC#6
    # ------------------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        """Serialise the mutable state to a plain dict.

        The serialised form includes:
        * ``schema_version`` — incremented when the layout changes.
        * ``config_hash`` — SHA-256 of the :class:`~inchworm.config.HardwareConfig`
          at creation time.  Checked at resume to reject stale configs.
        * ``tube_pose`` — including the monotonically increasing ``advance_count``.
        * ``lattice_state`` — per-arm held/target indices.
        * ``ts_iso`` — UTC timestamp of serialisation.
        * ``consistency`` — derived check fields (expected held positions).
        """
        left_held_pos = (
            attachment_point_in_chassis_frame(
                self._tube_pose, self._lattice.left_index
            )[:3, 3].tolist()
            if self._lattice.left_index is not None
            else None
        )
        right_held_pos = (
            attachment_point_in_chassis_frame(
                self._tube_pose, self._lattice.right_index
            )[:3, 3].tolist()
            if self._lattice.right_index is not None
            else None
        )
        return {
            "schema_version": _SCHEMA_VERSION,
            "config_hash": self._config_hash,
            "ts_iso": _utc_iso(),
            "tube_pose": self._tube_pose.to_dict(),
            "lattice_state": self._lattice.to_dict(),
            "consistency": {
                "left_held_chassis_pos_m": left_held_pos,
                "right_held_chassis_pos_m": right_held_pos,
                "advance_count": self._tube_pose.advance_count,
                "pitch_mm": float(ATTACHMENT_PITCH_MM),
            },
        }

    def to_json(self, **kwargs: Any) -> str:
        """Serialise to a JSON string."""
        return json.dumps(self.to_dict(), **kwargs)

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        T_chassis_left: Mat44,
        T_chassis_right: Mat44,
        coarse_offset_left_m: Sequence[float],
        insertion_offset_left_m: Sequence[float],
        coarse_offset_right_m: Sequence[float],
        insertion_offset_right_m: Sequence[float],
        left_offsets_unverified: bool = True,
        right_offsets_unverified: bool = True,
        transforms_unverified: bool = True,
        current_config_hash: str = "",
        session_logger: Any = None,
    ) -> "FrameModel":
        """Deserialise and validate; raises :class:`ResumeError` on inconsistency.

        Consistency checks:
        1. ``schema_version`` must match :data:`_SCHEMA_VERSION`.
        2. ``config_hash`` must match ``current_config_hash`` (if both non-empty).
        3. ``advance_count`` must be non-negative.
        4. ``consistency.pitch_mm`` must equal ``ATTACHMENT_PITCH_MM`` (±0.001 mm).
        5. ``consistency.advance_count`` must equal ``tube_pose.advance_count``.
        """
        _check_resume(data, current_config_hash)

        tube_pose   = TubePose.from_dict(data["tube_pose"])
        lattice     = LatticeState.from_dict(data["lattice_state"])

        return cls(
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_chassis_left,
            T_chassis_right=T_chassis_right,
            coarse_offset_left_m=coarse_offset_left_m,
            insertion_offset_left_m=insertion_offset_left_m,
            coarse_offset_right_m=coarse_offset_right_m,
            insertion_offset_right_m=insertion_offset_right_m,
            left_offsets_unverified=left_offsets_unverified,
            right_offsets_unverified=right_offsets_unverified,
            transforms_unverified=transforms_unverified,
            tube_pose=tube_pose,
            lattice_state=lattice,
            config_hash=data.get("config_hash", ""),
            session_logger=session_logger,
        )

    @classmethod
    def from_json(
        cls,
        json_str: str,
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        T_chassis_left: Mat44,
        T_chassis_right: Mat44,
        coarse_offset_left_m: Sequence[float],
        insertion_offset_left_m: Sequence[float],
        coarse_offset_right_m: Sequence[float],
        insertion_offset_right_m: Sequence[float],
        left_offsets_unverified: bool = True,
        right_offsets_unverified: bool = True,
        transforms_unverified: bool = True,
        current_config_hash: str = "",
        session_logger: Any = None,
    ) -> "FrameModel":
        """Deserialise from JSON string; raises :class:`ResumeError` on failure."""
        data = json.loads(json_str)
        return cls.from_dict(
            data,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_chassis_left,
            T_chassis_right=T_chassis_right,
            coarse_offset_left_m=coarse_offset_left_m,
            insertion_offset_left_m=insertion_offset_left_m,
            coarse_offset_right_m=coarse_offset_right_m,
            insertion_offset_right_m=insertion_offset_right_m,
            left_offsets_unverified=left_offsets_unverified,
            right_offsets_unverified=right_offsets_unverified,
            transforms_unverified=transforms_unverified,
            current_config_hash=current_config_hash,
            session_logger=session_logger,
        )

    # ------------------------------------------------------------------
    # Convenience: from_config + from_json (resume)
    # ------------------------------------------------------------------

    @classmethod
    def from_config_and_json(
        cls,
        cfg: Any,
        json_str: str,
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        *,
        session_logger: Any = None,
    ) -> "FrameModel":
        """Resume a serialised model with the *current* config.

        Validates that the serialised config hash matches the current config;
        raises :class:`ResumeError` if not.
        """
        kin = cfg.so101_kinematics
        T_l = _htm_from_rpy_xyz(
            kin.left_arm_base_to_chassis.xyz,
            kin.left_arm_base_to_chassis.rpy,
        )
        T_r = _htm_from_rpy_xyz(
            kin.right_arm_base_to_chassis.xyz,
            kin.right_arm_base_to_chassis.rpy,
        )
        transforms_unverified = (
            kin.left_arm_base_to_chassis.unverified
            or kin.right_arm_base_to_chassis.unverified
        )
        fm = cfg.frames_model
        if fm is not None:
            coarse_l   = fm.left.coarse_approach_offset_m
            ins_l      = fm.left.insertion_offset_m
            coarse_r   = fm.right.coarse_approach_offset_m
            ins_r      = fm.right.insertion_offset_m
            unverified_l = fm.left.unverified
            unverified_r = fm.right.unverified
        else:
            coarse_l = ins_l = coarse_r = ins_r = [0.0, 0.0, 0.0]
            unverified_l = unverified_r = True

        data = json.loads(json_str)
        return cls.from_dict(
            data,
            left_chain=left_chain,
            right_chain=right_chain,
            T_chassis_left=T_l,
            T_chassis_right=T_r,
            coarse_offset_left_m=coarse_l,
            insertion_offset_left_m=ins_l,
            coarse_offset_right_m=coarse_r,
            insertion_offset_right_m=ins_r,
            left_offsets_unverified=unverified_l,
            right_offsets_unverified=unverified_r,
            transforms_unverified=transforms_unverified,
            current_config_hash=_config_hash(cfg),
            session_logger=session_logger,
        )

    # ------------------------------------------------------------------
    # Telemetry helper
    # ------------------------------------------------------------------

    def _log(
        self,
        level: str,
        *,
        role: str,
        subsystem: str,
        event_type: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Emit to AA-7 SessionLogger if present, else to stdlib logger."""
        if self._slog is not None:
            getattr(self._slog, level)(
                role=role,
                subsystem=subsystem,
                event_type=event_type,
                payload=payload,
            )
        else:
            getattr(logger, level)(
                "%s.%s: %s %s", role, subsystem, event_type, payload or ""
            )


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _apply_offset(T_base_ap: Mat44, offset_m: Vec3) -> Mat44:
    """Apply a 3-vector offset (in the attachment-point frame) to the pose.

    Returns a new 4×4 HTM: ``T_base_ap @ translate(offset_m)``.
    """
    T_offset = np.eye(4, dtype=np.float64)
    T_offset[:3, 3] = offset_m
    return T_base_ap @ T_offset


def _require_arm(arm: str) -> None:
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right'; got {arm!r}")


def _utc_iso() -> str:
    """Return current UTC time as ISO 8601 string."""
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat()


# ─────────────────────────────────────────────────────────────────────────────
# Resume consistency check
# ─────────────────────────────────────────────────────────────────────────────


class ResumeError(ValueError):
    """Raised when deserialised state is stale, corrupt, or inconsistent."""


def _check_resume(data: Dict[str, Any], current_config_hash: str) -> None:
    """Validate a deserialised dict; raises :class:`ResumeError` on failure.

    Checks:
    1. ``schema_version`` matches :data:`_SCHEMA_VERSION`.
    2. ``config_hash`` matches ``current_config_hash`` (if both non-empty).
    3. ``advance_count`` is non-negative.
    4. ``consistency.pitch_mm`` equals ``ATTACHMENT_PITCH_MM`` within 0.001 mm.
    5. ``consistency.advance_count`` matches ``tube_pose.advance_count``.
    """
    sv = data.get("schema_version")
    if sv != _SCHEMA_VERSION:
        raise ResumeError(
            f"schema_version mismatch: serialised {sv!r} != current {_SCHEMA_VERSION!r}"
        )

    stored_hash = data.get("config_hash", "")
    if stored_hash and current_config_hash and stored_hash != current_config_hash:
        raise ResumeError(
            "config_hash mismatch: hardware config has changed since this state was saved. "
            f"stored={stored_hash[:12]}… current={current_config_hash[:12]}…"
        )

    tp = data.get("tube_pose", {})
    advance_count = int(tp.get("advance_count", -1))
    if advance_count < 0:
        raise ResumeError(
            f"invalid advance_count {advance_count!r}: must be >= 0"
        )

    consistency = data.get("consistency", {})

    stored_pitch = consistency.get("pitch_mm")
    if stored_pitch is not None:
        diff = abs(float(stored_pitch) - float(ATTACHMENT_PITCH_MM))
        if diff > 0.001:
            raise ResumeError(
                f"pitch_mm mismatch: serialised {stored_pitch!r} mm vs "
                f"current {ATTACHMENT_PITCH_MM!r} mm (diff={diff:.4f} mm)"
            )

    consistency_advance = consistency.get("advance_count")
    if consistency_advance is not None and int(consistency_advance) != advance_count:
        raise ResumeError(
            f"consistency.advance_count {consistency_advance!r} != "
            f"tube_pose.advance_count {advance_count!r}"
        )
