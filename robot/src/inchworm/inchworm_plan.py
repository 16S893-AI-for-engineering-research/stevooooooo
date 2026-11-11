"""AA-45: Inchworm print-crawl cycle planner on the owner cell URDF.

The owner cannot trust prose descriptions of physical motion; a rendered
animation from the real cell model (``CAD/robot_urdf_new``) is what confirms
behaviour. This module turns the owner-specified crawl cycle into a frame-by-
frame **timeline** (both arms' joint angles, gripper lock states, tube pose,
step name) and checks every frame against the mesh collision model, joint
limits, the gripper invariants and the rigid-tube constraint, so the render is
backed by the same guard the live machine uses.

Owner-specified cycle (2026-11-10, refined 2026-11-11)
------------------------------------------------------
Start with both arms LOCKED in the printing pose. The tube moves only because
the locked arms move it. Both arms (locked) first move the tube AWAY from both
arms (lift, chassis +Z), then translate it along its axis away from the DexArm
("to the right") until the left arm can regrip; the left arm (nearest the
DexArm) unlocks and regrips the attachment point one pitch closer to the
DexArm; the right arm then unlocks and regrips the point the left arm just
left; then both arms (locked) move the tube down again and return to their
start pose. Net tube advance: one attachment pitch (ATTACHMENT_PITCH_MM) away
from the DexArm. Owner 2026-11-11: this is a planar motion of joints 2-4
(shoulder_lift, elbow, wrist_flex); joint 1 (pan) should not be needed, and the
elbow must be used. Insertion is assumed perfect (straight-line pre-insertion
to seated, the learned policy is not modelled). No printing, no laser.

Binding decision (orchestrator, 2026-11-10)
-------------------------------------------
* Attachment frames = **mesh features**: the working-pose grips of the cell
  URDF, spaced one pitch along the tube axis. The point the LEFT gripper sits
  on at the working pose is the MIDDLE point; the RIGHT gripper sits on the
  FAR point; the point one pitch closer to the DexArm (free) is the NEAR point
  and is the left arm's regrip target.
* The start / fold-back **printing pose** is the IK solution that puts the
  LEFT gripper on the middle point and the RIGHT gripper on the far point,
  seeded from the ``hardware.yaml`` ``folded_transit`` placeholder and labelled
  ``"printing pose (IK, seeded from placeholder)"``. It is a single input so
  the owner's re-measured pose drops in later. It is **never** written to
  ``hardware.yaml``. If the IK cannot reach, planning stops and reports.

The gripper servo is commanded only by the AA-17 state machine, never here;
this module only records lock states for the invariant checks and the render.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from inchworm.constants import ATTACHMENT_PITCH_MM

logger = logging.getLogger(__name__)

JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

#: Printing pose joint seed = the hardware.yaml folded_transit placeholder.
#: Stored here only so the planner can seed IK and the report can compare; the
#: owner's real pose replaces this later.
FOLDED_TRANSIT_PLACEHOLDER = np.array([0.0, -1.25, 1.64, -1.31, 1.36])

#: Rigid-tube constraint tolerance: while both arms are locked they must keep a
#: constant relative gripper pose (the tube is rigid). A locked gripper is
#: pinned to its attachment point, so any drift means an arm cannot physically
#: hold its grip. 1 mm / 0.5 deg is tight relative to the pitch (183.7 mm).
RIGID_TUBE_POS_TOL_M = 1.0e-3
RIGID_TUBE_ORI_TOL_RAD = math.radians(0.5)

#: A locked gripper must sit on its attachment point within this residual, or
#: the arm cannot reach where the rigid tube demands (a reachability failure).
GRIP_REACH_TOL_M = 1.0e-3
GRIP_REACH_TOL_RAD = math.radians(0.5)

#: A matching (shared) or given start pose may sit off the attachment points
#: by the arm-spacing/pitch mismatch (~2 mm in the cell model). Above this the
#: start pose is not on the tube at all and planning stops. The insertion
#: policy absorbs the remainder (owner 2026-11-11).
START_GRIP_TOL_M = 5.0e-3
START_GRIP_TOL_RAD = math.radians(3.0)


class PlanStopped(RuntimeError):
    """Planning stopped because a step is impossible to model honestly.

    Raised when an owner-specified regrip target does not exist on the modelled
    tube (never invent one) or when the printing pose IK cannot reach.
    """


# ---------------------------------------------------------------------------
# Cell-URDF kinematics (gripper frame), reusing the collision checker's model
# ---------------------------------------------------------------------------


@dataclass
class CellKinematics:
    """FK/IK of a gripper link against the owner cell URDF.

    FK comes from the same :class:`CellCollisionChecker` the motion guard uses,
    so the pose the planner solves for is the pose that will be checked and
    rendered. IK is damped least-squares bounded by the given joint limits.
    """

    checker: object  # CellCollisionChecker
    lower_rad: np.ndarray
    upper_rad: np.ndarray
    #: The robot's configured soft limits (hardware.yaml motion_guard.<arm>_arm_limits),
    #: per arm. IK stays inside the URDF range (the physical model); frames are
    #: additionally CHECKED against these, because the guard enforces them live.
    soft_limits: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]] = None

    def gripper_pose(self, arm: str, q: Sequence[float]) -> np.ndarray:
        cell_q = self.checker.mapping.to_cell(arm, q)
        return self.checker.model.link_poses(cell_q)[f"{arm}_gripper"]

    def ik(
        self,
        arm: str,
        T_target: np.ndarray,
        q_seed: Sequence[float],
        max_iter: int = 500,
        dls_lambda: float = 0.03,
        weights: Optional[Sequence[float]] = None,
    ) -> Tuple[np.ndarray, float, float]:
        """Return (q, pos_err_m, ori_err_rad). q is clipped to the joint limits.

        ``weights`` (per joint, default all 1) is the joint-motion cost of the
        weighted damped least-squares step: a large weight makes that joint
        move only when the target cannot be reached otherwise.
        """
        Winv = np.diag(1.0 / np.asarray(weights if weights is not None else np.ones(5), dtype=float))
        q = np.clip(np.asarray(q_seed, dtype=float), self.lower_rad, self.upper_rad)
        pos_err = float("inf")
        ori_err = float("inf")
        for _ in range(max_iter):
            T = self.gripper_pose(arm, q)
            dp = T_target[:3, 3] - T[:3, 3]
            pos_err = float(np.linalg.norm(dp))
            R = T_target[:3, :3] @ T[:3, :3].T
            ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
            ori_err = float(ang)
            if ang < 1e-9:
                aa = np.zeros(3)
            else:
                aa = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) \
                    / (2.0 * math.sin(ang)) * ang
            if pos_err < 1e-4 and ori_err < 1e-3:
                break
            J = self._jacobian(arm, q, T)
            dx = np.concatenate([dp, aa])
            dq = Winv @ J.T @ np.linalg.solve(J @ Winv @ J.T + dls_lambda ** 2 * np.eye(6), dx)
            q = np.clip(q + dq, self.lower_rad, self.upper_rad)
        return q, pos_err, ori_err

    def ik_planar(
        self, arm: str, T_target: np.ndarray, q_seed: Sequence[float], max_iter: int = 200
    ) -> np.ndarray:
        """Solve only joints 2-4 (lift, elbow, wrist_flex) for the target's
        in-plane position (chassis Y, Z) and pitch; pan and roll keep the seed.

        The SO-101 arm plane is the chassis Y-Z plane at pan 0, so this is the
        owner's "no joint 1" motion. Out-of-plane error is left for :meth:`ik`.
        """
        q = np.clip(np.asarray(q_seed, dtype=float), self.lower_rad, self.upper_rad)
        for _ in range(max_iter):
            T = self.gripper_pose(arm, q)
            R = T_target[:3, :3] @ T[:3, :3].T
            e = np.r_[T_target[1:3, 3] - T[1:3, 3], (R[2, 1] - R[1, 2]) / 2.0]
            if float(np.linalg.norm(e)) < 1e-9:
                break
            J = self._jacobian(arm, q, T)
            Js = np.vstack([J[1:3, 1:4], J[3:4, 1:4]])
            dq = Js.T @ np.linalg.solve(Js @ Js.T + 1e-6 * np.eye(3), e)
            q[1:4] = np.clip(q[1:4] + dq, self.lower_rad[1:4], self.upper_rad[1:4])
        return q

    def ik_shared(
        self,
        targets: Dict[str, np.ndarray],
        q_seed: Sequence[float],
        max_iter: int = 300,
        ori_weight_m: float = 0.05,
    ) -> np.ndarray:
        """One joint vector for BOTH arms (owner 2026-11-11: at the start pose
        the arms match each other). Least squares over both grippers' targets.

        The arm bases (182.4 mm) and the attachment pitch (ATTACHMENT_PITCH_MM) differ,
        so the shared pose leaves a small residual on each grip; callers report
        it. Orientation error is weighted as ``ori_weight_m`` metres per radian.
        """
        q = np.clip(np.asarray(q_seed, dtype=float), self.lower_rad, self.upper_rad)
        for _ in range(max_iter):
            es, Js = [], []
            for arm, T_target in targets.items():
                T = self.gripper_pose(arm, q)
                R = T_target[:3, :3] @ T[:3, :3].T
                aa = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
                es.append(np.r_[T_target[:3, 3] - T[:3, 3], aa * ori_weight_m])
                J = self._jacobian(arm, q, T)
                J[3:] *= ori_weight_m
                Js.append(J)
            e, J = np.concatenate(es), np.vstack(Js)
            dq = np.linalg.lstsq(J, e, rcond=None)[0]
            q = np.clip(q + dq, self.lower_rad, self.upper_rad)
            if float(np.linalg.norm(dq)) < 1e-10:
                break
        return q

    def _jacobian(self, arm: str, q: np.ndarray, T: np.ndarray) -> np.ndarray:
        J = np.zeros((6, 5))
        eps = 1e-6
        for i in range(5):
            dq = np.zeros(5)
            dq[i] = eps
            Tp = self.gripper_pose(arm, q + dq)
            J[:3, i] = (Tp[:3, 3] - T[:3, 3]) / eps
            Rr = Tp[:3, :3] @ T[:3, :3].T
            a2 = math.acos(max(-1.0, min(1.0, (np.trace(Rr) - 1.0) / 2.0)))
            if a2 < 1e-9:
                ar = np.zeros(3)
            else:
                ar = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]) \
                    / (2.0 * math.sin(a2)) * a2 / eps
            J[3:, i] = ar
        return J


# ---------------------------------------------------------------------------
# Attachment-point model (mesh features)
# ---------------------------------------------------------------------------


@dataclass
class AttachmentModel:
    """Attachment points as cell-mesh features along the tube axis.

    ``index`` 0 is the MIDDLE point (left gripper at the printing pose), +1 the
    FAR point (right gripper), -1 the NEAR point (free, left regrip target).
    Point ``k`` at tube advance ``adv`` has chassis pose
    ``T0_middle`` translated by ``(k * pitch + adv)`` along ``tube_axis``.
    """

    tube_axis: np.ndarray
    pitch_m: float
    T0_middle: np.ndarray            # middle-point pose at the printing pose, adv=0
    axis_extent_m: Tuple[float, float]  # [min, max] of the tube mesh along the axis

    def point_pose(self, index: int, tube_advance_m: float) -> np.ndarray:
        T = self.T0_middle.copy()
        T[:3, 3] = T[:3, 3] + self.tube_axis * (index * self.pitch_m + tube_advance_m)
        return T

    def exists_on_tube(self, index: int) -> bool:
        """True if point ``index`` lies within the modelled tube.

        Checked in the tube's own frame (advance 0, the CAD placement that
        ``axis_extent_m`` was measured at): the tube and its attachment points
        move together, so the advance does not change which points exist.
        """
        proj = float(self.point_pose(index, 0.0)[:3, 3] @ self.tube_axis)
        lo, hi = self.axis_extent_m
        return lo - 1e-6 <= proj <= hi + 1e-6


# ---------------------------------------------------------------------------
# Timeline frame and plan
# ---------------------------------------------------------------------------


@dataclass
class CycleFrame:
    """One rendered/checked frame of the crawl cycle.

    ``left_locked`` / ``right_locked`` record whether each gripper is clamped on
    the tube. The tube is always attached in this cycle, so the invariant is:
    never both unlocked, at most one unlocked at a time.
    """

    step: str
    left_q: np.ndarray
    right_q: np.ndarray
    left_locked: bool
    right_locked: bool
    tube_advance_m: float
    note: str = ""
    #: Full rigid tube motion from its CAD placement (4x4, chassis frame), or
    #: None = translation by ``tube_advance_m`` along the axis only. When set,
    #: ``tube_advance_m`` is the displacement component along the tube axis
    #: (for display) and the checker gets the residual :meth:`tube_transform`.
    tube_motion: Optional[np.ndarray] = None
    tube_axis: Optional[np.ndarray] = None

    @property
    def tube_lift_m(self) -> float:
        return float(self.tube_motion[2, 3]) if self.tube_motion is not None else 0.0

    def tube_transform(self) -> Optional[np.ndarray]:
        """Residual transform for :class:`CellState` (applied after the advance)."""
        if self.tube_motion is None:
            return None
        A = np.eye(4)
        A[:3, 3] = -np.asarray(self.tube_axis, dtype=float) * float(self.tube_advance_m)
        return np.asarray(self.tube_motion, dtype=float) @ A

    def to_json_obj(self) -> Dict[str, object]:
        return {
            "step": self.step,
            "left": [float(x) for x in self.left_q],
            "right": [float(x) for x in self.right_q],
            "left_locked": bool(self.left_locked),
            "right_locked": bool(self.right_locked),
            "tube_advance_m": float(self.tube_advance_m),
            "tube_motion": (None if self.tube_motion is None
                            else np.asarray(self.tube_motion, dtype=float).tolist()),
            "note": self.note,
        }


@dataclass
class FrameViolation:
    frame_index: int
    step: str
    reason: str

    def __str__(self) -> str:
        return f"frame {self.frame_index} [{self.step}]: {self.reason}"


@dataclass
class RepositionLog:
    """An inserted both-arm reposition, with the reason it was inserted."""

    before_step: str
    reason: str
    tube_advance_m: float


@dataclass
class CyclePlan:
    frames: List[CycleFrame]
    printing_pose_left: np.ndarray
    printing_pose_right: np.ndarray
    printing_pose_ik_residual_m: Tuple[float, float]   # (left, right)
    printing_pose_ik_residual_rad: Tuple[float, float]
    violations: List[FrameViolation]
    repositions: List[RepositionLog]
    stopped_reason: Optional[str]
    report: str = ""
    start_kind: str = ""
    lift_m: float = 0.0
    first_carry_m: float = 0.0
    regrip_standoff_m: float = 0.0

    @property
    def ok(self) -> bool:
        return not self.violations and self.stopped_reason is None


# ---------------------------------------------------------------------------
# Per-frame validation
# ---------------------------------------------------------------------------


def _pose_error(Ta: np.ndarray, Tb: np.ndarray) -> Tuple[float, float]:
    dp = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    R = Ta[:3, :3] @ Tb[:3, :3].T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    return dp, float(ang)


def validate_frame(
    frame: CycleFrame,
    kin: CellKinematics,
    *,
    reference_relpose: Optional[np.ndarray],
    rigid_pos_tol_m: float = RIGID_TUBE_POS_TOL_M,
    rigid_ori_tol_rad: float = RIGID_TUBE_ORI_TOL_RAD,
) -> List[str]:
    """Return every reason this frame is invalid (empty list = valid).

    Checks, in order: gripper invariants, joint limits, mesh collision, and —
    only while both grippers are locked — the rigid-tube constraint (the
    relative gripper pose must equal ``reference_relpose`` within tolerance).
    """
    from inchworm.system_model.collision import CellState

    reasons: List[str] = []

    # --- Gripper invariants (tube always attached in this cycle) ---
    if not frame.left_locked and not frame.right_locked:
        reasons.append(
            "gripper invariant: both grippers unlocked while the tube is attached"
        )
    # "at most one unlocked" is the complement of the above for two grippers;
    # both-locked and one-unlocked are allowed. (Kept explicit for clarity.)

    # --- Joint limits (physical cell-URDF range) ---
    for arm, q in (("left", frame.left_q), ("right", frame.right_q)):
        for i, name in enumerate(JOINT_ORDER):
            lo, hi = float(kin.lower_rad[i]), float(kin.upper_rad[i])
            if not (lo - 1e-9 <= float(q[i]) <= hi + 1e-9):
                reasons.append(
                    f"joint limit: {arm} {name} {float(q[i]):.4f} rad outside "
                    f"[{lo:.4f}, {hi:.4f}] rad"
                )

    # --- Configured soft limits (what the live guard enforces) ---
    if kin.soft_limits is not None:
        for arm, q in (("left", frame.left_q), ("right", frame.right_q)):
            lo_s, hi_s = kin.soft_limits[arm]
            for i, name in enumerate(JOINT_ORDER):
                v = float(q[i])
                if not (lo_s[i] - 1e-9 <= v <= hi_s[i] + 1e-9):
                    reasons.append(
                        f"soft limit: {arm} {name} {math.degrees(v):.1f} deg outside the "
                        f"configured [{math.degrees(lo_s[i]):.1f}, {math.degrees(hi_s[i]):.1f}] deg"
                    )

    # --- Mesh collision (exactly what the motion guard checks: pairs that
    #     involve a moving SO-101 link, for each arm) ---
    try:
        state = CellState(
            left_q=np.asarray(frame.left_q, dtype=float),
            right_q=np.asarray(frame.right_q, dtype=float),
            dexarm_q={},
            tube_advance_m=(None if frame.tube_advance_m is None
                            else float(frame.tube_advance_m)),
            tube_transform=frame.tube_transform(),
        )
        seen, contacts = set(), []
        for arm in ("left", "right"):
            for c in kin.checker.check(state, arm=arm, first_only=False):
                k = frozenset((c.link_a, c.link_b))
                if k not in seen:
                    seen.add(k)
                    contacts.append(c)
    except Exception as exc:  # noqa: BLE001 - fail closed
        reasons.append(f"mesh collision check failed (fail closed): {exc}")
        contacts = []
    for c in contacts:
        reasons.append(f"mesh collision: {c}")

    # --- Rigid-tube constraint (both locked only) ---
    if frame.left_locked and frame.right_locked and reference_relpose is not None:
        TL = kin.gripper_pose("left", frame.left_q)
        TR = kin.gripper_pose("right", frame.right_q)
        rel = np.linalg.inv(TL) @ TR
        dpos, dang = _pose_error(rel, reference_relpose)
        if dpos > rigid_pos_tol_m or dang > rigid_ori_tol_rad:
            reasons.append(
                f"rigid-tube constraint: relative gripper pose drifted "
                f"{dpos * 1000:.2f} mm / {math.degrees(dang):.2f} deg from the "
                f"locked reference (tol {rigid_pos_tol_m * 1000:.1f} mm / "
                f"{math.degrees(rigid_ori_tol_rad):.2f} deg) — a locked arm "
                f"cannot hold its grip here"
            )

    return reasons


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


def build_cell_kinematics(cfg, checker) -> CellKinematics:
    """Cell FK/IK bounded by the URDF chain joint limits (the physical range)."""
    jl = {j.name: (j.limit_lower_rad, j.limit_upper_rad) for j in cfg.so101_kinematics.joints}
    lower = np.array([jl[n][0] for n in JOINT_ORDER], dtype=float)
    upper = np.array([jl[n][1] for n in JOINT_ORDER], dtype=float)
    return CellKinematics(checker=checker, lower_rad=lower, upper_rad=upper,
                          soft_limits=configured_soft_limits(cfg))


def configured_soft_limits(cfg) -> Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]]:
    """Per-arm (lower, upper) rad from motion_guard.<arm>_arm_limits; None if absent.

    A null field falls back to the URDF limit, like the motion guard does.
    """
    mg = getattr(cfg, "motion_guard", None)
    if mg is None:
        return None
    jl = {j.name: (j.limit_lower_rad, j.limit_upper_rad) for j in cfg.so101_kinematics.joints}
    out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for arm in ("left", "right"):
        blk = getattr(mg, f"{arm}_arm_limits", None)
        lo, hi = [], []
        for n in JOINT_ORDER:
            a = getattr(blk, f"{n}_lower_rad", None) if blk is not None else None
            b = getattr(blk, f"{n}_upper_rad", None) if blk is not None else None
            lo.append(jl[n][0] if a is None else a)
            hi.append(jl[n][1] if b is None else b)
        out[arm] = (np.array(lo, dtype=float), np.array(hi, dtype=float))
    return out


def build_attachment_model(kin: CellKinematics, working_pose: np.ndarray) -> AttachmentModel:
    """Attachment frames from the working-pose grips (mesh features)."""
    from inchworm.system_model.collision import TUBE_LINK, CellState

    axis = np.asarray(kin.checker.tube_axis, dtype=float)
    T0_middle = kin.gripper_pose("left", working_pose)
    # Tube mesh extent along the axis, at advance 0 (the CAD placement).
    v = np.asarray(kin.checker._mesh[TUBE_LINK].vertices, dtype=float)
    P = kin.checker.link_poses(CellState(np.zeros(5), np.zeros(5), tube_advance_m=0.0))[TUBE_LINK]
    vw = (P @ np.c_[v, np.ones(len(v))].T).T[:, :3]
    proj = vw @ axis
    return AttachmentModel(
        tube_axis=axis,
        pitch_m=float(ATTACHMENT_PITCH_MM) / 1000.0,
        T0_middle=T0_middle,
        axis_extent_m=(float(proj.min()), float(proj.max())),
    )


def _lerp(a: np.ndarray, b: np.ndarray, n: int) -> List[np.ndarray]:
    """n points from a (exclusive) to b (inclusive)."""
    return [(1.0 - t) * a + t * b for t in np.linspace(0.0, 1.0, max(2, n + 1))[1:]]


#: Default lift (chassis +Z) and first carry for the owner cycle (2026-11-11).
#: Chosen on the cell model, NOT measured: sweep 2026-11-11 (lift 130-190 mm x
#: carry 70-110 mm) had the fewest violations at 170 mm / half a pitch; below
#: ~150 mm the regrip retract runs into the arm's reach limit and the arms
#: collide while lowering. Override per run.
DEFAULT_LIFT_M = 0.17
DEFAULT_FIRST_CARRY_FRACTION = 0.5

#: Weighted IK cost per joint (pan, lift, elbow, wrist_flex, roll). The owner
#: (2026-11-11): the crawl should not need joint 1; it is a planar motion of
#: joints 2-4. Pan and roll are heavily penalised so they move only as much as
#: the 1.1 deg CAD tube-axis tilt forces; the report prints how far they moved.
IK_JOINT_WEIGHTS = np.array([1000.0, 1.0, 1.0, 1.0, 1000.0])

UP = np.array([0.0, 0.0, 1.0])


def plan_cycle(
    cfg,
    checker,
    *,
    printing_pose: Optional[Sequence[float]] = None,
    matching_start: bool = True,
    lift_m: float = DEFAULT_LIFT_M,
    first_carry_m: Optional[float] = None,
    regrip_standoff_m: Optional[float] = None,
    steps: int = 6,
) -> CyclePlan:
    """Plan the owner-specified inchworm cycle into a validated timeline.

    Owner cycle (2026-11-11), the tube moves only because the locked arms move it:

    1. start: both locked in the printing pose;
    2. both locked: lift the tube straight up (chassis +Z), away from both arms;
    3. both locked: carry the tube along its axis away from the DexArm
       ("to the right") by ``first_carry_m`` (default half a pitch), so the
       left arm can reach the next attachment point;
    4. left unlocks, retracts down by the standoff, moves under the point one
       pitch closer to the DexArm, inserts, relocks;
    5. right unlocks and does the same onto the point the left arm just left;
    6. both locked: lower the tube while the arms return to their start pose
       (this carries the rest of the pitch). Net advance: one pitch.

    ``printing_pose`` is the single input standing in for the owner's real
    pose; when ``None`` the planner solves it by IK (left on the middle point,
    right on the far point) seeded from the ``folded_transit`` placeholder.
    With ``matching_start`` (default, owner 2026-11-11: "when returning to the
    start position the arms should in general match each other") both arms
    use ONE joint pose, solved for both grips together; the grip residual it
    leaves (arm spacing vs pitch, ~2 mm) is reported, for the learned
    insertion to absorb. An explicit ``printing_pose`` is used as given for
    both arms.
    Raises :class:`PlanStopped` when a regrip target does not exist on the
    modelled tube or the printing-pose IK cannot reach.
    """
    kin = build_cell_kinematics(cfg, checker)
    working = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
    att = build_attachment_model(kin, working)
    pitch = att.pitch_m
    if first_carry_m is None:
        first_carry_m = DEFAULT_FIRST_CARRY_FRACTION * pitch
    if regrip_standoff_m is None:
        appr = getattr(cfg, "approach", None)
        regrip_standoff_m = float(appr.standoff_m) if appr is not None else 0.05
    if not (0.0 < first_carry_m <= pitch):
        raise PlanStopped(f"first carry {first_carry_m:.3f} m must be in (0, pitch]")

    # --- Printing pose: IK left->middle, right->far, seeded from placeholder ---
    seed = np.asarray(printing_pose if printing_pose is not None
                      else FOLDED_TRANSIT_PLACEHOLDER, dtype=float)
    T_mid, T_far = att.point_pose(0, 0.0), att.point_pose(+1, 0.0)
    if printing_pose is not None:
        qL = np.asarray(printing_pose, dtype=float).copy()
        qR = qL.copy()
        start_kind = "given (both arms)"
    elif matching_start:
        qL = kin.ik_shared({"left": T_mid, "right": T_far}, kin.ik("left", T_mid, seed)[0])
        qR = qL.copy()
        start_kind = "matching (one shared IK pose for both arms)"
    else:
        qL = kin.ik("left", T_mid, seed)[0]
        qR = kin.ik("right", T_far, seed)[0]
        start_kind = "per-arm IK"
    peL, aeL = _pose_error(kin.gripper_pose("left", qL), T_mid)
    peR, aeR = _pose_error(kin.gripper_pose("right", qR), T_far)
    # The start grips are where the locked grippers really are: the cycle
    # carries the tube from there, so the shared-pose residual is a start
    # offset, not a reach failure. Only a gross miss stops planning.
    start_tol_m = START_GRIP_TOL_M if printing_pose is not None or matching_start else GRIP_REACH_TOL_M
    start_tol_rad = START_GRIP_TOL_RAD if printing_pose is not None or matching_start else GRIP_REACH_TOL_RAD
    if peL > start_tol_m or aeL > start_tol_rad:
        raise PlanStopped(
            f"printing-pose IK (left on middle point) did not reach: residual "
            f"{peL * 1000:.2f} mm / {math.degrees(aeL):.2f} deg"
        )
    if peR > start_tol_m or aeR > start_tol_rad:
        raise PlanStopped(
            f"printing-pose IK (right on far point) did not reach: residual "
            f"{peR * 1000:.2f} mm / {math.degrees(aeR):.2f} deg"
        )

    # Regrip targets must exist on the modelled tube (never invent one).
    for who, index in (("left (near point)", -1), ("right (middle point)", 0)):
        if not att.exists_on_tube(index):
            raise PlanStopped(
                f"regrip target for {who}, lattice index {index}, does not exist "
                f"on the modelled tube (axis extent "
                f"{att.axis_extent_m[0]:.3f}..{att.axis_extent_m[1]:.3f} m); "
                f"stopping instead of inventing an attachment point"
            )

    # The tube is rigid and is moved only by the locked arms, so it may be
    # translated in any direction. Lift and carry are in the arm plane
    # (chassis Z, then chassis Y); the final segment ends exactly one pitch
    # along the (CAD-tilted, 1.1 deg) tube axis so the grips land back on the
    # printing pose.
    # No yaw: rotating the tube would rotate the attachment frames out of the
    # arm plane (tried 2026-11-11, worse). The ~1 deg CAD axis tilt is left
    # for the small pan/roll the weighted IK adds.
    # Carry direction = the arms' own plane (horizontal), measured from FK at
    # the start pose: a shoulder_lift nudge moves the gripper in that plane.
    # At pan 0 it is chassis +Y; with the matching start's small shared pan
    # it turns with the arms, so the carry needs no joint 1 (owner).
    def plane_dir(arm: str, q: np.ndarray) -> np.ndarray:
        dq = np.zeros(5)
        dq[1] = 1e-4
        d = kin.gripper_pose(arm, q + dq)[:3, 3] - kin.gripper_pose(arm, q)[:3, 3]
        d[2] = 0.0
        n = float(np.linalg.norm(d))
        return d / n if n > 0 else np.array([0.0, 1.0, 0.0])

    Y = plane_dir("left", qL) + plane_dir("right", qR)
    Y = Y / np.linalg.norm(Y)
    if Y[1] < 0:
        Y = -Y   # away from the DexArm
    ax = att.tube_axis
    yaw = 0.0
    pivot = att.point_pose(0, 0.0)[:3, 3]

    def motion(D: np.ndarray, phi: float) -> np.ndarray:
        c, sn = math.cos(phi), math.sin(phi)
        G = np.eye(4)
        G[:3, :3] = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        G[:3, 3] = pivot - G[:3, :3] @ pivot + D
        return G

    # Per-arm grip offset: where each locked gripper actually sits relative
    # to its attachment point at the start (identity for per-arm IK; ~2 mm
    # for the matching start). A regrip lands with the same offset (the same
    # arm inserting the same way), so the arms end in exactly the start pose.
    G = {
        "left": np.linalg.inv(T_mid) @ kin.gripper_pose("left", qL),
        "right": np.linalg.inv(T_far) @ kin.gripper_pose("right", qR),
    }

    def point(index: int, M: np.ndarray, arm: str) -> np.ndarray:
        return M @ att.point_pose(index, 0.0) @ G[arm]

    frames: List[CycleFrame] = []
    refs: List[np.ndarray] = []   # rigid-tube reference per frame (grip pair)
    repositions: List[RepositionLog] = []
    reach: List[Tuple[int, str, str, float, float]] = []  # frame, step, arm, mm, deg
    cur = {"left": qL.copy(), "right": qR.copy()}

    q_start = {"left": qL, "right": qR}

    def solve(arm: str, T: np.ndarray) -> Tuple[float, float]:
        # Planar joints 2-4 first (owner: the crawl is a joint 2-4 motion),
        # then a full weighted solve that adds only the pan/roll the CAD tube
        # tilt forces. Seeded from the previous frame for continuity.
        q0 = kin.ik_planar(arm, T, cur[arm])
        q, pe, ae = kin.ik(arm, T, q0, weights=IK_JOINT_WEIGHTS)
        # The heavily weighted pan converges slowly; where the start pose
        # itself reaches the target (the end of the cycle) and lies close to
        # the solve, use it, so the arms end exactly in the matching start.
        ps, as_ = _pose_error(kin.gripper_pose(arm, q_start[arm]), T)
        if (ps <= pe + 1e-6 and as_ <= max(ae, 1e-6) + 1e-6
                and float(np.abs(q_start[arm] - q).max()) < math.radians(2.0)):
            q, pe, ae = q_start[arm].copy(), ps, as_
        cur[arm] = q
        return pe, ae

    def emit(step, ll, rl, M, note, targets):
        """targets: {arm: T} the arm's gripper must reach this frame."""
        for arm, T in targets.items():
            pe, ae = solve(arm, T)
            if pe > GRIP_REACH_TOL_M or ae > GRIP_REACH_TOL_RAD:
                reach.append((len(frames), step, arm, pe * 1000, math.degrees(ae)))
        adv = float((M @ np.r_[pivot, 1.0])[:3] @ ax - pivot @ ax)
        frames.append(CycleFrame(step, cur["left"].copy(), cur["right"].copy(),
                                 ll, rl, adv, note, M.copy(), ax.copy()))
        # Rigid-tube reference for the CURRENT grip pair, from the attachment
        # geometry (it changes when a regrip relocks on a new point).
        refs.append(np.linalg.inv(att.point_pose(grip["left"], 0.0) @ G["left"])
                    @ (att.point_pose(grip["right"], 0.0) @ G["right"]))

    # grip index per arm (changes at the regrips)
    grip = {"left": 0, "right": +1}

    def carry(step, s0, s1, note):
        """s = (D, phi); both locked grips follow the rigid tube motion."""
        for v in _lerp(np.r_[s0[0], s0[1]], np.r_[s1[0], s1[1]], steps * 2):
            M = motion(v[:3], float(v[3]))
            emit(step, True, True, M, note,
                 {arm: point(grip[arm], M, arm) for arm in ("left", "right")})

    def regrip(arm: str, new_index: int, s, step: str, note: str):
        M = motion(s[0], s[1])
        other = "right" if arm == "left" else "left"
        ll, rl = (False, True) if arm == "left" else (True, False)
        down = UP * (-regrip_standoff_m)
        T_from = point(grip[arm], M, arm)
        T_to = point(new_index, M, arm)
        p_from, p_to = T_from[:3, 3], T_to[:3, 3]
        legs = (("retract", p_from, p_from + down),
                ("move under target", p_from + down, p_to + down),
                ("insert", p_to + down, p_to))
        for leg, a, b in legs:
            n = steps * 2 if leg == "move under target" else steps
            for p in _lerp(a, b, n):
                T = T_from.copy()
                T[:3, 3] = p
                emit(step, ll, rl, M, f"{note}: {leg}", {arm: T})
        grip[arm] = new_index
        emit(step, True, True, M, f"{note}: relocked", {arm: T_to})

    S0 = (np.zeros(3), 0.0)
    S_lift = (UP * lift_m, yaw)
    S_carry = (UP * lift_m + Y * first_carry_m, yaw)
    S_end = (ax * pitch, 0.0)
    emit("printing pose (start)", True, True, motion(*S0),
         "printing pose (IK, seeded from placeholder)", {})

    repositions.append(RepositionLog(
        "lift tube (both locked)",
        f"owner 2026-11-11: first move the tube away from both arms "
        f"(lift {lift_m * 1000:.0f} mm, chassis +Z) to make room for the regrips",
        0.0))
    carry("lift tube (both locked)", S0, S_lift,
          "tube moves only with the locked arms")

    repositions.append(RepositionLog(
        "carry tube right (both locked)",
        f"owner 2026-11-11: carry the tube away from the DexArm "
        f"{first_carry_m * 1000:.1f} mm so the left arm can reach the next point",
        first_carry_m))
    carry("carry tube right (both locked)", S_lift, S_carry,
          "tube moves only with the locked arms")

    regrip("left", -1, S_carry, "left regrip near point",
           "left regrips the point one pitch closer to the DexArm")
    regrip("right", 0, S_carry, "right regrip middle point",
           "right regrips the point the left arm just left")

    repositions.append(RepositionLog(
        "lower tube, arms return to start (both locked)",
        "owner 2026-11-11: move the tube down again and return the arms to the "
        "start pose (carries the rest of the pitch)",
        pitch))
    carry("lower tube, arms return to start (both locked)",
          S_carry, S_end,
          "net tube advance one pitch, away from the DexArm")

    # --- Validate every frame ---
    violations: List[FrameViolation] = []
    for i, step, arm, mm, deg in reach:
        violations.append(FrameViolation(
            i, step, f"reach: {arm} gripper misses its target by {mm:.2f} mm / "
                     f"{deg:.2f} deg (tol {GRIP_REACH_TOL_M * 1000:.1f} mm)"))
    for i, fr in enumerate(frames):
        for reason in validate_frame(fr, kin, reference_relpose=refs[i]):
            violations.append(FrameViolation(i, fr.step, reason))
    violations.sort(key=lambda v: v.frame_index)

    plan = CyclePlan(
        frames=frames,
        printing_pose_left=qL,
        printing_pose_right=qR,
        printing_pose_ik_residual_m=(peL, peR),
        printing_pose_ik_residual_rad=(aeL, aeR),
        violations=violations,
        repositions=repositions,
        stopped_reason=None,
        start_kind=start_kind,
        lift_m=float(lift_m),
        first_carry_m=float(first_carry_m),
        regrip_standoff_m=float(regrip_standoff_m),
    )
    plan.report = build_report(plan, att)
    return plan


def build_report(plan: CyclePlan, att: AttachmentModel) -> str:
    """Human-readable report; LEADS with the placeholder finding."""
    qL, qR = plan.printing_pose_left, plan.printing_pose_right
    peL, peR = plan.printing_pose_ik_residual_m
    aeL, aeR = plan.printing_pose_ik_residual_rad
    stored = FOLDED_TRANSIT_PLACEHOLDER

    def fmt(q):
        return "[" + ", ".join(f"{v:+.4f}" for v in q) + "]"

    lines: List[str] = []
    lines.append("AA-45 inchworm cycle plan — report")
    lines.append("=" * 60)
    lines.append("")
    lines.append("PLACEHOLDER FINDING (read first)")
    lines.append("-" * 60)
    lines.append(
        "The printing pose used here is NOT a measured pose. It is solved on\n"
        "the cell model with the LEFT gripper on the middle attachment point\n"
        "and the RIGHT gripper on the far point (tube at its CAD placement,\n"
        "accepted by the owner 2026-11-11), seeded from the hardware.yaml\n"
        "folded_transit PLACEHOLDER. It is never written to hardware.yaml.\n"
        "Lift, first carry and regrip standoff are model-chosen, not measured."
    )
    lines.append("")
    lines.append(f"  start pose kind : {plan.start_kind}")
    lines.append("")
    lines.append(f"  stored folded_transit placeholder : {fmt(stored)} rad")
    lines.append(f"  printing pose LEFT  on middle : {fmt(qL)} rad  "
                 f"(residual {peL * 1000:.2f} mm / {math.degrees(aeL):.2f} deg)")
    lines.append(f"  printing pose RIGHT on far    : {fmt(qR)} rad  "
                 f"(residual {peR * 1000:.2f} mm / {math.degrees(aeR):.2f} deg)")
    lines.append("")
    lines.append("CYCLE (owner 2026-11-11)")
    lines.append("-" * 60)
    lines.append("  1 lift tube (both locked) -> 2 carry right (both locked) ->")
    lines.append("  3 left regrip -> 4 right regrip -> 5 lower + return (both locked)")
    lines.append(f"  attachment pitch        : {att.pitch_m * 1000:.3f} mm")
    lines.append(f"  tube axis (chassis)     : {np.round(att.tube_axis, 4).tolist()}")
    lines.append(f"  tube axis extent        : "
                 f"{att.axis_extent_m[0]:.3f} .. {att.axis_extent_m[1]:.3f} m "
                 f"({(att.axis_extent_m[1] - att.axis_extent_m[0]) / att.pitch_m:.2f} pitches)")
    lines.append(f"  lift (chassis +Z)       : {plan.lift_m * 1000:.1f} mm")
    lines.append(f"  first carry             : {plan.first_carry_m * 1000:.1f} mm "
                 f"({plan.first_carry_m / att.pitch_m:.2f} pitch)")
    lines.append(f"  regrip standoff (down)  : {plan.regrip_standoff_m * 1000:.1f} mm")
    lines.append(f"  frames                  : {len(plan.frames)}")
    lines.append(f"  both-arm moves          : {len(plan.repositions)}")
    for rp in plan.repositions:
        lines.append(f"    - {rp.before_step}: {rp.reason}")
    lines.append("")
    lines.append("JOINT USE (max |q - start| over the cycle, deg)")
    lines.append("-" * 60)
    for arm, q0 in (("left", qL), ("right", qR)):
        Q = np.array([fr.left_q if arm == "left" else fr.right_q for fr in plan.frames])
        d = np.degrees(np.abs(Q - q0).max(axis=0))
        lines.append(f"  {arm:5}: " + "  ".join(f"{n} {v:6.2f}" for n, v in zip(JOINT_ORDER, d)))
    lines.append("")
    lines.append("VALIDATION")
    lines.append("-" * 60)
    if plan.stopped_reason:
        lines.append(f"  STOPPED: {plan.stopped_reason}")
    lines.append(f"  violations              : {len(plan.violations)}")
    shown = 0
    for v in plan.violations:
        if shown < 25:
            lines.append(f"    - {v}")
            shown += 1
    if len(plan.violations) > shown:
        lines.append(f"    ... and {len(plan.violations) - shown} more")
    lines.append("")
    lines.append(f"  result: {'OK (no violations)' if plan.ok else 'VIOLATIONS PRESENT'}")
    return "\n".join(lines)


def write_timeline_jsonl(plan: CyclePlan, path) -> None:
    """Write one JSON object per frame to ``path`` (JSONL)."""
    import json
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for fr in plan.frames:
            f.write(json.dumps(fr.to_json_obj()) + "\n")
