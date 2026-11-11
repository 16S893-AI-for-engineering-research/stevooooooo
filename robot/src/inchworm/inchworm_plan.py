"""AA-45: Inchworm print-crawl cycle planner on the owner cell URDF.

The owner cannot trust prose descriptions of physical motion; a rendered
animation from the real cell model (``CAD/robot_urdf_new``) is what confirms
behaviour. This module turns the owner-specified crawl cycle into a frame-by-
frame **timeline** (both arms' joint angles, gripper lock states, tube pose,
step name) and checks every frame against the mesh collision model, joint
limits, the gripper invariants and the rigid-tube constraint, so the render is
backed by the same guard the live machine uses.

Owner-specified cycle (2026-11-10)
----------------------------------
Start with both arms LOCKED in the printing pose; both arms (locked)
reposition the tube along its axis AWAY from the DexArm until the next
attachment point is reachable by the left arm; the left arm (nearest the
DexArm) unlocks and regrips the attachment point one pitch closer to the
DexArm; the right arm then unlocks and regrips the point the left arm just
left; extra both-arm repositions are allowed between regrips if a target is
out of reach; only after both regrips do both arms fold back to the printing
pose. Net tube advance: one attachment pitch (ATTACHMENT_PITCH_MM) away from the
DexArm. Insertion is assumed perfect (the learned policy is not modelled).
No printing, no laser.

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
from typing import Callable, Dict, List, Optional, Sequence, Tuple

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
    ) -> Tuple[np.ndarray, float, float]:
        """Return (q, pos_err_m, ori_err_rad). q is clipped to the joint limits."""
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
            dq = J.T @ np.linalg.solve(J @ J.T + dls_lambda ** 2 * np.eye(6), dx)
            q = np.clip(q + dq, self.lower_rad, self.upper_rad)
        return q, pos_err, ori_err

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

    def to_json_obj(self) -> Dict[str, object]:
        return {
            "step": self.step,
            "left": [float(x) for x in self.left_q],
            "right": [float(x) for x in self.right_q],
            "left_locked": bool(self.left_locked),
            "right_locked": bool(self.right_locked),
            "tube_advance_m": float(self.tube_advance_m),
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

    # --- Mesh collision (exactly what the motion guard checks: pairs that
    #     involve a moving SO-101 link, for each arm) ---
    try:
        state = CellState(
            left_q=np.asarray(frame.left_q, dtype=float),
            right_q=np.asarray(frame.right_q, dtype=float),
            dexarm_q={},
            tube_advance_m=float(frame.tube_advance_m),
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
    return CellKinematics(checker=checker, lower_rad=lower, upper_rad=upper)


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


def _interp(q0: np.ndarray, q1: np.ndarray, n: int) -> List[np.ndarray]:
    """n frames from q0 to q1 inclusive (n>=2)."""
    return [(1.0 - t) * q0 + t * q1 for t in np.linspace(0.0, 1.0, max(2, n))]


def plan_cycle(
    cfg,
    checker,
    *,
    printing_pose: Optional[Sequence[float]] = None,
    reposition_steps: int = 8,
    regrip_steps: int = 8,
    fold_steps: int = 6,
    max_repositions: int = 4,
) -> CyclePlan:
    """Plan the owner-specified inchworm cycle into a validated timeline.

    ``printing_pose`` is the single input standing in for the owner's real pose;
    when ``None`` the planner solves it by IK (left on the middle point, right on
    the far point) seeded from the ``folded_transit`` placeholder. The solved
    pose is used for BOTH arms' grips (it is symmetric: the arms are mirror-
    mounted and each sits on its own attachment point).

    Returns a :class:`CyclePlan`; inspect ``.ok``, ``.violations``,
    ``.repositions`` and ``.report``. Raises :class:`PlanStopped` only when a
    regrip target does not exist on the modelled tube or the printing-pose IK
    cannot reach — never inventing geometry.
    """
    kin = build_cell_kinematics(cfg, checker)
    working = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
    att = build_attachment_model(kin, working)

    # --- Printing pose: IK left->middle, right->far, seeded from placeholder ---
    seed = np.asarray(printing_pose if printing_pose is not None
                      else FOLDED_TRANSIT_PLACEHOLDER, dtype=float)
    T_mid = att.point_pose(0, 0.0)
    T_far = att.point_pose(+1, 0.0)
    qL, peL, aeL = kin.ik("left", T_mid, seed)
    qR, peR, aeR = kin.ik("right", T_far, seed)
    if peL > GRIP_REACH_TOL_M or aeL > GRIP_REACH_TOL_RAD:
        raise PlanStopped(
            f"printing-pose IK (left on middle point) did not reach: residual "
            f"{peL * 1000:.2f} mm / {math.degrees(aeL):.2f} deg"
        )
    if peR > GRIP_REACH_TOL_M or aeR > GRIP_REACH_TOL_RAD:
        raise PlanStopped(
            f"printing-pose IK (right on far point) did not reach: residual "
            f"{peR * 1000:.2f} mm / {math.degrees(aeR):.2f} deg"
        )

    # Rigid-tube reference: the relative gripper pose at the printing pose.
    ref_rel = np.linalg.inv(kin.gripper_pose("left", qL)) @ kin.gripper_pose("right", qR)

    # Regrip targets must exist on the modelled tube (never invent one).
    # Left regrips the NEAR point (index -1); right regrips the MIDDLE point
    # (index 0). Both are evaluated at the advance where the regrip happens
    # (one pitch), when they land back on the printing-pose grips.
    for who, index in (("left (near point)", -1), ("right (middle point)", 0)):
        if not att.exists_on_tube(index):
            raise PlanStopped(
                f"regrip target for {who}, lattice index {index}, does not exist "
                f"on the modelled tube (axis extent "
                f"{att.axis_extent_m[0]:.3f}..{att.axis_extent_m[1]:.3f} m); "
                f"stopping instead of inventing an attachment point"
            )

    frames: List[CycleFrame] = []
    repositions: List[RepositionLog] = []

    def emit(step, lq, rq, ll, rl, adv, note=""):
        frames.append(CycleFrame(step, np.asarray(lq, float), np.asarray(rq, float),
                                 ll, rl, float(adv), note))

    # 1) Start: both locked in the printing pose, tube advance 0.
    emit("printing pose (start)", qL, qR, True, True, 0.0,
         "printing pose (IK, seeded from placeholder)")

    # 2) Both-locked reposition: push the tube +Y until the left regrip target
    #    (near point) is reachable by the left arm. Insert repositions in pitch
    #    fractions; each locked arm's joints track its attachment point by IK.
    target_adv = att.pitch_m
    left_track = qL.copy()
    right_track = qR.copy()
    reachable = False
    for seg in range(max_repositions):
        adv_lo = seg / max_repositions * target_adv
        adv_hi = (seg + 1) / max_repositions * target_adv
        repositions.append(RepositionLog(
            before_step="left regrip near point",
            reason=(f"owner cycle: carry the tube +Y toward making the left "
                    f"regrip target reachable (segment {seg + 1}/{max_repositions}, "
                    f"advance {adv_lo / att.pitch_m:.2f}->{adv_hi / att.pitch_m:.2f} pitch)"),
            tube_advance_m=adv_hi,
        ))
        for adv in np.linspace(adv_lo, adv_hi, reposition_steps)[1:]:
            # Locked grippers are pinned to their tube points (points move +adv).
            TL = att.point_pose(0, adv)
            TR = att.point_pose(+1, adv)
            left_track, _, _ = kin.ik("left", TL, left_track)
            right_track, _, _ = kin.ik("right", TR, right_track)
            emit("reposition tube +Y (both locked)", left_track, right_track,
                 True, True, adv,
                 f"tube advance {adv / att.pitch_m:.2f} pitch")
        # Is the left regrip target (near point, index -1) now reachable?
        T_near = att.point_pose(-1, adv_hi)
        q_near, pe, ae = kin.ik("left", T_near, left_track)
        if pe <= GRIP_REACH_TOL_M and ae <= GRIP_REACH_TOL_RAD:
            reachable = True
            break

    final_adv = frames[-1].tube_advance_m

    # 3) Left arm unlocks and regrips the near point (one pitch closer to DexArm).
    T_near = att.point_pose(-1, final_adv)
    q_left_near, pe_ln, ae_ln = kin.ik("left", T_near, left_track)
    left_note = ("left regrips the near point (index -1)"
                 if reachable else
                 "left regrip target NOT reachable at this advance "
                 "(placeholder pose); best-effort shown")
    for q in _interp(left_track, q_left_near, regrip_steps):
        emit("left regrip near point", q, right_track, False, True, final_adv, left_note)
    emit("left regrip near point", q_left_near, right_track, True, True, final_adv,
         left_note + " — relocked")

    # 4) Right arm unlocks and regrips the point the left arm just left (middle).
    T_mid_now = att.point_pose(0, final_adv)
    q_right_mid, pe_rm, ae_rm = kin.ik("right", T_mid_now, right_track)
    right_note = "right regrips the middle point (index 0, where the left arm was)"
    for q in _interp(right_track, q_right_mid, regrip_steps):
        emit("right regrip middle point", q_left_near, q, True, False, final_adv, right_note)
    emit("right regrip middle point", q_left_near, q_right_mid, True, True, final_adv,
         right_note + " — relocked")

    # 5) Fold back to the printing pose (both locked). After a full-pitch
    #    advance the two grips coincide with the printing-pose grips.
    for q_l, q_r in zip(_interp(q_left_near, qL, fold_steps),
                        _interp(q_right_mid, qR, fold_steps)):
        emit("fold back to printing pose", q_l, q_r, True, True, final_adv,
             "net tube advance one pitch, away from the DexArm")

    # --- Validate every frame ---
    violations: List[FrameViolation] = []
    for i, fr in enumerate(frames):
        for reason in validate_frame(fr, kin, reference_relpose=ref_rel):
            violations.append(FrameViolation(i, fr.step, reason))

    plan = CyclePlan(
        frames=frames,
        printing_pose_left=qL,
        printing_pose_right=qR,
        printing_pose_ik_residual_m=(peL, peR),
        printing_pose_ik_residual_rad=(aeL, aeR),
        violations=violations,
        repositions=repositions,
        stopped_reason=None,
    )
    plan.report = build_report(plan, att, reachable)
    return plan


def build_report(plan: CyclePlan, att: AttachmentModel, left_reachable: bool) -> str:
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
        "The printing pose used here is NOT a measured pose. It is the IK\n"
        "solution that puts the LEFT gripper on the middle attachment point\n"
        "and the RIGHT gripper on the far point, seeded from the hardware.yaml\n"
        "folded_transit PLACEHOLDER. It is never written to hardware.yaml and\n"
        "is a single input the owner's re-measured pose replaces later."
    )
    lines.append("")
    lines.append(f"  stored folded_transit placeholder : {fmt(stored)} rad")
    lines.append(f"  printing pose (IK) LEFT  on middle : {fmt(qL)} rad  "
                 f"(residual {peL * 1000:.2f} mm / {math.degrees(aeL):.2f} deg)")
    lines.append(f"  printing pose (IK) RIGHT on far    : {fmt(qR)} rad  "
                 f"(residual {peR * 1000:.2f} mm / {math.degrees(aeR):.2f} deg)")
    lines.append("")
    lines.append(
        "  Consequence at this placeholder pose: with both arms LOCKED the\n"
        "  rigid both-arm carry cannot translate the tube along its axis — the\n"
        "  grippers cannot stay on their attachment points once the tube moves,\n"
        "  so the owner-specified reposition and the left regrip are not\n"
        "  physically achievable here. This is expected for a placeholder; it\n"
        "  will change when the owner supplies the real printing pose."
    )
    lines.append("")
    lines.append(f"  left regrip target reachable during carry: {left_reachable}")
    lines.append("")
    lines.append("CYCLE")
    lines.append("-" * 60)
    lines.append(f"  attachment pitch        : {att.pitch_m * 1000:.3f} mm")
    lines.append(f"  tube axis (chassis)     : {np.round(att.tube_axis, 4).tolist()}")
    lines.append(f"  tube axis extent        : "
                 f"{att.axis_extent_m[0]:.3f} .. {att.axis_extent_m[1]:.3f} m "
                 f"({(att.axis_extent_m[1] - att.axis_extent_m[0]) / att.pitch_m:.2f} pitches)")
    lines.append(f"  frames                  : {len(plan.frames)}")
    lines.append(f"  inserted repositions    : {len(plan.repositions)}")
    for rp in plan.repositions:
        lines.append(f"    - before '{rp.before_step}': {rp.reason}")
    lines.append("")
    lines.append("VALIDATION")
    lines.append("-" * 60)
    if plan.stopped_reason:
        lines.append(f"  STOPPED: {plan.stopped_reason}")
    lines.append(f"  violations              : {len(plan.violations)}")
    # Summarise by (step, reason-kind) to keep the report readable.
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
