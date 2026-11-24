"""AA-54 (owner plan 2026-11-21): perturbed tube placement for the per-arm insertion recordings.

Owner rule: the attachment point in front of the INSERTING gripper is rotated up to 2 deg (any
axis) and translated up to 20 mm (each of x/y/z, cell frame) from its pose at the FROZEN
training position (``config/training_poses.json``). NOT a per-joint jitter. The HOLDING arm
(gripper LOCKED, tube rigid in it) realises the perturbation; the inserting arm starts at its
frozen pose every episode.

* Attachment point P0: the inserting arm's ``<arm>_gripper`` link pose at its inserted timeline
  frame (``INSERTED_FRAME``), carried into the frozen tube placement (fixed on the tube).
* Requested perturbation D = rotation about P0 (angle U[0, ROT_MAX_DEG], uniform random axis)
  then translation t (U[-TRANS_MAX_MM, TRANS_MAX_MM] per axis). Holding-gripper target G* = D G0.
* The SO-101 has 5 joints: the solve is a weighted least-squares fit (normalised by the bounds)
  in the holding arm's joint ticks, inside its soft/EEPROM limits. What counts is the ACHIEVED
  perturbation D_ach = G(q) G0^-1 (FK, after rounding to ticks): kept only if it is within the
  bounds, the inserting arm can still reach the perturbed inserted pose (residual within
  ``REACH_*``), and the joint line from the frozen holding pose to it is mesh-clean with the
  tube carried. Otherwise resampled (``MAX_TRIES``, then PerturbError).
* Schedule: N_EPISODES = N_NOMINAL exact (zero perturbation, frozen ticks) + the rest perturbed,
  order shuffled by the seed. Deterministic for a seed.

AA-58 profile ``v6`` (owner 2026-11-24, selectable; AA-54 above stays the default ``aa54``):
per episode the HOLDING arm's wrist_roll (servo 5) is offset from its frozen tick by
U[-V6_ROLL_MAX_DEG, +V6_ROLL_MAX_DEG] and the attachment point is translated by
U[-V6_TRANS_MAX_MM, +V6_TRANS_MAX_MM] per x/y/z (cell frame), each uniform over the whole range
(owner: values near zero as likely as near the limits). The roll tick is held fixed and the other
four holding joints are fitted (same Gauss-Newton) so that P0, carried by the rolled gripper, lands
at P0 + t. Kept only if the FK roll (about the holding wrist_roll axis) and the FK translation of
P0 are inside the bounds, plus every AA-54 check (limits, reach, mesh-clean holding line).
Default V6_N_NOMINAL = 0 (every episode differs).

Nothing here opens a port.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from inchworm import training_poses as tp

ROT_MAX_DEG = 2.0                 # owner 2026-11-21
TRANS_MAX_MM = 20.0               # owner 2026-11-21, per axis
N_EPISODES = 50                   # owner 2026-11-21: per arm
N_NOMINAL = 10                    # owner 2026-11-21: at the exact frozen position
INSERTED_FRAME = {"left": 48, "right": 73}   # last open frame of each arm's insertion (timeline)
MAX_TRIES = 200
REACH_POS_MM = 3.0                # inserting arm: residual to the perturbed inserted pose
REACH_ROT_DEG = 1.0
LINE_STEP_DEG = 1.0
SOLVE_STEP_MAX_TICKS = 45.0      # Gauss-Newton step cap (~4 deg)
SCHEMA = "inchworm.perturbations/1"
ARMS = ("left", "right")
DEG_PER_TICK = 360.0 / 4096
V6_ROLL_MAX_DEG = 1.0             # owner 2026-11-24 (AA-58): holding wrist_roll offset
V6_TRANS_MAX_MM = 5.0             # owner 2026-11-24 (AA-58): per axis
V6_N_NOMINAL = 0                  # owner 2026-11-24 (AA-58): every episode differs
PROFILES = ("aa54", "v6")         # aa54 = the AA-54 default
ROLL_JOINT = 4                    # wrist_roll (servo 5) in the J5 order


class PerturbError(RuntimeError):
    pass


def _rotvec(R: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-12:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = math.sin(a)
    if s < 1e-9:                                           # ~180 deg (never here)
        w, V = np.linalg.eigh((R + R.T) / 2)
        return V[:, int(np.argmax(w))] * a
    return v / (2 * s) * a


def _axis_angle(axis: Sequence[float], ang: float) -> np.ndarray:
    k = np.asarray(axis, float)
    n = np.linalg.norm(k)
    if n < 1e-12 or ang == 0.0:
        return np.eye(3)
    k = k / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def about_point(P: np.ndarray, R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    """Rigid motion: rotate by R about the point P[:3, 3], then translate by t (metres)."""
    p = np.asarray(P, float)[:3, 3]
    D = np.eye(4)
    D[:3, :3] = R
    D[:3, 3] = p + np.asarray(t, float) - R @ p
    return D


def measure(D: np.ndarray, P0: np.ndarray) -> Dict[str, object]:
    """Achieved perturbation of the attachment point: rotation (deg) and translation of P (mm)."""
    rv = np.degrees(_rotvec(np.asarray(D)[:3, :3]))
    p0 = np.asarray(P0)[:3, 3]
    p1 = (np.asarray(D) @ np.append(p0, 1.0))[:3]
    return {"rot_deg": round(float(np.linalg.norm(rv)), 4), "rotvec_deg": [round(float(x), 4) for x in rv],
            "trans_mm": [round(float(x), 3) for x in (p1 - p0) * 1000.0]}


def within_bounds(m: Dict[str, object], rot_max: float = ROT_MAX_DEG, trans_max: float = TRANS_MAX_MM) -> bool:
    return m["rot_deg"] <= rot_max + 1e-9 and all(abs(x) <= trans_max + 1e-9 for x in m["trans_mm"])


@dataclass
class Episode:
    index: int
    nominal: bool
    holding_ticks: List[int]
    tube_pose: List[List[float]]
    requested: Dict[str, object] = field(default_factory=dict)
    achieved: Dict[str, object] = field(default_factory=dict)
    insert_reach: Dict[str, float] = field(default_factory=dict)
    insert_ticks: List[int] = field(default_factory=list)   # inserting arm at the perturbed inserted pose
    tries: int = 0
    line_checks: int = 0


class Planner:
    """Perturbations for one inserting arm at its frozen training position. ``checker`` = the cell
    mesh checker (CellCollisionChecker: link_poses + check). ``timeline_frames`` = the clamped
    timeline (cycle_replay.clamp_to_eeprom ... elbow_cap=FOLD_ELBOW_CAP_TICKS)."""

    def __init__(self, cfg, checker, inserting_arm: str, timeline_frames, limits: Dict[str, Dict[str, tuple]],
                 poses_path: Path = tp.PATH):
        from inchworm.handoff_return import ticks_to_q
        if inserting_arm not in ARMS:
            raise PerturbError(f"inserting arm must be left/right, got {inserting_arm!r}")
        self.cfg, self.checker, self.arm = cfg, checker, inserting_arm
        self.pos = tp.for_arm(inserting_arm, poses_path)
        self.hold = self.pos["holding_arm"]
        self.ticks = {a: [int(t) for t in v] for a, v in self.pos["joint_ticks"].items()}
        self.limits = limits
        self._t2q = lambda a, t: np.asarray(ticks_to_q(cfg, a, t), float)
        self.tube0 = np.asarray(self.pos["tube_pose"], float)
        self.G0 = self._grip(self.hold, self.ticks[self.hold])
        fi = timeline_frames[INSERTED_FRAME[inserting_arm]]
        if fi.tube is None:
            raise PerturbError(f"timeline frame {fi.index} has no tube pose")
        if fi.locked[inserting_arm] or not fi.locked[self.hold]:
            raise PerturbError(f"timeline frame {fi.index}: want the {inserting_arm} open and the {self.hold} LOCKED")
        lp = self._poses(fi.q["left"], fi.q["right"])
        # the attachment point is fixed on the tube: carry it into the frozen tube placement
        self.P0 = self.tube0 @ np.linalg.inv(np.asarray(fi.tube, float)) @ lp[f"{inserting_arm}_gripper"]
        self.q_ins_inserted = np.asarray(fi.q[inserting_arm], float)
        self.line_checks = 0

    # -- kinematics in the cell frame ---------------------------------------------------
    def _poses(self, ql, qr) -> Dict[str, np.ndarray]:
        from inchworm.system_model.collision import CellState
        return self.checker.link_poses(CellState(left_q=np.asarray(ql, float), right_q=np.asarray(qr, float),
                                                 dexarm_q={}))

    def _q_pair(self, arm, ticks) -> tuple:
        other = "right" if arm == "left" else "left"
        q = {arm: self._t2q(arm, ticks), other: self._t2q(other, self.ticks[other])}
        return q["left"], q["right"]

    def _grip(self, arm, ticks) -> np.ndarray:
        return np.asarray(self._poses(*self._q_pair(arm, ticks))[f"{arm}_gripper"], float)

    def _solve(self, arm, x0, target, P, iters: int = 30, free=(0, 1, 2, 3, 4),
               w_mm: float = TRANS_MAX_MM, w_deg: float = ROT_MAX_DEG):
        """Weighted Gauss-Newton in ticks: pose error of the point P carried by ``arm``'s gripper
        (P = G(x0)-relative), normalised by the bounds (``w_mm`` / ``w_deg``). Only the joints in
        ``free`` move (AA-58 v6 holds wrist_roll). Returns float ticks inside the limits."""
        lim = self.limits[arm]
        lo = np.array([lim[j][0] for j in lim], float)
        hi = np.array([lim[j][1] for j in lim], float)
        x = np.clip(np.asarray(x0, float), lo, hi)
        rel = np.linalg.inv(self._grip(arm, x0)) @ P            # P in the gripper frame
        tp_, tR = target[:3, 3], target[:3, :3]
        w = np.array([1 / w_mm] * 3 + [1 / w_deg] * 3)
        free = list(free)
        nf = len(free)

        def res(xx):
            Q = self._grip(arm, xx) @ rel
            e = np.concatenate([(Q[:3, 3] - tp_) * 1000.0, np.degrees(_rotvec(Q[:3, :3] @ tR.T))])
            return e * w

        r = res(x)
        for _ in range(iters):
            J = np.empty((6, nf))
            for c, k in enumerate(free):
                d = np.zeros(5); d[k] = 1.0
                J[:, c] = res(x + d) - r
            dx = np.zeros(5)
            dx[free] = np.linalg.solve(J.T @ J + 1e-4 * np.eye(nf), -J.T @ r)
            step = float(np.abs(dx).max())
            if step > SOLVE_STEP_MAX_TICKS:
                dx *= SOLVE_STEP_MAX_TICKS / step
            xn = np.clip(x + dx, lo, hi)
            rn = res(xn)
            if float(rn @ rn) >= float(r @ r) - 1e-12:
                break
            x, r = xn, rn
            if step < 0.05:
                break
        return x

    # -- one perturbation ---------------------------------------------------------------
    def realise(self, D: np.ndarray):
        """(holding ticks, achieved D, reach residual) for the requested D, or None if refused."""
        target_G = D @ self.G0
        x = self._solve(self.hold, self.ticks[self.hold], target_G, self.G0)
        ht = [int(round(v)) for v in x]
        D_ach = self._grip(self.hold, ht) @ np.linalg.inv(self.G0)
        return ht, D_ach

    def roll_axis(self) -> np.ndarray:
        """Unit axis (cell frame) of the holding wrist_roll at the frozen pose, from FK."""
        if getattr(self, "_roll_axis", None) is None:
            x = np.asarray(self.ticks[self.hold], float)
            d = np.zeros(5); d[ROLL_JOINT] = 10.0
            rv = _rotvec(self._grip(self.hold, x + d)[:3, :3] @ self.G0[:3, :3].T)
            self._roll_axis = rv / np.linalg.norm(rv)
        return self._roll_axis

    def realise_v6(self, roll_deg: float, t_mm: Sequence[float]):
        """AA-58 v6: holding wrist_roll at frozen + round(roll_deg) ticks (held), the other four
        joints fitted so P0 (carried by the rolled gripper) lands at P0 + t. (ticks, D_ach)."""
        h = self.hold
        x0 = np.asarray(self.ticks[h], float)
        x0[ROLL_JOINT] += round(roll_deg / DEG_PER_TICK)
        Pr = self._grip(h, x0) @ np.linalg.inv(self.G0) @ self.P0      # P0 carried by the rolled gripper
        target = Pr.copy()
        target[:3, 3] = self.P0[:3, 3] + np.asarray(t_mm, float) / 1000.0
        x = self._solve(h, x0, target, Pr, free=(0, 1, 2, 3), w_mm=V6_TRANS_MAX_MM, w_deg=V6_ROLL_MAX_DEG)
        ht = [int(round(v)) for v in x]
        ht[ROLL_JOINT] = int(x0[ROLL_JOINT])
        return ht, self._grip(h, ht) @ np.linalg.inv(self.G0)

    def measure_v6(self, ht: Sequence[int], D_ach: np.ndarray) -> Dict[str, object]:
        """measure() + the servo roll offset, the FK roll about the wrist_roll axis and the rest
        of the FK rotation (tilt), all deg."""
        m = measure(D_ach, self.P0)
        rv = np.degrees(_rotvec(np.asarray(D_ach)[:3, :3]))
        roll = float(rv @ self.roll_axis())
        m.update({"roll_offset_deg": round((ht[ROLL_JOINT] - self.ticks[self.hold][ROLL_JOINT]) * DEG_PER_TICK, 4),
                  "roll_deg": round(roll, 4),
                  "tilt_deg": round(float(np.linalg.norm(rv - roll * self.roll_axis())), 4)})
        return m

    def reach(self, D_ach) -> Dict[str, float]:
        """Inserting arm: best fit to the perturbed inserted pose (residual mm / deg)."""
        from inchworm.handoff_return import q_to_ticks_f
        P1 = D_ach @ self.P0
        x0 = q_to_ticks_f(self.cfg, self.arm, self.q_ins_inserted)
        g0 = self._grip(self.arm, x0)
        x = self._solve(self.arm, x0, P1, g0)
        Q = self._grip(self.arm, x)
        rot = float(np.degrees(np.linalg.norm(_rotvec(Q[:3, :3] @ P1[:3, :3].T))))
        self.last_reach_ticks = [int(round(v)) for v in x]
        return {"pos_mm": round(float(np.linalg.norm(Q[:3, 3] - P1[:3, 3])) * 1000.0, 3),
                "rot_deg": round(rot, 4)}

    def line_problem(self, hold_to: Sequence[int]) -> Optional[str]:
        """Mesh check of the holding arm's joint line frozen -> ``hold_to`` (tube carried), the
        inserting arm at its frozen start. Also the teleop/EEPROM limits. Fails closed."""
        from inchworm.handoff_return import CarriedTubeModel, _line
        lim = self.limits[self.hold]
        model = CarriedTubeModel(self.cfg, self.checker, self.ticks, self.tube0, follower=self.arm)
        a = np.asarray(self.ticks[self.hold], float)
        b = np.asarray(hold_to, float)
        n = max(1, int(math.ceil(float(np.abs(b - a).max()) * 360 / 4096 / LINE_STEP_DEG)))
        for k in range(1, n + 1):
            w = list(a + (b - a) * k / n)
            out = [j for j, t in zip(lim, w) if not lim[j][0] <= t <= lim[j][1]]
            if out:
                return f"outside the limits on {out}"
            p = model.problem(self.ticks[self.arm], w)
            self.line_checks += 1
            if p is not None:
                return p
        return None

    def tube_after(self, hold_ticks) -> np.ndarray:
        return self._grip(self.hold, hold_ticks) @ np.linalg.inv(self.G0) @ self.tube0

    def nominal(self, index: int) -> Episode:
        from inchworm.handoff_return import q_to_ticks_f
        return Episode(index, True, list(self.ticks[self.hold]), self.tube0.tolist(),
                       requested={"angle_deg": 0.0, "axis": [0.0, 0.0, 0.0], "trans_mm": [0.0, 0.0, 0.0]},
                       achieved=measure(np.eye(4), self.P0), insert_reach={"pos_mm": 0.0, "rot_deg": 0.0},
                       insert_ticks=[int(round(v)) for v in q_to_ticks_f(self.cfg, self.arm, self.q_ins_inserted)])

    def sample(self, index: int, rng: np.random.Generator) -> Episode:
        last = None
        for k in range(1, MAX_TRIES + 1):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            ang = float(rng.uniform(0.0, ROT_MAX_DEG))
            t_mm = rng.uniform(-TRANS_MAX_MM, TRANS_MAX_MM, size=3)
            D = about_point(self.P0, _axis_angle(axis, math.radians(ang)), t_mm / 1000.0)
            ht, D_ach = self.realise(D)
            m = measure(D_ach, self.P0)
            if not within_bounds(m):
                last = f"achieved {m} outside the bounds"; continue
            if m["rot_deg"] < 1e-6 and max(abs(x) for x in m["trans_mm"]) < 0.5:
                last = "achieved no perturbation"; continue
            rch = self.reach(D_ach)
            if rch["pos_mm"] > REACH_POS_MM or rch["rot_deg"] > REACH_ROT_DEG:
                last = f"inserting arm cannot reach the perturbed point ({rch})"; continue
            c0 = self.line_checks
            p = self.line_problem(ht)
            if p is not None:
                last = f"holding line: {p}"; continue
            return Episode(index, False, ht, self.tube_after(ht).tolist(),
                           requested={"angle_deg": round(ang, 4), "axis": [round(float(x), 5) for x in axis],
                                      "trans_mm": [round(float(x), 3) for x in t_mm]},
                           achieved=m, insert_reach=rch, insert_ticks=list(self.last_reach_ticks), tries=k, line_checks=self.line_checks - c0)
        raise PerturbError(f"episode {index}: no usable perturbation in {MAX_TRIES} tries (last: {last})")


    def sample_v6(self, index: int, rng: np.random.Generator) -> Episode:
        """AA-58 v6: roll U[-1, 1] deg and t U[-5, 5] mm per axis, each uniform over the whole
        range (rejection only by the checks, never by magnitude)."""
        last = None
        for k in range(1, MAX_TRIES + 1):
            roll, t_mm = draw_v6(rng)
            ht, D_ach = self.realise_v6(roll, t_mm)
            m = self.measure_v6(ht, D_ach)
            if not within_bounds_v6(m):
                last = f"achieved {m} outside the bounds"; continue
            out = [j for j, t in zip(self.limits[self.hold], ht)
                   if not self.limits[self.hold][j][0] <= t <= self.limits[self.hold][j][1]]
            if out:
                last = f"outside the limits on {out}"; continue
            rch = self.reach(D_ach)
            if rch["pos_mm"] > REACH_POS_MM or rch["rot_deg"] > REACH_ROT_DEG:
                last = f"inserting arm cannot reach the perturbed point ({rch})"; continue
            c0 = self.line_checks
            p = self.line_problem(ht)
            if p is not None:
                last = f"holding line: {p}"; continue
            return Episode(index, False, ht, self.tube_after(ht).tolist(),
                           requested={"roll_offset_deg": round(roll, 4),
                                      "trans_mm": [round(float(x), 3) for x in t_mm]},
                           achieved=m, insert_reach=rch, insert_ticks=list(self.last_reach_ticks), tries=k,
                           line_checks=self.line_checks - c0)
        raise PerturbError(f"episode {index}: no usable perturbation in {MAX_TRIES} tries (last: {last})")


def draw_v6(rng: np.random.Generator):
    """One requested v6 perturbation: (roll offset deg, translation mm [3]), each uniform over
    the whole range."""
    roll = float(rng.uniform(-V6_ROLL_MAX_DEG, V6_ROLL_MAX_DEG))
    return roll, rng.uniform(-V6_TRANS_MAX_MM, V6_TRANS_MAX_MM, size=3)


def within_bounds_v6(m: Dict[str, object]) -> bool:
    return (abs(m["roll_deg"]) <= V6_ROLL_MAX_DEG + 1e-9
            and all(abs(x) <= V6_TRANS_MAX_MM + 1e-9 for x in m["trans_mm"]))


def default_nominal(profile: str = "aa54") -> int:
    return {"aa54": N_NOMINAL, "v6": V6_N_NOMINAL}[_profile(profile)]


def bounds(profile: str = "aa54") -> Dict[str, float]:
    if _profile(profile) == "v6":
        return {"roll_max_deg": V6_ROLL_MAX_DEG, "trans_max_mm_per_axis": V6_TRANS_MAX_MM}
    return {"rot_max_deg": ROT_MAX_DEG, "trans_max_mm_per_axis": TRANS_MAX_MM}


def _profile(profile: str) -> str:
    if profile not in PROFILES:
        raise PerturbError(f"unknown perturbation profile {profile!r} (want one of {PROFILES})")
    return profile


def schedule_flags(seed: int, n: int = N_EPISODES, n_nominal: int = N_NOMINAL) -> List[bool]:
    """True = nominal. Exactly ``n_nominal`` of ``n``, order shuffled by the seed."""
    if not 0 <= n_nominal <= n:
        raise PerturbError(f"need 0 <= nominal ({n_nominal}) <= episodes ({n})")
    flags = [True] * n_nominal + [False] * (n - n_nominal)
    np.random.default_rng(seed).shuffle(flags)
    return flags


def plan_schedule(planner: Planner, seed: int, n: int = N_EPISODES, n_nominal: int = N_NOMINAL,
                  profile: str = "aa54") -> List[Episode]:
    sample = planner.sample_v6 if _profile(profile) == "v6" else planner.sample
    flags = schedule_flags(seed, n, n_nominal)
    rng = np.random.default_rng([seed, 1])
    return [planner.nominal(i) if nom else sample(i, rng) for i, nom in enumerate(flags)]


def summary(eps: Sequence[Episode]) -> Dict[str, object]:
    pert = [e for e in eps if not e.nominal]
    out = {"episodes": len(eps), "nominal": len(eps) - len(pert), "perturbed": len(pert)}
    if pert:
        out.update({
            "max_rot_deg": max(e.achieved["rot_deg"] for e in pert),
            "max_abs_trans_mm": [max(abs(e.achieved["trans_mm"][k]) for e in pert) for k in range(3)],
            "mean_rot_deg": round(float(np.mean([e.achieved["rot_deg"] for e in pert])), 3),
            "max_reach_pos_mm": max(e.insert_reach["pos_mm"] for e in pert),
            "tries_total": sum(e.tries for e in pert)})
        if all("roll_deg" in e.achieved for e in pert):          # AA-58 v6
            out.update({"max_abs_roll_deg": max(abs(e.achieved["roll_deg"]) for e in pert),
                        "max_tilt_deg": max(e.achieved["tilt_deg"] for e in pert)})
    return out


def write(path: Path, planner: Planner, seed: int, eps: Sequence[Episode], extra: Optional[dict] = None,
          profile: str = "aa54") -> dict:
    rec = {"schema": SCHEMA, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "profile": _profile(profile),
           "inserting_arm": planner.arm, "holding_arm": planner.hold, "seed": int(seed),
           "training_position": f"train_{planner.arm}", "frozen_ticks": planner.ticks,
           "bounds": bounds(profile),
           "attachment_point_P0": planner.P0.tolist(), "summary": summary(eps),
           "episodes": [asdict(e) for e in eps], **(extra or {})}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rec, indent=1))
    return rec


def teleop_limits(cfg) -> Dict[str, Dict[str, tuple]]:
    """Per arm, per joint tick limits: soft limits inside the EEPROM limits (10-tick margin)."""
    from inchworm import teleop_live as tl
    lim = {}
    for arm in ARMS:
        soft = tl.soft_limit_ticks(cfg, arm)
        y = getattr(cfg.motion_guard, f"{arm}_arm_limits").servo_tick_limits
        lim[arm] = {j: (max(soft[j][0], y[j][0] + 10), min(soft[j][1], y[j][1] - 10)) for j in tl.J5}
    return lim


def build_planner(cfg, inserting_arm: str, timeline: Path, checker=None) -> Planner:
    from inchworm import cycle_replay as cr
    if checker is None:
        from inchworm.motion_guard import build_mesh_collision_guard
        mesh = build_mesh_collision_guard(cfg, "left")
        checker = None if mesh is None else mesh.checker
        if checker is None:
            raise PerturbError("mesh collision model not available")
    frames, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(timeline), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    return Planner(cfg, checker, inserting_arm, frames, teleop_limits(cfg))
