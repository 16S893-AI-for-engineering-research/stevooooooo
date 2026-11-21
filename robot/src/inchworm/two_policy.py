"""AA-55 two-policy crawl sequence (owner plan 2026-11-21): the phase list and the pure
planning helpers of ``lerobot_env/crawl_two_policy.py``. Nothing here opens a port or moves.

Sequence (one launcher, ``lerobot_env/crawl_two_policy.sh``):

1. ``replay_0_42``           cycle frames 0..42 with the tube (scripts/replay_cycle.py).
2. ``left_policy``           LEFT checkpoint inserts, from the frozen ``train_left`` position.
3. ``left_lock``             left gripper LOCKS (GripperLockSM) + owner look y/n.
4. ``right_out``             right gripper UNLOCKS (GripperLockSM: only with the left LOCKED; owner
                             y first), straight out of its key 50 mm (-z of its base frame, as
                             handoff_return's first leg), then a joint line to its clear pose
                             (timeline frame 55 = the end of the cycle's own right retract).
5. ``left_to_train_right``   the left (LOCKED, carrying the tube) joint line to its ``train_right``
                             ticks, ~5 deg/s, right static at its clear pose.
6. ``right_to_train_right``  the right along timeline frames 55..train_right frame (its approach under the next
                             point) to its ``train_right`` ticks, left static.
7. ``right_policy``          RIGHT checkpoint inserts (no joint-offset map), from ``train_right``.
8. ``right_lock``            right gripper LOCKS + owner look y/n.
9. ``home``                  both arms (both LOCKED) to timeline frame 86 (start / printing pose):
                             timeline frames 74..86 with the start offset of each arm faded out
                             if that preflights clean, else one joint line.

Tube model from step 4 on: rigidly carried by the LEFT gripper from the pose where the left
locked (the tube is where frame 42 put it at that moment). Owner 2026-11-21 (supervisor): use
that carried pose (also in the train_right handoff state) and only LOG its difference from the
frozen train_right tube pose.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from inchworm import cycle_replay as cr
from inchworm import training_poses as tp

ARMS = ("left", "right")
J5 = cr.JOINTS
DEG_PER_TICK = 360.0 / 4096

PHASES: Tuple[Tuple[str, str], ...] = (
    ("replay_0_42", "cycle frames 0..42 with the tube (right LOCKED holding, left open at train_left)"),
    ("left_policy", "LEFT policy inserts (left checkpoint)"),
    ("left_lock", "left gripper LOCKS (AA-17) + owner look"),
    ("right_out", "right gripper UNLOCKS (left LOCKED + owner y), right straight out of its key, then clear"),
    ("left_to_train_right", "left (LOCKED, with the tube) to its train_right holding pose"),
    ("right_to_train_right", "right to its train_right start pose (timeline frames 55..train_right frame)"),
    ("right_policy", "RIGHT policy inserts (right checkpoint, no joint-offset map)"),
    ("right_lock", "right gripper LOCKS (AA-17) + owner look"),
    ("home", "both arms (both LOCKED) to the cycle home pose (timeline frame 86)"),
)
PHASE_NAMES = tuple(n for n, _ in PHASES)

POSE_TOL_TICKS = 34                 # = inchworm.handoff.POSE_TOL_TICKS (~3 deg)
LOCK_NEAR_DEG = 8.0                 # = policy_handoff.FINISH_NEAR_DEG
RETRACT_MM = 50.0                   # = handoff_return.RETRACT_MM
RETRACT_STEP_MM = 2.0               # one plan frame (0.2 s) each = 10 mm/s (handoff_return.RETRACT_MM_S)
IK_JUMP_MAX_DEG = 3.0               # = handoff_return.IK_JUMP_MAX_DEG
LINE_DEG_PER_FRAME = 1.0            # joint lines: 1 deg per frame (0.2 s) = 5 deg/s at tempo 1
RIGHT_CLEAR_FRAME = 55              # timeline: right out of its key, under it (end of frames 50..55)
RIGHT_APPROACH_FRAMES = (55, tp.position("train_right")["timeline_frame"])    # timeline: right from clear to under its next point (train_right)
RIGHT_INSERTED_FRAME = 73           # timeline: right planned insert end (finish reference placeholder)
HOME_FRAMES = (74, 86)              # timeline: both LOCKED, lowering to the start / printing pose


class PlanRefused(RuntimeError):
    """A plan could not be made or fails its checks: nothing moved."""


# ---------------------------------------------------------------------------
# Frozen training positions
# ---------------------------------------------------------------------------
def training_pose_problems(name: str, present: Dict[str, Sequence[int]], tol_ticks: int = POSE_TOL_TICKS,
                           path=tp.PATH) -> List[str]:
    """Both arms must read within ``tol_ticks`` of the frozen position ``name`` (every joint)."""
    want = tp.joint_ticks(name, path)
    out = []
    for a in ARMS:
        off = [f"{j} {int(t) - w:+d}" for j, t, w in zip(J5, present[a], want[a]) if abs(int(t) - w) > tol_ticks]
        if off:
            out.append(f"{a} arm off the frozen {name} pose (ticks {', '.join(off)}; max {tol_ticks})")
    return out


def lock_pose_problem(present_ticks: Sequence[int], ref_ticks: Sequence[int], near_deg: float = LOCK_NEAR_DEG,
                      arm: str = "left") -> Optional[str]:
    """The arm locked too far from its policy finish reference to move it with the tube locked."""
    d = np.abs(np.asarray(present_ticks, float) - np.asarray(ref_ticks, float)) * DEG_PER_TICK
    if float(d.max()) > near_deg:
        j = J5[int(d.argmax())]
        return (f"the {arm} arm locked {d.max():.1f} deg ({j}) from its finish reference {list(ref_ticks)} "
                f"(max {near_deg:g}); not moving it with the tube locked")
    return None


# ---------------------------------------------------------------------------
# Tube carried by one gripper
# ---------------------------------------------------------------------------
class CarriedTube:
    """Tube rigidly carried by ``holder``'s gripper: equal to ``tube_ref`` with the arms at ``q_ref``."""

    def __init__(self, checker, holder: str, q_ref: Dict[str, Sequence[float]], tube_ref):
        self.checker, self.holder = checker, holder
        self.carry = np.linalg.inv(self._grip(q_ref["left"], q_ref["right"])) @ np.asarray(tube_ref, float)

    def _grip(self, ql, qr) -> np.ndarray:
        from inchworm.system_model.collision import CellState
        st = CellState(left_q=np.asarray(ql, float), right_q=np.asarray(qr, float), dexarm_q={})
        return np.asarray(self.checker.link_poses(st)[f"{self.holder}_gripper"], dtype=float)

    def __call__(self, ql, qr) -> np.ndarray:
        T = self._grip(ql, qr) @ self.carry
        u, _, vt = np.linalg.svd(T[:3, :3])              # keep it exactly rigid (checker requires it)
        T[:3, :3] = u @ vt
        return T


def tube_offset(a, b) -> Tuple[float, float]:
    """(translation mm, rotation deg) between two tube poses."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    R = a[:3, :3].T @ b[:3, :3]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))))
    return round(float(np.linalg.norm(a[:3, 3] - b[:3, 3])) * 1000.0, 2), round(ang, 3)


# ---------------------------------------------------------------------------
# Frame plans (cycle_replay.Frame lists with contiguous indices from 0 or a timeline index)
# ---------------------------------------------------------------------------
TubeFn = Callable[[np.ndarray, np.ndarray], np.ndarray]


def _frames(qs: Sequence[Dict[str, np.ndarray]], step: str, locked: Dict[str, bool], tube: TubeFn,
            start_index: int = 0) -> List[cr.Frame]:
    return [cr.Frame(start_index + k, step, {a: np.asarray(q[a], float).copy() for a in ARMS}, dict(locked),
                     tube(q["left"], q["right"])) for k, q in enumerate(qs)]


def joint_line(q_from: Dict[str, Sequence[float]], q_to: Dict[str, Sequence[float]],
               deg_per_frame: float = LINE_DEG_PER_FRAME) -> List[Dict[str, np.ndarray]]:
    """Both arms on one straight joint line, at least one point per ``deg_per_frame``."""
    a = {k: np.asarray(q_from[k], float) for k in ARMS}
    b = {k: np.asarray(q_to[k], float) for k in ARMS}
    d = max(math.degrees(float(np.abs(b[k] - a[k]).max())) for k in ARMS)
    n = max(1, int(math.ceil(d / deg_per_frame - 1e-9)))
    return [{k: a[k] + (b[k] - a[k]) * i / n for k in ARMS} for i in range(n + 1)]


def straight_out(chain, q_now: Sequence[float], depth_mm: float = RETRACT_MM,
                 step_mm: float = RETRACT_STEP_MM) -> List[np.ndarray]:
    """IK path straight out of the key: -z of the arm's base frame, orientation held (the
    handoff_return first leg). Raises PlanRefused if unreachable or a step swings a joint."""
    q = np.asarray(q_now, float)
    T0 = chain.fk(q)
    out = [q]
    n = int(math.ceil(depth_mm / step_mm - 1e-9))
    for k in range(1, n + 1):
        Tk = T0.copy()
        Tk[2, 3] -= k * depth_mm / n / 1000.0
        r = chain.ik(Tk, q_seed=out[-1])
        if not r.reachable:
            raise PlanRefused(f"straight out: {k * depth_mm / n:.0f} mm is not reachable with the gripper "
                              f"orientation held (IK error {r.pos_err_m * 1000:.2f} mm)")
        if np.degrees(np.abs(r.q - out[-1])).max() > IK_JUMP_MAX_DEG:
            raise PlanRefused(f"straight out: the arm would swing at {k * depth_mm / n:.0f} mm (IK jump)")
        out.append(np.asarray(r.q, float))
    return out


def plan_right_out(chain_right, timeline: Sequence[cr.Frame], now: Dict[str, np.ndarray],
                   tube: TubeFn) -> List[cr.Frame]:
    """Step 4: right straight out 50 mm, then a joint line to its clear pose (frame 55). Left
    static (LOCKED), right open. Frames 0..n."""
    out = [{"left": now["left"], "right": q} for q in straight_out(chain_right, now["right"])]
    clear = {"left": now["left"], "right": timeline[RIGHT_CLEAR_FRAME].q["right"]}
    out += joint_line(out[-1], clear)[1:]
    return _frames(out, "right straight out + clear", {"left": True, "right": False}, tube)


def plan_left_to_train_right(cfg, now: Dict[str, np.ndarray], tube: TubeFn) -> List[cr.Frame]:
    """Step 5: the left (with the tube) joint line to its frozen train_right ticks; right static."""
    from inchworm.so101_pose_reader import ticks_to_q
    goal = {"left": np.asarray(ticks_to_q(cfg, "left", tp.joint_ticks("train_right")["left"])), "right": now["right"]}
    return _frames(joint_line(now, goal), "left to train_right", {"left": True, "right": False}, tube)


def plan_right_to_train_right(timeline: Sequence[cr.Frame], left_q: np.ndarray, tube: TubeFn) -> List[cr.Frame]:
    """Step 6: timeline frames 55..train_right frame for the right, the left static where it is (LOCKED)."""
    a, b = RIGHT_APPROACH_FRAMES
    sub = cr.resume_frames(timeline, a, b, "left", left_q)
    for f in sub:
        f.tube = tube(f.q["left"], f.q["right"])
    return sub


def plan_home_timeline(timeline: Sequence[cr.Frame], now: Dict[str, np.ndarray], tube: TubeFn) -> List[cr.Frame]:
    """Step 9 (preferred): timeline frames 74..86 with each arm's start offset (actual minus
    frame 74) faded linearly to zero at frame 86. Indices 74..86; frame 74 = where the arms are."""
    a, b = HOME_FRAMES
    if not all(timeline[k].locked["left"] and timeline[k].locked["right"] for k in range(a, b + 1)):
        raise PlanRefused(f"timeline frames {a}..{b} are not all both-LOCKED")
    off = {s: np.asarray(now[s], float) - timeline[a].q[s] for s in ARMS}
    qs = [{s: timeline[k].q[s] + off[s] * (b - k) / (b - a) for s in ARMS} for k in range(a, b + 1)]
    return _frames(qs, "home (timeline 74..86)", {"left": True, "right": True}, tube, start_index=a)


def plan_home_line(timeline: Sequence[cr.Frame], now: Dict[str, np.ndarray], tube: TubeFn) -> List[cr.Frame]:
    """Step 9 (fallback): both arms on one joint line to timeline frame 86."""
    end = timeline[HOME_FRAMES[1]].q
    return _frames(joint_line(now, end), "home (joint line)", {"left": True, "right": True}, tube)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
def frames_problems(cfg, checker, frames: Sequence[cr.Frame], samples: int = 4) -> List[str]:
    """Soft / EEPROM limits and the cell mesh (per-arm pairs, tube interpolated between frames,
    as cycle_replay.preflight) plus, at every frame, ALL pairs (arm=None: the carried tube
    against the chassis too). Fails closed."""
    probs = cr.preflight(cfg, frames, checker, samples=samples, tube=True)
    if probs:
        return probs
    for f in frames:
        try:
            hits = checker.check(cr.cell_state(f.q["left"], f.q["right"], f.tube), arm=None)
        except Exception as exc:  # noqa: BLE001 - fail closed
            return [f"frame {f.index}: collision check failed: {exc}"]
        if hits:
            return [f"frame {f.index} ({f.step}): {hits[0]}"]
    return []


def fluid_problems(cfg, checker, frames: Sequence[cr.Frame]) -> Tuple[List[cr.Segment], List[str]]:
    """Segments of a streamed path + its stream preflight (every 20 ms, speed ceiling)."""
    segs = cr.build_segments(frames, cr.DEFAULT_TEMPO)
    return segs, cr.stream_preflight(cfg, segs, checker, frames=frames)
