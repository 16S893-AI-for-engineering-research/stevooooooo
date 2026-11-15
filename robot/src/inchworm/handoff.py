"""AA-23 / AA-44: demo start with the tube: the cycle replay hands the left arm to teleop.

Owner decisions 2026-11-15 (AA-23 / AA-44 notes, doc-3):

* Route A: the real cycle (``scripts/replay_cycle.py --stop-after-frame 42``) carries the
  tube to the left-regrip handoff and STOPS. Both arms keep holding after the script exits
  (torque on); the right gripper stays LOCKED on the tube, the left gripper is open, 50 mm
  under its next attachment point. The replay writes :data:`STATE_PATH`.
* The LeRobot follower in ``tube_support=handoff`` mode takes over the powered left arm
  WITHOUT cutting torque. It refuses unless the state file matches what the servos read:
  both arms torque ON at the stop pose, the other arm's gripper LOCKED, the follower's gripper
  not locked.
* Catch-up replaces the 10 deg start gate on this path: the follower moves slowly (default
  5 deg/s per joint) from the stop pose to where the leader is, then follows normally.
* The mesh collision check is ON on this path, with the tube modelled where the cycle left it
  and the other arm at its holding pose. A goal that touches anything is not sent; the arm
  holds its last goal until the leader asks for a clear pose again.
* End of session: support the tube and both arms, then joints 1-5 of BOTH arms torque OFF.
  The grippers are never touched (the right one stays LOCKED). Re-locking / finishing the
  cycle is a separate task.

Nothing here opens a port; callers inject read functions (real or simulated).
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

ROBOT_ROOT = Path(__file__).resolve().parents[2]
STATE_PATH = ROBOT_ROOT / "out" / "handoff_state.json"
ARMS = ("left", "right")
J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
REG_TORQUE_ENABLE = 0x28
REG_PRESENT_POSITION = 56
DEG_PER_TICK = 360.0 / 4096
#: Owner proposal accepted 2026-11-15: catch-up speed ~5 deg/s. Max allowed by flag.
DEFAULT_CATCHUP_DEG_S = 5.0
MAX_CATCHUP_DEG_S = 10.0
#: The servos must read within this of the stop pose when teleop takes over.
#: UNVERIFIED placeholder (agent 2026-11-15): the replay's arrival tolerance is 20 ticks
#: (~1.8 deg); holding under the tube's weight is unmeasured.
POSE_TOL_TICKS = 34                 # ~3 deg
#: Joint motion between two mesh-check samples of one goal step.
CHECK_STEP_DEG = 2.0
SCHEMA = "inchworm.handoff/1"


class HandoffRefused(RuntimeError):
    pass


def write_state(path: Path, *, stop_frame: int, step: str, timeline_digest: str, tube_in_grippers: bool,
                goal_ticks: Dict[str, List[int]], present_ticks: Dict[str, List[int]],
                gripper_ticks: Dict[str, int], locked: Dict[str, bool], q_rad: Dict[str, Sequence[float]],
                tube_pose: Optional[np.ndarray]) -> dict:
    rec = {"schema": SCHEMA, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "stop_frame": int(stop_frame), "step": step, "timeline_sha256": timeline_digest,
           "tube_in_grippers": bool(tube_in_grippers),
           "goal_ticks": {a: [int(t) for t in goal_ticks[a]] for a in ARMS},
           "present_ticks": {a: [int(t) for t in present_ticks[a]] for a in ARMS},
           "gripper_ticks": {a: int(gripper_ticks[a]) for a in ARMS},
           "locked": {a: bool(locked[a]) for a in ARMS},
           "q_rad": {a: [float(v) for v in q_rad[a]] for a in ARMS},
           "tube_pose": None if tube_pose is None else np.asarray(tube_pose, dtype=float).tolist()}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rec, indent=1))
    return rec


def load_state(path: Path = STATE_PATH) -> dict:
    path = Path(path)
    if not path.exists():
        raise HandoffRefused(f"no handoff state {path}: run scripts/replay_cycle.py --stop-after-frame N first")
    st = json.loads(path.read_text())
    if st.get("schema") != SCHEMA:
        raise HandoffRefused(f"{path.name}: unknown schema {st.get('schema')!r}")
    for a in ARMS:
        if len(st["goal_ticks"][a]) != 5 or len(st["q_rad"][a]) != 5:
            raise HandoffRefused(f"{path.name}: {a} needs 5 joints")
    if st.get("tube_pose") is None:
        raise HandoffRefused(f"{path.name}: no tube pose (the mesh check needs the tube where the cycle left it)")
    T = np.asarray(st["tube_pose"], dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise HandoffRefused(f"{path.name}: tube pose is not a finite 4x4")
    return st


def _u16(b: bytes) -> int:
    return b[0] | (b[1] << 8)


def arm_problems(arm: str, read_bytes, state: dict, *, tol_ticks: int = POSE_TOL_TICKS) -> List[str]:
    """Read-only: joints 1-5 torque ON and within ``tol_ticks`` of the state's goal ticks."""
    out = []
    off = [s for s in range(1, 6) if not read_bytes(s, REG_TORQUE_ENABLE, 1)[0]]
    if off:
        out.append(f"{arm} joints {off} torque OFF (the replay must leave both arms holding)")
    for k, j in enumerate(J5):
        t = _u16(read_bytes(k + 1, REG_PRESENT_POSITION, 2))
        g = state["goal_ticks"][arm][k]
        if abs(t - g) > tol_ticks:
            out.append(f"{arm} {j} reads {t} ticks, the stop pose is {g} "
                       f"({(t - g) * DEG_PER_TICK:+.1f} deg; max {tol_ticks * DEG_PER_TICK:.1f})")
    return out


def state_problems(state: dict, follower: str) -> List[str]:
    """The stop frame must leave the follower's gripper open and the other one LOCKED."""
    other = "right" if follower == "left" else "left"
    out = []
    if state["locked"][follower]:
        out.append(f"stop frame {state['stop_frame']}: the {follower} gripper is LOCKED on the tube there "
                   "(teleop of a locked arm would pull on the tube)")
    if not state["locked"][other]:
        out.append(f"stop frame {state['stop_frame']}: the {other} gripper is not locked there")
    return out


def catch_up_target(last_goal: Sequence[float], target: Sequence[float], max_step_ticks: float) -> tuple:
    """One rate-limited step from ``last_goal`` toward ``target`` (ticks). Returns (goal, done);
    done = every joint reached the target in this step (then normal following takes over)."""
    goal = [float(g) + max(-max_step_ticks, min(max_step_ticks, float(t) - float(g)))
            for g, t in zip(last_goal, target)]
    return goal, all(abs(float(t) - x) < 1e-6 for t, x in zip(target, goal))


class CollisionGate:
    """Mesh check of the follower's goal steps against the cell model: the other arm at its
    holding pose and the tube at the pose where the cycle left it. Fails closed."""

    def __init__(self, cfg, follower: str, other_q: Sequence[float], tube_pose: np.ndarray, checker=None):
        from inchworm.so101_pose_reader import ticks_to_q  # noqa: F401  (calibration check)
        self.cfg, self.follower = cfg, follower
        self.other_q = np.asarray(other_q, dtype=float)
        self.tube = np.asarray(tube_pose, dtype=float)
        if checker is None:
            from inchworm.motion_guard import build_mesh_collision_guard
            mesh = build_mesh_collision_guard(cfg, follower)
            checker = None if mesh is None else mesh.checker
            if mesh is not None and checker is None:
                raise HandoffRefused(f"mesh collision model failed to load: {mesh.load_error}")
        if checker is None:
            raise HandoffRefused("mesh collision model not configured (motion_guard.collision.mesh)")
        self.checker = checker
        self.checks = 0
        self.ms_total = 0.0

    def q(self, ticks: Sequence[float]) -> np.ndarray:
        cal = getattr(self.cfg.motion_guard.servo_calibration, f"{self.follower}_arm")
        return np.array([getattr(cal, f"{j}_sign") * math.radians(float(t) * DEG_PER_TICK
                                                                  - getattr(cal, f"{j}_zero_offset_deg"))
                         for j, t in zip(J5, ticks)])

    def _state(self, q):
        from inchworm.system_model.collision import CellState
        ql, qr = (q, self.other_q) if self.follower == "left" else (self.other_q, q)
        return CellState(left_q=ql, right_q=qr, dexarm_q={}, tube_advance_m=0.0, tube_transform=self.tube)

    def pose_problem(self, ticks: Sequence[float]) -> Optional[str]:
        t0 = time.perf_counter()
        try:
            hits = self.checker.check(self._state(self.q(ticks)), arm=self.follower)
        except Exception as exc:  # noqa: BLE001 - fail closed
            return f"collision check failed (fail closed): {exc}"
        finally:
            self.checks += 1
            self.ms_total += (time.perf_counter() - t0) * 1000
        return f"mesh collision: {hits[0]}" if hits else None

    def step_problem(self, from_ticks: Optional[Sequence[float]], to_ticks: Sequence[float]) -> Optional[str]:
        """The goal and, for a long step, straight-line samples every CHECK_STEP_DEG on the way."""
        if from_ticks is None:
            return self.pose_problem(to_ticks)
        a, b = np.asarray(from_ticks, dtype=float), np.asarray(to_ticks, dtype=float)
        n = max(1, int(math.ceil(float(np.abs(b - a).max()) * DEG_PER_TICK / CHECK_STEP_DEG)))
        for k in range(1, n + 1):
            p = self.pose_problem(a + (b - a) * k / n)
            if p is not None:
                return p
        return None


def joints_off(read_bytes, write_bytes) -> List[int]:
    """Torque OFF on joints 1-5 that are on. The gripper (6) is never touched."""
    off = []
    for s in range(1, 6):
        if read_bytes(s, REG_TORQUE_ENABLE, 1)[0]:
            write_bytes(s, REG_TORQUE_ENABLE, b"\x00")
            off.append(s)
    return off
