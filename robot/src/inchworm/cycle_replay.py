"""Replay the planned AA-45 inchworm cycle on the real arms, WITHOUT the tube.

Owner request 2026-11-12: "a full movement test of the arms", the full cycle, no tube,
with the grippers moving (unlock / lock at the regrips as in the plan).

Input: the timeline the planner wrote (``out/inchworm/timeline.jsonl``, the cycle the
owner approved as a GIF). Each frame holds both arms' joint angles (rad) and whether each
gripper is locked.

Safety structure
----------------
* Offline preflight (no hardware): every frame-to-frame move is sampled with BOTH arms
  interpolated together and checked against the cell mesh model with no tube, the
  configured soft limits and the EEPROM tick limits. Any problem = refuse.
* Live: every arm goal goes through :class:`~inchworm.motion_guard.GuardedArm` (soft limits,
  keep-out primitives, mesh collision, calibration; fails closed when unverified). The
  gripper is commanded only through :class:`~inchworm.gripper_lock.GripperLockSM`; unlock
  needs the other arm LOCKED (as in the real cycle).
* Moves are slow: the largest joint step of a frame runs at ``vmax_ticks_s``; the other
  joints get proportional speeds so they arrive together. Goal_Speed is never 0 (0 = full
  speed on the STS3215).
* Abort on: stop file, Ctrl-C, timeout, a joint that stops short of its goal, a servo
  error, over-temperature. On abort the arms HOLD in place (owner decision: e-stop holds),
  then the owner supports them and releases torque.

Pure planning helpers are hardware-free and tested; :class:`ReplayRunner` takes injected
buses so the live loop is tested against simulated servos.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

ARMS = ("left", "right")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICKS_PER_RAD = 4096 / (2 * math.pi)

#: Defaults. UNVERIFIED placeholders for the first supervised run (2026-11-12).
DEFAULT_VMAX_TICKS_S = 150      # ~13 deg/s on the joint that moves most in a frame
MAX_VMAX_TICKS_S = 300
MIN_SPEED_TICKS_S = 5           # never 0: Goal_Speed 0 = full speed
ARM_ACCEL = 20
DEFAULT_ARM_TORQUE_LIMIT = 500  # of 1000; the gripper uses its lock torque limit
REACH_TOL_TICKS = 20            # ~1.8 deg
STALL_S = 1.0
GRIPPER_OPEN_TIMEOUT_S = 10.0
PREFLIGHT_SAMPLES = 20          # per frame-to-frame move, both arms together


class ReplayAborted(RuntimeError):
    """The replay stopped; arms were put on hold."""


@dataclass
class Frame:
    index: int
    step: str
    q: Dict[str, np.ndarray]          # arm -> 5 joint radians
    locked: Dict[str, bool]


def load_timeline(path: Path) -> List[Frame]:
    frames = []
    for i, line in enumerate(Path(path).read_text().splitlines()):
        if not line.strip():
            continue
        d = json.loads(line)
        q = {a: np.asarray(d[a], dtype=np.float64) for a in ARMS}
        if any(v.shape != (5,) or not np.all(np.isfinite(v)) for v in q.values()):
            raise ValueError(f"{path}: frame {i} needs 5 finite joint values per arm")
        frames.append(Frame(len(frames), d.get("step", ""), q,
                            {a: bool(d[f"{a}_locked"]) for a in ARMS}))
    if not frames:
        raise ValueError(f"{path}: empty timeline")
    if not all(frames[0].locked.values()):
        raise ValueError("the timeline must start with both grippers locked")
    for f in frames:
        if not any(f.locked.values()):
            raise ValueError(f"frame {f.index}: both grippers unlocked (never allowed)")
    return frames


def gripper_events(frames: Sequence[Frame]) -> Dict[int, List[Tuple[str, str]]]:
    """Frame index -> [(arm, 'unlock'|'lock')], executed BEFORE moving to that frame.

    Unlocks come before locks within a frame so the other arm is never asked to unlock
    while this one is still open (the state machine refuses that anyway).
    """
    ev: Dict[int, List[Tuple[str, str]]] = {}
    for prev, cur in zip(frames, frames[1:]):
        items = []
        for a in ARMS:
            if prev.locked[a] and not cur.locked[a]:
                items.append((a, "unlock"))
            elif not prev.locked[a] and cur.locked[a]:
                items.append((a, "lock"))
        if items:
            ev[cur.index] = sorted(items, key=lambda x: x[1] != "lock")  # locks first, then unlocks
    return ev


def frame_speeds(q_from: np.ndarray, q_to: np.ndarray, vmax_ticks_s: int) -> Tuple[List[int], float]:
    """Per-joint Goal_Speed (ticks/s) so all joints arrive together; and the move time (s)."""
    d = np.abs(np.asarray(q_to) - np.asarray(q_from)) * TICKS_PER_RAD
    dmax = float(d.max())
    if dmax < 1.0:
        return [MIN_SPEED_TICKS_S] * len(d), 0.0
    sp = [max(MIN_SPEED_TICKS_S, int(round(vmax_ticks_s * x / dmax))) for x in d]
    return sp, dmax / vmax_ticks_s


def q_to_ticks(cfg, arm: str, q: Sequence[float]) -> List[int]:
    cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
    return [int(round((getattr(cal, f"{j}_zero_offset_deg")
                       + math.degrees(float(v)) / getattr(cal, f"{j}_sign")) * 4096 / 360))
            for j, v in zip(JOINTS, q)]


EEPROM_INSIDE_TICKS = 10   # commanded goals stay this far inside the servo's own limits

#: Owner 2026-11-13: the elbow EEPROM Max was raised (L 3091->3151, R 3046->3106) so the arm can
#: reach the touching fold (lower arm resting on its shoulder). HOW to press into that contact
#: (force cap, verification) is still the owner's open decision, so until then the replay keeps
#: the elbow where the 2026-11-13 runs held it: at most these ticks (the old limit - 10).
FOLD_ELBOW_CAP_TICKS = {"left": 3081, "right": 3036}


def clamp_to_eeprom(cfg, frames: Sequence[Frame], inside: int = EEPROM_INSIDE_TICKS,
                    elbow_cap: Optional[Dict[str, int]] = None) -> Tuple[List[Frame], List[str]]:
    """Frames whose joint goals lie past the EEPROM tick limits, pulled to ``inside`` ticks within.

    Found 2026-11-12: the measured fold (start and end of the cycle) has the elbow at its
    swept end stop, 30 ticks past the EEPROM limit (the limits carry a 30-tick margin). The
    servo cannot be commanded there; the replay opens the elbow by that much instead. Each
    change is reported; the result is re-checked by :func:`preflight`.
    """
    out, notes = [], []
    for f in frames:
        q = {a: f.q[a].copy() for a in ARMS}
        for a in ARMS:
            cal = getattr(cfg.motion_guard.servo_calibration, f"{a}_arm")
            lim = getattr(cfg.motion_guard, f"{a}_arm_limits").servo_tick_limits or {}
            for k, (j, t) in enumerate(zip(JOINTS, q_to_ticks(cfg, a, q[a]))):
                if j not in lim:
                    continue
                lo, hi = lim[j][0] + inside, lim[j][1] - inside
                if j == "elbow_flex" and elbow_cap and a in elbow_cap:
                    hi = min(hi, int(elbow_cap[a]))
                if lo <= t <= hi:
                    continue
                tc = min(max(t, lo), hi)
                sign, z = getattr(cal, f"{j}_sign"), getattr(cal, f"{j}_zero_offset_deg")
                q[a][k] = sign * math.radians(tc * 360 / 4096 - z)
                notes.append(f"frame {f.index} {a} {j}: tick {t} -> {tc} "
                             f"({math.degrees(q[a][k] - f.q[a][k]):+.1f} deg, EEPROM {lim[j]}"
                             + (f", fold cap {elbow_cap[a]}" if j == "elbow_flex" and elbow_cap and a in elbow_cap else "")
                             + ")")
        out.append(Frame(f.index, f.step, q, dict(f.locked)))
    return out, notes


def preflight(cfg, frames: Sequence[Frame], checker, samples: int = PREFLIGHT_SAMPLES) -> List[str]:
    """Offline check of the whole replay with no tube in the cell. Returns problems (empty = OK)."""
    from inchworm.inchworm_plan import configured_soft_limits
    from inchworm.system_model.collision import CellState

    problems: List[str] = []
    soft = configured_soft_limits(cfg)
    if soft is None:
        problems.append("soft limits not configured")
    for f in frames:
        for a in ARMS:
            if soft is not None:
                lo, hi = soft[a]
                for j, v, l, h in zip(JOINTS, f.q[a], lo, hi):
                    if not (l - 1e-9 <= v <= h + 1e-9):
                        problems.append(f"frame {f.index} {a} {j} {math.degrees(v):.1f} deg outside soft limits")
            lim = getattr(cfg.motion_guard, f"{a}_arm_limits").servo_tick_limits or {}
            for j, t in zip(JOINTS, q_to_ticks(cfg, a, f.q[a])):
                if j in lim and not (lim[j][0] <= t <= lim[j][1]):
                    problems.append(f"frame {f.index} {a} {j} tick {t} outside EEPROM {lim[j]}")
    if checker is None:
        problems.append("mesh collision model unavailable")
        return problems
    for prev, cur in zip(frames, frames[1:]):
        for k in range(1, samples + 1):
            t = k / samples
            ql = (1 - t) * prev.q["left"] + t * cur.q["left"]
            qr = (1 - t) * prev.q["right"] + t * cur.q["right"]
            st = CellState(left_q=ql, right_q=qr, dexarm_q={}, tube_advance_m=None)
            try:
                hits = [c for a in ARMS for c in checker.check(st, arm=a)]
            except Exception as exc:  # noqa: BLE001 - fail closed
                problems.append(f"move {prev.index}->{cur.index}: collision check failed: {exc}")
                break
            if hits:
                problems.append(f"move {prev.index}->{cur.index} (t={t:.2f}): {hits[0]}")
                break
    return problems


#: Owner request 2026-11-13: when the hand-placed arms are CLOSE to the start pose, the
#: robot makes the final adjustment itself (guard- and collision-checked) instead of the
#: owner placing them exactly. Further than this on any joint = refuse (not "close").
#: UNVERIFIED placeholder; the measured fold is 3.5 deg off on the elbows.
APPROACH_MAX_DEG = 10.0


def approach_problems(cfg, measured: Dict[str, Sequence[float]], frame0: Frame, checker,
                      max_deg: float = APPROACH_MAX_DEG, samples: int = PREFLIGHT_SAMPLES) -> List[str]:
    """Check the automatic final adjustment from the measured pose to frame 0.

    Refuses when any joint is more than ``max_deg`` away, or when the straight-line move
    (both arms together, no tube) touches anything in the cell model. Fails closed.
    """
    from inchworm.system_model.collision import CellState

    problems: List[str] = []
    q0 = {}
    for a in ARMS:
        m = measured.get(a)
        if m is None or len(m) != 5 or not all(math.isfinite(float(v)) for v in m):
            problems.append(f"{a}: no valid measured pose")
            continue
        q0[a] = np.asarray(m, dtype=np.float64)
        for j, d in zip(JOINTS, np.degrees(frame0.q[a] - q0[a])):
            if abs(d) > max_deg:
                problems.append(f"{a} {j}: {d:+.1f} deg to the start pose (more than {max_deg:.0f}; "
                                f"place the arm closer by hand)")
    if problems:
        return problems
    if checker is None:
        return ["mesh collision model unavailable"]
    for k in range(samples + 1):
        t = k / samples
        ql = (1 - t) * q0["left"] + t * frame0.q["left"]
        qr = (1 - t) * q0["right"] + t * frame0.q["right"]
        try:
            hits = [c for a in ARMS for c in checker.check(
                CellState(left_q=ql, right_q=qr, dexarm_q={}, tube_advance_m=None), arm=a)]
        except Exception as exc:  # noqa: BLE001 - fail closed
            return [f"adjustment move: collision check failed: {exc}"]
        if hits:
            return [f"adjustment move (t={t:.2f}): {hits[0]}"]
    return []


def summary(frames: Sequence[Frame], vmax_ticks_s: int) -> str:
    ev = gripper_events(frames)
    total = 0.0
    lines = []
    phase = None
    for prev, cur in zip(frames, frames[1:]):
        _, tl = frame_speeds(prev.q["left"], cur.q["left"], vmax_ticks_s)
        _, tr = frame_speeds(prev.q["right"], cur.q["right"], vmax_ticks_s)
        total += max(tl, tr)
        if cur.step != phase:
            phase = cur.step
            lines.append(f"  frame {cur.index:3d}: {phase}")
        for a, what in ev.get(cur.index, []):
            lines.append(f"  frame {cur.index:3d}:   {a} gripper {what.upper()}")
    head = (f"{len(frames)} frames, arm motion ~{total:.0f} s at {vmax_ticks_s} ticks/s "
            f"(plus ~2 s of guard checks per frame and the gripper moves)")
    return head + "\n" + "\n".join(lines)


# ---------------------------------------------------------------------------
# Fluid motion (owner 2026-11-13): both arms together, no stop between frames
# ---------------------------------------------------------------------------

#: The approved cycle GIF plays one timeline frame per 0.2 s (scripts/inchworm_animation.py,
#: PillowWriter fps=5). ``tempo`` = fraction of that speed (owner: start at 0.5).
ANIM_FRAME_S = 0.2
DEFAULT_TEMPO = 0.5
MAX_TEMPO = 1.0
MAX_STREAM_SPEED_TICKS_S = 700   # ceiling on any joint's planned speed (~62 deg/s); above = refuse
STREAM_PERIOD_S = 0.02          # a new goal for both arms every 20 ms (50 Hz), from the path clock
STREAM_LEAD_S = 0.06            # each goal is the path point this far ahead, sent with the path's
                                # speed, so the servo is always travelling and never stops at a point
STREAM_ACCEL = 100              # servo ramp (x100 ticks/s^2): smooths the speed changes between goals
STREAM_SPEED_CAP_TICKS_S = 1000 # Goal_Speed ceiling while streaming
SETTLE_S = 1.0                  # at the end of a piece: wait at most this long for the last frame
PREFLIGHT_STREAM_DT_S = 0.02    # offline check of the streamed path at the stream rate (every 20 ms)


@dataclass
class Segment:
    """Fluid motion between two gripper actions (the arms stop only at the gripper actions)."""
    first: int                       # timeline frame index at the start
    last: int                        # ... and at the end
    before: List[Tuple[str, str]]    # gripper actions done (arms still) before this segment
    duration_s: float
    path: Dict[str, Callable[[float], np.ndarray]]   # arm -> q(t), t in [0, duration_s]

    def q(self, t: float) -> Dict[str, np.ndarray]:
        t = min(max(float(t), 0.0), self.duration_s)
        return {a: self.path[a](t) for a in ARMS}


def _time_to_frames(n: int, dt: float) -> Callable[[float], float]:
    """Timeline position s (frames) at time t: one frame per ``dt``, easing in and out over the
    first and last frame (constant acceleration) so each segment starts and ends at rest."""
    V, ta, T = 1.0 / dt, dt, (n + 1) * dt

    def s(t: float) -> float:
        t = min(max(t, 0.0), T)
        if t < ta:
            return 0.5 * V * t * t / ta
        if t > T - ta:
            r = T - t
            return n - 0.5 * V * r * r / ta
        return 0.5 * V * ta + V * (t - ta)
    return s


def build_segments(frames: Sequence[Frame], tempo: float = DEFAULT_TEMPO) -> List[Segment]:
    """Split the timeline at the gripper actions; each piece becomes one smooth (C1, no
    overshoot: monotone cubic per joint) path through its frames, both arms on one clock."""
    from scipy.interpolate import PchipInterpolator
    if not (0.0 < tempo <= MAX_TEMPO):
        raise ValueError(f"tempo must be in (0, {MAX_TEMPO}]")
    dt = ANIM_FRAME_S / tempo
    ev = gripper_events(frames)
    # A gripper action at frame c happens with the arms still at frame c-1 (as in run()).
    starts = [0] + [c - 1 for c in sorted(ev)]
    ends = starts[1:] + [len(frames) - 1]
    segs: List[Segment] = []
    for i, (a0, a1) in enumerate(zip(starts, ends)):
        before = ev.get(a0 + 1, []) if i else []
        n = a1 - a0
        if n <= 0 or all(np.allclose(frames[a0].q[a], frames[k].q[a]) for k in range(a0, a1 + 1) for a in ARMS):
            segs.append(Segment(a0, a1, before, 0.0, {a: (lambda t, q=frames[a1].q[a].copy(): q) for a in ARMS}))
            continue
        s_of_t = _time_to_frames(n, dt)
        paths = {}
        for a in ARMS:
            Q = np.stack([frames[k].q[a] for k in range(a0, a1 + 1)])
            f = PchipInterpolator(np.arange(n + 1, dtype=float), Q, axis=0) if n >= 2 else \
                (lambda x, Q=Q: Q[0] + (Q[1] - Q[0]) * x)
            paths[a] = (lambda t, f=f, s_of_t=s_of_t: np.asarray(f(s_of_t(t)), dtype=np.float64))
        segs.append(Segment(a0, a1, before, (n + 1) * dt, paths))
    return segs


def stream_samples(seg: Segment, dt: float = PREFLIGHT_STREAM_DT_S) -> np.ndarray:
    if seg.duration_s <= 0:
        return np.array([0.0])
    k = max(2, int(math.ceil(seg.duration_s / dt)) + 1)
    return np.linspace(0.0, seg.duration_s, k)


def peak_speed_ticks_s(segs: Sequence[Segment]) -> float:
    peak = 0.0
    for seg in segs:
        ts = stream_samples(seg)
        if len(ts) < 2:
            continue
        for a in ARMS:
            Q = np.stack([seg.path[a](t) for t in ts])
            v = np.abs(np.diff(Q, axis=0)).max(1) * TICKS_PER_RAD / np.diff(ts)
            peak = max(peak, float(v.max()))
    return peak


def stream_preflight(cfg, segs: Sequence[Segment], checker) -> List[str]:
    """Owner option (b), 2026-11-13: the WHOLE fluid path, both arms together, no tube, every
    25 ms of path time: soft limits, EEPROM tick limits, the cell mesh model, and the speed
    ceiling. Refuse on any problem. (Live, each streamed point also gets the full pose check.)"""
    from inchworm.inchworm_plan import configured_soft_limits
    from inchworm.system_model.collision import CellState

    problems: List[str] = []
    soft = configured_soft_limits(cfg)
    if soft is None:
        return ["soft limits not configured"]
    if checker is None:
        return ["mesh collision model unavailable"]
    peak = peak_speed_ticks_s(segs)
    if peak > MAX_STREAM_SPEED_TICKS_S:
        problems.append(f"planned joint speed {peak:.0f} ticks/s above the {MAX_STREAM_SPEED_TICKS_S} ceiling")
    for seg in segs:
        for t in stream_samples(seg):
            q = seg.q(t)
            where = f"frames {seg.first}->{seg.last} t={t:.2f}s"
            bad = None
            for a in ARMS:
                lo, hi = soft[a]
                for j, v, l, h in zip(JOINTS, q[a], lo, hi):
                    if not (l - 1e-9 <= v <= h + 1e-9):
                        bad = f"{where}: {a} {j} {math.degrees(v):.1f} deg outside soft limits"
                lim = getattr(cfg.motion_guard, f"{a}_arm_limits").servo_tick_limits or {}
                for j, tk in zip(JOINTS, q_to_ticks(cfg, a, q[a])):
                    if j in lim and not (lim[j][0] <= tk <= lim[j][1]):
                        bad = f"{where}: {a} {j} tick {tk} outside EEPROM {lim[j]}"
            if bad is None:
                try:
                    hits = checker.check(CellState(left_q=q["left"], right_q=q["right"], dexarm_q={},
                                                   tube_advance_m=None))
                except Exception as exc:  # noqa: BLE001 - fail closed
                    hits = [f"collision check failed: {exc}"]
                if hits:
                    bad = f"{where}: {hits[0]}"
            if bad:
                problems.append(bad)
                break
    return problems


# ---------------------------------------------------------------------------
# Live runner (injected buses; tested against simulated servos)
# ---------------------------------------------------------------------------

@dataclass
class ArmHandles:
    bus: object            # ArmServoBus
    guarded: object        # GuardedArm (live)
    sm: object             # GripperLockSM
    ids: Dict[str, int]    # joint -> servo id (1-5)


@dataclass
class ReplayRunner:
    cfg: object
    frames: Sequence[Frame]
    arms: Dict[str, ArmHandles]
    vmax_ticks_s: int = DEFAULT_VMAX_TICKS_S
    arm_torque_limit: int = DEFAULT_ARM_TORQUE_LIMIT
    should_abort: Callable[[], bool] = lambda: False
    log: Callable[[dict], None] = lambda rec: None
    sleep: Callable[[float], None] = time.sleep
    clock: Callable[[], float] = time.monotonic
    targets: Dict[str, np.ndarray] = field(default_factory=dict)
    #: OWNER DECISION 2026-11-13 ("aborting like that is more dangerous than whatever it is
    #: trying to prevent"; "I don't need safety checks once we have confirmed a motion"): when
    #: False, temperature/current/stall/lag readings are LOGGED and never stop the arms; only
    #: the owner (Ctrl-C / stop file) or a gripper state-machine failure stops the run. True
    #: (the step-by-step mode) keeps the old automatic aborts.
    abort_on_faults: bool = True

    def __post_init__(self):
        for h in self.arms.values():
            mesh = h.guarded.guard.mesh
            if mesh is None:
                raise ValueError("mesh collision model not configured")
            mesh.tube_source = lambda: None          # stated explicitly: NO tube in the cell
            mesh.dexarm_pose_source = lambda: {}     # DexArm at its model pose (owner checks)

    # -- helpers ----------------------------------------------------------
    def _ticks(self, arm: str) -> List[int]:
        b = self.arms[arm].bus
        return [round(b.read_position_deg(self.arms[arm].ids[j]) * 4096 / 360) for j in JOINTS]

    def _check_abort(self):
        if self.should_abort():
            raise ReplayAborted("stop requested")

    def _wait_arrival(self, goals: Dict[str, List[int]], expected_s: float, frame: int):
        t0 = self.clock()
        last = {a: None for a in goals}
        last_move = t0
        peak_load = {a: [0] * 5 for a in goals}
        timeout = 3.0 * expected_s + 3.0
        while True:
            self._check_abort()
            now = self.clock()
            done = True
            moved = False
            cur = {}
            for a, g in goals.items():
                ids = self.arms[a].ids
                ticks = []
                for k, j in enumerate(JOINTS):
                    try:
                        st = self._read_state(a, ids[j])
                    except Exception:  # noqa: BLE001
                        if self.abort_on_faults:
                            raise
                        ticks.append(last[a][k] if last[a] else g[k])
                        continue
                    ticks.append(round(st.position_deg * 4096 / 360))
                    peak_load[a][k] = max(peak_load[a][k], abs(st.load_raw))
                    self._fault(a, j, st)
                cur[a] = ticks
                if any(abs(t - gg) > REACH_TOL_TICKS for t, gg in zip(ticks, g)):
                    done = False
                if last[a] is None or any(abs(t - l) > 1 for t, l in zip(ticks, last[a])):
                    moved = True
                last[a] = ticks
            if done:
                return cur, peak_load, now - t0
            if moved:
                last_move = now
            elif not self.abort_on_faults and now - t0 > SETTLE_S:
                return cur, peak_load, now - t0       # logged by the caller; never stops the run
            elif now - last_move > STALL_S:
                raise ReplayAborted(f"frame {frame}: stopped short of the goal: "
                                    + "; ".join(f"{a} err {[c - g for c, g in zip(cur[a], goals[a])]}"
                                                for a in goals))
            if now - t0 > timeout:
                if not self.abort_on_faults:
                    return cur, peak_load, now - t0
                raise ReplayAborted(f"frame {frame}: timeout after {now - t0:.1f} s")
            self.sleep(0.02)

    def _wait_gripper_open(self, arm: str):
        sm, b = self.arms[arm].sm, self.arms[arm].bus
        band = getattr(self.cfg.gripper_lock, f"arm_{arm}")
        t0 = self.clock()
        while True:
            self._check_abort()
            deg = b.read_position_deg(6)
            if band.unlocked_low_deg <= deg <= band.unlocked_high_deg:
                return deg
            if self.clock() - t0 > GRIPPER_OPEN_TIMEOUT_S:
                raise ReplayAborted(f"{arm} gripper did not reach the open band (at {deg:.1f} deg)")
            self.sleep(0.02)

    def _lock(self, arm: str):
        sm = self.arms[arm].sm
        from inchworm.gripper_lock import GripperState
        if sm.state != GripperState.UNLOCKED:
            sm.reset()
            self._wait_gripper_open(arm)
        sm.set_approach_ready()
        sm.lock()            # raises on failure (FAULT, torque off)
        self.log({"event": "gripper_lock", "arm": arm, "state": sm.state.name,
                  "ticks": round(self.arms[arm].bus.read_position_deg(6) * 4096 / 360)})

    def _unlock(self, arm: str):
        sm = self.arms[arm].sm
        sm.unlock()          # refused unless the other arm is LOCKED
        deg = self._wait_gripper_open(arm)
        self.log({"event": "gripper_unlock", "arm": arm, "state": sm.state.name,
                  "ticks": round(deg * 4096 / 360)})

    # -- phases -----------------------------------------------------------
    def energise(self):
        """Arm joints: slow profile, torque limit, torque ON (goal := present). Grippers: slow drive."""
        for a, h in self.arms.items():
            for j in JOINTS:
                sid = h.ids[j]
                h.bus.set_goal_speed(sid, MIN_SPEED_TICKS_S)
                h.bus.set_acceleration(sid, ARM_ACCEL)
                h.bus.set_torque_limit(sid, self.arm_torque_limit)
                h.bus.set_torque_enable(sid, True)
            h.sm.energise_slow()
            self.targets[a] = None

    def lock_both(self):
        for a in ARMS:
            self._lock(a)

    def present_q(self) -> Dict[str, np.ndarray]:
        from inchworm.so101_pose_reader import ticks_to_q
        return {a: np.asarray(ticks_to_q(self.cfg, a, self._ticks(a))) for a in ARMS}

    def move_to_start(self):
        """Final adjustment from the measured pose to frame 0 (after :func:`approach_problems`)."""
        start = Frame(-1, "measured start", self.present_q(), dict(self.frames[0].locked))
        self._move(start, self.frames[0])

    def run(self):
        """Step-by-step replay (each frame reached before the next is sent). No prompts."""
        ev = gripper_events(self.frames)
        for prev, cur in zip(self.frames, self.frames[1:]):
            self._check_abort()
            for a, what in ev.get(cur.index, []):
                (self._lock if what == "lock" else self._unlock)(a)
            self._move(prev, cur)

    def _move(self, prev: Frame, cur: Frame):
        """One frame-to-frame move of both arms: guard-checked writes, then wait for arrival."""
        goals, exp = {}, 0.0
        for a in ARMS:
            if np.allclose(prev.q[a], cur.q[a]):
                continue
            sp, t = frame_speeds(prev.q[a], cur.q[a], self.vmax_ticks_s)
            exp = max(exp, t)
            goals[a] = (sp, cur.q[a])
        if not goals:
            return
        # Guard checks (each arm against the other arm's TARGET) before anything moves.
        others = {a: cur.q[a] for a in ARMS}
        tick_goals = {}
        for a, (sp, q) in goals.items():
            h = self.arms[a]
            h.guarded.guard.mesh.other_arm_pose_source = (lambda o=others["right" if a == "left" else "left"]: o)
            for j, s in zip(JOINTS, sp):
                h.bus.set_goal_speed(h.ids[j], s)
            tick_goals[a] = q_to_ticks(self.cfg, a, q)
        for a, (sp, q) in goals.items():
            self.arms[a].guarded.write_positions_rad(q)
        final, peak, dt = self._wait_arrival(tick_goals, exp, cur.index)
        for a in ARMS:
            from inchworm.gripper_lock import GripperState
            if self.arms[a].sm.state == GripperState.LOCKED:
                self.arms[a].sm.hold_tick()
        self.log({"event": "frame", "frame": cur.index, "step": cur.step,
                  "goal_ticks": tick_goals, "final_ticks": final,
                  "error_ticks": {a: [f - g for f, g in zip(final[a], tick_goals[a])] for a in final},
                  "peak_abs_load": peak, "expected_s": round(exp, 2), "took_s": round(dt, 2)})

    def _read_state(self, arm: str, sid: int):
        """One joint-state read; a single garbled reply (seen once live 2026-11-13: checksum
        error on servo 2) is retried once and logged. A second failure aborts."""
        from inchworm.servo_bus import ServoError
        try:
            return self.arms[arm].bus.read_joint_state(sid)
        except ServoError as exc:
            self.log({"event": "read_retry", "arm": arm, "servo": sid, "error": str(exc)})
            return self.arms[arm].bus.read_joint_state(sid)

    def _fault(self, arm: str, joint: str, st) -> None:
        """Over-temperature / overcurrent reading: abort (strict mode) or log it (fluid)."""
        msg = None
        if st.temperature_c >= self.cfg.fault_thresholds.overtemperature_c:
            msg = f"{arm} {joint} at {st.temperature_c} C"
        elif st.current_raw >= self.cfg.fault_thresholds.overcurrent_raw:
            msg = f"{arm} {joint} current {st.current_raw} raw (overcurrent)"
        if msg is None:
            return
        if self.abort_on_faults:
            raise ReplayAborted(msg)
        self.log({"event": "reading_warning", "arm": arm, "joint": joint, "what": msg})

    def run_fluid(self, segs: Sequence[Segment]):
        """Owner 2026-11-13: fluid motion, both arms at the same time, no stop between frames.
        The arms come to rest only for the gripper actions between segments."""
        from inchworm.gripper_lock import GripperState
        for seg in segs:
            for a, what in seg.before:
                (self._lock if what == "lock" else self._unlock)(a)
            if seg.duration_s <= 0:
                continue
            self._stream_segment(seg)
            for a in ARMS:
                if self.arms[a].sm.state == GripperState.LOCKED:
                    self.arms[a].sm.hold_tick()

    def _stream_segment(self, seg: Segment):
        """Stream the confirmed path: both arms get the path point for the CURRENT time every
        20 ms (one sync-write packet per arm). No per-point checks (the whole path was checked
        before the owner's YES). Read-back is for the log only and never stops the motion."""
        from inchworm.servo_bus import ServoError
        for h in self.arms.values():
            for j in JOINTS:
                h.bus.set_acceleration(h.ids[j], STREAM_ACCEL)
        lag = {a: 0 for a in ARMS}
        hot = {a: 0 for a in ARMS}
        peak_load = {a: [0] * 5 for a in ARMS}
        read_errors, n = 0, 0
        t0 = self.clock()
        while True:
            if self.should_abort():
                raise ReplayAborted("stop requested")
            now = self.clock()
            t = now - t0
            q = seg.q(t)
            ahead = seg.q(t + STREAM_LEAD_S)
            for a in ARMS:
                now_t = q_to_ticks(self.cfg, a, q[a])
                goal = q_to_ticks(self.cfg, a, ahead[a])
                sp = [int(min(STREAM_SPEED_CAP_TICKS_S, max(MIN_SPEED_TICKS_S, abs(g - c) / STREAM_LEAD_S)))
                      for g, c in zip(goal, now_t)]
                self.arms[a].guarded.write_preflighted_ticks(goal, sp)
            if t >= seg.duration_s:
                break
            a = ARMS[n % 2]                           # read back one arm per tick (log only)
            try:
                want = q_to_ticks(self.cfg, a, q[a])
                for k, j in enumerate(JOINTS):
                    st = self.arms[a].bus.read_joint_state(self.arms[a].ids[j])
                    lag[a] = max(lag[a], abs(round(st.position_deg * 4096 / 360) - want[k]))
                    peak_load[a][k] = max(peak_load[a][k], abs(st.load_raw))
                    hot[a] = max(hot[a], st.temperature_c)
            except ServoError:
                read_errors += 1
            n += 1
            if n % 25 == 0:
                self.log({"event": "stream", "t": round(t, 2), "frames": [seg.first, seg.last]})
            rest = STREAM_PERIOD_S - (self.clock() - now)
            if rest > 0:
                self.sleep(rest)
        end = self.frames[seg.last]
        goals = {a: q_to_ticks(self.cfg, a, end.q[a]) for a in ARMS}
        strict, self.abort_on_faults = self.abort_on_faults, False
        try:
            final, peak2, _ = self._wait_arrival(goals, SETTLE_S, seg.last)
        finally:
            self.abort_on_faults = strict
        self.log({"event": "segment", "frames": [seg.first, seg.last], "step": end.step,
                  "planned_s": round(seg.duration_s, 2), "took_s": round(self.clock() - t0, 2),
                  "loops": n, "worst_path_error_ticks": lag, "max_temp_read_c": hot,
                  "read_errors": read_errors,
                  "final_error_ticks": {a: [f - g for f, g in zip(final[a], goals[a])] for a in ARMS},
                  "peak_abs_load": {a: [max(x, y) for x, y in zip(peak_load[a], peak2[a])] for a in ARMS}})

    def hold(self):
        """Owner decision: e-stop HOLDS. Re-command present positions with torque on."""
        from inchworm.torque import TorqueManager
        return TorqueManager({f"arm_{a}": h.bus for a, h in self.arms.items()}).hold_all()

    def release(self):
        """Torque OFF on all arm joints and grippers (the owner supports the arms first)."""
        from inchworm.torque import TorqueManager
        errs = TorqueManager({f"arm_{a}": h.bus for a, h in self.arms.items()}).disable_all()
        for a, h in self.arms.items():
            try:
                h.sm.release(tube_attached=False)
            except Exception as exc:  # noqa: BLE001
                errs[f"gripper_{a}"] = exc
        return errs


def build_runner(cfg, frames, buses: Dict[str, object], *, live: bool,
                 soft_limits_override: Optional[str] = None, **kw) -> ReplayRunner:
    """Wire GuardedArm + GripperLockSM around the given buses (real or simulated)."""
    from inchworm.gripper_lock import GripperLockSM, GripperState
    from inchworm.motion_guard import build_guarded_arm

    arms: Dict[str, ArmHandles] = {}
    sms: Dict[str, object] = {}
    for a in ARMS:
        g = build_guarded_arm(buses[a], cfg, a, live=live,
                              unverified_soft_limits_override=soft_limits_override)
        sm = GripperLockSM(bus=buses[a], role=f"arm_{a}", config=cfg.gripper_lock, dry_run=False)
        sms[a] = sm
        arms[a] = ArmHandles(buses[a], g, sm, {j: i + 1 for i, j in enumerate(JOINTS)})
    sms["left"]._opposite_arm_locked_fn = lambda: sms["right"].state == GripperState.LOCKED
    sms["right"]._opposite_arm_locked_fn = lambda: sms["left"].state == GripperState.LOCKED
    clock, sleep = kw.get("clock"), kw.get("sleep")
    if clock is not None:
        for sm in sms.values():
            sm._clock, sm._sleep = clock, sleep
    return ReplayRunner(cfg, frames, arms, **kw)
