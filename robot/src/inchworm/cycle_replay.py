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


def clamp_to_eeprom(cfg, frames: Sequence[Frame], inside: int = EEPROM_INSIDE_TICKS) -> Tuple[List[Frame], List[str]]:
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
                if lo <= t <= hi:
                    continue
                tc = min(max(t, lo), hi)
                sign, z = getattr(cal, f"{j}_sign"), getattr(cal, f"{j}_zero_offset_deg")
                q[a][k] = sign * math.radians(tc * 360 / 4096 - z)
                notes.append(f"frame {f.index} {a} {j}: tick {t} -> {tc} "
                             f"({math.degrees(q[a][k] - f.q[a][k]):+.1f} deg, EEPROM {lim[j]})")
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
    prompt: Callable[[str], str] = input
    pause_each_phase: bool = False
    log: Callable[[dict], None] = lambda rec: None
    sleep: Callable[[float], None] = time.sleep
    clock: Callable[[], float] = time.monotonic
    targets: Dict[str, np.ndarray] = field(default_factory=dict)

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
                b, ids = self.arms[a].bus, self.arms[a].ids
                ticks = []
                for k, j in enumerate(JOINTS):
                    st = b.read_joint_state(ids[j])
                    ticks.append(round(st.position_deg * 4096 / 360))
                    peak_load[a][k] = max(peak_load[a][k], abs(st.load_raw))
                    if st.temperature_c >= self.cfg.fault_thresholds.overtemperature_c:
                        raise ReplayAborted(f"{a} {j} at {st.temperature_c} C")
                    if st.current_raw >= self.cfg.fault_thresholds.overcurrent_raw:
                        raise ReplayAborted(f"{a} {j} current {st.current_raw} raw (overcurrent)")
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
            elif now - last_move > STALL_S:
                raise ReplayAborted(f"frame {frame}: stopped short of the goal: "
                                    + "; ".join(f"{a} err {[c - g for c, g in zip(cur[a], goals[a])]}"
                                                for a in goals))
            if now - t0 > timeout:
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
        """Small correction from the measured pose to frame 0 (after the start-pose check)."""
        start = Frame(-1, "measured start", self.present_q(), dict(self.frames[0].locked))
        self._move(start, self.frames[0])

    def run(self):
        ev = gripper_events(self.frames)
        phase = self.frames[0].step
        for prev, cur in zip(self.frames, self.frames[1:]):
            self._check_abort()
            if self.pause_each_phase and cur.step != phase:
                if self.prompt(f"next phase: {cur.step!r} (frame {cur.index}). Enter to go, q to stop: ").strip().lower() == "q":
                    raise ReplayAborted("stopped by the owner at a phase pause")
            phase = cur.step
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
