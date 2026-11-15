"""AA-44 / AA-23: live leader -> follower teleop core (no LeRobot import).

The same core runs the supervised slow mapping check (``scripts/teleop_live.py``) and,
later, the LeRobot plugin used by ``lerobot-teleoperate`` / ``lerobot-record``.

Per loop: read leader joints 1-5 (ticks; only error bit 0x08 tolerated), map them with the
software fit (``calibration_data/leader_map_<arm>.json``, sign/offset per joint), pick the
4096-wrap representative nearest the last goal (never jump across the seam), clamp to the
tighter of the configured soft limits and the servo EEPROM limits (10 ticks inside), and
send joints 1-5 in ONE sync-write with a capped Goal_Speed.

Safety
------
* Joint 6 (gripper) is never mapped or written (sync_write rejects it; AA-17 owns it).
* Limits are ENFORCED by clamping: a goal past a limit is sent AT the limit and counted.
* Start gate: every joint's mapped target must be within ``start_max_deg`` of the follower's
  present pose, or nothing is energised (match the leader to the follower first).
* Mesh collision check: NOT run on this path (owner decision 2026-11-13, first teleop).
  The caller must pass the owner's reason; it is logged.
* Stop = HOLD (present position re-commanded, torque on); torque off only after the owner
  supports the arm (caller), and a final torque-off sweep on exit.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICKS = 4096
DEG_PER_TICK = 360.0 / TICKS
REG_PRESENT_POSITION = 56
REG_TORQUE_ENABLE = 0x28
REG_MIN_LIMIT, REG_MAX_LIMIT = 9, 11
EEPROM_INSIDE_TICKS = 10
DEFAULT_SPEED_TICKS_S = 250       # ~22 deg/s: owner choice 2026-11-15 (best of 150/250 x smooth 0.2/0.35/1)
MAX_SPEED_TICKS_S = 600
DEFAULT_ACCEL = 20                # energise only
STREAM_ACCEL = 100                # while following: same ramp as the AA-50 fluid replay
#: Smoothing (2026-11-15, owner: "a little jittery"). Leader readings go through a 1-tick
#: deadband and an exponential filter; Goal_Speed is set per joint from the remaining distance
#: (cover each step in about SPEED_FACTOR x the step period) and capped at the configured speed, so the servo
#: no longer starts and stops at full speed every 33 ms step.
DEFAULT_SMOOTH_ALPHA = 1.0        # filter OFF: owner 2026-11-15 preferred no smoothing at 250 ticks/s (0.2/0.35 tried)
DEADBAND_TICKS = 1.0
SPEED_FACTOR = 1.5                # Goal_Speed = 1.5 x the step distance per period
MIN_SPEED_TICKS_S = 5             # never 0 (0 = full speed)
DEFAULT_TORQUE_LIMIT = 500        # of 1000, same as the AA-50 replay
START_MAX_DEG = 10.0


class TeleopRefused(RuntimeError):
    pass


def load_leader_map(path: Path, follower: str) -> dict:
    if not path.exists():
        raise TeleopRefused(f"no leader mapping {path}; run scripts/leader_mapping.py capture/fit first")
    m = json.loads(path.read_text())
    if m.get("follower") != follower:
        raise TeleopRefused(f"{path.name} is for follower {m.get('follower')!r}, not {follower!r}")
    if m.get("problems"):
        raise TeleopRefused(f"{path.name} has fit problems: {m['problems']}")
    for j in J5:
        jm = m.get("joints", {}).get(j)
        if jm is None or jm.get("sign") not in (1, -1) or not jm.get("sign_determined", False):
            raise TeleopRefused(f"{path.name}: joint {j} has no determined sign")
    return m


def map_unwrapped(mapping: dict, leader_ticks: Sequence[int]) -> List[float]:
    """Follower ticks (NOT wrapped) for joints 1-5 from leader ticks."""
    return [mapping["joints"][j]["sign"] * leader_ticks[i] + mapping["joints"][j]["offset_ticks"]
            for i, j in enumerate(J5)]


def nearest_rep(t: float, ref: float) -> float:
    """The representative t + k*4096 closest to ref (no seam jump)."""
    return t + TICKS * round((ref - t) / TICKS)


def soft_limit_ticks(cfg, arm: str) -> Dict[str, tuple]:
    """Soft limits (rad) of joints 1-5 as tick intervals via the verified calibration."""
    cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
    if cal is None or cal.unverified:
        raise TeleopRefused(f"{arm} servo_calibration missing or unverified")
    lim = getattr(cfg.motion_guard, f"{arm}_arm_limits")
    out = {}
    for j in J5:
        lo, hi = getattr(lim, f"{j}_lower_rad"), getattr(lim, f"{j}_upper_rad")
        if lo is None or hi is None:
            raise TeleopRefused(f"{arm} {j}: soft limit missing")
        z, s = getattr(cal, f"{j}_zero_offset_deg"), getattr(cal, f"{j}_sign")
        a, b = [(z + math.degrees(v) / s) / DEG_PER_TICK for v in (lo, hi)]
        out[j] = (min(a, b), max(a, b))
    return out


@dataclass
class TeleopCore:
    """One follower arm driven from the leader. Buses are injected (real or simulated)."""
    leader_read: Callable[[int, int, int], bytes]     # read_bytes_tolerating(sid, reg, n)
    follower_bus: object                              # ArmServoBus surface
    mapping: dict
    limits: Dict[str, tuple]                          # tick interval per joint (soft ∩ EEPROM)
    speed_ticks_s: int = DEFAULT_SPEED_TICKS_S
    log: Callable[[dict], None] = lambda r: None
    start_max_deg: float = START_MAX_DEG
    last_goal: Optional[List[float]] = None
    energised: bool = False
    clamps: Dict[str, int] = field(default_factory=lambda: {j: 0 for j in J5})
    smooth_alpha: float = DEFAULT_SMOOTH_ALPHA
    period_s: float = 1.0 / 30
    filtered: Optional[List[float]] = None
    est: Optional[List[float]] = None

    def __post_init__(self):
        if not 1 <= int(self.speed_ticks_s) <= MAX_SPEED_TICKS_S:
            raise TeleopRefused(f"speed must be 1..{MAX_SPEED_TICKS_S} ticks/s")
        if not 0.0 < float(self.smooth_alpha) <= 1.0:
            raise TeleopRefused("smooth_alpha must be in (0, 1]")

    # -- reads ---------------------------------------------------------------
    def leader_ticks(self) -> List[int]:
        return [int.from_bytes(self.leader_read(s, REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]

    def follower_ticks(self) -> List[int]:
        return [round(self.follower_bus.read_position_deg(s) / DEG_PER_TICK) for s in range(1, 6)]

    # -- target --------------------------------------------------------------
    def target(self, leader_ticks: Sequence[int], ref: Sequence[float]) -> tuple:
        raw = map_unwrapped(self.mapping, leader_ticks)
        goal, clamped = [], []
        for k, j in enumerate(J5):
            t = nearest_rep(raw[k], ref[k])
            lo, hi = self.limits[j]
            c = min(max(t, lo), hi)
            if c != t:
                clamped.append(j)
            goal.append(c)
        return goal, clamped

    def start_problems(self) -> List[str]:
        present = self.follower_ticks()
        goal, _ = self.target(self.leader_ticks(), present)
        out = []
        for k, j in enumerate(J5):
            d = (goal[k] - present[k]) * DEG_PER_TICK
            if abs(d) > self.start_max_deg:
                out.append(f"{j}: leader asks {d:+.1f} deg from the follower's pose (max {self.start_max_deg:g}); "
                           "move the LEADER to match the follower")
        return out

    # -- motion --------------------------------------------------------------
    def energise(self, accel: int = DEFAULT_ACCEL, torque_limit: int = DEFAULT_TORQUE_LIMIT) -> None:
        b = self.follower_bus
        for s in range(1, 6):
            b.set_goal_speed(s, int(self.speed_ticks_s))
            b.set_acceleration(s, accel)
            b.set_torque_limit(s, torque_limit)
            b.set_torque_enable(s, True)        # goal := present first (servo_bus)
        for s in range(1, 6):
            b.set_acceleration(s, STREAM_ACCEL)
        self.last_goal = [float(t) for t in self.follower_ticks()]
        self.filtered = list(self.last_goal)      # filter starts at the follower's pose
        self.est = list(self.last_goal)           # estimated servo position (for Goal_Speed)
        self.energised = True

    def _smooth(self, goal: List[float]) -> List[float]:
        """Deadband + exponential filter on the (clamped, unwrapped) goal."""
        if self.filtered is None:
            self.filtered = list(goal)
            return list(goal)
        out = []
        for k, g in enumerate(goal):
            f = self.filtered[k]
            if abs(g - f) > DEADBAND_TICKS:
                f = f + self.smooth_alpha * (g - f)
            out.append(f)
        self.filtered = out
        return out

    def step(self) -> dict:
        """One teleop step: read leader, map, clamp, sync-write joints 1-5."""
        if not self.energised:
            raise TeleopRefused("not energised")
        lt = self.leader_ticks()
        goal, clamped = self.target(lt, self.last_goal)
        for j in clamped:
            self.clamps[j] += 1
        goal = self._smooth(goal)            # filtered values stay inside the clamped interval
        ticks = {j: int(round(g)) for j, g in zip(J5, goal)}
        cap = int(self.speed_ticks_s)
        speeds = {}
        for k, j in enumerate(J5):
            # Goal_Speed from the distance the servo still has to go (estimated, no bus read):
            # small corrections move slowly, a lagging joint gets up to the cap.
            d = goal[k] - self.est[k]
            v = int(min(cap, max(MIN_SPEED_TICKS_S, SPEED_FACTOR * abs(d) / self.period_s)))
            speeds[j] = v
            self.est[k] += max(-v * self.period_s, min(v * self.period_s, d))
        self.follower_bus.sync_write_goal_ticks(ticks, speeds)
        self.last_goal = goal
        return {"leader": lt, "goal": [ticks[j] for j in J5], "clamped": clamped}


def eeprom_limits(read_bytes, inside: int = EEPROM_INSIDE_TICKS) -> Dict[str, tuple]:
    out = {}
    for k, j in enumerate(J5):
        lo = int.from_bytes(read_bytes(k + 1, REG_MIN_LIMIT, 2), "little")
        hi = int.from_bytes(read_bytes(k + 1, REG_MAX_LIMIT, 2), "little")
        if not 0 <= lo < hi <= 4095:
            raise TeleopRefused(f"{j}: EEPROM limits {lo}..{hi} invalid")
        out[j] = (lo + inside, hi - inside)
    return out


def intersect(a: Dict[str, tuple], b: Dict[str, tuple]) -> Dict[str, tuple]:
    out = {}
    for j in J5:
        lo, hi = max(a[j][0], b[j][0]), min(a[j][1], b[j][1])
        if lo >= hi:
            raise TeleopRefused(f"{j}: soft and EEPROM limits do not overlap ({a[j]} vs {b[j]})")
        out[j] = (lo, hi)
    return out


def run_loop(core: TeleopCore, *, hz: float, should_stop: Callable[[], bool],
             clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep,
             report_every_s: float = 1.0, max_s: Optional[float] = None) -> dict:
    """Run steps at ``hz`` until should_stop() (or max_s). Logs a stats line every report_every_s."""
    period = 1.0 / hz
    core.period_s = period
    t0 = last = clock()
    n = n_win = 0
    worst_lag = [0] * 5
    slow = 0
    while not should_stop() and (max_s is None or clock() - t0 < max_s):
        ts = clock()
        out = core.step()
        n += 1
        n_win += 1
        if n % 5 == 0:     # follower read-back for the log only (lag between goal and present)
            pres = core.follower_ticks()
            worst_lag = [max(w, abs(p - g)) for w, p, g in zip(worst_lag, pres, out["goal"])]
        now = clock()
        if now - ts > period:
            slow += 1
        if now - last >= report_every_s:
            core.log({"event": "teleop_stats", "t": round(now - t0, 2), "hz": round(n_win / (now - last), 1),
                      "worst_lag_ticks": worst_lag, "clamps": dict(core.clamps), "overruns": slow,
                      "goal": out["goal"]})
            last, n_win, worst_lag, slow = now, 0, [0] * 5, 0
        rest = period - (clock() - ts)
        if rest > 0:
            sleep(rest)
    return {"steps": n, "s": round(clock() - t0, 2), "clamps": dict(core.clamps)}
