"""AA-23 rapid-fire handoff recording: the automatic return between demos.

Owner decisions 2026-11-15 (AA-23 notes "OWNER SPEC" + "OWNER CONFIRMED"):

* After each demo (left-arm insertion up to just before locking) both arms go back to the
  handoff stop pose written by ``scripts/replay_cycle.py --stop-after-frame 42``.
* (a) The left arm first moves STRAIGHT out of the key: -z of its base frame (the reverse of
  the planned insertion, timeline frames 42->48), 50 mm, orientation held, 10 mm/s. Agent
  refinement 2026-11-15 (to confirm with the owner): never lower than the stop pose height
  (50 mm under the key), so a shallow demo end pulls out only as far as the stop height. Then a
  joint-space straight line to its stop pose. The demo path is NOT retraced (it has
  extraneous motion).
* (b) The right arm then goes back to its stop pose. The tube is modelled rigidly carried by
  the right gripper. More than 5 deg of right drift on any joint = refuse and ask the owner.
* (c) The mesh check stays on; nothing is loosened.
* (d) Return speed 10 deg/s (joint lines) / 10 mm/s (retract). The plan is fully checked
  before anything moves; the first live return of a session needs a typed YES.

Grippers are never commanded here. Nothing opens a port: callers inject cores and readers.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DEG_PER_TICK = 360.0 / 4096
RETRACT_MM = 50.0                  # owner (a): the insertion depth of the timeline
RETRACT_MM_S = 10.0                # owner (d)
RETRACT_STEP_MM = 1.0
RETURN_DEG_S = 10.0                # owner (d)
RIGHT_DRIFT_MAX_DEG = 5.0          # owner (b)
EPISODE_MAX_S = 180.0              # owner: 3 min safety cap, auto stop + save
LINE_STEP_DEG = 2.0                # mesh-check spacing on a joint line (= handoff.CHECK_STEP_DEG)
IK_JUMP_MAX_DEG = 3.0              # one 1 mm retract step may not swing a joint more than this
#: Agent choice 2026-11-15, UNVERIFIED: present more than this behind the commanded goal
#: during a return = something resists (contact) -> stop and hold. The cycle replay at up to
#: 57 deg/s tracked within 36 ticks; the return runs at 10 deg/s.
TRACK_TOL_TICKS = 45
SETTLE_TICKS = 10
SETTLE_TIMEOUT_S = 3.0


class ReturnRefused(RuntimeError):
    """The return was not started: nothing moved."""


class ReturnAborted(RuntimeError):
    """The return stopped part-way; the arm holds where it is."""


def q_to_ticks_f(cfg, arm: str, q: Sequence[float]) -> List[float]:
    cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
    return [(getattr(cal, f"{j}_zero_offset_deg") + math.degrees(float(v)) / getattr(cal, f"{j}_sign"))
            / DEG_PER_TICK for j, v in zip(J5, q)]


def ticks_to_q(cfg, arm: str, ticks: Sequence[float]) -> List[float]:
    cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
    return [getattr(cal, f"{j}_sign") * math.radians(float(t) * DEG_PER_TICK - getattr(cal, f"{j}_zero_offset_deg"))
            for j, t in zip(J5, ticks)]


class CarriedTubeModel:
    """Cell model for the return: the tube rigidly carried by the holding (other) arm's gripper,
    equal to the handoff tube pose when that arm is at its stop pose."""

    def __init__(self, cfg, checker, home: Dict[str, Sequence[float]], tube_home, follower: str = "left"):
        self.cfg, self.checker, self.follower = cfg, checker, follower
        self.other = "right" if follower == "left" else "left"
        self.home = {a: [float(t) for t in v] for a, v in home.items()}
        G0 = self._grip(self.home[self.other])
        self.carry = np.linalg.inv(G0) @ np.asarray(tube_home, dtype=float)
        self.checks = 0

    def _q(self, arm, ticks):
        return ticks_to_q(self.cfg, arm, ticks)

    def _grip(self, other_ticks) -> np.ndarray:
        from inchworm.system_model.collision import CellState
        qo = self._q(self.other, other_ticks)
        qf = self._q(self.follower, self.home[self.follower])
        ql, qr = (qf, qo) if self.follower == "left" else (qo, qf)
        return np.asarray(self.checker.link_poses(CellState(left_q=ql, right_q=qr, dexarm_q={}))
                          [f"{self.other}_gripper"], dtype=float)

    def tube(self, other_ticks) -> np.ndarray:
        return self._grip(other_ticks) @ self.carry

    def problem(self, follower_ticks, other_ticks) -> Optional[str]:
        """Every checked pair (arm=None: the moving tube against the follower too). Fails closed."""
        from inchworm.cycle_replay import cell_state
        self.checks += 1
        try:
            qf, qo = self._q(self.follower, follower_ticks), self._q(self.other, other_ticks)
            ql, qr = (qf, qo) if self.follower == "left" else (qo, qf)
            hits = self.checker.check(cell_state(ql, qr, self.tube(other_ticks)), arm=None)
        except Exception as exc:  # noqa: BLE001
            return f"collision check failed (fail closed): {exc}"
        return f"mesh collision: {hits[0]}" if hits else None


@dataclass
class Leg:
    arm: str
    name: str
    waypoints: List[List[float]]          # ticks; waypoints[0] = where the leg starts
    duration_s: float


@dataclass
class Plan:
    legs: List[Leg]
    checks: int
    summary: List[str] = field(default_factory=list)


def _line(a: Sequence[float], b: Sequence[float]) -> List[List[float]]:
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(math.ceil(float(np.abs(b - a).max()) * DEG_PER_TICK / LINE_STEP_DEG)))
    return [list(a + (b - a) * k / n) for k in range(n + 1)]


def _deg(a, b) -> float:
    return float(np.abs(np.asarray(a, float) - np.asarray(b, float)).max()) * DEG_PER_TICK


def plan_return(cfg, model: CarriedTubeModel, chain, limits: Dict[str, Dict[str, tuple]],
                now: Dict[str, Sequence[float]]) -> Plan:
    """Plan and fully check the return from ``now`` to ``model.home``. Raises ReturnRefused."""
    f, o = model.follower, model.other
    home = model.home
    drift = [(j, (float(n) - float(h)) * DEG_PER_TICK) for j, n, h in zip(J5, now[o], home[o])]
    far = [f"{j} {d:+.1f} deg" for j, d in drift if abs(d) > RIGHT_DRIFT_MAX_DEG]
    if far:
        raise ReturnRefused(f"the {o} arm has drifted from its stop pose ({', '.join(far)}; max "
                            f"{RIGHT_DRIFT_MAX_DEG:g} deg): the tube was probably moved. Nothing moved; "
                            "decide by hand (q = release)")
    # leg 1: straight out of the key, -z of the follower base frame, orientation held
    # (never lower than the stop pose, which is RETRACT_MM under the key: going further down
    # would only add a dip the joint line then climbs back out of)
    q = np.asarray(ticks_to_q(cfg, f, now[f]), dtype=float)
    T0 = chain.fk(q)
    z_stop = float(chain.fk(np.asarray(ticks_to_q(cfg, f, home[f]), dtype=float))[2, 3])
    depth_mm = min(RETRACT_MM, max(0.0, (float(T0[2, 3]) - z_stop) * 1000.0))
    pts = [[float(t) for t in now[f]]]
    n = int(math.ceil(depth_mm / RETRACT_STEP_MM - 1e-9))
    for k in range(1, n + 1):
        Tk = T0.copy()
        Tk[2, 3] -= k * depth_mm / n / 1000.0
        r = chain.ik(Tk, q_seed=q)
        if not r.reachable:
            raise ReturnRefused(f"retract: {k * depth_mm / n:.0f} mm out is not reachable with the gripper "
                                f"orientation held (IK error {r.pos_err_m * 1000:.2f} mm). Nothing moved")
        if np.degrees(np.abs(r.q - q)).max() > IK_JUMP_MAX_DEG:
            raise ReturnRefused(f"retract: the arm would swing at {k * depth_mm / n:.0f} mm out (IK jump). "
                                "Nothing moved")
        q = np.asarray(r.q, dtype=float)
        pts.append(q_to_ticks_f(cfg, f, q))
    legs = [Leg(f, f"{f} straight out {depth_mm:.0f} mm", pts, depth_mm / RETRACT_MM_S)] if n else []
    # leg 2: follower joint line to its stop pose
    d = _deg(pts[-1], home[f])
    legs.append(Leg(f, f"{f} to its stop pose", _line(pts[-1], home[f]), max(0.5, d / RETURN_DEG_S)))
    # leg 3: other arm (with the tube) to its stop pose
    d = _deg(now[o], home[o])
    if d / DEG_PER_TICK > SETTLE_TICKS:
        legs.append(Leg(o, f"{o} (with the tube) to its stop pose", _line(now[o], home[o]),
                        max(0.5, d / RETURN_DEG_S)))
    c0 = model.checks
    for leg in legs:
        lim = limits[leg.arm]
        for w in leg.waypoints[1:]:
            out = [j for j, t in zip(J5, w) if not lim[j][0] <= t <= lim[j][1]]
            if out:
                raise ReturnRefused(f"{leg.name}: outside the teleop limits on {out}. Nothing moved")
            fw, ow = (w, now[o]) if leg.arm == f else (home[f], w)
            p = model.problem(fw, ow)
            if p is not None:
                raise ReturnRefused(f"{leg.name}: {p}. Nothing moved")
    summary = [f"{leg.name}: {leg.duration_s:.1f} s, {len(leg.waypoints) - 1} checked steps" for leg in legs]
    return Plan(legs, model.checks - c0, summary)


def run_leg(core, leg: Leg, *, hz: float = 30.0, clock: Callable[[], float] = time.monotonic,
            sleep: Callable[[float], None] = time.sleep, log: Callable[[dict], None] = lambda r: None,
            interrupted: Callable[[], bool] = lambda: False) -> dict:
    """Stream ``leg`` to ``core`` (TeleopCore: clamps, Goal_Speed). Present lagging the goal by
    more than TRACK_TOL_TICKS = stop and hold where it is (ReturnAborted)."""
    wps = [np.asarray(w, float) for w in leg.waypoints]
    period = 1.0 / hz
    t0 = clock()
    n = 0
    worst = 0
    while True:
        ts = clock()
        if interrupted():
            hold_here(core)
            raise ReturnAborted(f"{leg.name}: interrupted")
        s = min(1.0, (ts - t0) / leg.duration_s) * (len(wps) - 1)
        k = min(int(s), len(wps) - 2) if len(wps) > 1 else 0
        goal = wps[-1] if len(wps) == 1 else wps[k] + (wps[k + 1] - wps[k]) * (s - k)
        core.command([float(x) for x in goal])
        n += 1
        if n % 3 == 0:
            pres = core.follower_ticks()
            err = max(abs(p - g) for p, g in zip(pres, core.last_goal))
            worst = max(worst, err)
            if err > TRACK_TOL_TICKS:
                hold_here(core)
                raise ReturnAborted(f"{leg.name}: the arm is {err:.0f} ticks behind its goal (max "
                                    f"{TRACK_TOL_TICKS}): something resists. Holding where it is")
        if s >= len(wps) - 1:
            break
        rest = period - (clock() - ts)
        if rest > 0:
            sleep(rest)
    t1 = clock()
    while True:
        pres = core.follower_ticks()
        err = max(abs(p - g) for p, g in zip(pres, wps[-1]))
        if err <= SETTLE_TICKS or clock() - t1 > SETTLE_TIMEOUT_S:
            break
        sleep(period)
    rep = {"leg": leg.name, "steps": n, "s": round(clock() - t0, 2), "worst_track_ticks": round(worst),
           "final_error_ticks": round(err)}
    log({"event": "return_leg", **rep})
    return rep


def hold_here(core) -> None:
    """Command the present position (stop moving, keep holding)."""
    core.command([float(t) for t in core.follower_ticks()])


def holding_core(cfg, arm: str, bus, read_bytes, log=lambda r: None):
    """TeleopCore for an arm that is already holding (no torque cut, no goal write)."""
    from inchworm import teleop_live as tl
    limits = tl.follower_preflight(cfg, arm, read_bytes, require_torque_off=False)
    core = tl.TeleopCore(leader_read=_no_leader, follower_bus=bus, mapping={}, limits=limits, log=log)
    core.adopt_holding()
    return core


def _no_leader(*_a):
    raise RuntimeError("return core has no leader")
