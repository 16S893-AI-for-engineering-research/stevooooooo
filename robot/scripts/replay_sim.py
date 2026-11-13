"""Simulated SO-101 servos for the cycle replay: dry-run and tests (no hardware).

Kept outside src/ on purpose: the simulated gripper channel does not check the AA-17
token (it is not a real servo), and src/ must never hold a way around that gate.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from inchworm.cycle_replay import (  # noqa: E402
    ARMS, DEFAULT_VMAX_TICKS_S, JOINTS, ReplayRunner, build_runner, q_to_ticks,
)

_REG_TORQUE_ENABLE = 0x28



class SimArmBus:
    """Six simulated STS3215 servos behind the ArmServoBus surface used by the replay.

    Time comes from an injected clock; a servo with torque on moves toward its goal at
    its Goal_Speed (ticks/s; 0 = 3000, i.e. full speed). A goal write enables torque, as
    measured on the real servos (2026-11-12). ``stall`` = {servo id: tick} stops a servo
    there (an obstruction).
    """

    def __init__(self, role: str, start_ticks: Sequence[int], clock, limits=None, stall=None, contact=None):
        from types import SimpleNamespace
        self.role, self.clock = role, clock
        self._joint_id_map = {n: i + 1 for i, n in enumerate((*JOINTS, "gripper"))}
        self.pos = {i + 1: float(t) for i, t in enumerate(start_ticks)}
        self.goal = dict(self.pos)
        self.speed = {i: 0 for i in self.pos}
        self.torque = {i: False for i in self.pos}
        self.tlimit = {i: 1000 for i in self.pos}
        self.limits = limits or {i: (0, 4095) for i in self.pos}
        self.stall = dict(stall or {})
        # {servo id: tick}: a one-sided stop (AA-52: the lower arm on its shoulder). It blocks
        # only motion toward higher ticks past it; moving back off it is free.
        self.contact = dict(contact or {})
        self.t_last = clock()
        self.dry_run = False
        self._dry_run = False
        self.max_speed_seen = {i: 0 for i in self.pos}
        self._bus = SimpleNamespace(read_bytes=self._read_bytes, write_bytes=self._write_bytes)

    # time
    def _step(self):
        now = self.clock()
        dt, self.t_last = now - self.t_last, now
        for i in self.pos:
            if not self.torque[i]:
                continue
            v = self.speed[i] or 3000
            lo, hi = self.limits[i]
            tgt = min(max(self.goal[i], lo), hi)
            if i in self.stall:
                a, b = sorted((self.pos[i], tgt))
                if a <= self.stall[i] <= b:
                    tgt = self.stall[i]
            if i in self.contact and tgt > self.contact[i]:
                tgt = max(self.contact[i], min(self.pos[i], tgt))
            d = tgt - self.pos[i]
            self.pos[i] += max(-v * dt, min(v * dt, d))

    # raw registers used by GripperLockSM / scripts
    def _read_bytes(self, sid, reg, n):
        self._step()
        val = {9: self.limits[sid][0], 11: self.limits[sid][1], 0x30: self.tlimit[sid],
               0x2E: self.speed[sid], 0x29: 20, _REG_TORQUE_ENABLE: int(self.torque[sid]),
               56: int(round(self.pos[sid]))}.get(reg, 0)
        return bytes([val & 0xFF, (val >> 8) & 0xFF])[:n]

    def _write_bytes(self, sid, reg, data):
        if reg == _REG_TORQUE_ENABLE:
            self.torque[sid] = bool(data[0])
        elif reg == 42:
            self.goal[sid] = data[0] | (data[1] << 8)
            self.torque[sid] = True

    # ArmServoBus surface
    def _servo_id_for_joint(self, name):
        return self._joint_id_map[name]

    def _emit(self, *a, **k):
        pass

    def read_position_deg(self, sid):
        self._step()
        return round(self.pos[sid]) * 360 / 4096

    def read_joint_state(self, sid):
        from inchworm.servo_bus import JointState
        self._step()
        return JointState(sid, round(self.pos[sid]) * 360 / 4096, 0, 40, 12.2, 31, 2)

    def write_position_deg(self, joint, deg):
        if joint == "gripper":
            raise PermissionError("gripper writes only through the AA-17 channel")
        sid = self._joint_id_map[joint]
        self._step()
        self.goal[sid] = int(round(deg * 4096 / 360))
        self.torque[sid] = True
        self.max_speed_seen[sid] = max(self.max_speed_seen[sid], self.speed[sid] or 3000)

    def sync_write_goal_ticks(self, ticks_by_joint, speed_by_joint=None):
        if "gripper" in ticks_by_joint:
            raise PermissionError("gripper writes only through the AA-17 channel")
        self._step()
        for name, t in ticks_by_joint.items():
            sid = self._joint_id_map[name]
            self.goal[sid] = int(t)
            if speed_by_joint is not None:
                self.speed[sid] = int(speed_by_joint[name])
            self.torque[sid] = True
            self.max_speed_seen[sid] = max(self.max_speed_seen[sid], self.speed[sid] or 3000)

    def gripper_channel(self, token):
        from types import SimpleNamespace

        def w(deg):
            self._step()
            self.goal[6] = int(round(deg * 4096 / 360))
            self.torque[6] = True
            self.max_speed_seen[6] = max(self.max_speed_seen[6], self.speed[6] or 3000)
        return SimpleNamespace(write_position_deg=w)

    def set_goal_speed(self, sid, v):
        self._step(); self.speed[sid] = int(v)

    def set_acceleration(self, sid, v):
        pass

    def set_torque_limit(self, sid, v):
        self.tlimit[sid] = int(v)

    def set_torque_enable(self, sid, on):
        self._step()
        if on:
            self.goal[sid] = round(self.pos[sid])
        self.torque[sid] = bool(on)


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def simulate(cfg, frames, *, vmax_ticks_s: int = DEFAULT_VMAX_TICKS_S, start_ticks=None,
             stall=None, log=lambda r: None, segments=None, fold_press=False, contact=None) -> Tuple[ReplayRunner, Dict[str, "SimArmBus"]]:
    """Run the full replay sequence against simulated servos (same code as live)."""
    clock = SimClock()
    buses = {}
    for a in ARMS:
        t0 = start_ticks[a] if start_ticks else q_to_ticks(cfg, a, frames[0].q[a]) + [2516 if a == "left" else 2499]
        lim = getattr(cfg.motion_guard, f"{a}_arm_limits").servo_tick_limits
        limits = {i + 1: tuple(lim[j]) for i, j in enumerate((*JOINTS, "gripper"))}
        buses[a] = SimArmBus(f"arm_{a}", t0, clock, limits=limits, stall=(stall or {}).get(a),
                             contact=(contact or {}).get(a))
    r = build_runner(cfg, frames, buses, live=False, vmax_ticks_s=vmax_ticks_s,
                     clock=clock, sleep=clock.sleep, log=log)
    r.energise()
    r.lock_both()
    r.move_to_start()
    if fold_press:
        r.fold_press(frames[0], "start")
        r.fold_unpress(frames[0])
    if segments is None:
        r.run()
    else:
        r.run_fluid(segments)
    if fold_press:
        r.fold_press(frames[-1], "end")
    return r, buses
