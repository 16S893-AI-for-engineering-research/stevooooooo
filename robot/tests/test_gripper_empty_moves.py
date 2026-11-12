"""Empty-gripper close and end measurement (AA-43, owner-approved 2026-11-12).

A small simulated gripper servo stands in for the bus: it moves toward its goal
at a fixed rate while torque is on, and stops at an optional obstruction.
"""
from __future__ import annotations

import pytest

from inchworm.config import GripperBandConfig, GripperLockConfig
from inchworm.gripper_lock import (
    BandsNotConfigured,
    GripperLockSM,
    GripperObstructed,
    GripperState,
    IllegalTransition,
    TubeAttachedRefused,
)
from inchworm.servo_bus import JointState

T = lambda ticks: ticks * 360.0 / 4096  # noqa: E731

# Left gripper as configured in hardware.yaml: open band 1506..1610, closed 2521.
_BANDS = GripperBandConfig(
    unlocked_low_deg=T(1506), unlocked_high_deg=T(1610),
    engaged_low_deg=T(1506), engaged_high_deg=T(1610),
    locked_low_deg=T(1776), locked_high_deg=T(1827),
    lock_overtravel_goal_deg=T(2174), closed_empty_deg=T(2521),
    lock_torque_limit_raw=None, unverified=True,
)
_CFG = GripperLockConfig(arm_left=_BANDS, arm_right=_BANDS)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class _SimServo:
    def __init__(self, clock, pos=1569, lo=1521, hi=2531, obstruction=None, rate=150.0):
        self.clock, self.pos, self.lo, self.hi = clock, float(pos), lo, hi
        self.obstruction, self.rate = obstruction, rate
        self.goal, self.torque, self.t_last = pos, False, 0.0
        self.regs = {0x30: 1000, 0x2E: 0, 0x29: 0}
        self.torque_log = []

    def step(self):
        dt, self.t_last = self.clock.t - self.t_last, self.clock.t
        if not self.torque:
            return
        target = min(max(self.goal, self.lo), self.hi)
        if self.obstruction is not None:
            a, b = sorted((self.pos, target))
            if a <= self.obstruction <= b:
                target = self.obstruction
        d = target - self.pos
        self.pos += max(-self.rate * dt, min(self.rate * dt, d))


class _RawBus:
    def __init__(self, s):
        self.s = s

    def read_bytes(self, sid, reg, n):
        v = {9: self.s.lo, 11: self.s.hi}.get(reg, self.s.regs.get(reg, 0))
        return bytes([v & 0xFF, v >> 8])[:n]


class _Channel:
    def __init__(self, s):
        self.s = s

    def write_position_deg(self, deg):
        self.s.goal = round(deg * 4096 / 360)
        self.s.torque = True   # measured: a goal write enables torque


class _FakeBus:
    def __init__(self, s):
        self.s, self._bus = s, _RawBus(s)

    def gripper_channel(self, token):
        return _Channel(self.s)

    def _servo_id_for_joint(self, name):
        assert name == "gripper"
        return 6

    def read_position_deg(self, sid):
        self.s.step()
        return T(round(self.s.pos))

    def read_joint_state(self, sid):
        self.s.step()
        return JointState(6, T(round(self.s.pos)), 0, 40, 12.2, 31, 2)

    def set_goal_speed(self, sid, v):
        self.s.regs[0x2E] = v

    def set_acceleration(self, sid, v):
        self.s.regs[0x29] = v

    def set_torque_limit(self, sid, v):
        self.s.regs[0x30] = v

    def set_torque_enable(self, sid, on):
        if on:
            self.s.goal = round(self.s.pos)
        self.s.torque = on
        self.s.torque_log.append(on)


def _sm(servo, dry_run=False):
    return GripperLockSM(bus=_FakeBus(servo), role="arm_left", config=_CFG,
                         opposite_arm_locked_fn=lambda: False, dry_run=dry_run)


def _unlocked(servo):
    sm = _sm(servo)
    sm._state = GripperState.UNLOCKED  # as after reset(); avoids re-driving the sim
    return sm


class TestCloseEmpty:
    def test_closes_to_closed_empty_and_switches_torque_off(self):
        c = _Clock(); s = _SimServo(c)
        sm = _unlocked(s)
        res = sm.close_empty(tube_attached=False, sleep=c.sleep, clock=c)
        assert res["status"] == "SETTLED" and abs(res["final_ticks"] - 2521) <= 1
        assert sm.state == GripperState.CLOSED_EMPTY
        assert s.torque is False and s.torque_log[-1] is False
        assert s.regs == {0x30: 1000, 0x2E: 0, 0x29: 0}   # previous RAM settings restored

    def test_obstruction_faults_and_switches_torque_off(self):
        c = _Clock(); s = _SimServo(c, obstruction=1900)
        sm = _unlocked(s)
        with pytest.raises(GripperObstructed):
            sm.close_empty(tube_attached=False, sleep=c.sleep, clock=c)
        assert sm.state == GripperState.FAULT
        assert s.torque is False
        assert abs(s.pos - 1900) <= 1

    def test_refused_unless_tube_explicitly_not_attached(self):
        c = _Clock(); s = _SimServo(c)
        sm = _unlocked(s)
        for v in (True, None, 0):
            with pytest.raises(TubeAttachedRefused):
                sm.close_empty(tube_attached=v, sleep=c.sleep, clock=c)
        assert s.torque_log == [] and s.goal == 1569

    def test_refused_from_locked_or_unknown(self):
        c = _Clock(); s = _SimServo(c)
        sm = _sm(s)
        for st in (GripperState.UNKNOWN, GripperState.LOCKED, GripperState.APPROACH_READY, GripperState.FAULT):
            sm._state = st
            with pytest.raises(IllegalTransition):
                sm.close_empty(tube_attached=False, sleep=c.sleep, clock=c)
        assert s.torque_log == []

    def test_refused_without_closed_empty_configured(self):
        c = _Clock(); s = _SimServo(c)
        cfg = GripperLockConfig(arm_left=_BANDS.model_copy(update={"closed_empty_deg": None}))
        sm = GripperLockSM(bus=_FakeBus(s), role="arm_left", config=cfg, dry_run=False)
        sm._state = GripperState.UNLOCKED
        with pytest.raises(BandsNotConfigured):
            sm.close_empty(tube_attached=False, sleep=c.sleep, clock=c)

    def test_abort_stops_and_switches_torque_off(self):
        c = _Clock(); s = _SimServo(c)
        sm = _unlocked(s)
        res = sm.close_empty(tube_attached=False, should_abort=lambda: c.t > 1.0, sleep=c.sleep, clock=c)
        assert res["status"] == "ABORTED" and sm.state == GripperState.FAULT
        assert s.torque is False

    def test_dry_run_sends_nothing(self):
        c = _Clock(); s = _SimServo(c)
        sm = _sm(s, dry_run=True); sm._state = GripperState.UNLOCKED
        res = sm.close_empty(tube_attached=False)
        assert res["status"] == "DRY_RUN" and s.torque_log == [] and s.goal == 1569


class TestMeasureEnd:
    def test_open_end_goes_to_eeprom_min_and_reports_reached(self):
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _unlocked(s)
        res = sm.measure_end("open", tube_attached=False, sleep=c.sleep, clock=c)
        assert res["goal_ticks"] == 1521 and res["stopped_short"] is False
        assert sm.state == GripperState.UNKNOWN and s.torque is False

    def test_closed_end_goes_to_eeprom_max(self):
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _unlocked(s)
        res = sm.measure_end("closed", tube_attached=False, sleep=c.sleep, clock=c)
        assert res["goal_ticks"] == 2531 and abs(res["final_ticks"] - 2531) <= 1
        assert res["eeprom_limits"] == [1521, 2531]

    def test_mechanical_stop_before_limit_reported(self):
        c = _Clock(); s = _SimServo(c, pos=1569, obstruction=1545)
        sm = _unlocked(s)
        res = sm.measure_end("open", tube_attached=False, sleep=c.sleep, clock=c)
        assert res["stopped_short"] is True and abs(res["final_ticks"] - 1545) <= 1
        assert s.torque is False

    def test_refused_while_locked_and_with_tube(self):
        c = _Clock(); s = _SimServo(c)
        sm = _sm(s)
        sm._state = GripperState.LOCKED
        with pytest.raises(IllegalTransition):
            sm.measure_end("open", tube_attached=False, sleep=c.sleep, clock=c)
        sm._state = GripperState.UNLOCKED
        with pytest.raises(TubeAttachedRefused):
            sm.measure_end("open", tube_attached=True, sleep=c.sleep, clock=c)
        with pytest.raises(ValueError):
            sm.measure_end("middle", tube_attached=False)
        assert s.torque_log == []

    def test_cleanup_runs_when_a_read_fails(self):
        c = _Clock(); s = _SimServo(c)
        bus = _FakeBus(s)
        n = {"k": 0}
        orig = bus.read_joint_state

        def flaky(sid):
            n["k"] += 1
            if n["k"] == 5:
                raise RuntimeError("bus timeout")
            return orig(sid)

        bus.read_joint_state = flaky
        sm = GripperLockSM(bus=bus, role="arm_left", config=_CFG, dry_run=False)
        sm._state = GripperState.UNLOCKED
        with pytest.raises(RuntimeError):
            sm.measure_end("closed", tube_attached=False, sleep=c.sleep, clock=c)
        assert s.torque is False and s.regs == {0x30: 1000, 0x2E: 0, 0x29: 0}


def test_close_empty_read_failure_faults_with_torque_off():
    c = _Clock(); s = _SimServo(c)
    bus = _FakeBus(s)
    orig, n = bus.read_joint_state, {"k": 0}

    def flaky(sid):
        n["k"] += 1
        if n["k"] == 5:
            raise RuntimeError("bus timeout")
        return orig(sid)

    bus.read_joint_state = flaky
    sm = GripperLockSM(bus=bus, role="arm_left", config=_CFG, dry_run=False)
    sm._state = GripperState.UNLOCKED
    with pytest.raises(RuntimeError):
        sm.close_empty(tube_attached=False, sleep=c.sleep, clock=c)
    assert sm.state == GripperState.FAULT and s.torque is False


# ---------------------------------------------------------------------------
# Position lock (owner decision 2026-11-12): 45 deg past the open end
# ---------------------------------------------------------------------------

from inchworm.gripper_lock import LockVerdict, LockVerificationFailed  # noqa: E402

_PBANDS = _BANDS.model_copy(update={
    "locked_low_deg": T(2013), "locked_high_deg": T(2063),
    "lock_overtravel_goal_deg": T(2174), "lock_mode": "position",
    "lock_goal_deg": T(2038), "lock_torque_limit_raw": 300,
})
_PCFG = GripperLockConfig(arm_left=_PBANDS, arm_right=_PBANDS)


def _psm(servo, clock, dry_run=False, other_locked=True):
    sm = GripperLockSM(bus=_FakeBus(servo), role="arm_left", config=_PCFG,
                       opposite_arm_locked_fn=lambda: other_locked, dry_run=dry_run)
    sm._sleep, sm._clock = clock.sleep, clock
    return sm


class TestPositionLock:
    def test_locks_at_the_goal_and_holds_torque(self):
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _psm(s, c)
        sm._state = GripperState.APPROACH_READY
        assert sm.lock() == LockVerdict.LATCHED
        assert sm.state == GripperState.LOCKED
        assert abs(s.pos - 2038) <= 12 and s.torque is True        # holds the lock
        assert s.regs[0x2E] == 150 and s.regs[0x29] == 20 and s.regs[0x30] == 300
        assert s.torque_log[0] is True                              # enabled once, goal := present

    def test_obstruction_faults_and_switches_torque_off(self):
        c = _Clock(); s = _SimServo(c, pos=1569, obstruction=1800)
        sm = _psm(s, c)
        sm._state = GripperState.APPROACH_READY
        with pytest.raises(LockVerificationFailed) as ei:
            sm.lock()
        assert ei.value.verdict == LockVerdict.MISS
        assert sm.state == GripperState.FAULT and s.torque is False

    def test_config_rejects_goal_outside_band(self):
        with pytest.raises(ValueError):
            _PBANDS.model_copy(update={"lock_goal_deg": T(2100)}).model_validate(
                {**_PBANDS.model_dump(), "lock_goal_deg": T(2100)})

    def test_hold_reasserts_when_out_of_band(self):
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _psm(s, c)
        sm._state = GripperState.APPROACH_READY
        sm.lock()
        assert sm.hold_tick() is False
        s.pos = 1950.0; s.goal = 1950          # piece slipped / pushed open
        assert sm.hold_tick() is True and s.goal == 2038

    def test_unlock_needs_the_other_arm_locked(self):
        from inchworm.gripper_lock import UnlockRefused
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _psm(s, c, other_locked=False)
        sm._state = GripperState.APPROACH_READY
        sm.lock()
        with pytest.raises(UnlockRefused):
            sm.unlock()
        assert sm.state == GripperState.LOCKED

    def test_dry_run_sends_nothing(self):
        c = _Clock(); s = _SimServo(c, pos=1569)
        sm = _psm(s, c, dry_run=True)
        sm._state = GripperState.APPROACH_READY
        assert sm.lock() == LockVerdict.LATCHED
        assert s.torque_log == [] and s.goal == 1569

    def test_hardware_yaml_uses_the_45_degree_position_lock(self):
        from inchworm.config import load_config
        gl = load_config().gripper_lock
        for arm, open_end in (("arm_left", 1526), ("arm_right", 1529)):
            b = getattr(gl, arm)
            assert b.lock_mode == "position"
            assert round(b.lock_goal_deg * 4096 / 360) == open_end + 512   # 45 deg
            assert b.locked_low_deg <= b.lock_goal_deg <= b.locked_high_deg
