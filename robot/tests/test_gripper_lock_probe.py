"""AA-17 gripper_lock_probe.py: dry-run and trial logic (no hardware)."""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from inchworm.config import GripperBandConfig
from inchworm.gripper_lock import LockVerdict, LockVerificationFailed

_SPEC = importlib.util.spec_from_file_location(
    "gripper_lock_probe", Path(__file__).parents[1] / "scripts" / "gripper_lock_probe.py")
probe = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(probe)

_BANDS = GripperBandConfig(
    unlocked_low_deg=132.0, unlocked_high_deg=141.0, engaged_low_deg=132.0,
    engaged_high_deg=141.0, locked_low_deg=156.0, locked_high_deg=160.6,
    lock_overtravel_goal_deg=191.0, closed_empty_deg=221.6,
)


class _SM:
    def __init__(self):
        self.calls = []

    def reset(self):
        self.calls.append("reset")

    def set_approach_ready(self):
        self.calls.append("approach")

    def lock(self):
        self.calls.append("lock")
        raise LockVerificationFailed(LockVerdict.UNVERIFIED, "thresholds null")


class _Bus:
    def __init__(self, positions):
        self._p = list(positions)

    def _servo_id_for_joint(self, name):
        assert name == "gripper"
        return 6

    def read_joint_state(self, sid):
        return SimpleNamespace(position_deg=self._p.pop(0), load_raw=120, current_raw=40,
                               speed_raw=0, temperature_c=30)


def test_classify():
    assert probe.classify(158.0, _BANDS) == "in_locked_band"
    assert probe.classify(191.0, _BANDS) == "overtravel_past_locked_band"
    assert probe.classify(140.0, _BANDS) == "short_of_locked_band"


def test_trials_go_through_the_state_machine_and_reopen():
    sm, bus = _SM(), _Bus([158.2, 158.0])
    recs = probe.run_trials(bus, sm, _BANDS, role="arm_left", torque_raw=200, key=True,
                            trials=2, prompt=lambda _: "", pause_s=0)
    assert sm.calls == ["reset", "approach", "lock", "reset"] * 2
    assert [r["classification"] for r in recs] == ["in_locked_band"] * 2
    assert recs[0]["sm_outcome"].startswith("LockVerificationFailed")
    assert recs[0]["torque_limit_raw"] == 200 and recs[0]["position_ticks"] == 1800


def test_quit_before_lock_commands_nothing_further():
    sm, bus = _SM(), _Bus([])
    recs = probe.run_trials(bus, sm, _BANDS, role="arm_left", torque_raw=200, key=False,
                            trials=3, prompt=lambda _: "q", pause_s=0)
    assert recs == [] and "lock" not in sm.calls


def test_cli_dry_run_touches_nothing(capsys):
    assert probe.main(["--arm", "left", "--torque", "200", "--key"]) == 0
    out = capsys.readouterr().out
    assert "DRY-RUN" in out and "191.07" in out


@pytest.mark.parametrize("t", [0, 49, 501, 1000])
def test_cli_refuses_out_of_range_torque(t):
    assert probe.main(["--arm", "left", "--torque", str(t), "--key"]) == 2


def test_drive_config_faults_flags_zeroed_eeprom():
    """AA-41 finding: Max_Torque/P/Protection_Current = 0 means no torque; the probe must refuse."""
    import importlib.util, pathlib
    p = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "gripper_lock_probe.py"
    spec = importlib.util.spec_from_file_location("glp_drive", p)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    zeroed = lambda sid, addr, n: bytes(n)                      # as read live on IDs 4-6
    healthy = {16: b"\xe8\x03", 21: b"\x20", 28: b"\x36\x01", 48: b"\xe8\x03"}  # as read on IDs 1-3
    assert set(x.split("(")[0] for x in m.drive_config_faults(zeroed, 6)) == set(m.DRIVE_REGS)
    assert m.drive_config_faults(lambda sid, addr, n: healthy[addr], 6) == []


def test_probe_cleanup_restores_the_eeprom_cap_not_a_hardcoded_1000():
    """Owner 2026-11-09: grippers use LeRobot's lower torque cap (500); the probe must not undo it."""
    src = (Path(__file__).parents[1] / "scripts" / "gripper_lock_probe.py").read_text()
    assert "restore_torque" in src and "bus.set_torque_limit(sid, _TORQUE_LIMIT_FULL)" not in src


class _TimedBus:
    """Positions read in order; the clock advances 0.1 s per read."""

    def __init__(self, ticks, loads=None):
        self._t = list(ticks)
        self._l = list(loads or [10] * len(ticks))
        self.now = 0.0

    def read_joint_state(self, sid):
        self.now += 0.1
        t = self._t.pop(0) if len(self._t) > 1 else self._t[0]
        l = self._l.pop(0) if len(self._l) > 1 else self._l[0]
        return SimpleNamespace(position_deg=t * 360 / 4096, load_raw=l, current_raw=0, speed_raw=0,
                               temperature_c=30)


def test_wait_settled_waits_for_the_slow_move_to_stop():
    bus = _TimedBus([1600, 1700, 1790, 1800, 1801, 1801, 1801, 1801, 1801, 1801, 1801], [50, 80, 290, 120])
    st, settled, peak = probe.wait_settled(bus, 6, settle_s=0.5, timeout_s=5, sleep=lambda _: None,
                                           clock=lambda: bus.now)
    assert settled and round(st.position_deg * 4096 / 360) == 1801 and peak == 290


def test_wait_settled_times_out_while_moving():
    bus = _TimedBus(list(range(1500, 2500, 5)))
    st, settled, _ = probe.wait_settled(bus, 6, settle_s=0.5, timeout_s=2, sleep=lambda _: None,
                                        clock=lambda: bus.now)
    assert settled is False


def test_unsettled_trial_is_not_classified():
    sm, bus = _SM(), _Bus([])
    calls = []

    def settle(b, sid):
        calls.append(sid)
        return SimpleNamespace(position_deg=158.0, load_raw=0, current_raw=0, speed_raw=0, temperature_c=30), \
            len(calls) != 2, 99

    recs = probe.run_trials(bus, sm, _BANDS, role="arm_left", torque_raw=300, key=True, trials=1,
                            prompt=lambda _: "", pause_s=0, settle=settle)
    assert len(calls) == 3                      # after open, after lock, after re-open
    assert recs[0]["classification"] == "not_settled" and recs[0]["settled"] is False


def test_probe_sets_slow_speed_before_torque_and_restores_it():
    src = (Path(__file__).parents[1] / "scripts" / "gripper_lock_probe.py").read_text()
    i_speed = src.index("bus.set_goal_speed(sid, _PROBE_SPEED_TICKS_S)")
    i_torque = src.index("bus.set_torque_enable(sid, True)")
    assert i_speed < i_torque
    assert "bus.set_goal_speed(sid, restore_speed)" in src and probe._TORQUE_MAX == 500
