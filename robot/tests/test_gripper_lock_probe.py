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


@pytest.mark.parametrize("t", [0, 49, 601, 1000])
def test_cli_refuses_out_of_range_torque(t):
    assert probe.main(["--arm", "left", "--torque", str(t), "--key"]) == 2
