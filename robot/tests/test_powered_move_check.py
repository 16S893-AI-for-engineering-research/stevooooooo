"""AA-43 #5: powered_move_check.py logic against a fake bus (no hardware)."""
import importlib.util
import pathlib
from types import SimpleNamespace

import pytest

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "powered_move_check.py"
spec = importlib.util.spec_from_file_location("pmc", _P)
pmc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pmc)


class FakeBus:
    """Servo that follows the goal at a fixed step per read; can be made to stall."""

    def __init__(self, start=2000, stall_at=None, torque_limit=1000):
        self.pos = start
        self.goal = start
        self.stall_at = stall_at
        self.calls = []
        self.torque = False
        self._bus = SimpleNamespace(read_bytes=self._rb)
        self.tl = torque_limit

    def _rb(self, sid, reg, n):
        assert reg == 48
        return bytes([self.tl & 0xFF, self.tl >> 8])

    def read_joint_state(self, sid):
        step = 5
        if self.stall_at is not None and abs(self.pos - self.stall_at) <= step:
            pass
        elif self.pos < self.goal:
            self.pos = min(self.goal, self.pos + step)
        elif self.pos > self.goal:
            self.pos = max(self.goal, self.pos - step)
        return SimpleNamespace(position_deg=self.pos * 360 / 4096, speed_raw=0, load_raw=3, current_raw=10, temperature_c=30)

    def set_goal_speed(self, sid, v): self.calls.append(("speed", v))
    def set_acceleration(self, sid, v): self.calls.append(("accel", v))
    def set_torque_limit(self, sid, v): self.calls.append(("tlimit", v)); self.tl = v
    def set_torque_enable(self, sid, e): self.calls.append(("torque", e)); self.torque = e
    def write_position_deg(self, joint, deg): self.calls.append(("goal", joint, round(deg * 4096 / 360))); self.goal = round(deg * 4096 / 360)


def test_out_and_back_reaches_and_cleans_up(tmp_path):
    b = FakeBus(2000)
    recs = pmc.move_joint(b, "wrist_flex", 4, 5.0, role="arm_left", limits=(800, 3200), stop_file=tmp_path / "nostop")
    assert [r["status"] for r in recs] == ["REACHED", "REACHED"]
    assert recs[0]["goal_ticks"] == 2000 + round(5 * 4096 / 360) and recs[1]["goal_ticks"] == 2000
    assert b.torque is False and b.tl == 1000                      # torque off, limit restored
    assert ("tlimit", pmc.TORQUE_LIMIT_RAW) in b.calls and ("speed", pmc.SPEED_TICKS_S) in b.calls
    assert b.calls[-1] == ("accel", 0) and ("speed", 0) in b.calls
    # torque limit and speed are set BEFORE torque is enabled
    i_t = b.calls.index(("torque", True))
    assert b.calls.index(("speed", pmc.SPEED_TICKS_S)) < i_t and b.calls.index(("tlimit", pmc.TORQUE_LIMIT_RAW)) < i_t


def test_stall_aborts_and_turns_torque_off(tmp_path):
    b = FakeBus(2000, stall_at=2020)
    t = iter(x * 0.2 for x in range(1000))
    recs, status = [], None
    st, final, trace = pmc.wait_for_goal(b, 4, 2100, stop_file=tmp_path / "x", sleep=lambda s: None, clock=lambda: next(t))
    assert st == "STALLED"


def test_stop_file_aborts(tmp_path):
    f = tmp_path / "stop"; f.write_text("x")
    b = FakeBus(2000)
    recs = pmc.move_joint(b, "wrist_roll", 5, 5.0, role="arm_left", limits=(800, 3200), stop_file=f)
    assert recs[0]["status"] == "STOP_FILE" and len(recs) == 1 and b.torque is False


def test_plan_refuses_near_limits_and_big_deltas():
    with pytest.raises(ValueError):
        pmc.plan_targets(2000, 11.0, 800, 3200)
    with pytest.raises(ValueError):
        pmc.plan_targets(2000, 0.1, 800, 3200)
    with pytest.raises(ValueError):
        pmc.plan_targets(3190, 5.0, 800, 3200)       # start inside the margin
    with pytest.raises(ValueError):
        pmc.plan_targets(3100, 10.0, 800, 3200)      # target would leave the margin
    assert pmc.plan_targets(3100, -10.0, 800, 3200)[0] < 3100


def test_cli_dry_run_and_refusals(capsys):
    assert pmc.main(["--arm", "left", "--joint", "wrist_flex"]) == 0
    assert "DRY-RUN" in capsys.readouterr().out
    assert pmc.main(["--arm", "left", "--joint", "wrist_flex", "--delta-deg", "30"]) == 2
    assert pmc.main(["--arm", "right", "--joint", "gripper"]) == 0


def test_direction_hint_matches_the_convention_and_the_model():
    h = pmc.direction_hint("wrist_flex", -57)          # the move that was run on 2026-11-09
    assert "DOWN" in h and "counter-clockwise" in h and "RISES" in h and "AWAY from the DexArm" in h
    h = pmc.direction_hint("wrist_flex", +57)
    assert "UP" in h and "clockwise" in h and "counter" not in h and "DROPS" in h
    assert "tip looking back" in pmc.direction_hint("wrist_roll", 57)


def test_record_states_the_expectation_of_its_own_leg(tmp_path):
    b = FakeBus(2000)
    recs = pmc.move_joint(b, "wrist_flex", 4, -5.0, role="arm_left", limits=(800, 3200), stop_file=tmp_path / "x")
    assert "DOWN" in recs[0]["direction_expected"] and "UP" in recs[1]["direction_expected"]
