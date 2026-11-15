"""AA-23 / AA-44: scripts/hold_tube.py against simulated servos (no hardware)."""
import importlib.util
import pathlib
from types import SimpleNamespace

import pytest

from inchworm.config import load_config

ROOT = pathlib.Path(__file__).parents[1]


def _load(name, rel):
    s = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m


sim = _load("replay_sim", "scripts/replay_sim.py")
ht = _load("hold_tube", "scripts/hold_tube.py")
CFG = load_config()
OPEN_R = 1546                      # inside the right open band 1504..1589
POSE = [2100, 1400, 3000, 1500, 2000]


class Bus(sim.SimArmBus):
    def _read_bytes(self, sid, reg, n):
        if reg in (16, 21, 28, 48):
            return bytes([100, 0])[:n]
        return super()._read_bytes(sid, reg, n)


def _bus(clock, grip=OPEN_R, stall=None):
    lim = CFG.motion_guard.right_arm_limits.servo_tick_limits
    b = Bus("arm_right", POSE + [grip], clock, stall=stall,
            limits={i + 1: tuple(lim[j]) for i, j in enumerate((*ht.J5, "gripper"))})
    b._bus.read_bytes = b._read_bytes
    return b


def _run(argv, bus, clock, answers=("YES",)):
    a = ht.argparse.Namespace(action=argv[0], arm="right", live=True, tube_in_gripper="--tube-in-gripper" in argv,
                              no_tube="--no-tube" in argv, gripper_off="--gripper-off" in argv)
    it = iter(answers)
    logs = []
    rc = ht.run(a, open_bus=lambda role: (bus, lambda: None), cfg=CFG, prompt=lambda *_: next(it),
                log=logs.append, clock=clock, sleep=clock.sleep)
    return rc, logs


def test_lock_holds_pose_and_locks_gripper():
    clock = sim.SimClock()
    bus = _bus(clock)
    rc, logs = _run(["lock", "--tube-in-gripper"], bus, clock)
    assert rc == 0 and logs[-1]["event"] == "locked" and logs[-1]["gripper_band"] == "locked"
    assert all(bus.torque[s] for s in range(1, 7))                   # holding, gripper on
    assert [round(bus.pos[s]) for s in range(1, 6)] == POSE          # nothing moved
    assert abs(bus.pos[6] - 2041) <= 12


def test_locked_arm_then_passes_the_plugin_style_check():
    clock = sim.SimClock()
    bus = _bus(clock)
    _run(["lock", "--no-tube"], bus, clock)
    st = ht.read_state(bus._read_bytes, CFG.gripper_lock.arm_right)
    assert st["gripper_band"] == "locked" and all(st["torque"].values())


@pytest.mark.parametrize("mutate, needle", [
    (lambda b: b.torque.__setitem__(2, True), "torque is ON"),
    (lambda b: b.pos.__setitem__(6, 2400.0), "not in the open band"),
    (lambda b: b.pos.__setitem__(3, 4000.0), "outside EEPROM"),
])
def test_lock_refusals_energise_nothing(mutate, needle):
    clock = sim.SimClock()
    bus = _bus(clock)
    mutate(bus)
    before = dict(bus.torque)
    rc, logs = _run(["lock", "--no-tube"], bus, clock)
    assert rc == 4 and needle in logs[-1]["reason"] and bus.torque == before


def test_not_confirmed_energises_nothing():
    clock = sim.SimClock()
    bus = _bus(clock)
    rc, _ = _run(["lock", "--no-tube"], bus, clock, answers=("no",))
    assert rc == 0 and not any(bus.torque.values())


def test_obstructed_lock_faults_gripper_off_then_releases_joints():
    clock = sim.SimClock()
    bus = _bus(clock, stall={6: 1800})                  # something stops the gripper short
    rc, logs = _run(["lock", "--no-tube"], bus, clock, answers=("YES", ""))
    assert rc == 7 and any(l["event"] == "lock_failed" for l in logs)
    assert not any(bus.torque.values())


def test_release_keeps_gripper_locked_unless_asked():
    clock = sim.SimClock()
    bus = _bus(clock)
    _run(["lock", "--no-tube"], bus, clock)
    rc, logs = _run(["release"], bus, clock)
    assert rc == 0 and logs[-1]["joints_off"] == [1, 2, 3, 4, 5]
    assert bus.torque[6] and not any(bus.torque[s] for s in range(1, 6))
    _run(["release", "--gripper-off"], bus, clock)
    assert not bus.torque[6]


def test_cli_dry_run_and_statement_required(capsys):
    assert ht.main(["lock"]) == 2
    assert ht.main(["lock", "--no-tube"]) == 0 and "DRY-RUN" in capsys.readouterr().out
    assert ht.main(["status"]) == 0
