"""AA-41: restore_drive_config.py logic against a register-memory fake (no hardware)."""
import importlib.util
import pathlib

import pytest

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "restore_drive_config.py"
spec = importlib.util.spec_from_file_location("rdc", _P)
rdc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rdc)

# Register values as read live 2026-10-27 (ID 3 healthy, ID 4-6 zeroed).
HEALTHY = {13: 70, 16: 232, 17: 3, 19: 44, 20: 47, 21: 32, 22: 32, 23: 0, 24: 16, 25: 0,
           26: 1, 27: 1, 28: 54, 29: 1, 34: 20, 35: 200, 36: 80, 37: 10, 38: 200, 39: 200}
ZEROED = {13: 0, 16: 0, 17: 0, 19: 1, 20: 1, 21: 0, 22: 0, 23: 0, 24: 0, 25: 0,
          26: 0, 27: 0, 28: 0, 29: 0, 34: 0, 35: 0, 36: 0, 37: 0, 38: 0, 39: 0}


class FakeBus:
    def __init__(self, lock_honoured=True):
        self.mem = {sid: [0] * 71 for sid in range(1, 7)}
        for sid in range(1, 7):
            for a, v in (HEALTHY if sid <= 3 else ZEROED).items():
                self.mem[sid][a] = v
            self.mem[sid][5] = sid
            self.mem[sid][55] = 1
        self.lock_honoured = lock_honoured
        self.writes = []

    def read(self, sid, addr, n):
        return bytes(self.mem[sid][addr:addr + n])

    def write(self, sid, addr, data):
        self.writes.append((sid, addr, bytes(data)))
        if addr != 55 and addr < 40 and self.lock_honoured and self.mem[sid][55] == 1:
            return  # EEPROM locked: write ignored
        for i, b in enumerate(data):
            self.mem[sid][addr + i] = b


def test_plan_copies_id3_and_caps_gripper():
    b = FakeBus()
    t = rdc.plan(b.read)
    assert t[4]["max_torque_limit"] == 1000 and t[4]["p_coefficient"] == 32
    assert t[4]["protection_current"] == 310 and t[5] == t[4]
    assert t[6]["max_torque_limit"] == 500 and t[6]["protection_current"] == 250
    assert t[6]["overload_torque"] == 25 and t[6]["p_coefficient"] == 16 and t[6]["d_coefficient"] == 32
    assert t[6]["max_temperature"] == 70


def test_apply_unlocks_writes_relocks_and_reads_back():
    b = FakeBus()
    bad = rdc.apply(b.read, b.write, rdc.plan(b.read), settle_s=0)
    assert bad == {}
    assert rdc.drive_values(b.read, 6)["max_torque_limit"] == 500
    assert all(b.mem[sid][55] == 1 for sid in (4, 5, 6))
    for sid in (4, 5, 6):
        ws = [w for w in b.writes if w[0] == sid]
        assert ws[0][1:] == (55, b"\x00") and ws[-1][1:] == (55, b"\x01")
    assert not any(w[0] in (1, 2, 3) for w in b.writes)          # references never written
    written = {w[1] for w in b.writes}
    assert not written & {5, 6, 9, 10, 11, 12, 18, 31, 32, 33}   # id, baud, limits, phase, offset, mode


def test_apply_reports_mismatch_when_writes_do_not_stick():
    class Stubborn(FakeBus):
        def write(self, sid, addr, data):
            self.writes.append((sid, addr, bytes(data)))
            if addr == 55:
                self.mem[sid][55] = data[0]
    b = Stubborn()
    bad = rdc.apply(b.read, b.write, rdc.plan(b.read), settle_s=0)
    assert set(bad) == {4, 5, 6} and bad[6]["max_torque_limit"] == (500, 0)


def test_plan_refuses_when_references_disagree_or_zeroed():
    b = FakeBus()
    b.mem[2][21] = 16
    with pytest.raises(SystemExit):
        rdc.plan(b.read)
    b = FakeBus()
    for sid in (1, 2, 3):
        for a, v in ZEROED.items():
            b.mem[sid][a] = v
    with pytest.raises(SystemExit):
        rdc.plan(b.read)


def test_fw310_mask_values_exclude_overcurrent_and_overheat():
    m = rdc.FW310_MASK
    assert m["unloading_condition"] == 33 and m["led_alarm_condition"] == 35
    for v in m.values():
        assert not v & 8 and not v & 4      # overcurrent, overheat masked
        assert v & 1 and v & 32             # voltage, overload kept


def test_fw310_mask_apply_only_touches_two_registers():
    b = FakeBus()
    bad = rdc.apply(b.read, b.write, {sid: dict(rdc.FW310_MASK) for sid in (4, 5, 6)}, settle_s=0)
    assert bad == {}
    assert {w[1] for w in b.writes} == {55, 19, 20}
    assert b.mem[4][19] == 33 and b.mem[6][20] == 35 and b.mem[3][19] == 44
