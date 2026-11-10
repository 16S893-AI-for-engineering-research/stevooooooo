"""AA-43: home_servos.py against a register-memory fake (no hardware)."""
import importlib.util
import pathlib

import pytest

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "home_servos.py"
spec = importlib.util.spec_from_file_location("hs", _P)
hs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hs)


class FakeServos:
    """STS3215-like memory: present = actual - homing_offset; EEPROM honours lock."""

    def __init__(self, actual):
        self.actual = dict(actual)
        self.mem = {sid: [0] * 71 for sid in range(1, 7)}
        for sid in self.mem:
            self.mem[sid][5] = sid
            self.mem[sid][55] = 1
            self.mem[sid][11], self.mem[sid][12] = 0xFF, 0x0F
            self.mem[sid][31] = 85  # factory-ish offset
        self.writes = []

    def _present(self, sid):
        off = hs.decode_offset(self.mem[sid][31] | (self.mem[sid][32] << 8))
        return (self.actual[sid] - off) % 4096

    def read(self, sid, addr, n):
        p = self._present(sid)
        self.mem[sid][56], self.mem[sid][57] = p & 0xFF, p >> 8
        return bytes(self.mem[sid][addr:addr + n])

    def write(self, sid, addr, data):
        self.writes.append((sid, addr, bytes(data)))
        if addr != 55 and addr < 40 and self.mem[sid][55] == 1:
            return
        for i, b in enumerate(data):
            self.mem[sid][addr + i] = b


def test_offset_encoding_sign_magnitude():
    assert hs.encode_offset(-1352) == 3400  # left ID1 2026-10-24: raw 3400 = -1352
    assert hs.decode_offset(3400) == -1352
    assert hs.encode_offset(384) == 384
    with pytest.raises(ValueError):
        hs.encode_offset(2048)


def test_homing_centres_targets_and_relocks():
    f = FakeServos({1: 100, 2: 200, 3: 300, 4: 368, 5: 3557, 6: 3639})
    before = hs.dump(f.read, range(1, 7))
    res = hs.do_homing(f.read, f.write, [4, 5, 6], settle_s=0)
    for sid in (4, 5, 6):
        assert abs(res[sid]["position_after"] - 2047) <= 1
        assert f.mem[sid][55] == 1
    after = hs.dump(f.read, range(1, 7))
    written = {sid: {31, 32} for sid in (4, 5, 6)}
    assert hs.unexpected_changes(before, after, written) == []
    assert not any(w[0] in (1, 2, 3) for w in f.writes)


def _ranges(j, sid, start, lo, hi, wrapped=False, seam=False):
    return {"joints": {j: {"servo_id": sid, "start_ticks": start, "travel_lo_ticks_rel_start": lo,
                           "travel_hi_ticks_rel_start": hi, "wrapped": wrapped, "crosses_seam": seam}}}


JM = dict(zip(hs.JOINTS, range(1, 7)))


def test_plan_limits_adds_margin_and_refuses_bad_sweeps():
    assert hs.plan_limits(_ranges("wrist_flex", 4, 2047, -1100, 1100), JM, [4], 30) == {4: (917, 3177)}
    with pytest.raises(SystemExit):
        hs.plan_limits(_ranges("wrist_flex", 4, 2047, -1100, 1100, wrapped=True), JM, [4], 30)
    with pytest.raises(SystemExit):
        hs.plan_limits(_ranges("wrist_flex", 4, 2047, -2040, 100), JM, [4], 30)  # below 0
    with pytest.raises(SystemExit):
        hs.plan_limits(_ranges("wrist_flex", 4, 2047, -5, 5), JM, [4], 30)  # not swept
    with pytest.raises(SystemExit):
        hs.plan_limits(_ranges("wrist_flex", 5, 2047, -1100, 1100), JM, [4], 30)  # ID mismatch


def test_limits_written_and_read_back():
    f = FakeServos({sid: 2047 for sid in range(1, 7)})
    res = hs.do_limits(f.read, f.write, {4: (917, 3177)}, settle_s=0)
    assert res[4]["read_back"] == [917, 3177] and f.mem[4][55] == 1


def test_dry_run_opens_nothing(capsys):
    assert hs.main(["homing", "--arm", "left", "--ids", "4,5,6"]) == 0
    assert "DRY-RUN" in capsys.readouterr().out


def test_rejects_ids_outside_arm():
    assert hs.main(["homing", "--arm", "left", "--ids", "4,7"]) == 4


def test_homing_to_a_known_end_stop_reading():
    f = FakeServos({4: 1000, 5: 1000, 6: 3642, 1: 0, 2: 0, 3: 0})
    res = hs.do_homing(f.read, f.write, [6], settle_s=0, readings={6: 2521})
    assert res[6]["wanted_reading"] == 2521
    assert abs(f.read(6, 56, 2)[0] + (f.read(6, 56, 2)[1] << 8) - 2521) <= 1
    assert f.mem[6][55] == 1
    assert not any(w[0] != 6 for w in f.writes)


def test_shortest_offset_wraps_across_the_seam():
    assert hs.shortest_offset(100, 2047) == -1947
    assert hs.shortest_offset(4000, 100) == -196
    assert hs.shortest_offset(3642, 2521) == 1121
    assert -2048 <= hs.shortest_offset(0, 4095) <= 2047


def test_cli_reading_and_set_limits_validation(capsys):
    assert hs.main(["homing", "--arm", "left", "--ids", "6", "--reading", "5=100"]) == 4
    assert hs.main(["homing", "--arm", "left", "--ids", "6", "--reading", "6=9999"]) == 4
    assert hs.main(["limits", "--arm", "left", "--ids", "6", "--reading", "6=100"]) == 4
    assert hs.main(["homing", "--arm", "left", "--ids", "6", "--set-limits", "6=1:2"]) == 4
    assert hs.main(["limits", "--arm", "left", "--ids", "6", "--set-limits", "6=2531:1521"]) == 4
    assert hs.main(["limits", "--arm", "left", "--ids", "4,6", "--set-limits", "6=1521:2531"]) == 4  # 4 has no source
    capsys.readouterr()
    assert hs.main(["homing", "--arm", "left", "--ids", "6", "--reading", "6=2521"]) == 0
    assert "so the CURRENT pose reads 2521" in capsys.readouterr().out
    assert hs.main(["limits", "--arm", "left", "--ids", "6", "--set-limits", "6=1521:2531"]) == 0
    assert "1521..2531" in capsys.readouterr().out
