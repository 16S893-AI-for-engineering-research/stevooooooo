"""AA-43 #4: direction_check.py logic (no hardware)."""
import importlib.util
import pathlib

import pytest

from inchworm.config import load_config

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "direction_check.py"
spec = importlib.util.spec_from_file_location("dc", _P)
dc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dc)

REST_L = [1948, 1425, 3119, 1006, 2035, 2520]
REST_R = [2089, 1490, 3078, 920, 1719, 2494]


def test_convention_rule():
    assert dc.convention_matches(+57, "o") is True       # opposite the cable: clockwise = ticks UP
    assert dc.convention_matches(-57, "o") is False
    assert dc.convention_matches(-57, "c") is True       # cable side: clockwise = ticks DOWN
    assert dc.convention_matches(+57, "c") is False


def test_model_says_fewer_wrist_flex_ticks_raise_the_tip_away_from_the_dexarm():
    d = dc.model_tip_motion(load_config(), "left", "wrist_flex", REST_L, REST_R, -57)
    # Owner-observed direction (2026-11-10): ticks DOWN = tip AWAY from the DexArm (+Y).
    # The vertical component depends on the pose and the elbow zero (2026-11-12 landmark
    # zero: only ~0.6 mm up), so only the observed horizontal direction is asserted
    # (owner approved 2026-11-12).
    assert d[1] > 1.0 and "AWAY" in dc.tip_words(d)
    d2 = dc.model_tip_motion(load_config(), "left", "wrist_flex", REST_L, REST_R, +57)
    assert d2[1] < -1.0 and "TOWARD" in dc.tip_words(d2)


class _Raw:
    def __init__(self, seq):
        self.seq = list(seq)

    def read_bytes(self, sid, reg, n):
        assert reg == 56
        return self.seq[0][sid - 1].to_bytes(2, "little")

    def advance(self):
        self.seq.pop(0)


def test_run_records_direction_and_never_writes(tmp_path):
    class Raw(_Raw):
        def write_bytes(self, *a):
            raise AssertionError("read-only check must not write")
    a = Raw([REST_L])
    answers = iter(["o", "", "y"])                       # side, move done, tip matches
    def prompt(msg):
        v = next(answers)
        if msg.startswith("Now move"):
            a.seq = [[REST_L[0], REST_L[1], REST_L[2], REST_L[3] - 60, REST_L[4], REST_L[5]]]
        return v
    recs = dc.run(a, _Raw([REST_R]), load_config(), "left", ["wrist_flex"], prompt=prompt, out=tmp_path / "o.jsonl")
    r = recs[0]
    assert r["delta_ticks"] == -60 and r["your_clockwise_is"] == "ticks DOWN"
    assert r["matches_doc1_convention"] is False         # stood opposite the cable, saw clockwise on ticks DOWN
    assert "AWAY" in r["model_says_tip_goes"] and r["real_tip_matches_model"] is True
    assert (tmp_path / "o.jsonl").read_text().count("\n") == 1


def test_small_movement_is_not_counted():
    a = _Raw([REST_L])
    answers = iter(["c", ""])
    recs = dc.run(a, _Raw([REST_R]), load_config(), "left", ["wrist_flex"], prompt=lambda m: next(answers))
    assert recs[0]["result"] == "NO_MOVEMENT_DETECTED"


def test_cli_dry_run_and_bad_joint(capsys):
    assert dc.main(["--arm", "left"]) == 0 and "DRY-RUN" in capsys.readouterr().out
    assert dc.main(["--arm", "left", "--joints", "gripper"]) == 2


def test_view_joint_move_renders_and_states_the_sense(tmp_path, capsys):
    spec = importlib.util.spec_from_file_location("vjm", _P.parent / "view_joint_move.py")
    vjm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vjm)
    png = tmp_path / "m.png"
    assert vjm.main(["--arm", "left", "--joint", "wrist_flex", "--delta-deg", "10", "--png", str(png)]) == 0
    out = capsys.readouterr().out
    assert png.stat().st_size > 10_000
    assert "ticks DOWN = CLOCKWISE seen from the cable side (+X), COUNTER-clockwise seen from the opposite side (-X)" in out
