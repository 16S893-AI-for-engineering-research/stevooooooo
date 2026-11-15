"""AA-44: leader_mapping.py fit logic and read-only refusals (no hardware)."""
import importlib.util
import json
import pathlib

import pytest

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "leader_mapping.py"
spec = importlib.util.spec_from_file_location("lm", _P)
lm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lm)

SIGNS = [1, -1, 1, -1, 1]
OFFS = [100, 3000, -50, 4000, 2047]


def _cap(label, leader, noise=(0, 0, 0, 0, 0)):
    fol = [(s * l + o + n) % 4096 for s, l, o, n in zip(SIGNS, leader, OFFS, noise)]
    return {"event": "capture", "label": label, "leader_ticks": list(leader) + [2000], "follower_ticks": fol + [1500]}


POSES = [_cap("P1", [2000, 1500, 3000, 1000, 2047]),
         _cap("P2", [2400, 1900, 2600, 1400, 2600], noise=(3, -4, 2, 0, 5)),
         _cap("P3", [1700, 1200, 3300, 800, 1500], noise=(-2, 3, -1, 4, -3))]


def test_fit_recovers_sign_and_offset_with_wrap():
    fit = lm.fit_mapping(POSES)
    assert fit["problems"] == []
    for i, j in enumerate(lm.J5):
        r = fit["joints"][j]
        assert r["sign"] == SIGNS[i]
        assert abs(lm.wrap(r["offset_ticks"] - OFFS[i])) < 6
        assert r["worst_deg"] < 1.0


def test_predict_matches_follower_and_excludes_gripper():
    fit = lm.fit_mapping(POSES)
    pred = lm.predict_follower(fit, POSES[0]["leader_ticks"])
    assert len(pred) == 5   # joint 6 never mapped
    for p, f in zip(pred, POSES[0]["follower_ticks"][:5]):
        assert abs(lm.wrap(p - f)) < 6


def test_small_span_flags_sign_not_determined():
    caps = [_cap("A", [2000] * 5), _cap("B", [2050, 2050, 2050, 2050, 2050])]  # ~4.4 deg
    fit = lm.fit_mapping(caps)
    assert all(not r["sign_determined"] for r in fit["joints"].values())
    assert len(fit["problems"]) >= 5


def test_large_residual_is_a_problem():
    bad = POSES[:2] + [_cap("P3", [1700, 1200, 3300, 800, 1500], noise=(200, 0, 0, 0, 0))]
    fit = lm.fit_mapping(bad)
    assert any(p.startswith("shoulder_pan") for p in fit["problems"])


def test_needs_two_poses():
    with pytest.raises(lm.MappingError):
        lm.fit_mapping(POSES[:1])


def test_fit_cli_saves_unverified_and_refuses_on_problems(tmp_path):
    log = tmp_path / "leader_map_left.jsonl"
    log.write_text("\n".join(json.dumps(c) for c in POSES) + "\n")
    assert lm.main(["fit", "--follower", "left", "--dir", str(tmp_path)]) == 0
    saved = json.loads((tmp_path / "leader_map_left.json").read_text())
    assert saved["unverified"] is True and saved["follower"] == "left"
    log.write_text("\n".join(json.dumps(c) for c in [_cap("A", [2000] * 5), _cap("B", [2010] * 5)]) + "\n")
    (tmp_path / "leader_map_left.json").unlink()
    assert lm.main(["fit", "--follower", "left", "--dir", str(tmp_path)]) == 3
    assert not (tmp_path / "leader_map_left.json").exists()


def test_recaptured_label_replaces_earlier(tmp_path):
    log = tmp_path / "x.jsonl"
    old = _cap("P1", [100] * 5)
    log.write_text("\n".join(json.dumps(c) for c in [old] + POSES) + "\n")
    caps = lm.load_captures(log, ["P1", "P2"])
    assert caps[0]["leader_ticks"] == POSES[0]["leader_ticks"]


def test_capture_dry_run_opens_nothing(capsys):
    assert lm.main(["capture", "--follower", "left", "--label", "P1"]) == 0
    assert "DRY-RUN" in capsys.readouterr().out


class _Raw:
    def __init__(self, torque):
        self.torque = torque
        self.writes = 0

    def read_bytes(self, sid, addr, n):
        if addr == lm.REG_TORQUE_ENABLE:
            return bytes([self.torque])
        return (2047).to_bytes(2, "little")

    read_bytes_tolerating = read_bytes

    def write_bytes(self, *a):  # pragma: no cover - must never be called
        self.writes += 1


def test_follower_read_refuses_torque_on_and_never_writes():
    r = _Raw(torque=1)
    with pytest.raises(lm.MappingError):
        lm.read_follower(r)
    r0 = _Raw(torque=0)
    assert lm.read_follower(r0) == [2047] * 6
    assert lm.read_leader(r0) == [2047] * 6
    assert r.writes == 0 and r0.writes == 0
