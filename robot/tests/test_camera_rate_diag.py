"""camera_rate_diag.py: dry-run on the mock camera, and the verdict logic (no hardware)."""
import importlib.util
import json
import pathlib

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "camera_rate_diag", pathlib.Path(__file__).resolve().parents[1] / "scripts" / "camera_rate_diag.py")
diag = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(diag)


def _v(raw=None, new=None, prod=None, loop=None):
    row = {"prod": {"new_per_s": prod}, "loop": {"new_per_s": loop}}
    if raw is not None:
        row["raw"] = {"grab_per_s": raw}
        row["decode"] = {"new_per_s": new}
    return row


def _res(**variants):
    return {"variants": variants}


def test_dry_run_writes_report(tmp_path, monkeypatch):
    import inchworm.camera_arbiter as ca
    out = tmp_path / "r.json"
    orig = ca.CameraArbiter.__init__

    def init(self, backend, *, lock_dir=tmp_path / "locks"):
        orig(self, backend, lock_dir=lock_dir)
    monkeypatch.setattr(ca.CameraArbiter, "__init__", init)
    assert diag.main(["--seconds", "0.05", "--out", str(out)]) == 0
    rep = json.loads(out.read_text())
    assert set(rep["variants"]) == {v[0] for v in diag.VARIANTS}
    assert rep["live"] is False and "verdict" in rep
    assert all("prod" in r and "loop" in r for r in rep["variants"].values())


def test_verdict_exposure():
    r = _res(fps30_auto=_v(10, 10, 10, 10), fps60_auto=_v(10, 10, 10, 10),
             fps60_exp10ms=_v(55, 55, 50, 30), fps60_exp5ms=_v(58, 58, 50, 30))
    assert diag.verdict(r).startswith("EXPOSURE")


def test_verdict_transport():
    r = _res(fps30_auto=_v(9, 9, 9, 9), fps60_auto=_v(10, 10, 9, 9),
             fps60_exp10ms=_v(11, 11, 10, 9), fps60_exp5ms=_v(11, 11, 10, 9))
    assert diag.verdict(r).startswith("TRANSPORT")


def test_verdict_duplicates():
    r = _res(fps30_auto=_v(30, 10, 10, 10), fps60_auto=_v(60, 20, 20, 10))
    assert diag.verdict(r).startswith("DUPLICATE")


def test_verdict_reject():
    r = _res(fps30_auto=_v(30, 30, 12, 10), fps60_auto=_v(60, 60, 20, 10))
    assert diag.verdict(r).startswith("OUR DECODE")


def test_verdict_loop():
    r = _res(fps30_auto=_v(30, 30, 29, 9), fps60_auto=_v(60, 60, 58, 9))
    assert diag.verdict(r).startswith("LOOP")


def test_verdict_dry_run():
    assert diag.verdict(_res(fps30_auto=_v(prod=30, loop=30))).startswith("Dry-run")


def test_no_device_exits_cleanly(monkeypatch, capsys):
    def boom(args):
        raise RuntimeError("no camera")
    monkeypatch.setattr(diag, "run", boom)
    assert diag.main(["--live"]) == 2
    assert "attach_usb.py" in capsys.readouterr().err


def test_verdict_bridge_pass_and_fail():
    ok = _res(fps30_auto=_v(prod=30, loop=29.5))
    ok["transport"] = "bridge"
    assert diag.verdict(ok).startswith("BRIDGE PASS")
    bad = _res(fps30_auto=_v(prod=8, loop=5))
    bad["transport"] = "bridge"
    assert diag.verdict(bad).startswith("BRIDGE FAIL")


def test_transport_flag_parses():
    args = diag._parser().parse_args(["--live", "--transport", "bridge"])
    assert args.transport == "bridge"
    assert diag._parser().parse_args([]).transport == "config"
