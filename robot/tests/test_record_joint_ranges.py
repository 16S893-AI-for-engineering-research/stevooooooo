"""Orchestrator 2026-10-24: read-only joint-range recorder logic."""
import importlib.util
import pathlib

spec = importlib.util.spec_from_file_location(
    "rjr", pathlib.Path(__file__).resolve().parents[1] / "scripts" / "record_joint_ranges.py")
import sys
rjr = importlib.util.module_from_spec(spec)
sys.modules["rjr"] = rjr  # dataclasses need the module registered
spec.loader.exec_module(rjr)


def test_tracks_min_max_and_flags_wrap():
    seq = {1: iter([2000, 1500, 2500, 2400]), 2: iter([100, 4050, 4000, 4000])}
    rec = rjr.Recorder(lambda sid: next(seq[sid]), {"a": 1, "b": 2})
    rec.start(); rec.poll(); rec.poll(); rec.poll()
    a, b = rec.ranges["a"], rec.ranges["b"]
    assert (a.lo, a.hi, a.wrapped) == (1500, 2500, False)
    assert b.wrapped is True  # 100 -> 4050 crosses the 4095/0 seam


def test_dry_run_exits_zero_without_hardware():
    assert rjr.main(["--arm", "left"]) == 0


def test_unwrapped_range_across_seam():
    # start 100 ticks, sweep down through 0 to 4000 (i.e. -196 ticks), then up to 300
    seq = iter([100, 50, 4090, 4000, 4050, 10, 300])
    rec = rjr.Recorder(lambda sid: next(seq), {"j": 1})
    rec.start()
    for _ in range(6):
        rec.poll()
    r = rec.ranges["j"].to_dict()
    assert r["travel_lo_ticks_rel_start"] == -196
    assert r["travel_hi_ticks_rel_start"] == 200
    assert r["crosses_seam"] is True
    assert abs(r["travel_span_deg"] - 396 * 360 / 4096) < 0.1
