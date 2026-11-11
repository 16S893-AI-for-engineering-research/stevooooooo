"""Start-pose check (owner 2026-11-11): refuse to start when the real arms are
not where the plan starts, and the read-only pose snapshot path."""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest

from inchworm.config import load_config
from inchworm.so101_pose_reader import PoseReadError, ticks_to_q
from inchworm.start_pose_check import (
    StartPoseMismatch,
    compare_start_pose,
    plan_start_from_timeline,
    require_start_pose,
)

_ROBOT = Path(__file__).resolve().parents[1]
PLAN = {"left": [0.0, -1.5, 1.5, -1.5, 0.0], "right": [0.0, -1.5, 1.5, -1.5, 0.0]}


def test_matching_pose_passes():
    meas = {a: [v + math.radians(1.0) for v in q] for a, q in PLAN.items()}
    res = require_start_pose(meas, PLAN, tol_deg=3.0)
    assert res.ok and "OK" in res.summary()


def test_one_joint_off_refuses_and_names_it():
    meas = {a: list(q) for a, q in PLAN.items()}
    meas["right"][1] += math.radians(10.0)
    res = compare_start_pose(meas, PLAN, tol_deg=3.0)
    assert not res.ok
    assert any("right shoulder_lift" in f and "+10.0" in f for f in res.failures)
    with pytest.raises(StartPoseMismatch):
        require_start_pose(meas, PLAN, tol_deg=3.0)


@pytest.mark.parametrize("meas", [
    {"left": PLAN["left"]},                                    # missing arm
    {"left": PLAN["left"], "right": PLAN["right"][:4]},        # wrong length
    {"left": PLAN["left"], "right": [float("nan")] * 5},       # non-finite
])
def test_fails_closed_on_bad_measurement(meas):
    assert not compare_start_pose(meas, PLAN).ok


def test_bad_tolerance_rejected():
    with pytest.raises(ValueError):
        compare_start_pose(PLAN, PLAN, tol_deg=0.0)


def test_plan_start_from_timeline(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text(json.dumps({"left": [1, 2, 3, 4, 5], "right": [5, 4, 3, 2, 1], "step": "s"}) + "\n"
                 + json.dumps({"left": [0] * 5, "right": [0] * 5}) + "\n")
    assert plan_start_from_timeline(p) == {"left": [1, 2, 3, 4, 5], "right": [5, 4, 3, 2, 1]}


def test_ticks_to_q_uses_calibration():
    cfg = load_config()
    cal = cfg.motion_guard.servo_calibration.left_arm
    zero_ticks = [round(getattr(cal, f"{j}_zero_offset_deg") * 4096 / 360) for j in
                  ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]
    q = ticks_to_q(cfg, "left", zero_ticks)
    assert all(abs(v) < math.radians(0.1) for v in q)
    with pytest.raises(PoseReadError):
        ticks_to_q(cfg, "left", zero_ticks[:4])


def _load_snapshot_module():
    spec = importlib.util.spec_from_file_location("pose_snapshot", _ROBOT / "scripts" / "pose_snapshot.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_pose_snapshot_dry_run_opens_nothing(capsys):
    mod = _load_snapshot_module()
    assert mod.main(["--label", "fold"]) == 0
    assert "DRY-RUN" in capsys.readouterr().out


def test_pose_snapshot_logs_reading_render_and_answer(tmp_path):
    """Fake reader + fake renderer: the record carries ticks, radians, the PNG,
    the plan comparison and the owner's answer."""
    mod = _load_snapshot_module()
    cfg = load_config()
    ticks = {"left": [2048] * 5, "right": [2048] * 5}
    rendered = {}

    def fake_render(checker, state, png, title):
        rendered["png"], rendered["title"] = png, title
        return []

    log = tmp_path / "snap.jsonl"
    q_l = ticks_to_q(cfg, "left", ticks["left"])
    q_r = ticks_to_q(cfg, "right", ticks["right"])
    rec = mod.snapshot(cfg, lambda arm: ticks[arm], "fold", out_dir=tmp_path, log_path=log,
                       plan_start={"left": q_l, "right": q_r}, ask=lambda _p: "y",
                       render_fn=fake_render, checker=object())
    assert rec["owner_match"] == "y" and rec["plan_start_ok"] is True
    assert rendered["title"].startswith("SNAPSHOT fold")
    row = json.loads(log.read_text().splitlines()[0])
    assert row["ticks"] == ticks and row["label"] == "fold"
    assert len(row["q_rad"]["left"]) == 5
