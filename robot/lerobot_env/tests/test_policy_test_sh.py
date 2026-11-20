"""AA-39 single launcher policy_test.sh: the policy step starts only after a replay that wrote a
fresh handoff state for this arm (uv stubbed; no hardware, no real replay)."""
from __future__ import annotations

import json
import os
import pathlib
import shutil
import subprocess
import time

import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]


@pytest.fixture
def tree(tmp_path):
    """Copy of the launcher in a fake robot/ tree; `uv` stub logs its args and can write the state."""
    robot = tmp_path / "robot"
    (robot / "lerobot_env").mkdir(parents=True)
    (robot / "out").mkdir()
    shutil.copy(HERE / "policy_test.sh", robot / "lerobot_env")
    b = tmp_path / "bin"; b.mkdir()
    (b / "uv").write_text('#!/bin/sh\necho "$*" >> "$UV_LOG"\n'
                          'case "$*" in *replay_cycle*) [ -n "$WRITE_STATE" ] && cp "$WRITE_STATE" "$STATE_OUT";; esac\n'
                          'exit "${UV_RC:-0}"\n')
    (b / "uv").chmod(0o755)
    return robot, b, tmp_path / "uv.log"


def _run(tree, *args, state=None, rc=0):
    robot, b, log = tree
    src = robot.parent / "state.json"
    env = {**os.environ, "PATH": f"{b}:{os.environ['PATH']}", "UV_LOG": str(log), "UV_RC": str(rc),
           "STATE_OUT": str(robot / "out" / "sim_handoff_state.json"), "WRITE_STATE": ""}
    if state is not None:
        src.write_text(json.dumps(state)); env["WRITE_STATE"] = str(src)
    time.sleep(1.1)                                     # the launcher compares whole-second mtimes
    p = subprocess.run(["bash", str(robot / "lerobot_env" / "policy_test.sh"), *args], env=env,
                       capture_output=True, text=True)
    return p, (log.read_text().splitlines() if log.exists() else [])


def _st(frame, locked):
    return {"stop_frame": frame, "locked": locked, "tube_in_grippers": True, "utc": "x"}


def test_right_runs_replay_to_67_then_the_mapped_policy(tree):
    p, calls = _run(tree, "--arm", "right", "--dry-run", state=_st(67, {"left": True, "right": False}))
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--stop-after-frame 67 --tube-in-grippers" in calls[0] and "--live" not in calls[0]
    assert "policy_handoff.py --arm right --policy-arm left" in calls[1] and "--dry-run" in calls[1]
    assert "--fps 15" in calls[1]                       # owner 2026-11-20: right arm slow for now


def test_left_stops_at_42(tree):
    p, calls = _run(tree, "--arm", "left", "--dry-run", state=_st(42, {"left": False, "right": True}))
    assert p.returncode == 0 and "--stop-after-frame 42" in calls[0] and "--arm left" in calls[1]
    assert "--fps" not in calls[1]


@pytest.mark.parametrize("state, rc", [
    (None, 0),                                               # YES not given: no state written
    (_st(42, {"left": False, "right": True}), 0),            # wrong arm's stop
    (_st(67, {"left": True, "right": False}), 4),            # replay failed
])
def test_policy_never_starts_without_a_good_replay(tree, state, rc):
    p, calls = _run(tree, "--arm", "right", "--dry-run", state=state, rc=rc)
    assert p.returncode != 0 and len(calls) == 1 and "replay_cycle" in calls[0]


def test_arm_is_required(tree):
    p, calls = _run(tree)
    assert p.returncode == 2 and calls == []
