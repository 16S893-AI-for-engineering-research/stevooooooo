"""AA-25 offline evaluator (eval_offline.py) on a tiny generated ACT checkpoint + dataset."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(sys.version_info < (3, 12), reason="lerobot_env only")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tests.test_policy_runner_lerobot import _make_checkpoint, _make_dataset  # noqa: E402


def test_evaluate_reports_per_joint_errors_and_hold_baseline(tmp_path):
    import eval_offline

    root, ckpt = tmp_path / "ds", tmp_path / "pretrained_model"
    _make_dataset(root, n_frames=8)
    _make_checkpoint(ckpt, root)
    res = eval_offline.evaluate(ckpt, root, [0], device="cpu", batch=3)
    ep = res["episodes"][0]
    assert ep["frames"] == 8
    for k in ("step1", "exec", "hold_baseline", "final"):
        assert len(ep[k]["mean"]) == 5 and all(v >= 0 for v in ep[k]["mean"])
    # Recorded action = state + 0.5 deg on every joint at t, and state grows 0.1 deg/frame:
    # holding the current state misses the next n_action_steps actions by 0.5..0.8 deg.
    assert 0.5 - 1e-3 <= ep["hold_baseline"]["mean_all"] <= 0.8 + 1e-3
    assert set(res["overall"]) == {"step1", "exec", "hold_baseline", "final"}
