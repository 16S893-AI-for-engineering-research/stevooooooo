"""Owner 2026-11-21: the frozen training positions load and match the current timeline."""
from pathlib import Path

import pytest

from inchworm import training_poses as tp

ROBOT = Path(__file__).resolve().parents[1]
TIMELINE = ROBOT / "out" / "inchworm" / "timeline.jsonl"


def test_frozen_file_loads():
    d = tp.load()
    left, right = d["positions"]["train_left"], d["positions"]["train_right"]
    assert left["timeline_frame"] == 42 and left["inserting_arm"] == "left" and left["holding_arm"] == "right"
    assert right["timeline_frame"] == 67 and right["inserting_arm"] == "right" and right["holding_arm"] == "left"
    assert tp.for_arm("left") == left


@pytest.mark.skipif(not TIMELINE.exists(), reason="timeline not generated on this machine")
def test_timeline_reproduces_frozen_ticks():
    from inchworm.config import load_config
    assert tp.check_against_timeline(load_config(), TIMELINE) == []


def test_unknown_position_refused():
    with pytest.raises(tp.TrainingPoseError):
        tp.position("train_middle")
