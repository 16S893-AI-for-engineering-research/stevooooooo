"""AA-22 lerobot_env integration tests — run with the real lerobot installed.

These tests are in robot/lerobot_env/tests/ and must be run via:
    cd robot/lerobot_env && uv run pytest tests/ -v

They verify the adapter works with the real lerobot 0.6.1 base class:
  AC#1 — lerobot 0.6.1 importable; smoke imports succeed.
  AC#2 — adapter exposes correct joint map and camera feature via real Robot base.
  AC#3 — camera comes from injected frame_source (arbiter path, structural check).
  AC#4 — gripper key structurally absent; send_action blocks gripper keys.
  AC#5 — construction-time contract check integrates with real policy_contract.
  AC#8 — runs in Python >=3.12 environment.
  AC#9 — camera_wh from caller (AA-5 discovery) reflected in observation_features.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

# --- AC#8: must be Python >= 3.12 -----------------------------------------
pytestmark = pytest.mark.skipif(
    sys.version_info < (3, 12),
    reason="lerobot_env tests require Python >= 3.12 (AC#8)",
)

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

import lerobot  # must succeed (AC#1)
import torch
from lerobot.robots.robot import Robot as LeRobotBase

from inchworm.lerobot_adapter import (
    ADAPTER_JOINT_ORDER,
    ACTION_JOINT_KEYS,
    AdapterContractMismatch,
    CAMERA_KEY,
    CAMERA_SHAPE,
    OBSERVATION_JOINT_KEYS,
    _check_contract,
    _make_adapter_class,
    _make_config_class,
)
from inchworm.policy_actions import GripperCommandBlocked


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mock_arm(joint_positions: Optional[Dict[str, float]] = None) -> MagicMock:
    mock = MagicMock(name="GuardedArm")
    positions = joint_positions or {j: 0.0 for j in ADAPTER_JOINT_ORDER}
    mock.read_joint_deg.side_effect = lambda joint: positions.get(joint, 0.0)
    mock.apply_joint_command.return_value = None
    return mock


def _make_frame_src(h: int = 480, w: int = 640) -> MagicMock:
    mock = MagicMock(name="frame_source")
    mock.return_value = np.zeros((h, w), dtype=np.uint8)
    return mock


def _build(
    arm_role: str = "arm_left",
    frame_h: int = 480,
    frame_w: int = 640,
    camera_wh: Optional[Tuple[int, int]] = None,
    joint_positions: Optional[Dict[str, float]] = None,
    tmp_dir: Optional[Path] = None,
) -> Any:
    Config = _make_config_class()
    config = Config(arm_role=arm_role, frame_height=frame_h, frame_width=frame_w)
    AdapterCls = _make_adapter_class()
    adapter = AdapterCls(
        config,
        guarded_arm=_make_mock_arm(joint_positions),
        frame_source=_make_frame_src(frame_h, frame_w),
        camera_wh=camera_wh,
        calibration_dir=tmp_dir,
    )
    return adapter


# ---------------------------------------------------------------------------
# AC#1 — lerobot 0.6.1 importable
# ---------------------------------------------------------------------------

def test_ac1_lerobot_version():
    """AC#1: lerobot is importable and version is 0.6.1."""
    assert lerobot.__version__ == "0.6.1"


def test_ac1_python_version_is_312():
    """AC#8: Python >= 3.12 in this environment."""
    assert sys.version_info >= (3, 12)


# ---------------------------------------------------------------------------
# AC#2 — adapter is a proper Robot subclass
# ---------------------------------------------------------------------------

def test_ac2_adapter_is_robot_subclass(tmp_path):
    """AC#2: InchwormLeRobotAdapter is a lerobot Robot subclass."""
    AdapterCls = _make_adapter_class()
    assert issubclass(AdapterCls, LeRobotBase)


def test_ac2_adapter_name_attribute(tmp_path):
    """AC#2: adapter has name attribute set to inchworm_so101."""
    AdapterCls = _make_adapter_class()
    assert AdapterCls.name == "inchworm_so101"


def test_ac2_observation_features_joint_order(tmp_path):
    """AC#2: observation_features keys match JOINT_ORDER with .pos suffix."""
    adapter = _build(tmp_dir=tmp_path)
    obs_ft = adapter.observation_features
    for joint in ADAPTER_JOINT_ORDER:
        assert f"{joint}.pos" in obs_ft


def test_ac2_observation_features_no_gripper(tmp_path):
    """AC#2 + AC#4: gripper not in observation or action features."""
    adapter = _build(tmp_dir=tmp_path)
    for key in adapter.observation_features:
        assert "gripper" not in key
    for key in adapter.action_features:
        assert "gripper" not in key


# ---------------------------------------------------------------------------
# AC#3 — camera frame via injected frame_source
# ---------------------------------------------------------------------------

def test_ac3_camera_via_frame_source(tmp_path):
    """AC#3: frame_source is called on get_observation (arbiter path)."""
    Config = _make_config_class()
    config = Config(arm_role="arm_left", frame_height=240, frame_width=320)
    frame_source = MagicMock(return_value=np.zeros((240, 320), dtype=np.uint8))
    AdapterCls = _make_adapter_class()
    adapter = AdapterCls(
        config,
        guarded_arm=_make_mock_arm(),
        frame_source=frame_source,
        calibration_dir=tmp_path,
    )
    adapter.connect()
    obs = adapter.get_observation()
    frame_source.assert_called_once()
    assert obs[CAMERA_KEY].shape == (240, 320, 1)
    assert obs[CAMERA_KEY].dtype == np.uint8


# ---------------------------------------------------------------------------
# AC#4 — gripper excluded at action level
# ---------------------------------------------------------------------------

def test_ac4_gripper_raises_at_send_action(tmp_path):
    """AC#4: gripper.pos key in send_action raises GripperCommandBlocked."""
    adapter = _build(tmp_dir=tmp_path)
    adapter.connect()
    with pytest.raises(GripperCommandBlocked):
        adapter.send_action({"gripper.pos": 50.0})


def test_ac4_action_features_length_is_five(tmp_path):
    """AC#4: exactly 5 joint floats in action_features (no gripper)."""
    adapter = _build(tmp_dir=tmp_path)
    assert len(adapter.action_features) == 5


# ---------------------------------------------------------------------------
# AC#5 — construction-time contract check
# ---------------------------------------------------------------------------

def test_ac5_contract_check_passes_with_real_contract():
    """AC#5: _check_contract() does not raise with real policy_contract."""
    _check_contract()  # must not raise


# ---------------------------------------------------------------------------
# AC#9 — camera_wh from AA-5 discovery
# ---------------------------------------------------------------------------

def test_ac9_camera_wh_reflected_in_features(tmp_path):
    """AC#9: camera shape in observation_features uses camera_wh from caller."""
    adapter = _build(camera_wh=(800, 600), tmp_dir=tmp_path)
    obs_ft = adapter.observation_features
    h, w, c = obs_ft[CAMERA_KEY]
    assert (h, w, c) == (600, 800, 1)  # camera_wh=(width, height)


# ---------------------------------------------------------------------------
# AC#8 — inchworm importable from lerobot_env
# ---------------------------------------------------------------------------

def test_ac8_inchworm_importable_from_lerobot_env():
    """AC#8: inchworm main package is importable from the lerobot_env."""
    import inchworm  # noqa: F401
    from inchworm.config import load_config  # noqa: F401
    from inchworm.policy_contract import CONTRACT_VERSION  # noqa: F401
    assert CONTRACT_VERSION >= 1
