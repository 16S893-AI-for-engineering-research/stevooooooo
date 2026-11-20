"""AA-39: left policy on the right arm through the per-arm zero-offset map (no hardware)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from inchworm.arm_transfer import J5, ArmTransferRefused, JointOffsetMap, MappedPolicy
from inchworm.config import load_config
from inchworm.handoff_return import q_to_ticks_f, ticks_to_q
from inchworm.policy_contract import Observation

CFG = load_config()


def test_hardware_yaml_offsets_and_round_trip():
    m = JointOffsetMap(CFG, source="left", target="right")
    # AA-39 plan: zero offsets L 173.320/214.893/182.900/178.286/175.687, R 181.318/220.518/179.033/175.254/175.115
    assert m.summary() == pytest.approx({"shoulder_pan": -7.998, "shoulder_lift": -5.625, "elbow_flex": 3.867,
                                         "wrist_flex": 3.032, "wrist_roll": 0.572}, abs=1e-3)
    x = np.array([181.3, 200.0, 170.0, 100.0, 175.0])
    assert np.allclose(m.to_target(m.to_source(x)), x)
    assert np.allclose(m.to_source(m.to_target(x)), x)


def test_map_preserves_joint_space_pose():
    """A right pose and its mapped left-equivalent are the same URDF joint angles q."""
    m = JointOffsetMap(CFG, source="left", target="right")
    q = [0.0, -0.3811, 0.1574, -1.3183, 0.0]          # pre-insertion pose (frames 42 / 67)
    r_deg = np.array(q_to_ticks_f(CFG, "right", q)) * 360 / 4096
    l_deg = m.to_source(r_deg)
    assert np.allclose(ticks_to_q(CFG, "left", l_deg * 4096 / 360), q, atol=1e-9)
    assert np.allclose(l_deg, np.array(q_to_ticks_f(CFG, "left", q)) * 360 / 4096)


def _with_sign(arm, joint, sign):
    cal = CFG.motion_guard.servo_calibration
    a = getattr(cal, f"{arm}_arm").model_copy(update={f"{joint}_sign": sign})
    mg = CFG.motion_guard.model_copy(update={"servo_calibration": cal.model_copy(update={f"{arm}_arm": a})})
    return CFG.model_copy(update={"motion_guard": mg})


def test_sign_mismatch_is_refused():
    with pytest.raises(ArmTransferRefused, match="elbow_flex sign"):
        JointOffsetMap(_with_sign("right", "elbow_flex", -1), source="left", target="right")
    with pytest.raises(ArmTransferRefused, match="sign"):
        JointOffsetMap(_with_sign("left", "wrist_roll", 2), source="left", target="right")
    with pytest.raises(ArmTransferRefused):
        JointOffsetMap(CFG, source="left", target="middle")


class _Rec:
    def __init__(self, extra=None):
        self.seen, self.resets, self.extra = [], 0, extra or {}

    def reset(self):
        self.resets += 1

    def step(self, obs):
        self.seen.append(np.array(obs.joint_positions_deg, float))
        return {**{j: float(v) + 1.0 for j, v in zip(J5, obs.joint_positions_deg)}, **self.extra}


def test_mapped_policy_maps_in_and_out_and_passes_frame():
    m = JointOffsetMap(CFG, source="left", target="right")
    inner = _Rec()
    p = MappedPolicy(inner, m)
    p.reset()
    frame = np.full((4, 4), 7, np.uint8)
    r = np.array([181.3, 200.0, 170.0, 100.0, 175.0], np.float32)
    act = p.step(Observation(frame=frame, joint_positions_deg=r, step_index=3))
    assert inner.resets == 1
    assert np.allclose(inner.seen[0], r + m.delta_deg, atol=1e-4)      # policy saw left-equivalent degrees
    assert np.allclose([act[j] for j in J5], r + 1.0, atol=1e-4)       # +1 deg in left = +1 deg on the right


def test_mapped_policy_keeps_extra_keys_for_the_callers_refusal():
    p = MappedPolicy(_Rec(extra={"gripper": 3.0}), JointOffsetMap(CFG, source="left", target="right"))
    act = p.step(Observation(frame=np.zeros((2, 2), np.uint8), joint_positions_deg=np.zeros(5, np.float32),
                             step_index=0))
    assert "gripper" in act and not math.isnan(act["gripper"])
