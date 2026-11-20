"""AA-39 (owner 2026-11-20): run one arm's trained policy on the other arm WITHOUT retraining.

The ACT policy works in raw servo degrees (ticks x 360/4096) of the arm it was trained on.
doc-1: both SO-101s are mounted the same way (not mirrored) and every joint sign is +1 on
both arms; only the per-arm zero offsets differ (hardware.yaml
``motion_guard.servo_calibration.<arm>_arm.<joint>_zero_offset_deg``). So a pose of the
target arm in URDF joint space q is the same pose for the source arm, and

    source_deg = target_deg - zero_target + zero_source        (observation, per joint)
    target_deg = source_deg - zero_source + zero_target        (policy action back)

The map REFUSES if any joint sign differs between the arms (a sign flip would need a mirrored
policy, not an offset) or is not +/-1. Nothing here opens a port or touches the gripper.
"""
from __future__ import annotations

from typing import Dict, Sequence

import numpy as np

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


class ArmTransferRefused(RuntimeError):
    pass


class JointOffsetMap:
    """Degree map between the ``target`` arm (the one moving) and the ``source`` arm (the one
    the policy was trained on), from the servo calibration in ``cfg``."""

    def __init__(self, cfg, *, source: str, target: str):
        if {source, target} - {"left", "right"}:
            raise ArmTransferRefused(f"arms must be 'left'/'right' (got {source!r}, {target!r})")
        cal = cfg.motion_guard.servo_calibration
        cs, ct = getattr(cal, f"{source}_arm"), getattr(cal, f"{target}_arm")
        bad = []
        for j in J5:
            ss, st = getattr(cs, f"{j}_sign"), getattr(ct, f"{j}_sign")
            if ss not in (1, -1) or st not in (1, -1) or ss != st:
                bad.append(f"{j} sign {source} {ss} vs {target} {st}")
        if bad:
            raise ArmTransferRefused("joint-offset map refused (signs must match on every joint; a flip needs "
                                     "a mirrored policy): " + "; ".join(bad))
        self.source, self.target = source, target
        self.zero_source = np.array([float(getattr(cs, f"{j}_zero_offset_deg")) for j in J5])
        self.zero_target = np.array([float(getattr(ct, f"{j}_zero_offset_deg")) for j in J5])
        #: degrees added to a target-arm reading to get the source-arm equivalent
        self.delta_deg = self.zero_source - self.zero_target

    def to_source(self, target_deg: Sequence[float]) -> np.ndarray:
        return np.asarray(target_deg, dtype=np.float64) + self.delta_deg

    def to_target(self, source_deg: Sequence[float]) -> np.ndarray:
        return np.asarray(source_deg, dtype=np.float64) - self.delta_deg

    def summary(self) -> Dict[str, float]:
        return {j: round(float(d), 3) for j, d in zip(J5, self.delta_deg)}


class MappedPolicy:
    """Wraps a policy (``reset()``, ``step(Observation) -> {joint: deg}``) trained on
    ``m.source`` so it drives ``m.target``: joints are mapped in and out, the frame passes
    through unchanged. Any non-joint action key is passed back unmapped so the caller's
    joints-1-5 check still refuses it."""

    def __init__(self, policy, m: JointOffsetMap):
        self.policy, self.map = policy, m

    def reset(self) -> None:
        self.policy.reset()

    def step(self, obs):
        from dataclasses import replace
        mapped = replace(obs, joint_positions_deg=self.map.to_source(obs.joint_positions_deg).astype(np.float32))
        act = self.policy.step(mapped)
        out = dict(act)
        if all(j in act for j in J5):
            back = self.map.to_target([act[j] for j in J5])
            out.update({j: float(v) for j, v in zip(J5, back)})
        return out
