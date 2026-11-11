"""Start-pose check: does the real robot start where the plan starts?

Owner 2026-11-11: "how are we checking that the animation will match real
life ... if the robot is starting in a different place than it actually is?"
Before any planned motion (the AA-45 cycle, a replay, a dry cycle), the runner
reads where the arms really are and compares that with the plan's first frame.
If any joint differs by more than the tolerance it refuses to start, naming the
joint and the difference.

Pure functions; no hardware. The live read is ``so101_pose_reader``.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Mapping, Sequence

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

#: Default per-joint tolerance. UNVERIFIED placeholder (2026-11-11): a few
#: degrees is the hand-placement accuracy expected when the owner poses the
#: arms; tighten once replays show the real tracking error.
DEFAULT_TOL_DEG = 3.0


class StartPoseMismatch(RuntimeError):
    """The measured pose is not the plan's start pose; motion must not start."""


@dataclass
class StartPoseResult:
    tol_deg: float
    #: arm -> per-joint (measured - planned) in degrees
    diff_deg: Dict[str, List[float]] = field(default_factory=dict)
    failures: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.failures

    def summary(self) -> str:
        lines = [f"start-pose check (tol {self.tol_deg:.1f} deg per joint): "
                 f"{'OK' if self.ok else 'MISMATCH'}"]
        for arm, d in self.diff_deg.items():
            lines.append(f"  {arm:5}: " + "  ".join(f"{j} {v:+6.1f}" for j, v in zip(JOINTS, d)))
        for f in self.failures:
            lines.append(f"  - {f}")
        return "\n".join(lines)


def compare_start_pose(
    measured: Mapping[str, Sequence[float]],
    planned: Mapping[str, Sequence[float]],
    tol_deg: float = DEFAULT_TOL_DEG,
) -> StartPoseResult:
    """Compare measured vs planned joint radians for every planned arm.

    Fails closed: an arm in the plan with no measurement, a wrong length or a
    non-finite value is a failure, never a pass.
    """
    if not (tol_deg > 0 and math.isfinite(tol_deg)):
        raise ValueError("tol_deg must be a positive finite number")
    res = StartPoseResult(tol_deg=float(tol_deg))
    for arm, qp in planned.items():
        qm = measured.get(arm)
        if qm is None:
            res.failures.append(f"{arm}: no measured pose")
            continue
        if len(qm) != len(JOINTS) or len(qp) != len(JOINTS):
            res.failures.append(f"{arm}: need {len(JOINTS)} joints, got {len(qm)} measured / {len(qp)} planned")
            continue
        d = []
        for j, a, b in zip(JOINTS, qm, qp):
            a, b = float(a), float(b)
            if not (math.isfinite(a) and math.isfinite(b)):
                res.failures.append(f"{arm} {j}: non-finite value")
                d.append(float("nan"))
                continue
            dd = math.degrees(a - b)
            d.append(dd)
            if abs(dd) > tol_deg:
                res.failures.append(
                    f"{arm} {j}: measured {math.degrees(a):+.1f} deg vs planned "
                    f"{math.degrees(b):+.1f} deg (off by {dd:+.1f}, tol {tol_deg:.1f})"
                )
        res.diff_deg[arm] = d
    return res


def require_start_pose(measured, planned, tol_deg: float = DEFAULT_TOL_DEG) -> StartPoseResult:
    """Like :func:`compare_start_pose` but raises :class:`StartPoseMismatch` on failure."""
    res = compare_start_pose(measured, planned, tol_deg)
    if not res.ok:
        raise StartPoseMismatch(res.summary())
    return res


def plan_start_from_timeline(path: Path) -> Dict[str, List[float]]:
    """First frame of a timeline JSONL ({"left": [5], "right": [5], ...} per line)."""
    for line in Path(path).read_text().splitlines():
        if line.strip():
            d = json.loads(line)
            return {"left": list(d["left"]), "right": list(d["right"])}
    raise ValueError(f"{path}: empty timeline")
