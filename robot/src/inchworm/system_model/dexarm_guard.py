"""
dexarm_guard.py
===============
Mesh collision check for DexArm motion on the owner cell model (AA40 / AA-12).

The SO-101 mesh guard checks pairs that involve a moving SO-101 link, so it
never sees the DexArm moving into a parked SO-101, the base mount, the bases
or the tube. This guard covers that: every pair (DexArm moving link, any
non-DexArm link) is checked along the move, at the firmware's own joint
interpolation (G0 in the firmware's default fast mode moves in joint space).

Fail closed: an unknown SO-101 pose, an unknown tube state, an unreachable
target, a pose that cannot be read back from the firmware, or any model error
is a refusal.

Not checked: DexArm self-contact (the firmware's job) and the table (owner
2026-10-27: no table object).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .collision import ARMS, CellState, Contact, TUBE_LINK
from .dexarm_kinematics import (
    DexArmAngles,
    DexArmKinematicsError,
    MODULE_OFFSET_3D_MM,
    inverse,
    to_cell_joints,
)

#: DexArm-internal links that are not moved by its joints but belong to it.
_DEXARM_STATIC = ("B_Fix",)


class DexArmGuardRefused(RuntimeError):
    """A DexArm move is refused by the cell-model collision check (fail closed)."""


@dataclass
class DexArmCellGuard:
    """Check DexArm moves against the cell model.

    Parameters
    ----------
    checker:
        A configured :class:`~inchworm.system_model.collision.CellCollisionChecker`.
    so101_pose_source:
        Returns ``{"left": q5, "right": q5}`` (rad). Required.
    tube_source:
        Returns the tube advance in metres, or None for "no tube in the cell".
        Required (None means "stated: no tube"; a missing source refuses).
    leveling_slopes:
        Firmware M891 slopes (x, y) as stored in the DexArm (read with M892).
    step_deg:
        Joint interpolation step for path checks.
    """

    checker: object
    so101_pose_source: Optional[Callable[[], Dict[str, Sequence[float]]]] = None
    tube_source: Optional[Callable[[], Optional[float]]] = None
    leveling_slopes: Optional[Tuple[float, float]] = None
    module_offset_mm: float = MODULE_OFFSET_3D_MM
    step_deg: float = 1.0

    def __post_init__(self) -> None:
        drv = self.checker._driver
        self._dex_links = sorted(
            l for l in self.checker._geom
            if l != TUBE_LINK and (l in _DEXARM_STATIC or self._is_dexarm(l))
        )
        self._moving = [l for l in self._dex_links if l not in _DEXARM_STATIC]
        others = [l for l in self.checker._geom if l not in self._dex_links]
        self._pairs: List[Tuple[str, str]] = []
        for a in self._moving:
            for b in others:
                if self.checker.allowed_reason(a, b) is not None and frozenset((a, b)) in self.checker._adjacent:
                    continue
                if frozenset((a, b)) in self.checker._srdf_pairs:
                    continue
                self._pairs.append((a, b))

    def _is_dexarm(self, link: str) -> bool:
        model = self.checker.model
        cur = link
        parents = {j.child: j.parent for j in model.joints.values()}
        while cur in parents:
            if cur == "B_Fix":
                return True
            cur = parents[cur]
        return False

    # ------------------------------------------------------------------
    def _state(self, angles: DexArmAngles) -> CellState:
        if self.so101_pose_source is None:
            raise DexArmGuardRefused("SO-101 poses unknown (no so101_pose_source)")
        if self.tube_source is None:
            raise DexArmGuardRefused("tube state not given (no tube_source)")
        poses = self.so101_pose_source()
        missing = [a for a in ARMS if a not in poses]
        if missing:
            raise DexArmGuardRefused(f"SO-101 pose missing for {missing}")
        return CellState(
            left_q=np.asarray(poses["left"], dtype=float),
            right_q=np.asarray(poses["right"], dtype=float),
            dexarm_q=to_cell_joints(angles),
            tube_advance_m=self.tube_source(),
        )

    def contacts_at(self, angles: DexArmAngles, first_only: bool = True) -> List[Contact]:
        import fcl
        st = self._state(angles)
        P = self.checker.link_poses(st)
        m = self.checker.margin_m
        out: List[Contact] = []
        with_tube = st.tube_advance_m is not None
        objs = {}
        boxes = {}

        def obj(l):
            if l not in objs:
                T = P[l]
                objs[l] = fcl.CollisionObject(self.checker._geom[l], fcl.Transform(T[:3, :3], T[:3, 3]))
            return objs[l]

        def box(l):
            if l not in boxes:
                lo, hi = self.checker._mesh[l].bounds
                c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                w = c @ P[l][:3, :3].T + P[l][:3, 3]
                boxes[l] = (w.min(0), w.max(0))
            return boxes[l]

        for a, b in self._pairs:
            if b == TUBE_LINK and not with_tube:
                continue
            (la, ha), (lb, hb) = box(a), box(b)
            if np.any(la - m > hb) or np.any(lb - m > ha):
                continue  # bounding boxes farther apart than the margin
            oa, ob = obj(a), obj(b)
            if fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()):
                out.append(Contact(a, b, 0.0))
            elif m > 0:
                d = fcl.distance(oa, ob, fcl.DistanceRequest(), fcl.DistanceResult())
                if d < m:
                    out.append(Contact(a, b, float(max(d, 0.0))))
            if out and first_only:
                break
        return out

    def target_angles(self, x: float, y: float, z: float) -> DexArmAngles:
        if self.leveling_slopes is None:
            raise DexArmGuardRefused("DexArm leveling slopes unknown (read M892 first)")
        try:
            return inverse(x, y, z, self.leveling_slopes, self.module_offset_mm)
        except DexArmKinematicsError as exc:
            raise DexArmGuardRefused(f"target unreachable: {exc}") from exc

    def check_move(self, start: DexArmAngles, x: float, y: float, z: float) -> None:
        """Raise :class:`DexArmGuardRefused` if the joint-space move start->target hits anything."""
        try:
            end = self.target_angles(x, y, z)
            span = max(abs(end.a - start.a), abs(end.b - start.b), abs(end.c - start.c))
            n = max(2, int(math.ceil(span / max(self.step_deg, 1e-3))) + 1)
            for i in range(n):
                t = i / (n - 1)
                q = DexArmAngles(
                    start.a + t * (end.a - start.a),
                    start.b + t * (end.b - start.b),
                    start.c + t * (end.c - start.c),
                )
                hit = self.contacts_at(q)
                if hit:
                    raise DexArmGuardRefused(
                        f"DexArm move to ({x:.1f}, {y:.1f}, {z:.1f}) mm: sample {i}/{n-1}: {hit[0]}"
                    )
        except DexArmGuardRefused:
            raise
        except Exception as exc:  # noqa: BLE001 - fail closed on any model error
            raise DexArmGuardRefused(f"DexArm cell check failed (fail closed): {exc}") from exc


def build_dexarm_cell_guard(
    cfg,
    so101_pose_source: Optional[Callable[[], Dict[str, Sequence[float]]]] = None,
    tube_source: Optional[Callable[[], Optional[float]]] = None,
) -> Optional[DexArmCellGuard]:
    """Guard from hardware.yaml, or None if ``dexarm.cell_guard`` is absent/disabled.

    A configured guard whose model cannot load raises (fail closed): the caller
    must not fall back to unchecked motion.
    """
    gcfg = getattr(cfg.dexarm, "cell_guard", None)
    if gcfg is None or not gcfg.enabled:
        return None
    from inchworm.motion_guard import build_mesh_collision_guard
    mg = build_mesh_collision_guard(cfg, "left")
    if mg is None or mg.checker is None:
        raise DexArmGuardRefused(f"cell model unavailable: {mg and mg.load_error}")
    slopes = None
    if gcfg.leveling_slope_x is not None and gcfg.leveling_slope_y is not None:
        slopes = (gcfg.leveling_slope_x, gcfg.leveling_slope_y)
    return DexArmCellGuard(
        checker=mg.checker,
        so101_pose_source=so101_pose_source,
        tube_source=tube_source,
        leveling_slopes=slopes,
        step_deg=gcfg.step_deg,
    )
