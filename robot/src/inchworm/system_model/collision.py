"""
collision.py
============
Mesh collision checking for the whole cell (AA40 stage 2), built on the
owner's cell URDF (``CAD/robot_urdf_new/three_arm_cell.urdf``) with python-fcl.

What is checked
---------------
Every pair (A, B) where A is a MOVING SO-101 link (shoulder onward, including
the custom gripper and camera mount) and B is any other link in the cell:
the other arm, both arm bases, the base mount ``baseforreal``, every DexArm
link, the laser camera and the tube (when modelled). This covers arm-arm,
arm-chassis, arm-DexArm and arm-tube contact plus SO-101 self-collision.

A pair is reported when the meshes intersect OR their separation is below
``margin_m``.

Allowed pairs (never reported), each with its reason:
- links joined directly by a joint (kinematically adjacent; they touch at the
  joint by construction);
- links fixed to the same moving link (e.g. ``left_camera_mount`` is fixed to
  ``left_wrist``, so it is adjacent to the wrist);
- pairs in the owner SRDF ``config/three_arm_cell.srdf`` (each SO-101 base on
  the base mount);
- pairs where NEITHER link moves with an SO-101 joint (static-vs-static or
  DexArm-internal): SO-101 motion cannot change them, and DexArm self-contact
  is the DexArm firmware's job.

NOT checked: the table. Owner decision 2026-10-27: the arms never need to go
near the table, so it is not a collision object.

Fail closed
-----------
Any problem building or evaluating the model (missing URDF, missing mesh,
unknown joint name, a non-finite joint value) raises :class:`CollisionModelError`;
the motion guard turns that into a violation.
"""
from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, FrozenSet, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from .urdf import URDFError, URDFModel, load_geometry_mesh, load_urdf

_HERE = Path(__file__).resolve()
REPO_ROOT = _HERE.parents[4]
DEFAULT_CELL_URDF = REPO_ROOT / "CAD" / "robot_urdf_new" / "three_arm_cell.urdf"
DEFAULT_CELL_SRDF = REPO_ROOT / "CAD" / "robot_urdf_new" / "config" / "three_arm_cell.srdf"

#: Our joint order (kinematics.py / hardware.yaml) and the owner URDF joint names.
OUR_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
CELL_JOINT_SUFFIX = ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll")
ARMS = ("left", "right")


class CollisionModelError(RuntimeError):
    """The collision model cannot be built or evaluated (fail closed)."""


@dataclass(frozen=True)
class JointMapping:
    """Map our 5-vector (rad) onto the owner URDF joints: q_cell = sign*q + offset.

    Pan, lift, elbow and wrist_flex were verified against the vendored
    so101_new_calib chain (identical link-mesh poses to 0.015 mm, sign +1,
    offset 0; tests/test_cell_collision.py). Wrist roll is NOT: the owner
    URDF puts roll 0 at the end of travel (range -254..0 deg) while our zero
    is mid-range; its offset comes from config and stays unverified until
    the owner confirms it.
    """
    sign: Tuple[int, int, int, int, int] = (1, 1, 1, 1, 1)
    offset_rad: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def to_cell(self, arm: str, q: Sequence[float]) -> Dict[str, float]:
        if arm not in ARMS:
            raise CollisionModelError(f"unknown arm {arm!r}")
        q = np.asarray(q, dtype=float)
        if q.shape != (5,) or not np.all(np.isfinite(q)):
            raise CollisionModelError(f"{arm} joint vector must be 5 finite values, got {q!r}")
        return {
            f"{arm}_{suf}": self.sign[i] * float(q[i]) + self.offset_rad[i]
            for i, suf in enumerate(CELL_JOINT_SUFFIX)
        }


#: The tube link in the owner cell URDF. Its URDF placement is the CAD
#: position (attachment points lined up with the grippers); the tube is NOT
#: fixed there: the arms move it along its axis.
TUBE_LINK = "tube_1"


@dataclass(frozen=True)
class PoseScopedAllowance:
    """Contacts accepted ONLY near a measured real pose (owner decision).

    Owner 2026-10-27: "go with the real fold". The hand-placed, measured fold
    is physically fine, so contacts the model reports there are model error.
    A listed pair is ignored only while every arm owning one of its links is
    within ``radius_rad`` (per joint, pan..wrist_flex) of that arm's centre
    pose. Anywhere else the pair is checked like any other.
    """
    label: str
    pairs: FrozenSet[FrozenSet[str]]
    centre_rad: Mapping[str, Sequence[float]]  # arm -> 5-vector
    radius_rad: float

    def covers(self, a: str, b: str, driver: Mapping[str, Optional[str]], state: "CellState") -> bool:
        if frozenset((a, b)) not in self.pairs:
            return False
        arms = {driver.get(a), driver.get(b)} - {None}
        if not arms:
            return False
        for arm in arms:
            q = np.asarray(state.left_q if arm == "left" else state.right_q, dtype=float)
            c = np.asarray(self.centre_rad[arm], dtype=float)
            if np.any(np.abs(q[:4] - c[:4]) > self.radius_rad):
                return False
        return True


@dataclass(frozen=True)
class CellState:
    """Everything the checker needs besides the arm under test."""
    left_q: Sequence[float]
    right_q: Sequence[float]
    dexarm_q: Mapping[str, float] = field(default_factory=dict)
    #: Tube advance along its axis (m) from the URDF placement, or None when
    #: there is no tube in the cell. One crawl cycle = one attachment pitch.
    tube_advance_m: Optional[float] = None


@dataclass(frozen=True)
class Contact:
    link_a: str
    link_b: str
    distance_m: float  # 0.0 when the meshes intersect

    def __str__(self) -> str:
        if self.distance_m <= 0.0:
            return f"{self.link_a} intersects {self.link_b}"
        return f"{self.link_a} within {self.distance_m*1000:.1f} mm of {self.link_b}"


def tube_axis_from_mesh(mesh, pitch_m: Optional[float] = None) -> np.ndarray:
    """Unit tube axis (pointing +Y, away from the DexArm) from the tube mesh.

    The tube is repeated sections at the attachment pitch, so the axis is the
    direction along which the mesh maps onto itself shifted by one pitch.
    Refined from the principal axis by minimising the median vertex distance
    of the shifted copy (the owner tube_1 mesh: 0.000 mm, axis tilted about
    1.1 deg from cell +Y).
    """
    from scipy.optimize import minimize
    from scipy.spatial import cKDTree
    if pitch_m is None:
        from inchworm.constants import ATTACHMENT_PITCH_MM
        pitch_m = float(ATTACHMENT_PITCH_MM) / 1000.0
    v = np.asarray(mesh.vertices, dtype=float)
    c = v.mean(0)
    ax = np.linalg.svd(v - c, full_matrices=False)[2][0]
    if ax[1] < 0:
        ax = -ax
    tree = cKDTree(v)

    def unit(x):
        a = ax + np.array([x[0], 0.0, x[1]])
        return a / np.linalg.norm(a)

    def cost(x):
        d, _ = tree.query(v + unit(x) * pitch_m)
        return float(np.median(np.minimum(d, 0.02)))

    r = minimize(cost, [0.0, 0.0], method="Nelder-Mead",
                 options={"xatol": 1e-7, "fatol": 1e-9, "maxiter": 400})
    a = unit(r.x)
    if r.fun > 1e-4:  # 0.1 mm: the mesh is not periodic at the pitch
        raise CollisionModelError(
            f"tube mesh does not repeat at the attachment pitch (median {r.fun*1000:.2f} mm)"
        )
    return a


def _bvh(mesh) -> "object":
    import fcl
    b = fcl.BVHModel()
    b.beginModel(len(mesh.vertices), len(mesh.faces))
    b.addSubModel(np.asarray(mesh.vertices, dtype=np.float64), np.asarray(mesh.faces, dtype=np.int32))
    b.endModel()
    return b


def _read_srdf_pairs(path: Optional[Path]) -> Set[FrozenSet[str]]:
    if path is None:
        return set()
    if not path.exists():
        raise CollisionModelError(f"SRDF missing: {path}")
    root = ET.parse(path).getroot()
    return {frozenset((d.get("link1"), d.get("link2"))) for d in root.iter("disable_collisions")}


class CellCollisionChecker:
    """FCL mesh collision checker over the owner cell URDF."""

    def __init__(
        self,
        urdf_path: Path = DEFAULT_CELL_URDF,
        srdf_path: Optional[Path] = DEFAULT_CELL_SRDF,
        margin_m: float = 0.005,
        joint_mapping: JointMapping = JointMapping(),
        allowances: Sequence[PoseScopedAllowance] = (),
    ) -> None:
        self.allowances = tuple(allowances)
        try:
            import fcl  # noqa: F401
            import trimesh  # noqa: F401
        except ImportError as e:  # pragma: no cover - deps are in the lock
            raise CollisionModelError(f"collision dependencies missing: {e}") from e
        if not (margin_m >= 0.0 and math.isfinite(margin_m)):
            raise CollisionModelError(f"margin must be finite and >= 0, got {margin_m}")
        try:
            self.model: URDFModel = load_urdf(urdf_path)
        except (URDFError, OSError, ET.ParseError) as e:
            raise CollisionModelError(f"cell URDF: {e}") from e
        self.margin_m = float(margin_m)
        self.mapping = joint_mapping
        self._srdf_pairs = _read_srdf_pairs(srdf_path)

        import trimesh
        self._mesh: Dict[str, object] = {}
        self._geom: Dict[str, object] = {}
        try:
            for link, gs in self.model.collisions.items():
                m = trimesh.util.concatenate([load_geometry_mesh(g) for g in gs])
                self._mesh[link] = m
                self._geom[link] = _bvh(m)
        except (URDFError, OSError, ValueError) as e:
            raise CollisionModelError(f"collision mesh: {e}") from e

        for arm in ARMS:
            for suf in CELL_JOINT_SUFFIX:
                if f"{arm}_{suf}" not in self.model.joints:
                    raise CollisionModelError(f"cell URDF lacks joint {arm}_{suf}")

        self._driver = self._driving_arm_joint()
        self.moving_links: Dict[str, List[str]] = {
            arm: sorted(l for l, a in self._driver.items() if a == arm and l in self._geom)
            for arm in ARMS
        }
        self._adjacent = self._adjacent_pairs()
        self._pairs_cache: Dict[Tuple[str, bool], List[Tuple[str, str]]] = {}
        if TUBE_LINK not in self._geom:
            raise CollisionModelError(f"cell URDF lacks the tube link {TUBE_LINK!r}")
        self.tube_axis = tube_axis_from_mesh(self._mesh[TUBE_LINK])

    # ------------------------------------------------------------------
    # Structure
    # ------------------------------------------------------------------
    def _driving_arm_joint(self) -> Dict[str, Optional[str]]:
        """link -> 'left'/'right' if it moves with that SO-101's joints, else None."""
        out: Dict[str, Optional[str]] = {self.model.root: None}
        arm_joints = {f"{a}_{s}": a for a in ARMS for s in CELL_JOINT_SUFFIX}
        stack = [self.model.root]
        while stack:
            p = stack.pop()
            for jn in self.model._children.get(p, []):
                j = self.model.joints[jn]
                out[j.child] = arm_joints.get(jn, out[p])
                stack.append(j.child)
        return out

    def _rigid_body(self) -> Dict[str, str]:
        """link -> representative of the rigid group it belongs to (merged over fixed joints)."""
        rep = {self.model.root: self.model.root}
        stack = [self.model.root]
        while stack:
            p = stack.pop()
            for jn in self.model._children.get(p, []):
                j = self.model.joints[jn]
                rep[j.child] = rep[p] if j.type == "fixed" else j.child
                stack.append(j.child)
        return rep

    def _adjacent_pairs(self) -> Set[FrozenSet[str]]:
        rep = self._rigid_body()
        groups: Dict[str, Set[str]] = {}
        for l, r in rep.items():
            groups.setdefault(r, set()).add(l)
        adj: Set[FrozenSet[str]] = set()
        for g in groups.values():  # same rigid body
            for a in g:
                for b in g:
                    if a != b:
                        adj.add(frozenset((a, b)))
        for j in self.model.joints.values():
            if j.type == "fixed":
                continue
            for a in groups[rep[j.parent]]:
                for b in groups[rep[j.child]]:
                    adj.add(frozenset((a, b)))
        return adj

    def allowed_reason(self, a: str, b: str) -> Optional[str]:
        """Why pair (a, b) is never reported, or None if it is checked."""
        key = frozenset((a, b))
        if key in self._adjacent:
            return "kinematically adjacent (joined by a joint or fixed to the same body)"
        if key in self._srdf_pairs:
            return "owner SRDF: always in contact (SO-101 base on base mount)"
        if TUBE_LINK not in key and self._driver.get(a) is None and self._driver.get(b) is None:
            return "neither link moves with an SO-101 joint"
        return None

    def checked_pairs(self, arm: Optional[str] = None, with_tube: bool = False) -> List[Tuple[str, str]]:
        """Pairs that are checked. ``arm`` restricts to pairs involving that arm's moving links."""
        k = (arm or "*", with_tube)
        if k in self._pairs_cache:
            return self._pairs_cache[k]
        links = [l for l in sorted(self._geom) if with_tube or l != TUBE_LINK]
        out = []
        for i, a in enumerate(links):
            for b in links[i + 1:]:
                if self.allowed_reason(a, b) is not None:
                    continue
                if arm is not None and self._driver.get(a) != arm and self._driver.get(b) != arm:
                    continue
                out.append((a, b))
        self._pairs_cache[k] = out
        return out

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------
    def cell_joint_values(self, state: CellState) -> Dict[str, float]:
        q = {}
        q.update(self.mapping.to_cell("left", state.left_q))
        q.update(self.mapping.to_cell("right", state.right_q))
        for name, v in dict(state.dexarm_q).items():
            if name not in self.model.joints or self.model.joints[name].mimic is not None:
                raise CollisionModelError(f"unknown or mimic DexArm joint {name!r}")
            if not math.isfinite(float(v)):
                raise CollisionModelError(f"DexArm joint {name} is not finite")
            q[name] = float(v)
        return q

    def link_poses(self, state: CellState) -> Dict[str, np.ndarray]:
        try:
            poses = self.model.link_poses(self.cell_joint_values(state))
        except URDFError as e:
            raise CollisionModelError(str(e)) from e
        if state.tube_advance_m is not None:
            adv = float(state.tube_advance_m)
            if not math.isfinite(adv):
                raise CollisionModelError("tube advance must be finite")
            T = np.eye(4)
            T[:3, 3] = self.tube_axis * adv
            poses[TUBE_LINK] = T @ poses[TUBE_LINK]
        return poses

    def check(
        self,
        state: CellState,
        arm: Optional[str] = None,
        first_only: bool = True,
    ) -> List[Contact]:
        """Contacts in ``state``. ``arm`` limits the check to pairs involving that arm."""
        import fcl
        poses = self.link_poses(state)
        objs: Dict[str, object] = {}
        aabbs: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}

        def obj(link: str):
            if link not in objs:
                geom, T, mesh = self._geom[link], poses[link], self._mesh[link]
                objs[link] = fcl.CollisionObject(geom, fcl.Transform(T[:3, :3], T[:3, 3]))
                # cheap world AABB from the 8 local bbox corners (conservative)
                lo, hi = mesh.bounds
                c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                w = c @ T[:3, :3].T + T[:3, 3]
                aabbs[link] = (w.min(0), w.max(0))
            return objs[link]

        contacts: List[Contact] = []
        m = self.margin_m
        for a, b in self.checked_pairs(arm, with_tube=state.tube_advance_m is not None):
            oa, ob = obj(a), obj(b)
            (la, ha), (lb, hb) = aabbs[a], aabbs[b]
            if np.any(la - m > hb) or np.any(lb - m > ha):
                continue  # boxes farther apart than the margin
            if any(al.covers(a, b, self._driver, state) for al in self.allowances):
                continue
            if fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()):
                contacts.append(Contact(a, b, 0.0))
            elif m > 0.0:
                d = fcl.distance(oa, ob, fcl.DistanceRequest(), fcl.DistanceResult())
                if d < m:
                    contacts.append(Contact(a, b, float(max(d, 0.0))))
            if contacts and first_only:
                break
        return contacts

    def min_distance(self, state: CellState, a: str, b: str) -> float:
        """Exact separation of two links (0 if intersecting); for tests and reports."""
        import fcl
        poses = self.link_poses(state)
        oa = fcl.CollisionObject(self._geom[a], fcl.Transform(poses[a][:3, :3], poses[a][:3, 3]))
        ob = fcl.CollisionObject(self._geom[b], fcl.Transform(poses[b][:3, :3], poses[b][:3, 3]))
        if fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()):
            return 0.0
        return float(fcl.distance(oa, ob, fcl.DistanceRequest(), fcl.DistanceResult()))

    def world_meshes(self, state: CellState) -> List[Tuple[str, object]]:
        """(link, mesh in the cell frame) for rendering. Uses collision meshes."""
        poses = self.link_poses(state)
        out = []
        for link, m in self._mesh.items():
            if link == TUBE_LINK and state.tube_advance_m is None:
                continue
            mm = m.copy()
            mm.apply_transform(poses[link])
            out.append((link, mm))
        return out


TUBE_NAME = TUBE_LINK

_CACHE: Dict[Tuple[str, str], CellCollisionChecker] = {}


def get_cell_checker(
    urdf_path: Path = DEFAULT_CELL_URDF,
    srdf_path: Optional[Path] = DEFAULT_CELL_SRDF,
    margin_m: float = 0.005,
    joint_mapping: JointMapping = JointMapping(),
    allowances: Sequence[PoseScopedAllowance] = (),
) -> CellCollisionChecker:
    """Checker with the given margin/mapping; the meshes (~13 s) load once per process."""
    import copy
    key = (str(Path(urdf_path).resolve()), str(srdf_path))
    base = _CACHE.get(key)
    if base is None:
        base = CellCollisionChecker(urdf_path, srdf_path, margin_m=0.0)
        _CACHE[key] = base
    if not (margin_m >= 0.0 and math.isfinite(margin_m)):
        raise CollisionModelError(f"margin must be finite and >= 0, got {margin_m}")
    c = copy.copy(base)  # shares meshes/BVHs (read-only); own margin and mapping
    c.margin_m = float(margin_m)
    c.mapping = joint_mapping
    c.allowances = tuple(allowances)
    return c
