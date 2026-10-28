"""
urdf.py
=======
Minimal, dependency-light URDF loader and forward kinematics for the owner's
cell model (``CAD/robot_urdf_new/three_arm_cell.urdf``, AA40).

Supports exactly what the cell URDF uses: ``fixed``, ``revolute`` and
``continuous`` joints, ``mimic`` joints, per-link ``collision`` and ``visual``
elements with ``<mesh filename scale>`` geometry. Anything else (prismatic,
primitive geometry, ``package://`` paths that do not resolve) raises
:class:`URDFError` instead of being silently skipped, so a model problem is
loud (fail closed).

Frames: every pose is ``T_world_link`` (4x4, metres) where ``world`` is the
URDF root. In the owner cell URDF, ``world`` is the Onshape assembly frame
(= the chassis frame of doc-1; base mount ``baseforreal`` at the origin).
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

import numpy as np


class URDFError(RuntimeError):
    """The URDF is malformed, uses an unsupported feature, or a mesh is missing."""


def rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: R = Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_tf(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle(axis: np.ndarray, theta: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)
    return T


def _parse_origin(el: Optional[ET.Element]) -> np.ndarray:
    if el is None:
        return np.eye(4)
    xyz = [float(v) for v in (el.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (el.get("rpy") or "0 0 0").split()]
    return origin_tf(xyz, rpy)


@dataclass(frozen=True)
class Geometry:
    """One mesh element of a link (collision or visual)."""
    link: str
    mesh_path: Path
    scale: Tuple[float, float, float]
    T_link_geom: np.ndarray


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    T_parent_joint: np.ndarray
    axis: np.ndarray
    lower: Optional[float]
    upper: Optional[float]
    mimic: Optional[Tuple[str, float, float]]  # (joint, multiplier, offset)


@dataclass
class URDFModel:
    path: Path
    name: str
    root: str
    links: List[str]
    joints: Dict[str, Joint]
    collisions: Dict[str, List[Geometry]]
    visuals: Dict[str, List[Geometry]]
    _children: Dict[str, List[str]] = field(default_factory=dict, repr=False)

    # ------------------------------------------------------------------
    @property
    def actuated_joints(self) -> List[str]:
        """Revolute/continuous joints that are not mimics (the free variables)."""
        return [j.name for j in self.joints.values()
                if j.type in ("revolute", "continuous") and j.mimic is None]

    def joint_value(self, name: str, q: Mapping[str, float]) -> float:
        j = self.joints[name]
        if j.type == "fixed":
            return 0.0
        if j.mimic is not None:
            src, mult, off = j.mimic
            return mult * self.joint_value(src, q) + off
        return float(q.get(name, 0.0))

    def link_poses(self, q: Optional[Mapping[str, float]] = None) -> Dict[str, np.ndarray]:
        """T_world_link for every link. Joints missing from ``q`` are at 0.

        Unknown names in ``q`` raise :class:`URDFError` (a typo must not
        silently leave a joint at zero).
        """
        q = dict(q or {})
        unknown = set(q) - set(self.joints)
        if unknown:
            raise URDFError(f"unknown joint name(s): {sorted(unknown)}")
        poses = {self.root: np.eye(4)}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for jn in self._children.get(parent, []):
                j = self.joints[jn]
                T = poses[parent] @ j.T_parent_joint
                if j.type in ("revolute", "continuous"):
                    T = T @ axis_angle(j.axis, self.joint_value(jn, q))
                poses[j.child] = T
                stack.append(j.child)
        return poses


def _resolve_mesh(filename: str, urdf_dir: Path) -> Path:
    if filename.startswith("package://"):
        # package://<pkg>/<rest>: resolve <rest> against the URDF's package dir
        rest = filename[len("package://"):].split("/", 1)[1]
        for base in (urdf_dir, urdf_dir.parent):
            cand = base / rest
            if cand.exists():
                return cand
        raise URDFError(f"cannot resolve mesh {filename!r}")
    p = Path(filename)
    p = p if p.is_absolute() else urdf_dir / p
    if not p.exists():
        raise URDFError(f"mesh file missing: {p}")
    return p


def load_urdf(path: Path | str) -> URDFModel:
    path = Path(path)
    if not path.exists():
        raise URDFError(f"URDF missing: {path}")
    root_el = ET.parse(path).getroot()
    links = [l.get("name") for l in root_el.findall("link")]
    joints: Dict[str, Joint] = {}
    children: Dict[str, List[str]] = {}
    child_set = set()
    for j in root_el.findall("joint"):
        jt = j.get("type")
        if jt not in ("fixed", "revolute", "continuous"):
            raise URDFError(f"joint {j.get('name')}: unsupported type {jt!r}")
        lim = j.find("limit")
        lower = upper = None
        if lim is not None and jt == "revolute":
            lower = float(lim.get("lower")) if lim.get("lower") is not None else None
            upper = float(lim.get("upper")) if lim.get("upper") is not None else None
        mi = j.find("mimic")
        mimic = None
        if mi is not None:
            mimic = (mi.get("joint"), float(mi.get("multiplier", 1.0)),
                     float(mi.get("offset", 0.0)))
        ax = j.find("axis")
        axis = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()])
        jo = Joint(
            name=j.get("name"), type=jt,
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            T_parent_joint=_parse_origin(j.find("origin")), axis=axis,
            lower=lower, upper=upper, mimic=mimic,
        )
        joints[jo.name] = jo
        children.setdefault(jo.parent, []).append(jo.name)
        child_set.add(jo.child)
    roots = [l for l in links if l not in child_set]
    if len(roots) != 1:
        raise URDFError(f"expected exactly one root link, got {roots}")
    for jo in joints.values():
        if jo.mimic is not None and jo.mimic[0] not in joints:
            raise URDFError(f"joint {jo.name} mimics unknown joint {jo.mimic[0]}")

    def geoms(tag: str) -> Dict[str, List[Geometry]]:
        out: Dict[str, List[Geometry]] = {}
        for l in root_el.findall("link"):
            for g in l.findall(tag):
                m = g.find("geometry/mesh")
                if m is None:
                    raise URDFError(f"link {l.get('name')}: only mesh {tag} geometry is supported")
                scale = tuple(float(v) for v in (m.get("scale") or "1 1 1").split())
                out.setdefault(l.get("name"), []).append(Geometry(
                    link=l.get("name"),
                    mesh_path=_resolve_mesh(m.get("filename"), path.parent),
                    scale=scale, T_link_geom=_parse_origin(g.find("origin")),
                ))
        return out

    return URDFModel(
        path=path, name=root_el.get("name"), root=roots[0], links=links,
        joints=joints, collisions=geoms("collision"), visuals=geoms("visual"),
        _children=children,
    )


def load_geometry_mesh(g: Geometry):
    """Load one geometry's mesh in its LINK frame (scale and origin applied)."""
    import trimesh
    m = trimesh.load(g.mesh_path, force="mesh", process=False)
    if not isinstance(m, trimesh.Trimesh) or len(m.faces) == 0:
        raise URDFError(f"mesh {g.mesh_path} is empty or not a triangle mesh")
    m = m.copy()
    m.apply_scale(np.asarray(g.scale))
    m.apply_transform(g.T_link_geom)
    return m
