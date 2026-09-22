"""Planar homogeneous transforms (SE(2) embedded in 4x4 SE(3) matrices).

The paper's control equations are written with 4x4 homogeneous transforms and
position vectors in the form [x, y, z, 1]^T (Sec. 4.2, Eqs. 3-5), so this
module keeps that shape even though the experiment is planar. That way
``compensation.py`` can be read straight against the paper, and extending to
the 6-DoF case the paper discusses in Sec. 5.3 means relaxing this module
rather than rewriting the algorithm.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "transform",
    "identity",
    "compose",
    "invert",
    "apply",
    "translation_of",
    "yaw_of",
    "wrap_angle",
]


def wrap_angle(theta: float | np.ndarray) -> float | np.ndarray:
    """Wrap an angle (or array of angles) to [-pi, pi].

    Uses the arctan2 round trip so the result is always in range and matches
    the wrapping done inside :func:`ffam.dynamics.rigid_body_step`.
    """
    theta = np.asarray(theta, dtype=float)
    wrapped = np.arctan2(np.sin(theta), np.cos(theta))
    return float(wrapped) if wrapped.ndim == 0 else wrapped


def identity() -> np.ndarray:
    """Return a 4x4 identity transform."""
    return np.eye(4, dtype=float)


def transform(x: float = 0.0, y: float = 0.0, theta: float = 0.0,
              z: float = 0.0) -> np.ndarray:
    """Build a 4x4 homogeneous transform for a planar pose.

    Args:
        x, y: translation in metres.
        theta: yaw about the z axis in radians.
        z: out-of-plane translation in metres (kept for 6-DoF extension).
    """
    c, s = np.cos(theta), np.sin(theta)
    t = np.eye(4, dtype=float)
    t[0, 0], t[0, 1] = c, -s
    t[1, 0], t[1, 1] = s, c
    t[0, 3], t[1, 3], t[2, 3] = x, y, z
    return t


def compose(*transforms: np.ndarray) -> np.ndarray:
    """Return the matrix product of the given transforms, left to right."""
    if not transforms:
        return identity()
    out = np.asarray(transforms[0], dtype=float)
    for t in transforms[1:]:
        out = out @ np.asarray(t, dtype=float)
    return out


def invert(t: np.ndarray) -> np.ndarray:
    """Invert a rigid transform analytically (R^T, -R^T p).

    Cheaper and numerically cleaner than ``np.linalg.inv`` for SE(3), and it
    cannot silently succeed on a non-rigid matrix the way a general solve can.
    """
    t = np.asarray(t, dtype=float)
    rot = t[:3, :3]
    pos = t[:3, 3]
    out = np.eye(4, dtype=float)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ pos
    return out


def apply(t: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply transform ``t`` to a 2D or 3D point, returning the same dimension."""
    point = np.asarray(point, dtype=float)
    dim = point.shape[-1]
    if dim not in (2, 3):
        raise ValueError(f"point must be 2D or 3D, got {dim}D")
    homogeneous = np.ones(4, dtype=float)
    homogeneous[:dim] = point
    result = t @ homogeneous
    return result[:dim]


def translation_of(t: np.ndarray) -> np.ndarray:
    """Extract the (x, y, z) translation of a transform."""
    return np.asarray(t, dtype=float)[:3, 3].copy()


def yaw_of(t: np.ndarray) -> float:
    """Extract the yaw angle about z from a transform."""
    t = np.asarray(t, dtype=float)
    return float(np.arctan2(t[1, 0], t[0, 0]))
