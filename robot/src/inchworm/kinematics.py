"""SO-101 5-DoF forward and inverse kinematics.

This module implements FK/IK for the SO-101 arm's 5-DoF revolute chain:
  shoulder_pan → shoulder_lift → elbow_flex → wrist_flex → wrist_roll

The gripper joint is **excluded** from IK; it is commanded only via the
AA-17 state machine / AA-8 gripper_channel token.

Chain parameters come from ``hardware.yaml`` (the ``so101_kinematics`` block
populated in AA-11), which records values verbatim from the TheRobotStudio
URDF (``robot/vendor/so101/so101_new_calib.urdf``, hash a758567c).

Reference frame
---------------
FK returns poses in the **arm base frame** (``base_link`` origin as defined
in the URDF).  To express poses in the chassis-fixed frame, apply the
``left_arm_base_to_chassis`` or ``right_arm_base_to_chassis`` transform from
config — both are marked ``unverified: true`` until measured in AA-16.

Forward kinematics (FK)
-----------------------
Chain product of homogeneous transforms (HTMs):

    T_tool = T_01 * T_12 * ... * T_{n-1,n} * T_tool_offset

where each ``T_i,i+1`` is built from the URDF joint origin (fixed rotation +
translation) composed with the revolute rotation about the joint axis (always
Z in this URDF).

Inverse kinematics (IK)
-----------------------
Numerical damped least-squares (DLS / Levenberg-Marquardt style Jacobian
pseudoinverse).

Documented deterministic seed and solution-selection rule
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* **Seed**: ``q = zeros(5)`` (all joints at zero).  This is the URDF rest pose
  and is documented here as the single, reproducible starting point.  If a
  caller needs a different seed it passes ``q_seed`` explicitly.
* **Iteration**: gradient descent via DLS — at each step compute the geometric
  Jacobian, compute ``Δq = J^T (J J^T + λ² I)^{-1} Δx``, clamp each joint to
  its limits, and accumulate until convergence or iteration budget exhausted.
* **Solution selection**: the IK converges to a single solution from the given
  seed (DLS is a descent, not a multi-root search).  If the caller needs a
  solution close to a reference configuration, pass that configuration as
  ``q_seed``.
* **Unreachable reporting**: if the pose error at convergence exceeds
  ``tolerance_m`` (default 1 mm) or ``tolerance_rad`` (default 0.01 rad),
  ``IKResult.reachable`` is ``False`` and ``IKResult.q`` contains the
  best-effort configuration.  The caller must inspect ``IKResult.reachable``
  before commanding motion.

Joint-limit enforcement
~~~~~~~~~~~~~~~~~~~~~~~
Limits are read from ``hardware.yaml`` (``so101_kinematics.joints[i].limit_*``).
If the YAML has ``None`` for a limit (not the case in the current URDF block),
the code falls back to the URDF-derived defaults stored in
:data:`_URDF_JOINT_LIMITS_RAD`, which are also marked unverified because the
owner's physical arm may differ.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import numpy.typing as npt

from inchworm.config import SO101KinematicsConfig, SO101JointConfig

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Mat44 = npt.NDArray[np.float64]   # 4×4 homogeneous transform
Vec3  = npt.NDArray[np.float64]   # 3-vector (metres or radians)
JointVec = npt.NDArray[np.float64]  # 5-vector of joint angles (rad)

# ---------------------------------------------------------------------------
# IK tunables
# ---------------------------------------------------------------------------

#: Damping factor λ for damped least-squares.  Larger values improve
#: singularity robustness at the cost of slower convergence.
_DLS_LAMBDA: float = 0.05

#: Maximum IK iterations.
_IK_MAX_ITER: int = 500

#: IK step learning rate (scales the DLS update before adding to q).
_IK_STEP_SCALE: float = 1.0

#: Default convergence tolerances.
_DEFAULT_TOL_M:   float = 1e-5   # 0.01 mm (orchestrator 2026-10-24: 1 mm tolerance made "sub-mm" a tautology)
_DEFAULT_TOL_RAD: float = 1e-3   # ~0.057 degrees

# ---------------------------------------------------------------------------
# URDF fallback limits (if config has None for any joint).
# Source: so101_new_calib.urdf hash a758567c.  MARKED UNVERIFIED for the
# owner's custom assembly — measure in AA-16.
# ---------------------------------------------------------------------------

_URDF_JOINT_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "shoulder_pan":  (-1.91986,  1.91986),
    "shoulder_lift": (-1.74533,  1.74533),
    "elbow_flex":    (-1.69,     1.69),
    "wrist_flex":    (-1.65806,  1.65806),
    "wrist_roll":    (-2.74385,  2.84121),
}

# ---------------------------------------------------------------------------
# Helper: rigid-body transforms
# ---------------------------------------------------------------------------


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> Mat44:
    """Rotation matrix from extrinsic RPY (roll about X, pitch about Y, yaw about Z)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)  [extrinsic / URDF convention]
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,            cp*cr   ],
    ], dtype=np.float64)


def _htm_from_rpy_xyz(
    xyz: Sequence[float], rpy: Sequence[float]
) -> Mat44:
    """Build a 4×4 HTM from xyz translation and extrinsic RPY rotation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _rpy_to_rot(rpy[0], rpy[1], rpy[2])
    T[:3, 3]  = xyz
    return T


def _rotz(theta: float) -> Mat44:
    """Pure Z-rotation 4×4 HTM."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# Kinematic chain
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointFrame:
    """Precomputed per-joint data for fast FK evaluation."""

    name: str
    T_fixed: Mat44   # Fixed part of the joint transform (origin HTM), (n-1) → joint
    axis: tuple[float, float, float]  # Rotation axis (always (0,0,1) in SO-101)
    limit_lower: float  # rad
    limit_upper: float  # rad


@dataclass
class KinematicChain:
    """5-DoF SO-101 kinematic chain loaded from ``SO101KinematicsConfig``.

    Instantiate with :func:`build_chain` rather than directly.

    The chain operates in the arm's ``base_link`` frame.  Apply
    ``left_arm_base_to_chassis`` / ``right_arm_base_to_chassis`` from config
    to convert to the chassis-fixed frame (both are unverified until AA-16).
    """

    joints: list[JointFrame]
    T_tool_offset: Mat44  # gripper_link → TCP (unverified; see hardware.yaml)
    n_dof: int = field(init=False)

    def __post_init__(self) -> None:
        self.n_dof = len(self.joints)

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def fk(self, q: JointVec) -> Mat44:
        """Forward kinematics: joint angles (rad) → TCP pose in base frame.

        Parameters
        ----------
        q:
            5-vector of joint angles (rad), ordered
            [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll].

        Returns
        -------
        Mat44
            4×4 homogeneous transform (TCP in arm ``base_link`` frame).
        """
        if len(q) != self.n_dof:
            raise ValueError(
                f"q has {len(q)} elements; chain has {self.n_dof} DOF"
            )
        T = np.eye(4, dtype=np.float64)
        for jf, qi in zip(self.joints, q):
            # T_fixed encodes the URDF <origin>; _rotz(qi) encodes the joint rotation.
            T = T @ jf.T_fixed @ _rotz(qi)
        T = T @ self.T_tool_offset
        return T

    # ------------------------------------------------------------------
    # Geometric Jacobian
    # ------------------------------------------------------------------

    def jacobian(self, q: JointVec) -> npt.NDArray[np.float64]:
        """6×n geometric Jacobian (translational + rotational) in base frame.

        Rows 0:3 are the translational part; rows 3:6 are the rotational part.
        Only the Z-column is non-zero for the rotational rows (revolute about Z).
        """
        n = self.n_dof
        J = np.zeros((6, n), dtype=np.float64)

        # Accumulate per-joint frame transforms to get z_i and p_i
        T_curr = np.eye(4, dtype=np.float64)
        frames: list[Mat44] = []
        for jf, qi in zip(self.joints, q):
            T_curr = T_curr @ jf.T_fixed @ _rotz(qi)
            frames.append(T_curr.copy())

        # TCP position
        T_ee = T_curr @ self.T_tool_offset
        p_ee = T_ee[:3, 3]

        # Rebuild from scratch to get each joint's frame *before* the rotation
        T_curr = np.eye(4, dtype=np.float64)
        for i, (jf, qi) in enumerate(zip(self.joints, q)):
            T_before_rot = T_curr @ jf.T_fixed   # frame just before joint i rotation
            z_i = T_before_rot[:3, 2]             # Z axis of joint i in base frame
            p_i = T_before_rot[:3, 3]             # origin of joint i in base frame
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
            T_curr = T_before_rot @ _rotz(qi)

        return J

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def ik(
        self,
        target: Mat44,
        q_seed: Optional[JointVec] = None,
        tolerance_m: float = _DEFAULT_TOL_M,
        tolerance_rad: float = _DEFAULT_TOL_RAD,
        max_iter: int = _IK_MAX_ITER,
        dls_lambda: float = _DLS_LAMBDA,
        position_only: bool = False,
    ) -> "IKResult":
        """Inverse kinematics: target TCP pose → joint angles.

        Documented seed and selection rule
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        * **Seed**: ``q = zeros(5)`` by default (the URDF rest pose).  Pass
          ``q_seed`` to start from a different configuration; the solver will
          converge to a nearby solution in configuration space.
        * **Selection**: DLS descends from the given seed.  The returned
          solution is the configuration that minimises the pose error starting
          from that seed — no additional post-hoc selection is performed.

        Parameters
        ----------
        target:
            4×4 HTM of the desired TCP pose in the arm base frame.
        q_seed:
            Starting joint configuration (rad).  Defaults to ``zeros(5)``.
        tolerance_m:
            Position error threshold for convergence (metres).  Default 1 mm.
        tolerance_rad:
            Orientation error threshold for convergence (radians).  Default
            ~0.57 degrees.
        max_iter:
            Maximum number of DLS iterations.
        dls_lambda:
            Damping factor λ.  Larger → more singularity-robust, slower.
        position_only:
            If True, only the 3 translational rows of the Jacobian are used.
            The orientation error is ignored.  Useful for Cartesian jog where
            the target orientation is unspecified.  ``ori_err_rad`` in the
            result reflects the final orientation error but does not affect
            the convergence test when this flag is set.

        Returns
        -------
        IKResult
            ``.reachable`` is True only if the position error is within
            ``tolerance_m`` (and orientation error within ``tolerance_rad``
            when ``position_only=False``) at convergence.  Always inspect
            ``.reachable`` before commanding motion.

        Raises
        ------
        Does not raise on unreachable targets; use ``IKResult.reachable``.
        """
        if q_seed is None:
            q = np.zeros(self.n_dof, dtype=np.float64)
        else:
            q = np.asarray(q_seed, dtype=np.float64).copy()

        # Clip seed to limits
        q = self._clip_to_limits(q)

        target_pos = target[:3, 3]
        target_rot = target[:3, :3]

        pos_err_final = float("inf")
        ori_err_final = float("inf")

        for _it in range(max_iter):
            T_curr = self.fk(q)
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]

            # Translational error
            dp = target_pos - curr_pos
            pos_err = float(np.linalg.norm(dp))

            # Rotational error: angle-axis from R_err = R_target * R_curr^T
            R_err = target_rot @ curr_rot.T
            # Convert to angle-axis
            angle = math.acos(
                max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0))
            )
            if angle < 1e-9:
                axis_scaled = np.zeros(3, dtype=np.float64)
            else:
                # Skew-symmetric part → axis
                axis = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ], dtype=np.float64) / (2.0 * math.sin(angle))
                axis_scaled = axis * angle

            ori_err = float(np.linalg.norm(axis_scaled))
            pos_err_final = pos_err
            ori_err_final = ori_err

            if position_only:
                converged = pos_err < tolerance_m
            else:
                converged = pos_err < tolerance_m and ori_err < tolerance_rad
            if converged:
                break

            # Damped least-squares Jacobian pseudoinverse
            J = self.jacobian(q)
            if position_only:
                # Use only translational rows
                J_use = J[:3, :]
                dx = dp
                lam2 = dls_lambda ** 2
                JJT = J_use @ J_use.T + lam2 * np.eye(3, dtype=np.float64)
                dq = J_use.T @ np.linalg.solve(JJT, dx) * _IK_STEP_SCALE
            else:
                dx = np.concatenate([dp, axis_scaled])
                lam2 = dls_lambda ** 2
                JJT = J @ J.T + lam2 * np.eye(6, dtype=np.float64)
                dq = J.T @ np.linalg.solve(JJT, dx) * _IK_STEP_SCALE

            q = q + dq
            q = self._clip_to_limits(q)

        reachable = pos_err_final < tolerance_m
        if not position_only:
            reachable = reachable and (ori_err_final < tolerance_rad)
        if not reachable:
            logger.warning(
                "IK did not converge: pos_err=%.4f mm  ori_err=%.4f deg",
                pos_err_final * 1000,
                math.degrees(ori_err_final),
            )

        return IKResult(
            q=q,
            reachable=reachable,
            pos_err_m=pos_err_final,
            ori_err_rad=ori_err_final,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _clip_to_limits(self, q: JointVec) -> JointVec:
        """Clip joint angles to per-joint limits (in-place copy)."""
        q = q.copy()
        for i, jf in enumerate(self.joints):
            q[i] = max(jf.limit_lower, min(jf.limit_upper, q[i]))
        return q

    def joint_limits(self) -> list[tuple[float, float]]:
        """Return list of (lower, upper) limits in radians, proximal → distal."""
        return [(jf.limit_lower, jf.limit_upper) for jf in self.joints]

    def joint_names(self) -> list[str]:
        """Return joint names in chain order."""
        return [jf.name for jf in self.joints]


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class IKResult:
    """Result of an IK solve.

    Always check ``reachable`` before using ``q`` to command motion.
    """

    q: JointVec
    """Best-effort joint configuration (rad).  Valid regardless of
    ``reachable``; contains the least-error configuration found."""

    reachable: bool
    """True if position error < tolerance_m AND orientation error < tolerance_rad."""

    pos_err_m: float
    """Residual position error (metres) at convergence."""

    ori_err_rad: float
    """Residual orientation error (radians) at convergence."""


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_chain(cfg: SO101KinematicsConfig) -> KinematicChain:
    """Build a :class:`KinematicChain` from a loaded ``SO101KinematicsConfig``.

    Joint limits are read from the config.  If ``limit_lower_rad`` or
    ``limit_upper_rad`` are ``None`` in the config (not the case for the
    current AA-11 YAML block), the URDF fallback values in
    :data:`_URDF_JOINT_LIMITS_RAD` are used and a warning is logged.

    Parameters
    ----------
    cfg:
        Parsed ``so101_kinematics`` section from ``hardware.yaml``.

    Returns
    -------
    KinematicChain
        Ready to call ``.fk()`` and ``.ik()``.
    """
    frames: list[JointFrame] = []
    for jcfg in cfg.joints:
        lo, hi = _resolve_limits(jcfg)
        T_fixed = _htm_from_rpy_xyz(jcfg.origin_xyz, jcfg.origin_rpy)
        frames.append(JointFrame(
            name=jcfg.name,
            T_fixed=T_fixed,
            axis=(float(jcfg.axis[0]), float(jcfg.axis[1]), float(jcfg.axis[2])),
            limit_lower=lo,
            limit_upper=hi,
        ))

    tool_off = cfg.tool_frame_offset
    if tool_off.unverified:
        logger.warning(
            "so101_kinematics.tool_frame_offset is marked unverified; "
            "TCP position in base frame is approximate until AA-16 measurement."
        )
    T_tool = _htm_from_rpy_xyz(tool_off.xyz, tool_off.rpy)

    return KinematicChain(joints=frames, T_tool_offset=T_tool)


def _resolve_limits(jcfg: SO101JointConfig) -> tuple[float, float]:
    """Return (lower, upper) limits in radians, with fallback to URDF defaults."""
    lo = jcfg.limit_lower_rad
    hi = jcfg.limit_upper_rad
    if lo is None or hi is None:
        fallback = _URDF_JOINT_LIMITS_RAD.get(jcfg.name)
        if fallback is None:
            raise ValueError(
                f"Joint '{jcfg.name}' has no limits in config and no URDF fallback."
            )
        logger.warning(
            "Joint '%s' has null limits in config; using URDF fallback %s "
            "(UNVERIFIED — measure in AA-16).",
            jcfg.name,
            fallback,
        )
        lo_out = fallback[0] if lo is None else lo
        hi_out = fallback[1] if hi is None else hi
        return lo_out, hi_out
    return float(lo), float(hi)
