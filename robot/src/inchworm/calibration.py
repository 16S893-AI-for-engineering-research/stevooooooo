"""AA-16: Chassis-arm-tube calibration solvers and result management.

This module implements three calibration solvers and a loader with
stale-calibration warnings for the inchworm robot:

Solvers
-------
(a) :func:`kabsch_transform`
    Rigid transform from N point correspondences (Kabsch/Umeyama).
    Used for arm-base-to-chassis and DexArm-to-chassis calibration,
    using the arm TCP at known chassis features as fiducials.

(b) :func:`hand_eye_tsai`
    Hand-eye AX=XB calibration (Park-Martin / Tsai-Lenz style).
    Implemented in pure NumPy so it works regardless of the OpenCV build.
    ``cv2.calibrateHandEye`` (method TSAI) is preferred when available.
    Used to determine wrist-camera pose relative to arm end-effector.

(c) :func:`fit_tube_axis`
    Line fit through measured attachment-point centres.
    Expresses the tube axis relative to the lattice index and the
    FrameModel tube pose, as required when the tube is movable.

Result container
----------------
:class:`CalibrationResult` serialises to a hardware.yaml-compatible dict
with an ISO timestamp, residuals, and method name.

Loader warning
--------------
:func:`check_calibration_age` emits a ``warnings.warn`` and an AA-7 event
when calibration is missing or older than the configured max age.

Verification helper
-------------------
:func:`predict_attachment_point` predicts the Cartesian position of a known
attachment point from a calibrated transform and compares it to a measured
position, reporting error per axis.

Safety invariants
-----------------
* No real serial ports or cameras are opened; solvers operate on supplied data.
* All live-data collection is in the companion scripts (dry-run default).
* Motion is never commanded from this module.
"""

from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# Type aliases
# ─────────────────────────────────────────────────────────────────────────────

Mat44 = npt.NDArray[np.float64]   # 4×4 homogeneous transform
Vec3  = npt.NDArray[np.float64]   # 3-vector (metres)

# ─────────────────────────────────────────────────────────────────────────────
# Default stale threshold (days) — overridden by config field
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_MAX_CALIBRATION_AGE_DAYS: int = 90


# ─────────────────────────────────────────────────────────────────────────────
# CalibrationResult
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class CalibrationResult:
    """Result of one calibration step.

    Attributes
    ----------
    transform:
        4×4 homogeneous transform (the calibrated pose).
    method:
        Human-readable name of the solver used
        (e.g. ``"kabsch"``, ``"hand_eye_park_martin"``).
    residual_m:
        RMS residual in metres (point correspondences) or NaN for hand-eye.
    residual_rad:
        RMS residual in radians (rotation part; NaN when not applicable).
    ts_iso:
        ISO 8601 UTC timestamp of when the calibration was performed.
    n_samples:
        Number of correspondence pairs used.
    notes:
        Free-form notes (instruments used, etc.).
    """

    transform: Mat44
    method: str
    residual_m: float
    residual_rad: float = float("nan")
    ts_iso: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    n_samples: int = 0
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """Serialise to a hardware.yaml-compatible dict."""
        return {
            "transform": self.transform.tolist(),
            "method": self.method,
            "residual_m": float(self.residual_m),
            "residual_rad": float(self.residual_rad),
            "ts_iso": self.ts_iso,
            "n_samples": int(self.n_samples),
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "CalibrationResult":
        return cls(
            transform=np.array(d["transform"], dtype=np.float64),
            method=str(d.get("method", "")),
            residual_m=float(d.get("residual_m", float("nan"))),
            residual_rad=float(d.get("residual_rad", float("nan"))),
            ts_iso=str(d.get("ts_iso", "")),
            n_samples=int(d.get("n_samples", 0)),
            notes=str(d.get("notes", "")),
        )


# ─────────────────────────────────────────────────────────────────────────────
# (a) Kabsch / Umeyama rigid transform from point correspondences
# ─────────────────────────────────────────────────────────────────────────────


def kabsch_transform(
    points_src: npt.NDArray[np.float64],
    points_dst: npt.NDArray[np.float64],
) -> Tuple[Mat44, float]:
    """Estimate a rigid transform T such that T @ points_src ≈ points_dst.

    Uses the Kabsch (SVD-based Umeyama) algorithm.  Rotation is constrained
    to SO(3) (det = +1).

    Parameters
    ----------
    points_src:
        (N, 3) array of source points (e.g. TCP positions in arm base frame).
    points_dst:
        (N, 3) array of corresponding destination points (e.g. the same
        features measured in the chassis frame).

    Returns
    -------
    T:
        4×4 homogeneous transform: ``T @ p_src ≈ p_dst``.
    rms_residual_m:
        Root-mean-square Euclidean residual in metres after applying ``T``.

    Raises
    ------
    ValueError
        If fewer than 3 correspondence pairs are supplied (under-determined).
    """
    pts_src = np.asarray(points_src, dtype=np.float64)
    pts_dst = np.asarray(points_dst, dtype=np.float64)

    if pts_src.shape != pts_dst.shape:
        raise ValueError(
            f"points_src shape {pts_src.shape} != points_dst shape {pts_dst.shape}"
        )
    if pts_src.ndim != 2 or pts_src.shape[1] != 3:
        raise ValueError(
            f"Expected (N, 3) arrays; got {pts_src.shape}"
        )
    n = pts_src.shape[0]
    if n < 3:
        raise ValueError(
            f"At least 3 correspondence pairs required (Kabsch); got {n}"
        )

    # Centroid subtraction
    c_src = pts_src.mean(axis=0)
    c_dst = pts_dst.mean(axis=0)
    A = pts_src - c_src
    B = pts_dst - c_dst

    # SVD of cross-covariance matrix
    H = A.T @ B  # 3×3
    U, S, Vt = np.linalg.svd(H)

    # Ensure right-handed rotation (det = +1)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])

    R = Vt.T @ D @ U.T  # 3×3 rotation matrix
    t = c_dst - R @ c_src  # 3-vector translation

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t

    # RMS residual
    pts_src_h = np.hstack([pts_src, np.ones((n, 1))])
    pts_pred = (T @ pts_src_h.T).T[:, :3]
    residuals = np.linalg.norm(pts_pred - pts_dst, axis=1)
    rms = float(np.sqrt(np.mean(residuals ** 2)))

    logger.debug(
        "kabsch_transform: n=%d, rms_residual_m=%.6f", n, rms
    )
    return T, rms


def calibrate_rigid(
    points_src: npt.NDArray[np.float64],
    points_dst: npt.NDArray[np.float64],
    *,
    notes: str = "",
) -> CalibrationResult:
    """Wrapper around :func:`kabsch_transform` that returns a :class:`CalibrationResult`.

    Use this for arm-base-to-chassis and DexArm-to-chassis calibration.

    Parameters
    ----------
    points_src:
        (N, 3) TCP positions in the arm/DexArm base frame.
    points_dst:
        (N, 3) Corresponding positions in the chassis frame (measured from
        observable features, e.g. printed fiducials or chassis bolt centres).
    notes:
        Instrument/procedure notes to embed in the result.
    """
    T, rms = kabsch_transform(points_src, points_dst)
    return CalibrationResult(
        transform=T,
        method="kabsch",
        residual_m=rms,
        n_samples=int(np.asarray(points_src).shape[0]),
        notes=notes,
    )


# ─────────────────────────────────────────────────────────────────────────────
# (b) Hand-eye AX=XB calibration (Park-Martin / Tsai-Lenz)
# ─────────────────────────────────────────────────────────────────────────────


def _rot_to_axisangle(R: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """Convert a 3×3 rotation matrix to an axis-angle vector (Rodrigues form).

    Returns the axis scaled by the angle in radians.  The returned vector has
    magnitude == angle.
    """
    # Clamp trace to valid range to avoid NaN from acos
    trace = float(np.trace(R))
    angle = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
    if abs(angle) < 1e-10:
        return np.zeros(3, dtype=np.float64)
    axis = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    )
    axis /= 2.0 * np.sin(angle)
    return axis * angle


def _axisangle_to_rot(v: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """Convert an axis-angle vector to a 3×3 rotation matrix (Rodrigues)."""
    angle = float(np.linalg.norm(v))
    if angle < 1e-10:
        return np.eye(3, dtype=np.float64)
    axis = v / angle
    K = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def hand_eye_park_martin(
    R_gripper2base: List[npt.NDArray[np.float64]],
    t_gripper2base: List[npt.NDArray[np.float64]],
    R_target2cam: List[npt.NDArray[np.float64]],
    t_target2cam: List[npt.NDArray[np.float64]],
) -> Tuple[Mat44, float, float]:
    """Hand-eye calibration: solve AX = XB (Park & Martin 1994).

    Solves for the wrist-camera extrinsic transform X (camera in end-effector
    frame) given at least 3 non-collinear robot arm poses and corresponding
    fiducial observations.

    This implementation follows the Park-Martin formulation:

    1. For each motion (i→i+1), compute the relative transforms A_i and B_i.
    2. Solve R using the linear system from the Tsai-Lenz/Park-Martin method
       (SVD on the skew-symmetric decomposition).
    3. Solve t given R.

    Parameters
    ----------
    R_gripper2base, t_gripper2base:
        Lists of N rotation matrices (3×3) and translation vectors (3,)
        giving the end-effector (gripper) pose in the robot base frame for
        each capture pose.
    R_target2cam, t_target2cam:
        Lists of N rotation matrices (3×3) and translation vectors (3,)
        giving the calibration target pose in the camera frame for each
        capture pose.

    Returns
    -------
    X:
        4×4 HTM: camera pose in end-effector frame (the hand-eye transform).
    rms_rot_rad:
        Rotation residual RMS (rad) across all motion pairs.
    rms_trans_m:
        Translation residual RMS (m) across all motion pairs.

    Raises
    ------
    ValueError
        If fewer than 3 poses are supplied (under-determined).

    Notes
    -----
    cv2.calibrateHandEye (method CALIB_HAND_EYE_TSAI or CALIB_HAND_EYE_PARK)
    wraps the same algorithm; use this implementation when that function is
    not available in the current OpenCV build.
    """
    n = len(R_gripper2base)
    if n < 3:
        raise ValueError(
            f"hand_eye_park_martin requires >= 3 poses; got {n}"
        )

    Rs_g2b = [np.asarray(R, dtype=np.float64) for R in R_gripper2base]
    ts_g2b = [np.asarray(t, dtype=np.float64).ravel() for t in t_gripper2base]
    Rs_t2c = [np.asarray(R, dtype=np.float64) for R in R_target2cam]
    ts_t2c = [np.asarray(t, dtype=np.float64).ravel() for t in t_target2cam]

    # Build motion pairs (A_i: end-effector motion, B_i: camera motion)
    M = n - 1  # number of pairs
    A_rots: list[npt.NDArray[np.float64]] = []
    A_trans: list[npt.NDArray[np.float64]] = []
    B_rots: list[npt.NDArray[np.float64]] = []
    B_trans: list[npt.NDArray[np.float64]] = []

    for i in range(M):
        # A_i = T_{i+1}^{-1} T_i  (relative motion in gripper-to-base frames)
        Ra = Rs_g2b[i + 1].T @ Rs_g2b[i]
        ta = Rs_g2b[i + 1].T @ (ts_g2b[i] - ts_g2b[i + 1])
        A_rots.append(Ra)
        A_trans.append(ta)

        # B_i = T_{i+1} T_i^{-1} (relative motion in target-to-cam frames)
        Rb = Rs_t2c[i + 1] @ Rs_t2c[i].T
        tb = Rs_t2c[i + 1] @ (-Rs_t2c[i].T @ ts_t2c[i]) + ts_t2c[i + 1]
        B_rots.append(Rb)
        B_trans.append(tb)

    # Solve rotation: build linear system for Rx from axis-angle form
    # alpha_i = log(Ra_i), beta_i = log(Rb_i)
    # Skew(alpha_i + beta_i) Rx_col = Rx_col x (beta_i - alpha_i)  (Tsai)
    # Park-Martin: alpha_i - Rx beta_i = 0  →  sum (alpha_i)(beta_i^T) = S
    # S = U Σ V^T,  Rx = V diag(1,1,det(VU^T)) U^T
    S = np.zeros((3, 3), dtype=np.float64)
    for Ra, Rb in zip(A_rots, B_rots):
        alpha = _rot_to_axisangle(Ra)
        beta = _rot_to_axisangle(Rb)
        S += np.outer(alpha, beta)  # alpha = Rx @ beta → S = sum alpha beta^T

    U, _, Vt = np.linalg.svd(S)
    det_fix = np.linalg.det(U @ Vt)
    D = np.diag([1.0, 1.0, det_fix])
    Rx = U @ D @ Vt  # 3×3 rotation (eye transform rotation part)

    # Solve translation: AX = XB → (Ra - I) tx = Rx tb - ta
    lhs_rows: list[npt.NDArray[np.float64]] = []
    rhs_rows: list[npt.NDArray[np.float64]] = []
    for Ra, ta, Rb, tb in zip(A_rots, A_trans, B_rots, B_trans):
        lhs_rows.append(Ra - np.eye(3, dtype=np.float64))
        rhs_rows.append(Rx @ tb - ta)

    C = np.vstack(lhs_rows)        # (3M, 3)
    d = np.hstack(rhs_rows)        # (3M,)
    tx, _, _, _ = np.linalg.lstsq(C, d, rcond=None)

    # Build 4×4 result
    X = np.eye(4, dtype=np.float64)
    X[:3, :3] = Rx
    X[:3, 3] = tx

    # Compute residuals over all pairs
    rot_errs = []
    trans_errs = []
    for Ra, ta, Rb, tb in zip(A_rots, A_trans, B_rots, B_trans):
        Xa = np.eye(4)
        Xa[:3, :3] = Ra
        Xa[:3, 3] = ta
        Xb = np.eye(4)
        Xb[:3, :3] = Rb
        Xb[:3, 3] = tb
        AX = Xa @ X
        XB = X @ Xb
        dR = AX[:3, :3].T @ XB[:3, :3]
        rot_errs.append(float(np.linalg.norm(_rot_to_axisangle(dR))))
        trans_errs.append(float(np.linalg.norm(AX[:3, 3] - XB[:3, 3])))

    rms_rot = float(np.sqrt(np.mean(np.array(rot_errs) ** 2)))
    rms_trans = float(np.sqrt(np.mean(np.array(trans_errs) ** 2)))

    logger.debug(
        "hand_eye_park_martin: n=%d pairs=%d rms_rot_rad=%.6f rms_trans_m=%.6f",
        n, M, rms_rot, rms_trans,
    )
    return X, rms_rot, rms_trans


def calibrate_hand_eye(
    R_gripper2base: List[npt.NDArray[np.float64]],
    t_gripper2base: List[npt.NDArray[np.float64]],
    R_target2cam: List[npt.NDArray[np.float64]],
    t_target2cam: List[npt.NDArray[np.float64]],
    *,
    notes: str = "",
) -> CalibrationResult:
    """Wrapper around :func:`hand_eye_park_martin` returning a :class:`CalibrationResult`.

    Tries ``cv2.calibrateHandEye`` (CALIB_HAND_EYE_PARK) first when available
    in the current OpenCV build; falls back to the pure-NumPy implementation.

    Use this for wrist-camera-to-end-effector calibration.
    """
    try:
        import cv2  # type: ignore
        if hasattr(cv2, "calibrateHandEye"):
            R_arr = [r.reshape(3, 3) for r in R_gripper2base]
            t_arr = [t.reshape(3, 1) for t in t_gripper2base]
            Rc_arr = [r.reshape(3, 3) for r in R_target2cam]
            tc_arr = [t.reshape(3, 1) for t in t_target2cam]
            R_out, t_out = cv2.calibrateHandEye(
                R_arr, t_arr, Rc_arr, tc_arr,
                method=cv2.CALIB_HAND_EYE_PARK,
            )
            X = np.eye(4, dtype=np.float64)
            X[:3, :3] = R_out
            X[:3, 3] = t_out.ravel()
            method = "cv2_CALIB_HAND_EYE_PARK"
            # compute residuals with our helper
            _, rms_rot, rms_trans = hand_eye_park_martin(
                R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
            )
            return CalibrationResult(
                transform=X,
                method=method,
                residual_m=rms_trans,
                residual_rad=rms_rot,
                n_samples=len(R_gripper2base),
                notes=notes,
            )
    except Exception:
        pass

    X, rms_rot, rms_trans = hand_eye_park_martin(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
    )
    return CalibrationResult(
        transform=X,
        method="hand_eye_park_martin",
        residual_m=rms_trans,
        residual_rad=rms_rot,
        n_samples=len(R_gripper2base),
        notes=notes,
    )


# ─────────────────────────────────────────────────────────────────────────────
# (c) Tube-axis line fit
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class TubeAxisResult:
    """Result of the tube-axis line fit.

    Attributes
    ----------
    axis_direction:
        Unit vector giving the fitted tube axis direction in the chassis frame.
    axis_point:
        A point on the fitted axis (the centroid of the input points)
        in the chassis frame, in metres.
    residual_m:
        RMS deviation of the measured attachment-point centres from the fitted
        axis (metres; zero for collinear points).
    lattice_origin_in_chassis:
        The chassis-frame position estimated for lattice index 0.
        Computed by back-projecting along the axis from the centroid using
        the supplied lattice indices.
    pitch_estimate_m:
        Estimated lattice pitch from the data (metres).  Should match
        ``ATTACHMENT_PITCH_MM / 1000`` within measurement error.
    n_samples:
        Number of input attachment-point centres used.
    ts_iso:
        ISO 8601 UTC calibration timestamp.
    method:
        Method name string.
    notes:
        Free-form notes.
    """

    axis_direction: Vec3          # unit vector (chassis frame)
    axis_point: Vec3              # point on axis / centroid (chassis frame, metres)
    residual_m: float             # RMS deviation from fitted axis
    lattice_origin_in_chassis: Vec3   # chassis-frame position of lattice index 0
    pitch_estimate_m: float       # estimated attachment pitch (metres)
    n_samples: int
    ts_iso: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    method: str = "pca_line_fit"
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "axis_direction": self.axis_direction.tolist(),
            "axis_point": self.axis_point.tolist(),
            "residual_m": float(self.residual_m),
            "lattice_origin_in_chassis": self.lattice_origin_in_chassis.tolist(),
            "pitch_estimate_m": float(self.pitch_estimate_m),
            "n_samples": int(self.n_samples),
            "ts_iso": self.ts_iso,
            "method": self.method,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TubeAxisResult":
        return cls(
            axis_direction=np.array(d["axis_direction"], dtype=np.float64),
            axis_point=np.array(d["axis_point"], dtype=np.float64),
            residual_m=float(d.get("residual_m", float("nan"))),
            lattice_origin_in_chassis=np.array(
                d["lattice_origin_in_chassis"], dtype=np.float64
            ),
            pitch_estimate_m=float(d.get("pitch_estimate_m", float("nan"))),
            n_samples=int(d.get("n_samples", 0)),
            ts_iso=str(d.get("ts_iso", "")),
            method=str(d.get("method", "pca_line_fit")),
            notes=str(d.get("notes", "")),
        )


def fit_tube_axis(
    attachment_centers_m: npt.NDArray[np.float64],
    lattice_indices: npt.NDArray[np.int64],
    *,
    notes: str = "",
) -> TubeAxisResult:
    """Fit a line through measured attachment-point centres.

    The tube is movable: positions are measured with the tube at an arbitrary
    but fixed position.  The fit gives the axis direction and the chassis-frame
    position of each lattice point (relative to the lattice index).

    The estimated lattice origin (index 0) and pitch are computed from the
    projections of each measured point onto the fitted axis.

    Parameters
    ----------
    attachment_centers_m:
        (N, 3) array of measured attachment-point centre positions in the
        chassis frame (metres).  N >= 2.
    lattice_indices:
        (N,) integer array giving the lattice index for each measured point.
        Indices need not be consecutive or start at 0.

    Returns
    -------
    TubeAxisResult
        Fitted axis, residuals, estimated lattice origin and pitch.

    Raises
    ------
    ValueError
        If fewer than 2 points are supplied.
    """
    pts = np.asarray(attachment_centers_m, dtype=np.float64)
    idx = np.asarray(lattice_indices, dtype=np.int64)

    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"Expected (N, 3) array; got {pts.shape}")
    n = pts.shape[0]
    if n < 2:
        raise ValueError(f"At least 2 points required for line fit; got {n}")
    if idx.shape[0] != n:
        raise ValueError(
            f"lattice_indices length {idx.shape[0]} != points length {n}"
        )

    # PCA-based line fit: first principal component gives the axis direction
    centroid = pts.mean(axis=0)
    A = pts - centroid
    _, _, Vt = np.linalg.svd(A)
    axis = Vt[0]  # first right singular vector (direction of max variance)
    # Ensure consistent orientation (positive dot with mean index direction)
    projs = A @ axis
    idx_centred = idx - idx.mean()
    if np.dot(projs, idx_centred) < 0:
        axis = -axis

    # RMS residual: distance from each point to the line
    proj_vecs = np.outer(projs, axis)
    deviations = A - proj_vecs
    residual = float(np.sqrt(np.mean(np.linalg.norm(deviations, axis=1) ** 2)))

    # Estimate pitch and lattice origin
    # Point_k = origin + k * pitch * axis
    # projs[i] ≈ (idx[i] - idx_mean) * pitch  →  pitch = mean(projs / idx_centred)
    mask = np.abs(idx_centred) > 0
    if mask.sum() >= 1:
        pitch_estimates = projs[mask] / idx_centred[mask].astype(float)
        pitch_m = float(np.median(pitch_estimates))
    else:
        # All same index (degenerate); pitch undefined
        pitch_m = float("nan")

    # Lattice origin (index 0) = centroid - idx_mean * pitch * axis
    if not np.isnan(pitch_m):
        origin = centroid - float(idx.mean()) * pitch_m * axis
    else:
        origin = centroid.copy()

    logger.debug(
        "fit_tube_axis: n=%d residual_m=%.6f pitch_m=%.6f",
        n, residual, pitch_m,
    )
    return TubeAxisResult(
        axis_direction=axis,
        axis_point=centroid,
        residual_m=residual,
        lattice_origin_in_chassis=origin,
        pitch_estimate_m=pitch_m,
        n_samples=n,
        notes=notes,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Stale-calibration checker
# ─────────────────────────────────────────────────────────────────────────────


def check_calibration_age(
    result: Optional[CalibrationResult],
    *,
    label: str = "calibration",
    max_age_days: int = DEFAULT_MAX_CALIBRATION_AGE_DAYS,
    session_logger: Any = None,
) -> None:
    """Warn if ``result`` is missing or older than ``max_age_days``.

    Emits a ``warnings.warn`` (category :class:`CalibrationStalenessWarning`)
    and an AA-7 structured event when calibration is absent or stale.

    Parameters
    ----------
    result:
        A :class:`CalibrationResult`, or ``None`` if not yet measured.
    label:
        Human-readable label for the calibration (e.g. ``"left_arm_base"``).
    max_age_days:
        Maximum acceptable age in days.  0 = disabled (always fresh).
    session_logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    """
    if result is None:
        msg = (
            f"AA-16 calibration missing for '{label}': "
            "arm-base-to-chassis transform has not been measured. "
            "Run scripts/calibrate_arm_base.py --live (owner supervision required)."
        )
        warnings.warn(msg, CalibrationStalenessWarning, stacklevel=2)
        _slog(
            session_logger,
            severity="WARNING",
            event_type="calibration_missing",
            payload={"label": label},
        )
        return

    if not result.ts_iso:
        # No timestamp recorded — treat as missing
        msg = (
            f"AA-16 calibration for '{label}' has no timestamp; "
            "treating as missing. Re-run the calibration script."
        )
        warnings.warn(msg, CalibrationStalenessWarning, stacklevel=2)
        _slog(
            session_logger,
            severity="WARNING",
            event_type="calibration_no_timestamp",
            payload={"label": label},
        )
        return

    if max_age_days <= 0:
        return  # age check disabled

    try:
        ts = datetime.fromisoformat(result.ts_iso)
    except ValueError:
        warnings.warn(
            f"AA-16 calibration for '{label}' has unparseable timestamp "
            f"{result.ts_iso!r}; treating as stale.",
            CalibrationStalenessWarning,
            stacklevel=2,
        )
        _slog(
            session_logger,
            severity="WARNING",
            event_type="calibration_bad_timestamp",
            payload={"label": label, "ts_iso": result.ts_iso},
        )
        return

    now = datetime.now(timezone.utc)
    # Make ts timezone-aware for comparison
    if ts.tzinfo is None:
        ts = ts.replace(tzinfo=timezone.utc)
    age_days = (now - ts).total_seconds() / 86400.0

    if age_days > max_age_days:
        msg = (
            f"AA-16 calibration for '{label}' is stale: "
            f"{age_days:.1f} days old (max {max_age_days} days). "
            "Re-run the calibration script to refresh."
        )
        warnings.warn(msg, CalibrationStalenessWarning, stacklevel=2)
        _slog(
            session_logger,
            severity="WARNING",
            event_type="calibration_stale",
            payload={
                "label": label,
                "age_days": round(age_days, 1),
                "max_age_days": max_age_days,
                "ts_iso": result.ts_iso,
            },
        )


class CalibrationStalenessWarning(UserWarning):
    """Emitted when a calibration result is missing or older than the configured threshold."""


# ─────────────────────────────────────────────────────────────────────────────
# Verification helper: predict attachment-point position
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class AttachmentPointPrediction:
    """Comparison of a predicted and measured attachment-point position.

    Attributes
    ----------
    predicted_m:
        Predicted chassis-frame position from the calibrated transform (metres).
    measured_m:
        Measured chassis-frame position (e.g. probed with calipers) (metres).
    error_m:
        Per-axis error vector ``predicted - measured`` (metres).
    error_norm_m:
        Euclidean norm of the per-axis error (metres).
    """

    predicted_m: Vec3
    measured_m: Vec3
    error_m: Vec3
    error_norm_m: float


def predict_attachment_point(
    T_chassis_base: Mat44,
    tcp_in_base_m: Vec3,
    measured_in_chassis_m: Vec3,
) -> AttachmentPointPrediction:
    """Predict an attachment-point position and compare it to measurement.

    Given the calibrated base-to-chassis transform and the FK-predicted TCP
    position in the arm base frame, predict the TCP in the chassis frame and
    compare it to the measured chassis-frame position.

    Parameters
    ----------
    T_chassis_base:
        4×4 HTM: arm base in chassis frame (from :func:`calibrate_rigid`).
    tcp_in_base_m:
        TCP position (3-vector, metres) in the arm base frame, from FK at
        the arm configuration used for the verification measurement.
    measured_in_chassis_m:
        Measured attachment-point (or TCP) position in the chassis frame
        (3-vector, metres), e.g. probed with calibrated calipers.

    Returns
    -------
    AttachmentPointPrediction
        Per-axis error and Euclidean error norm.
    """
    tcp_h = np.array([*tcp_in_base_m, 1.0], dtype=np.float64)
    predicted = (T_chassis_base @ tcp_h)[:3]
    measured = np.asarray(measured_in_chassis_m, dtype=np.float64)
    err = predicted - measured
    return AttachmentPointPrediction(
        predicted_m=predicted,
        measured_m=measured,
        error_m=err,
        error_norm_m=float(np.linalg.norm(err)),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────


def _slog(
    session_logger: Any,
    *,
    severity: str,
    event_type: str,
    payload: Optional[Dict[str, Any]] = None,
) -> None:
    """Emit to AA-7 SessionLogger if present, else to stdlib logger."""
    if session_logger is not None:
        try:
            session_logger.emit(
                role="orchestrator",
                subsystem="calibration",
                event_type=event_type,
                severity=severity,  # type: ignore[arg-type]
                payload=payload or {},
            )
        except Exception:
            pass
    else:
        lvl = getattr(logging, severity, logging.WARNING)
        logger.log(lvl, "calibration.%s %s", event_type, payload or "")
