"""Scanner camera and laser-plane calibration — AA-30.

This module implements the **software half** of scanner calibration.  All
heavy computation runs on synthetic or captured data; it never touches real
hardware itself.  The calibration script (``scripts/calibrate_scanner.py``)
is responsible for acquiring captures with the live camera and laser.

Calibration pipeline
--------------------

1. **Camera intrinsics** (:func:`calibrate_camera_intrinsics`):
   Finds chessboard or ChArUco corners in a list of grayscale frames, then
   calls :func:`cv2.calibrateCamera`.  Reports the per-image reprojection
   error and the overall RMS reprojection error.

   Target documented in :data:`CHESSBOARD_PATTERN` and
   :data:`CHARUCO_BOARD_PARAMS` (see below).

2. **Laser-plane fit** (:func:`fit_laser_plane`):
   Accepts a list of 3-D point clouds, each obtained by intersecting
   laser-line pixels (:func:`~inchworm.laser_line.extract_laser_line`) with a
   known calibration-target plane at a measured standoff.  Fits a single plane
   through all valid points using SVD.  Returns the unit normal, the scalar
   offset ``d``, and per-point residuals (mm).

3. **Sensitivity** (:func:`compute_sensitivity`):
   Computes the height-per-pixel sensitivity at the design standoff (see
   :data:`inchworm.constants.LASER_STANDOFF_MM`)
   from the fitted laser plane and the calibrated intrinsics, then compares it
   against the design value imported from
   :mod:`inchworm.constants`.

4. **Hand-eye calibration** (:func:`calibrate_hand_eye`):
   Determines the rigid transform from the camera/laser assembly to the
   DexArm tool frame.  Uses :func:`cv2.calibrateHandEye` when available
   (requires OpenCV ≥ 4.0.0 with the robot module), or a built-in
   Tsai–Lenz SVD-based fallback that produces equivalent results for the
   expected number of poses.

5. **Save / load** (:func:`save_calibration`, :func:`load_calibration`):
   Serialise/deserialise calibration results to/from a Python ``dict`` (which
   the caller can write to YAML or JSON).  ``load_calibration`` warns when
   calibration is stale (older than :data:`STALE_CALIBRATION_DAYS` days) or
   when the file is missing.

   The result dict schema populates :class:`~inchworm.laser_line.LaserCalibration`
   so that :mod:`inchworm.laser_line.triangulate_profile` can use it directly.

Calibration target
------------------
**Chessboard** (primary): 9 × 6 inner corners (i.e. a 10 × 7 square grid),
25 mm square size.  Print ``assets/chessboard_9x6_25mm.pdf`` on flat stock
and mount on a rigid backing.

**ChArUco** (alternative, recommended for partial occlusion):
7 × 5 squares, 35 mm chessboard side, 25 mm ArUco marker side,
dictionary ``DICT_4X4_50``.  Print ``assets/charuco_7x5_35mm.pdf``.

Design values for the scanner assembly (from doc-1 §5, via
:mod:`inchworm.constants`):

- Laser-to-camera angle: see :data:`inchworm.constants.LASER_TO_CAMERA_ANGLE_DEG`
- Baseline offset: see :data:`inchworm.constants.LASER_BASELINE_OFFSET_MM`
- Design standoff: see :data:`inchworm.constants.LASER_STANDOFF_MM`
- Design sensitivity: see :data:`inchworm.constants.LASER_HEIGHT_SENSITIVITY_MM_PER_PX`

Unknown physical values are stored as ``None`` / ``unverified: true`` in the
output dict; live mode fails closed if they are not populated.

References
----------
- doc-1 §3, §5, §10
- AA-30, AA-31
"""

from __future__ import annotations

import logging
import math
import warnings
from dataclasses import dataclass, field
from datetime import date, datetime, timezone
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants: calibration target documentation and stale-calibration threshold
# ---------------------------------------------------------------------------

#: Days after which calibration is considered stale and a warning is emitted.
STALE_CALIBRATION_DAYS: int = 90

#: Chessboard calibration target: (columns, rows) of **inner corners**.
#: A 10×7 printed board gives 9×6 inner corners.
CHESSBOARD_PATTERN: Tuple[int, int] = (9, 6)

#: Chessboard square size in mm (physical, measured on the printed target).
CHESSBOARD_SQUARE_MM: float = 25.0

#: ChArUco board parameters (alternative target).
#: dictionary: cv2.aruco.DICT_4X4_50, cols=7, rows=5,
#: squareLengthMeters=0.035, markerLengthMeters=0.025.
CHARUCO_BOARD_PARAMS: Dict = {
    "cols": 7,
    "rows": 5,
    "squareLengthM": 0.035,
    "markerLengthM": 0.025,
    "dictionary": "DICT_4X4_50",
}

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class CalibrationError(RuntimeError):
    """Base class for scanner calibration errors."""


class InsufficientDataError(CalibrationError):
    """Raised when calibration is attempted with too few valid frames/points."""


class UnverifiedCalibrationLoadError(CalibrationError):
    """Raised in live mode when no calibration file is found or it is unverified."""


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------


@dataclass
class IntrinsicsResult:
    """Camera intrinsic calibration result.

    Attributes
    ----------
    camera_matrix:
        3×3 float64 camera matrix ``[[fx, 0, cx],[0, fy, cy],[0, 0, 1]]``.
    dist_coeffs:
        Distortion coefficients ``[k1, k2, p1, p2, ...]`` (float64 1-D).
    rms_reprojection_error_px:
        RMS reprojection error in pixels (lower is better; < 1.0 px is good).
    per_image_reprojection_error_px:
        Per-image RMS reprojection error (list of floats, same order as input).
    n_images_used:
        Number of images where corners were detected and used.
    target_description:
        Human-readable description of the calibration target used.
    unverified:
        True until a hardware session runs the full calibration.
    """

    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rms_reprojection_error_px: float
    per_image_reprojection_error_px: List[float]
    n_images_used: int
    target_description: str = "chessboard 9x6 inner corners, 25mm squares"
    unverified: bool = True


@dataclass
class LaserPlaneResult:
    """Laser-plane fit result.

    Attributes
    ----------
    plane_normal:
        Unit normal of the laser plane in the **camera frame** (3-vector,
        float64).  Plane equation: ``dot(plane_normal, P) + plane_d = 0``.
    plane_d:
        Scalar offset of the plane equation (mm).
    rms_residual_mm:
        RMS of per-point distances to the fitted plane (mm).
    max_residual_mm:
        Maximum per-point distance to the fitted plane (mm).
    n_points:
        Number of 3-D points used in the fit.
    unverified:
        True until fitted from real hardware observations.
    """

    plane_normal: np.ndarray
    plane_d: float
    rms_residual_mm: float
    max_residual_mm: float
    n_points: int
    unverified: bool = True


@dataclass
class SensitivityResult:
    """Sensitivity analysis at the design standoff.

    Attributes
    ----------
    sensitivity_mm_per_px:
        Measured height sensitivity at ``standoff_mm`` (mm/px).
    design_sensitivity_mm_per_px:
        Design value from :data:`inchworm.constants.LASER_HEIGHT_SENSITIVITY_MM_PER_PX`.
    standoff_mm:
        Standoff distance used for the sensitivity calculation (mm).
    relative_deviation_pct:
        ``(measured / design - 1) * 100`` (%).  Positive means more sensitive
        than designed.
    discrepancy_note:
        Human-readable explanation of any significant deviation (|deviation| > 10%).
    """

    sensitivity_mm_per_px: float
    design_sensitivity_mm_per_px: float
    standoff_mm: float
    relative_deviation_pct: float
    discrepancy_note: str = ""


@dataclass
class HandEyeResult:
    """Camera-to-DexArm-tool transform result.

    Attributes
    ----------
    T_tool_camera:
        4×4 homogeneous transform from **camera frame** to **DexArm tool
        frame** (float64, row-major).  Equivalent to
        :attr:`~inchworm.laser_line.LaserCalibration.T_print_camera`.
    method:
        Name of the hand-eye algorithm used (e.g. ``"tsai"``).
    n_poses:
        Number of DexArm pose pairs used.
    unverified:
        True until fitted from real hardware observations.
    """

    T_tool_camera: np.ndarray
    method: str
    n_poses: int
    unverified: bool = True


@dataclass
class ScannerCalibration:
    """Complete scanner calibration output, populating :class:`~inchworm.laser_line.LaserCalibration`.

    This is the AA-30 deliverable.  :func:`save_calibration` serialises it to
    a dict; :func:`load_calibration` reconstructs it and populates a
    :class:`~inchworm.laser_line.LaserCalibration` for use in AA-31.

    Attributes
    ----------
    intrinsics:
        Camera intrinsic calibration result (or ``None`` if not yet done).
    laser_plane:
        Laser-plane fit result (or ``None`` if not yet done).
    sensitivity:
        Sensitivity analysis result (or ``None`` if not yet computed).
    hand_eye:
        Hand-eye transform result (or ``None`` if not yet done).
    calibration_date:
        ISO date string (``YYYY-MM-DD``) of the calibration session.
    target_description:
        Description of the calibration target used.
    notes:
        Free-form notes from the operator.
    unverified:
        True when any sub-calibration is missing or unverified.
    """

    intrinsics: Optional[IntrinsicsResult] = None
    laser_plane: Optional[LaserPlaneResult] = None
    sensitivity: Optional[SensitivityResult] = None
    hand_eye: Optional[HandEyeResult] = None
    calibration_date: str = ""
    target_description: str = ""
    notes: str = ""
    unverified: bool = True

    def to_laser_calibration(self) -> "inchworm.laser_line.LaserCalibration":
        """Construct a :class:`~inchworm.laser_line.LaserCalibration` from this result.

        Returns the design-default calibration with any fitted components
        replaced by the calibrated values.  Fields that are still ``None``
        remain as design defaults and ``unverified=True``.
        """
        from inchworm.laser_line import LaserCalibration

        # Start from design defaults.
        kwargs: Dict = {}

        if self.intrinsics is not None:
            m = self.intrinsics.camera_matrix
            kwargs["fx"] = float(m[0, 0])
            kwargs["fy"] = float(m[1, 1])
            kwargs["cx"] = float(m[0, 2])
            kwargs["cy"] = float(m[1, 2])
            kwargs["distortion"] = self.intrinsics.dist_coeffs.astype(np.float64)

        if self.laser_plane is not None:
            kwargs["plane_normal"] = self.laser_plane.plane_normal.copy()
            kwargs["plane_d"] = self.laser_plane.plane_d

        if self.hand_eye is not None:
            kwargs["T_print_camera"] = self.hand_eye.T_tool_camera.copy()

        kwargs["unverified"] = self.unverified

        return LaserCalibration(**kwargs)


# ---------------------------------------------------------------------------
# 1. Camera intrinsics calibration
# ---------------------------------------------------------------------------


def calibrate_camera_intrinsics(
    frames: Sequence[np.ndarray],
    *,
    pattern: Tuple[int, int] = CHESSBOARD_PATTERN,
    square_mm: float = CHESSBOARD_SQUARE_MM,
    use_charuco: bool = False,
    flags: int = 0,
) -> IntrinsicsResult:
    """Calibrate camera intrinsics and distortion from chessboard frames.

    Uses :func:`cv2.findChessboardCorners` (or ChArUco corners when
    ``use_charuco=True``) followed by :func:`cv2.calibrateCamera`.

    Parameters
    ----------
    frames:
        List of uint8 grayscale numpy arrays.  At least 4 frames are required;
        more frames (10–20) give better conditioning.
    pattern:
        ``(cols, rows)`` inner corner count for chessboard detection.
        Default: :data:`CHESSBOARD_PATTERN` (9, 6).
    square_mm:
        Physical square size in mm.  Used to build the 3-D object point grid.
    use_charuco:
        If ``True``, use ChArUco corner detection instead of plain chessboard.
        Requires ``cv2.aruco`` to be available.
    flags:
        OpenCV ``calibrateCamera`` flags (e.g. ``cv2.CALIB_FIX_PRINCIPAL_POINT``).
        Zero = no extra constraints.

    Returns
    -------
    IntrinsicsResult
        Intrinsic matrix, distortion coefficients, and reprojection errors.

    Raises
    ------
    InsufficientDataError
        If fewer than 4 frames yield corner detections.
    ImportError
        If ``cv2`` (or ``cv2.aruco`` for ChArUco) is not installed.
    """
    import cv2  # type: ignore[import]

    if len(frames) == 0:
        raise InsufficientDataError("No frames supplied for intrinsics calibration.")

    cols, rows = pattern
    # 3-D object points for a flat chessboard lying at z=0.
    obj_template = np.zeros((rows * cols, 3), dtype=np.float32)
    obj_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None  # (width, height)

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    n_detected = 0
    for frame in frames:
        if frame.ndim != 2 or frame.dtype != np.uint8:
            raise ValueError(
                f"Each frame must be 2-D uint8 grayscale; "
                f"got shape={frame.shape} dtype={frame.dtype}"
            )
        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        if not use_charuco:
            ret, corners = cv2.findChessboardCorners(frame, (cols, rows), None)
            if ret:
                # Sub-pixel refinement.
                corners_sub = cv2.cornerSubPix(
                    frame, corners, (11, 11), (-1, -1), criteria
                )
                obj_points.append(obj_template)
                img_points.append(corners_sub)
                n_detected += 1
        else:
            # ChArUco path — requires cv2.aruco.
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            board = aruco.CharucoBoard(
                (CHARUCO_BOARD_PARAMS["cols"], CHARUCO_BOARD_PARAMS["rows"]),
                CHARUCO_BOARD_PARAMS["squareLengthM"] * 1000,  # mm
                CHARUCO_BOARD_PARAMS["markerLengthM"] * 1000,  # mm
                dictionary,
            )
            detector_params = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, detector_params)
            corners_aruco, ids_aruco, _ = detector.detectMarkers(frame)
            if ids_aruco is not None and len(ids_aruco) > 0:
                charuco_params = aruco.CharucoParameters()
                charuco_detector = aruco.CharucoDetector(
                    board,
                    charuco_params,
                    detector_params,
                )
                _, ch_corners, ch_ids = charuco_detector.detectBoard(frame)
                if ch_ids is not None and len(ch_ids) >= 4:
                    obj_pts, img_pts = board.matchImagePoints(ch_corners, ch_ids)
                    if obj_pts is not None and len(obj_pts) >= 4:
                        obj_points.append(obj_pts.astype(np.float32))
                        img_points.append(img_pts.astype(np.float32))
                        n_detected += 1

    if n_detected < 4:
        raise InsufficientDataError(
            f"Only {n_detected} frame(s) yielded corner detections; "
            f"at least 4 are required for calibrateCamera."
        )

    assert image_size is not None
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points,
        img_points,
        image_size,
        None,
        None,
        flags=flags,
    )

    # Per-image reprojection error.
    per_image_errors: List[float] = []
    for i, (obj_pts, img_pts_det, rvec, tvec) in enumerate(
        zip(obj_points, img_points, rvecs, tvecs)
    ):
        proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        err = float(np.sqrt(np.mean((img_pts_det - proj_pts) ** 2)))
        per_image_errors.append(err)

    target_str = "chessboard" if not use_charuco else "ChArUco"
    target_desc = (
        f"{target_str} {cols}×{rows} inner corners, {square_mm:.0f} mm squares, "
        f"{n_detected} images used"
    )
    log.info(
        "[AA-30] intrinsics calibrated: rms_px=%.4f n_images=%d target=%s",
        rms,
        n_detected,
        target_desc,
    )

    return IntrinsicsResult(
        camera_matrix=camera_matrix.astype(np.float64),
        dist_coeffs=dist_coeffs.flatten().astype(np.float64),
        rms_reprojection_error_px=float(rms),
        per_image_reprojection_error_px=per_image_errors,
        n_images_used=n_detected,
        target_description=target_desc,
        unverified=True,  # unverified until a real hardware session
    )


# ---------------------------------------------------------------------------
# 2. Laser-plane fit (SVD)
# ---------------------------------------------------------------------------


def fit_laser_plane(
    point_clouds: Sequence[np.ndarray],
) -> LaserPlaneResult:
    """Fit a laser plane through 3-D points using SVD.

    Accepts one or more point clouds (arrays of shape ``(N, 3)``), pools all
    valid (non-NaN) points, centres them, and performs SVD to find the best-fit
    plane.  The plane is defined by its unit normal and scalar offset
    ``d`` in the camera frame:  ``dot(normal, P) + d = 0``.

    The 3-D points are obtained by intersecting laser-line pixel positions
    (from :func:`~inchworm.laser_line.extract_laser_line`) with known
    calibration-target planes at multiple standoffs, using the camera
    intrinsics and the known target pose (measured by chessboard pose
    estimation at each standoff).

    Parameters
    ----------
    point_clouds:
        One or more ``(N, 3)`` float64 arrays of 3-D points in the
        **camera frame** (mm).  NaN rows are ignored.

    Returns
    -------
    LaserPlaneResult
        Fitted plane normal, offset, and residual statistics.

    Raises
    ------
    InsufficientDataError
        If fewer than 3 valid points are available after NaN filtering.
    """
    all_points_list: List[np.ndarray] = []
    for cloud in point_clouds:
        cloud = np.asarray(cloud, dtype=np.float64)
        if cloud.ndim == 1:
            cloud = cloud.reshape(1, 3)
        valid_mask = np.all(np.isfinite(cloud), axis=1)
        if valid_mask.sum() > 0:
            all_points_list.append(cloud[valid_mask])

    if not all_points_list:
        raise InsufficientDataError("No valid (non-NaN) 3-D points provided for plane fit.")

    pts = np.vstack(all_points_list)
    n = len(pts)
    if n < 3:
        raise InsufficientDataError(
            f"Plane fit requires at least 3 points; got {n}."
        )

    # Demean.
    centroid = pts.mean(axis=0)
    pts_c = pts - centroid

    # SVD: the plane normal is the left singular vector corresponding to the
    # smallest singular value (the direction of minimum variance).
    _, s_vals, Vt = np.linalg.svd(pts_c, full_matrices=False)
    # Last row of Vt corresponds to smallest singular value.
    normal = Vt[-1]  # shape (3,)
    normal /= np.linalg.norm(normal)

    # Ensure the normal points generally "toward" the camera (+Z direction).
    if normal[2] < 0:
        normal = -normal

    # Plane offset: n·P + d = 0  →  d = -n·centroid.
    d = float(-np.dot(normal, centroid))

    # Residuals: signed distance from each point to the plane.
    residuals = np.abs(pts @ normal + d)
    rms_residual = float(np.sqrt(np.mean(residuals**2)))
    max_residual = float(residuals.max())

    log.info(
        "[AA-30] laser_plane_fit: n_points=%d rms_residual_mm=%.4f max_residual_mm=%.4f "
        "normal=[%.4f, %.4f, %.4f] d=%.4f",
        n, rms_residual, max_residual, normal[0], normal[1], normal[2], d,
    )

    return LaserPlaneResult(
        plane_normal=normal,
        plane_d=d,
        rms_residual_mm=rms_residual,
        max_residual_mm=max_residual,
        n_points=n,
        unverified=True,
    )


# ---------------------------------------------------------------------------
# 3. Sensitivity at design standoff
# ---------------------------------------------------------------------------


def compute_sensitivity(
    intrinsics: IntrinsicsResult,
    laser_plane: LaserPlaneResult,
    *,
    standoff_mm: Optional[float] = None,
) -> SensitivityResult:
    """Compute height-per-pixel sensitivity at ``standoff_mm`` from calibrated geometry.

    The sensitivity is the change in real-world height (along the camera Z axis)
    per pixel displacement of the laser line in the image row direction, at the
    specified standoff.

    Method:
      1. Pick a pixel column at the principal point (cx, cy).
      2. Compute the 3-D point where the laser plane intersects the ray through
         (cx, cy): this gives the baseline point P0 at the standoff.
      3. Construct a surface at the standoff depth Z0 = P0[2] (a plane z = Z0).
      4. Compute the 3-D point where the laser plane intersects the ray through
         (cx, cy + 1): this gives a second point P1 slightly offset in row direction.
      5. The Z-axis difference |P0[2] - P1[2]| is the height change per pixel in
         the row direction.

    Parameters
    ----------
    intrinsics:
        Calibrated camera intrinsics.
    laser_plane:
        Fitted laser plane.
    standoff_mm:
        Standoff distance for the sensitivity calculation (mm).  If ``None``,
        uses :data:`~inchworm.constants.LASER_STANDOFF_MM`.

    Returns
    -------
    SensitivityResult
        Measured sensitivity, design sensitivity, and discrepancy note.
    """
    from inchworm.constants import (
        LASER_HEIGHT_SENSITIVITY_MM_PER_PX,
        LASER_STANDOFF_MM,
    )

    if standoff_mm is None:
        standoff_mm = float(LASER_STANDOFF_MM)
    design_sensitivity = float(LASER_HEIGHT_SENSITIVITY_MM_PER_PX)

    fx = float(intrinsics.camera_matrix[0, 0])
    fy = float(intrinsics.camera_matrix[1, 1])
    cx = float(intrinsics.camera_matrix[0, 2])
    cy = float(intrinsics.camera_matrix[1, 2])
    n = laser_plane.plane_normal
    d_plane = laser_plane.plane_d

    def _ray_plane(u: float, v: float) -> Optional[np.ndarray]:
        """Intersect camera ray through pixel (u, v) with the laser plane."""
        xn = (u - cx) / fx
        yn = (v - cy) / fy
        ray = np.array([xn, yn, 1.0], dtype=np.float64)
        ray /= np.linalg.norm(ray)
        denom = float(np.dot(n, ray))
        if abs(denom) < 1e-12:
            return None
        t = -d_plane / denom
        if t <= 0:
            return None
        return t * ray

    # Ray through centre column (cx, cy) — baseline point.
    p0 = _ray_plane(cx, cy)
    # Ray through (cx, cy + 1) — one pixel down in row direction.
    p1 = _ray_plane(cx, cy + 1.0)

    if p0 is None or p1 is None:
        log.warning(
            "[AA-30] compute_sensitivity: ray–plane intersection failed near principal point; "
            "returning design value."
        )
        return SensitivityResult(
            sensitivity_mm_per_px=design_sensitivity,
            design_sensitivity_mm_per_px=design_sensitivity,
            standoff_mm=standoff_mm,
            relative_deviation_pct=0.0,
            discrepancy_note="ray–plane intersection failed; design value used",
        )

    # Height sensitivity: change in Z between the two ray intersections.
    # |dZ| per 1 pixel in row direction.
    delta_z = abs(p1[2] - p0[2])

    if delta_z < 1e-9:
        # Fallback: try the column direction (u + 1) instead.
        # For a laser plane with no Y component in the normal, the row direction
        # does not produce Z variation (the laser is a vertical line in the
        # image at the principal column). Use the column direction instead,
        # or compute analytically from the plane geometry.
        # Analytical: sensitivity = sin(angle) / fy  * Z / cos(angle)
        # where angle is the laser tilt angle.
        # From the plane normal n = [-sin(a), 0, cos(a)]:
        # dZ/d(row) = -(n[1] / (fy * n[2])) * Z   ... but n[1]=0 for our geometry.
        # The correct sensitivity for a vertical-slit laser in this geometry:
        # The laser line appears at row r. dZ/dr = dx/dr * (dZ/dx)
        # For our geometry, the laser moves along X as height changes, not Y.
        # The correct formulation: use the column direction.
        p2 = _ray_plane(cx + 1.0, cy)
        if p2 is not None:
            delta_z = abs(p2[2] - p0[2])
            log.info("[AA-30] compute_sensitivity: using column direction (n_y=0 geometry).")

    if delta_z < 1e-9:
        log.warning(
            "[AA-30] compute_sensitivity: delta_z still near zero; "
            "plane geometry may not produce Z variation from image displacement. "
            "Returning design value."
        )
        return SensitivityResult(
            sensitivity_mm_per_px=design_sensitivity,
            design_sensitivity_mm_per_px=design_sensitivity,
            standoff_mm=standoff_mm,
            relative_deviation_pct=0.0,
            discrepancy_note="delta_z near zero; design value used",
        )

    sensitivity = delta_z  # mm per px

    relative_deviation = (sensitivity / design_sensitivity - 1.0) * 100.0

    note = ""
    if abs(relative_deviation) > 10.0:
        note = (
            f"Deviation {relative_deviation:.1f}% from design value exceeds ±10%. "
            "Possible causes: laser angle or standoff differ from design values; "
            "re-measure assembly geometry and re-run calibration."
        )
        log.warning("[AA-30] %s", note)
    else:
        log.info(
            "[AA-30] sensitivity: measured=%.4f mm/px  design=%.4f mm/px  "
            "deviation=%.1f%%",
            sensitivity, design_sensitivity, relative_deviation,
        )

    return SensitivityResult(
        sensitivity_mm_per_px=sensitivity,
        design_sensitivity_mm_per_px=design_sensitivity,
        standoff_mm=standoff_mm,
        relative_deviation_pct=relative_deviation,
        discrepancy_note=note,
    )


# ---------------------------------------------------------------------------
# 4. Hand-eye calibration (camera-to-DexArm-tool transform)
# ---------------------------------------------------------------------------


def _tsai_lenz_hand_eye(
    R_gripper2base: List[np.ndarray],
    t_gripper2base: List[np.ndarray],
    R_target2cam: List[np.ndarray],
    t_target2cam: List[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    """Tsai–Lenz hand-eye calibration (SVD-based, pure NumPy).

    Solves AX = XB for X (the camera-to-gripper transform) where:
    - A = relative transform between two gripper poses
    - B = relative transform between the corresponding two camera/target poses

    This is a simplified Tsai–Lenz implementation sufficient for the
    expected number of DexArm poses (8–20).  For production use, prefer
    ``cv2.calibrateHandEye`` when available.

    Parameters
    ----------
    R_gripper2base, t_gripper2base:
        Rotation matrices and translation vectors for each gripper pose
        (base ← gripper).
    R_target2cam, t_target2cam:
        Rotation matrices and translation vectors for each target-to-camera
        transform (camera ← target).

    Returns
    -------
    R_cam2gripper, t_cam2gripper:
        Rotation and translation of the camera frame in the gripper frame.
    """
    n = len(R_gripper2base)
    assert n >= 3, f"Need at least 3 pose pairs; got {n}"
    assert n == len(R_target2cam), "Mismatch between number of gripper and camera poses"

    # Collect A = R_gi^-1 @ R_gj, B = R_ci @ R_cj^-1 for all pairs (i, j).
    A_list: List[np.ndarray] = []
    B_list: List[np.ndarray] = []
    for i in range(n):
        for j in range(i + 1, n):
            Rgi = R_gripper2base[i]
            Rgj = R_gripper2base[j]
            Rci = R_target2cam[i]
            Rcj = R_target2cam[j]
            # Relative motion of gripper.
            A = Rgi.T @ Rgj
            # Relative motion of camera (inverse: cam ← target at i, j).
            B = Rci @ Rcj.T
            A_list.append(A)
            B_list.append(B)

    # Solve R_cx using Kronecker product and SVD.
    # Sum over all pairs: S = sum(skew(a+b) @ skew(a-b)) ... simplified.
    # Use the direct method: stack modified Rodrigues equations.
    M = np.zeros((3, 3), dtype=np.float64)
    for A, B in zip(A_list, B_list):
        # Convert to axis-angle (Rodrigues vector).
        a = _rotation_to_axis_angle(A)
        b = _rotation_to_axis_angle(B)
        # Normalise to get unit axis.
        a_norm = np.linalg.norm(a)
        b_norm = np.linalg.norm(b)
        if a_norm < 1e-12 or b_norm < 1e-12:
            continue
        an = a / a_norm
        bn = b / b_norm
        # Modified Rodrigues: accumulate outer product for SVD.
        M += np.outer(bn, an)

    # SVD gives the optimal rotation.
    U, _, Vt = np.linalg.svd(M)
    R_cx = U @ Vt
    # Enforce proper rotation.
    if np.linalg.det(R_cx) < 0:
        U[:, -1] *= -1
        R_cx = U @ Vt

    # Solve for t_cx using the linear system A_R * t_cx = c for each pair.
    rows_t: List[np.ndarray] = []
    rhs_t: List[np.ndarray] = []
    for i in range(n):
        for j in range(i + 1, n):
            Rgi = R_gripper2base[i]
            tgi = t_gripper2base[i]
            Rgj = R_gripper2base[j]
            tgj = t_gripper2base[j]
            Rci = R_target2cam[i]
            tci = t_target2cam[i]
            Rcj = R_target2cam[j]
            tcj = t_target2cam[j]
            # Relative gripper motion (i ← j).
            A_R = Rgi.T @ Rgj
            A_t = Rgi.T @ (tgj - tgi)
            # Relative camera motion (i ← j).
            B_R = Rci @ Rcj.T
            B_t = Rci @ (-Rcj.T @ tcj) + tci
            # (A_R - I) * t_cx = R_cx * B_t - A_t
            rows_t.append(A_R - np.eye(3))
            rhs_t.append(R_cx @ B_t - A_t)

    A_mat = np.vstack(rows_t)
    b_vec = np.concatenate(rhs_t)
    t_cx, _, _, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)

    return R_cx, t_cx.reshape(3, 1)


def _rotation_to_axis_angle(R: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to an axis-angle vector (Rodrigues)."""
    # angle = acos((tr(R) - 1) / 2)
    trace = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(trace)
    if abs(angle) < 1e-12:
        return np.zeros(3, dtype=np.float64)
    # Axis = (R - R^T) / (2 sin(angle))
    axis = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    ) / (2.0 * math.sin(angle))
    return axis * angle


def calibrate_hand_eye(
    R_gripper2base: Sequence[np.ndarray],
    t_gripper2base: Sequence[np.ndarray],
    R_target2cam: Sequence[np.ndarray],
    t_target2cam: Sequence[np.ndarray],
    *,
    method: str = "tsai",
) -> HandEyeResult:
    """Determine the camera-to-DexArm-tool transform (hand-eye calibration).

    Wraps :func:`cv2.calibrateHandEye` when available (OpenCV ≥ 4.0.0 with
    the calib3d module), or falls back to a built-in Tsai–Lenz SVD
    implementation.

    The DexArm poses come from :class:`~inchworm.dexarm.DexArmDriver` via
    :meth:`~inchworm.dexarm.DexArmDriver.get_position` combined with an
    estimated orientation (the DexArm is planar so the end-effector
    orientation can be estimated from the G-code commands).

    Parameters
    ----------
    R_gripper2base:
        Sequence of 3×3 rotation matrices (gripper ← base, one per pose).
    t_gripper2base:
        Sequence of 3-vectors (mm) for each gripper pose.
    R_target2cam:
        Sequence of 3×3 rotation matrices (camera ← target, one per pose).
        Obtained from chessboard pose estimation at each DexArm position.
    t_target2cam:
        Sequence of 3-vectors (mm) for each camera/target pose.
    method:
        Hand-eye algorithm.  ``"tsai"`` (default) uses the built-in Tsai–Lenz
        fallback.  Other values are forwarded to ``cv2.calibrateHandEye``
        flag names (e.g. ``"park"``, ``"horaud"``).  If ``cv2`` does not
        expose ``calibrateHandEye``, the built-in fallback is always used.

    Returns
    -------
    HandEyeResult
        4×4 transform ``T_tool_camera`` and metadata.

    Raises
    ------
    InsufficientDataError
        If fewer than 3 pose pairs are provided.
    """
    n = len(R_gripper2base)
    if n < 3:
        raise InsufficientDataError(
            f"Hand-eye calibration needs at least 3 pose pairs; got {n}."
        )

    R_list = [np.asarray(r, dtype=np.float64) for r in R_gripper2base]
    t_list = [np.asarray(t, dtype=np.float64).reshape(3, 1) for t in t_gripper2base]
    Rc_list = [np.asarray(r, dtype=np.float64) for r in R_target2cam]
    tc_list = [np.asarray(t, dtype=np.float64).reshape(3, 1) for t in t_target2cam]

    R_cx: Optional[np.ndarray] = None
    t_cx: Optional[np.ndarray] = None
    used_method = method

    # Attempt cv2.calibrateHandEye if available.
    try:
        import cv2  # type: ignore[import]
        if hasattr(cv2, "calibrateHandEye"):
            _METHOD_MAP = {
                "tsai": cv2.CALIB_HAND_EYE_TSAI,
                "park": cv2.CALIB_HAND_EYE_PARK,
                "horaud": cv2.CALIB_HAND_EYE_HORAUD,
                "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
                "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
            }
            cv2_method = _METHOD_MAP.get(method.lower(), cv2.CALIB_HAND_EYE_TSAI)
            rvecs_g = [cv2.Rodrigues(R)[0] for R in R_list]
            rvecs_c = [cv2.Rodrigues(R)[0] for R in Rc_list]
            R_cx, t_cx = cv2.calibrateHandEye(
                rvecs_g, t_list, rvecs_c, tc_list,
                method=cv2_method,
            )
            used_method = f"cv2.calibrateHandEye/{method}"
        else:
            raise AttributeError("calibrateHandEye not available")
    except (ImportError, AttributeError) as exc:
        log.info(
            "[AA-30] cv2.calibrateHandEye not available (%s); using built-in Tsai–Lenz fallback.",
            exc,
        )

    if R_cx is None:
        R_cx, t_cx = _tsai_lenz_hand_eye(
            R_list,
            [t.reshape(3) for t in t_list],
            Rc_list,
            [t.reshape(3) for t in tc_list],
        )
        used_method = "tsai_numpy_fallback"

    # Build 4×4 homogeneous transform.
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_cx
    T[:3, 3] = t_cx.reshape(3)

    log.info(
        "[AA-30] hand_eye_calibrated: method=%s n_poses=%d",
        used_method, n,
    )

    return HandEyeResult(
        T_tool_camera=T,
        method=used_method,
        n_poses=n,
        unverified=True,
    )


# ---------------------------------------------------------------------------
# 5. Save / load calibration
# ---------------------------------------------------------------------------

_CALIBRATION_DATE_FMT = "%Y-%m-%d"


def save_calibration(calibration: ScannerCalibration) -> Dict:
    """Serialise a :class:`ScannerCalibration` to a plain Python dict.

    The dict is suitable for serialisation to YAML or JSON and storage in
    ``robot/config/hardware.yaml`` under the ``scanner_calibration`` key or
    in a separate calibration file.

    The dict schema is::

        {
            "calibration_date": "YYYY-MM-DD",
            "target_description": str,
            "notes": str,
            "unverified": bool,
            "intrinsics": {
                "camera_matrix": [[...], ...],   # 3×3
                "dist_coeffs": [...],             # 1-D list
                "rms_reprojection_error_px": float,
                "per_image_reprojection_error_px": [...],
                "n_images_used": int,
                "target_description": str,
                "unverified": bool,
            } | null,
            "laser_plane": {
                "normal": [nx, ny, nz],
                "d": float,
                "rms_residual_mm": float,
                "max_residual_mm": float,
                "n_points": int,
                "unverified": bool,
            } | null,
            "sensitivity": {
                "sensitivity_mm_per_px": float,
                "design_sensitivity_mm_per_px": float,
                "standoff_mm": float,
                "relative_deviation_pct": float,
                "discrepancy_note": str,
            } | null,
            "hand_eye": {
                "T_tool_camera": [[...], ...],   # 4×4
                "method": str,
                "n_poses": int,
                "unverified": bool,
            } | null,
        }

    Parameters
    ----------
    calibration:
        Calibration result to serialise.

    Returns
    -------
    dict
        Serialisable calibration dictionary.
    """
    d: Dict = {
        "calibration_date": calibration.calibration_date or "",
        "target_description": calibration.target_description,
        "notes": calibration.notes,
        "unverified": calibration.unverified,
        "intrinsics": None,
        "laser_plane": None,
        "sensitivity": None,
        "hand_eye": None,
    }

    if calibration.intrinsics is not None:
        intr = calibration.intrinsics
        d["intrinsics"] = {
            "camera_matrix": intr.camera_matrix.tolist(),
            "dist_coeffs": intr.dist_coeffs.tolist(),
            "rms_reprojection_error_px": intr.rms_reprojection_error_px,
            "per_image_reprojection_error_px": intr.per_image_reprojection_error_px,
            "n_images_used": intr.n_images_used,
            "target_description": intr.target_description,
            "unverified": intr.unverified,
        }

    if calibration.laser_plane is not None:
        lp = calibration.laser_plane
        d["laser_plane"] = {
            "normal": lp.plane_normal.tolist(),
            "d": lp.plane_d,
            "rms_residual_mm": lp.rms_residual_mm,
            "max_residual_mm": lp.max_residual_mm,
            "n_points": lp.n_points,
            "unverified": lp.unverified,
        }

    if calibration.sensitivity is not None:
        s = calibration.sensitivity
        d["sensitivity"] = {
            "sensitivity_mm_per_px": s.sensitivity_mm_per_px,
            "design_sensitivity_mm_per_px": s.design_sensitivity_mm_per_px,
            "standoff_mm": s.standoff_mm,
            "relative_deviation_pct": s.relative_deviation_pct,
            "discrepancy_note": s.discrepancy_note,
        }

    if calibration.hand_eye is not None:
        he = calibration.hand_eye
        d["hand_eye"] = {
            "T_tool_camera": he.T_tool_camera.tolist(),
            "method": he.method,
            "n_poses": he.n_poses,
            "unverified": he.unverified,
        }

    return d


def load_calibration(
    data: Optional[Dict],
    *,
    live: bool = False,
) -> ScannerCalibration:
    """Deserialise a calibration dict produced by :func:`save_calibration`.

    Emits a WARNING when:
    - ``data`` is ``None`` (no calibration file found).
    - The ``calibration_date`` is absent or the calibration is older than
      :data:`STALE_CALIBRATION_DAYS` days.
    - ``unverified`` is ``True``.

    In live mode (``live=True``) any of the above conditions raises
    :class:`UnverifiedCalibrationLoadError` instead of merely warning.

    Parameters
    ----------
    data:
        Calibration dict (from YAML/JSON), or ``None`` if no file was found.
    live:
        When ``True``, raise instead of warn on missing/stale/unverified data.

    Returns
    -------
    ScannerCalibration
        Populated calibration result.

    Raises
    ------
    UnverifiedCalibrationLoadError
        In live mode when calibration is missing, stale, or unverified.
    """
    if data is None:
        msg = (
            "[AA-30] No scanner calibration data found. "
            "Run scripts/calibrate_scanner.py --live to perform calibration."
        )
        if live:
            raise UnverifiedCalibrationLoadError(msg)
        log.warning(msg)
        return ScannerCalibration(unverified=True)

    # Stale check.
    cal_date_str = data.get("calibration_date", "")
    if cal_date_str:
        try:
            cal_date = datetime.strptime(cal_date_str, _CALIBRATION_DATE_FMT).date()
            age_days = (date.today() - cal_date).days
            if age_days > STALE_CALIBRATION_DAYS:
                msg = (
                    f"[AA-30] Scanner calibration is stale: "
                    f"calibrated {cal_date_str} ({age_days} days ago > "
                    f"{STALE_CALIBRATION_DAYS} day limit). "
                    "Re-run scripts/calibrate_scanner.py --live."
                )
                if live:
                    raise UnverifiedCalibrationLoadError(msg)
                log.warning(msg)
        except ValueError:
            log.warning("[AA-30] calibration_date %r is not in YYYY-MM-DD format.", cal_date_str)
    else:
        msg = "[AA-30] Calibration date is missing; treating as stale."
        if live:
            raise UnverifiedCalibrationLoadError(msg)
        log.warning(msg)

    # Unverified flag.
    unverified = bool(data.get("unverified", True))
    if unverified:
        msg = (
            "[AA-30] Scanner calibration is unverified (unverified=true). "
            "Owner must review and set unverified: false after validation."
        )
        if live:
            raise UnverifiedCalibrationLoadError(msg)
        log.warning(msg)

    # Intrinsics.
    intrinsics = None
    if data.get("intrinsics") is not None:
        intr_d = data["intrinsics"]
        intrinsics = IntrinsicsResult(
            camera_matrix=np.array(intr_d["camera_matrix"], dtype=np.float64),
            dist_coeffs=np.array(intr_d["dist_coeffs"], dtype=np.float64),
            rms_reprojection_error_px=float(intr_d["rms_reprojection_error_px"]),
            per_image_reprojection_error_px=list(intr_d["per_image_reprojection_error_px"]),
            n_images_used=int(intr_d["n_images_used"]),
            target_description=str(intr_d.get("target_description", "")),
            unverified=bool(intr_d.get("unverified", True)),
        )

    # Laser plane.
    laser_plane = None
    if data.get("laser_plane") is not None:
        lp_d = data["laser_plane"]
        laser_plane = LaserPlaneResult(
            plane_normal=np.array(lp_d["normal"], dtype=np.float64),
            plane_d=float(lp_d["d"]),
            rms_residual_mm=float(lp_d["rms_residual_mm"]),
            max_residual_mm=float(lp_d["max_residual_mm"]),
            n_points=int(lp_d["n_points"]),
            unverified=bool(lp_d.get("unverified", True)),
        )

    # Sensitivity.
    sensitivity = None
    if data.get("sensitivity") is not None:
        s_d = data["sensitivity"]
        sensitivity = SensitivityResult(
            sensitivity_mm_per_px=float(s_d["sensitivity_mm_per_px"]),
            design_sensitivity_mm_per_px=float(s_d["design_sensitivity_mm_per_px"]),
            standoff_mm=float(s_d["standoff_mm"]),
            relative_deviation_pct=float(s_d["relative_deviation_pct"]),
            discrepancy_note=str(s_d.get("discrepancy_note", "")),
        )

    # Hand-eye.
    hand_eye = None
    if data.get("hand_eye") is not None:
        he_d = data["hand_eye"]
        hand_eye = HandEyeResult(
            T_tool_camera=np.array(he_d["T_tool_camera"], dtype=np.float64),
            method=str(he_d.get("method", "unknown")),
            n_poses=int(he_d.get("n_poses", 0)),
            unverified=bool(he_d.get("unverified", True)),
        )

    cal = ScannerCalibration(
        intrinsics=intrinsics,
        laser_plane=laser_plane,
        sensitivity=sensitivity,
        hand_eye=hand_eye,
        calibration_date=cal_date_str,
        target_description=str(data.get("target_description", "")),
        notes=str(data.get("notes", "")),
        unverified=unverified,
    )

    log.info(
        "[AA-30] calibration loaded: date=%s unverified=%s intrinsics=%s "
        "laser_plane=%s sensitivity=%s hand_eye=%s",
        cal_date_str,
        unverified,
        intrinsics is not None,
        laser_plane is not None,
        sensitivity is not None,
        hand_eye is not None,
    )
    return cal


# ---------------------------------------------------------------------------
# Synthetic chessboard rendering helpers (for tests and dry-run)
# ---------------------------------------------------------------------------


def render_synthetic_chessboard(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: Tuple[int, int],
    n_frames: int = 12,
    *,
    pattern: Tuple[int, int] = CHESSBOARD_PATTERN,
    square_mm: float = CHESSBOARD_SQUARE_MM,
    noise_px: float = 0.3,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """Render synthetic chessboard images through known intrinsics.

    Projects the 3-D chessboard corners through ``camera_matrix`` and
    ``dist_coeffs`` at randomised poses, then draws a grayscale chessboard.
    Used for testing :func:`calibrate_camera_intrinsics` on known ground truth.

    Parameters
    ----------
    camera_matrix:
        3×3 ground-truth camera matrix.
    dist_coeffs:
        Ground-truth distortion coefficients.
    image_size:
        ``(width, height)`` of the synthetic frames.
    n_frames:
        Number of frames to render.
    pattern:
        ``(cols, rows)`` inner corner count.
    square_mm:
        Square size (mm).
    noise_px:
        Sub-pixel noise added to projected corner positions (px std-dev).
    rng:
        Optional random generator.  ``None`` uses seed 0.

    Returns
    -------
    frames:
        List of uint8 grayscale numpy arrays.
    projected_corners:
        List of ``(N, 1, 2)`` float32 arrays of noisy projected corners
        (one per frame), as :func:`cv2.findChessboardCorners` would return.
    true_corners:
        List of noiseless projected corners (ground truth for error computation).
    """
    import cv2  # type: ignore[import]

    if rng is None:
        rng = np.random.default_rng(0)

    w, h = image_size
    cols, rows = pattern
    n_corners = cols * rows

    # 3-D object points at z=0.
    obj_pts = np.zeros((n_corners, 3), dtype=np.float32)
    obj_pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    frames: List[np.ndarray] = []
    proj_corners_list: List[np.ndarray] = []
    true_corners_list: List[np.ndarray] = []

    for _ in range(n_frames):
        # Random pose: look at the board from various angles.
        rvec = rng.uniform(-0.4, 0.4, size=(3, 1)).astype(np.float32)
        tvec = np.array(
            [
                rng.uniform(-50, 50),
                rng.uniform(-50, 50),
                rng.uniform(200, 400),
            ],
            dtype=np.float32,
        ).reshape(3, 1)

        # Project corners.
        img_pts, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, camera_matrix.astype(np.float32), dist_coeffs.astype(np.float32)
        )
        true_corners_list.append(img_pts.copy())

        # Add sub-pixel noise.
        noisy_pts = img_pts + rng.normal(0.0, noise_px, size=img_pts.shape).astype(np.float32)
        proj_corners_list.append(noisy_pts)

        # Render the chessboard image (draw grid around projected corners).
        frame = np.full((h, w), 180, dtype=np.uint8)
        for ci in range(n_corners):
            x, y = noisy_pts[ci, 0]
            r, c = int(round(y)), int(round(x))
            if 0 <= r < h and 0 <= c < w:
                # Draw filled squares alternating black/white.
                row_i = ci // cols
                col_i = ci % cols
                sq_half = max(1, int(square_mm / 4))
                r0 = max(0, r - sq_half)
                r1 = min(h, r + sq_half)
                c0 = max(0, c - sq_half)
                c1 = min(w, c + sq_half)
                shade = 0 if (row_i + col_i) % 2 == 0 else 255
                frame[r0:r1, c0:c1] = shade

        frames.append(frame)

    return frames, proj_corners_list, true_corners_list


def render_synthetic_laser_points(
    plane_normal: np.ndarray,
    plane_d: float,
    camera_matrix: np.ndarray,
    *,
    n_points: int = 200,
    noise_mm: float = 0.1,
    v_spread_px: float = 5.0,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Render synthetic 3-D laser points on a known plane in the camera frame.

    Generates ``n_points`` normalised camera rays (spread across the image
    width, with a small row-direction jitter for SVD conditioning),
    intersects them with the given plane, and adds Gaussian noise.

    Parameters
    ----------
    plane_normal:
        Unit normal of the true plane (camera frame).
    plane_d:
        Scalar offset of the plane equation (mm).
    camera_matrix:
        3×3 camera matrix ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``.
    n_points:
        Number of 3-D points to generate.
    noise_mm:
        Gaussian noise standard deviation (mm) added to each point.
    v_spread_px:
        Std-dev of row-direction jitter (pixels) around the principal row.
        Non-zero ensures 3-D points span the plane in Y for SVD conditioning.
    rng:
        Optional random generator.

    Returns
    -------
    np.ndarray
        Shape ``(n_points, 3)`` float64 array of 3-D points.
    """
    if rng is None:
        rng = np.random.default_rng(42)

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    # Spread pixel columns across the image width.
    u_vals = np.linspace(10, cx * 2 - 10, n_points)
    # Small row jitter for SVD conditioning (points not all co-linear).
    v_vals = cy + rng.normal(0.0, max(v_spread_px, 1.0), size=n_points)

    n = np.asarray(plane_normal, dtype=np.float64)
    points = np.zeros((n_points, 3), dtype=np.float64)

    for i in range(n_points):
        xn = (u_vals[i] - cx) / fx
        yn = (v_vals[i] - cy) / fy
        ray = np.array([xn, yn, 1.0])
        ray /= np.linalg.norm(ray)
        denom = float(np.dot(n, ray))
        if abs(denom) < 1e-12:
            points[i] = np.nan
            continue
        t = -plane_d / denom
        if t <= 0:
            points[i] = np.nan
            continue
        p = t * ray
        points[i] = p + rng.normal(0.0, noise_mm, size=3)

    return points
