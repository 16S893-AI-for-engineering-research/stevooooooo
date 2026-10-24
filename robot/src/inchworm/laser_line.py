"""Laser-line extraction and height-profile triangulation — AA-31.

This module implements two stages of the leveling pipeline:

1. **Extraction** (:func:`extract_laser_line`):
   Find the sub-pixel row position of the laser line in each column of a
   single-channel uint8 frame.  The method is:

   a. Subtract a per-column background estimate (row-wise median of the
      full column, masked outside the search window).
   b. Find the column maximum (peak).
   c. Fit a 3-point parabola around the peak to get sub-pixel position.
      Falls back to the integer peak if the parabola vertex is outside the
      window.
   d. Classify each column with a :class:`ColumnFlag`:
      - ``SATURATED``: peak value == 255 AND at least *N* neighbouring rows
        are also 255 (plateau beyond the threshold ``saturated_plateau_px``).
      - ``WEAK``: peak SNR (peak / background_rms) < ``weak_snr_threshold``.
      - ``MULTI_PEAK``: a second local maximum above
        ``multi_peak_fraction * peak_value`` exists outside the search window.
      - ``MISSING``: peak is below the absolute floor ``min_peak_value`` or
        the column is fully zero.
      - ``VALID``: none of the above apply.

   Invalid columns yield ``NaN`` position, never a fabricated value.

2. **Triangulation** (:func:`triangulate_profile`):
   Convert sub-pixel row positions to 3D points in the DexArm print frame
   via ray–plane intersection using a :class:`LaserCalibration` that holds
   pinhole intrinsics, distortion, the laser plane (in camera frame), and a
   rigid transform from camera to print frame.

Design values are imported from :mod:`inchworm.constants` (see doc-1 §5 for
their numeric values; they are **not** re-stated here to keep a single source
of truth).  These populate the *default* :class:`LaserCalibration`; they are
marked
**unverified** and must be replaced by the AA-30 calibration result before
production use.  Live mode raises :class:`UnverifiedCalibrationError` when
calibration is unverified.

References:
  - doc-1 §4 (cameras) and §5 (laser scanner geometry)
  - AA-30 (calibration task — not yet complete)
  - AA-14 (AppliedSettings for exposure/gain recording)
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np

from inchworm.constants import (
    LASER_TO_CAMERA_ANGLE_DEG,
    LASER_BASELINE_OFFSET_MM,
    LASER_STANDOFF_MM,
)

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Column validity flags
# ---------------------------------------------------------------------------


class ColumnFlag(Enum):
    """Validity classification for one image column's laser-line position.

    Only ``VALID`` columns carry a reliable sub-pixel position; all others
    yield ``NaN`` and must not be used for triangulation.

    Attributes
    ----------
    VALID:
        Peak found, parabolic sub-pixel fit succeeded, no degenerate condition.
    SATURATED:
        Peak value is 255 and a plateau of at least
        ``LaserLineConfig.saturated_plateau_px`` rows are also at 255.
        The true centroid is indeterminate in the flat top region.
    WEAK:
        Peak SNR (peak / background rms) is below
        ``LaserLineConfig.weak_snr_threshold``.  The peak may be noise.
    MULTI_PEAK:
        A second local maximum above
        ``LaserLineConfig.multi_peak_fraction * peak_value`` exists outside
        the primary search window.  The true line position is ambiguous.
    MISSING:
        No signal: peak is below ``LaserLineConfig.min_peak_value`` or the
        column is all zeros (gap in surface coverage or laser occluded).
    """

    VALID = auto()
    SATURATED = auto()
    WEAK = auto()
    MULTI_PEAK = auto()
    MISSING = auto()


# ---------------------------------------------------------------------------
# Extraction configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LaserLineConfig:
    """Thresholds for laser-line extraction.

    All numeric values are **unverified** defaults; calibrate against
    measured frames.  These live in the AA-31 block of hardware.yaml and
    are loaded into :class:`inchworm.config.LaserLineConfigSchema`.

    Parameters
    ----------
    window_half_width_px:
        Half-width of the row search window around the peak for the
        parabolic fit (px).  Full window is
        ``[peak - window_half_width_px, peak + window_half_width_px]``.
    saturated_plateau_px:
        Minimum number of rows at 255 required to trigger ``SATURATED``.
        Set N=2 (unverified) so a 1-px clip is not mis-flagged.
    weak_snr_threshold:
        Minimum peak-SNR = peak / background_rms below which a column is
        ``WEAK``.  background_rms is the rms of rows outside the window.
        Value 3.0 (unverified).
    multi_peak_fraction:
        Fraction of the primary peak above which a secondary maximum
        outside the window triggers ``MULTI_PEAK``.  Value 0.4 (unverified).
    min_peak_value:
        Absolute minimum pixel value to consider the laser line present.
        Below this → ``MISSING``.  Value 20 (unverified).
    background_rows:
        Number of rows from the top and bottom of the frame (outside the
        window) used to estimate the per-column background rms.  0 = use
        all rows outside the window.
    """

    # unverified: true — calibrate against measured frames
    window_half_width_px: int = 15
    saturated_plateau_px: int = 2
    weak_snr_threshold: float = 3.0
    multi_peak_fraction: float = 0.4
    min_peak_value: int = 20
    background_rows: int = 0  # 0 = use all rows outside window


# ---------------------------------------------------------------------------
# Extraction result
# ---------------------------------------------------------------------------


@dataclass
class LaserLineResult:
    """Result of laser-line extraction on one frame.

    Attributes
    ----------
    row_positions:
        Sub-pixel row position per column (shape ``(width,)``, float64).
        Invalid columns carry ``NaN``.
    flags:
        Per-column :class:`ColumnFlag` (shape ``(width,)``).
    exposure:
        Exposure setting recorded from :class:`~inchworm.camera_capture.AppliedSettings`,
        or ``None`` if not available.
    gain:
        Gain setting recorded from :class:`~inchworm.camera_capture.AppliedSettings`,
        or ``None`` if not available.
    frame_shape:
        ``(height, width)`` of the input frame.
    n_valid:
        Number of valid columns.
    n_saturated:
        Number of saturated columns.
    n_weak:
        Number of weak columns.
    n_multi_peak:
        Number of multi-peak columns.
    n_missing:
        Number of missing columns.
    """

    row_positions: np.ndarray          # shape (width,) float64, NaN for invalid
    flags: np.ndarray                  # shape (width,) object dtype → ColumnFlag
    exposure: Optional[int]
    gain: Optional[int]
    frame_shape: Tuple[int, int]       # (height, width)

    # Derived counts (filled by __post_init__)
    n_valid: int = field(init=False)
    n_saturated: int = field(init=False)
    n_weak: int = field(init=False)
    n_multi_peak: int = field(init=False)
    n_missing: int = field(init=False)

    def __post_init__(self) -> None:
        self.n_valid = int(np.sum(self.flags == ColumnFlag.VALID))
        self.n_saturated = int(np.sum(self.flags == ColumnFlag.SATURATED))
        self.n_weak = int(np.sum(self.flags == ColumnFlag.WEAK))
        self.n_multi_peak = int(np.sum(self.flags == ColumnFlag.MULTI_PEAK))
        self.n_missing = int(np.sum(self.flags == ColumnFlag.MISSING))


# ---------------------------------------------------------------------------
# Extraction implementation
# ---------------------------------------------------------------------------


def _parabolic_peak(col: np.ndarray, peak_row: int) -> float:
    """Return the sub-pixel peak row via 3-point parabolic interpolation.

    Given column intensities ``col`` and the integer row of the maximum
    ``peak_row``, fits a parabola through
    ``(peak_row-1, col[peak_row-1])``,
    ``(peak_row,   col[peak_row])``,
    ``(peak_row+1, col[peak_row+1])``
    and returns the vertex position.

    Falls back to the integer ``peak_row`` if the vertex is outside
    ``[peak_row-0.5, peak_row+0.5]`` or if boundary conditions prevent
    the 3-point fit.

    Parameters
    ----------
    col:
        1-D float array of pixel intensities in one image column.
    peak_row:
        Integer row index of the maximum.

    Returns
    -------
    float
        Sub-pixel row position of the laser line.
    """
    if peak_row <= 0 or peak_row >= len(col) - 1:
        return float(peak_row)

    y0 = float(col[peak_row - 1])
    y1 = float(col[peak_row])
    y2 = float(col[peak_row + 1])

    denom = y0 - 2.0 * y1 + y2
    if abs(denom) < 1e-12:
        return float(peak_row)

    delta = 0.5 * (y0 - y2) / denom
    if abs(delta) > 0.5:
        # Vertex outside the 3-point bracket; integer fallback
        return float(peak_row)
    return float(peak_row) + delta


def _find_second_peak(col: np.ndarray, peak_row: int, window: int) -> Optional[float]:
    """Return the value of the highest local max outside ``[peak_row±window]``.

    A "local maximum" is a value strictly greater than both its immediate
    neighbours.  The edges of the array are not treated as local maxima.

    Returns ``None`` if no local maximum exists outside the exclusion zone.
    """
    n = len(col)
    lo_excl = max(0, peak_row - window)
    hi_excl = min(n - 1, peak_row + window)

    best: Optional[float] = None
    for r in range(1, n - 1):
        if lo_excl <= r <= hi_excl:
            continue
        v = float(col[r])
        if v > float(col[r - 1]) and v > float(col[r + 1]):
            if best is None or v > best:
                best = v
    return best


def extract_laser_line(
    frame: np.ndarray,
    config: Optional[LaserLineConfig] = None,
    applied_settings: object = None,
) -> LaserLineResult:
    """Extract the sub-pixel laser-line position from a single mono frame.

    Parameters
    ----------
    frame:
        Single-channel uint8 grayscale numpy array, shape ``(height, width)``.
        Must be 2-D.
    config:
        Extraction thresholds.  ``None`` uses :class:`LaserLineConfig` defaults.
    applied_settings:
        Optional :class:`~inchworm.camera_capture.AppliedSettings` instance.
        ``exposure`` and ``gain`` are recorded in the result metadata if
        present.

    Returns
    -------
    LaserLineResult
        Per-column sub-pixel row positions (``NaN`` for invalid columns) and
        per-column :class:`ColumnFlag` values.

    Raises
    ------
    ValueError
        If ``frame`` is not a 2-D uint8 array.

    Notes
    -----
    The extraction method is documented per column:

    1. Cast to float64.
    2. Locate the row maximum (integer ``peak_row``).
    3. Flag ``MISSING`` if peak < ``config.min_peak_value``.
    4. Flag ``SATURATED`` if peak == 255 and the 255-plateau width ≥
       ``config.saturated_plateau_px``.
    5. Define the search window
       ``[peak_row - window_half_width_px, peak_row + window_half_width_px]``.
    6. Compute background rms from rows **outside** the window.
    7. Flag ``WEAK`` if peak / background_rms < ``config.weak_snr_threshold``.
    8. Check for a second peak outside the window; flag ``MULTI_PEAK`` if it
       exceeds ``config.multi_peak_fraction * peak_value``.
    9. Sub-pixel refine via 3-point parabolic fit.
    """
    if config is None:
        config = LaserLineConfig()

    if frame.ndim != 2 or frame.dtype != np.uint8:
        raise ValueError(
            f"frame must be a 2-D uint8 array; got shape={frame.shape} dtype={frame.dtype}"
        )

    height, width = frame.shape
    frame_f = frame.astype(np.float64)

    row_positions = np.full(width, np.nan, dtype=np.float64)
    flags = np.empty(width, dtype=object)

    # Extract exposure/gain from applied_settings (duck-typed)
    exposure: Optional[int] = None
    gain: Optional[int] = None
    if applied_settings is not None:
        exposure = getattr(applied_settings, "exposure", None)
        gain = getattr(applied_settings, "gain", None)

    w = config.window_half_width_px

    for col_idx in range(width):
        col = frame_f[:, col_idx]  # shape (height,)

        # --- 1. Find integer peak ---
        peak_row = int(np.argmax(col))
        peak_val = col[peak_row]

        # --- 2. MISSING: below absolute floor ---
        if peak_val < config.min_peak_value:
            flags[col_idx] = ColumnFlag.MISSING
            continue

        # --- 3. SATURATED: plateau of 255s ---
        if peak_val >= 255.0:
            plateau = int(np.sum(col >= 255.0))
            if plateau >= config.saturated_plateau_px:
                flags[col_idx] = ColumnFlag.SATURATED
                continue

        # --- 4. Search window ---
        win_lo = max(0, peak_row - w)
        win_hi = min(height - 1, peak_row + w)

        # --- 5. Background rms (rows outside window) ---
        bg_mask = np.ones(height, dtype=bool)
        bg_mask[win_lo : win_hi + 1] = False
        bg_vals = col[bg_mask]
        if bg_vals.size > 1:
            bg_rms = float(np.sqrt(np.mean(bg_vals**2)))
        else:
            bg_rms = 1.0  # fallback: treat as noise=1 so SNR = peak/1

        # --- 6. WEAK: SNR check ---
        snr = peak_val / max(bg_rms, 1e-9)
        if snr < config.weak_snr_threshold:
            flags[col_idx] = ColumnFlag.WEAK
            continue

        # --- 7. MULTI_PEAK: second peak outside window ---
        second_peak = _find_second_peak(col, peak_row, w)
        if second_peak is not None and second_peak >= config.multi_peak_fraction * peak_val:
            flags[col_idx] = ColumnFlag.MULTI_PEAK
            continue

        # --- 8. Sub-pixel refinement ---
        subpixel_row = _parabolic_peak(col, peak_row)

        row_positions[col_idx] = subpixel_row
        flags[col_idx] = ColumnFlag.VALID

    return LaserLineResult(
        row_positions=row_positions,
        flags=flags,
        exposure=exposure,
        gain=gain,
        frame_shape=(height, width),
    )


# ---------------------------------------------------------------------------
# Calibration dataclass
# ---------------------------------------------------------------------------


class UnverifiedCalibrationError(RuntimeError):
    """Raised when triangulation is attempted in live mode with unverified calibration.

    Calibration is unverified until AA-30 provides measured values.
    Pass ``dry_run=True`` to :func:`triangulate_profile` to bypass this check
    (returns the same geometry, but logs a warning).
    """


@dataclass
class LaserCalibration:
    """Laser-plane + pinhole model for one camera/laser pair.

    All values are **design defaults** sourced from doc-1 §5 via
    :mod:`inchworm.constants`.  They are marked ``unverified=True`` and
    must be replaced by the AA-30 calibration result.

    Parameters
    ----------
    fx, fy:
        Focal lengths in pixels.  Design placeholder: 800 px (unverified).
    cx, cy:
        Principal point in pixels.  Design placeholder: image centre
        (640, 400) for 1280×800.  (unverified)
    distortion:
        OpenCV-style ``[k1, k2, p1, p2]`` radial+tangential distortion.
        Placeholder: all zeros (unverified).
    plane_normal:
        Unit normal of the laser plane in the **camera frame** (3-vector,
        float64).  Derived from the design angle
        :data:`~inchworm.constants.LASER_TO_CAMERA_ANGLE_DEG`.
        Plane equation: ``dot(plane_normal, P) + plane_d = 0``.
        (unverified)
    plane_d:
        Scalar offset in the laser-plane equation (mm).  Derived from
        standoff :data:`~inchworm.constants.LASER_STANDOFF_MM`.
        (unverified)
    T_print_camera:
        4×4 homogeneous transform from **camera frame** to **DexArm print
        frame** (float64, row-major).  Design placeholder: identity +
        translation from :data:`~inchworm.constants.LASER_BASELINE_OFFSET_MM`
        along X.  (unverified)
    unverified:
        ``True`` means AA-30 calibration has not been applied.  Live mode
        raises :class:`UnverifiedCalibrationError`.
    baseline_mm:
        Camera/laser baseline offset (mm), stored for reference only.
    standoff_mm:
        Design standoff distance (mm), stored for reference only.
    angle_deg:
        Design laser-to-camera angle (degrees), stored for reference only.

    Notes
    -----
    Design values for the laser plane:

    The camera is pointed orthogonally at the surface.  The laser is
    displaced by ``baseline_mm`` along the camera X-axis and angled toward
    the surface at ``angle_deg`` from vertical.  In the camera frame the
    laser plane normal is approximately::

        n = [-sin(angle_deg), 0, cos(angle_deg)]

    The plane passes through the point at standoff_mm::

        d = -dot(n, [0, 0, standoff_mm])

    These are design estimates; the calibration step (AA-30) will measure
    the actual plane with sub-pixel accuracy.
    """

    # Pinhole intrinsics (pixels) — unverified placeholders
    fx: float = 800.0       # unverified: true
    fy: float = 800.0       # unverified: true
    cx: float = 640.0       # unverified: true
    cy: float = 400.0       # unverified: true

    # Distortion [k1, k2, p1, p2] — all-zero placeholder
    distortion: np.ndarray = field(
        default_factory=lambda: np.zeros(4, dtype=np.float64)
    )

    # Laser plane in camera frame: n·P + d = 0
    plane_normal: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    plane_d: float = 0.0

    # 4×4 transform: print_P = T @ camera_P (homogeneous)
    T_print_camera: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )

    # Metadata
    unverified: bool = True
    baseline_mm: float = field(default_factory=lambda: float(LASER_BASELINE_OFFSET_MM))
    standoff_mm: float = field(default_factory=lambda: float(LASER_STANDOFF_MM))
    angle_deg: float = field(default_factory=lambda: float(LASER_TO_CAMERA_ANGLE_DEG))

    def __post_init__(self) -> None:
        # Replace placeholder plane_normal/plane_d with design values from
        # constants if they were not explicitly overridden.
        if np.allclose(self.plane_normal, [0.0, 0.0, 1.0]) and self.plane_d == 0.0:
            # Design laser plane from doc-1 §5 (unverified)
            ang_rad = math.radians(self.angle_deg)
            # Normal points from surface toward camera, tilted by angle_deg
            # around the Y axis.
            n = np.array(
                [-math.sin(ang_rad), 0.0, math.cos(ang_rad)], dtype=np.float64
            )
            n /= np.linalg.norm(n)
            # Plane passes through the standoff point [0, 0, standoff_mm]
            d = -float(np.dot(n, [0.0, 0.0, self.standoff_mm]))
            self.plane_normal = n
            self.plane_d = d

        if np.allclose(self.T_print_camera, np.eye(4)):
            # Design placeholder: camera is at [-baseline_mm, 0, 0] in print frame
            T = np.eye(4, dtype=np.float64)
            T[0, 3] = self.baseline_mm
            self.T_print_camera = T  # baseline shift in print frame


def default_laser_calibration() -> LaserCalibration:
    """Return a :class:`LaserCalibration` with doc-1 design defaults.

    Values are **unverified** (see :attr:`LaserCalibration.unverified`).
    Import constants from :mod:`inchworm.constants`; never retype them here.
    """
    return LaserCalibration()


# ---------------------------------------------------------------------------
# Triangulation
# ---------------------------------------------------------------------------


def _undistort_point(
    u: float, v: float, calib: LaserCalibration
) -> Tuple[float, float]:
    """Apply Brown–Conrady undistortion to a single pixel (u, v).

    With the all-zero placeholder distortion this is a no-op.  The
    undistorted normalised camera coordinates ``(x, y)`` are returned.

    This is a one-iteration approximation sufficient for small distortion.
    """
    k1, k2, p1, p2 = calib.distortion[:4]
    # Normalised camera coordinates (ideal pinhole)
    xn = (u - calib.cx) / calib.fx
    yn = (v - calib.cy) / calib.fy

    if abs(k1) < 1e-12 and abs(k2) < 1e-12 and abs(p1) < 1e-12 and abs(p2) < 1e-12:
        return xn, yn

    # Iterative undistortion (1 iteration, sufficient for small distortion)
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    xn_u = (xn - p2 * (r2 + 2 * xn * xn) - 2 * p1 * xn * yn) / radial
    yn_u = (yn - p1 * (r2 + 2 * yn * yn) - 2 * p2 * xn * yn) / radial
    return xn_u, yn_u


def _ray_plane_intersect(
    ray_dir: np.ndarray,
    plane_normal: np.ndarray,
    plane_d: float,
) -> Optional[np.ndarray]:
    """Intersect a ray from the camera origin with a plane.

    Ray: ``P(t) = t * ray_dir``  (origin at camera centre = [0,0,0]).
    Plane: ``dot(normal, P) + d = 0``.

    Returns the 3-D intersection point in the camera frame (mm), or ``None``
    if the ray is parallel to the plane.

    Parameters
    ----------
    ray_dir:
        Unit direction of the ray (3-vector, camera frame).
    plane_normal:
        Unit normal of the laser plane (3-vector, camera frame).
    plane_d:
        Scalar offset of the plane equation.

    Returns
    -------
    np.ndarray or None
        3-D point (mm) in camera frame, or ``None`` for parallel ray.
    """
    denom = float(np.dot(plane_normal, ray_dir))
    if abs(denom) < 1e-12:
        return None
    t = -plane_d / denom
    if t < 0:
        # Intersection behind the camera; degenerate
        return None
    return t * ray_dir


def triangulate_profile(
    result: LaserLineResult,
    calibration: Optional[LaserCalibration] = None,
    *,
    dry_run: bool = False,
) -> np.ndarray:
    """Convert laser-line row positions to 3-D points in the print frame.

    For each valid column ``col_idx``, the sub-pixel row ``r`` is mapped to a
    normalised camera ray ``[xn, yn, 1]`` (after undistorting), then
    intersected with the laser plane to get a 3-D point in the camera frame,
    then transformed to the DexArm print frame.

    Invalid columns (``NaN`` positions or non-``VALID`` flags) produce
    ``[NaN, NaN, NaN]`` rows in the output.

    Parameters
    ----------
    result:
        :class:`LaserLineResult` from :func:`extract_laser_line`.
    calibration:
        :class:`LaserCalibration` to use.  ``None`` uses
        :func:`default_laser_calibration` (design defaults, unverified).
    dry_run:
        If ``True``, bypass the :class:`UnverifiedCalibrationError` check
        and log a warning instead.  Intended for testing and dry-run scripts.

    Returns
    -------
    np.ndarray
        Shape ``(width, 3)`` float64 array of 3-D points (X, Y, Z) in mm
        in the DexArm print frame.  Invalid columns contain ``NaN``.

    Raises
    ------
    UnverifiedCalibrationError
        If ``calibration.unverified`` is ``True`` and ``dry_run`` is ``False``.
    """
    if calibration is None:
        calibration = default_laser_calibration()

    if calibration.unverified:
        if dry_run:
            log.warning(
                "AA-31 triangulate_profile: calibration is unverified (design defaults). "
                "Results are approximate. Run AA-30 to obtain real calibration."
            )
        else:
            raise UnverifiedCalibrationError(
                "LaserCalibration.unverified is True. "
                "AA-30 calibration has not been applied. "
                "Pass dry_run=True to override for testing."
            )

    width = result.frame_shape[1]
    points = np.full((width, 3), np.nan, dtype=np.float64)

    n = calibration.plane_normal
    d = calibration.plane_d
    T = calibration.T_print_camera

    for col_idx in range(width):
        if result.flags[col_idx] is not ColumnFlag.VALID:
            continue
        row = result.row_positions[col_idx]
        if not np.isfinite(row):
            continue

        # Undistort: pixel (col_idx, row) -> normalised camera coords
        xn, yn = _undistort_point(float(col_idx), float(row), calibration)

        # Ray direction in camera frame (unit vector)
        ray = np.array([xn, yn, 1.0], dtype=np.float64)
        ray /= np.linalg.norm(ray)

        # Ray–plane intersection in camera frame (mm)
        p_cam = _ray_plane_intersect(ray, n, d)
        if p_cam is None:
            continue

        # Transform to print frame
        p_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=np.float64)
        p_print = T @ p_h
        points[col_idx] = p_print[:3]

    return points


# ---------------------------------------------------------------------------
# Telemetry helper
# ---------------------------------------------------------------------------


def log_extraction_result(
    result: LaserLineResult,
    logger: logging.Logger,
    *,
    subsystem: str = "laser_line",
    role: str = "scanner",
) -> None:
    """Emit an AA-7 compatible structured log event for an extraction result.

    Logs at INFO level with a payload that records frame shape, column
    statistics, exposure, and gain.

    Parameters
    ----------
    result:
        The :class:`LaserLineResult` to summarise.
    logger:
        A stdlib :class:`logging.Logger` to emit to.
    subsystem:
        AA-7 subsystem tag (default ``"laser_line"``).
    role:
        AA-7 role tag (default ``"scanner"``).
    """
    logger.info(
        "[AA-31] laser_line_extracted  frame=%dx%d  valid=%d  sat=%d  weak=%d  "
        "multi_peak=%d  missing=%d  exposure=%s  gain=%s",
        result.frame_shape[1],
        result.frame_shape[0],
        result.n_valid,
        result.n_saturated,
        result.n_weak,
        result.n_multi_peak,
        result.n_missing,
        result.exposure,
        result.gain,
    )
