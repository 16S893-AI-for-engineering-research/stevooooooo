"""Scan pattern and surface fit for leveling correction — AA-32.

This module sweeps the DexArm-mounted laser scanner across the end face of
the printed tube, stitches the resulting height profiles into a single point
cloud in the DexArm print frame, and fits a plane to the annular end face to
produce the Z-offset and two tilt angles (rx, ry) needed to correct the print
frame before the next hop.

Architecture overview
---------------------
* :class:`ScanConfig`       — all tunable parameters with unverified defaults.
* :func:`scan_pattern`      — generates the list of (X, Y) stops at the
                              calibrated standoff Z; documented raster path.
* :func:`run_scan`          — executes the pattern: move → settle → laser on
                              → capture → laser off, per stop.  Laser is on
                              ONLY during capture (AA-29 context manager).
* :func:`stitch_profiles`   — transforms per-stop point clouds into the print
                              frame using DexArm-reported position +
                              ``T_tool_scanner`` (AA-30 seam).
* :func:`fit_surface`       — RANSAC plane fit of the annular end-face cloud
                              → :class:`LevelingResult`.
* :class:`LevelingResult`   — Z offset, rx, ry, residual RMS, uncertainty.
* :class:`LevelingRefused`  — typed failure when quality thresholds fail.

Design rationale — plane fit for the tube end face
---------------------------------------------------
The printed tube end face is nominally flat (a planar annulus, i.e. the
cut/printed end of a cylindrical tube).  The tilt and height of this plane
relative to the DexArm print frame determines the leveling correction.  A
cylinder fit would give the axis but not the end-face orientation; a simple
flat-plane fit of the annular end-face region is both the correct geometric
model and the most robust for 3-DOF correction (Z, rx, ry).

The plane is parameterised as ``Z = a*X + b*Y + c`` where X, Y are print-frame
coordinates. rx = -arctan(b), ry = arctan(a), Z_offset = c (at X=Y=0).

Robustness
-----------
RANSAC (minimum 3-point consensus, configurable inlier threshold and trials)
is used to reject outlier profiles from occluded or noisy stops.  The Huber
loss alternative was considered but RANSAC was chosen because:
  1. Outliers from occluded stops are structurally separated from inliers
     (large residual), well-suited to RANSAC hard inlier threshold.
  2. Huber requires gradient optimisation; RANSAC is simpler and more
     interpretable for this geometry.
  3. The plane equation is linear and can be solved exactly for 3 points,
     making RANSAC exact and fast.

Thresholds
-----------
All thresholds are in :class:`ScanConfig` with ``unverified: true`` comments.
Live mode FAILS CLOSED on ``None`` thresholds (raises :class:`LevelingRefused`
with ``reason="config_unverified"``).

AA-30 T_tool_scanner seam
--------------------------
The transform from DexArm tool frame to scanner camera frame is injected via
the ``T_tool_scanner`` parameter (4×4 numpy float64 HTM).  The default is the
4×4 identity (unverified; AA-30 will measure this).  In live mode a WARNING is
logged when the default is used.  ``T_tool_scanner_unverified`` flag propagates
to :class:`LevelingResult`.

Laser safety
-------------
:meth:`run_scan` uses the :class:`~inchworm.laser.LaserDriver` as a context
manager (``__enter__`` / ``__exit__``) on *each individual capture* so the
laser is off between stops and on every exit path including exceptions.

Telemetry
----------
All significant events are emitted via AA-7 :class:`~inchworm.telemetry.SessionLogger`
when one is supplied.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import numpy.typing as npt

log = logging.getLogger(__name__)

# Type aliases
Mat44 = npt.NDArray[np.float64]
PointCloud = npt.NDArray[np.float64]   # shape (N, 3)

# ---------------------------------------------------------------------------
# Typed failure
# ---------------------------------------------------------------------------


class LevelingRefused(RuntimeError):
    """Raised when the leveling scan cannot produce a trustworthy correction.

    Attributes
    ----------
    reason:
        Machine-readable reason code:
        ``"insufficient_coverage"`` — fewer valid points than
        ``ScanConfig.min_valid_points``.
        ``"high_residual"`` — plane-fit residual RMS exceeds
        ``ScanConfig.max_residual_rms_mm``.
        ``"high_uncertainty"`` — estimated uncertainty exceeds
        ``ScanConfig.max_uncertainty_mm``.
        ``"config_unverified"`` — a required config threshold is ``None``
        (live mode fails closed).
        ``"no_inliers"`` — RANSAC found no consensus set (degenerate data).
    detail:
        Human-readable explanation.
    """

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        self.detail = detail
        super().__init__(f"LevelingRefused({reason}): {detail}")


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass
class LevelingResult:
    """Output of a successful leveling scan.

    Attributes
    ----------
    z_offset_mm:
        Z offset of the tube end face relative to the DexArm print-frame
        origin, in mm.  Positive = end face is farther from the DexArm.
    rx_rad:
        Tilt of the end face about the print-frame X axis (radians).
        ``rx = -arctan(b)`` where ``Z = a*X + b*Y + c``.
    ry_rad:
        Tilt of the end face about the print-frame Y axis (radians).
        ``ry = arctan(a)`` where ``Z = a*X + b*Y + c``.
    residual_rms_mm:
        RMS of inlier residuals after the plane fit (mm).
    uncertainty_mm:
        Estimated 1-sigma uncertainty in z_offset_mm derived from the
        residual covariance (mm).  Computed as the square-root of the
        (c, c) diagonal element of the least-squares covariance matrix
        on the inlier set.
    n_inliers:
        Number of inlier points used for the final plane fit.
    n_total:
        Total number of valid points in the stitched cloud before RANSAC.
    T_tool_scanner_unverified:
        True when the ``T_tool_scanner`` transform used for stitching is
        the unverified default identity (AA-30 not yet calibrated).
    plane_normal:
        Unit normal of the fitted plane in the print frame (3-vector,
        float64).  For a perfectly flat face aligned with XY the normal
        is approximately ``[0, 0, 1]``.
    """

    z_offset_mm: float
    rx_rad: float
    ry_rad: float
    residual_rms_mm: float
    uncertainty_mm: float
    n_inliers: int
    n_total: int
    T_tool_scanner_unverified: bool
    plane_normal: npt.NDArray[np.float64]


# ---------------------------------------------------------------------------
# Scan configuration
# ---------------------------------------------------------------------------


@dataclass
class ScanConfig:
    """All tunable parameters for the leveling scan.

    All values are **unverified** defaults; calibrate against hardware.
    Loaded from ``config.leveling`` in hardware.yaml (AA-32 block).

    Parameters
    ----------
    standoff_z_mm:
        DexArm Z coordinate of the scanner standoff above the print surface
        (mm, arm frame).  **unverified: true** — must be measured in a
        hardware session; live mode fails closed when ``None``.
    scan_x_min_mm, scan_x_max_mm:
        X-axis extent of the raster scan grid (mm, DexArm arm frame).
        **unverified: true**.
    scan_y_min_mm, scan_y_max_mm:
        Y-axis extent of the raster scan grid (mm, DexArm arm frame).
        **unverified: true**.
    scan_x_steps:
        Number of stops along X (≥ 2).  **unverified: true**.
    scan_y_steps:
        Number of stops along Y (≥ 2).  **unverified: true**.
    feedrate_mm_min:
        DexArm move feedrate (mm/min).  ``None`` → firmware default.
        **unverified: true**.
    settle_wait_s:
        Seconds to wait after confirming the arm position before capturing.
        Allows mechanical vibrations to damp.  **unverified: true**.
    ransac_inlier_threshold_mm:
        Maximum residual (mm) for a point to count as a RANSAC inlier.
        **unverified: true**.
    ransac_trials:
        Number of RANSAC iterations.  **unverified: true**.
    min_valid_points:
        Minimum number of valid (non-NaN) stitched points required.
        Refusal if fewer.  **unverified: true**.
    min_inlier_fraction:
        Minimum inlier fraction (inliers / total) required.
        Refusal if below.  **unverified: true**.
    max_residual_rms_mm:
        Maximum allowed residual RMS (mm) after plane fit.
        Refusal if exceeded.  **unverified: true**.
    max_uncertainty_mm:
        Maximum allowed estimated z_offset uncertainty (mm).
        Refusal if exceeded.  **unverified: true**.
    """

    # --- Geometry (unverified: true) ---
    standoff_z_mm: Optional[float] = None    # unverified: true; live fails closed on None

    scan_x_min_mm: float = -30.0             # unverified: true
    scan_x_max_mm: float = 30.0              # unverified: true
    scan_y_min_mm: float = 100.0             # unverified: true (nominal scan range in front)
    scan_y_max_mm: float = 160.0             # unverified: true
    scan_x_steps: int = 5                    # unverified: true
    scan_y_steps: int = 7                    # unverified: true

    feedrate_mm_min: Optional[float] = 1500.0   # unverified: true

    # Settle wait after M114 confirms position
    settle_wait_s: float = 0.05             # unverified: true

    # RANSAC plane fit
    ransac_inlier_threshold_mm: float = 0.5  # unverified: true
    ransac_trials: int = 200                 # unverified: true

    # Refusal thresholds
    min_valid_points: int = 10               # unverified: true
    min_inlier_fraction: float = 0.5         # unverified: true
    max_residual_rms_mm: float = 0.5         # unverified: true
    max_uncertainty_mm: float = 0.2          # unverified: true


# ---------------------------------------------------------------------------
# Scan pattern
# ---------------------------------------------------------------------------


def scan_pattern(config: ScanConfig) -> List[Tuple[float, float]]:
    """Generate the raster scan path as a list of (X, Y) DexArm coordinates.

    The pattern is a boustrophedon (snake) raster over the rectangle
    [scan_x_min_mm, scan_x_max_mm] × [scan_y_min_mm, scan_y_max_mm],
    with ``scan_x_steps`` columns and ``scan_y_steps`` rows.

    Traversal order (documented):

    * Outer loop over Y rows, from ``scan_y_min_mm`` to ``scan_y_max_mm``.
    * Inner loop over X columns: left-to-right on even rows,
      right-to-left on odd rows (boustrophedon / snake raster).
    * This minimises DexArm travel distance.

    Parameters
    ----------
    config:
        :class:`ScanConfig` with grid dimensions and step counts.

    Returns
    -------
    list of (x_mm, y_mm) tuples:
        Ordered DexArm XY positions for the scan, length
        ``scan_x_steps × scan_y_steps``.

    Raises
    ------
    ValueError
        If ``scan_x_steps < 2`` or ``scan_y_steps < 2``.
    """
    if config.scan_x_steps < 2 or config.scan_y_steps < 2:
        raise ValueError(
            f"scan_x_steps and scan_y_steps must be >= 2; "
            f"got ({config.scan_x_steps}, {config.scan_y_steps})"
        )

    xs = np.linspace(config.scan_x_min_mm, config.scan_x_max_mm, config.scan_x_steps)
    ys = np.linspace(config.scan_y_min_mm, config.scan_y_max_mm, config.scan_y_steps)

    stops: List[Tuple[float, float]] = []
    for row_idx, y in enumerate(ys):
        # Reverse direction on odd rows (boustrophedon).
        x_row = xs if row_idx % 2 == 0 else xs[::-1]
        for x in x_row:
            stops.append((float(x), float(y)))
    return stops


# ---------------------------------------------------------------------------
# run_scan
# ---------------------------------------------------------------------------


def run_scan(
    arm,                                 # DexArmDriver
    laser,                               # LaserDriver
    capture_fn: Callable[[], object],    # AA-14/AA-31: () -> frame (or profile)
    extract_fn: Callable[[object], Optional[PointCloud]],  # AA-31: frame -> (N,3) or None
    *,
    config: Optional[ScanConfig] = None,
    dry_run: bool = True,
    logger: object = None,
) -> List[Tuple[Tuple[float, float, float], PointCloud]]:
    """Execute the leveling scan pattern and return one profile per stop.

    Moves the DexArm through each stop in :func:`scan_pattern`, confirms
    the arm position via M114 (settle check), then captures one height
    profile per stop.  **The laser is on ONLY during the capture window**,
    enforced via the :class:`~inchworm.laser.LaserDriver` context manager
    (``__enter__`` / ``__exit__``).  If any exception escapes the context
    manager the laser is guaranteed off.

    Parameters
    ----------
    arm:
        :class:`~inchworm.dexarm.DexArmDriver` connected to the DexArm.
    laser:
        :class:`~inchworm.laser.LaserDriver` for the line laser (AA-29).
    capture_fn:
        Zero-argument callable that returns a raw camera frame (or any
        object passed to ``extract_fn``).  Implements the AA-14 capture
        interface; injected for testability.
    extract_fn:
        Callable ``(frame_or_profile) -> Optional[np.ndarray shape (N,3)]``
        that triangulates a height profile into 3D print-frame points via
        AA-31.  Returns ``None`` or an empty array when the profile is invalid.
    config:
        :class:`ScanConfig`.  ``None`` → :class:`ScanConfig` defaults.
    dry_run:
        When ``True``, log planned moves without sending them; return
        synthetic zero profiles instead of real captures.  The laser context
        manager is still entered/exited in dry-run mode (tests laser
        on/off enforcement).
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.

    Returns
    -------
    list of (reported_pos, points):
        ``reported_pos`` is the (x, y, z) position from M114 after settling.
        ``points`` is the shape-(N, 3) array from ``extract_fn``; may be
        empty (shape (0, 3)) when no profile was extracted.

    Raises
    ------
    LevelingRefused(reason="config_unverified")
        If ``config.standoff_z_mm`` is ``None`` and ``dry_run`` is ``False``.
    Any exception from arm.move_absolute or laser.on propagates after
        ensuring the laser is off via the context manager.
    """
    if config is None:
        config = ScanConfig()

    if not dry_run and config.standoff_z_mm is None:
        raise LevelingRefused(
            "config_unverified",
            "standoff_z_mm is None; set it in hardware.yaml before live use.",
        )

    standoff_z = config.standoff_z_mm if config.standoff_z_mm is not None else 150.0

    stops = scan_pattern(config)
    results: List[Tuple[Tuple[float, float, float], PointCloud]] = []

    _emit(logger, "INFO", "leveling_scan_start", {
        "n_stops": len(stops),
        "standoff_z_mm": standoff_z,
        "dry_run": dry_run,
    })

    t_start = time.monotonic()

    for stop_idx, (x, y) in enumerate(stops):
        # Move to stop position at calibrated standoff Z.
        arm.move_absolute(x, y, standoff_z, feedrate=config.feedrate_mm_min, dry_run=dry_run)

        if dry_run:
            # In dry-run: synthesise a reported position equal to commanded.
            reported_pos = (x, y, standoff_z)
        else:
            # Wait for settle then confirm position via M114.
            if config.settle_wait_s > 0:
                time.sleep(config.settle_wait_s)
            pos = arm.get_position()
            reported_pos = (pos.x, pos.y, pos.z)

        _emit(logger, "DEBUG", "leveling_stop_settled", {
            "stop_idx": stop_idx,
            "commanded": (x, y, standoff_z),
            "reported": reported_pos,
        })

        # Laser ON only during capture — context manager enforces off on
        # every exit path (normal return, exception, KeyboardInterrupt).
        # Orchestrator fix 2026-10-24: LaserDriver.__enter__ does NOT switch
        # the laser on (only __exit__ switches it off), so on() must be called
        # explicitly inside the context. The earlier mock-only test hid that
        # real scans would have captured with the laser dark.
        with laser:
            if not dry_run:
                laser.on(duration_s=getattr(config, "max_capture_s", None))
            if dry_run:
                profile_pts: Optional[PointCloud] = np.zeros((0, 3), dtype=np.float64)
            else:
                frame = capture_fn()
                profile_pts = extract_fn(frame)

        # After the `with laser:` block the laser is off (laser.__exit__ calls off()).
        if profile_pts is None or len(profile_pts) == 0:
            pts = np.zeros((0, 3), dtype=np.float64)
        else:
            pts = np.asarray(profile_pts, dtype=np.float64)
            if pts.ndim != 2 or pts.shape[1] != 3:
                pts = np.zeros((0, 3), dtype=np.float64)

        results.append((reported_pos, pts))

        _emit(logger, "DEBUG", "leveling_stop_captured", {
            "stop_idx": stop_idx,
            "n_points": len(pts),
        })

    elapsed_s = time.monotonic() - t_start
    _emit(logger, "INFO", "leveling_scan_complete", {
        "n_stops": len(stops),
        "elapsed_s": round(elapsed_s, 3),
        "dry_run": dry_run,
    })
    return results


# ---------------------------------------------------------------------------
# T_tool_scanner default (AA-30 seam)
# ---------------------------------------------------------------------------

# Identity transform — unverified placeholder until AA-30 calibration.
# T_tool_scanner: 4×4 HTM from DexArm tool frame to scanner camera frame.
_T_TOOL_SCANNER_DEFAULT: Mat44 = np.eye(4, dtype=np.float64)
_T_TOOL_SCANNER_DEFAULT_FLAG = True   # mark as unverified


def default_T_tool_scanner() -> Mat44:
    """Return the default (unverified) T_tool_scanner identity transform.

    This is the AA-30 seam: once AA-30 calibration is run, replace this
    with the measured transform.  The flag :data:`_T_TOOL_SCANNER_DEFAULT_FLAG`
    is used to propagate ``T_tool_scanner_unverified`` in results.
    """
    return _T_TOOL_SCANNER_DEFAULT.copy()


# ---------------------------------------------------------------------------
# stitch_profiles
# ---------------------------------------------------------------------------


def stitch_profiles(
    scan_results: List[Tuple[Tuple[float, float, float], PointCloud]],
    T_tool_scanner: Optional[Mat44] = None,
    *,
    logger: object = None,
) -> Tuple[PointCloud, bool]:
    """Transform per-stop profiles into a single point cloud in the print frame.

    For each stop, the DexArm-reported position (x, y, z) defines the tool-
    frame origin in the DexArm arm frame (which is the print frame at this
    level of the pipeline).  Each point from ``extract_fn`` is expressed in
    the scanner camera frame; it is first transformed by ``T_tool_scanner``
    into the tool frame, then translated by the reported arm position.

    The transform chain:
    ``P_print = T_arm_origin + R_arm * T_tool_scanner @ P_scanner``

    For the unverified default (``T_tool_scanner = I``), the scanner frame
    equals the tool frame and ``P_print = P_scanner + arm_position``.

    Parameters
    ----------
    scan_results:
        Output of :func:`run_scan`.
    T_tool_scanner:
        4×4 HTM from tool frame to scanner frame.  ``None`` → identity
        (unverified default, AA-30 seam).  A WARNING is logged when the
        default is used.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.

    Returns
    -------
    (cloud, T_tool_scanner_unverified):
        ``cloud`` is shape-(N, 3) float64 in the DexArm print frame (all
        valid non-NaN points from all stops concatenated).
        ``T_tool_scanner_unverified`` is ``True`` when the identity default
        was used.
    """
    if T_tool_scanner is None:
        T_tool_scanner = default_T_tool_scanner()
        t_unverified = True
        log.warning(
            "AA-32 stitch_profiles: T_tool_scanner is the unverified identity default. "
            "Run AA-30 calibration to obtain the real transform."
        )
    else:
        t_unverified = False

    R = T_tool_scanner[:3, :3]
    t = T_tool_scanner[:3, 3]

    all_points: List[PointCloud] = []

    for (ax, ay, az), pts in scan_results:
        if len(pts) == 0:
            continue
        # Remove NaN rows
        valid_mask = np.all(np.isfinite(pts), axis=1)
        pts_valid = pts[valid_mask]
        if len(pts_valid) == 0:
            continue
        # Transform: scanner frame → tool frame → arm/print frame
        # P_tool = R @ P_scanner + t
        # P_print = P_tool + arm_position
        pts_tool = (R @ pts_valid.T).T + t
        arm_pos = np.array([ax, ay, az], dtype=np.float64)
        pts_print = pts_tool + arm_pos
        all_points.append(pts_print)

    if all_points:
        cloud = np.vstack(all_points)
    else:
        cloud = np.zeros((0, 3), dtype=np.float64)

    _emit(logger, "INFO", "leveling_stitch_complete", {
        "n_stops_with_data": len(all_points),
        "n_total_points": len(cloud),
        "T_tool_scanner_unverified": t_unverified,
    })
    return cloud, t_unverified


# ---------------------------------------------------------------------------
# fit_surface
# ---------------------------------------------------------------------------


def fit_surface(
    cloud: PointCloud,
    config: Optional[ScanConfig] = None,
    *,
    T_tool_scanner_unverified: bool = True,
    dry_run: bool = False,
    logger: object = None,
) -> LevelingResult:
    """Fit a robust plane to the stitched point cloud and produce a correction.

    The plane model is ``Z = a*X + b*Y + c`` where X, Y, Z are print-frame
    coordinates.  The fit gives:

    * ``z_offset_mm = c``  (Z at the frame origin)
    * ``ry_rad = arctan(a)``  (tilt about Y axis from X-slope)
    * ``rx_rad = -arctan(b)``  (tilt about X axis from Y-slope)

    RANSAC is used to identify inliers (robust to outlier profiles from
    occluded stops or bad triangulations).  The final fit uses only inliers,
    and the residual covariance is used to estimate z-offset uncertainty.

    Parameters
    ----------
    cloud:
        Shape-(N, 3) point cloud in the DexArm print frame (mm).
    config:
        :class:`ScanConfig`.  ``None`` → defaults.
    T_tool_scanner_unverified:
        Propagated from :func:`stitch_profiles`.
    dry_run:
        When ``True``, bypass the ``config_unverified`` check on standoff_z_mm.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.

    Returns
    -------
    LevelingResult

    Raises
    ------
    LevelingRefused(reason="insufficient_coverage")
        If the point cloud has fewer than ``min_valid_points`` finite points.
    LevelingRefused(reason="no_inliers")
        If RANSAC finds no inlier consensus.
    LevelingRefused(reason="high_residual")
        If inlier residual RMS > ``max_residual_rms_mm``.
    LevelingRefused(reason="high_uncertainty")
        If estimated z_offset uncertainty > ``max_uncertainty_mm``.
    """
    if config is None:
        config = ScanConfig()

    # --- Coverage check ---
    if len(cloud) == 0:
        raise LevelingRefused("insufficient_coverage", "point cloud is empty")

    finite_mask = np.all(np.isfinite(cloud), axis=1)
    pts = cloud[finite_mask]
    n_total = len(pts)

    if n_total < config.min_valid_points:
        raise LevelingRefused(
            "insufficient_coverage",
            f"only {n_total} valid points < min_valid_points={config.min_valid_points}",
        )

    X = pts[:, 0]
    Y = pts[:, 1]
    Z = pts[:, 2]

    # --- RANSAC plane fit ---
    best_inliers: Optional[npt.NDArray[np.bool_]] = None
    best_n = 0
    rng = np.random.default_rng(seed=42)

    for _ in range(config.ransac_trials):
        # Sample 3 points (minimum for plane)
        idx = rng.choice(n_total, size=3, replace=False)
        a, b, c = _fit_plane_3pts(X[idx], Y[idx], Z[idx])
        if a is None:
            continue  # degenerate sample
        residuals = np.abs(Z - (a * X + b * Y + c))
        inliers = residuals < config.ransac_inlier_threshold_mm
        n_in = int(np.sum(inliers))
        if n_in > best_n:
            best_n = n_in
            best_inliers = inliers

    if best_inliers is None or best_n == 0:
        raise LevelingRefused("no_inliers", "RANSAC found no consensus set (degenerate cloud)")

    # Check inlier fraction
    inlier_fraction = best_n / n_total
    if inlier_fraction < config.min_inlier_fraction:
        raise LevelingRefused(
            "insufficient_coverage",
            f"inlier fraction {inlier_fraction:.2f} < min={config.min_inlier_fraction:.2f}",
        )

    # Refit on all inliers (least-squares OLS for Z = a*X + b*Y + c)
    Xi = X[best_inliers]
    Yi = Y[best_inliers]
    Zi = Z[best_inliers]

    a_fit, b_fit, c_fit, cov = _fit_plane_ols(Xi, Yi, Zi)

    # Residual RMS
    residuals_in = Zi - (a_fit * Xi + b_fit * Yi + c_fit)
    rms = float(np.sqrt(np.mean(residuals_in ** 2)))

    if rms > config.max_residual_rms_mm:
        raise LevelingRefused(
            "high_residual",
            f"residual RMS {rms:.4f} mm > max={config.max_residual_rms_mm:.4f} mm",
        )

    # Uncertainty from covariance diagonal (c is index 2)
    if cov is not None:
        uncertainty = float(np.sqrt(max(cov[2, 2], 0.0)))
    else:
        uncertainty = float("inf")

    if uncertainty > config.max_uncertainty_mm:
        raise LevelingRefused(
            "high_uncertainty",
            f"z_offset uncertainty {uncertainty:.4f} mm > max={config.max_uncertainty_mm:.4f} mm",
        )

    # Plane normal (in print frame)
    # Plane: a*X + b*Y - Z + c = 0  →  normal = [-a, -b, 1] (unnormalized, pointing toward +Z)
    normal_raw = np.array([-a_fit, -b_fit, 1.0], dtype=np.float64)
    normal = normal_raw / np.linalg.norm(normal_raw)

    rx_rad = float(-math.atan(b_fit))
    ry_rad = float(math.atan(a_fit))

    result = LevelingResult(
        z_offset_mm=float(c_fit),
        rx_rad=rx_rad,
        ry_rad=ry_rad,
        residual_rms_mm=rms,
        uncertainty_mm=uncertainty,
        n_inliers=best_n,
        n_total=n_total,
        T_tool_scanner_unverified=T_tool_scanner_unverified,
        plane_normal=normal,
    )

    _emit(logger, "INFO", "leveling_fit_complete", {
        "z_offset_mm": round(result.z_offset_mm, 4),
        "rx_rad": round(result.rx_rad, 6),
        "ry_rad": round(result.ry_rad, 6),
        "residual_rms_mm": round(result.residual_rms_mm, 4),
        "uncertainty_mm": round(result.uncertainty_mm, 4),
        "n_inliers": result.n_inliers,
        "n_total": result.n_total,
        "T_tool_scanner_unverified": result.T_tool_scanner_unverified,
    })
    return result


# ---------------------------------------------------------------------------
# Convenience: full pipeline
# ---------------------------------------------------------------------------


def level_scan(
    arm,
    laser,
    capture_fn: Callable[[], object],
    extract_fn: Callable[[object], Optional[PointCloud]],
    *,
    config: Optional[ScanConfig] = None,
    T_tool_scanner: Optional[Mat44] = None,
    dry_run: bool = True,
    logger: object = None,
) -> LevelingResult:
    """Run the full scan→stitch→fit pipeline and return a :class:`LevelingResult`.

    Convenience wrapper that calls :func:`run_scan`, :func:`stitch_profiles`,
    and :func:`fit_surface` in sequence.

    Parameters
    ----------
    arm:
        :class:`~inchworm.dexarm.DexArmDriver`.
    laser:
        :class:`~inchworm.laser.LaserDriver`.
    capture_fn:
        Frame capture callable (AA-14 interface).
    extract_fn:
        Profile extraction callable (AA-31 interface).
    config:
        :class:`ScanConfig`.  ``None`` → defaults.
    T_tool_scanner:
        4×4 HTM tool→scanner transform.  ``None`` → unverified identity.
    dry_run:
        When ``True``, no hardware motion or capture; returns result from
        synthetic data.
    logger:
        AA-7 :class:`~inchworm.telemetry.SessionLogger`.

    Returns
    -------
    LevelingResult

    Raises
    ------
    LevelingRefused
        When the data does not support a trustworthy correction.
    """
    scan_data = run_scan(
        arm, laser, capture_fn, extract_fn,
        config=config, dry_run=dry_run, logger=logger,
    )
    cloud, t_unverified = stitch_profiles(scan_data, T_tool_scanner, logger=logger)
    return fit_surface(
        cloud, config,
        T_tool_scanner_unverified=t_unverified,
        dry_run=dry_run,
        logger=logger,
    )


# ---------------------------------------------------------------------------
# Plane fitting helpers
# ---------------------------------------------------------------------------


def _fit_plane_3pts(
    x: npt.NDArray[np.float64],
    y: npt.NDArray[np.float64],
    z: npt.NDArray[np.float64],
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """Fit ``Z = a*X + b*Y + c`` to exactly 3 points; return (a, b, c) or (None, None, None)."""
    # Ax = b: [X Y 1] @ [a, b, c]^T = Z
    A = np.column_stack([x, y, np.ones(3)])
    b = z
    try:
        sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
        if rank < 3:
            return None, None, None
        return float(sol[0]), float(sol[1]), float(sol[2])
    except np.linalg.LinAlgError:
        return None, None, None


def _fit_plane_ols(
    x: npt.NDArray[np.float64],
    y: npt.NDArray[np.float64],
    z: npt.NDArray[np.float64],
) -> Tuple[float, float, float, Optional[npt.NDArray[np.float64]]]:
    """Ordinary least-squares fit of ``Z = a*X + b*Y + c``.

    Returns (a, b, c, cov) where ``cov`` is the 3×3 parameter covariance
    matrix (or ``None`` when rank is deficient).
    """
    n = len(x)
    A = np.column_stack([x, y, np.ones(n)])
    sol, residuals, rank, _ = np.linalg.lstsq(A, z, rcond=None)
    a, b, c = float(sol[0]), float(sol[1]), float(sol[2])

    if rank < 3 or n <= 3:
        return a, b, c, None

    # Covariance: sigma^2 * (A^T A)^{-1}
    ATA = A.T @ A
    try:
        ATA_inv = np.linalg.inv(ATA)
    except np.linalg.LinAlgError:
        return a, b, c, None

    sigma2 = float(np.sum((z - A @ sol) ** 2)) / (n - 3)
    cov = sigma2 * ATA_inv
    return a, b, c, cov


# ---------------------------------------------------------------------------
# Telemetry helper
# ---------------------------------------------------------------------------


def _emit(logger: object, severity: str, event_type: str, payload: dict) -> None:
    if logger is None:
        return
    try:
        logger.emit(
            role="dexarm",
            subsystem="leveling",
            event_type=event_type,
            severity=severity,
            payload=payload,
        )
    except Exception:
        pass
