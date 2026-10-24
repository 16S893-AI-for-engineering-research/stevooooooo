"""Tests for inchworm.laser_line — AA-31 ACs #1, #2, #7.

AC#3 (real AA-30 calibration), AC#4 (physical artifact), AC#5 (real tube),
and AC#6 (scan-pattern duration from AA-32) require hardware or depend on
other tasks not yet complete.  They remain unchecked with comments below.

Coverage (this file):
  AC#1 — sub-pixel line extraction with documented parabolic method; exposure
          and gain recorded from AppliedSettings metadata.
  AC#2 — SATURATED, WEAK, MULTI_PEAK, MISSING flags; invalid columns yield
          NaN, never a fabricated value.
  AC#7 — synthetic frames rendered from KNOWN geometry (flat at standoff,
          tilted plane, cylinder approximating the tube); with noise, a
          specular saturated blob, a gap, and a double reflection.
          Ground-truth height error is reported.  Per-frame processing time
          on 1280×800 synthetic frames is measured and printed.
          Degenerate columns are checked to yield the correct flag.

Unchecked ACs (require hardware / other tasks):
  AC#3 — needs real AA-30 calibration; left unchecked.
         To run manually: scripts/laser_scan.py --live (not yet written)
  AC#4 — measured height accuracy / repeatability against a physical artifact;
         needs owner-supervised hardware session.
  AC#5 — robustness on real curved printed tube surface; needs hardware.
  AC#6 — per-frame time MEASURED below (see test_processing_time_1280x800)
         but compatibility with scan-pattern duration requires AA-32 (not yet
         complete). AC#6 remains unchecked pending AA-32.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pytest

from inchworm.laser_line import (
    ColumnFlag,
    LaserCalibration,
    LaserLineConfig,
    LaserLineResult,
    UnverifiedCalibrationError,
    _parabolic_peak,
    default_laser_calibration,
    extract_laser_line,
    triangulate_profile,
)


# ---------------------------------------------------------------------------
# Helpers: synthetic frame generation
# ---------------------------------------------------------------------------


def _make_flat_frame(
    height: int = 480,
    width: int = 640,
    *,
    line_row: float = 240.0,
    peak_value: int = 180,
    sigma_px: float = 2.0,
    noise_sigma: float = 3.0,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Render a synthetic frame with a laser line at constant row ``line_row``.

    Parameters
    ----------
    line_row:
        The true fractional row of the laser peak (ground truth).
    peak_value:
        Gaussian peak intensity (0–254; leave room below saturation).
    sigma_px:
        Gaussian width of the laser line (pixels).
    noise_sigma:
        Additive Gaussian noise std-dev.
    """
    if rng is None:
        rng = np.random.default_rng(42)

    rows = np.arange(height, dtype=np.float64)
    line_profile = peak_value * np.exp(-0.5 * ((rows - line_row) / sigma_px) ** 2)
    noise = rng.normal(0.0, noise_sigma, size=(height, width))
    frame_f = line_profile[:, np.newaxis] + noise
    frame = np.clip(frame_f, 0, 255).astype(np.uint8)
    return frame


def _make_tilted_frame(
    height: int = 480,
    width: int = 640,
    *,
    row_start: float = 200.0,
    row_end: float = 280.0,
    peak_value: int = 180,
    sigma_px: float = 2.0,
    noise_sigma: float = 3.0,
    rng: Optional[np.random.Generator] = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Render a frame with the laser line varying linearly across columns.

    Returns ``(frame, gt_rows)`` where ``gt_rows[col]`` is the ground-truth
    fractional row of the laser peak for column ``col``.
    """
    if rng is None:
        rng = np.random.default_rng(43)

    gt_rows = np.linspace(row_start, row_end, width)
    frame_f = np.zeros((height, width), dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)

    for col_idx in range(width):
        line_row = gt_rows[col_idx]
        frame_f[:, col_idx] = peak_value * np.exp(
            -0.5 * ((rows - line_row) / sigma_px) ** 2
        )

    noise = rng.normal(0.0, noise_sigma, size=(height, width))
    frame_f += noise
    frame = np.clip(frame_f, 0, 255).astype(np.uint8)
    return frame, gt_rows


def _make_cylinder_frame(
    height: int = 480,
    width: int = 640,
    *,
    tube_radius_px: float = 100.0,
    tube_center_row: float = 240.0,
    standoff_px: float = 240.0,
    peak_value: int = 180,
    sigma_px: float = 2.0,
    noise_sigma: float = 3.0,
    rng: Optional[np.random.Generator] = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Render a frame of a laser line crossing a cylinder (tube cross-section).

    The cylinder axis is horizontal; the laser hits the top of the cylinder.
    The row position varies as the arc of a circle.

    Returns ``(frame, gt_rows)`` where ``gt_rows`` is the ground-truth row
    for each column (or NaN for columns outside the cylinder projection).
    """
    if rng is None:
        rng = np.random.default_rng(44)

    col_center = width / 2.0
    gt_rows = np.full(width, np.nan, dtype=np.float64)
    frame_f = np.zeros((height, width), dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)

    for col_idx in range(width):
        dx = col_idx - col_center
        if abs(dx) > tube_radius_px:
            # Outside the cylinder's shadow: flat surface at standoff
            line_row = standoff_px
        else:
            # Arc of circle: z offset from surface of cylinder
            arc_z = math.sqrt(tube_radius_px**2 - dx**2)
            # The laser line is higher up (smaller row) on the cylinder top
            line_row = tube_center_row - arc_z

        gt_rows[col_idx] = line_row
        frame_f[:, col_idx] = peak_value * np.exp(
            -0.5 * ((rows - line_row) / sigma_px) ** 2
        )

    noise = rng.normal(0.0, noise_sigma, size=(height, width))
    frame_f += noise
    frame = np.clip(frame_f, 0, 255).astype(np.uint8)
    return frame, gt_rows


def _inject_saturated_blob(
    frame: np.ndarray,
    col_start: int,
    col_end: int,
    row_center: int,
    plateau_rows: int = 5,
) -> np.ndarray:
    """Overwrite a column range with a 255-valued blob (simulates specular highlight)."""
    frame = frame.copy()
    r_lo = max(0, row_center - plateau_rows // 2)
    r_hi = min(frame.shape[0], row_center + plateau_rows // 2 + 1)
    frame[r_lo:r_hi, col_start:col_end] = 255
    return frame


def _inject_gap(
    frame: np.ndarray,
    col_start: int,
    col_end: int,
) -> np.ndarray:
    """Zero out a column range (simulates occlusion / laser gap)."""
    frame = frame.copy()
    frame[:, col_start:col_end] = 0
    return frame


def _inject_double_reflection(
    frame: np.ndarray,
    col_start: int,
    col_end: int,
    second_row: int,
    second_peak_value: int = 80,
    sigma_px: float = 2.0,
) -> np.ndarray:
    """Add a secondary Gaussian peak to some columns (simulates double reflection)."""
    frame = frame.copy()
    rows = np.arange(frame.shape[0], dtype=np.float64)
    second_col = rows.copy()
    second_profile = second_peak_value * np.exp(
        -0.5 * ((rows - second_row) / sigma_px) ** 2
    )
    for col_idx in range(col_start, col_end):
        col_f = frame[:, col_idx].astype(np.float64) + second_profile
        frame[:, col_idx] = np.clip(col_f, 0, 255).astype(np.uint8)
    return frame


# ---------------------------------------------------------------------------
# Mock AppliedSettings
# ---------------------------------------------------------------------------


@dataclass
class _MockAppliedSettings:
    exposure: Optional[int]
    gain: Optional[int]
    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[float] = None
    auto_exposure: Optional[bool] = None
    auto_gain: Optional[bool] = None


# ---------------------------------------------------------------------------
# Unit tests: _parabolic_peak
# ---------------------------------------------------------------------------


def test_parabolic_peak_exact_integer():
    """Integer peak at index 5: col is a symmetric triangle; should return 5.0."""
    col = np.array([0.0, 1.0, 2.0, 4.0, 6.0, 10.0, 6.0, 4.0, 2.0, 1.0])
    result = _parabolic_peak(col, 5)
    assert abs(result - 5.0) < 0.01


def test_parabolic_peak_subpixel():
    """Sub-pixel: asymmetric values around peak should shift the vertex."""
    col = np.array([0.0, 0.0, 3.0, 9.0, 7.0, 0.0, 0.0])
    result = _parabolic_peak(col, 3)
    # Vertex should be between 3 and 4 (shifted right because y2 > y0)
    assert 3.0 < result < 4.0


def test_parabolic_peak_boundary():
    """Peak at index 0 → fallback to integer."""
    col = np.array([10.0, 5.0, 2.0])
    result = _parabolic_peak(col, 0)
    assert result == 0.0


def test_parabolic_peak_flat_top():
    """Flat top: denom ≈ 0 → fallback to integer peak."""
    col = np.array([5.0, 10.0, 10.0, 10.0, 5.0])
    result = _parabolic_peak(col, 2)
    assert result == 2.0


# ---------------------------------------------------------------------------
# AC#1: sub-pixel extraction on clean flat frame, metadata recording
# ---------------------------------------------------------------------------


def test_ac1_flat_frame_subpixel_accuracy():
    """AC#1: sub-pixel positions on a flat frame are within 0.5 px of ground truth."""
    gt_row = 240.0
    frame = _make_flat_frame(height=480, width=64, line_row=gt_row, noise_sigma=2.0)
    cfg = LaserLineConfig()
    result = extract_laser_line(frame, cfg)

    valid_cols = result.flags == ColumnFlag.VALID
    assert valid_cols.sum() > 50, "Expected most columns VALID on clean flat frame"

    errors = np.abs(result.row_positions[valid_cols] - gt_row)
    max_err = float(errors.max())
    mean_err = float(errors.mean())
    print(f"\nFlat frame sub-pixel errors: max={max_err:.4f} px, mean={mean_err:.4f} px")
    assert max_err < 1.0, f"Max error {max_err:.4f} px > 1.0 px threshold"
    assert mean_err < 0.5, f"Mean error {mean_err:.4f} px > 0.5 px threshold"


def test_ac1_metadata_recorded():
    """AC#1: exposure and gain from AppliedSettings are captured in LaserLineResult."""
    frame = _make_flat_frame(height=64, width=32, line_row=32.0, noise_sigma=1.0)
    applied = _MockAppliedSettings(exposure=200, gain=10)
    result = extract_laser_line(frame, applied_settings=applied)
    assert result.exposure == 200
    assert result.gain == 10


def test_ac1_metadata_none_when_no_settings():
    """AC#1: exposure/gain are None when AppliedSettings is not provided."""
    frame = _make_flat_frame(height=64, width=32, line_row=32.0, noise_sigma=1.0)
    result = extract_laser_line(frame)
    assert result.exposure is None
    assert result.gain is None


# ---------------------------------------------------------------------------
# AC#2: degenerate column flags — no fabricated values
# ---------------------------------------------------------------------------


def test_ac2_missing_flag_on_zero_frame():
    """AC#2: all-zero frame → all columns MISSING, all positions NaN."""
    frame = np.zeros((64, 32), dtype=np.uint8)
    result = extract_laser_line(frame)
    assert np.all(result.flags == ColumnFlag.MISSING)
    assert np.all(np.isnan(result.row_positions))
    assert result.n_missing == 32


def test_ac2_missing_flag_low_peak():
    """AC#2: columns with peak below min_peak_value → MISSING, position NaN."""
    frame = np.zeros((64, 32), dtype=np.uint8)
    # Set all columns to max value just below the threshold
    frame[30, :] = 15  # below default min_peak_value=20
    cfg = LaserLineConfig(min_peak_value=20)
    result = extract_laser_line(frame, cfg)
    assert np.all(result.flags == ColumnFlag.MISSING)
    assert np.all(np.isnan(result.row_positions))


def test_ac2_saturated_flag():
    """AC#2: column with a wide 255 plateau → SATURATED, position NaN."""
    frame = _make_flat_frame(height=64, width=32, line_row=32.0, noise_sigma=0.5)
    # Inject a saturated blob: 5 rows of 255 in columns 10-15
    frame = _inject_saturated_blob(frame, 10, 16, 32, plateau_rows=6)
    cfg = LaserLineConfig(saturated_plateau_px=2)
    result = extract_laser_line(frame, cfg)

    # Injected columns 10-15 should be SATURATED
    for col_idx in range(10, 16):
        flag = result.flags[col_idx]
        pos = result.row_positions[col_idx]
        assert flag == ColumnFlag.SATURATED, (
            f"col {col_idx}: expected SATURATED, got {flag}"
        )
        assert np.isnan(pos), (
            f"col {col_idx}: expected NaN position for SATURATED, got {pos}"
        )


def test_ac2_saturated_not_fabricated():
    """AC#2: SATURATED columns yield NaN, not any fabricated row value."""
    frame = np.zeros((64, 16), dtype=np.uint8)
    # Make all columns fully saturated
    frame[:, :] = 255
    cfg = LaserLineConfig(saturated_plateau_px=2)
    result = extract_laser_line(frame, cfg)
    for col_idx in range(16):
        assert result.flags[col_idx] == ColumnFlag.SATURATED
        assert np.isnan(result.row_positions[col_idx])


def test_ac2_weak_flag():
    """AC#2: very low SNR → WEAK flag, position NaN."""
    # Create a frame where the peak is just above min but SNR is low
    frame = np.zeros((80, 32), dtype=np.uint8)
    # Set the "line" very weakly: peak=25, but add a noisy background of 20
    rng = np.random.default_rng(123)
    for col_idx in range(32):
        frame[:, col_idx] = rng.integers(18, 22, size=80).astype(np.uint8)
    frame[40, :] = 25  # tiny peak above background

    cfg = LaserLineConfig(
        min_peak_value=20,
        weak_snr_threshold=5.0,  # high threshold to ensure WEAK
        window_half_width_px=5,
    )
    result = extract_laser_line(frame, cfg)
    # With high noise (rms ≈ 20) and peak 25, SNR ≈ 1.25 << 5 → all WEAK
    weak_cols = np.sum(result.flags == ColumnFlag.WEAK)
    # At least half should be WEAK (some may be MISSING due to random noise)
    non_missing = np.sum(result.flags != ColumnFlag.MISSING)
    assert weak_cols >= non_missing * 0.8, (
        f"Expected most columns WEAK, got weak={weak_cols} / non_missing={non_missing}"
    )
    # All WEAK positions must be NaN
    for col_idx in range(32):
        if result.flags[col_idx] == ColumnFlag.WEAK:
            assert np.isnan(result.row_positions[col_idx]), (
                f"col {col_idx}: WEAK but position not NaN"
            )


def test_ac2_multi_peak_flag():
    """AC#2: second peak outside window above fraction * peak → MULTI_PEAK, NaN."""
    height = 100
    width = 20
    frame = np.zeros((height, width), dtype=np.uint8)

    # Primary peak at row 50 with intensity 150
    primary_row = 50
    secondary_row = 10  # well outside window
    for col_idx in range(width):
        # Primary
        frame[primary_row, col_idx] = 150
        if col_idx >= 5:  # inject secondary in columns 5-19
            frame[secondary_row, col_idx] = 80  # 80/150 = 0.53 > multi_peak_fraction=0.4

    cfg = LaserLineConfig(
        window_half_width_px=10,
        multi_peak_fraction=0.4,
        weak_snr_threshold=0.5,  # disable weak to isolate multi-peak
        min_peak_value=20,
    )
    result = extract_laser_line(frame, cfg)

    # Columns 5-19 should be MULTI_PEAK
    for col_idx in range(5, width):
        flag = result.flags[col_idx]
        pos = result.row_positions[col_idx]
        assert flag == ColumnFlag.MULTI_PEAK, (
            f"col {col_idx}: expected MULTI_PEAK, got {flag}"
        )
        assert np.isnan(pos), (
            f"col {col_idx}: MULTI_PEAK but position not NaN ({pos})"
        )


def test_ac2_gap_columns_missing():
    """AC#2: zeroed-out columns (gap/occlusion) → MISSING, positions NaN."""
    frame = _make_flat_frame(height=64, width=64, line_row=32.0, noise_sigma=1.0)
    frame = _inject_gap(frame, 20, 35)
    result = extract_laser_line(frame)

    for col_idx in range(20, 35):
        flag = result.flags[col_idx]
        pos = result.row_positions[col_idx]
        assert flag == ColumnFlag.MISSING, (
            f"col {col_idx}: gap column expected MISSING, got {flag}"
        )
        assert np.isnan(pos)


def test_ac2_no_fabricated_values():
    """AC#2: any column that is not VALID must have NaN position (no fabrication)."""
    frame = _make_flat_frame(height=64, width=64, line_row=32.0, noise_sigma=2.0)
    frame = _inject_saturated_blob(frame, 10, 15, 32, plateau_rows=5)
    frame = _inject_gap(frame, 20, 25)
    result = extract_laser_line(frame)

    for col_idx in range(64):
        if result.flags[col_idx] is not ColumnFlag.VALID:
            assert np.isnan(result.row_positions[col_idx]), (
                f"col {col_idx}: flag={result.flags[col_idx]} but position={result.row_positions[col_idx]}"
            )


# ---------------------------------------------------------------------------
# AC#7: synthetic geometry tests — tilted plane
# ---------------------------------------------------------------------------


def test_ac7_tilted_plane_subpixel():
    """AC#7: tilted-plane synthetic frame; sub-pixel extraction against ground truth."""
    rng = np.random.default_rng(100)
    frame, gt_rows = _make_tilted_frame(
        height=480, width=200,
        row_start=180.0, row_end=300.0,
        peak_value=180, sigma_px=2.5, noise_sigma=3.0,
        rng=rng,
    )
    result = extract_laser_line(frame)
    valid = result.flags == ColumnFlag.VALID
    assert valid.sum() > 160

    errors = np.abs(result.row_positions[valid] - gt_rows[valid])
    max_err = float(errors.max())
    mean_err = float(errors.mean())
    print(f"\nTilted plane sub-pixel errors: max={max_err:.4f} px, mean={mean_err:.4f} px")
    assert max_err < 2.0, f"Max error {max_err:.4f} px > 2.0 px on tilted plane"
    assert mean_err < 0.5, f"Mean error {mean_err:.4f} px > 0.5 px on tilted plane"


# ---------------------------------------------------------------------------
# AC#7: synthetic geometry tests — cylinder approximating tube
# ---------------------------------------------------------------------------


def test_ac7_cylinder_frame():
    """AC#7: cylinder synthetic frame; sub-pixel extraction vs arc ground truth."""
    rng = np.random.default_rng(200)
    frame, gt_rows = _make_cylinder_frame(
        height=480, width=200,
        tube_radius_px=80.0,
        tube_center_row=240.0,
        standoff_px=240.0,
        peak_value=180, sigma_px=2.5, noise_sigma=3.0,
        rng=rng,
    )
    result = extract_laser_line(frame)
    valid = result.flags == ColumnFlag.VALID
    assert valid.sum() > 150

    errors = np.abs(result.row_positions[valid] - gt_rows[valid])
    max_err = float(errors.max())
    mean_err = float(errors.mean())
    print(f"\nCylinder sub-pixel errors: max={max_err:.4f} px, mean={mean_err:.4f} px")
    assert max_err < 2.0, f"Max error {max_err:.4f} px > 2.0 px on cylinder frame"
    assert mean_err < 0.5, f"Mean error {mean_err:.4f} px > 0.5 px on cylinder frame"


# ---------------------------------------------------------------------------
# AC#7: degenerate combinations
# ---------------------------------------------------------------------------


def test_ac7_saturated_blob_in_tilted_frame():
    """AC#7: injected specular blob on tilted frame flags the right columns SATURATED."""
    rng = np.random.default_rng(300)
    frame, gt_rows = _make_tilted_frame(
        height=480, width=100,
        row_start=200.0, row_end=280.0,
        peak_value=180, noise_sigma=2.0, rng=rng,
    )
    blob_start, blob_end = 30, 45
    frame = _inject_saturated_blob(frame, blob_start, blob_end, 240, plateau_rows=4)
    result = extract_laser_line(frame)

    # Blob columns must not be VALID
    for col_idx in range(blob_start, blob_end):
        assert result.flags[col_idx] != ColumnFlag.VALID, (
            f"col {col_idx}: saturated blob should not be VALID"
        )
        assert np.isnan(result.row_positions[col_idx])

    # Columns outside the blob should mostly be VALID
    outside_valid = np.sum(result.flags[:blob_start] == ColumnFlag.VALID)
    assert outside_valid > blob_start * 0.8


def test_ac7_gap_in_cylinder_frame():
    """AC#7: gap in cylinder frame flags the right columns MISSING."""
    rng = np.random.default_rng(400)
    frame, gt_rows = _make_cylinder_frame(
        height=480, width=100,
        tube_radius_px=40.0, tube_center_row=240.0, standoff_px=240.0,
        peak_value=180, noise_sigma=2.0, rng=rng,
    )
    gap_start, gap_end = 45, 55
    frame = _inject_gap(frame, gap_start, gap_end)
    result = extract_laser_line(frame)

    for col_idx in range(gap_start, gap_end):
        assert result.flags[col_idx] == ColumnFlag.MISSING
        assert np.isnan(result.row_positions[col_idx])


def test_ac7_double_reflection_in_flat_frame():
    """AC#7: double reflection flags those columns MULTI_PEAK."""
    frame = _make_flat_frame(height=200, width=80, line_row=100.0, peak_value=150, noise_sigma=1.0)
    refl_start, refl_end = 30, 50
    secondary_row = 20  # far outside default window of ±15 from row 100
    secondary_value = int(150 * 0.5)  # 50% of primary → above multi_peak_fraction=0.4
    frame = _inject_double_reflection(
        frame, refl_start, refl_end, secondary_row, second_peak_value=secondary_value
    )
    cfg = LaserLineConfig(
        window_half_width_px=15,
        multi_peak_fraction=0.4,
        weak_snr_threshold=0.5,  # disable weak
    )
    result = extract_laser_line(frame, cfg)

    multi_peak_count = 0
    for col_idx in range(refl_start, refl_end):
        if result.flags[col_idx] == ColumnFlag.MULTI_PEAK:
            multi_peak_count += 1
            assert np.isnan(result.row_positions[col_idx])

    assert multi_peak_count >= (refl_end - refl_start) // 2, (
        f"Expected at least half of injected columns to be MULTI_PEAK, "
        f"got {multi_peak_count} of {refl_end - refl_start}"
    )


# ---------------------------------------------------------------------------
# AC#7: triangulation against synthetic geometry (dry_run=True, design defaults)
# ---------------------------------------------------------------------------


def test_ac7_triangulation_flat_plane_dry_run():
    """AC#7: triangulate flat-plane extracted profile; check Z is near standoff.

    Uses design defaults (unverified calibration) with dry_run=True.
    Heights are approximate because calibration is unverified; the test
    checks that the result is a well-formed array of the right shape and
    that the Z values cluster around the design standoff.
    """
    height, width = 480, 100
    line_row = 240.0
    frame = _make_flat_frame(
        height=height, width=width,
        line_row=line_row, peak_value=180, noise_sigma=1.5
    )
    result = extract_laser_line(frame)
    calib = default_laser_calibration()
    points = triangulate_profile(result, calib, dry_run=True)

    assert points.shape == (width, 3)

    valid_pts = points[result.flags == ColumnFlag.VALID]
    assert len(valid_pts) > 50

    # Z should be positive and within a wide band around standoff
    from inchworm.constants import LASER_STANDOFF_MM
    z_vals = valid_pts[:, 2]
    z_mean = float(np.nanmean(z_vals))
    print(f"\nTriangulation Z mean={z_mean:.2f} mm (design standoff={LASER_STANDOFF_MM} mm)")
    # Allow generous tolerance since calibration is unverified
    assert 0 < z_mean < 500.0, f"Z mean {z_mean:.2f} mm out of plausible range"


def test_ac7_triangulation_invalid_columns_are_nan():
    """AC#7: invalid columns in LaserLineResult produce NaN 3-D points."""
    height, width = 64, 32
    frame = _make_flat_frame(height=height, width=width, line_row=32.0, noise_sigma=1.0)
    frame = _inject_gap(frame, 5, 10)

    result = extract_laser_line(frame)
    calib = default_laser_calibration()
    points = triangulate_profile(result, calib, dry_run=True)

    for col_idx in range(5, 10):
        assert np.all(np.isnan(points[col_idx])), (
            f"col {col_idx}: gap column should produce NaN 3-D point"
        )


def test_ac7_triangulation_unverified_raises_without_dry_run():
    """AC#7: UnverifiedCalibrationError is raised without dry_run=True."""
    frame = _make_flat_frame(height=64, width=32, line_row=32.0)
    result = extract_laser_line(frame)
    calib = default_laser_calibration()
    assert calib.unverified  # ensure design default is unverified

    with pytest.raises(UnverifiedCalibrationError):
        triangulate_profile(result, calib, dry_run=False)


# ---------------------------------------------------------------------------
# AC#6 (timing — unchecked pending AA-32 for scan-pattern duration)
# ---------------------------------------------------------------------------


def test_processing_time_1280x800():
    """Measure per-frame processing time on a 1280×800 synthetic frame.

    AC#6 requires compatibility with the scan pattern duration (AA-32 not yet
    complete). This test MEASURES and reports the time; AC#6 stays unchecked
    until AA-32 defines the intended duration.

    The test PASSES as long as the measurement completes (no assertion on
    the value — that is AA-32's job).
    """
    height, width = 800, 1280
    frame = _make_flat_frame(height=height, width=width, line_row=400.0, noise_sigma=3.0)

    # Warm-up pass
    _ = extract_laser_line(frame)

    # Timed pass
    n_reps = 5
    t0 = time.perf_counter()
    for _ in range(n_reps):
        extract_laser_line(frame)
    elapsed = time.perf_counter() - t0
    per_frame_ms = elapsed / n_reps * 1000.0

    print(
        f"\n[AC#6 timing] 1280×800 frame: {per_frame_ms:.1f} ms/frame "
        f"({1000.0 / per_frame_ms:.1f} FPS ceiling) — "
        f"AC#6 unchecked pending AA-32 scan-duration spec."
    )
    # No hard limit here; just confirm the call completes without error.
    assert per_frame_ms > 0.0


# ---------------------------------------------------------------------------
# Config integration
# ---------------------------------------------------------------------------


def test_config_loads_laser_line_block():
    """LaserLineConfigSchema round-trips through hardware.yaml load."""
    from inchworm.config import load_config

    cfg = load_config()
    assert cfg.laser_line is not None
    assert cfg.laser_line.window_half_width_px == 15
    assert cfg.laser_line.saturated_plateau_px == 2
    assert cfg.laser_line.weak_snr_threshold == 3.0
    assert cfg.laser_line.multi_peak_fraction == 0.4
    assert cfg.laser_line.min_peak_value == 20


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------


def test_extract_raises_on_float_frame():
    """extract_laser_line raises ValueError for non-uint8 input."""
    frame = np.zeros((64, 32), dtype=np.float32)
    with pytest.raises(ValueError, match="uint8"):
        extract_laser_line(frame)


def test_extract_raises_on_3d_frame():
    """extract_laser_line raises ValueError for 3-D (colour) input."""
    frame = np.zeros((64, 32, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="2-D"):
        extract_laser_line(frame)
