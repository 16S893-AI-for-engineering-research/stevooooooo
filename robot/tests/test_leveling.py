"""Tests for AA-32 leveling.py — scan pattern, stitching, surface fit.

All tests use synthetic data with known ground truth.  Hardware-dependent
ACs (#1, #5, #6) are not tested here; see scripts/level_scan.py --live.

Test coverage:
- AC#2: stitching correctness (known arm position + identity T_tool_scanner)
- AC#3: plane fit with known tilt/Z-offset; Z recovery error reported
- AC#4: LevelingRefused on sparse coverage, high residual, high uncertainty
- AC#7: laser is ON only during capture and OFF between stops / on exceptions
- AC#8: synthetic recovery (documented error), outlier rejection, refusal paths
"""

from __future__ import annotations

import math
from typing import Optional
from unittest.mock import MagicMock

import numpy as np
import pytest

from inchworm.leveling import (
    LevelingRefused,
    LevelingResult,
    ScanConfig,
    _fit_plane_ols,
    fit_surface,
    run_scan,
    scan_pattern,
    stitch_profiles,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _MockArm:
    """Minimal DexArmDriver stub that records calls and returns commanded pos."""

    def __init__(self) -> None:
        self.moves: list[tuple] = []
        self._last_x: float = 0.0
        self._last_y: float = 0.0
        self._last_z: float = 0.0

    def move_absolute(
        self,
        x: float,
        y: float,
        z: float,
        feedrate=None,
        *,
        dry_run: bool = False,
    ) -> None:
        self.moves.append((x, y, z))
        self._last_x = x
        self._last_y = y
        self._last_z = z

    def get_position(self):
        pos = MagicMock()
        pos.x = self._last_x
        pos.y = self._last_y
        pos.z = self._last_z
        return pos


class _MockLaser:
    """Minimal LaserDriver stub that tracks on/off calls.

    Used to verify the laser is ON only during capture and OFF between stops.
    """

    def __init__(self) -> None:
        self._on: bool = False
        self.on_events: list[str] = []  # "on" or "off"

    @property
    def commanded_on(self) -> bool:
        return self._on

    def on(self, duration_s: Optional[float] = None) -> None:
        assert not self._on, "laser.on() called while already ON"
        self._on = True
        self.on_events.append("on")

    def off(self) -> None:
        self._on = False
        self.on_events.append("off")

    # Context manager mirrors the REAL LaserDriver: __enter__ does nothing,
    # __exit__ turns off (orchestrator fix 2026-10-24; the old mock turned
    # the laser on in __enter__, which the real driver does not).
    def __enter__(self) -> "_MockLaser":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.off()


# ---------------------------------------------------------------------------
# scan_pattern
# ---------------------------------------------------------------------------


class TestScanPattern:
    def test_count(self) -> None:
        cfg = ScanConfig(scan_x_steps=3, scan_y_steps=4)
        stops = scan_pattern(cfg)
        assert len(stops) == 12

    def test_boustrophedon(self) -> None:
        """Odd rows should be right-to-left (boustrophedon)."""
        cfg = ScanConfig(scan_x_steps=3, scan_y_steps=2)
        stops = scan_pattern(cfg)
        # Row 0 (y=y_min): left→right
        row0 = stops[:3]
        xs0 = [s[0] for s in row0]
        assert xs0 == sorted(xs0), "row 0 should be left-to-right"
        # Row 1 (y=y_max): right→left
        row1 = stops[3:]
        xs1 = [s[0] for s in row1]
        assert xs1 == sorted(xs1, reverse=True), "row 1 should be right-to-left"

    def test_x_extent(self) -> None:
        cfg = ScanConfig(scan_x_min_mm=-10.0, scan_x_max_mm=10.0, scan_x_steps=2, scan_y_steps=2)
        stops = scan_pattern(cfg)
        xs = [s[0] for s in stops]
        assert min(xs) == pytest.approx(-10.0)
        assert max(xs) == pytest.approx(10.0)

    def test_raises_on_single_step(self) -> None:
        cfg = ScanConfig(scan_x_steps=1, scan_y_steps=2)
        with pytest.raises(ValueError, match="scan_x_steps"):
            scan_pattern(cfg)


# ---------------------------------------------------------------------------
# run_scan — laser on/off enforcement (AC#7)
# ---------------------------------------------------------------------------


class TestRunScanLaserEnforcement:
    def _make_config(self) -> ScanConfig:
        return ScanConfig(
            scan_x_steps=2,
            scan_y_steps=2,
            settle_wait_s=0.0,
            standoff_z_mm=150.0,
        )

    def test_laser_off_between_stops(self) -> None:
        """Laser must be OFF between stops (not captured between context managers)."""
        arm = _MockArm()
        laser = _MockLaser()
        cfg = self._make_config()
        n_stops = cfg.scan_x_steps * cfg.scan_y_steps  # 4

        # Live path with mock arm/laser/capture (no hardware) so the laser is
        # really switched (orchestrator fix 2026-10-24).
        run_scan(arm, laser, capture_fn=lambda: None, extract_fn=lambda f: None,
                 config=cfg, dry_run=False)

        # After all stops the laser should be off.
        assert not laser.commanded_on

        # Pattern should be: on → off → on → off → … (n_stops pairs).
        assert laser.on_events.count("on") == n_stops
        assert laser.on_events.count("off") == n_stops

        # Every "on" should be immediately followed by "off".
        for i in range(0, len(laser.on_events) - 1, 2):
            assert laser.on_events[i] == "on"
            assert laser.on_events[i + 1] == "off"

    def test_laser_off_on_exception(self) -> None:
        """Laser must be OFF even if capture_fn raises (AA-7 / AA-29 invariant)."""
        arm = _MockArm()
        laser = _MockLaser()
        cfg = self._make_config()

        call_count = [0]

        def bad_capture():
            call_count[0] += 1
            raise RuntimeError("simulated capture failure")

        # Run with dry_run=True: capture is synthetic, no real raises expected.
        # To test real exception path we need dry_run=False.
        # Use dry_run=False but a capture_fn that raises on the first real call.
        cfg2 = ScanConfig(
            scan_x_steps=2,
            scan_y_steps=2,
            settle_wait_s=0.0,
            standoff_z_mm=150.0,
        )
        # run_scan with dry_run=False will call capture_fn.
        with pytest.raises(RuntimeError, match="simulated capture failure"):
            run_scan(arm, laser, capture_fn=bad_capture, extract_fn=lambda f: None,
                     config=cfg2, dry_run=False)

        # Laser must be off after exception (context manager __exit__ was called).
        assert not laser.commanded_on

    def test_arm_moves_to_every_stop(self) -> None:
        arm = _MockArm()
        laser = _MockLaser()
        cfg = self._make_config()

        run_scan(arm, laser, capture_fn=lambda: None, extract_fn=lambda f: None,
                 config=cfg, dry_run=True)

        n_stops = cfg.scan_x_steps * cfg.scan_y_steps
        assert len(arm.moves) == n_stops

    def test_live_fails_closed_without_standoff(self) -> None:
        arm = _MockArm()
        laser = _MockLaser()
        cfg = ScanConfig(scan_x_steps=2, scan_y_steps=2, standoff_z_mm=None)

        with pytest.raises(LevelingRefused) as exc_info:
            run_scan(arm, laser, capture_fn=lambda: None, extract_fn=lambda f: None,
                     config=cfg, dry_run=False)
        assert exc_info.value.reason == "config_unverified"


# ---------------------------------------------------------------------------
# stitch_profiles (AC#2)
# ---------------------------------------------------------------------------


class TestStitchProfiles:
    def test_identity_transform_adds_arm_position(self) -> None:
        """With T_tool_scanner = I, profile points should be shifted by arm position."""
        arm_pos = (10.0, 20.0, 30.0)
        pts_scanner = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float64)
        scan_data = [(arm_pos, pts_scanner)]

        cloud, unverified = stitch_profiles(scan_data, T_tool_scanner=np.eye(4))
        assert unverified is False
        assert cloud.shape == (2, 3)
        expected = pts_scanner + np.array([10.0, 20.0, 30.0])
        np.testing.assert_allclose(cloud, expected, atol=1e-10)

    def test_custom_T_tool_scanner(self) -> None:
        """Non-identity T_tool_scanner rotates and translates scanner points."""
        arm_pos = (0.0, 0.0, 0.0)
        pts_scanner = np.array([[0.0, 0.0, 5.0]], dtype=np.float64)  # along scanner Z

        # T_tool_scanner: translate by [1, 0, 0] in tool frame
        T = np.eye(4, dtype=np.float64)
        T[0, 3] = 1.0  # shift X by 1 mm in tool frame

        scan_data = [(arm_pos, pts_scanner)]
        cloud, unverified = stitch_profiles(scan_data, T_tool_scanner=T)
        assert unverified is False
        # P_tool = R @ P_scanner + t = I @ [0,0,5] + [1,0,0] = [1,0,5]
        # P_print = [1,0,5] + [0,0,0] = [1,0,5]
        np.testing.assert_allclose(cloud[0], [1.0, 0.0, 5.0], atol=1e-10)

    def test_filters_nan_points(self) -> None:
        arm_pos = (0.0, 0.0, 0.0)
        pts = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0], [4.0, 5.0, 6.0]], dtype=np.float64)
        scan_data = [(arm_pos, pts)]
        cloud, _ = stitch_profiles(scan_data, T_tool_scanner=np.eye(4))
        assert cloud.shape == (2, 3), "NaN row should be filtered out"

    def test_empty_profile_skipped(self) -> None:
        scan_data = [
            ((0.0, 0.0, 0.0), np.zeros((0, 3), dtype=np.float64)),
            ((1.0, 0.0, 0.0), np.array([[1.0, 2.0, 3.0]], dtype=np.float64)),
        ]
        cloud, _ = stitch_profiles(scan_data, T_tool_scanner=np.eye(4))
        assert cloud.shape[0] == 1

    def test_unverified_flag_set_for_default(self) -> None:
        scan_data = [((0.0, 0.0, 0.0), np.array([[1.0, 0.0, 0.0]]))]
        _, unverified = stitch_profiles(scan_data, T_tool_scanner=None)
        assert unverified is True


# ---------------------------------------------------------------------------
# fit_surface — known tilt/Z recovery (AC#3, AC#6, AC#8)
# ---------------------------------------------------------------------------


def _make_plane_cloud(
    a: float, b: float, c: float,
    n: int = 100,
    noise_std: float = 0.01,
    seed: int = 0,
) -> np.ndarray:
    """Generate N points from Z = a*X + b*Y + c with Gaussian noise."""
    rng = np.random.default_rng(seed=seed)
    X = rng.uniform(-20.0, 20.0, n)
    Y = rng.uniform(-20.0, 20.0, n)
    Z = a * X + b * Y + c + rng.normal(0.0, noise_std, n)
    return np.column_stack([X, Y, Z]).astype(np.float64)


class TestFitSurface:
    def _relaxed_config(self) -> ScanConfig:
        return ScanConfig(
            min_valid_points=5,
            min_inlier_fraction=0.3,
            max_residual_rms_mm=5.0,
            max_uncertainty_mm=10.0,
            ransac_inlier_threshold_mm=1.0,
            ransac_trials=500,
        )

    def test_flat_surface_recovery(self) -> None:
        """Flat surface: rx=0, ry=0, z=50; recovery error reported."""
        c_true = 50.0
        cloud = _make_plane_cloud(0.0, 0.0, c_true, n=200, noise_std=0.02)
        result = fit_surface(cloud, self._relaxed_config(), T_tool_scanner_unverified=False)
        z_error = abs(result.z_offset_mm - c_true)
        rx_error = abs(result.rx_rad)
        ry_error = abs(result.ry_rad)
        print(f"\n[synthetic] flat: z_error={z_error:.4f} mm, rx_err={rx_error:.6f} rad, ry_err={ry_error:.6f} rad")
        assert z_error < 0.5, f"Z recovery error {z_error:.4f} mm > 0.5 mm"
        assert rx_error < 0.01
        assert ry_error < 0.01

    def test_known_tilt_z_recovery(self) -> None:
        """Known tilt (rx≈5°, ry≈3°) and Z offset (80 mm) recovered within tolerance.

        AC#6 (synthetic): recovered vs. true error is reported.
        Note: AC#6 says 'introduced deliberately'; this is a synthetic test.
        Hardware AC#6 remains unchecked.
        """
        # Z = a*X + b*Y + c
        # ry = arctan(a) → a = tan(ry)
        # rx = -arctan(b) → b = -tan(rx)
        rx_true = math.radians(5.0)
        ry_true = math.radians(3.0)
        c_true = 80.0
        a_true = math.tan(ry_true)
        b_true = -math.tan(rx_true)

        rng = np.random.default_rng(seed=7)
        n = 300
        X = rng.uniform(-25.0, 25.0, n)
        Y = rng.uniform(-25.0, 25.0, n)
        noise = rng.normal(0.0, 0.05, n)
        Z = a_true * X + b_true * Y + c_true + noise
        cloud = np.column_stack([X, Y, Z]).astype(np.float64)

        result = fit_surface(cloud, self._relaxed_config(), T_tool_scanner_unverified=False)

        z_err = abs(result.z_offset_mm - c_true)
        rx_err = abs(result.rx_rad - rx_true)
        ry_err = abs(result.ry_rad - ry_true)
        print(
            f"\n[synthetic tilt recovery] "
            f"z_err={z_err:.4f} mm (true={c_true}), "
            f"rx_err={math.degrees(rx_err):.4f} deg (true={math.degrees(rx_true):.2f}°), "
            f"ry_err={math.degrees(ry_err):.4f} deg (true={math.degrees(ry_true):.2f}°)"
        )
        assert z_err < 0.5, f"Z recovery error {z_err:.4f} mm > 0.5 mm"
        assert math.degrees(rx_err) < 0.5, f"rx recovery error {math.degrees(rx_err):.4f} deg > 0.5°"
        assert math.degrees(ry_err) < 0.5, f"ry recovery error {math.degrees(ry_err):.4f} deg > 0.5°"

    def test_outlier_rejection(self) -> None:
        """RANSAC must reject outlier points (AC#8)."""
        # Generate inlier plane
        rng = np.random.default_rng(seed=3)
        n_in = 200
        X = rng.uniform(-20.0, 20.0, n_in)
        Y = rng.uniform(-20.0, 20.0, n_in)
        Z_in = 0.1 * X + 0.0 * Y + 50.0 + rng.normal(0.0, 0.05, n_in)
        inliers = np.column_stack([X, Y, Z_in])

        # Add 30 gross outliers (Z offset by 20 mm)
        n_out = 30
        X_out = rng.uniform(-20.0, 20.0, n_out)
        Y_out = rng.uniform(-20.0, 20.0, n_out)
        Z_out = np.full(n_out, 70.0)  # 20 mm above the plane
        outliers = np.column_stack([X_out, Y_out, Z_out])

        cloud = np.vstack([inliers, outliers]).astype(np.float64)

        cfg = ScanConfig(
            min_valid_points=5,
            min_inlier_fraction=0.3,
            max_residual_rms_mm=5.0,
            max_uncertainty_mm=10.0,
            ransac_inlier_threshold_mm=0.5,
            ransac_trials=500,
        )
        result = fit_surface(cloud, cfg, T_tool_scanner_unverified=False)

        # Inlier count should be close to n_in (≥ 80%).
        assert result.n_inliers >= int(0.8 * n_in), \
            f"expected ≥{int(0.8 * n_in)} inliers, got {result.n_inliers}"
        # Z recovery should be close to 50 mm
        z_err = abs(result.z_offset_mm - 50.0)
        print(f"\n[outlier rejection] z_err={z_err:.4f} mm, inliers={result.n_inliers}/{len(cloud)}")
        assert z_err < 1.0

    def test_sparse_coverage_refused(self) -> None:
        """Fewer than min_valid_points points → LevelingRefused (AC#4)."""
        cfg = ScanConfig(min_valid_points=20, ransac_trials=10)
        cloud = _make_plane_cloud(0.0, 0.0, 50.0, n=5)
        with pytest.raises(LevelingRefused) as exc_info:
            fit_surface(cloud, cfg)
        assert exc_info.value.reason == "insufficient_coverage"

    def test_high_residual_refused(self) -> None:
        """Noisy data where residual RMS exceeds threshold → LevelingRefused (AC#4)."""
        # Very noisy cloud (noise >> threshold)
        rng = np.random.default_rng(seed=5)
        n = 100
        X = rng.uniform(-20.0, 20.0, n)
        Y = rng.uniform(-20.0, 20.0, n)
        Z = 50.0 + rng.normal(0.0, 10.0, n)  # 10 mm std dev noise
        cloud = np.column_stack([X, Y, Z]).astype(np.float64)

        # Very tight residual threshold to force refusal
        cfg = ScanConfig(
            min_valid_points=5,
            min_inlier_fraction=0.05,  # very loose
            max_residual_rms_mm=0.1,   # very tight
            max_uncertainty_mm=0.01,
            ransac_inlier_threshold_mm=0.1,
            ransac_trials=100,
        )
        with pytest.raises(LevelingRefused) as exc_info:
            fit_surface(cloud, cfg)
        # Could be high_residual, high_uncertainty, or no_inliers / insufficient_coverage
        # depending on how RANSAC lands; accept all refusal reasons
        assert exc_info.value.reason in (
            "high_residual", "high_uncertainty", "no_inliers", "insufficient_coverage"
        )

    def test_empty_cloud_refused(self) -> None:
        cloud = np.zeros((0, 3), dtype=np.float64)
        with pytest.raises(LevelingRefused) as exc_info:
            fit_surface(cloud, ScanConfig())
        assert exc_info.value.reason == "insufficient_coverage"

    def test_result_fields(self) -> None:
        """LevelingResult has all expected fields with plausible values."""
        cloud = _make_plane_cloud(0.05, -0.03, 60.0, n=200, noise_std=0.02)
        result = fit_surface(cloud, self._relaxed_config(), T_tool_scanner_unverified=True)
        assert isinstance(result.z_offset_mm, float)
        assert isinstance(result.rx_rad, float)
        assert isinstance(result.ry_rad, float)
        assert result.residual_rms_mm >= 0.0
        assert result.uncertainty_mm >= 0.0
        assert result.n_inliers > 0
        assert result.n_total >= result.n_inliers
        assert result.T_tool_scanner_unverified is True
        assert result.plane_normal.shape == (3,)
        assert abs(np.linalg.norm(result.plane_normal) - 1.0) < 1e-9


# ---------------------------------------------------------------------------
# _fit_plane_ols helper
# ---------------------------------------------------------------------------


class TestFitPlaneOls:
    def test_exact_plane(self) -> None:
        X = np.array([0.0, 1.0, 0.0, 1.0, 2.0], dtype=np.float64)
        Y = np.array([0.0, 0.0, 1.0, 1.0, 0.5], dtype=np.float64)
        a_true, b_true, c_true = 0.1, -0.2, 5.0
        Z = a_true * X + b_true * Y + c_true
        a, b, c, cov = _fit_plane_ols(X, Y, Z)
        assert abs(a - a_true) < 1e-9
        assert abs(b - b_true) < 1e-9
        assert abs(c - c_true) < 1e-9
        assert cov is not None
        assert cov.shape == (3, 3)


# ---------------------------------------------------------------------------
# End-to-end dry-run pipeline
# ---------------------------------------------------------------------------


class TestDryRunPipeline:
    def test_dry_run_returns_empty_cloud(self) -> None:
        """Dry-run produces no real points; fit_surface should refuse (empty cloud)."""
        from inchworm.leveling import level_scan

        arm = _MockArm()
        laser = _MockLaser()
        cfg = ScanConfig(scan_x_steps=2, scan_y_steps=2, standoff_z_mm=150.0)

        with pytest.raises(LevelingRefused) as exc_info:
            level_scan(
                arm, laser,
                capture_fn=lambda: None,
                extract_fn=lambda f: None,
                config=cfg,
                dry_run=True,
            )
        # dry-run returns no points → insufficient_coverage
        assert exc_info.value.reason == "insufficient_coverage"

    def test_dry_run_laser_off_at_end(self) -> None:
        """After a dry-run scan, laser must be off regardless."""
        arm = _MockArm()
        laser = _MockLaser()
        cfg = ScanConfig(scan_x_steps=2, scan_y_steps=2, standoff_z_mm=150.0)

        try:
            from inchworm.leveling import level_scan
            level_scan(
                arm, laser,
                capture_fn=lambda: None,
                extract_fn=lambda f: None,
                config=cfg,
                dry_run=True,
            )
        except LevelingRefused:
            pass
        assert not laser.commanded_on



# --- Orchestrator 2026-10-24: run_scan against the REAL LaserDriver ---
def test_run_scan_real_laser_driver_on_during_capture_off_after():
    from inchworm.backends import MockSerialTransport
    from inchworm.config import load_config
    from inchworm.laser import LaserDriver, Lcus1Protocol
    relay = MockSerialTransport("relay")
    drv = LaserDriver(relay, load_config())
    on_b, off_b = Lcus1Protocol().encode_on(), Lcus1Protocol().encode_off()
    states_during_capture = []

    def capture():
        states_during_capture.append(drv.commanded_on)
        return None

    cfg = ScanConfig(scan_x_steps=2, scan_y_steps=2, settle_wait_s=0.0, standoff_z_mm=150.0)
    run_scan(_MockArm(), drv, capture_fn=capture, extract_fn=lambda f: None,
             config=cfg, dry_run=False)
    assert states_during_capture == [True] * 4
    assert not drv.commanded_on
    assert relay.written.count(on_b) == 4 and relay.written[-1] == off_b
    drv.close()
