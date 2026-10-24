"""Tests for AA-16: Chassis-arm-tube calibration solvers and stale-warning path.

Evidence matrix (AC#4 and AC#7):
- AC#4: test_result_dict_round_trip, test_tube_axis_dict_round_trip,
        test_stale_calibration_warning_missing, test_stale_calibration_warning_old,
        test_fresh_calibration_no_warning
- AC#7: test_kabsch_identity, test_kabsch_known_transform,
        test_kabsch_known_transform_with_noise, test_kabsch_rotation_only,
        test_kabsch_translation_only, test_kabsch_requires_3_points,
        test_hand_eye_identity_poses, test_hand_eye_known_transform,
        test_hand_eye_known_transform_with_noise,
        test_fit_tube_axis_straight_line, test_fit_tube_axis_with_noise,
        test_fit_tube_axis_pitch_estimate, test_fit_tube_axis_requires_2_points,
        test_predict_attachment_point_zero_error,
        test_predict_attachment_point_known_error,
        test_stale_calibration_warning_missing,
        test_stale_calibration_warning_old,
        test_fresh_calibration_no_warning,
        test_stale_calibration_slog_called

Hardware-only ACs:
- AC#1: needs owner-supervised hardware session:
    uv run python scripts/calibrate_arm_base.py --live --device arm_left
- AC#2: needs owner-supervised hardware session:
    uv run python scripts/hand_eye.py --live --arm left
- AC#3: needs owner-supervised hardware session:
    uv run python scripts/tube_axis.py --live
- AC#5: needs owner-supervised hardware session:
    uv run python scripts/calibrate_arm_base.py --live --device arm_left --verify
- AC#6: needs owner-supervised hardware session (measure coarse-approach error).
"""

from __future__ import annotations

import math
import warnings
from datetime import datetime, timedelta, timezone
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from inchworm.calibration import (
    CalibrationResult,
    CalibrationStalenessWarning,
    TubeAxisResult,
    calibrate_hand_eye,
    calibrate_rigid,
    check_calibration_age,
    fit_tube_axis,
    hand_eye_park_martin,
    kabsch_transform,
    predict_attachment_point,
    _rot_to_axisangle,
    _axisangle_to_rot,
)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_rigid(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4×4 HTM from a 3×3 rotation and 3-vector translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rotz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _roty(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rotx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# AC#7: Kabsch solver on synthetic data
# ─────────────────────────────────────────────────────────────────────────────


class TestKabschTransform:
    def test_kabsch_identity(self) -> None:
        """Identity transform: src == dst → T = I, residual = 0."""
        pts = np.array([[0.1, 0.0, 0.0],
                        [0.0, 0.1, 0.0],
                        [0.0, 0.0, 0.1],
                        [0.1, 0.1, 0.1]], dtype=np.float64)
        T, rms = kabsch_transform(pts, pts)
        assert rms < 1e-10, f"Identity: rms={rms}"
        assert np.allclose(T, np.eye(4), atol=1e-10), "Identity transform not recovered"

    def test_kabsch_known_transform(self) -> None:
        """Known R+t: recovered transform should match to < 0.01 mm."""
        R_true = _rotz(0.5) @ _roty(0.3)
        t_true = np.array([0.12, -0.05, 0.08])
        T_true = _make_rigid(R_true, t_true)

        np.random.seed(42)
        pts_src = np.random.randn(10, 3) * 0.15

        pts_h = np.hstack([pts_src, np.ones((10, 1))])
        pts_dst = (T_true @ pts_h.T).T[:, :3]

        T_est, rms = kabsch_transform(pts_src, pts_dst)
        assert rms < 1e-9, f"Exact data: rms={rms}"

        # Check recovered transform matches true
        assert np.allclose(T_est[:3, :3], R_true, atol=1e-9), "Rotation not recovered"
        assert np.allclose(T_est[:3, 3], t_true, atol=1e-9), "Translation not recovered"

    def test_kabsch_known_transform_with_noise(self) -> None:
        """Noisy data: recovered transform error < noise level (0.5 mm noise → < 1 mm error)."""
        R_true = _rotx(0.2) @ _rotz(1.1)
        t_true = np.array([0.20, 0.05, -0.10])
        T_true = _make_rigid(R_true, t_true)

        np.random.seed(0)
        n = 20
        pts_src = np.random.randn(n, 3) * 0.2

        pts_h = np.hstack([pts_src, np.ones((n, 1))])
        pts_dst_exact = (T_true @ pts_h.T).T[:, :3]

        noise_sigma = 0.0005  # 0.5 mm
        pts_dst_noisy = pts_dst_exact + np.random.randn(n, 3) * noise_sigma

        T_est, rms = kabsch_transform(pts_src, pts_dst_noisy)

        # Recovered vs true: apply T_est to source and compare to exact dst
        pts_pred_h = np.hstack([pts_src, np.ones((n, 1))])
        pts_pred = (T_est @ pts_pred_h.T).T[:, :3]
        err_vs_exact = np.linalg.norm(pts_pred - pts_dst_exact, axis=1)
        rms_vs_true = float(np.sqrt(np.mean(err_vs_exact ** 2)))

        # Error vs true should be within ~2× noise
        assert rms_vs_true < noise_sigma * 3, (
            f"Noisy Kabsch: rms_vs_true={rms_vs_true*1000:.2f} mm "
            f"vs noise {noise_sigma*1000:.2f} mm"
        )
        assert rms < noise_sigma * 5, f"Residual too large: {rms}"

    def test_kabsch_rotation_only(self) -> None:
        """Pure rotation (t=0): recovered R should match, t should be near zero."""
        R_true = _roty(math.pi / 4)
        pts_src = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1], [0.05, 0.05, 0.05]])
        pts_dst = (R_true @ pts_src.T).T
        T_est, rms = kabsch_transform(pts_src, pts_dst)
        assert rms < 1e-10
        assert np.allclose(T_est[:3, :3], R_true, atol=1e-9)
        assert np.allclose(T_est[:3, 3], 0.0, atol=1e-9)

    def test_kabsch_translation_only(self) -> None:
        """Pure translation (R=I): recovered t should match, R should be I."""
        t_true = np.array([0.05, -0.03, 0.08])
        pts_src = np.random.randn(8, 3) * 0.1
        pts_dst = pts_src + t_true
        T_est, rms = kabsch_transform(pts_src, pts_dst)
        assert rms < 1e-10
        assert np.allclose(T_est[:3, :3], np.eye(3), atol=1e-9)
        assert np.allclose(T_est[:3, 3], t_true, atol=1e-9)

    def test_kabsch_requires_3_points(self) -> None:
        """Fewer than 3 points raises ValueError."""
        pts = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float64)
        with pytest.raises(ValueError, match="3"):
            kabsch_transform(pts, pts)

    def test_kabsch_shape_mismatch_raises(self) -> None:
        pts_a = np.zeros((4, 3))
        pts_b = np.zeros((5, 3))
        with pytest.raises(ValueError):
            kabsch_transform(pts_a, pts_b)

    def test_calibrate_rigid_wrapper(self) -> None:
        """calibrate_rigid returns CalibrationResult with method='kabsch'."""
        pts_src = np.random.randn(6, 3) * 0.1
        pts_dst = pts_src + np.array([0.1, 0.0, 0.0])
        result = calibrate_rigid(pts_src, pts_dst, notes="test")
        assert isinstance(result, CalibrationResult)
        assert result.method == "kabsch"
        assert result.n_samples == 6
        assert result.residual_m < 1e-9


# ─────────────────────────────────────────────────────────────────────────────
# AC#7: Hand-eye solver on synthetic data
# ─────────────────────────────────────────────────────────────────────────────


def _make_hand_eye_poses(
    X_true: np.ndarray,
    n: int = 8,
    noise_sigma: float = 0.0,
) -> tuple:
    """Synthesise (R_g2b, t_g2b, R_t2c, t_t2c) satisfying AX = XB for X = X_true."""
    np.random.seed(123)
    R_g2b_list = []
    t_g2b_list = []
    R_t2c_list = []
    t_t2c_list = []

    # Random "world" target transform T_base_target (fixed)
    R_base_target = _rotz(0.7) @ _roty(0.3)
    t_base_target = np.array([0.3, 0.1, 0.15])
    T_base_target = _make_rigid(R_base_target, t_base_target)

    for _ in range(n):
        # Random gripper pose in base frame
        ang_x = np.random.uniform(-0.5, 0.5)
        ang_z = np.random.uniform(-1.0, 1.0)
        R_g = _rotx(ang_x) @ _rotz(ang_z)
        t_g = np.random.uniform(-0.15, 0.15, 3)
        T_g2b = _make_rigid(R_g, t_g)

        # T_target_in_cam = X^{-1} @ T_g2b^{-1} @ T_base_target
        # Equivalently: T_cam = T_ee^{-1} and target in cam = (T_cam @ T_base_target)
        # AX = XB: T_g2b @ X = X @ T_cam2cam_motion
        # For simulation: cam = ee_frame @ X^{-1}:
        # T_target2cam = inv(T_g2b @ X) @ T_base_target^{-1} ...
        # Simpler: T_target2cam = inv(X) @ inv(T_g2b) @ T_base_target
        X_inv = np.linalg.inv(X_true)
        T_g2b_inv = np.linalg.inv(T_g2b)
        T_t2c = X_inv @ T_g2b_inv @ T_base_target

        if noise_sigma > 0:
            t_noise = np.random.randn(3) * noise_sigma
            T_t2c[:3, 3] += t_noise
            # Small rotation noise
            dR = _rotx(np.random.randn() * noise_sigma * 5)
            T_t2c[:3, :3] = dR @ T_t2c[:3, :3]

        R_g2b_list.append(R_g)
        t_g2b_list.append(t_g)
        R_t2c_list.append(T_t2c[:3, :3])
        t_t2c_list.append(T_t2c[:3, 3])

    return R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list


class TestHandEye:
    def test_hand_eye_identity_poses(self) -> None:
        """Identity X (X_true = I): solver should recover near-identity."""
        X_true = np.eye(4, dtype=np.float64)
        R_g2b, t_g2b, R_t2c, t_t2c = _make_hand_eye_poses(X_true, n=8)
        X_est, rms_rot, rms_trans = hand_eye_park_martin(R_g2b, t_g2b, R_t2c, t_t2c)
        # Identity: rotation should be I, translation near 0
        assert np.allclose(X_est[:3, :3], np.eye(3), atol=1e-4), (
            f"Identity hand-eye: R not I: {X_est[:3,:3]}"
        )
        assert np.linalg.norm(X_est[:3, 3]) < 1e-3, (
            f"Identity hand-eye: t not 0: {X_est[:3,3]}"
        )

    def test_hand_eye_known_transform(self) -> None:
        """Known X_true: recovered transform should match closely (exact data)."""
        R_x = _rotx(0.3) @ _rotz(0.8)
        t_x = np.array([0.05, -0.02, 0.07])
        X_true = _make_rigid(R_x, t_x)

        R_g2b, t_g2b, R_t2c, t_t2c = _make_hand_eye_poses(X_true, n=8)
        X_est, rms_rot, rms_trans = hand_eye_park_martin(R_g2b, t_g2b, R_t2c, t_t2c)

        # Check recovered rotation
        R_err = X_est[:3, :3].T @ R_x
        angle_err = float(np.linalg.norm(_rot_to_axisangle(R_err)))
        assert angle_err < 0.01, f"Rotation error: {math.degrees(angle_err):.3f} deg"

        # Check recovered translation
        t_err = np.linalg.norm(X_est[:3, 3] - t_x)
        assert t_err < 0.005, f"Translation error: {t_err*1000:.2f} mm"

    def test_hand_eye_known_transform_with_noise(self) -> None:
        """Noisy data: recovered transform error should be bounded."""
        R_x = _roty(0.5) @ _rotx(-0.2)
        t_x = np.array([0.03, 0.01, 0.08])
        X_true = _make_rigid(R_x, t_x)

        np.random.seed(99)
        R_g2b, t_g2b, R_t2c, t_t2c = _make_hand_eye_poses(X_true, n=12, noise_sigma=0.002)
        X_est, rms_rot, rms_trans = hand_eye_park_martin(R_g2b, t_g2b, R_t2c, t_t2c)

        R_err = X_est[:3, :3].T @ R_x
        angle_err = float(np.linalg.norm(_rot_to_axisangle(R_err)))
        t_err = np.linalg.norm(X_est[:3, 3] - t_x)

        # With 2 mm noise, expect < 10 mm / 0.05 rad error
        assert angle_err < 0.05, f"Rotation error with noise: {math.degrees(angle_err):.3f} deg"
        assert t_err < 0.010, f"Translation error with noise: {t_err*1000:.2f} mm"

    def test_hand_eye_requires_3_poses(self) -> None:
        """Fewer than 3 poses raises ValueError."""
        R = [np.eye(3)] * 2
        t = [np.zeros(3)] * 2
        with pytest.raises(ValueError, match="3"):
            hand_eye_park_martin(R, t, R, t)

    def test_calibrate_hand_eye_wrapper(self) -> None:
        """calibrate_hand_eye returns CalibrationResult with hand_eye method."""
        R_x = _rotz(0.4)
        t_x = np.array([0.02, -0.01, 0.05])
        X_true = _make_rigid(R_x, t_x)
        R_g2b, t_g2b, R_t2c, t_t2c = _make_hand_eye_poses(X_true, n=6)
        result = calibrate_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c, notes="test")
        assert isinstance(result, CalibrationResult)
        assert "hand_eye" in result.method
        assert result.n_samples == 6

    def test_axisangle_roundtrip(self) -> None:
        """Axis-angle round-trip: R → v → R' should recover R."""
        R_orig = _rotx(0.7) @ _rotz(1.2)
        v = _rot_to_axisangle(R_orig)
        R_rec = _axisangle_to_rot(v)
        assert np.allclose(R_rec, R_orig, atol=1e-9), "Axis-angle round-trip failed"


# ─────────────────────────────────────────────────────────────────────────────
# AC#7: Tube-axis line fit on synthetic data
# ─────────────────────────────────────────────────────────────────────────────


class TestFitTubeAxis:
    def _make_tube_points(
        self,
        n: int = 6,
        pitch_m: float = 0.183676,
        noise_sigma: float = 0.0,
        axis: np.ndarray | None = None,
        origin: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Generate synthetic attachment-point centres on a known axis."""
        if axis is None:
            axis = np.array([1.0, 0.0, 0.0])
        axis = axis / np.linalg.norm(axis)
        if origin is None:
            origin = np.zeros(3)
        indices = np.arange(n, dtype=np.int64)
        pts = origin[None, :] + indices[:, None] * pitch_m * axis[None, :]
        if noise_sigma > 0:
            np.random.seed(7)
            pts += np.random.randn(*pts.shape) * noise_sigma
        return pts, indices

    def test_fit_tube_axis_straight_line(self) -> None:
        """Exact collinear points: residual = 0, axis recovered exactly."""
        pts, idx = self._make_tube_points(n=5, pitch_m=0.183676)
        result = fit_tube_axis(pts, idx)
        assert result.residual_m < 1e-10, f"Residual not zero: {result.residual_m}"
        # Axis should align with X
        cos_angle = abs(float(np.dot(result.axis_direction, [1, 0, 0])))
        assert cos_angle > 0.9999, f"Axis not aligned: {result.axis_direction}"

    def test_fit_tube_axis_with_noise(self) -> None:
        """Noisy points: residual ≈ noise level."""
        noise_sigma = 0.001  # 1 mm
        pts, idx = self._make_tube_points(n=8, noise_sigma=noise_sigma)
        result = fit_tube_axis(pts, idx)
        # Residual should be within ~2× noise
        assert result.residual_m < noise_sigma * 3, (
            f"Residual {result.residual_m*1000:.2f} mm > 3× noise {noise_sigma*1000:.2f} mm"
        )

    def test_fit_tube_axis_pitch_estimate(self) -> None:
        """Pitch estimate should match true pitch within noise."""
        true_pitch = 0.183676
        pts, idx = self._make_tube_points(n=6, pitch_m=true_pitch)
        result = fit_tube_axis(pts, idx)
        err_mm = abs(result.pitch_estimate_m - true_pitch) * 1000.0
        assert err_mm < 0.01, f"Pitch error: {err_mm:.4f} mm"

    def test_fit_tube_axis_non_axis_aligned(self) -> None:
        """Arbitrary axis direction is recovered to < 0.01 rad."""
        axis_true = np.array([0.6, 0.8, 0.0])
        axis_true = axis_true / np.linalg.norm(axis_true)
        origin = np.array([0.1, 0.2, 0.05])
        pts, idx = self._make_tube_points(n=7, axis=axis_true, origin=origin)
        result = fit_tube_axis(pts, idx)
        cos_angle = abs(float(np.dot(result.axis_direction, axis_true)))
        angle_err = math.acos(min(cos_angle, 1.0))
        assert angle_err < 0.01, f"Axis angle error: {math.degrees(angle_err):.3f} deg"

    def test_fit_tube_axis_lattice_origin(self) -> None:
        """Lattice origin (index 0) should be recovered within tolerance."""
        origin_true = np.array([0.05, 0.10, 0.02])
        pts, idx = self._make_tube_points(n=5, origin=origin_true)
        result = fit_tube_axis(pts, idx)
        err_m = np.linalg.norm(result.lattice_origin_in_chassis - origin_true)
        assert err_m < 1e-10, f"Origin error: {err_m*1000:.4f} mm"

    def test_fit_tube_axis_requires_2_points(self) -> None:
        """Fewer than 2 points raises ValueError."""
        pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        idx = np.array([0], dtype=np.int64)
        with pytest.raises(ValueError, match="2"):
            fit_tube_axis(pts, idx)

    def test_fit_tube_axis_dict_round_trip(self) -> None:
        """TubeAxisResult.to_dict / from_dict round-trip."""
        pts, idx = self._make_tube_points(n=5)
        result = fit_tube_axis(pts, idx)
        d = result.to_dict()
        result2 = TubeAxisResult.from_dict(d)
        assert np.allclose(result2.axis_direction, result.axis_direction, atol=1e-12)
        assert abs(result2.pitch_estimate_m - result.pitch_estimate_m) < 1e-12
        assert result2.method == result.method


# ─────────────────────────────────────────────────────────────────────────────
# AC#4 / AC#7: Result round-trips and stale-calibration warnings
# ─────────────────────────────────────────────────────────────────────────────


class TestCalibrationResult:
    def test_result_dict_round_trip(self) -> None:
        """CalibrationResult.to_dict / from_dict preserves all fields."""
        T = np.eye(4, dtype=np.float64)
        T[0, 3] = 0.1
        result = CalibrationResult(
            transform=T,
            method="kabsch",
            residual_m=0.001,
            residual_rad=0.002,
            ts_iso="2026-10-01T12:00:00+00:00",
            n_samples=6,
            notes="test notes",
        )
        d = result.to_dict()
        result2 = CalibrationResult.from_dict(d)
        assert np.allclose(result2.transform, result.transform)
        assert result2.method == result.method
        assert abs(result2.residual_m - result.residual_m) < 1e-12
        assert result2.n_samples == result.n_samples
        assert result2.notes == result.notes


class TestCheckCalibrationAge:
    def test_stale_calibration_warning_missing(self) -> None:
        """None result → CalibrationStalenessWarning with 'missing' in message."""
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            check_calibration_age(None, label="left_arm_base", max_age_days=90)
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 1
        assert "missing" in str(stale_warnings[0].message).lower()

    def test_stale_calibration_warning_old(self) -> None:
        """Old calibration (> max_age_days) → CalibrationStalenessWarning."""
        old_ts = (datetime.now(timezone.utc) - timedelta(days=120)).isoformat()
        result = CalibrationResult(
            transform=np.eye(4),
            method="kabsch",
            residual_m=0.001,
            ts_iso=old_ts,
        )
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            check_calibration_age(result, label="left_arm_base", max_age_days=90)
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 1
        assert "stale" in str(stale_warnings[0].message).lower()

    def test_fresh_calibration_no_warning(self) -> None:
        """Fresh calibration (< max_age_days) → no CalibrationStalenessWarning."""
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=5)).isoformat()
        result = CalibrationResult(
            transform=np.eye(4),
            method="kabsch",
            residual_m=0.001,
            ts_iso=fresh_ts,
        )
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            check_calibration_age(result, label="left_arm_base", max_age_days=90)
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 0, f"Unexpected warning: {[str(x.message) for x in stale_warnings]}"

    def test_stale_calibration_slog_called(self) -> None:
        """Missing calibration: AA-7 session logger 'emit' is called."""
        mock_slog = MagicMock()
        with warnings.catch_warnings(record=True):
            warnings.simplefilter("always")
            check_calibration_age(None, label="dexarm_base", session_logger=mock_slog)
        mock_slog.emit.assert_called_once()
        call_kwargs = mock_slog.emit.call_args[1]
        assert call_kwargs.get("event_type") == "calibration_missing"
        assert call_kwargs.get("severity") == "WARNING"

    def test_age_check_disabled(self) -> None:
        """max_age_days=0 disables age check → no warning for old calibration."""
        old_ts = (datetime.now(timezone.utc) - timedelta(days=9999)).isoformat()
        result = CalibrationResult(
            transform=np.eye(4),
            method="kabsch",
            residual_m=0.0,
            ts_iso=old_ts,
        )
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            check_calibration_age(result, label="tube_axis", max_age_days=0)
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 0

    def test_bad_timestamp_warns(self) -> None:
        """Unparseable ts_iso triggers a CalibrationStalenessWarning."""
        result = CalibrationResult(
            transform=np.eye(4),
            method="kabsch",
            residual_m=0.0,
            ts_iso="not-a-timestamp",
        )
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            check_calibration_age(result, label="right_arm_base", max_age_days=90)
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 1


# ─────────────────────────────────────────────────────────────────────────────
# AC#4 / AC#5: predict_attachment_point verification helper
# ─────────────────────────────────────────────────────────────────────────────


class TestPredictAttachmentPoint:
    def test_predict_attachment_point_zero_error(self) -> None:
        """Perfect transform: predicted == measured → error = 0."""
        T = _make_rigid(_rotz(0.5), np.array([0.1, 0.2, 0.3]))
        tcp_base = np.array([0.05, 0.0, 0.08])
        # Compute expected chassis position
        tcp_h = np.array([*tcp_base, 1.0])
        expected_chassis = (T @ tcp_h)[:3]

        pred = predict_attachment_point(T, tcp_base, expected_chassis)
        assert pred.error_norm_m < 1e-10, f"Zero error case: {pred.error_norm_m}"
        assert np.allclose(pred.error_m, 0.0, atol=1e-10)

    def test_predict_attachment_point_known_error(self) -> None:
        """Known 5 mm error in X: per-axis error should reflect it."""
        T = np.eye(4, dtype=np.float64)  # Identity: base == chassis
        tcp_base = np.array([0.10, 0.05, 0.08])
        measured = np.array([0.105, 0.05, 0.08])  # 5 mm offset in X

        pred = predict_attachment_point(T, tcp_base, measured)
        # predicted = tcp_base (T is identity), measured has 5 mm extra in X
        # error = predicted - measured = -0.005 in X
        assert abs(pred.error_m[0] + 0.005) < 1e-10
        assert abs(pred.error_m[1]) < 1e-10
        assert abs(pred.error_m[2]) < 1e-10
        assert abs(pred.error_norm_m - 0.005) < 1e-10


# ─────────────────────────────────────────────────────────────────────────────
# Config integration: AA-16 block loads without error
# ─────────────────────────────────────────────────────────────────────────────


class TestConfigIntegration:
    def test_hardware_config_loads_aa16_block(self) -> None:
        """hardware.yaml with null calibration block loads cleanly."""
        from inchworm.config import load_config
        cfg = load_config()
        # calibration block is present but all null
        assert cfg.calibration is not None
        assert cfg.calibration.left_arm_base is None
        assert cfg.calibration.right_arm_base is None
        assert cfg.calibration.dexarm_base is None
        assert cfg.calibration.left_wrist_camera is None
        assert cfg.calibration.right_wrist_camera is None
        assert cfg.calibration.tube_axis is None
        assert cfg.calibration.max_calibration_age_days == 90

    def test_check_calibration_age_from_config(self) -> None:
        """check_calibration_age with None from config → warning."""
        from inchworm.config import load_config
        cfg = load_config()
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            if cfg.calibration:
                check_calibration_age(
                    cfg.calibration.left_arm_base,
                    label="left_arm_base",
                    max_age_days=cfg.calibration.max_calibration_age_days,
                )
        stale_warnings = [x for x in w if issubclass(x.category, CalibrationStalenessWarning)]
        assert len(stale_warnings) == 1  # null → missing warning
