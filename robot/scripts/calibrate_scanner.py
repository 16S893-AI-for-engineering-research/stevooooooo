#!/usr/bin/env python3
"""Scanner camera and laser-plane calibration script — AA-30.

This script is a **DRY-RUN by default**: it exercises all calibration stages
using synthetic data and the mock backend.  Pass ``--live`` to perform a real
calibration session with the Arducam scanner camera and laser.

Usage:
------
.. code-block:: bash

    # Dry-run (safe, no hardware, uses synthetic data):
    uv run python scripts/calibrate_scanner.py

    # Live calibration (owner supervision required):
    uv run python scripts/calibrate_scanner.py --live

    # Individual steps (live):
    uv run python scripts/calibrate_scanner.py --live --step intrinsics
    uv run python scripts/calibrate_scanner.py --live --step laser_plane
    uv run python scripts/calibrate_scanner.py --live --step hand_eye
    uv run python scripts/calibrate_scanner.py --live --step validate_artifact

Steps
-----
1. **intrinsics**: Capture chessboard images, calibrate camera intrinsics and
   distortion.  Laser is OFF during this step.

2. **laser_plane**: Capture laser-line images at multiple standoffs with the
   chessboard target present (to measure target pose).  Laser is switched ON
   via :class:`~inchworm.laser.LaserDriver` only during exposure.

3. **hand_eye**: Move the DexArm to multiple commanded positions while
   observing the chessboard with the scanner camera.  Laser is OFF.

4. **validate_artifact**: Image a known-geometry artifact (e.g. a gauge block)
   to validate the calibrated sensitivity (AC#6, hardware-only).

Output
------
Calibration results are written to ``robot/config/scanner_calibration.yaml``
in the dict format of :func:`~inchworm.scanner_calibration.save_calibration`.

Safety
------
* Laser is switched via :class:`~inchworm.laser.LaserDriver` only.
* Every motion goes through the AA-12 ``DexArmDriver`` with workspace bounds.
* ``--live`` is required to open any real serial port or camera.
* Dry-run produces log output and a synthetic calibration file; no hardware
  is touched.

AC Coverage:
------------
ACs #1–#4 and #6 require real hardware captures and remain unchecked until
an owner-supervised session runs this script with ``--live``.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Configure logging early so all module-level log calls are captured.
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("calibrate_scanner")

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument(
    "--live",
    action="store_true",
    default=False,
    help=(
        "Open real hardware (camera + laser + DexArm). "
        "OWNER SUPERVISION REQUIRED."
    ),
)
parser.add_argument(
    "--step",
    choices=["all", "intrinsics", "laser_plane", "hand_eye", "validate_artifact"],
    default="all",
    help="Which calibration step to run (default: all).",
)
parser.add_argument(
    "--n-frames",
    type=int,
    default=15,
    metavar="N",
    help="Number of calibration frames to capture per step (default: 15).",
)
parser.add_argument(
    "--standoffs",
    type=float,
    nargs="+",
    default=[65.0, 70.0, 75.0, 80.0, 85.0, 90.0],
    metavar="MM",
    help="Standoff distances (mm) for laser-plane calibration (default: 65..90 mm).",
)
parser.add_argument(
    "--output",
    type=Path,
    default=Path(__file__).resolve().parents[1] / "config" / "scanner_calibration.yaml",
    metavar="PATH",
    help="Output YAML file for calibration results.",
)


# ---------------------------------------------------------------------------
# Helper: safe LaserDriver context (live or mock)
# ---------------------------------------------------------------------------


def _laser_on_off(laser_driver, duration_s: float, *, live: bool) -> None:
    """Switch the laser on for ``duration_s`` then off.

    In dry-run mode, only logs the action without touching hardware.
    """
    if not live:
        log.info("  [DRY-RUN] laser ON for %.2f s", duration_s)
        return
    laser_driver.on(duration_s=duration_s)


# ---------------------------------------------------------------------------
# Step 1: Intrinsics
# ---------------------------------------------------------------------------


def run_intrinsics(
    config, backend, *,
    live: bool,
    n_frames: int = 15,
) -> "IntrinsicsResult":
    """Calibrate camera intrinsics and distortion.

    In live mode:
    1. Opens the scanner camera via CameraArbiter.
    2. Captures ``n_frames`` images of the chessboard at various angles.
    3. Runs :func:`~inchworm.scanner_calibration.calibrate_camera_intrinsics`.

    In dry-run mode:
    1. Renders synthetic chessboard frames through a placeholder camera matrix.
    2. Calibrates from the synthetic frames.

    The laser is OFF during this step.

    Returns
    -------
    IntrinsicsResult
    """
    from inchworm.scanner_calibration import (
        calibrate_camera_intrinsics,
        render_synthetic_chessboard,
    )
    import numpy as np

    if not live:
        log.info("[DRY-RUN] Step 1: intrinsics — using synthetic chessboard frames.")
        # Placeholder camera matrix (design defaults).
        true_K = np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 400.0], [0.0, 0.0, 1.0]])
        true_dist = np.zeros(4, dtype=np.float32)
        rng = __import__("numpy").random.default_rng(0)
        frames, noisy_corners, _ = render_synthetic_chessboard(
            true_K, true_dist, (1280, 800),
            n_frames=n_frames,
            rng=rng,
        )
        # Calibrate using synthetic projected corners directly (bypass findChessboardCorners).
        import cv2  # type: ignore[import]
        from inchworm.scanner_calibration import CHESSBOARD_PATTERN, CHESSBOARD_SQUARE_MM
        cols, rows = CHESSBOARD_PATTERN
        n_corners = cols * rows
        obj_template = np.zeros((n_corners, 3), dtype=np.float32)
        obj_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * CHESSBOARD_SQUARE_MM
        obj_pts = []
        img_pts = []
        image_size = (1280, 800)
        for nc in noisy_corners:
            nc_sq = nc.reshape(-1, 2)
            mask = (
                (nc_sq[:, 0] >= 1) & (nc_sq[:, 0] < image_size[0] - 1) &
                (nc_sq[:, 1] >= 1) & (nc_sq[:, 1] < image_size[1] - 1)
            )
            if mask.sum() < 6:
                continue
            obj_pts.append(obj_template[mask])
            img_pts.append(nc[mask].reshape(-1, 1, 2))
        if len(obj_pts) < 4:
            log.warning("[DRY-RUN] Fewer than 4 valid corner sets; using even fewer.")
        rms, cam_mat, dist_c, rvecs, tvecs = cv2.calibrateCamera(
            obj_pts or [obj_template.reshape(-1, 1, 3)],
            img_pts or [noisy_corners[0] if noisy_corners else obj_template[:, :2].reshape(-1, 1, 2)],
            image_size,
            None,
            None,
        )
        from inchworm.scanner_calibration import IntrinsicsResult
        result = IntrinsicsResult(
            camera_matrix=cam_mat.astype(np.float64),
            dist_coeffs=dist_c.flatten().astype(np.float64),
            rms_reprojection_error_px=float(rms),
            per_image_reprojection_error_px=[],
            n_images_used=len(obj_pts),
            target_description="synthetic chessboard 9x6 25mm (dry-run)",
            unverified=True,
        )
        log.info(
            "[DRY-RUN] Intrinsics done: rms=%.4f px  fx=%.1f fy=%.1f",
            result.rms_reprojection_error_px,
            result.camera_matrix[0, 0],
            result.camera_matrix[1, 1],
        )
        return result

    # ------------------------------------------------------------------
    # LIVE path (hardware required)
    # ------------------------------------------------------------------
    log.info("LIVE Step 1: Capturing %d chessboard frames for intrinsics...", n_frames)
    log.info(
        "  Target: chessboard 9×6 inner corners, 25 mm squares  "
        "(print assets/chessboard_9x6_25mm.pdf on flat stock)."
    )
    log.info("  Present the board at various angles, ~200–400 mm from camera.")

    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraCapture, CameraSettings

    arbiter = CameraArbiter(config)
    frames = []
    with arbiter.acquire("camera_scanner") as transport:
        capture = CameraCapture(transport)
        settings = CameraSettings(
            width=1280, height=800, fps=30.0,
            auto_exposure=False, exposure=5000, gain=1,
        )
        capture.apply_settings(settings)
        for i in range(n_frames):
            input(f"  Frame {i+1}/{n_frames}: position the board, then press Enter.")
            frame = capture.read_gray()
            frames.append(frame)
            log.info("  Captured frame %d/%d.", i + 1, n_frames)

    result = calibrate_camera_intrinsics(frames)
    log.info(
        "Intrinsics: rms=%.4f px  n_images=%d  fx=%.1f  fy=%.1f",
        result.rms_reprojection_error_px,
        result.n_images_used,
        result.camera_matrix[0, 0],
        result.camera_matrix[1, 1],
    )
    return result


# ---------------------------------------------------------------------------
# Step 2: Laser-plane fit
# ---------------------------------------------------------------------------


def run_laser_plane(
    config, backend, intrinsics, *,
    live: bool,
    standoffs,
) -> "LaserPlaneResult":
    """Calibrate the laser plane from observations at multiple standoffs.

    In live mode:
    1. For each standoff distance, position the chessboard target.
    2. Estimate the target pose from the chessboard corners (using calibrated
       intrinsics from step 1).
    3. Switch laser ON via LaserDriver for one frame, capture the laser-line image.
    4. Extract the laser-line pixel positions with
       :func:`~inchworm.laser_line.extract_laser_line`.
    5. Intersect the laser-line rays with the estimated target plane to get 3-D points
       in the camera frame.
    6. Fit a plane through all pooled 3-D points with SVD.

    In dry-run mode:
    Generates synthetic 3-D points on the design-geometry plane and fits.

    Laser safety
    ------------
    The laser is switched **exclusively** via :class:`~inchworm.laser.LaserDriver`.
    It is switched off after each single-frame capture.

    Returns
    -------
    LaserPlaneResult
    """
    import math
    import numpy as np
    from inchworm.constants import LASER_TO_CAMERA_ANGLE_DEG, LASER_STANDOFF_MM
    from inchworm.scanner_calibration import (
        fit_laser_plane,
        render_synthetic_laser_points,
    )

    if not live:
        log.info(
            "[DRY-RUN] Step 2: laser_plane — using synthetic points on design plane."
        )
        ang_rad = math.radians(float(LASER_TO_CAMERA_ANGLE_DEG))
        normal = np.array([-math.sin(ang_rad), 0.0, math.cos(ang_rad)])
        normal /= np.linalg.norm(normal)
        K = intrinsics.camera_matrix
        rng = np.random.default_rng(99)
        clouds = []
        for standoff in standoffs:
            d = -float(np.dot(normal, [0.0, 0.0, standoff]))
            pts = render_synthetic_laser_points(normal, d, K, n_points=100, noise_mm=0.1, rng=rng)
            clouds.append(pts)
        result = fit_laser_plane(clouds)
        log.info(
            "[DRY-RUN] Laser plane: rms_residual=%.4f mm  n_points=%d",
            result.rms_residual_mm,
            result.n_points,
        )
        return result

    # ------------------------------------------------------------------
    # LIVE path
    # ------------------------------------------------------------------
    log.info("LIVE Step 2: Laser-plane calibration at %d standoff(s).", len(standoffs))
    log.info(
        "  Procedure: for each standoff, position the flat chessboard target "
        "at the measured distance, press Enter, then the laser fires for one frame."
    )
    log.warning("  WARNING: The laser will be switched ON briefly for each capture.")

    from inchworm.laser import open_laser
    from inchworm.laser_line import extract_laser_line
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraCapture, CameraSettings
    import cv2  # type: ignore[import]

    from inchworm.scanner_calibration import (
        CHESSBOARD_PATTERN,
        CHESSBOARD_SQUARE_MM,
    )

    cols, rows = CHESSBOARD_PATTERN
    square_mm = CHESSBOARD_SQUARE_MM
    obj_template = np.zeros((cols * rows, 3), dtype=np.float32)
    obj_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    clouds = []
    arbiter = CameraArbiter(config)

    with open_laser(config, backend) as laser:
        with arbiter.acquire("camera_scanner") as transport:
            capture = CameraCapture(transport)
            settings = CameraSettings(
                width=1280, height=800, fps=30.0,
                auto_exposure=False, exposure=1000, gain=4,
            )
            capture.apply_settings(settings)

            for standoff_mm in standoffs:
                input(
                    f"\n  Position chessboard flat at {standoff_mm:.0f} mm from camera, "
                    "then press Enter."
                )
                # First capture background (laser OFF).
                bg_frame = capture.read_gray()

                # Estimate board pose (target plane) from background frame.
                ret, corners = cv2.findChessboardCorners(bg_frame, (cols, rows), None)
                if not ret:
                    log.warning("  Chessboard not detected at standoff %.0f mm; skipping.", standoff_mm)
                    continue

                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_sub = cv2.cornerSubPix(bg_frame, corners, (11, 11), (-1, -1), criteria)
                cam_mat = intrinsics.camera_matrix.astype(np.float32)
                dist_c = intrinsics.dist_coeffs.astype(np.float32)
                ret_pnp, rvec, tvec = cv2.solvePnP(
                    obj_template, corners_sub, cam_mat, dist_c
                )
                if not ret_pnp:
                    log.warning("  solvePnP failed at standoff %.0f mm; skipping.", standoff_mm)
                    continue

                # Compute target plane in camera frame.
                R_target, _ = cv2.Rodrigues(rvec)
                # Target z-axis (normal) in camera frame.
                target_normal = R_target[:, 2]
                target_origin = tvec.flatten()

                # Capture laser-on frame.
                laser.on(duration_s=0.5)
                laser_frame = capture.read_gray()
                laser.off()

                # Extract laser line.
                line_result = extract_laser_line(laser_frame - bg_frame.clip(min=0))

                # Intersect laser-line rays with the target plane to get 3-D points.
                from inchworm.laser_line import _undistort_point, _ray_plane_intersect, LaserCalibration
                # Use only intrinsics (no full calibration needed for this step).
                dummy_calib = LaserCalibration(
                    fx=float(cam_mat[0, 0]),
                    fy=float(cam_mat[1, 1]),
                    cx=float(cam_mat[0, 2]),
                    cy=float(cam_mat[1, 2]),
                    distortion=dist_c.astype(np.float64),
                )
                d_plane = -float(np.dot(target_normal, target_origin))
                pts_3d_list = []
                for col_idx, row_pos in enumerate(line_result.row_positions):
                    if not np.isfinite(row_pos):
                        continue
                    xn, yn = _undistort_point(float(col_idx), float(row_pos), dummy_calib)
                    ray = np.array([xn, yn, 1.0])
                    ray /= np.linalg.norm(ray)
                    p = _ray_plane_intersect(ray, target_normal, d_plane)
                    if p is not None:
                        pts_3d_list.append(p)
                if len(pts_3d_list) > 10:
                    clouds.append(np.array(pts_3d_list, dtype=np.float64))
                    log.info("  Standoff %.0f mm: %d 3-D points.", standoff_mm, len(pts_3d_list))
                else:
                    log.warning("  Standoff %.0f mm: only %d points; skipping.", standoff_mm, len(pts_3d_list))

    if not clouds:
        log.error("No laser-plane 3-D points collected; cannot fit plane.")
        raise RuntimeError("Laser-plane calibration failed: no valid 3-D points.")

    result = fit_laser_plane(clouds)
    log.info(
        "Laser plane: normal=[%.4f, %.4f, %.4f]  d=%.4f  rms=%.4f mm  n_points=%d",
        *result.plane_normal.tolist(),
        result.plane_d,
        result.rms_residual_mm,
        result.n_points,
    )
    return result


# ---------------------------------------------------------------------------
# Step 3: Hand-eye calibration
# ---------------------------------------------------------------------------


def run_hand_eye(
    config, backend, intrinsics, *,
    live: bool,
    n_poses: int = 10,
) -> "HandEyeResult":
    """Calibrate the camera-to-DexArm-tool transform (hand-eye).

    In live mode, moves the DexArm to ``n_poses`` commanded positions,
    captures the chessboard image at each, solves hand-eye using the
    DexArm position as the gripper pose and the chessboard pose as
    the target-to-camera pose.

    In dry-run mode, uses synthetic pose pairs.

    Returns
    -------
    HandEyeResult
    """
    import numpy as np

    if not live:
        log.info("[DRY-RUN] Step 3: hand_eye — using synthetic pose pairs.")
        from inchworm.scanner_calibration import calibrate_hand_eye

        # Synthetic: identity rotation, baseline offset.
        rng = np.random.default_rng(42)
        import math

        def _axis_angle_to_R(axis, angle):
            axis = axis / max(np.linalg.norm(axis), 1e-12)
            c, s = math.cos(angle), math.sin(angle)
            x, y, z = axis
            return np.array([
                [c + x*x*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                [y*x*(1-c) + z*s, c + y*y*(1-c),   y*z*(1-c) - x*s],
                [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)],
            ])

        T_true = np.eye(4, dtype=np.float64)
        T_true[0, 3] = 67.0
        R_cx = T_true[:3, :3]
        t_cx = T_true[:3, 3]

        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for _ in range(n_poses):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = rng.uniform(0.1, 0.5)
            R_g = _axis_angle_to_R(axis, angle)
            t_g = rng.uniform(-50.0, 50.0, size=3)
            R_tc = R_cx.T @ R_g.T
            t_tc = R_cx.T @ (-R_g.T @ t_g - t_cx)
            R_g2b.append(R_g)
            t_g2b.append(t_g)
            R_t2c.append(R_tc + rng.normal(0, 0.005, size=(3, 3)))
            t_t2c.append(t_tc + rng.normal(0, 0.5, size=3))

        result = calibrate_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c)
        log.info("[DRY-RUN] Hand-eye: method=%s n_poses=%d", result.method, result.n_poses)
        return result

    # ------------------------------------------------------------------
    # LIVE path
    # ------------------------------------------------------------------
    log.info("LIVE Step 3: Hand-eye calibration (%d DexArm poses).", n_poses)
    log.info("  The DexArm will move to pre-defined positions; chessboard must be visible.")

    import cv2  # type: ignore[import]
    from inchworm.backends import create_backend
    from inchworm.dexarm import DexArmDriver
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraCapture, CameraSettings
    from inchworm.scanner_calibration import (
        CHESSBOARD_PATTERN, CHESSBOARD_SQUARE_MM, calibrate_hand_eye
    )

    cols, rows = CHESSBOARD_PATTERN
    obj_template = np.zeros((cols * rows, 3), dtype=np.float32)
    obj_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * CHESSBOARD_SQUARE_MM
    cam_mat = intrinsics.camera_matrix.astype(np.float32)
    dist_c = intrinsics.dist_coeffs.astype(np.float32)

    # Pre-defined DexArm scan positions (mm, X Y Z in arm frame).
    # These stay well within the configured workspace.
    dexarm_positions = [
        (  0.0, 300.0, 80.0),
        ( 20.0, 300.0, 80.0),
        (-20.0, 300.0, 80.0),
        (  0.0, 320.0, 80.0),
        (  0.0, 280.0, 80.0),
        ( 10.0, 310.0, 80.0),
        (-10.0, 290.0, 80.0),
        ( 15.0, 305.0, 75.0),
        (-15.0, 295.0, 85.0),
        (  5.0, 315.0, 90.0),
    ][:n_poses]

    dexarm_transport = backend.open_serial("dexarm")
    workspace = config.dexarm.workspace
    driver = DexArmDriver(dexarm_transport, workspace=workspace)
    driver.connect()

    arbiter = CameraArbiter(config)
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []

    with arbiter.acquire("camera_scanner") as transport:
        capture = CameraCapture(transport)
        settings = CameraSettings(
            width=1280, height=800, fps=30.0,
            auto_exposure=False, exposure=5000, gain=1,
        )
        capture.apply_settings(settings)

        for i, (x, y, z) in enumerate(dexarm_positions):
            driver.move_absolute(x, y, z, feedrate=2000.0)
            pos = driver.get_position()
            log.info("  Pose %d/%d: arm at (%.1f, %.1f, %.1f) mm", i+1, n_poses, pos.x, pos.y, pos.z)

            frame = capture.read_gray()
            ret, corners = cv2.findChessboardCorners(frame, (cols, rows), None)
            if not ret:
                log.warning("  Pose %d: chessboard not detected; skipping.", i+1)
                continue

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(frame, corners, (11, 11), (-1, -1), criteria)
            ret_pnp, rvec, tvec = cv2.solvePnP(obj_template, corners_sub, cam_mat, dist_c)
            if not ret_pnp:
                log.warning("  Pose %d: solvePnP failed; skipping.", i+1)
                continue

            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.flatten()

            # DexArm gripper pose: identity rotation (planar arm), position from M114.
            R_gripper = np.eye(3, dtype=np.float64)
            t_gripper = np.array([pos.x, pos.y, pos.z], dtype=np.float64)

            R_g2b.append(R_gripper)
            t_g2b.append(t_gripper)
            R_t2c.append(R_target2cam.astype(np.float64))
            t_t2c.append(t_target2cam.astype(np.float64))

    driver.disconnect()

    if len(R_g2b) < 3:
        log.error("Fewer than 3 valid pose pairs; cannot calibrate hand-eye.")
        raise RuntimeError("Hand-eye calibration failed: insufficient valid poses.")

    result = calibrate_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c)
    log.info(
        "Hand-eye: method=%s  n_poses=%d\n  T_tool_camera=\n%s",
        result.method, result.n_poses, result.T_tool_camera
    )
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    args = parser.parse_args()

    import yaml  # type: ignore[import]
    import numpy as np
    from datetime import date

    from inchworm.config import load_config
    from inchworm.scanner_calibration import (
        ScannerCalibration,
        SensitivityResult,
        compute_sensitivity,
        save_calibration,
    )

    if args.live:
        log.warning("=" * 60)
        log.warning("LIVE MODE: Real hardware will be used.")
        log.warning("  Camera: Arducam OV9281 scanner camera")
        log.warning("  Laser: USB relay (LaserDriver only)")
        log.warning("  DexArm: motion with workspace bounds enforced")
        log.warning("  Owner supervision required.")
        log.warning("=" * 60)

        from inchworm.backends import create_backend
        config = load_config()
        backend = create_backend(config)
    else:
        log.info("DRY-RUN mode (mock backend). No hardware touched.")
        from inchworm.backends import MockBackend
        config = load_config()
        backend = MockBackend(config)

    step = args.step

    # ---- Step 1: Intrinsics ----
    intrinsics = None
    if step in ("all", "intrinsics"):
        intrinsics = run_intrinsics(config, backend, live=args.live, n_frames=args.n_frames)
    else:
        # Build placeholder intrinsics for downstream steps.
        from inchworm.scanner_calibration import IntrinsicsResult
        intrinsics = IntrinsicsResult(
            camera_matrix=np.array([[800., 0., 640.], [0., 800., 400.], [0., 0., 1.]]),
            dist_coeffs=np.zeros(4),
            rms_reprojection_error_px=0.0,
            per_image_reprojection_error_px=[],
            n_images_used=0,
            target_description="placeholder (step skipped)",
            unverified=True,
        )

    # ---- Step 2: Laser plane ----
    laser_plane = None
    if step in ("all", "laser_plane"):
        laser_plane = run_laser_plane(
            config, backend, intrinsics,
            live=args.live, standoffs=args.standoffs,
        )

    # ---- Sensitivity ----
    sensitivity = None
    if intrinsics is not None and laser_plane is not None:
        sensitivity = compute_sensitivity(intrinsics, laser_plane)
        log.info(
            "Sensitivity at 80 mm: %.4f mm/px  (design %.4f mm/px, deviation %.1f%%)",
            sensitivity.sensitivity_mm_per_px,
            sensitivity.design_sensitivity_mm_per_px,
            sensitivity.relative_deviation_pct,
        )

    # ---- Step 3: Hand-eye ----
    hand_eye = None
    if step in ("all", "hand_eye"):
        hand_eye = run_hand_eye(
            config, backend, intrinsics,
            live=args.live, n_poses=args.n_frames,
        )

    # ---- Step 4: Validate artifact (hardware-only, AC#6) ----
    if step in ("all", "validate_artifact"):
        if not args.live:
            log.info(
                "[DRY-RUN] Artifact validation (AC#6) skipped — needs real hardware.\n"
                "  To validate: place a gauge block of known height under the scanner,\n"
                "  then run: uv run python scripts/calibrate_scanner.py --live --step validate_artifact"
            )
        else:
            log.info(
                "LIVE artifact validation: AC#6.\n"
                "  Place a gauge block (known height) on the print surface.\n"
                "  Measure the reconstructed height and compare to the nominal.\n"
                "  This step requires the calibration from steps 1–3 to be loaded.\n"
                "  (Not yet automated; owner must run manually and record the result.)"
            )
            log.warning(
                "AC#6 — Artifact validation is not automated in this script version. "
                "Owner must perform the measurement manually and record the result."
            )

    # ---- Assemble and save ----
    cal = ScannerCalibration(
        intrinsics=intrinsics,
        laser_plane=laser_plane,
        sensitivity=sensitivity,
        hand_eye=hand_eye,
        calibration_date=date.today().isoformat(),
        target_description="chessboard 9×6 inner corners, 25 mm squares",
        notes="dry-run" if not args.live else "live calibration",
        unverified=(not args.live),
    )

    cal_dict = save_calibration(cal)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as fh:
        yaml.dump(cal_dict, fh, default_flow_style=False, allow_unicode=True)

    log.info("Calibration written to %s", args.output)

    if not args.live:
        log.info(
            "\nDRY-RUN complete.  To perform real calibration:\n"
            "  uv run python scripts/calibrate_scanner.py --live\n\n"
            "ACs #1–#4 and #6 require real hardware captures and are UNCHECKED.\n"
            "  Checked ACs (#5, #7, #8) are verified by the test suite:\n"
            "  uv run pytest tests/test_scanner_calibration.py -v"
        )
    else:
        log.info(
            "\nLive calibration complete.\n"
            "Review %s, verify values, then set unverified: false.\n"
            "ACs #1–#4 (and #6 if artifact was measured) are now evidence-ready;\n"
            "paste this log output as evidence when checking each AC.",
            args.output,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
