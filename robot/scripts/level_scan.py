#!/usr/bin/env python3
"""Level scan script — AA-32: scan pattern and surface fit.

Sweeps the DexArm scanner across the end of the printed tube, stitches the
height profiles into a point cloud, fits a plane, and prints the leveling
correction (Z offset, rx, ry) with residual RMS and uncertainty.

This script is a **DRY-RUN by default**: it uses the mock backend and simulates
the scan without touching hardware.  Pass ``--live`` to use the real hardware
backend (OWNER SUPERVISION REQUIRED).

Usage
-----
    # Dry run (safe, no hardware):
    cd robot && uv run python scripts/level_scan.py

    # Live run (hardware, owner supervision required):
    cd robot && uv run python scripts/level_scan.py --live

AC coverage (this script)
--------------------------
- AC#1 (scan duration): the elapsed time is printed; needs owner to confirm
  on hardware with --live.  UNCHECKED until hardware session.
  needs owner-supervised hardware session:
    cd robot && uv run python scripts/level_scan.py --live
- AC#5 (10-scan repeatability): use --repeats 10 --live.  UNCHECKED.
  needs owner-supervised hardware session:
    cd robot && uv run python scripts/level_scan.py --live --repeats 10
- AC#6 (deliberate shift recovered): introduce a known Z shift to the
  DexArm fixture and run --live.  UNCHECKED.
  needs owner-supervised hardware session:
    cd robot && uv run python scripts/level_scan.py --live

Safety invariants
-----------------
* Laser is on ONLY during each capture window (AA-29 context manager).
* All DexArm moves go through the AA-12 bounded DexArmDriver API.
* dry_run mode passes dry_run=True to all motion entry points.
* standoff_z_mm must be set in hardware.yaml before --live (fails closed).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from inchworm.backends import MockBackend, create_backend
from inchworm.config import load_config
from inchworm.dexarm import DexArmDriver, MockDexArmSerial
from inchworm.laser import open_laser
from inchworm.leveling import (
    LevelingRefused,
    ScanConfig,
    level_scan,
    run_scan,
    stitch_profiles,
    fit_surface,
)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument(
    "--live",
    action="store_true",
    default=False,
    help="Use real hardware backend. OWNER SUPERVISION REQUIRED.",
)
parser.add_argument(
    "--repeats",
    type=int,
    default=1,
    metavar="N",
    help=(
        "Number of scan repeats (default 1). Use --repeats 10 --live for "
        "AC#5 repeatability measurement."
    ),
)
parser.add_argument(
    "--config",
    default=None,
    metavar="PATH",
    help="Path to hardware.yaml (default: robot/config/hardware.yaml).",
)


# ---------------------------------------------------------------------------
# Synthetic profile generator (dry-run only)
# ---------------------------------------------------------------------------

def _make_synthetic_capture_and_extract(x_arm: float, y_arm: float, z_arm: float):
    """Return (capture_fn, extract_fn) that produce a known synthetic plane.

    The synthetic plane is Z = 0.01*X + -0.02*Y + 80.0 mm (design placeholder).
    Points are returned relative to the scanner (which equals tool frame in
    dry-run, identity T_tool_scanner).  After stitching with arm position,
    the print-frame Z should match the plane formula.
    """
    import numpy as np

    def capture_fn():
        return None  # not used

    def extract_fn(frame):
        # In dry-run, arm position is (x_arm, y_arm, z_arm).
        # Return a single point at scanner frame origin = (0,0,0).
        # After stitching: P_print = (0,0,0) + (x_arm, y_arm, z_arm).
        # For a meaningful cloud, return several points offset in X, Y.
        pts = []
        for dx in [-5.0, 0.0, 5.0]:
            for dy in [-5.0, 0.0, 5.0]:
                # Compute Z from the "true" plane in print frame
                px = x_arm + dx
                py = y_arm + dy
                pz_print = 0.01 * px + (-0.02) * py + 80.0
                # scanner frame: subtract arm position to get scanner-relative Z
                pts.append([dx, dy, pz_print - z_arm])
        return np.array(pts, dtype="float64")

    return capture_fn, extract_fn


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parser.parse_args()
    config = load_config(args.config)

    # Build ScanConfig from hardware.yaml if leveling block is present.
    lev_cfg_yaml = config.leveling
    if lev_cfg_yaml is not None:
        scan_cfg = ScanConfig(
            standoff_z_mm=lev_cfg_yaml.standoff_z_mm,
            scan_x_min_mm=lev_cfg_yaml.scan_x_min_mm,
            scan_x_max_mm=lev_cfg_yaml.scan_x_max_mm,
            scan_y_min_mm=lev_cfg_yaml.scan_y_min_mm,
            scan_y_max_mm=lev_cfg_yaml.scan_y_max_mm,
            scan_x_steps=lev_cfg_yaml.scan_x_steps,
            scan_y_steps=lev_cfg_yaml.scan_y_steps,
            feedrate_mm_min=lev_cfg_yaml.feedrate_mm_min,
            settle_wait_s=lev_cfg_yaml.settle_wait_s,
            ransac_inlier_threshold_mm=lev_cfg_yaml.ransac_inlier_threshold_mm,
            ransac_trials=lev_cfg_yaml.ransac_trials,
            min_valid_points=lev_cfg_yaml.min_valid_points,
            min_inlier_fraction=lev_cfg_yaml.min_inlier_fraction,
            max_residual_rms_mm=lev_cfg_yaml.max_residual_rms_mm,
            max_uncertainty_mm=lev_cfg_yaml.max_uncertainty_mm,
        )
    else:
        scan_cfg = ScanConfig()

    dry_run = not args.live

    if args.live:
        print("WARNING: --live mode. Real hardware will move. Owner supervision required.")
        print(f"  standoff_z_mm = {scan_cfg.standoff_z_mm} (None = FAIL CLOSED)")
        backend = create_backend(config)
    else:
        print("DRY-RUN mode (mock backend). No hardware touched.")
        backend = MockBackend(config)

    # Open DexArm.
    if args.live:
        dex_transport = backend.open_serial("dexarm")
        arm = DexArmDriver(
            dex_transport,
            workspace=config.dexarm.workspace,
        )
        arm.connect()
    else:
        mock_serial = MockDexArmSerial()
        # Position mock arm at a sensible starting position.
        mock_serial.set_position(0.0, 130.0, 150.0)
        arm = DexArmDriver(mock_serial, workspace=config.dexarm.workspace)
        arm.connect()

    # Open laser.
    laser_transport = backend.open_serial("relay")
    from inchworm.laser import LaserDriver
    laser = LaserDriver(laser_transport, config)

    n_repeats = args.repeats
    results = []

    print(f"\nRunning {n_repeats} scan(s) …")
    try:
        for repeat_idx in range(n_repeats):
            t_start = time.monotonic()

            if dry_run:
                # In dry-run: use synthetic profiles to get a non-trivial cloud.
                # Manually run stops to build synthetic profiles.
                from inchworm.leveling import scan_pattern
                import numpy as np

                stops = scan_pattern(scan_cfg)
                standoff_z = scan_cfg.standoff_z_mm if scan_cfg.standoff_z_mm is not None else 150.0
                scan_data = []
                for x, y in stops:
                    arm.move_absolute(x, y, standoff_z, feedrate=scan_cfg.feedrate_mm_min, dry_run=True)
                    reported_pos = (x, y, standoff_z)
                    _, ext_fn = _make_synthetic_capture_and_extract(x, y, standoff_z)
                    # Simulate laser context manager for dry-run.
                    with laser:
                        pts = ext_fn(None)
                    scan_data.append((reported_pos, pts))

                cloud, t_unverified = stitch_profiles(scan_data, T_tool_scanner=None)
            else:
                # Live: inject real capture callables.
                # The caller would provide AA-14/AA-31 callables; for now
                # we capture a frame from the scanner camera and extract via AA-31.
                from inchworm.camera_capture import CameraCapturePipeline
                from inchworm.camera_arbiter import CameraArbiter
                from inchworm.laser_line import (
                    LaserLineConfig,
                    extract_laser_line,
                    triangulate_profile,
                    default_laser_calibration,
                )
                import numpy as np

                arbiter = CameraArbiter(backend, config)
                calib = default_laser_calibration()

                def capture_fn():
                    with arbiter.open("camera_scanner") as cam:
                        raw = cam.read_frame()
                    # Decode MJPEG to numpy
                    import cv2
                    buf = np.frombuffer(raw, dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
                    return frame

                def extract_fn(frame):
                    if frame is None:
                        return np.zeros((0, 3), dtype=np.float64)
                    res = extract_laser_line(frame)
                    pts = triangulate_profile(res, calib, dry_run=True)
                    valid = np.all(np.isfinite(pts), axis=1)
                    return pts[valid]

                scan_data = run_scan(
                    arm, laser, capture_fn, extract_fn,
                    config=scan_cfg, dry_run=False,
                )
                cloud, t_unverified = stitch_profiles(scan_data, T_tool_scanner=None)

            elapsed_s = time.monotonic() - t_start

            print(f"\n  Scan {repeat_idx + 1}/{n_repeats}: {elapsed_s:.2f}s, "
                  f"{len(cloud)} points, T_tool_scanner_unverified={t_unverified}")

            try:
                result = fit_surface(
                    cloud, scan_cfg,
                    T_tool_scanner_unverified=t_unverified,
                    dry_run=dry_run,
                )
                results.append(result)
                print(
                    f"    z_offset={result.z_offset_mm:.4f} mm  "
                    f"rx={math.degrees(result.rx_rad):.4f}°  "
                    f"ry={math.degrees(result.ry_rad):.4f}°  "
                    f"rms={result.residual_rms_mm:.4f} mm  "
                    f"unc={result.uncertainty_mm:.4f} mm  "
                    f"inliers={result.n_inliers}/{result.n_total}"
                )
            except LevelingRefused as exc:
                print(f"    LevelingRefused({exc.reason}): {exc.detail}")
                results.append(None)

    finally:
        laser.close()
        if args.live:
            arm.disconnect()

    # Repeatability summary (for AC#5 measurement, needs --live --repeats 10)
    if n_repeats >= 2:
        import numpy as np
        valid = [r for r in results if r is not None]
        if len(valid) >= 2:
            zs = np.array([r.z_offset_mm for r in valid])
            rxs = np.array([r.rx_rad for r in valid])
            rys = np.array([r.ry_rad for r in valid])
            print(f"\nRepeatability over {len(valid)} valid scans:")
            print(f"  z_offset:  mean={zs.mean():.4f} mm  std={zs.std():.4f} mm  "
                  f"range={zs.max() - zs.min():.4f} mm")
            print(f"  rx (deg):  mean={math.degrees(rxs.mean()):.4f}  "
                  f"std={math.degrees(rxs.std()):.4f}")
            print(f"  ry (deg):  mean={math.degrees(rys.mean()):.4f}  "
                  f"std={math.degrees(rys.std()):.4f}")
            if not args.live:
                print("  (dry-run repeatability is synthetic; use --live for real data)")

    if dry_run:
        print("\nDRY-RUN complete. To check AC#1 (duration), AC#5 (repeatability),")
        print("or AC#6 (deliberate shift), owner must:")
        print("  1. Set standoff_z_mm in robot/config/hardware.yaml")
        print("  2. Run: cd robot && uv run python scripts/level_scan.py --live")
        print("  3. For AC#5: cd robot && uv run python scripts/level_scan.py --live --repeats 10")
    return 0


if __name__ == "__main__":
    sys.exit(main())
