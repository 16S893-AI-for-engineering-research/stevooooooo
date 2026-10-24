---
id: AA-30
title: Scanner camera and laser-plane calibration
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:13'
updated_date: '2026-10-24 18:33'
labels: []
milestone: m-5
dependencies:
  - AA-29
  - AA-14
  - AA-13
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 30000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Turning a laser line in an image into millimeters of height requires knowing the camera intrinsics, the lens distortion, and the laser plane pose relative to the camera. The owner designed the assembly for a 40 degree laser angle, a 67 mm offset and an 80 mm standoff, which should give roughly 0.1 mm of height change per pixel, but those are design values. The as-built geometry will differ, and an uncalibrated 0.1 mm/px assumption would silently bias every leveling correction.

Calibration must also be expressed in the DexArm frame, because the scanner rides on the DexArm and the leveling output has to be applied in the print frame. AA-13 is a dependency because DexArm pose error would otherwise be absorbed into the calibration as if it were optical error.

See doc-1 sections 3, 5 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Camera intrinsics and distortion for the Arducam scanner camera are calibrated with a documented target, and reprojection error is reported
- [ ] #2 The laser plane pose relative to the camera is calibrated from observations at multiple known standoffs, with residuals reported
- [ ] #3 The calibrated height-per-pixel sensitivity at the 80 mm design standoff is reported and compared against the 0.1 mm/px design expectation, with any discrepancy explained
- [ ] #4 The transform from the camera and laser assembly to the DexArm tool frame is determined, so scan results can be expressed in the print frame
- [x] #5 Calibration results, residuals, the target used and the date are written into the hardware config, and stale calibration produces a warning at load
- [ ] #6 Accuracy is validated against at least one artifact of known geometry (for example a gauge block or step of known height) and the measured error is reported
- [x] #7 The procedure is documented in enough detail to be repeated after any disturbance of the scanner assembly
- [x] #8 Tests cover the solver on synthetic data with known ground truth and the stale-calibration warning
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
AA-30 (software half) implementation plan:

1. Create inchworm/scanner_calibration.py with:
   - calibrate_camera_intrinsics(): chessboard/ChArUco cv2.calibrateCamera wrapper, returns intrinsics + reprojection error
   - fit_laser_plane(): SVD plane fit from 3D laser-line points, returns plane normal/d + residuals
   - compute_sensitivity(): mm/px at 80mm standoff from calibrated geometry vs design 0.1mm/px
   - calibrate_hand_eye(): cv2.calibrateHandEye wrapper using DexArm poses
   - load_calibration() / save_calibration(): with stale-calibration warning
   - ScannerCalibration dataclass wrapping LaserCalibration + metadata

2. Add scanner_calibration block to config.py and hardware.yaml

3. Create tests/test_scanner_calibration.py:
   - Synthetic chessboard rendering through known intrinsics, recover within reported error
   - Synthetic laser points on known planes with noise, recover plane within residuals
   - Stale calibration warning test

4. Create scripts/calibrate_scanner.py (dry-run default, --live flag, laser via LaserDriver only)

5. Create docs/scanner_calibration.md

ACs #1-#4, #6: requires real hardware captures, will be left unchecked with comment.
ACs #5, #7, #8: checked if synthetic tests pass.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation notes:
- New module: robot/src/inchworm/scanner_calibration.py
- New test:   robot/tests/test_scanner_calibration.py (23 tests, all pass)
- New script: robot/scripts/calibrate_scanner.py (dry-run default, --live flag)
- New doc:    robot/docs/scanner_calibration.md
- Config addition: ScannerCalibrationSchema in config.py; scanner_calibration block in hardware.yaml
- All design literals (angle/baseline/standoff) imported from inchworm.constants; none restated
- Laser controlled exclusively via LaserDriver (AA-29)
- All motion entries have dry_run support and go through DexArmDriver (AA-12)
- Telemetry via stdlib logging with [AA-30] prefix
- Hardware-only ACs (#1-#4, #6) have comments with exact commands
- cv2.calibrateHandEye not in OpenCV 5.0.0 alpha; pure-numpy Tsai-Lenz fallback implemented
- Full test suite: 924 pass, 1 pre-existing fail (test_fold.py hardcoded path to other worktree)
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
created: 2026-10-24 18:33
---
AC#1 needs owner-supervised hardware session: uv run python scripts/calibrate_scanner.py --live --step intrinsics
needs Arducam OV9281 scanner camera attached, chessboard 9x6 inner corners 25mm square target printed on flat stock.
---

created: 2026-10-24 18:33
---
AC#2 needs owner-supervised hardware session: uv run python scripts/calibrate_scanner.py --live --step laser_plane
needs laser and camera attached, flat chessboard target at multiple standoffs (65-90mm), laser via LaserDriver only.
---

created: 2026-10-24 18:33
---
AC#3 needs owner-supervised hardware session: computed automatically when ACs #1 and #2 are done.
uv run python scripts/calibrate_scanner.py --live --step all
---

created: 2026-10-24 18:33
---
AC#4 needs owner-supervised hardware session: uv run python scripts/calibrate_scanner.py --live --step hand_eye
needs DexArm and camera attached, chessboard visible from all DexArm positions.
---

created: 2026-10-24 18:33
---
AC#6 needs owner-supervised hardware session: uv run python scripts/calibrate_scanner.py --live --step validate_artifact
needs a gauge block or step artifact of known height, and ACs #1-#4 completed first.
---
<!-- COMMENTS:END -->
