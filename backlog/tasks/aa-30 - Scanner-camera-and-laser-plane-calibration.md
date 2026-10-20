---
id: AA-30
title: Scanner camera and laser-plane calibration
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
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
- [ ] #5 Calibration results, residuals, the target used and the date are written into the hardware config, and stale calibration produces a warning at load
- [ ] #6 Accuracy is validated against at least one artifact of known geometry (for example a gauge block or step of known height) and the measured error is reported
- [ ] #7 The procedure is documented in enough detail to be repeated after any disturbance of the scanner assembly
- [ ] #8 Tests cover the solver on synthetic data with known ground truth and the stale-calibration warning
<!-- AC:END -->
