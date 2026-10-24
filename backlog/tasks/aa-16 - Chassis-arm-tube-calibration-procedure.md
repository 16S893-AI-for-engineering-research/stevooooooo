---
id: AA-16
title: Chassis-arm-tube calibration procedure
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-24 18:30'
labels: []
milestone: m-2
dependencies:
  - AA-15
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 16000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The frame model is only as good as the transforms in it. The two arm bases, the DexArm base and the tube axis all have unknown real offsets relative to the chassis, and the wrist cameras have unknown transforms relative to their arm end-effectors. Coarse approach must land close enough that the learned insertion policy sees the attachment point in frame, so these transforms need to be measured, not guessed from CAD alone.

Produce a repeatable procedure plus a solver, so calibration can be redone after any mechanical change without re-deriving the method.

See doc-1 sections 2, 3, 4 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A documented procedure measures arm-base-to-chassis transforms for both SO-101 arms and the DexArm, using observable features and stated instruments
- [ ] #2 A hand-eye calibration determines each wrist camera pose relative to its arm end-effector, with reported residual error
- [ ] #3 Tube axis position and orientation relative to the chassis are determined, and the method handles the tube being movable rather than fixed
- [x] #4 Results are written back into the hardware config with timestamps and residuals, and loading a config with missing or stale calibration produces an explicit warning
- [ ] #5 A verification step predicts the Cartesian position of a known attachment point and compares it to measurement, reporting error per axis
- [ ] #6 Coarse-approach positioning error after calibration is quantified and recorded, since this value sets the capture range required of the insertion policy
- [x] #7 Tests cover the solver on synthetic data with known ground truth and the stale-calibration warning path
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Implement inchworm/calibration.py with:
   a. Kabsch/Umeyama rigid transform solver (arm-base-to-chassis, DexArm-to-chassis) with residuals
   b. Hand-eye AX=XB solver via cv2.calibrateHandEye (Tsai-Lenz) with residuals
   c. Tube-axis line fit (through attachment-point centers, relative to lattice index + FrameModel)
   d. Result loader with stale-calibration warning (warnings.warn + AA-7 event)
   e. Verification helper: predict attachment-point position, compare per-axis

2. Add AA-16 CalibrationConfig block to config.py with Optional fields and None defaults

3. Add aa16_calibration block to hardware.yaml (inside markers)

4. Write scripts/calibrate_arm_base.py (dry-run default, --live)
5. Write scripts/hand_eye.py (dry-run default, --live)
6. Write scripts/tube_axis.py (dry-run default, --live)

7. Write robot/docs/calibration_procedure.md with instruments/fiducial specs

8. Write tests/test_calibration.py covering:
   - Kabsch solver on synthetic data with known ground truth + noise
   - Hand-eye solver on synthetic data
   - Tube-axis line fit on synthetic data
   - Stale-calibration warning path

9. Finalize ACs per finalization guide
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implemented inchworm/calibration.py:
  (a) kabsch_transform / calibrate_rigid: Kabsch/Umeyama rigid transform from point correspondences.
  (b) hand_eye_park_martin / calibrate_hand_eye: Park-Martin AX=XB solver (pure NumPy; falls back from cv2.calibrateHandEye which is absent from this OpenCV 5.0.0 build). Bug fixed in outer product order.
  (c) fit_tube_axis: PCA line fit through attachment-point centres, recovers axis direction, lattice origin, pitch estimate.
  CalibrationResult + TubeAxisResult: serialise to hardware.yaml-compatible dicts with ISO timestamps and residuals.
  check_calibration_age: warnings.warn (CalibrationStalenessWarning) + AA-7 SessionLogger event on missing or stale calibration.
  predict_attachment_point: per-axis prediction vs measurement comparison.

config.py: Added AA16RigidCalibration, AA16TubeAxisCalibration, AA16CalibrationConfig classes + HardwareConfig.calibration Optional field.
hardware.yaml: Added AA-16 calibration block (all null, max_calibration_age_days: 90).
scripts/calibrate_arm_base.py, hand_eye.py, tube_axis.py: dry-run default, --live flag.
docs/calibration_procedure.md: Full procedure with instruments (calipers, ChArUco 5x7 30mm squares), chassis frame convention, step-by-step for each calibration, verification, stale-warning, re-calibration triggers.
tests/test_calibration.py: 32 tests all passing. Covers Kabsch/hand-eye/tube-axis on synthetic data with known ground truth + noise; residuals reported vs true; stale warning path.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @agent
created: 2026-10-24 18:30
---
AC#1: needs owner-supervised hardware session: uv run python scripts/calibrate_arm_base.py --live --device arm_left (and arm_right, dexarm)
---

author: @agent
created: 2026-10-24 18:30
---
AC#2: needs owner-supervised hardware session: uv run python scripts/hand_eye.py --live --arm left (and right)
---

author: @agent
created: 2026-10-24 18:30
---
AC#3: needs owner-supervised hardware session: uv run python scripts/tube_axis.py --live
---

author: @agent
created: 2026-10-24 18:30
---
AC#5: needs owner-supervised hardware session: uv run python scripts/calibrate_arm_base.py --live --device arm_left then verify predict_attachment_point output
---

author: @agent
created: 2026-10-24 18:30
---
AC#6: needs owner-supervised hardware session: run coarse-approach trajectory after calibration and measure residual with calipers
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented AA-16 software half: inchworm/calibration.py with (a) Kabsch solver, (b) Park-Martin hand-eye AX=XB, (c) PCA tube-axis line fit; CalibrationResult/TubeAxisResult dicts; check_calibration_age warning (warnings.warn + AA-7 event); predict_attachment_point verification helper. Added AA16CalibrationConfig to config.py (all Optional/None), null block in hardware.yaml. Scripts: calibrate_arm_base.py, hand_eye.py, tube_axis.py (dry-run default, --live). docs/calibration_procedure.md. tests/test_calibration.py: 32 tests pass (synthetic data with known ground truth + noise, stale-warning path). AC#4 and AC#7 checked via test output. AC#1, #2, #3, #5, #6 left unchecked (require owner-supervised hardware session).
<!-- SECTION:FINAL_SUMMARY:END -->
