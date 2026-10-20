---
id: AA-16
title: Chassis-arm-tube calibration procedure
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
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
- [ ] #4 Results are written back into the hardware config with timestamps and residuals, and loading a config with missing or stale calibration produces an explicit warning
- [ ] #5 A verification step predicts the Cartesian position of a known attachment point and compares it to measurement, reporting error per axis
- [ ] #6 Coarse-approach positioning error after calibration is quantified and recorded, since this value sets the capture range required of the insertion policy
- [ ] #7 Tests cover the solver on synthetic data with known ground truth and the stale-calibration warning path
<!-- AC:END -->
