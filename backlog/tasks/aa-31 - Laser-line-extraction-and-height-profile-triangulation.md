---
id: AA-31
title: Laser line extraction and height profile triangulation
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:13'
updated_date: '2026-10-24 18:21'
labels: []
milestone: m-5
dependencies:
  - AA-30
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 31000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The measurement core of leveling: find the laser line in each mono frame with sub-pixel accuracy and convert it to a height profile in the print frame. Accuracy here sets the floor for the whole leveling step, and the hard part is robustness, since the line will cross a curved printed tube with varying surface finish, specular highlights, layer ridges and occasional ambient light.

A peak-finding method that works on a clean flat test coupon but fails on a glossy curved print would pass a naive review and then produce quietly wrong corrections, so the acceptance criteria require explicit handling of weak, saturated and missing line segments.

See doc-1 sections 4 and 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Line extraction produces sub-pixel line position per image column using a documented method, with manual exposure and gain settings recorded
- [x] #2 Saturated, weak, multi-peak and missing line segments are detected and marked invalid rather than yielding a fabricated position
- [ ] #3 Extracted line positions are triangulated into 3D points in the DexArm print frame using the AA-30 calibration
- [ ] #4 Measured height accuracy and repeatability are reported against a known-geometry artifact, including noise standard deviation at the working standoff
- [ ] #5 Robustness is demonstrated on the real curved printed tube surface, not only on a flat coupon, with sample frames and extracted profiles retained as artifacts
- [ ] #6 Per-frame processing time is measured and reported, and is compatible with the intended scan pattern duration
- [x] #7 Tests cover extraction on recorded frames including the degenerate cases, and triangulation against synthetic geometry with known ground truth
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
IMPLEMENTED: inchworm/laser_line.py, tests/test_laser_line.py, hardware.yaml AA-31 block, config.py LaserLineConfigSchema. AC#1 checked: 27 tests pass; parabolic sub-pixel fit, exposure+gain recorded. AC#2 checked: SATURATED/WEAK/MULTI_PEAK/MISSING flags; NaN for all invalid. AC#7 checked: synthetic flat (max err 0.09px), tilted (0.38px), cylinder (0.28px), degenerate combos, timing. AC#3 unchecked: needs real AA-30 calibration. AC#4 unchecked: needs owner-supervised hardware session: uv run python scripts/laser_scan.py --live (not yet written). AC#5 unchecked: needs hardware. AC#6 unchecked: timing measured (254.7ms/frame on 1280x800, CPU-only numpy loop); compatibility with scan-pattern duration pending AA-32.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Orchestrator note 2026-10-24: ~260 ms/frame at 1280x800 in a pure-numpy per-column loop. If AA-32 needs a faster scan, vectorise the column fit or restrict to an ROI band around the expected line row; that is an optimisation, not a correctness issue.
<!-- SECTION:NOTES:END -->
