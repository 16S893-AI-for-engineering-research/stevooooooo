---
id: AA-31
title: Laser line extraction and height profile triangulation
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
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
- [ ] #1 Line extraction produces sub-pixel line position per image column using a documented method, with manual exposure and gain settings recorded
- [ ] #2 Saturated, weak, multi-peak and missing line segments are detected and marked invalid rather than yielding a fabricated position
- [ ] #3 Extracted line positions are triangulated into 3D points in the DexArm print frame using the AA-30 calibration
- [ ] #4 Measured height accuracy and repeatability are reported against a known-geometry artifact, including noise standard deviation at the working standoff
- [ ] #5 Robustness is demonstrated on the real curved printed tube surface, not only on a flat coupon, with sample frames and extracted profiles retained as artifacts
- [ ] #6 Per-frame processing time is measured and reported, and is compatible with the intended scan pattern duration
- [ ] #7 Tests cover extraction on recorded frames including the degenerate cases, and triangulation against synthetic geometry with known ground truth
<!-- AC:END -->
