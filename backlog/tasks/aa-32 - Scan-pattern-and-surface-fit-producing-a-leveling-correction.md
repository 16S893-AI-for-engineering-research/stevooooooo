---
id: AA-32
title: Scan pattern and surface fit producing a leveling correction
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
labels: []
milestone: m-5
dependencies:
  - AA-31
  - AA-12
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 32000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Individual height profiles have to become one actionable correction: the DexArm sweeps the scanner across the end of the printed tube, the profiles are stitched into a surface, and a fit yields the Z offset and the two tilt angles that the print frame needs. The tube is curved and its end face is the print interface, so the fit must represent that geometry rather than assuming a flat bed.

The output must also be trustworthy enough to act on autonomously, which means the fit has to report its own quality and refuse to emit a correction when the data does not support one.

See doc-1 sections 1, 3 and 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A scan pattern moves the DexArm through a documented path at the calibrated standoff, capturing profiles with the arm settled at each sample, and completes within a recorded duration
- [ ] #2 Profiles are stitched using DexArm position feedback into a single point set in the print frame
- [ ] #3 A geometric fit appropriate to the printed tube end (documented and justified, for example plane plus tube-axis or cylinder fit) yields a Z offset and two tilt angles with residuals and an estimated uncertainty
- [ ] #4 The fit rejects outliers and refuses to emit a correction when coverage, residual or uncertainty thresholds are not met, returning a typed failure instead
- [ ] #5 Repeatability of the computed correction is measured over at least 10 scans of an undisturbed part and reported, and meets or explains its relationship to a 0.2 mm Z target
- [ ] #6 A known tilt and a known Z shift introduced deliberately are recovered to within a reported tolerance
- [ ] #7 The laser is on only during capture and off for the rest of the sequence, enforced via AA-29
- [ ] #8 Tests cover stitching, fitting on synthetic surfaces with known parameters, outlier rejection and the refusal path
<!-- AC:END -->
