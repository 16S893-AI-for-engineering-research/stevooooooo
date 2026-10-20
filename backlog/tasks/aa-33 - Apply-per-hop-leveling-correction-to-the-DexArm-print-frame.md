---
id: AA-33
title: Apply per-hop leveling correction to the DexArm print frame
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
labels: []
milestone: m-5
dependencies:
  - AA-32
  - AA-19
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 33000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
A measured correction is useless until it changes where the print head actually goes. After every hop the part sits slightly differently, so the correction must be applied to the DexArm print frame before the next segment is printed, and it must be applied in a way that is auditable and bounded: a bad scan that produced a large correction must not be allowed to drive the nozzle into the part.

This closes the leveling loop and is the last piece needed before unattended multi-cycle operation.

See doc-1 sections 1, 3 and 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The Z offset and tilt correction are applied as a transform on the DexArm print frame, and the applied transform is logged with the scan that produced it
- [ ] #2 Corrections larger than a configured sanity bound are rejected and trigger a rescan, then a safe halt if the rescan also exceeds the bound
- [ ] #3 Correction is applied after folding and before any print motion, and the ordering is enforced rather than left to the caller
- [ ] #4 Verification rescans after applying the correction and confirms the residual Z offset and tilt are within tolerance, reporting the residual
- [ ] #5 Corrections are not silently accumulated across hops; the policy for absolute versus incremental application is documented and implemented explicitly
- [ ] #6 A per-hop record of measured correction, applied transform and verified residual is written to the session artifacts
- [ ] #7 Tests cover application, the sanity-bound rejection and rescan path, ordering enforcement and residual verification
<!-- AC:END -->
