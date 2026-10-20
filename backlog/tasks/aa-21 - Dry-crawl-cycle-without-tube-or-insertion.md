---
id: AA-21
title: Dry crawl cycle without tube or insertion
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
labels: []
milestone: m-2
dependencies:
  - AA-19
  - AA-20
  - AA-17
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 21000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The crawl cycle cannot be fully validated before the learned insertion exists, but the owner noted that the motion skeleton can be exercised without the tube attached. This task delivers exactly that: the full grip, unlock, advance, re-approach, lock and fold sequence running in free air with the stub policy standing in for insertion, so sequencing, state bookkeeping, timing, limits and abort handling are shaken out before any hardware can be damaged by a real insertion attempt.

This is the safe rehearsal that de-risks the tube-attached work, and it is also the regression test the real cycle will later be compared against.

See doc-1 sections 1, 2 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A cycle runner executes the complete sequence (hold check, unlock moving arm, retract, coarse approach to next lattice index, stub-policy handoff, lock, fold) with no tube present and no real insertion
- [ ] #2 Lattice bookkeeping advances correctly across at least 5 consecutive dry cycles and ends in a consistent, serializable state
- [ ] #3 Every state transition emits structured events, and a per-cycle timeline summary reports phase durations and outcomes
- [ ] #4 Injected faults (approach failure, stub-policy abort, lock verification failure) each drive the documented recovery path and the run ends in a safe pose
- [ ] #5 The runner refuses to start if calibration is missing, limits are unvalidated, or either gripper state is unknown
- [ ] #6 A dry-run mode plans all cycle motion without commanding servos, and the planned sequence is asserted in tests
- [ ] #7 Measured cycle time and per-phase timing are recorded in task notes as a baseline for the tube-attached cycle
<!-- AC:END -->
