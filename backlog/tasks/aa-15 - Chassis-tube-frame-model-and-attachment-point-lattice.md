---
id: AA-15
title: Chassis-tube frame model and attachment-point lattice
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
labels: []
milestone: m-2
dependencies:
  - AA-11
references:
  - pictures/robot.png
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 15000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
In this machine the chassis is fixed to the ground and the tube is what moves. That inverts the usual mobile-robot convention and must be modeled explicitly, or every advance will be computed with the wrong sign or in the wrong frame. The attachment points form a 1-D lattice along the tube axis at 183.676 mm pitch, and the whole crawl cycle is bookkeeping over which lattice index each arm currently holds.

The model must track tube pose relative to the chassis, predict where the next attachment point will be after an advance, and expose that as a Cartesian target both the coarse planner and the insertion policy supervisor can consume. Lattice indices, not accumulated relative moves, should be the source of truth so errors do not compound over many cycles.

See doc-1 sections 1, 2 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A frame model defines chassis, each arm base, tube and attachment-point frames with documented conventions, and states clearly that the chassis is fixed and the tube translates
- [ ] #2 Attachment points are addressed by integer lattice index with 183.676 mm pitch, and index-to-chassis-frame pose is computed from the current tube pose rather than from accumulated deltas
- [ ] #3 The model answers which lattice indices are currently held by each arm, which index is the next target for each arm, and whether that target is reachable given arm limits
- [ ] #4 Tube pose can be updated from a completed advance and from a measured correction, and the two paths are distinguishable in the model state
- [ ] #5 A reachability report lists, for a given tube pose, which lattice indices each arm can reach in both coarse approach and insertion poses
- [ ] #6 State is serializable so an interrupted run can resume with correct lattice bookkeeping, and resuming from a stale or inconsistent state is rejected
- [ ] #7 Tests cover index-to-pose math, advance updates, reachability limits and serialization round-trip
<!-- AC:END -->
