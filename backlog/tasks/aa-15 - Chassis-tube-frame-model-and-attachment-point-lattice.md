---
id: AA-15
title: Chassis-tube frame model and attachment-point lattice
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-24 18:03'
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
- [x] #1 A frame model defines chassis, each arm base, tube and attachment-point frames with documented conventions, and states clearly that the chassis is fixed and the tube translates
- [x] #2 Attachment points are addressed by integer lattice index with 183.676 mm pitch, and index-to-chassis-frame pose is computed from the current tube pose rather than from accumulated deltas
- [x] #3 The model answers which lattice indices are currently held by each arm, which index is the next target for each arm, and whether that target is reachable given arm limits
- [x] #4 Tube pose can be updated from a completed advance and from a measured correction, and the two paths are distinguishable in the model state
- [x] #5 A reachability report lists, for a given tube pose, which lattice indices each arm can reach in both coarse approach and insertion poses
- [x] #6 State is serializable so an interrupted run can resume with correct lattice bookkeeping, and resuming from a stale or inconsistent state is rejected
- [x] #7 Tests cover index-to-pose math, advance updates, reachability limits and serialization round-trip
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Add AA-15 block to config.py: AA15FrameConfig (coarse_approach_offset_m, insertion_offset_m, Optional) and AA15Config with both arm offsets. Add Optional[AA15Config] field frames_model to HardwareConfig.
2. Add AA-15 block to hardware.yaml with placeholder unverified offset values for both arms.
3. Implement inchworm/frames.py: TubePose, UpdateKind enum, FrameModel, AttachmentPoint, LatticeState. Chassis is fixed, tube translates.
4. Implement inchworm/lattice.py: LatticeState JSON serialization, schema version, consistency check (config hash, held indices vs tube pose, monotonic advance counter), and resume-from-file logic.
5. Write tests/test_frames.py covering index-to-pose math, advance/correction updates, reachability, and serialization round-trip.
6. Verify tests pass with uv run pytest tests/test_frames.py.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation: inchworm/frames.py (new module, ~650 lines) — TubePose, UpdateKind, FrameModel, ArmReachability, ReachabilityReport, LatticeState, ResumeError. AA-15 config block added to config.py and hardware.yaml with unverified placeholder offsets. tests/test_frames.py: 67 tests, all pass. Full suite: 650 passed. No hardware-only ACs; all 7 ACs checkable with automated tests.

Orchestrator addition 2026-10-24: FrameModel.complete_advance(direction) derives the new tube pose from exactly one ATTACHMENT_PITCH_MM step, so callers cannot inject drift. Test test_complete_advance_is_exact_over_many_cycles: 1000 advances give error < 1e-9 m. Internal units are metres (documented in module; the pitch constant is mm).
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented inchworm/frames.py (FrameModel, TubePose, UpdateKind, LatticeState, ArmReachability, ReachabilityReport, ResumeError). Chassis-fixed/tube-translates convention documented in module docstring and enforced throughout. ATTACHMENT_PITCH_MM imported from inchworm.constants. Arm-base transforms read from config (unverified AA-16 placeholders). Reachability via AA-11 IK on coarse-approach and insertion offsets. JSON serialisation with schema_version, config_hash, advance counter, pitch consistency check; ResumeError on stale/inconsistent state. AA-15 config block appended to config.py and hardware.yaml. 67 new tests (test_frames.py), all pass; full suite 650 passed.
<!-- SECTION:FINAL_SUMMARY:END -->
