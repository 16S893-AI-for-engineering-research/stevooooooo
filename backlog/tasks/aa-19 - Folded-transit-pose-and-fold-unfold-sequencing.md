---
id: AA-19
title: Folded transit pose and fold/unfold sequencing
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-25 23:04'
labels: []
milestone: m-2
dependencies:
  - AA-11
  - AA-9
references:
  - pictures/robot.png
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 19000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
After each regrip both arms must return to the folded pose visible in pictures/robot.png. The fold serves two purposes: it brings the part as close as possible to the DexArm print head, and it is the pose from which the system expects repeatable geometry for leveling and printing. Because the folded configuration is tight, the order in which joints move matters, and a naive simultaneous interpolation can collide.

The fold is also the pose whose repeatability the leveling step effectively depends on, so its achieved repeatability needs to be measured rather than assumed.

See doc-1 sections 1, 2 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The folded transit pose for both arms is defined in config, matches the configuration shown in pictures/robot.png, and passes joint-limit and collision validation
- [x] #2 Fold and unfold sequences execute in a documented joint order with intermediate waypoints that are individually collision-checked, not a single simultaneous interpolation
- [x] #3 Folding is only permitted when both grippers report locked state, and attempting to fold otherwise is rejected with a clear reason
- [ ] #4 Achieved folded-pose repeatability is measured over at least 10 fold cycles and reported per joint and at the end effector
- [x] #5 An unfold path returns both arms to a state from which a coarse approach can begin, without passing through the keep-out zones
- [x] #6 Tests cover sequencing order, the lock precondition rejection and collision validation of all intermediate waypoints
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Define small ArmMover Protocol (move_to(q_rad, *, dry_run)) in inchworm/fold.py - no GuardedArm import, avoids parallel-lane conflicts.
2. Define GripperStatus Protocol (state property returns GripperState-compatible object with LOCKED check).
3. Implement FoldSequencer class: loads folded_transit named pose from AA-9 config; holds left/right MotionGuard for per-waypoint check_pose and per-segment check_trajectory; fold() and unfold() execute waypoints in documented joint ORDER (NOT simultaneous interpolation) with guards at each step.
4. fold() precondition: both grippers report LOCKED state via injected GripperStatus; if not, refuse with FoldRefused typed error.
5. Intermediate waypoints as config list (unverified: true placeholders in hardware.yaml under fold_sequence block).
6. Dry-run default for every motion entry point; log via AA-7 SessionLogger (optional inject).
7. Add HardwareConfig field fold_sequence: Optional[FoldSequenceConfig] = None in config.py.
8. Write scripts/fold_cycle.py (dry-run default, --live flag, --cycles 10, repeatability logging per-joint and at end-effector).
9. Write tests/test_fold.py covering: sequencing order, lock precondition rejection, collision validation of waypoints, unfold path.
10. Append hardware.yaml fold_sequence block with AA-19 markers.
11. Finalize ACs with test output evidence; leave AC#1 and AC#4 unchecked (hardware).
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
IMPLEMENTATION COMPLETE (software half).

New module: robot/src/inchworm/fold.py
  - ArmMoverProtocol (Protocol, runtime_checkable): move_to(q_rad, *, dry_run)
  - GripperStatusProtocol (Protocol, runtime_checkable): is_locked property
  - GripperLockSMStatus adapter: wraps any GripperLockSM-like object
  - FoldSequencer: fold(dry_run=True), unfold(dry_run=True)
    - check_pose per-waypoint + check_trajectory per-segment (MotionGuard)
    - fold refused unless both grippers report LOCKED (FoldRefused typed error)
    - UnverifiedWaypointError raised in live mode for any unverified waypoint
    - FoldMotionViolation raised on any guard violation with arm_role + step
    - AA-7 telemetry logged per step
  - build_fold_sequencer factory

New test file: robot/tests/test_fold.py (42 tests, all passing)

Config additions:
  - config.py: FoldWaypoint, FoldSequenceConfig (AA-19 markers)
  - HardwareConfig.fold_sequence: Optional[FoldSequenceConfig] = None
  - hardware.yaml: fold_sequence block (3 fold + 3 unfold waypoints, all unverified)

Script: robot/scripts/fold_cycle.py
  - dry-run default, --live flag, --cycles N (default 10)
  - repeatability reporting per-joint (mean, std, min, max)
  - AC#4 note: 'needs owner-supervised hardware session: uv run python scripts/fold_cycle.py --live --cycles 10'

Test evidence:
  uv run pytest tests/test_fold.py -v -> 42 passed
  uv run pytest -> 809 passed (no regressions)
  uv run python scripts/fold_cycle.py --cycles 10 -> dry-run 10 cycles OK

AC#1 and AC#4: UNCHECKED (hardware ACs)

Orchestrator 2026-10-24: removed a hard-coded worktree path in test_build_fold_sequencer_missing_fold_sequence_raises (it now uses DEFAULT_CONFIG_PATH).

Folded pose read by hand 2026-10-25 (owner placed both arms per pictures/robot.png; read-only scripts/read_pose.py; calibration_data/poses_2026-10-25.jsonl):
LEFT  q_rad [0.6266, -1.1037, 1.6935, -1.6306, 1.2824] (deg 35.9, -63.2, 97.0, -93.4, 73.5)
RIGHT q_rad [-0.0836, -0.9802, 1.6874, -1.5509, 1.4435] (deg -4.8, -56.2, 96.7, -88.9, 82.7)
Findings: (1) both elbows are at their mechanical end-stop, outside the EEPROM/soft limits (L 3119 > max 3091; R 3076 > max 3046), so the fold uses the full elbow travel; the limit update must include it (owner approval, EEPROM). (2) left shoulder_pan reads +35.9 deg while right reads -4.8 deg; asked the owner whether this asymmetry is intended. Not yet written to config; AC#1 also needs real collision validation.

2026-10-25: folded_transit set in config to [0.0, -1.25, 1.64, -1.31, 1.36] (both arms) from the hand-placed fold. pan = 0 per the owner (both point at the DexArm). lift/elbow are 2-3 deg short of the end stops. wrist_roll = the mean of the arms. The OLD PLACEHOLDER HAD LIFT/ELBOW SIGNS REVERSED (+1.2/-1.57), so a powered fold would have folded the wrong way; fold and unfold waypoint signs corrected. Unfold ordering test made sign-independent (abs lift). EEPROM limits widened so the fold is in-limits (owner-approved). AC#1 is NOT checked: the collision validation still uses placeholder geometry.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented AA-19 (software half): fold/unfold sequencing for SO-101 transit pose. New module inchworm/fold.py defines ArmMoverProtocol, GripperStatusProtocol (both runtime_checkable Protocols), GripperLockSMStatus adapter, FoldSequencer class, and build_fold_sequencer factory. FoldSequencer runs waypoints from hardware.yaml fold_sequence config sequentially (not simultaneously), calling MotionGuard.check_pose per-waypoint and check_trajectory per-segment. fold() requires both grippers LOCKED (FoldRefused if not). Live mode fails closed on unverified waypoints (UnverifiedWaypointError). AA-7 telemetry emitted per step. Added FoldWaypoint + FoldSequenceConfig to config.py with Optional field on HardwareConfig. Added fold_sequence block to hardware.yaml (3 fold + 3 unfold waypoints, all unverified). Added scripts/fold_cycle.py (dry-run default, --live flag, --cycles 10 repeatability logging). 42 new tests in test_fold.py; all 809 tests pass. AC#2 (sequencing order), AC#3 (lock precondition), AC#5 (unfold to coarse-approach-ready), AC#6 (sequencing + lock + collision tests) checked with test output. AC#1 and AC#4 remain UNCHECKED: needs owner-supervised hardware session: uv run python scripts/fold_cycle.py --live --cycles 10.
<!-- SECTION:FINAL_SUMMARY:END -->
