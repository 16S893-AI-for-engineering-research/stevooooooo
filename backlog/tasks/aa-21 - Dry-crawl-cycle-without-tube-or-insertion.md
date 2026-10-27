---
id: AA-21
title: Dry crawl cycle without tube or insertion
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-27 20:25'
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
- [x] #1 A cycle runner executes the complete sequence (hold check, unlock moving arm, retract, coarse approach to next lattice index, stub-policy handoff, lock, fold) with no tube present and no real insertion
- [x] #2 Lattice bookkeeping advances correctly across at least 5 consecutive dry cycles and ends in a consistent, serializable state
- [x] #3 Every state transition emits structured events, and a per-cycle timeline summary reports phase durations and outcomes
- [x] #4 Injected faults (approach failure, stub-policy abort, lock verification failure) each drive the documented recovery path and the run ends in a safe pose
- [x] #5 The runner refuses to start if calibration is missing, limits are unvalidated, or either gripper state is unknown
- [x] #6 A dry-run mode plans all cycle motion without commanding servos, and the planned sequence is asserted in tests
- [ ] #7 Measured cycle time and per-phase timing are recorded in task notes as a baseline for the tube-attached cycle
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Implement inchworm/cycle.py: CycleRunner class with dry-run default, preflight checks (AC#5), per-cycle sequence (AC#1), fault injection recovery (AC#4), AA-7 event emission (AC#3), FrameModel bookkeeping via complete_advance (AC#2).
2. Implement ScriptedSeatingDetector for dry-cycle use (triggers success on configured step).
3. Add CycleRunnerConfig to config.py inside AA-21 markers as Optional field with default None.
4. Write tests/test_cycle.py covering AC#1-#6.
5. Write scripts/dry_cycle.py with dry-run default and --live flag.
6. Leave AC#7 unchecked (hardware); note planned dry-run durations in task notes.
Architecture decisions:
- Advance arm: the arm that just re-gripped (the previously moving arm M) completes the advance by locking onto the new attachment point; FrameModel.complete_advance() is called AFTER M locks, immediately before the fold phase. This means the HOLDING arm H defines the baseline; M leapfrogs it. Documented as config-driven choice (advance_direction_owner_confirmed: false) marked for owner confirmation.
- Moving arm alternates each cycle (left/right swap).
- Preflight checks: AA-16 calibration block presence, motion_guard.joint_limits_verified, both gripper SMs not in UNKNOWN state -> raise PreflightError listing all failing conditions.
- Fault recovery: approach failure -> retract M, lock M if possible else FAULT, fold; supervisor abort/halt -> retract M, re-lock M, fold; lock verification failure -> retract, fault-safe pose, both grippers assert H LOCKED.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete. New module: robot/src/inchworm/cycle.py. Tests: robot/tests/test_cycle.py (38 tests, all pass). Script: robot/scripts/dry_cycle.py (dry-run default, --live flag, --cycles N).

Config change: added CycleRunnerConfig to config.py and HardwareConfig.cycle_runner Optional field (AA-21 markers).

Planned dry-run durations (cycle 0, unverified):
  preflight               0.000 s
  unlock_moving           0.050 s
  retract_moving          0.200 s
  approach_moving         1.421 s (IK + trapezoidal, lattice index 1)
  supervisor_handoff      0.300 s (2 stub policy steps x 0.1s)
  lock_moving             0.050 s
  advance_bookkeeping     0.000 s
  fold                    0.500 s
  Total estimated:        ~2.521 s / cycle (wall-clock on dev machine: ~0.012 s due to dry-run no-ops)

Note: IK fails at lattice index 2+ with unverified arm-base transforms (ApproachUnreachableError fault path, safe recovery confirmed). 5-cycle AC#2 test passes via monkeypatched mock approach that bypasses IK. Script faults on cycles 1-4 with safe recovery -- expected until AA-16 calibration is complete.

Advance arm choice: the moving arm M leapfrogs the holding arm H. FrameModel.complete_advance() is called after M locks and before fold. Advance direction is +1 (config-driven, advance_direction_owner_confirmed: false by default -- owner must confirm). Both grippers-unlocked invariant asserted in all fault paths.

AC#7 needs owner-supervised hardware session: uv run python scripts/dry_cycle.py --live --cycles 5

Owner decision 2026-10-27: the tube advances to the right, AWAY from the DexArm. Recorded as cycle_runner.advance_physical: away_from_dexarm (new config field) with advance_direction: 1 in a new hardware.yaml cycle_runner block. The live preflight now checks the sign geometrically: advance_direction * calibration.tube_axis.axis_direction must point from dexarm_base toward the arm-base midpoint (|cos| >= 0.5); missing calibration, an undecided advance_physical, a perpendicular axis or a wrong sign are each preflight failures (fail closed). This also covers a PCA axis fit that comes out reversed. advance_direction_owner_confirmed stays false until the owner sees the tube move away from the DexArm in the first supervised live cycle. Tests: TestAdvancePhysicalDirection (9), tests/test_cycle.py 49 passed.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented AA-21 dry crawl cycle runner in robot/src/inchworm/cycle.py (CycleRunner, ScriptedSeatingDetector, CyclePlan, PreflightError, CycleFaultError, SafetyInvariantViolation, build_cycle_runner). Added CycleRunnerConfig to config.py. 38 new tests in robot/tests/test_cycle.py covering all software ACs; all pass (1103 total, 0 failures). Added scripts/dry_cycle.py (dry-run default, --live flag, --cycles N). ACs #1-#6 verified with test output. AC#7 requires hardware: uv run python scripts/dry_cycle.py --live --cycles 5.
<!-- SECTION:FINAL_SUMMARY:END -->
