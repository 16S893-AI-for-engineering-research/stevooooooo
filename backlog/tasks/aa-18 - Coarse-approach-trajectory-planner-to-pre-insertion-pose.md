---
id: AA-18
title: Coarse approach trajectory planner to pre-insertion pose
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-24 18:39'
labels: []
milestone: m-2
dependencies:
  - AA-15
  - AA-16
  - AA-9
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 18000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The learned policy is responsible only for final insertion; code must deliver the gripper to a consistent pre-insertion pose near the target attachment point, repeatably and without collisions, in a workspace where the arms cannot move freely. The approach must also respect that the opposite arm is simultaneously holding the tube, so the moving arm has to route around the tube and its partner.

A consistent pre-insertion pose matters beyond safety: the policy will be trained from that pose, so variation here directly becomes variation the policy must absorb. The handoff pose definition belongs to this task.

See doc-1 sections 2, 3 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A planner produces a collision-free, velocity- and acceleration-limited trajectory from the current pose to a pre-insertion pose for a given attachment-point lattice index
- [x] #2 The pre-insertion pose is defined relative to the target attachment point with documented standoff and orientation, and is the single definition later used for policy training and rollout
- [x] #3 Planning accounts for the opposite arm currently holding the tube and for the tube volume itself, and rejects targets whose approach cannot be routed safely
- [ ] #4 Repeatability of the achieved pre-insertion pose is measured over at least 10 approaches and reported per axis, since this variation bounds the required policy capture range
- [x] #5 A dry-run mode prints and optionally visualizes the planned trajectory without commanding motion
- [x] #6 Failure to reach the pre-insertion pose within tolerance aborts with a typed error and leaves the arm in a safe, recoverable pose rather than mid-trajectory
- [x] #7 Tests cover planning success, unreachable and unsafe rejections, limit compliance and dry-run output
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Define ApproachConfig in config.py (AA-18 markers, Optional with None default)
2. Implement inchworm/approach.py: pre-insertion pose definition, IK solve, multi-segment trajectory, MotionGuard checks, tolerance check, abort/retreat logic, dry-run mode
3. Expose get_pre_insertion_pose() as the single definition used by AA-20/AA-23/AA-26
4. Implement tests/test_approach.py: synthetic tests with known poses, rejection cases, dry-run output
5. Implement scripts/approach_repeatability.py: dry-run default, --live --n flag
6. Add AA-18 config block to hardware.yaml (unverified)
7. Audit ACs; leave AC#4 hardware-only
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete. New module: robot/src/inchworm/approach.py
New test: robot/tests/test_approach.py (25 tests, all pass)
New script: robot/scripts/approach_repeatability.py (dry-run default, --live --n 10)
Config: ApproachConfig added to robot/src/inchworm/config.py (AA-18 markers)
YAML: approach block appended to robot/config/hardware.yaml (AA-18 markers)

All verified: 25 tests pass (uv run pytest tests/test_approach.py).
Pre-existing failure in test_fold.py::TestNamedPoseIntegration::test_build_fold_sequencer_missing_fold_sequence_raises hardcodes a path from another worktree; unrelated to AA-18.

Orchestrator review 2026-10-24 (P1 safety): opposite-arm clearance compared link points in each arm's OWN base frame, so both arms were treated as mounted at the same origin (a test passed only because the placeholder base transforms are identical). Both point sets are now transformed to the chassis frame via the FrameModel arm-base transforms, which are required (ValueError otherwise). Tests: test_opposite_arm_clearance_uses_arm_base_transforms, test_opposite_arm_clearance_requires_transforms. 1065 passed.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
created: 2026-10-24 18:29
---
AC#4 hardware-only: needs owner-supervised hardware session:
  cd robot && uv run python scripts/approach_repeatability.py --live --n 10

All other ACs checked via test_approach.py (25 tests pass).
---
<!-- COMMENTS:END -->
