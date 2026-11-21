---
id: AA-54
title: >-
  Per-arm insertion recording programs from the frozen training positions with
  perturbed tube placement
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-21 01:54'
updated_date: '2026-11-21 01:55'
labels: []
milestone: m-3
dependencies:
  - AA-23
priority: high
type: feature
ordinal: 54000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner plan 2026-11-21 (replaces the AA-39 full test flow). The left policy failed live from frame 42 and the old dataset was recorded at the old wrist_roll zero. The owner will retrain BOTH arms from two FROZEN training positions saved in robot/config/training_poses.json (train_left = cycle frame 42: right LOCKED holding the tube, left open under its point; train_right = frame 67: left LOCKED holding at its gait pose, right open under its point). Owner rule: these positions must not change after training unless the owner says so. One recording program per arm: it moves both arms (no tube) to that arm's training position; the owner adds the tube and puts the HOLDING gripper's key into its attachment point; code locks the holding gripper (AA-17) after the owner confirms. Then rapid-fire teleop recording as in record_handoff.py. Robustness: before each episode the HOLDING arm moves the tube so the attachment point in front of the inserting gripper is rotated up to 2 deg (any axis) and translated up to 20 mm (any of x/y/z) from the frozen pose; NOT a per-joint jitter. 50 episodes per arm: 10 at the exact frozen pose, 40 perturbed. The SO-101 has 5 joints, so not every 6-DOF perturbation is reachable: the achieved perturbation is what is recorded.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Two launchers exist, one per arm (left, right); each moves both arms without the tube to that arm's frozen training position from config/training_poses.json and refuses if the live calibration/timeline no longer reproduce those ticks
- [ ] #2 After the owner places the tube and confirms by typed answer, the holding gripper is locked only through the AA-17 GripperLockSM; the inserting gripper is never commanded
- [ ] #3 Before each episode the holding arm moves the tube (slow, mesh-checked with the tube) so the attachment point is rotated at most 2 deg and translated at most 20 mm per axis from the frozen pose; the inserting arm starts at its frozen training pose
- [ ] #4 A seeded 50-episode schedule per arm has exactly 10 nominal (zero-perturbation) and 40 perturbed entries; the requested and the achieved (FK) perturbation of each episode are saved with the dataset
- [ ] #5 After save/discard the inserting arm returns out of the key and to its frozen start, and the holding arm returns to the frozen holding pose before the next perturbation, verified in a dry run with simulated servos and the real cell mesh
- [ ] #6 Tests cover the schedule counts, the perturbation bounds, refusal on frozen-pose drift, and the never-both-unlocked rule
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 Acceptance criteria are each verified by concrete evidence (test output, measurement, or a logged hardware run) and not by code inspection or intent
- [ ] #2 Acceptance criteria and task description were NOT altered to match the implementation; any scope or criterion change was approved by the owner first and is recorded in the task
- [ ] #3 Hardware facts used came from doc-1; any corrected measurement was written back to doc-1 via `backlog doc update doc-1` in the same task
- [ ] #4 Tests pass via `uv run pytest` from robot/, including mock-backend tests that run with no hardware attached
- [ ] #5 Safety invariants still hold and are covered by a test: single-camera USB exclusivity, always-one-arm-holding, gripper servo commanded only by the AA-17 state machine, laser off on every exit path
- [ ] #6 Implementation notes record measured values, deviations from plan, and anything deferred
<!-- DOD:END -->
