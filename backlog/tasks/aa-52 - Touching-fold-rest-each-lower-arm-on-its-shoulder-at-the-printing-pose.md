---
id: AA-52
title: 'Touching fold: rest each lower arm on its shoulder at the printing pose'
status: To Do
assignee: []
created_date: '2026-11-13 03:10'
labels:
  - hardware
dependencies:
  - AA-50
references:
  - >-
    backlog/tasks/aa-50 -
    Full-cycle-movement-test-on-the-real-arms-replay-the-AA-45-inchworm-cycle-without-the-tube-grippers-moving.md
priority: medium
ordinal: 52000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner request 2026-11-13: the folded (printing) pose should have each lower arm resting on its own shoulder, so the arm is supported while printing and not held only by motor torque. Prepared: the model allows lower-arm/shoulder contact at any pose (mesh.always_allowed_pairs); the elbow EEPROM Max was raised +60 (L 3151, R 3106; backups in calibration_data). The replay still caps the fold elbow at L 3081 / R 3036 (cycle_replay.FOLD_ELBOW_CAP_TICKS). The hand-placed touching fold measured elbow L 3120-3121, R 3075-3076. OPEN OWNER DECISION before any code: how to press into the contact (proposed: drive the elbow slowly into contact at a capped torque, e.g. about 15%, like the gripper lock, with the owner confirming by eye).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The owner has decided how the elbow presses into the shoulder contact (force cap, speed, how it is confirmed), recorded in this task
- [ ] #2 At the start and end of the replay both lower arms rest on their shoulders as decided, shown in a logged live run with the owner's observation
- [ ] #3 Contact is limited as decided (capped torque or position) and the run log shows elbow load and current at the fold
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
