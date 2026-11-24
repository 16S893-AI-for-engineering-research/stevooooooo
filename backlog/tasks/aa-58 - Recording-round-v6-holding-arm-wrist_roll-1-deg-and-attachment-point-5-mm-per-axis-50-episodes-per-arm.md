---
id: AA-58
title: >-
  Recording round v6: holding-arm wrist_roll +-1 deg and attachment point +-5 mm
  per axis, 50 episodes per arm
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 03:32'
updated_date: '2026-11-24 03:33'
labels:
  - recording
  - training
dependencies: []
priority: high
ordinal: 58000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner 2026-11-24: new round of insertion recordings to make the v5nom policies robust to small differences in tube placement (the left policy was unreliable on 2026-11-23, with the tube seating suspected). Per episode, servo 5 (wrist_roll) moves randomly by U[-1,+1] deg from its frozen position, and the attachment point moves by up to 5 mm in every direction. There are 50 episodes per arm, each with a slightly different rotation. Reuse AA-54 (inchworm/perturb.py, lerobot_env/record_train.py). AGENT ASSUMPTIONS TO CONFIRM WITH THE OWNER: servo 5 = the HOLDING arm's wrist_roll (the arm that moves the tube); translation U[-5,5] mm per x/y/z of the attachment point in the cell frame, realised by the holding arm; no nominal episodes by default (each test differs).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A selectable perturbation profile exists: holding-arm wrist_roll offset U[-1,+1] deg and attachment-point translation U[-5,+5] mm per axis; the AA-54 2 deg / 20 mm profile is unchanged and still the default for its launchers
- [ ] #2 A seeded 50-episode schedule per arm under the new profile; each episode has a different roll offset; requested and FK-achieved values are saved with the dataset, and the achieved values are inside the bounds
- [ ] #3 Every placement keeps the existing checks (soft/EEPROM limits, holding joint line mesh-clean with the tube carried, inserting arm reach); the grippers are commanded only through GripperLockSM
- [ ] #4 Tests cover the bounds, the per-episode variation, the counts and determinism for a seed; record_train --dry-run with the new profile plans all 50 placements per arm
- [ ] #5 First live recording owner-supervised; dataset path and outcome recorded in the notes
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Owner clarification 2026-11-24: each v6 perturbation is sampled UNIFORMLY at random across the whole allowed range (roll offset uniform in [-1,+1] deg; translation uniform in [-5,+5] mm per axis, independently), so some episodes land close to the actual position. Not only the maxima in each direction. Passed to the AA-58 sub-agent.
<!-- SECTION:NOTES:END -->
