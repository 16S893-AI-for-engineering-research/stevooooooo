---
id: AA-58
title: >-
  Recording round v6: holding-arm wrist_roll +-1 deg and attachment point +-5 mm
  per axis, 50 episodes per arm
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-24 03:32'
updated_date: '2026-11-24 04:25'
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

OWNER DECISION 2026-11-24 (coupling question): option A. Servo 5 (holding-arm wrist_roll) turns exactly frozen + r, r uniform in [-1,+1] deg; the tube is translated by t, uniform in [-5,+5] mm per axis, realised by the other holding joints (translation = the fit target). Requested and FK-achieved values are saved; the achieved x is expected to deviate from the requested x because of the roll/x coupling (6.4 mm/deg left inserting, 3.2 mm/deg right). Not option C (all joints free; servo 5 would turn up to ~17 deg).

2026-11-24 sub-agent result (reviewed by the orchestrator, applied to main): profile v6 via record_train.py --profile v6 (+ launchers). Roll offset U[-1,1] deg and translation U[-5,5] mm/axis drawn uniformly; servo 5 = frozen + tick-rounded offset (held); the other 4 holding joints are Gauss-Newton fitted to P0 + t. Kept only if the FK roll is within 1 deg, each axis within 5 mm, plus every AA-54 check (limits, reach, mesh-clean holding line). AA-54 default unchanged. Tests: test_perturb 13 passed, test_record_train 17 passed. Dry runs seed 58: 50/50 planned per arm, 53 tries (3 rejections by the checks), plan 53 s left / 27 s right, max reach residual 0.70 / 0.68 mm. Achieved |value| median/max: left roll servo 0.53/0.97 deg, FK 0.26/0.70; x 1.71/4.49 mm; y 2.60/4.95; z 2.39/4.73. Right roll servo 0.48/0.97, FK 0.40/0.97; x 1.46/3.08; y 2.38/4.96; z 2.62/4.93. Coupling: right achieved x is set by the roll (corr 1.00 with the servo offset, 0.003 with the requested x); left corr(requested x, achieved x) 0.71. Servo resolution 0.088 deg/tick -> 19 distinct roll offsets in 50 episodes. Open: confirm 0 nominal episodes; first live recording owner-supervised (AC #5).

OWNER 2026-11-24: (1) accepts that on the right arm (holding = left) the achieved x is set by the roll only (coupling, max ~3.1 mm); (2) all 50 episodes perturbed, 0 nominal (the v6 default as built).
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @agent
created: 2026-11-24 03:44
---
Sub-agent found a kinematic coupling: on the 5-joint holding arm, 1 deg of wrist_roll also moves the attachment point sideways (x) by 6.4 mm (left inserting) / 3.2 mm (right inserting). So roll +-1 deg and x +-5 mm cannot both be independent and uniform. Options: (A, provisional) servo 5 = frozen + r exactly, other joints fitted, achieved x tied to roll; (B) reject mismatches (biased, rejected); (C) all 5 joints free: tube rotation and translation exact, but servo 5 moves more than 1 deg (up to ~17 deg on the left). Owner to choose.
---
<!-- COMMENTS:END -->
