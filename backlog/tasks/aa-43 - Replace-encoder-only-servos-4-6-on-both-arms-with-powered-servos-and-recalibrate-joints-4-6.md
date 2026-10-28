---
id: AA-43
title: >-
  Replace encoder-only servos 4-6 on both arms with powered servos and
  recalibrate joints 4-6
status: To Do
assignee: []
created_date: '2026-10-28 22:45'
updated_date: '2026-10-28 22:45'
labels:
  - hardware
dependencies: []
priority: high
type: task
ordinal: 43000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner finding 2026-10-27: servos ID 4-6 (wrist_flex, wrist_roll, gripper) on both SO-101 arms are encoder-only leader-arm servos with no motor drive (doc-1 section 3). Joints 4-6 therefore cannot be driven, which blocks every powered wrist or gripper motion (AA-17 lock, AA-41 probe, AA-19 fold, AA-21 dry cycle and everything downstream). The owner will buy 6 powered servos and rebuild both arms. Then everything stored per servo for joints 4-6 is invalid and must be redone: EEPROM homing offsets and position limits, the joint zero/sign calibration, the soft limits, and the gripper position bands. The new servo model and gearing must be recorded in doc-1.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 doc-1 records the replacement servo model, gearing and firmware for IDs 4-6 on both arms, from the owner or the part listing
- [ ] #2 All 12 servos answer with error byte 0, plausible temperature, and near-zero present current at rest with torque off, recorded by a read-only scan
- [ ] #3 IDs 4-6 on both arms have homing offsets and Min/Max position limits re-measured and written with an EEPROM backup and read-back, owner-supervised
- [ ] #4 servo_calibration, soft limits and gripper bands for joints 4-6 are re-measured on hardware and updated in hardware.yaml with their source
- [ ] #5 A supervised small powered move of each of IDs 4-6 on both arms reaches its goal within tolerance, logged
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
