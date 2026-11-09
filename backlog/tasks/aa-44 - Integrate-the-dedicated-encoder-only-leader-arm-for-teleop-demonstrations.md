---
id: AA-44
title: Integrate the dedicated encoder-only leader arm for teleop demonstrations
status: To Do
assignee: []
created_date: '2026-11-09 00:41'
updated_date: '2026-11-09 01:21'
labels:
  - hardware
dependencies:
  - AA-43
priority: high
ordinal: 44000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner, 2026-11-09: a third arm exists that is a leader only, with encoder-only servos. It replaces cross-arm teleoperation (decision-1, where a torque-off follower acted as the leader) as the leader for demonstration capture (AA-23). Unknown so far: USB adapter serial, joint count/naming, gearing, and whether its servo IDs need assigning. See doc-1 s3.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 doc-1 records the leader arm's USB serial, joint map, servo model and gearing, from the owner or a read-only scan
- [ ] #2 hardware.yaml identifies the leader arm by USB serial and attach_usb.py/discover attach and find it, verified on hardware
- [ ] #3 A read-only scan reads all leader joints at >= 20 Hz with error byte 0, logged
- [ ] #4 Leader-to-follower joint mapping (sign, offset, joint 6 excluded) is verified at low speed under owner supervision, logged
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
2026-11-09: leader driver board = CH343 1a86:55d3 serial 5C93150485 (COM8, BusId 11-2), not yet bound for WSL (usbipd bind needs Windows admin).
<!-- SECTION:NOTES:END -->
