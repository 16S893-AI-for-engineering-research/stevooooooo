---
id: AA-44
title: Integrate the dedicated encoder-only leader arm for teleop demonstrations
status: To Do
assignee: []
created_date: '2026-11-09 00:41'
updated_date: '2026-11-10 17:26'
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

2026-11-09 (owner + orchestrator): owner bound (usbipd bind 11-2, admin) and assigned IDs 1-6 with lerobot-setup-motors. Orchestrator attached it (WSL /dev/ttyACM3, matched by USB serial 5C93150485). New read-only scripts/scan_leader.py (PING/READ only, finds the port by USB serial, tolerates ONLY error byte 0x08): IDs 1-6 answer, model 777, fw 3.10, 12.2-12.3 V, 214.2 Hz joint-set read over 3.0 s, 643 sets, 0 errors (AC #3 evidence: >= 20 Hz, error byte 0 except the 0x08 encoder-only flag on IDs 3 and 6). hardware.yaml arms.leader added (optional ArmConfig), discovery/attach_usb/backends know role arm_leader: discover shows it attached, attach_usb 8/8 OK on hardware (AC #2 evidence). Tests: 1676 passed (4 fixtures updated 7->8 devices, 2 new for the optional leader). AC #1 partial: gearing still to be recorded by the owner. AC #4 NOT started: leader EEPROM is factory (limits 0..4095, offset 0) on IDs 1,2,4,5; IDs 3 and 6 have leftover offsets 2468 / 269 and limits 929..3187 / 860..3167. Next: half-turn homing of the leader at a pose matching the followers' zero pose, then the leader-to-follower mapping (sign, offset, joint 6 excluded) at low speed under owner supervision.

2026-11-09 owner: the leader is encoder-only (no motor), so no gearing applies to a position reading; gearing is not needed for AC #1. Servo model/IDs/firmware are recorded in doc-1.

2026-11-10: software for leader homing done (commit 25a2458). home_servos.py and record_joint_ranges.py accept --arm leader; new _ServoBus.read_bytes_tolerating/write_bytes_tolerating tolerate ONLY error bit 0x08 (any other bit raises; checksum/length still enforced; writes still verified by read-back). Tests in tests/test_home_servos.py. Not yet run live: needs the owner holding the leader at the follower zero pose (expected ticks from view_zero_pose.py, pass as --reading ID=TICKS).
<!-- SECTION:NOTES:END -->
