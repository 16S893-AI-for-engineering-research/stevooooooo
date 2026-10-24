---
id: AA-8
title: SO-101 servo bus driver and per-arm servo ID map
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:09'
updated_date: '2026-10-24 16:26'
labels: []
milestone: m-1
dependencies:
  - AA-5
  - AA-7
references:
  - pictures/robot.png
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 8000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner has assembled both SO-101 arms and assigned servo IDs, but nothing has been commanded in software yet. Every later task (homing, IK, crawl, policy rollout) depends on a trustworthy low-level channel to the Feetech serial-bus servos, and on knowing which ID is which joint on which arm. Getting this wrong means commanding the wrong joint on the wrong arm.

Treat this as the lowest control layer: read and write position, read current/load/temperature/voltage, set torque enable, and nothing about kinematics. The custom gripper shares one servo for jaw and lock (doc-1 section 3), so the gripper servo must be readable at high enough rate to later detect seating and latch hold.

See doc-1 sections 3, 7 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A driver opens either arm bus by logical role and performs ping, read position, read current/load, read temperature/voltage, write goal position and torque enable/disable on an addressed servo
- [ ] #2 A discovery routine scans the bus and reports every responding servo ID with model and current position, and the resulting per-arm joint-to-ID map is persisted in the hardware config rather than hardcoded
- [x] #3 Driver surfaces checksum errors, timeouts and out-of-range responses as distinct typed errors and never returns a silently stale or partial reading
- [ ] #4 Readback rate sufficient for closed-loop supervision (at least 20 Hz for a full joint state read on one arm) is measured and recorded in the task notes
- [x] #5 All position values are exposed in degrees at the driver interface with raw-tick conversion isolated in one place
- [x] #6 A mock servo bus backend supports the full interface so higher layers are testable with no hardware, and tests cover both the mock and the error-mapping paths
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Fix sign-magnitude decoding for present_speed (bit15=sign) and present_load (bit10=sign) in read_joint_state and read_all_joint_states. Add decode helper _decode_sign_magnitude.
2. Add _REG_PRESENT_CURRENT = 0x45 and read it in read_joint_state (separate 2-byte read). Add current_raw field to JointState.
3. Fix docstring register table to match actual register addresses (verified against rustypot STS3215 source); cite rustypot.
4. Create inchworm/gripper_access.py with module-private _GRIPPER_TOKEN sentinel. Add GripperChannel class (write_position_deg method, dry_run support). Remove _write_gripper_position_deg from ArmServoBus. Add ArmServoBus.gripper_channel(token) that accepts only _GRIPPER_TOKEN.
5. Enforce gripper ID rejection in sync-write paths (write_position_deg and any sync variants). Keep existing GripperWriteNotAllowed for the public write_position_deg.
6. Fix test_checksum_error_is_distinct: assert directly on the raised error, remove dead comment.
7. Add tests: sign-magnitude decode with known raw values (load 0x0400|100=-100, 100=100; speed 0x8000|50=-50). Add test for present_current read. Add test for GripperChannel capability object. Add test that scans src/inchworm for token misuse.
8. Run uv run pytest -q, paste output into task notes.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete. Files added: robot/src/inchworm/servo_bus.py (ArmServoBus driver, _ServoBus framing, typed errors, degrees<->ticks conversion isolated in ticks_to_degrees/degrees_to_ticks, gripper restricted to _write_gripper_position_deg), robot/src/inchworm/servo_discovery.py (scan_bus_ids + scan_and_persist), robot/scripts/scan_servos.py (dry-run by default, --live flag for hardware, --benchmark for AC4). Config extended: hardware.yaml servo_joints stanzas added under arms.left and arms.right (all null pending live scan), config.py ServoJointMap model added to ArmConfig. 52 new tests in robot/tests/test_servo_bus.py. Full suite: 304 passed.

Audit run (post-implementation): uv run pytest -q => 324 passed in 3.30s. Sign-magnitude tests: TestSignMagnitudeDecode (11 tests) all pass including test_load_negative (0x0400|100 -> -100) and test_speed_negative (0x8000|50 -> -50). test_two_complement_would_differ confirms old struct '<h' gave wrong answer. present_current read: test_current_raw_returned and test_current_raw_zero both pass. Gripper restriction: TestGripperCapability (9 tests) all pass including test_no_token_import_outside_aa17 scan. test_checksum_error_is_distinct: direct asserts on .servo_id, .expected, .actual with no dead comment. Register addresses in docstring match code constants and rustypot (torque_enable 0x28=40, goal_position 0x2A=42, lock 0x37=55, present_position 0x38=56, speed 0x3A=58, load 0x3C=60, voltage 0x3E=62, temp 0x3F=63, current 0x45=69). No hardware I/O in tests (all use MockSerialTransport or MockBackend). AC1 re-verified: ping, read position, read current/load, read temp/voltage, write goal position, torque enable/disable all covered. AC3 re-verified: ServoChecksumError, ServoTimeoutError, ServoRangeError, ServoStatusError all distinct; no silent partial reads. AC5 re-verified: degrees at public interface, ticks isolated in ticks_to_degrees/degrees_to_ticks. AC6 re-verified: MockBackend tests pass. ACs 2 and 4 remain unchecked (hardware only).
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @agent
created: 2026-10-24 16:12
---
AC2 (partial): discovery routine code and config schema are implemented and tested against the mock backend. The persisted joint-to-ID map cannot be verified until a live bus scan confirms which IDs respond on each arm. needs owner-supervised hardware session: run 'uv run python scripts/scan_servos.py --arm left --live --persist' and 'uv run python scripts/scan_servos.py --arm right --live --persist', then assign joint names in hardware.yaml.
---

author: @agent
created: 2026-10-24 16:12
---
AC4 (readback rate): needs owner-supervised hardware session: run 'uv run python scripts/scan_servos.py --arm left --live --benchmark' after servos are connected. The script performs 20 iterations of read_joint_state and prints the achieved Hz, reporting pass/fail against the 20 Hz threshold.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented ArmServoBus (Feetech SCS protocol: ping, read/write registers, sync_read), degrees-at-interface with ticks isolated in ticks_to_degrees/degrees_to_ticks, typed errors (ServoError, ServoTimeoutError, ServoChecksumError, ServoRangeError, ServoStatusError, GripperWriteNotAllowed), gripper restricted behind _write_gripper_position_deg (raises GripperWriteNotAllowed for general callers), MockBackend integration, servo_discovery.py scan_and_persist. Config extended with ServoJointMap schema and null servo_joints stanzas in hardware.yaml. scan_servos.py script: dry-run by default, --live for hardware, --benchmark for AC4 Hz measurement. 52 new tests, 304 total passed. ACs 1/3/5/6 checked via test output. ACs 2 and 4 unchecked: need owner-supervised hardware session (see task comments).
<!-- SECTION:FINAL_SUMMARY:END -->
