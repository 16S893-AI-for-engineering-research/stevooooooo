---
id: AA-44
title: Integrate the dedicated encoder-only leader arm for teleop demonstrations
status: To Do
assignee: []
created_date: '2026-11-09 00:41'
updated_date: '2026-11-15 04:29'
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

2026-11-13 owner decisions (teleop/leader):
- Leader calibration: software mapping, no EEPROM writes. Both arms are placed by hand in the same pose, at 2-3 poses; the script reads leader + follower (both read-only) and fits per-joint sign and offset for joints 1-5. Joint 6 is excluded.
- First live teleop tries WITHOUT the mesh collision check (owner: "I do not believe it will be an issue"). Scoped to the teleop path only, via an explicit per-run flag, logged. Joint/soft/EEPROM limits stay enforced. Measured cost if re-enabled: 2.2 ms median, 18 ms p95 per pose check (first_only).
- lerobot-calibrate may rewrite servo memory (owner OK). Agent note: on the leader that is harmless. On the followers it would CHANGE homing offsets and limits that hardware.yaml zeros, the fold and the AA-45 timeline depend on. Plan instead: generate the LeRobot calibration file from the current follower EEPROM, so LeRobot writes back identical values.
- Tube holding during demos: the other arm holds the tube, gripper LOCKED and torque on (tube_support_source = locked_arm).

2026-11-13 owner: (1) handoff point confirmed = end of each regrip "move under target" (left: timeline frame 42), torque on in both arms, other arm LOCKED; code locks + re-plans from actual pose after. (2) Train the LEFT arm first (follower=left, leader=dedicated leader arm).

2026-11-13: scripts/leader_mapping.py (capture/fit/watch; read-only: leader READ tolerating only 0x08, follower refuses torque ON, no writes; fit sign+offset mod 4096 per joint 1-5, gripper excluded; needs >=15 deg leader span per joint and residual <=5 deg, saves calibration_data/leader_map_<arm>.json unverified:true). Tests tests/test_leader_mapping.py (9). Live probe not run yet: the arm ports were held by a running replay_cycle.py (AA-52 command, started 19:05 local).

2026-11-15 02:47Z: leader->left fit saved (calibration_data/leader_map_left.json, unverified:true) from owner hand-matched poses P1,P2,P3 (latest per label). All signs +1. Offsets ticks: pan 22.3, lift 2177.0, elbow 3083.7, wrist_flex 3741.7, wrist_roll 1443.0. Worst residual deg: 2.61, 1.49, 2.61, 2.00, 4.04 (tol 5). Leader span deg: 68.6, 93.4, 179.1, 91.9, 88.3. AC #4 still open: needs the supervised low-speed teleop check.

2026-11-15: src/inchworm/teleop_live.py (core: leader read 0x08-tolerant, software map, no-seam representative, clamp to soft limits ∩ EEPROM-10, ONE sync-write joints 1-5 with capped Goal_Speed, gripper never written, 10 deg start gate) + scripts/teleop_live.py (dry-run default; --check read-only; --live needs --no-collision-check reason and --allow-unverified-soft-limits approval; CONFIRM YES; Ctrl-C/stop file = HOLD then owner release; joints 1-5 torque-off sweep on exit; log calibration_data/teleop_live_<date>.jsonl). Tests tests/test_teleop_live.py (10, simulated servos). Read-only --check on hardware 03:11Z: refused at the start gate, wrist_roll leader -12.2 deg from the follower (others within 10). Observed: hardware.yaml servo_tick_limits shoulder_pan/lift differ from the live EEPROM on both arms (left pan yaml 674..2527 vs EEPROM 672..3340; right pan 1191..3044 vs 788..3447); teleop uses the live EEPROM intersected with the soft limits. Not yet: LeRobot plugins (next), live run (owner supervised).

2026-11-15 03:39Z first --live attempt: energised, then TypeError (run_loop() got unexpected keyword 'log') before any teleop step; the error path HELD (present positions, no motion) and the owner released torque at 03:40:05Z. Fixed; new end-to-end test drives the real --live path of scripts/teleop_live.py against simulated servos (follows leader, stop file, released, gripper untouched).

2026-11-15 04:13-04:14Z: FIRST LIVE TELEOP of the left follower from the leader, 51 s, owner supervised. Owner: 'it works well', 'a little jittery'. Log teleop_live_2026-11-14.jsonl: 28.2-29.8 Hz (median 29.7), 3 overruns total. Median per-second worst lag 2-6 ticks; peaks: wrist_flex 252, elbow 94. Clamps: shoulder_lift 407 and wrist_flex 572 steps at the limit (held at the soft/tick limit, as designed). Ctrl-C hold, released 04:14:18Z. Jitter fix (not yet run live): leader deadband 1 tick + exponential filter alpha 0.35 (--smooth, 1 = off); Goal_Speed per joint from the remaining distance (1.5 x step per period, capped at --speed) instead of a fixed 150; acceleration 100 while following (same as the AA-50 fluid replay). Simulated tracking of a 0.3 Hz +/-100 tick sweep: worst error 2.0 deg unfiltered vs 2.7 deg filtered. AC #4 not checked yet: the owner has not stated the per-joint direction check.
<!-- SECTION:NOTES:END -->
