---
id: AA-43
title: >-
  Replace encoder-only servos 4-6 on both arms with powered servos and
  recalibrate joints 4-6
status: In Progress
assignee:
  - '@orchestrator'
created_date: '2026-10-28 22:45'
updated_date: '2026-11-10 01:36'
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
- [x] #1 doc-1 records the replacement servo model, gearing and firmware for IDs 4-6 on both arms, from the owner or the part listing
- [x] #2 All 12 servos answer with error byte 0, plausible temperature, and near-zero present current at rest with torque off, recorded by a read-only scan
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-09: owner rebuilt both arms; all joints now have powered servos (per owner). Servo IDs are being re-assigned by the owner in their own terminal with lerobot-setup-motors (so101_follower), one servo connected at a time. lerobot_env now depends on lerobot[feetech] (scservo_sdk was missing). A read-only census before assignment found 0 servos on both buses (servos disconnected for assignment, expected). Next: owner gives the servo model/gearing/firmware (AC #1); then the read-only scan (AC #2).

2026-11-09 read-only census (calibration_data/servo_census_2026-11-08T20-15-27.json, script: PING 0-252 + register dump 0-70, no writes): both arms answer IDs 1-6 only, all model 777 fw 3.9, error byte 0 (any non-zero raises ServoStatusError), temp 26-30 C, present current 0, voltage 12.1-12.3 V, torque off, lock 1. Owner: replacement servos are the same model as the first set (STS3215 1/345). AC#1 + #2 evidence. IDs 1-3 EEPROM identical to the 2026-10-27 backups (offsets/limits); IDs 4-6 factory: offset 85, limits 0..4095, Max_Torque 1000, P/D 32, prot_current 310, unload 44 (gripper caps of 500/250/25 not applied). Added scripts/home_servos.py (homing + limits, dry-run default, torque-off required, backup/read-back) with tests/test_home_servos.py.

2026-11-09 paused by owner: safe state verified (torque_enable 0 read back on all 12 servos, laser relay OFF commanded, no robot processes). Open owner questions: servos 1-3 re-horned? re-apply gripper caps? Next: home_servos.py homing per arm (see doc-3).

2026-11-09 (owner session, live, torque off, nothing moved): owner answers: IDs 1-3 on both arms are unchanged (no re-homing needed); IDs 4-5 limits still to be measured; grippers cannot be swept by hand (powered servo too stiff), both rest at their full CLOCKWISE limit = the old closed_empty stop (clockwise limit short of entering the attachment point); owner: assume the previous gripper range and restore LeRobot's lower gripper torque settings. DONE (backups calibration_data/eeprom_backup_{left,right}_2026-11-09T19-21-4x_pre_gripper_caps.json, ...19-22-xx_pre_homing/pre_limits.json; whole-bus read-back, only ID 6 registers changed): (1) gripper caps via new restore_drive_config.py --gripper-caps-only: ID 6 Max_Torque_Limit 1000->500, Protection_Current 310->250, Overload_Torque 80->25, P 32->16 (D 32), both arms. (2) gripper homing offset so the closed stop reads the OLD closed_empty tick: left offset 1207 (raw 3728 -> reads 2521), right offset -1914 (raw 580 -> reads 2494), via new home_servos.py homing --reading ID=TICKS. (3) gripper Min/Max_Position_Limit = the previously written values: left 1521..2531, right 1563..2504 (home_servos.py limits --set-limits). ASSUMPTION (not measured): same offset/range as the old servo, i.e. the open end is at ~old open (L 1531, R 1576), so the existing hardware.yaml gripper bands (open/locked/closed_empty) stay valid in the new frame. Verify the open end with one supervised powered move before relying on the bands. Torque comparison with LeRobot recorded in doc-3. Tests: 3 new in test_home_servos.py, 1 in test_restore_drive_config.py. AC #3 still open for IDs 4-5 (both arms): homing at mid-travel + sweep + limits.

2026-11-09 (owner session, live, torque off): IDs 4 (wrist_flex) and 5 (wrist_roll) homed at mid-travel by the owner and swept, both arms. Sweeps clean (no wrap/seam): calibration_data/joint_ranges_{left,right}_ids45_2026-11-09.json. EEPROM Min/Max_Position_Limit written with margin 30 and read back (backups calibration_data/eeprom_backup_*_2026-11-09T19-35-47/19-38-14_pre_limits.json): left wrist_flex 831..3226 (offset 2673), wrist_roll 550..3573 (offset 1595); right wrist_flex 797..3191 (offset 1976), wrist_roll 533..3577 (offset 1019). Spans (205 deg flex, 260-262 deg roll) match the old servos. hardware.yaml updated (servo_tick_limits all 6 joints both arms; servo_calibration wrist_flex/wrist_roll zeros = mid-range of the new limits: L 178.286/181.187, R 175.254/180.615; wrist_roll soft limits capped at +-2.2165 rad because the cell URDF only models -254..0 deg, test_roll_mapping enforces it; wrist_flex soft limits unchanged at the URDF +-1.6581). tests/test_cell_collision.py::_measured_fold now converts the 2026-10-25 fold ticks with the OLD wrist zeros (pinned constant, with a comment): those ticks were read in the old servos' frame, the model's meaning of the fold is unchanged; re-measure the fold on the new servos and drop the pin. NOT claimed: AC #3 (gripper limits are the owner-assumed previous values, not re-measured), AC #4 (signs of joints 4-5 and the gripper band open end are not re-verified after the rebuild), AC #5 (no powered move yet).

2026-11-09 pan check: IDs 1-3 EEPROM (homing offset, limits) on BOTH arms are byte-identical to the pre-rebuild dumps (eeprom_after_*_2026-10-27T23-32-*_fw310_mask.json), i.e. kept, not re-measured; the owner did not re-run the motions after the rebuild. Whether the horns/mounting moved is unproven. Pan landmark: ticks L 1972 / R 2063 (arm pointing at the DexArm, poses_2026-10-25.jsonl); today at rest L 1947 (-2.2 deg), R 2091 (+2.5 deg), but the arms are not placed at the landmark. Verify by hand-placing both arms pointing at the DexArm (read_pose.py, read-only) and comparing with 1972/2063, and by hand-placing the fold and comparing joints 1-3 with the folded_transit record (L 2009/1226/3115, R 2061/1293/3071). New scripts/powered_move_check.py (dry-run default; one joint, wrist_flex/wrist_roll +-5 deg out-and-back at 150 ticks/s, torque limit 300, or the gripper to its open band via GripperLockSM.reset(); everything else torque-off; stop file /tmp/inchworm_stop_move; stall/timeout/error abort to torque off); 5 tests. gripper_lock_probe.py now restores the EEPROM Max_Torque_Limit (500) in cleanup instead of a hard-coded 1000, which would have undone the LeRobot cap. Pre-move mesh check at the current rest pose: both arms are in the accepted-fold state; +-5 deg of wrist_flex/wrist_roll creates no new model contacts except left wrist_flex +5 (B_Fix vs left_wrist), so use -5 for left wrist_flex.
<!-- SECTION:NOTES:END -->
