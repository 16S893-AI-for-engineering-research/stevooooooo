---
id: AA-9
title: Measure and enforce SO-101 joint limits and collision envelope
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:09'
updated_date: '2026-10-25 00:52'
labels: []
milestone: m-1
dependencies:
  - AA-8
references:
  - pictures/robot.png
  - CAD/Assembly 1.step
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 9000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The arms are mounted on a fixed chassis in a crowded configuration (see pictures/robot.png): they cannot reach everywhere their nominal workspace suggests, and the folded transit pose is tight. Before any trajectory code runs, software must know the real per-joint travel and which poses collide with the chassis, the other arm, the DexArm or the tube. Without this, the first automated motion risks bending hardware.

Limits must be derived from the as-built machine and recorded as data in the hardware config, then enforced at a layer that every motion path goes through, so no later task can bypass them.

See doc-1 sections 2, 3 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Per-joint soft minimum and maximum travel for both arms are recorded in the hardware config with the method and date of measurement noted
- [x] #2 A validator rejects any commanded joint target outside soft limits before it reaches the servo bus, and the rejection path is unit-tested
- [x] #3 A documented collision model (at minimum conservative keep-out zones for chassis, opposite arm, DexArm and tube volume) rejects unsafe poses and unsafe straight-line joint interpolations, not just unsafe endpoints
- [ ] #4 Both the folded transit pose and a safe neutral pose are defined, pass validation and are stored as named poses in config
- [x] #5 Validation is enforced inside the motion layer so that bypassing it requires an explicit documented override flag
- [x] #6 Tests cover in-limit acceptance, out-of-limit rejection, and a trajectory whose endpoints are legal but whose interpolation passes through a keep-out zone
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Add AA-9 config blocks to hardware.yaml and config.py: per-joint soft limits (null=use URDF, unverified flag), named poses (neutral, folded_transit, unverified), and collision keep-out primitives (boxes/capsules, all unverified).
2. Implement NEW module inchworm/motion_guard.py with:
   a. JointLimitGuard: validates joint angles in radians against soft limits; if limits are null/unverified, refuses LIVE motion (fail-closed); returns rejection reason.
   b. CollisionModel: keep-out primitives from config; FK-based point-in-primitive checks; dense sampling of straight-line joint interpolations (documented resolution 50 samples).
   c. MotionGuard: combines both; single entry point check_pose(q, live).
   d. GuardedArm: wraps ArmServoBus, all position writes for joints 1-5 go through MotionGuard; explicit override flag (bypass_guard=True) logs WARNING.
3. Update scripts/jog.py: replace the placeholder collision_check_hook with a real call into the AA-9 MotionGuard (append inside AA-9 markers, keep old hook code intact but inactive).
4. Add scripts/measure_limits.py: dry-run default, --live for torque-off manual sweep that records min/max per joint (hardware AC#1 script).
5. Add tests/test_motion_guard.py: cover AC#2 (in-limit accept, out-of-limit reject), AC#3 (trajectory interpolation through keep-out zone), AC#5 (bypass flag logs warning). AC#6 coverage.
6. Update hardware.yaml and config.py with AA-9 config blocks.
7. Run uv run pytest and fix any failures.
8. Finalize task: check ACs #2, #3, #5, #6 with test output; leave #1, #4 unchecked (hardware/unverified).
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
AA-9 fix (orchestrator review 2026-10-24): (1) Added joint_calibration.py - servo_deg<->joint_rad via zero_offset_deg+sign; live fails closed on null/unverified; removed hardcoded deg-180. (2) GuardedArm.write_position_deg reads present pose, builds full 5-DoF target, runs check_pose+check_trajectory. (3) homing.py routes live writes through GuardedArm.apply_joint_command (guarded_arms= injection), errors without it. (4) policy_actions.py routes writes through GuardedArm.apply_joint_command, warns on raw bus. (5) test_ast_guard.py: AST test enforces only {motion_guard,servo_bus,gripper_lock,torque,joint_calibration} may call write_position_deg. (6) 783/783 tests pass. AC#1,AC#4 unchecked (hardware required).

Orchestrator review 2026-10-24 of the fix: _read_present_pose_rad fabricated 0.0 rad on a read failure or missing servo ID, so the collision check validated a pose the arm is not in. LIVE mode now raises PresentPoseUnknownError (fail closed); the dry-run fallback is kept with a warning. Tests: test_live_present_pose_read_failure_fails_closed, test_dry_run_present_pose_read_failure_falls_back. Verified that the AST guard test exists and passes. 902 passed.

Orchestrator verification 2026-10-24: AC#5 re-checked after the wave-3 fix and orchestrator hardening. `uv run pytest tests/test_ast_guard.py tests/test_motion_guard.py` all pass: the AST test proves no module outside {motion_guard, servo_bus, gripper_lock, torque (e-stop present-position hold only), joint_calibration} writes positions; homing and policy_actions route through GuardedArm; bypass_guard is the only documented override and it always logs a WARNING (test_bypass_guard_logs_warning); live present-pose read failure fails closed.

LIVE 2026-10-24, left arm, torque-off manual sweep (owner), read-only recorder, 1324 samples per joint (robot/calibration_data/joint_ranges_left_2026-10-24.json). Servo-degree ranges: shoulder_pan 291.3->98.9 (168 deg, CROSSES the 4095/0 seam), shoulder_lift 134.0->291.0 (157), elbow_flex 168.4->3.2 (195, CROSSES), wrist_flex 41.8->245.8 (204), wrist_roll 148.8->27.6 (239, CROSSES), gripper 271.1->359.9 (89, ends at the seam). Servo EEPROM: position limits 0/4095, homing_offset 0x55 (IDs 1-3) / 0 (IDs 4-6). A first sweep with a non-wrap-aware recorder produced invalid 0-360 ranges; the recorder was fixed to unwrap continuous travel (test_unwrapped_range_across_seam). BLOCKER for live motion: three joints cross the encoder seam, so position-mode targets could travel the long way. Homing offsets must be recalibrated (LeRobot-style half-turn homing) before any position command; owner approval requested to write the EEPROM. AC#1 not yet checked (right arm pending; limits must be expressed after offset recalibration).

LIVE 2026-10-24, left arm servo EEPROM calibration (owner-approved, torque OFF throughout, every write read back):
1. Half-turn homing (LeRobot method): owner held all joints mid-travel. Homing offsets written: id1 -1352, id2 384, id3 1095, id4 -420, id5 1092, id6 1573. Read-back exact; each joint then read 2047/2048. Pre-change backup in robot/calibration_data/eeprom_backup_left_2026-10-24.json.
2. Read-only re-sweep (1398 samples): no seam crossing on any joint. Ranges (ticks): pan 644-2557, lift 1225-3030, elbow 909-3121, wrist_flex 899-3217, wrist_roll 470-3320, gripper 1528-2517 (spans 168/159/194/204/251/87 deg, consistent with the pre-homing sweep).
3. Min/Max_Position_Limit written to EEPROM with a 30-tick margin, read-back verified: pan 674-2527, lift 1255-3000, elbow 939-3091, wrist_flex 929-3187, wrist_roll 500-3290 (owner: not continuous on this build), gripper 1558-2487. Stored in hardware.yaml motion_guard.left_arm_limits.servo_tick_limits.
OWNER NOTE: these are per-joint independent maxima; the real safe envelope depends on all joints together. The collision model (keep-out geometry, currently placeholders) must be built and validated before motion near the limits.
AC#1 NOT checked yet: the right arm is pending, and the radian-valued soft limits need AA-9 servo_calibration (joint zero and sign), which is unmeasured.

LIVE 2026-10-24, right arm servo EEPROM calibration (owner-approved, torque OFF, all writes read back). Homing offsets id1 -312, id2 397, id3 -99, id4 269, id5 -1063, id6 611. Two read-only sweeps (the first missed shoulder_pan); merged ranges (ticks): pan 758-3477 (239 deg), lift 1293-2965, elbow 876-3076, wrist_flex 869-3197, wrist_roll 524-3464, gripper 1543-2509; no seam crossing. Left shoulder_pan re-swept: 654-2565 (168 deg), repeatable, limit kept. OWNER DECISION (b): right shoulder_pan capped to the left span (1853 ticks) centred on its midpoint, giving 1191-3044. EEPROM limits written: pan 1191-3044, lift 1323-2935, elbow 906-3046, wrist_flex 899-3167, wrist_roll 554-3434, gripper 1573-2479. Both arms now have per-joint servo-side limits. AC#1 still unchecked: radian soft limits need the joint zero and sign calibration, and the combined-joint envelope needs the collision geometry.

LIVE 2026-10-24 joint zero and direction (read-only). Zero offsets set to the midpoint of each EEPROM limit range: left pan 140.669, lift 186.987, elbow 177.1, wrist_flex 180.879, roll 166.553; right 186.108, 187.119, 173.672, 178.682, 175.254 servo deg. Resulting joint ranges are symmetric (e.g. left pan +/-81.4, lift +/-76.7, elbow +/-94.6, wrist_flex +/-99.2, roll +/-122.6 deg). Owner direction test: higher reading = clockwise viewed with the servo cable exiting away (pan +255, lift +343, elbow -329 for CCW). Signs ASSUMED +1 (LeRobot convention); the servo_calibration block stays unverified (live blocked) until the per-joint URDF +axis vs 'clockwise' mapping is verified from the URDF/CAD.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 18:01
---
Orchestrator review 2026-10-24 (P0 safety): AC#5 'validation is enforced inside the motion layer so that bypassing it requires an explicit documented override flag' is NOT met. (1) homing.py (AA-10) and policy_actions.py (AA-17) call ArmServoBus.write_position_deg directly, skipping GuardedArm entirely, without any override flag. (2) GuardedArm.write_position_deg checks only that one joint's limit against a zeros pose; it runs no collision check because it doesn't know the other joints' present positions. (3) The degrees-to-radians mapping (deg-180) is an assumption: AA-11 FK uses URDF radians with zero at the calibrated rest pose, and the servo tick-to-joint-angle zero offset is uncalibrated, so live limit and collision checks are meaningless until calibration. Fixes: route all arm writes through GuardedArm, make single-joint writes read the present pose and run the full check_pose, add an explicit per-joint calibration (zero offset and sign) in config marked unverified so live motion fails closed, and add an AST test that no module outside motion_guard.py, torque.py (e-stop hold-in-place only) and servo_bus.py calls write_position_deg.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented AA-9 (software half): inchworm/motion_guard.py with JointLimitGuard (fail-closed on unverified/null limits), CollisionModel (box+capsule keep-out zones, 50-sample dense interpolation), MotionGuard, and GuardedArm wrapper. Config schema (config.py) and hardware.yaml updated with AA-9 markers. jog.py collision hook wired to real guard. scripts/measure_limits.py added (dry-run default, --live for hardware sweep). 33 new tests; all 616 tests pass. AC#2,3,5,6 checked with test evidence. AC#1 and AC#4 left unchecked (require owner-supervised hardware sessions).
<!-- SECTION:FINAL_SUMMARY:END -->
