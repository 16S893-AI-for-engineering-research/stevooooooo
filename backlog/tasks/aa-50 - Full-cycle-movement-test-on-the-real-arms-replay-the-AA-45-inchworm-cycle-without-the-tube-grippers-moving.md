---
id: AA-50
title: >-
  Full-cycle movement test on the real arms: replay the AA-45 inchworm cycle
  without the tube, grippers moving
status: In Progress
assignee:
  - '@orchestrator'
created_date: '2026-11-12 21:02'
updated_date: '2026-11-12 21:02'
labels:
  - hardware
dependencies:
  - AA-43
references:
  - AA-45
priority: high
ordinal: 50000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner request 2026-11-12: a full movement test of both arms, running the whole inchworm cycle the owner approved as an animation (AA-45), on the real robot with NO tube, and with the grippers unlocking and locking at the regrips as in the plan. The grippers should eventually not need to move this way once the learned model exists, but the owner wants to see that they do move here. This is the first live multi-joint, two-arm motion on the new servos, so it is dry-run first and owner-supervised, at low speed. Gates that block live motion today: the soft limits are marked unverified (owner sign-off needed to clear them for this no-tube replay), the start pose must match the plan's first frame, and the measured fold has the elbow about 30 ticks past the EEPROM limit (the replay opens it by about 3.5 deg).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A replay script plays out/inchworm/timeline.jsonl through GuardedArm (arms) and GripperLockSM (grippers), dry-run by default, with a stop file, hold-on-abort and torque release at the end
- [ ] #2 An offline preflight checks every frame-to-frame move with both arms together against the cell mesh model with no tube, the soft limits and the EEPROM tick limits, and refuses on any problem; it passes on the current timeline
- [ ] #3 A simulated-servo run of the full cycle passes and is covered by tests (gripper unlock/lock order, speed never full, obstruction aborts)
- [ ] #4 The owner approves clearing the soft-limit live gate for this no-tube replay, recorded with its scope
- [ ] #5 A supervised live replay of the full cycle without the tube completes, logged per frame (goal vs reached, load), with the owner's observations recorded
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

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. DONE src/inchworm/cycle_replay.py: load_timeline, gripper_events, frame_speeds (all joints arrive together, never Goal_Speed 0), q_to_ticks, clamp_to_eeprom, preflight, summary, ReplayRunner (energise, lock_both, move_to_start, run, hold, release), build_runner. scripts/replay_sim.py: SimArmBus + simulate (dry-run). tests/test_cycle_replay.py (6). 2. NEXT scripts/replay_cycle.py CLI: dry-run = preflight + simulate + summary; --live = attach both arms, refuse unless torque off; read the pose, start_pose_check vs frame 0 (3 deg); owner confirms no tube; energise; lock both (45 deg position lock); move_to_start; run with --pause-each-phase by default; stop file /tmp/inchworm_stop_move; on abort hold(), the owner supports the arms, then release(); log JSONL to calibration_data/replay_cycle_<date>.jsonl. Live needs the soft-limit gate: a --no-tube-replay owner sign-off, NOT a global unverified:false (ask the owner). 3. Owner-supervised human_request: place the arms in the fold, dry-run, live at 150 ticks/s.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-12 night (orchestrator): replay core written and tested (suite passes). Real timeline (87 frames): clamp_to_eeprom opens the fold elbow L 3120->3081, R 3076->3036 ticks (about -3.5 deg; the measured fold sits at the swept end stop, 30 ticks past the EEPROM limit); preflight with both arms interpolated together, no tube: 0 problems (85 s). Simulated full cycle: OK, all 87 frames reached within 20 ticks, grippers lock L 2027 / R 2031, unlock/relock order L(25,49) R(50,74), max speed 150 ticks/s, about 70 s simulated (41 s arm motion). Live guard path: GuardedArm(live=True) refuses while motion_guard.*_arm_limits.unverified is true (needs the owner's decision). The last measured fold is within 0.8 deg of frame 0 on joints 1-4; roll is -2.7/-2.8 deg off (inside the 3 deg start check).
<!-- SECTION:NOTES:END -->
