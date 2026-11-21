---
id: AA-55
title: >-
  Two-policy crawl sequence: left policy, lock, right out, left to train_right,
  right policy, lock, home
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-21 01:55'
updated_date: '2026-11-21 03:38'
labels: []
milestone: m-3
dependencies:
  - AA-54
priority: high
type: feature
ordinal: 55000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner plan 2026-11-21: new full sequence with two per-arm policies (trained per AA-54 from the frozen positions in robot/config/training_poses.json). Sequence: (1) scripted cycle frames 0..42 as today (right LOCKED holding, left open at train_left). (2) LEFT policy (left checkpoint) inserts. (3) Left gripper locks (AA-17) + owner look y/n. (4) Right gripper unlocks (refused unless left LOCKED) and the right arm un-inserts (straight out of the key, then clear). (5) The left arm, carrying the tube, moves to its holding pose of train_right (the right arm is not inserted, so no bad flex). (6) The right arm moves to its train_right start pose. (7) RIGHT policy (right checkpoint, no joint-offset mapping) inserts. (8) Right gripper locks + owner look. (9) Both arms go back to home (the cycle's start/printing pose, both LOCKED). Replaces the AA-39 full_policy_test flow (demo-end pose + resumed frames 49..67).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 One launcher runs the 9-step sequence with dry-run first (simulated servos, real cell mesh, both checkpoints) and live mode; every motion phase is preflighted and mesh-checked with the tube before its typed YES
- [ ] #2 Each policy phase starts only if both arms read within tolerance of that arm's frozen training position; otherwise it refuses without moving
- [ ] #3 Gripper lock/unlock goes only through GripperLockSM; the right unlock is refused unless the left reads LOCKED and the owner confirmed it by eye; never both unlocked, verified by a test
- [ ] #4 The left arm's move to its train_right holding pose happens only after the right gripper is unlocked and the right arm is out of the key, verified by a test of the phase order
- [ ] #5 The end phase moves both arms (both LOCKED) to the cycle home pose; any stop holds both arms and gives the usual release prompt (joints off, grippers untouched)
- [ ] #6 The first live run is owner-supervised and its log path and outcome are recorded in the task notes
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
2026-11-21 session 1: robot/src/inchworm/two_policy.py (phase list PHASES, training_pose_problems, lock_pose_problem, CarriedTube, tube_offset, joint_line, straight_out, plan_right_out, plan_left_to_train_right, plan_right_to_train_right, plan_home_timeline/plan_home_line, frames_problems, fluid_problems) + tests/test_two_policy.py (9 tests pass), committed. Owner/supervisor decisions: step-5 lock check = 8 deg vs a --left-lock-ref flag defaulting to full_policy_test DEMO_END_LEFT_TICKS (later: the retrained left policy's demo-end median); tube rigidly carried by the left from its ACTUAL lock pose, that carried pose goes in the train_right handoff state, only LOG its offset from the frozen train_right tube pose. NOT done: lerobot_env/crawl_two_policy.py + .sh (reuse full_policy_test.py policy_phase/lock_and_look/hold_and_release/open_live_buses/load_policy by import), lerobot_env/tests/test_crawl_two_policy.py, dry run. Note: tests/test_constants.py forbids the bare literal 67 in src (it is a design value) -> take the train_right frame from training_poses.position('train_right')['timeline_frame'].
<!-- SECTION:NOTES:END -->
