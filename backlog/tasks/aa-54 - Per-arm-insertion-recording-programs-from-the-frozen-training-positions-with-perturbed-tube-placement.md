---
id: AA-54
title: >-
  Per-arm insertion recording programs from the frozen training positions with
  perturbed tube placement
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-21 01:54'
updated_date: '2026-11-21 04:41'
labels: []
milestone: m-3
dependencies:
  - AA-23
priority: high
type: feature
ordinal: 54000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner plan 2026-11-21 (replaces the AA-39 full test flow). The left policy failed live from frame 42 and the old dataset was recorded at the old wrist_roll zero. The owner will retrain BOTH arms from two FROZEN training positions saved in robot/config/training_poses.json (train_left = cycle frame 42: right LOCKED holding the tube, left open under its point; train_right = frame 67: left LOCKED holding at its gait pose, right open under its point). Owner rule: these positions must not change after training unless the owner says so. One recording program per arm: it moves both arms (no tube) to that arm's training position; the owner adds the tube and puts the HOLDING gripper's key into its attachment point; code locks the holding gripper (AA-17) after the owner confirms. Then rapid-fire teleop recording as in record_handoff.py. Robustness: before each episode the HOLDING arm moves the tube so the attachment point in front of the inserting gripper is rotated up to 2 deg (any axis) and translated up to 20 mm (any of x/y/z) from the frozen pose; NOT a per-joint jitter. 50 episodes per arm: 10 at the exact frozen pose, 40 perturbed. The SO-101 has 5 joints, so not every 6-DOF perturbation is reachable: the achieved perturbation is what is recorded.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Two launchers exist, one per arm (left, right); each moves both arms without the tube to that arm's frozen training position from config/training_poses.json and refuses if the live calibration/timeline no longer reproduce those ticks
- [x] #2 After the owner places the tube and confirms by typed answer, the holding gripper is locked only through the AA-17 GripperLockSM; the inserting gripper is never commanded
- [ ] #3 Before each episode the holding arm moves the tube (slow, mesh-checked with the tube) so the attachment point is rotated at most 2 deg and translated at most 20 mm per axis from the frozen pose; the inserting arm starts at its frozen training pose
- [x] #4 A seeded 50-episode schedule per arm has exactly 10 nominal (zero-perturbation) and 40 perturbed entries; the requested and the achieved (FK) perturbation of each episode are saved with the dataset
- [ ] #5 After save/discard the inserting arm returns out of the key and to its frozen start, and the holding arm returns to the frozen holding pose before the next perturbation, verified in a dry run with simulated servos and the real cell mesh
- [x] #6 Tests cover the schedule counts, the perturbation bounds, refusal on frozen-pose drift, and the never-both-unlocked rule
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
2026-11-21 session 1: a sub-agent timed out with nothing written. Design to use: robot/src/inchworm/perturb.py: attachment point P = the inserting arm's <arm>_gripper link pose at its inserted frame (48 left / 73 right) in the cell frame; perturbation = rotation about P (angle U[0,2] deg, random axis) + translation U[-20,20] mm per axis; HOLDING arm realises it: G' = D_aboutP @ G0, 5-DOF DLS IK (kinematics.KinematicChain.ik, seed = frozen holding q, soft limits); keep only if FK-achieved rotation <= 2 deg and |t| <= 20 mm per axis and the mesh check with the carried tube is clean, else resample (bounded). Seeded schedule 50 = 10 nominal + 40 perturbed, saved per episode (requested, achieved, holding ticks). Return model: use the current (perturbed) holding pose as the holding arm's home in the follower return (5 deg drift rule unchanged), return the inserting arm first, then move_holding() back to frozen, then the next perturbation. Files: perturb.py, tests/test_perturb.py, lerobot_env/record_train.py + .sh --arm, scripts/place_training_tube.py, move_holding in inchworm_follower.py, lerobot_env/tests/test_record_train.py. Leader map right needed (refuse without it).

2026-11-21 implemented (commit 27613f9 + follow-up). Files: src/inchworm/perturb.py (Planner: attachment point P0 = inserting gripper at timeline frame 48/73 carried into the frozen tube; D = rotation about P0 U[0,2] deg random axis + U[-20,20] mm per axis; holding arm solves a bound-normalised Gauss-Newton fit in ticks; ACHIEVED D by FK kept only if <=2 deg / <=20 mm per axis, inserting arm can reach the perturbed inserted pose within 3 mm/1 deg, and the holding joint line is mesh-clean with the tube carried; else resample, max 200), scripts/place_training_tube.py, lerobot_env/record_train.py + record_train.sh --arm + record_train_left.sh / record_train_right.sh, follower InchwormFollower.move_holding (mesh-checked holding-arm line, 5 deg drift refusal, typed YES on the first move; return model then uses the perturbed holding pose), tests tests/test_perturb.py (7) + lerobot_env/tests/test_record_train.py (12). Evidence: full suites pass (robot rc=0, lerobot_env rc=0). Dry runs (no hardware): replay_cycle --stop-after-frame 42 and 67 without the tube: simulation OK, stop holding; place_training_tube --arm left/right on simulated servos: holding gripper opened only after 'tube NOT attached' YES, locked via GripperLockSM, inserting gripper never commanded, state written (frame 42 locked R / frame 67 locked L, tube in). record_train --dry-run: left seed 1: 50 = 10 nominal + 40 perturbed, max rot 1.92 deg, max |t| [15.3,19.8,19.7] mm, mean rot 0.83 deg, 45 tries for 40 (sampling works almost first time), every tube move from the frozen placement and every return from the perturbed inserted pose plan clean with the real cell mesh; right seed 1: 50 = 10+40, max rot 1.81 deg, max |t| [18.5,19.9,19.2] mm, 41 tries, all clean. Planning 50 placements takes ~23 s. Session: a discarded take repeats its placement; between perturbed placements the tube goes back to the frozen placement first. AC #3/#5 left open until a supervised live run (the session test uses simulated servos with stand-in models; the mesh-checked dry run has no servo simulation). Before right recording: leader_map_right.json must be fitted (record_train refuses without it). Achieved rotation is limited by the 5-DOF holding arm: samples whose achieved pose leaves the bounds are rejected, not clipped.
<!-- SECTION:NOTES:END -->
