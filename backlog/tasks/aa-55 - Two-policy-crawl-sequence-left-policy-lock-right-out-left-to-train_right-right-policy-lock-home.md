---
id: AA-55
title: >-
  Two-policy crawl sequence: left policy, lock, right out, left to train_right,
  right policy, lock, home
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-21 01:55'
updated_date: '2026-11-21 16:59'
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

2026-11-21 session 2 (training running, memory-light): built robot/lerobot_env/crawl_two_policy.py + crawl_two_policy.sh (wraps replay_cycle frames 0..42, then the 9-phase sequence; reuses full_policy_test UI/policy_phase/lock_and_look/hold_and_release/open_live_buses by import; flags --left-checkpoint/--right-checkpoint, default the v2 020000 checkpoints). Finish refs = median last-frame observation.state per dataset, measured: left v2 (1982,2203,1919,1388,1986), right v2 (2062,2482,1646,1395,2007) ticks; the test recomputes them from the parquet. Steps 4-6 are planned and preflighted (frames_problems + stream preflight, tube carried by the left from its actual lock pose) BEFORE the right unlocks; each motion phase has its own typed YES. Right unlock (unlock_right) needs owner y + the left read-back inside its LOCKED band + the SM in LOCKED, then GripperLockSM.unlock (which refuses unless the left is LOCKED). Home: timeline 74..86 fade-out, joint-line fallback if that fails its preflight. New lerobot_env/tests/test_crawl_two_policy.py: 6 tests PASS (stand-in policy that drives to the finish ref, simulated servos, real mesh+timeline): full crawl order + gripper sequence lock L / unlock R / lock R, phases right_out < left_to_train_right, pose gates train_left+train_right, end both LOCKED + joints off; refusal 35 ticks off train_left (nothing moves); owner look n -> no unlock; unlock_right unit refusals; quit in the right policy keeps the left LOCKED. Dry-run sim log: carried tube vs frozen train_right 11.8 mm / 3.6 deg (logged only, as decided). Full suite NOT run (owner: wait until training ends), NOT committed yet. NOT done: CPU dry run with the real 5000-step checkpoints (left 005000 exists; the right does not until the left run ends) - run: bash robot/lerobot_env/crawl_two_policy.sh --dry-run -- --device cpu --left-checkpoint ../out/train/insert_left_v2_20261121T050907Z/checkpoints/005000/pretrained_model --right-checkpoint ../out/train/insert_right_v2_20261121T055710Z/checkpoints/005000/pretrained_model

2026-11-21 ~15:00Z: AA-25 training DONE, both exit 0 (left 06:31-09:13Z, final loss 0.094; right 09:13-11:32Z, loss 0.072; checkpoints 005000..020000). Full suites pass: robot 1960 passed, lerobot_env 99 passed; committed e0d8e74 and pushed. DRY RUN with the real 020000 checkpoints (GPU, simulated servos, frames 0..42 preflight 0 problems): stopped SAFELY at the step-3 lock check both times. (a) replay episode 0 (a perturbed episode), log out/rollout/crawl_two_policy_dry_20261121T145340Z.jsonl: left time_cap at 20 s, locked 9.4 deg (wrist_roll) from the finish ref; the policy matched episode 0's own end (wrist_roll 184 vs ep0 182.5). (b) nominal episode 23, log crawl_two_policy_dry_20261121T145649Z.jsonl: 18.5 deg (elbow_flex) off. CAVEAT: the dry run does not close the loop (replayed camera frames do not follow the simulated arm), so it shows the plumbing and checkpoint loading, not policy quality. Finding: the 8 deg lock check vs ONE median fits only the nominal demos (all 10 nominal ends within 4.6 deg of the median); 11/50 left and 26/50 right demo ends lie > 8 deg from it (max 17.1 / 25.1 deg) because the perturbed tube moves the insert pose. Owner decision needed before live: how the step-3/8 lock check should treat a perturbed tube placement (e.g. widen the tolerance, or check the right_out / left_to_train_right plan from the actual lock pose and drop the 8 deg check). Not changed without the owner (doc-2).

2026-11-21 ~15:50Z OWNER DECISION: drop the 8 deg lock-pose check (both locks). Reason given: it came from AA-39, where the left was moved with the tube toward the demo-end pose while both grippers were locked (a far lock would have pushed/flexed the tube); in AA-55 the right is unlocked and out of the key first, and every later move is planned from the ACTUAL lock pose and mesh-checked with the carried tube, so the check only rejected the perturbed placements (11/50 left, 26/50 right demo ends > 8 deg from the median). Built: crawl_two_policy.log_lock_offset (logged only, event lock_offset_from_finish_ref); policy_phase(warn_far=False) suppresses the old 'cycle will NOT continue' warning; new test test_lock_far_from_the_finish_ref_is_logged_and_the_crawl_continues (left 12 deg off: unlock + right_out + both train_right moves run). Dry run with the real 020000 checkpoints (GPU, sim servos): ALL 9 steps DONE, log out/rollout/crawl_two_policy_dry_20261121T154504Z.jsonl: left lock 9.4 deg off ref (logged), plans ok, carried tube 24.6 mm / 5.1 deg off frozen train_right (logged), right unlock, right_out, left_to_train_right, right_to_train_right, train_right gate 0 problems, right lock 27.5 deg off ref (open loop, see caveat), home, release. Suites: robot 1960 passed, lerobot_env all passed. Caveat stays: the dry run is open-loop (replayed frames), not a test of policy quality. NOTE for the owner: tube-pose differences up to ~25 mm are only logged; the right policy was trained with tube offsets <= 20 mm per axis.

2026-11-21 16:15Z FIRST LIVE crawl_two_policy.sh (owner supervising), log robot/out/rollout/crawl_two_policy_20261121T161520Z.jsonl, trace policy_crawl_20261121T161520Z.jsonl. train_left gate 0 problems. LEFT POLICY FAILED: 3 attempts (r, r, l), each hit the 20 s cap; it went the WRONG way: elbow 193 -> 199 deg, wrist_flex 103 -> 96.5 (finish ref elbow 168.7, wrist_flex 122). Same pattern as the AA-39 live failure 2026-11-21 00:11Z (old checkpoint). The open-loop dry run on recorded training frames moves the right way (elbow 192 -> 163), so the policy follows the training images and not the live ones -> suspect an image difference (daylight vs the night recording 05:09-06:30Z = local 00:09-01:30; or the scene/camera). Owner locked anyway (look y); lock 30.9 deg (elbow) off the ref; plans passed; carried tube 61.8 mm / 4.4 deg off the frozen train_right tube. Right unlocked and came out OK (right_out worst path error 5 ticks). 16:19:45Z the AGENT stopped the left_to_train_right move (touch /tmp/inchworm_stop_move, own judgement, not asked): failed insertion + tube 62 mm off training. Both arms HOLD; left LOCKED holding the tube alone, right UNLOCKED and clear. Release prompt pending.

2026-11-21 ~16:45Z: live view added: policy_handoff.start_preview (browser MJPEG of robot._last_frame = exactly what the policy gets, mean/std overlay; never opens a camera; record_handoff.Preview), --preview-port 8765 (0 = off) in policy_handoff.py (policy_test.sh) and crawl_two_policy.py. Suites 1960 + 100 pass. Camera exposure: the bridge handshake sends auto_exposure=None = camera default (doc-1 AA-53: use auto exposure, DirectShow manual exposure not mapped), same in recording and policy runs, so not a recording/run mismatch; auto exposure does not remove daylight vs lamp differences. OWNER PLAN: record daylight demos, merge with the night v2 sets, retrain both arms. Merge verified 2026-11-21 (lerobot-edit-dataset merge of the two v2 sets -> /tmp/merge_test, 100 ep / 36159 frames, exit 0).
<!-- SECTION:NOTES:END -->
