---
id: AA-56
title: >-
  Autonomous crawl: left to the regrip pose before right out, timed locks, no
  prompts
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-23 16:25'
updated_date: '2026-11-24 02:05'
labels: []
dependencies: []
priority: high
type: feature
ordinal: 56000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner 2026-11-23 after the first successful live two-policy crawl (AA-55): make it run better and unattended. After the left insert + lock, the left goes straight to its known train_right (regrip) pose BEFORE the right un-inserts; owner: no collision avoidance for this move, a little flex is OK. Then the right unlocks, comes straight out 50 mm from wherever it is, and goes to its train_right start. No YES / l-r-q / look prompts after the program starts; grippers lock on a timer. Owner chose all recommended options (2026-11-23): (1) only the cell-mesh check is skipped on the forced move, joint/soft limits stay; (2) joint line 5 deg/s; (3) right moves after are planned from the actual pose and keep their checks, tube carried by the left; (4) forced move refused if the left locked > 8 deg from its finish reference; (5) no per-phase YES, no l/r/q, no look, but the right unlocks only if the left reads LOCKED by read-back; (6) the start replay's YES + tube-keys YES stay, the end holds and asks Enter before joints off; (7) Ctrl-C / stop files still hold both arms; (8) lock when finished by the rule or after T = 25 s; (9) at T and > 8 deg from the finish reference: return + one automatic retry, then stop and hold; (10) new --autonomous mode, the supervised mode stays. Builds on lerobot_env/crawl_two_policy.py and inchworm/two_policy.py.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 --autonomous mode: after the start replay's prompts, the crawl runs to home with no YES, l/r/q or look prompts; the supervised mode is unchanged (tests)
- [x] #2 After the left lock, the left moves on a 5 deg/s joint line to its frozen train_right ticks with both grippers LOCKED, before the right unlocks; joint/soft limits checked, cell-mesh check skipped for this move only; refused (nothing moves) if the left locked > 8 deg from its finish reference (tests)
- [x] #3 Then the right unlocks only if the left reads LOCKED by read-back, goes straight out 50 mm from its actual pose and then to its train_right start, planned from the actual pose and preflighted with the cell mesh and the carried tube (tests)
- [x] #4 Each policy locks automatically when finished by the rule or at 25 s; at 25 s more than 8 deg from the finish reference: return + one automatic retry, then stop and hold both arms (tests)
- [x] #5 Ctrl-C and the stop files still hold both arms at any point; the end holds at home and asks Enter before joints off (tests)
- [ ] #6 Dry run of --autonomous with the v5nom checkpoints exits 0 with all phases done, then the first live run is owner-supervised and its log and outcome recorded in the notes
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
1. two_policy: AUTO_PHASES, plan_left_to_train_right(right_locked=True), forced_line_problems (soft + EEPROM limits, speed ceiling; no mesh). 2. crawl_two_policy --autonomous/--lock-s 25: follower confirm=False; auto_policy_phase (lock when finished or at the timer within 8 deg, else return + 1 retry, then Stop/hold); auto_lock with read-back; auto_crawl: left lock, 8 deg lock-pose guard, forced left line to train_right ticks with both LOCKED, re-plan right out + approach from the actual pose (mesh-checked with the carried tube), unlock right after read-back, train_right gate + state; right policy, auto lock, home without YES, final Enter kept. Supervised mode unchanged. 3. Tests in tests/test_crawl_two_policy.py. 4. Dry run with v5nom, then owner-supervised live run.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-23 implemented (owner chose all recommended options). Files: robot/src/inchworm/two_policy.py (AUTO_PHASES, plan_left_to_train_right right_locked, forced_line_problems), robot/lerobot_env/crawl_two_policy.py (--autonomous, --lock-s 25; locked_readback, auto_policy_phase, auto_lock, auto_crawl, home). Supervised mode untouched (its 7 tests pass unchanged). New tests (5): autonomous end to end with only the final Enter prompt and the phase/gripper order; far at the 25 s timer -> 1 retry then hold (nothing locks, right stays LOCKED); left locked 12 deg off -> forced move refused, both stay LOCKED; stop file in the right policy holds with the left LOCKED; forced line checks limits (not the mesh) at ~5 deg/s. Suites: lerobot_env 106 passed, robot 1960 passed. Dry run with insert_*_v5nom 016000, --max-deg-s 8 (log robot/out/rollout/crawl_two_policy_dry_20261123T170834Z.jsonl): exit 0, 0 'Type YES' after the start replay; left finished 14.8 s (4.2 deg) -> lock read back 178.2 deg; forced left_to_regrip 13 frames, left ended within 1 tick of train_right; right out + approach re-planned from the actual pose, mesh OK; right finished 15.5 s -> lock; home; crawl_done. Before the live run: owner-measured (previous live run) the forced move drags the right gripper point ~21 mm if the tube were perfectly rigid; the owner accepted flex. Live run pending, owner-supervised.

Correction to the note above: the ~21 mm is the AGENT's model estimate from the 2026-11-23 14:04Z live log (rigid tube, left lock ticks -> train_right ticks; right gripper point displacement), not an owner measurement. Shown to the owner before the first live run.

2026-11-23 21:33Z first live --autonomous attempt stopped before any policy motion (after the start replay; right LOCKED on the tube, left open at train_left): camera bridge 'camera_left_arm: 0 DirectShow cameras at BusId 9-4'. usbipd list: the hub with the relay (CH340), the right arm adapter (CH343 5B91030298) and the left camera now enumerates as bus 6 (6-2 / 6-3 / 6-4; were 9-2 / 9-3 / 9-4). OWNER: use the new port. hardware.yaml left camera windows_busid + camera_bridge left_arm busid = 6-4 (observed relay 6-2, right arm 6-3); tests pinned to the committed config updated (test_config, test_camera_bridge, test_attach_usb, test_discovery fixtures); attach_usb --dry-run 8/8 OK (cam_left SKIP at 6-4); robot suite 1960 passed; doc-1 updated.

2026-11-24 02:05Z STATE FOR NEXT SESSION. Live --autonomous runs: 22:23Z left failed (2x25 s, 22 deg off) -> stop/hold OK; 22:27Z COMPLETED (left retry finished 4.4 deg, forced regrip move, right finished 14.9 s / 7.8 deg, home, crawl_done); 01:46Z left failed (18.3 deg) -> stop; 02:01Z COMPLETED (left locked at the 25 s timer 6.8 deg off, forced left_to_regrip 4.8 s peak load left [56,84,124,100,44] right [0,48,28,36,0] (low), right out, right finished 13.1 s / 3.4 deg, home, crawl_done; log robot/out/rollout/crawl_two_policy_20261124T020111Z.jsonl). Owner has NOT yet said whether the keys were really inserted on the completed runs or how much the tube flexed. Left-policy unreliability tonight: loop 26-27.5 Hz and mesh check ~6 ms/step, holds 0 on every attempt (not the cause). Camera check (robot/out/camchk/*.jpg, git-ignored): brightness/contrast/position identical to the v5nom demos, but live vs pose-matched demo frames differ 8-10 (demo vs demo 2.4), concentrated in the top-left quarter (tube end / right gripper); owner: 'the arm is a slightly different angle, barely anything'. Likely the tube seating in the right gripper. Owner options offered: re-seat tube; record ~15 more nominal demos/arm + retrain; small shift/rotation augmentation (needs owner choice, changes the recipe). OWNER 2026-11-24 ~02:05Z: GOES BACK on option (forced move): wants the autonomous crawl to use the ORIGINAL order again (right unlocks + comes out first, then the left (with tube) to train_right, mesh-checked), keeping no prompts + timed locks. NOT YET IMPLEMENTED: next session = add that (e.g. --autonomous keeps AA-55 order; the forced move only behind an explicit flag or removed), update AC #2 with owner approval recorded, tests, dry run, owner-supervised live run.
<!-- SECTION:NOTES:END -->
