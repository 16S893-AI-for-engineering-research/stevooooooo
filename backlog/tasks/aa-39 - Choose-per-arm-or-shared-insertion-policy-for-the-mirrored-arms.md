---
id: AA-39
title: Choose per-arm or shared insertion policy for the mirrored arms
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-24 16:07'
updated_date: '2026-11-21 01:55'
labels: []
milestone: m-3
dependencies:
  - AA-25
type: task
ordinal: 39000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The left and right SO-101 arms insert from mirrored sides of the tube. The owner approved starting with one arm's policy (AA-23/AA-25) and adding the other later. Afterwards a choice is needed: train a second, separate policy, or one shared policy on mirrored data with an arm-identity input. That choice drives the owner's demonstration burden, since a second policy roughly doubles it.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Once the first arm's policy passes AA-25, a written comparison of a separate second policy versus a shared mirrored policy states the expected demo count, the risk and the recommendation
- [ ] #2 The owner's choice is recorded as a decision record, and follow-up capture and training tasks for the second arm are created to match
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
Owner 2026-11-20: try the LEFT ACT policy on the RIGHT arm before any retraining; merge replay_cycle + policy_handoff into one launcher.
Facts: doc-1 says the arms are mounted the same way (not mirrored) and all joint signs are +1 on both arms. Timeline: the right pre-insertion pose (frame 67) = the left one (frame 42) = [0,-21.8,9.0,-75.5,0] deg, and inserted 73 = 48. The policy works in raw servo degrees, so the per-arm zero offsets differ (pan 173.320 L vs 181.318 R, lift 214.893 vs 220.518, elbow 182.900 vs 179.033, wrist_flex 178.286 vs 175.254, roll 175.687 vs 175.115).
1. policy_handoff --follower right: observation right raw deg -> left-equivalent (deg - zero_R + zero_L, per joint, sign +1 both, asserted from hardware.yaml; refuse if any sign differs), policy, action back (deg - zero_L + zero_R). Plugin follower=right (handoff gate, mesh, limits, return already arm-generic; check that return/handoff_return work with follower=right, the other arm = left, LOCKED). Tests: round-trip mapping, sign refusal, right-arm sim attempt.
2. replay_cycle --stop-after-frame 67 --tube-in-grippers: check that it accepts frame 67 (right open, left LOCKED) and writes handoff_state for the right arm; dry-run.
3. ONE launcher (owner): robot/lerobot_env/policy_test.sh --arm left|right [--dry-run]: runs replay_cycle to the arm's stop frame (42 left / 67 right) and then the policy session in the same terminal. Decide the mechanism: subprocess call of scripts/replay_cycle.py in the main env, then policy_handoff (keeps each env's deps), or import inside lerobot_env (it installs inchworm-robot[model]). The YES prompts of both stay. If the replay fails, the policy step never starts.
4. Dry runs: both arms (simulated servos + real cell mesh). The right arm sees the camera frames of a left episode in its dry run (a mapping check only).
5. Owner-supervised first live right-arm trial with the same guards (5 deg/s, 10 deg jump, 20 s). Report the result; then the AA-39 comparison (AC1) and the owner's decision (AC2).
Risks: the right wrist camera sees a different scene (the left arm holding, background, lighting); real key offsets; camera mount differences. If it fails: 10-20 right demos (fit leader_map_right first), then fine-tune the existing checkpoint on left+right.

FULL TEST (owner 2026-11-20, replaces policy_test.sh --arm right; decisions 1a/2a/3 in notes). One script lerobot_env/full_policy_test.py (+ .sh), one terminal, dry-run first:
F1. replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live (exists). Arms hold after exit.
F2. Left policy (InchwormFollower handoff, follower=left, finish rule). Time cap without finish = hold + owner prompt: l lock anyway / r return + retry / q quit.
F3. Left lock via GripperLockSM (AA-17) on the left gripper only; then owner y/n by eye 'key IN and LOCKED'. n = hold, nothing unlocks; quit path releases joints only (grippers untouched).
F4. Left joint line (slow, 5 deg/s, mesh-checked, right arm static) to the demo-end median pose (left ticks 1977/2195/1930/1396/1988). Owner accepted the tube push.
F5. Port handover WITHOUT torque cut: a new follower disconnect mode 'keep holding' (today handoff disconnect turns joints of both arms off), then a new replay_cycle mode --resume-from 49 --stop-after-frame 67 for arms already holding with the left LOCKED: no torque-off refusal, no start adjustment, no fold press, GripperLockSM adopts LOCKED for left (read-back in band) and LOCKED for right; frames 49-67 = the timeline with the left q replaced by the demo-end pose (left static there), right frames and tube unchanged (the actual tube is held where the right gripper holds it; the 29 mm is left-arm model error; owner 3: the right aims at the model point). Preflight + fluid preflight of that path before the YES. Right unlock refused unless left LOCKED (exists).
F6. Right policy (follower=right, mapped, finish rule; same prompts).
F7. Right lock + owner y/n; end: both arms hold, then the usual release prompt (support tube + both arms; joints off, grippers untouched).
Open technical risk: the cell mesh may report left gripper vs tube contact at the demo-end pose (model 29 mm off); if so, ask the owner before adding any allowance.
Tests: sim end-to-end (replay_sim buses + plugin sim HW), refusals at each gate, gripper commanded only via GripperLockSM, never both unlocked.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-20 step 2: dry-run 'replay_cycle.py --stop-after-frame 67 --tube-in-grippers' passes unchanged (preflight 0 problems/61 s, fluid preflight 0/24 s, sim OK worst 12 ticks, stop at frame 67: left LOCKED, right open, both holding; writes sim_handoff_state.json). NOTE: frames 43-49 are the scripted LEFT insertion + lock WITH the tube, then the right unlock (frame 50); this live path with the tube was never run (the left demo stopped at 42). Owner supervises the first live run.

2026-11-20 steps 1-4 BUILT, dry-run OK, NOT live.
- src/inchworm/arm_transfer.py: JointOffsetMap (source deg = target deg + zero_source - zero_target; refuses unless every joint sign matches and is +/-1) + MappedPolicy (maps obs in / action out, frame unchanged, extra keys passed through so the joints-1-5 check still refuses). Map L = R + pan -7.998, lift -5.625, elbow +3.867, wrist_flex +3.032, roll +0.572 deg. Tests tests/test_arm_transfer.py (5: offsets, round trip, same URDF q, sign refusal, wrapper).
- policy_handoff.py: --arm left|right, --policy-arm (default left), --handoff-state; _SimHW arm-generic; logs event policy_arm with the map. Tests: left policy drives the right arm (+3 deg lift in left space = +3 deg on the right, left arm + both grippers untouched, release turns joints of both off, left gripper stays on); right follower refuses a frame-42 state.
- Return with follower=right: 2 new tests in tests/test_handoff_return.py on the real cell mesh + timeline (stop 67, inserted 73): straight-out 50 mm, then right line, then left (with tube) to its stop; every step checked. Pass.
- lerobot_env/policy_test.sh --arm left|right [--dry-run] [-- policy args]: subprocess replay_cycle.py (main env) to frame 42/67 with --tube-in-grippers, then policy_handoff.py in lerobot_env. The policy step starts only if the replay exited 0 AND wrote a fresh state file (mtime after launch) for this arm's frame with this arm open, other LOCKED, tube in (replay exits 0 when YES is refused). Tests tests/test_policy_test_sh.py (6, uv stubbed).
- Dry runs (real checkpoint 20k + real cell mesh, simulated servos): policy_test.sh --arm right --dry-run: replay preflight 0/fluid 0, sim worst 12 ticks, stop 67; policy 20 s, 0 holds, 0 jump stops, max target-present 3.93 deg. --arm left: preflight 0, stop 42, policy OK.
- FINDING (not fixed, needs owner): right-arm dry run had 359/361 loop overruns (left 63/498). Cause measured: the mesh gate's margin distance query baseforreal vs right_upper_arm (AABBs overlap, meshes 107 mm apart; 14k x 45k faces) costs ~21-32 ms per check on the right vs ~3 ms on the left (46 mm). Median gate 37 ms right vs 12 ms left > 33 ms period. Effect: loop ~18 Hz instead of 30; the 5 deg/s cap is per step, so the arm moves SLOWER, never faster; the policy's action chunk plays slower in wall time. Options for the owner: (a) accept for the first trial; (b) --fps 15 for the right trial; (c) a coarser hull for that one pair (changes the collision model: owner approval).
- Untested live: replay frames 43-67 with the tube (scripted left insertion + lock at 43-49, right unlock 50, right carry to 67) were never run on hardware. First live policy_test.sh --arm right needs the owner supervising.

2026-11-20 owner: (1) will supervise the first live right run incl. the never-run frames 43-67 with the tube; (2) right-arm loop overruns: accept as is, NO change ('being okay with slow' = keep 30 Hz; the loop runs at ~18 Hz on its own; a 15 Hz commit was reverted). Owner question answered: after the policy + lock, the plan of record (AA-23 note 2026-11-13) is to re-plan from the actual pose; not built yet (policy_handoff only holds / returns / releases; lock + continue = AA-26/AA-27).

2026-11-20 OWNER: no seating detector is needed any more (AA-24). Owner proposal (pending agent caveat): after the policy + lock, move the arm to a FIXED known pose and continue the scripted cycle, no live re-plan. Agent measured: the 29 demo ends (median) and the 3 live policy ends differ from the timeline's planned inserted pose (frame 48) by lift ~-12.5 / elbow ~+13.6 deg = gripper ~28-29 mm in x and 4 deg (FK, tool offset unverified); live ends are within 0-6 deg of the demo median. AC/status changes to AA-24/AA-26/AA-27 await the owner's answers.

2026-11-20 owner decisions: (1) post-lock pose = measured demo-end pose (redo the timeline from frame 49 from it); owner may switch to accepting the push on the tube; not built. (2) finish rule = still 2 deg / 2 s + within 8 deg of the demo end (built, see AA-26). (3) AA-24 closed not needed; AA-26 ACs reworded.

2026-11-20 21:52-21:59Z FIRST LIVE policy_test.sh --arm right (owner present). Replay 0..67 with the tube: carry worst path error 34/38 ticks; left UNLOCK 1606; frames 24-48 = SCRIPTED left insertion (no policy); left LOCK read LOCKED at 2028 ticks (position band only); right UNLOCK 1589; stop 67. Owner: the scripted left insert failed ('obviously'). SAFETY NOTE: the lock band reads LOCKED by position alone, so a failed key insert can still read LOCKED, and the right gripper then unlocked; the tube may have been held by one unverified gripper. Policy right (mapped): 2 attempts, both hit the 20 s time cap (finish rule did not fire), 0 holds, 0 jump stops, overruns 183/514 and 155/520, return between them clean (worst track 18 ticks). Attempt 1 ended within 1.5 deg of the mapped demo-end pose on joints 1-4 (lift 197.0 vs 198.5). Trace out/rollout/policy_right_20261120T215418Z.jsonl. As of 22:01Z policy_handoff was still waiting at its release prompt (hold logged 21:59:12Z). Owner: the script was wrong; wants a FULL test: left runs the policy, lock, cycle continues, right runs the policy. policy_test.sh --arm right DISABLED (refuses) until that replaces it.

2026-11-20 OWNER decisions for the FULL test (left policy -> lock -> continue -> right policy): (1a) after each policy lock the owner confirms by eye (y/n) that the key is IN and LOCKED before the other gripper unlocks; (2a) time cap without the finish rule = hold and ask the owner: lock anyway / return + retry / quit; (3) yes: frames 49-67 re-planned from the measured left end pose, the right approach aims at the model point (~29 mm off) and the right policy must absorb it.

2026-11-20 23:50Z FULL TEST BUILT (plan F1-F7), NOT yet run live. Launcher: bash ~/893/robot/lerobot_env/full_policy_test.sh [--dry-run] (replay 0..42 -> full_policy_test.py). New code: GripperLockSM.adopt_readback (read-only adoption: locked band=LOCKED, unlocked band=UNLOCKED, else FAULT); ReplayRunner.adopt_holding(require=...) (no torque cut, no motion); cycle_replay.resume_frames (frames 49..67 with the left at the demo-end pose, refused unless the left is LOCKED and static there); build_segments fixed for a timeline piece not starting at 0; InchwormFollower.detach_holding (hold + close, no torque-off); policy_handoff.sim_buses shared across phases; attempt_end logs present_deg. Flow: left policy -> owner l/r/q -> detach holding -> runner adopts (left UNLOCKED, right LOCKED required) -> AA-17 left lock -> owner y/n look -> refuses if the left is >8 deg from the demo end or the right is >34 ticks off frame 49, mesh-checks the left line -> one YES -> left line at ~5 deg/s (abort_on_faults on) -> fluid 49..67 (right unlock refused unless left LOCKED) -> state file out/full_test_state_67.json -> right policy (mapped) -> right lock + look -> end hold + release prompt (joints off, grippers untouched). Resumed 49..67 preflight with the tube: 0 problems. Dry run (real checkpoint + mesh): left finished 11.7 s (1.76 deg), lock, demo end, 49..67 3.8 s, right finished 12.4 s (1.1 deg), lock, DONE. Note: one earlier dry run had a one-off 11.85 s inference stall (infer_ms_max 11850; not reproduced; first-time CUDA/page-in suspected), so that attempt hit the time cap. Tests: robot/tests/test_resume_cycle.py (11), lerobot_env/tests/test_full_policy_test.py (5: full run, owner n = nothing unlocks, quit = nothing locks, lock far from the demo end = stop before the right unlock, state not frame 42 refused). Both suites pass. NEXT: owner-supervised first live run of full_policy_test.sh (after reset_start.sh).

2026-11-21 00:11-00:15Z FIRST LIVE full_policy_test.sh (owner present): replay 0..42 with the tube OK (carry worst 35/37 ticks; stop present L [1973,2190,2196,1171,1997], R [2062,1728,1949,1866,1993]). LEFT policy FAILED: 3 attempts (time cap, time cap, owner stop), owner chose r, r, q; 0 holds, 0 jump stops; quit = hold, then joints of both arms off (grippers untouched: right LOCKED 2036, left open). Policy targets barely moved and went the WRONG way: elbow 193 -> 199 deg (demos 193 -> 170; the 2026-11-20 03:00Z live run of the same checkpoint 193 -> 167), wrist_flex only +2..4 (demos +16), roll drift +7. Commanded = target within median 0.09 deg, so the 5 deg/s cap was not the cause. Loop 16-18 Hz on attempts 1-2 (inference stalls 1.4 s / 2.1 s), 27 Hz on attempt 3 with the same behaviour. Logs out/rollout/full_test_20261121T001120Z.jsonl, policy_full_20261121T001120Z.jsonl, calibration_data/teleop_lerobot_2026-11-20.jsonl.
Checks after the run: (a) right-arm placement at the handoff = training within 3 ticks on every joint (all 29 demos 11-19 vs tonight; gripper 2036 both); left start also matches. (b) Camera: Windows bridge server running (PID 66604); live left camera 29.4 new images/s, 0 corrupt, mean 128 (not black/frozen). (c) Camera identity: owner covered the LEFT wrist camera; only busid 9-4 (camera_left_arm) changed (mean 129 -> 97); 10-2 unchanged. Not swapped; busids unchanged since before the training recording. Images robot/out/camcheck/. Remaining suspects: the scene in view (tube position in the right gripper, DexArm, lighting) or policy brittleness (26 demos). Proposed next: replay to 42 and compare left-camera frames vs training frames at the same pose.
2026-11-21 OWNER: wrist_roll (servo 5) start 1 deg further CCW on BOTH arms -> zero moved 6.5 deg in total: wrist_roll_zero_offset_deg L 174.687, R 174.115; fold_allowance roll centres L 0.0666, R 0.0643; start/stop roll ticks L 1988, R 1981 (-74 from original). LEFT roll soft lower limit TIGHTENED -2.2165 -> -2.2036 rad (tick 551, inside EEPROM Min 550; -2.2165 would be tick 542.6). Arm transfer map unchanged (both zeros -1). Timeline out/inchworm/timeline.jsonl unchanged (roll 0 in radians), so the start pose is now 1 deg CCW. CAUTION: the trained policy and dataset are in raw servo degrees from the OLD start roll; the policy will start 1 deg off its training roll. Dry-run replay_cycle --stop-after-frame 42 --tube-in-grippers: simulation OK, worst 12 ticks. NOT run live; first live run needs the owner.

2026-11-21 OWNER NEW PLAN (owner authorised task updates): choice made = SEPARATE per-arm policies, each retrained from a frozen training position (robot/config/training_poses.json, commit e8b6ee9): train_left = frame 42, train_right = frame 67 (left LOCKED holding at its gait pose). Recording = AA-54 (two programs, 50 episodes per arm: 10 nominal + 40 with the attachment point perturbed <= 2 deg rotation and <= 20 mm per axis by the HOLDING arm; NOT a per-joint jitter, owner correction). Run sequence = AA-55 (left policy -> left lock -> right unlock + out -> left to train_right holding pose -> right to its start -> right policy -> right lock -> both home). The joint-offset mapped left->right transfer and full_policy_test.sh (demo-end pose + resumed 49..67) are superseded; keep the code, do not delete. AC #2 (decision record + follow-up tasks) still needs the decision record; AA-54/AA-55 are the follow-up tasks.
<!-- SECTION:NOTES:END -->
