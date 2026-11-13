---
id: AA-50
title: >-
  Full-cycle movement test on the real arms: replay the AA-45 inchworm cycle
  without the tube, grippers moving
status: In Progress
assignee:
  - '@orchestrator'
created_date: '2026-11-12 21:02'
updated_date: '2026-11-13 02:20'
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
- [x] #1 A replay script plays out/inchworm/timeline.jsonl through GuardedArm (arms) and GripperLockSM (grippers), dry-run by default, with a stop file, hold-on-abort and torque release at the end
- [x] #2 An offline preflight checks every frame-to-frame move with both arms together against the cell mesh model with no tube, the soft limits and the EEPROM tick limits, and refuses on any problem; it passes on the current timeline
- [x] #3 A simulated-servo run of the full cycle passes and is covered by tests (gripper unlock/lock order, speed never full, obstruction aborts)
- [x] #4 The owner approves clearing the soft-limit live gate for this no-tube replay, recorded with its scope
- [x] #5 A supervised live replay of the full cycle without the tube completes, logged per frame (goal vs reached, load), with the owner's observations recorded
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

2026-11-13: OWNER DECISION (chat): 'lets do a move even though the joint soft limits are marked unverified' -> AC #4. Implemented as a per-run waiver, NOT a config change: JointLimitGuard/build_motion_guard/build_guarded_arm take unverified_soft_limits_override (owner approval text); it waives only the unverified=>refuse-live gate, logs a WARNING with the text, and still enforces the limit values, keep-out boxes, mesh collision and calibration gate. hardware.yaml soft limits stay unverified: true. scripts/replay_cycle.py: dry-run = EEPROM clamp + preflight (cached by timeline/config/code hash) + full simulated cycle; --live requires --allow-unverified-soft-limits TEXT, refuses on torque ON / zeroed drive registers / start pose >3 deg from frame 0, asks YES (no tube, DexArm parked, hands clear), lock both (45-deg), move to frame 0, cycle with a pause before each phase, overcurrent/overtemp/stall/timeout abort -> HOLD -> owner supports -> release; final torque-off sweep; log calibration_data/replay_cycle_<date>.jsonl. Evidence: dry-run on the real timeline 'preflight: 0 problem(s) in 98 s', 'simulation OK: 85 moves, 6 gripper actions, worst error 20 ticks, 70 s simulated'; '--live' without the waiver -> REFUSED rc=2. Tests: tests/test_cycle_replay.py 9 pass (waiver keeps values enforced, CLI refusal, stop->hold->release, obstruction abort, sequence/speeds); full suite passes. AC #5 (live run) waits on AA-49 (gripper lock confirmation) and a supervised session.

2026-11-13 owner request: 'can the robot move to the right start pose itself ... if I'm already close, all I need it to do is make the final adjustment'. Live start no longer refuses at 3 deg: cycle_replay.approach_problems() allows the automatic final adjustment when every joint is within APPROACH_MAX_DEG = 10 deg (UNVERIFIED placeholder) of frame 0 and the straight move (both arms together, no tube) is clear in the cell model; else REFUSED with the joint and 'place the arm closer by hand'. The move itself still goes through GuardedArm. The real fold (2026-11-11 snapshot, elbows 3.5 deg off) passes; 2 new tests. AA-51 steps 3-4 revised, so the owner must re-accept.

2026-11-13 AA-51 step 3 live start REFUSED (calibration_data/replay_cycle_2026-11-12.jsonl, 22:39:24Z): 'adjustment move (t=0.00): right_lower_arm within 0.1 mm of right_shoulder' at the owner's hand-placed fold. OWNER DECISION: 'I want those two to be able to touch! This gives the robot arm more stability when holding the cylinder.' Implemented: motion_guard.collision.mesh.always_allowed_pairs [left_lower_arm,left_shoulder], [right_lower_arm,right_shoulder] (PoseScopedAllowance with infinite radius); all other pairs still checked. Owner also asked what else would refuse for no reason. Audit with the owner's real pose (ticks L 1979/1402/3121/1014/2002, R 2075/1455/3075/970/2017): approach check now passes; full cycle in live-mode guard path (GuardedArm live=True + waiver) on simulated servos starting there: OK, 85 moves, 6 gripper actions. Random hand placements within 10 deg of the start pose: 16/300 still refused, all by the wrist folding into its own upper arm (left/right_upper_arm vs _wrist), which the model allows only within 0.05 rad of the fold. Preflight after the change: 0 problems. test_fold_allowance_is_pose_scoped updated to show the scoping with the wrist pair (the lower-arm pair is now allowed everywhere by owner decision). Full suite passes.

2026-11-13 LIVE (owner present), calibration_data/replay_cycle_2026-11-12.jsonl, step-by-step mode at 150 ticks/s: run 1 start 00:23:01Z -> done 00:27:31Z -> released {} ; run 2 start 00:51:59Z -> done 00:59:48Z -> released {} (one false start 00:50:46Z: ServoChecksumError servo 2 -> hold -> released). Each run: grippers LOCKED L 2028 / R 2030, L unlock(1609)/lock, R unlock(1586)/lock at the regrips; frames worst error 20 ticks, peak load 172. Owner observation: 'It worked great!'. AC #5 evidence.

2026-11-13 owner requests after the run: (1) 'fluid motion matching the speed of the animation', start at half speed, both arms moving at the same time, no stopping; (2) no phase pauses / Enter prompts, only the start prompt; (3) the fold should be the touching pose (lower arm resting on the shoulder, supported while printing). Owner approved option (b): the whole fluid path is checked against the cell model before the start prompt; live, each streamed point gets the full GuardedArm pose check. Done: cycle_replay.build_segments (split only at gripper actions; monotone cubic through the frames, eased start/end; ANIM_FRAME_S 0.2 = GIF fps 5; tempo 0.5 default), stream_preflight (every 25 ms, both arms, soft+EEPROM+mesh, speed ceiling 700 ticks/s), ReplayRunner.run_fluid/_stream_segment (50 ms loop, target 0.1 s ahead, per-joint Goal_Speed, abort if > 100 ticks off the path, stall/overcurrent/overtemp, single read retry on a garbled reply), GuardedArm.write_stream_point_rad (pose check + target within 0.2 rad of present). Phase pauses removed; --step-by-step keeps the old mode. Dry-run on the real timeline: 'fluid-path preflight: 0 problem(s)', '36 s of arm motion in 5 pieces, peak joint speed 54 deg/s', 'simulation OK ... worst error 7 ticks'; real-time loop with live guards on simulated servos: 10.0 s planned / 10.02 s taken, lag 5-7 ticks. Collision checker: per-check AABB corners cached and FCL objects built only for pairs whose boxes overlap (same checks, ~2x faster). ELBOW EEPROM (owner approved): Max raised +60, LEFT 3091->3151 (backup eeprom_backup_left_2026-11-12T20-03-01_pre_limits.json), RIGHT 3046->3106 (eeprom_backup_right_2026-11-12T20-03-02_pre_limits.json), read back OK, torque was off. The replay still caps the fold elbow at the old place (FOLD_ELBOW_CAP_TICKS L 3081 / R 3036) until the owner decides how to press into the contact (owner: 'wait on my answer'). Tests: 9 new in test_cycle_replay.py; full suite passes.

2026-11-13 fluid live runs (log replay_cycle_2026-11-12.jsonl, mode 'fluid tempo 0.5'): 5 starts, 1 done (01:55:09Z, 4 segments, lag <= 30 ticks); 4 aborted on single temperature readings: L wrist_flex 70 C, L elbow 84 C, R shoulder_lift 76 C, L shoulder_lift 70 C (+ earlier one checksum error). Torque-off probe right after (300 reads per servo, both arms): 28-31 C, max 38, no errors, so the hot values were single bad reads, not real heat (servo EEPROM Max_Temperature 70 still protects in the servo itself). Owner: 'the arms are really jerky'; 'I'm not sure why we're stopping to check at all while its running'; OWNER DECISION: 'I don't need safety checks once we have confirmed a motion'; 'aborting like that is more dangerous than whatever is trying to be prevented'. Done: fluid mode streams the preflighted path (checked every 20 ms, both arms, soft+EEPROM+mesh+speed ceiling, before YES) with NO per-point checks: GuardedArm.write_preflighted_ticks -> ArmServoBus.sync_write_goal_ticks (one SYNC WRITE 0x83 packet per arm, regs 42-47 position+speed, no reply, gripper rejected, speed 1..3000 never 0), 50 Hz, each goal 60 ms ahead at the path speed, accel 100. abort_on_faults=False in fluid mode: temperature/current/lag/read errors are logged (reading_warning, segment max_temp_read_c, read_errors) and never stop the arms; only Ctrl-C / stop file / a gripper state-machine failure stop it. Step-by-step mode keeps the old aborts. Real-time sim with live wiring: 495 loops in 10.0 s (50 Hz), lag 9 ticks. Dry-run: preflight 0 problems, sim OK worst 12 ticks. Tests updated/added; full suite passes.
<!-- SECTION:NOTES:END -->
