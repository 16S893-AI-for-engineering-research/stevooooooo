---
id: AA-23
title: Cross-arm teleoperated demonstration capture with tube fixture
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:12'
updated_date: '2026-11-15 18:39'
labels: []
milestone: m-3
dependencies:
  - AA-22
  - AA-18
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 23000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The insertion policy is learned from human demonstrations, and the owner will record many of them. Per decision-1, demonstrations use cross-arm teleoperation. One SO-101 follower, with torque off, is moved by hand as the leader, and the other follower mirrors it and performs the insertion. LeRobot's so101_leader only reads Present_Position with torque disabled, so a follower can serve as a leader. But followers are geared 1/345 on every joint, unlike a purpose-built leader, so backdrive effort must be measured.

Both arms normally hold the tube (doc-1 s2), and a torque-off leader cannot hold anything. So during recording the tube, or a representative attachment-point section, is supported by a fixture. The fixture must reproduce the real attachment-point pose relative to the inserting arm and its wrist camera, or the policy will be trained outside the distribution it sees at rollout. Joint 6 is the shared jaw/lock servo and must never be driven by teleop.

Every demonstration must start from the same pre-insertion pose definition used at rollout (AA-18). Start with one arm, then add the other (owner-approved).

See decision-1 and doc-1 sections 2, 3, 4, 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A cross-arm teleoperation mode maps the torque-off leader arm's joints 1-5 to the follower's, with per-joint sign and offset handling for mirrored mounting, and the mapping is verified at low speed under owner supervision with the result logged
- [x] #2 Joint 6 (shared jaw/lock servo) is excluded from the teleop mapping and the recorded action space at the code level, and lock and unlock remain under the AA-17 state machine, verified by a test
- [x] #3 Recording refuses to start unless the tube is supported by the fixture or by a locked, torque-enabled arm; a torque-off arm is never permitted to be the tube holder, verified by a test
- [ ] #4 A tube fixture is specified (design file or documented build) and validated by comparing wrist-camera frames and follower joint poses at the pre-insertion pose in the fixture against the real crawl configuration, with the measured difference recorded
- [ ] #5 Leader backdrive effort and teleop tracking latency are measured and recorded, with a stated judgement of whether demo smoothness is acceptable
- [x] #6 Each episode records synchronized wrist-camera frames, follower joint state, leader joint state, timestamps and the starting pre-insertion pose, in a LeRobot-compatible dataset stored locally only (never pushed to the Hub)
- [x] #7 Episodes always begin from the AA-18 pre-insertion pose definition, with controlled randomization of the starting offset whose range is recorded in the dataset metadata
- [x] #8 Episode success or failure is labeled, and failed or aborted episodes are retained and distinguishable rather than discarded
- [x] #9 Only the active wrist camera streams during capture, enforced via the arbiter
- [x] #10 A dataset inspection tool reports episode count, duration distribution, success rate and starting-offset coverage, and flags under-covered regions of the offset range
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Software implementation plan:
1. Add TeleopCaptureConfig to config.py (Optional, default None) with per-joint sign/offset mirror map, start-offset range, tube-support attestation field.
2. Add teleop_capture block to config/hardware.yaml inside # --- AA-23 --- markers.
3. Implement inchworm/teleop_capture.py: TELEOP_JOINTS (joints 1-5 only; joint 6 excluded), TubeSupportSource enum, precondition_check(), MirrorMapper, LeRobotWriter (local only, lerobot import with fallback), EpisodeRecorder (leader+follower state, wrist frames via arbiter, labels, seed+offset metadata), TeleopSession.
4. scripts/teleop.py: dry-run default, --live --speed-scale flags.
5. scripts/inspect_dataset.py: episode count, duration distribution, success rate, offset coverage histogram.
6. tests/test_teleop_capture.py: mock bus + camera, synthetic dataset, inspector output.
7. robot/docs/tube_fixture.md: fixture spec (owner deliverable, no invented dimensions).
ACs #1, #4, #5 need hardware; left unchecked with commands listed.

2026-11-15 demo-start handoff build (owner decisions 2026-11-15): A) cycle_replay: Frame keeps the timeline tube state; preflights (frame, stream, start adjustment) can model the tube (interpolated between frames). B) replay_cycle.py --stop-after-frame N [--tube-in-grippers]: runs frames 0..N, no end fold press, exits with both arms holding (no torque-off sweep), writes out/handoff_state.json (goal/final ticks, q, locks, tube transform). With the tube the abort release never touches the grippers. C) inchworm.handoff (no LeRobot): state load/validation, holding-pose check, rate-limited catch-up (5 deg/s), mesh check of each goal segment with the tube where the cycle left it. D) plugin tube_support=handoff: no torque-cut, refuses unless both arms torque ON at the state pose (tol placeholder 3 deg) and the right gripper LOCKED; catch-up replaces the start gate; mesh hits hold the last goal; disconnect = support tube + both arms, Enter, joints 1-5 of BOTH arms off, grippers untouched. E) sim tests + dry-run; no live run (first live step without the tube, owner present).
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
AC#1 (low-speed mapping verification): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2

AC#4 (fixture validation): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --validate-fixture
  See also robot/docs/tube_fixture.md (owner deliverable for dimensions).

AC#5 (backdrive effort and latency): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --measure-latency

AC#2 checked: 26 tests in tests/test_teleop_capture.py pass; gripper excluded at code level (TELEOP_JOINTS has no gripper key; MirrorMapper and EpisodeRecorder raise GripperInTeleopError if gripper key appears).

AC#3 checked: 9 precondition tests cover all refusal paths.

AC#6 checked: dataset written locally; info.json contains hub_push=False.

AC#7 checked: seed and start_offset_deg in every EpisodeMeta.

AC#8 checked: all three labels (success/failure/aborted) retained and distinguishable.

AC#9 checked: mock arbiter verified called with follower camera role only; leader camera role never acquired.

AC#10 checked: inspector reports episode count, duration distribution, success rate, per-joint offset coverage histogram with under-covered bins flagged. Output pasted in test_teleop_capture.py::test_inspect_dataset_runs.

Owner 2026-10-27: no designed fixture; the owner will prop the tube up simply for demonstrations. AC#4 (fixture specified and validated against the real crawl configuration) is NOT changed: the 'documented build' will be a short description of the prop setup, and the frame/joint-pose comparison still has to be done once it exists.

2026-11-13 owner decisions (teleop/leader):
- Leader calibration: software mapping, no EEPROM writes. Both arms are placed by hand in the same pose, at 2-3 poses; the script reads leader + follower (both read-only) and fits per-joint sign and offset for joints 1-5. Joint 6 is excluded.
- First live teleop tries WITHOUT the mesh collision check (owner: "I do not believe it will be an issue"). Scoped to the teleop path only, via an explicit per-run flag, logged. Joint/soft/EEPROM limits stay enforced. Measured cost if re-enabled: 2.2 ms median, 18 ms p95 per pose check (first_only).
- lerobot-calibrate may rewrite servo memory (owner OK). Agent note: on the leader that is harmless. On the followers it would CHANGE homing offsets and limits that hardware.yaml zeros, the fold and the AA-45 timeline depend on. Plan instead: generate the LeRobot calibration file from the current follower EEPROM, so LeRobot writes back identical values.
- Tube holding during demos: the other arm holds the tube, gripper LOCKED and torque on (tube_support_source = locked_arm).

2026-11-13 owner: (1) handoff point confirmed = end of each regrip "move under target" (left: timeline frame 42), torque on in both arms, other arm LOCKED; code locks + re-plans from actual pose after. (2) Train the LEFT arm first (follower=left, leader=dedicated leader arm).

2026-11-15 OWNER DECISIONS (demo start with the tube, AA-23/AA-44):
1. Route A: the real cycle carries the tube to the left-regrip handoff and stops there (both arms holding; right arm extended and LOCKED on the tube at its regrip pose, frames 30-43: pan 0, lift ~-69, elbow ~-8, wrist_flex ~-11, roll 0 deg; tube lifted ~170 mm). The tube is available; the attachment point matches the simulation.
2. The left arm starts where the LEADER drives it (owner: no good way to get the leader in the exact pose).
3. Catch-up = option A (owner, replacing the 10 deg start gate on this path): the powered left arm moves slowly (proposed ~5 deg/s) from its handoff pose to the leader's pose, then follows normally. The agent's proposal accepted with it: the mesh collision check is ON for this path, with the tube modelled where the cycle left it (~2 ms/step). Owner: 'In real life I'll try to start as close to the real pose as possible but this way it's easier for me.'
STILL TO CONFIRM with the owner: (a) stop frame = 42 (end of left 'move under target', 50 mm standoff, left gripper unlocked and clear); (b) end of session = support tube + right arm, joints torque-off with the right gripper still LOCKED on the key (re-lock/finish the cycle is a separate task).
Build plan (not started): replay_cycle.py --stop-after-frame N leaving both arms holding on exit; plugin 'handoff' mode taking over the powered left arm (no torque cut; refuses unless the right arm reads its regrip pose and LOCKED) with the slow catch-up under the mesh check; sim tests + dry-run; first live step WITHOUT the tube to measure that servos keep holding after the script exits (assumed, not measured).

2026-11-15 owner confirmed both: (a) stop frame 42 (end of left 'move under target', 50 mm standoff); (b) end of session = support tube + right arm, joints of both arms torque-off, right gripper stays LOCKED on the key; re-lock / finishing the cycle is a separate task.

2026-11-15 demo-start handoff BUILT + sim-tested (NOT run on hardware):
- cycle_replay: Frame.tube (timeline tube_motion @ tube_base); preflight/approach_problems/fold_press_preflight(tube=...) and stream_preflight(frames=...) model the tube where the timeline puts it (rigid slerp between frames). Segment.s_at for the tube position on the fluid path. ReplayRunner.release(grippers=False) never touches the grippers.
- scripts/replay_cycle.py --stop-after-frame 42 [--tube-in-grippers]: plays frames 0..42 (fluid), stops, writes out/handoff_state.json, exits with BOTH arms holding (final torque-off sweep skipped only on that deliberate stop). Refuses a stop frame that is not one-gripper-open or has no tube pose; --tube-in-grippers needs --stop-after-frame and the fluid mode; it skips the fold press, and the abort path releases joints only. Dry-run both variants: preflights 0 problems, sim stop with both arms torque on.
- FOUND: with the tube modelled, the DexArm head in its MODEL pose intersects the tube at the fold (GripperAxis/GripperFinger* vs tube_1 at frames 0-5). The tube-aware stream preflight checks SO-101 pairs only (as AA-45 plan validation and preflight()), and the YES prompt asks the owner to confirm the DexArm head is clear of the tube and its lift path. Owner should confirm this choice.
- src/inchworm/handoff.py: state file (schema inchworm.handoff/1), arm_problems (joints 1-5 torque ON, within POSE_TOL_TICKS=34 (~3 deg, UNVERIFIED placeholder) of the stop pose), state_problems, catch_up_target, CollisionGate (real cell mesh, tube at the stop pose, other arm where it reads; fail closed; samples every 2 deg), joints_off (never servo 6).
- Plugin tube_support=handoff: refuses --robot.no_collision_check; follower_preflight without the torque-off requirement; locked_arm check of the other arm + handoff checks + follower gripper NOT in its locked band + present pose clear in the model + stop pose inside the teleop limits. YES prompt, then TeleopCore.adopt_holding (speed/accel/torque-limit registers only, no torque cut, no goal write). Catch-up at catchup_deg_s (default 5, max 10) replaces the start gate; a mesh hit holds the last goal and later resumes with a slow catch-up. disconnect: support tube + both arms, Enter, joints 1-5 of BOTH arms off, grippers untouched.
- Tests: tests/test_handoff.py (12), lerobot_env plugin tests +9 (40 total). Main suite passes.
- Measured mesh check at the stop pose (real model, tube): median 2.9 ms, p95 6 ms per pose.
- Next (owner present): 1) dry-run printout review; 2) first live step WITHOUT the tube: replay_cycle.py --stop-after-frame 42 --live ..., check both arms keep holding after exit (read-only hold_tube.py status --arm left/right), then a handoff teleop run with TUBE=... (needs a launcher flag set: --robot.tube_support=handoff, no no_collision_check); 3) then with the tube (--tube-in-grippers).

2026-11-15 owner decision: do not collision-check the DexArm on the demo-start path; it will be out of the way (prompt text updated). Added robot/lerobot_env/teleop_handoff.sh. No-tube dry-run of --stop-after-frame 42 re-run: preflight 0, fluid 0, sim OK, both arms holding.

2026-11-15 FIRST LIVE no-tube demo-start rehearsal (owner present): OK.
- 18:28 first attempt refused (left shoulder_lift +10.3 deg to start; owner placed the arm by hand). Earlier refusal: right gripper torque left on by hold_tube release without --gripper-off (released with --gripper-off).
- 18:30 replay_cycle.py --stop-after-frame 42 --live: frames 0..42 fluid, worst path error 17 ticks (left) / 3 (right), handoff_stop, state written; script exited, both arms holding.
- Found: lerobot_env lacked python-fcl -> handoff refused safely (mesh model not loaded). Fixed: lerobot_env depends on inchworm-robot[model].
- 18:37 teleop_handoff.sh: connect checks passed ~6.5 min after the stop: left [1973,2198,2196,1170,2060] vs stop present [1973,2192,2197,1170,2060] (max drift 6 ticks); right [2064,1727,1944,1862,2054] vs [2064,1728,1944,1862,2054] (1 tick); right gripper 2036 (locked band). The servos DO hold after the script exits (measured, no tube load). handoff_adopted, catch-up done in ~2 s (84 mesh checks, mean 8.9 ms), followed the leader ~36 s, no collision_hold, no clamps; hold -> released, joints 1-5 of both arms off, grippers untouched. Owner: 'it worked'. Logs: calibration_data/replay_cycle_2026-11-15.jsonl, teleop_lerobot_2026-11-15.jsonl.
- Next: the same with the tube (--tube-in-grippers), owner present.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
Rewritten 2026-10-24 with owner approval (decision-1, plan items A-C). The earlier version was 'Leaderless demonstration capture'.
---
<!-- COMMENTS:END -->
