---
id: AA-23
title: Cross-arm teleoperated demonstration capture with tube fixture
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:12'
updated_date: '2026-11-17 03:24'
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

2026-11-15 19:28Z first WITH-tube attempt ABORTED ~4 s after YES, before the arms moved (present ticks after = start ticks) and before any gripper lock: ServoChecksumError servo 2 (expected 0x7B, got 0xFB; arm not named in the message). The hold then failed on arm_right servo 5 (expected 0xDA got 0xEA). Owner released: all 12 servos torque 0, both grippers open (L 1567, R 1557). Earlier: one servo-2 checksum abort on 2026-11-12 (0xF9 vs 0xFB). Read-only status of both arms after that was clean. Hardware anomaly: asking the owner before more probes (AGENTS.md rule 8). No retry was added to the motion path.

2026-11-15 19:31Z WITH-tube cycle to the handoff OK on rerun (no checksum error this time): both grippers LOCKED on the tube keys (L 2027, R 2029), carry frames 0-24 worst path error 36/34 ticks, left unlock 1606, frames 24-42 worst 16/6, read_errors 0, handoff_stop at frame 42 with present within 13 ticks of goal; out/handoff_state.json tube_in_grippers=true. Right arm now holds the tube alone. Next: teleop_handoff.sh with the tube.

2026-11-15 OWNER SPEC for rapid-fire handoff recording (record_handoff.py; NOT built yet):
- Started after replay_cycle.py --stop-after-frame 42 --tube-in-grippers. Own loop on the LeRobot dataset API (add_frame/save_episode/finalize; stock lerobot-record has fixed-time episodes + timed reset, no press-to-start, no return).
- Keys: r = start recording, s = stop + save, d = stop + discard, q = quit (then the normal release). Recording includes the slow catch-up (owner OK). The owner moves the leader to the new start pose before pressing r.
- One demo = left arm INSERTION into the next attachment point, up to just before locking (lock is not in the demo; gripper never in action/observation; stays open).
- After s: save (arms hold while encoding), then automatic return, not recorded: FIRST a straight move away from the tube (out of the key), then a straight move back to the left start pose (NOT retracing the demo: owner says demo paths have extraneous motion). The RIGHT arm also resets to its stop pose every time. All mesh-checked, slow.
- Wrist camera of the left arm only.
Open points to confirm with the owner before building:
(a) retract direction/distance: in the timeline the insertion (frames 42->48) is a straight +50 mm along the left arm base z (FK; tool_frame_offset unverified). Proposal: retract along -z (reverse of the insertion axis) by the insertion depth (~50 mm) via IK, then a joint-space line to the start pose.
(b) right-arm reset order and model: only after the left gripper is out of the key and back; the right arm carries the tube, so its check must move the tube with the right gripper (the gate today has a fixed tube pose).
(c) frames 48/49 (inserted) pass the mesh gate in the model, but real near-key poses could be flagged and held during demos; watch the collision_hold events on the first runs.
(d) return speed proposal 10 deg/s; first live return owner-supervised (dry-run rule).
Physical state at this note: after the with-tube cycle, the right arm holds the tube alone (both arms torque on) unless the owner has since run teleop_handoff.sh and released.

2026-11-15 19:34Z WITH-tube handoff teleop OK (owner-run teleop_handoff.sh): connect checks passed (right gripper 2036 ticks = locked band, both arms on stop pose: left read [1973,2198,2196,1172,2060]), adopted without torque cut, catch-up done in 3 s (104 mesh checks, mean 10.9 ms), followed the leader ~2 min with 0 collision holds and 0 clamps, released joints 1-5 of both arms, right gripper left locked. Demo-start handoff with the tube is now working end to end.

2026-11-15 OWNER CONFIRMED record_handoff open points: (a) retract = straight -z of the left arm base frame (reverse of insertion), 50 mm, orientation held, at 10 mm/s, then a joint-space line to the left stop pose; (b) right arm resets to its stop pose after the left is back, tube modelled rigidly carried by the right gripper, refuse and ask if right drift >5 deg on any joint; (c) mesh check on every demo goal kept, holds logged per episode, no loosening; (d) return 10 deg/s, dry-run first, first live return owner-supervised with typed YES; episode safety cap 3 min (auto stop + save). Building now.

2026-11-15 BUILT rapid-fire handoff recording (not yet run live):
- robot/lerobot_env/record_handoff.py (+ record_handoff.sh): own loop on the LeRobot dataset API. Keys r record / s save / d discard / q quit (idle only); 3 min auto stop+save; left wrist camera; arms hold while idle and while saving; typed prompts pause the key listener.
- src/inchworm/handoff_return.py + InchwormFollower.handoff_return(): plan first, fully mesh-checked (CarriedTubeModel: tube rigid with the right gripper, = handoff tube at the stop pose; arm=None so the moving tube is checked against the left arm too), then (1) left straight -z of its base frame, orientation held, 10 mm/s, (2) joint line to the left stop pose 10 deg/s, (3) right arm to its stop pose 10 deg/s. Refuses (nothing moved) on right drift >5 deg, IK unreachable or jump, limits, any contact. During a leg, present >45 ticks behind the goal = stop and hold (TRACK_TOL_TICKS, agent choice, UNVERIFIED). First return of a session needs typed YES. Grippers never commanded (tests assert goal/torque of servo 6 unchanged on both arms).
- AGENT REFINEMENT TO CONFIRM WITH OWNER: the straight retract is 50 mm but never below the stop-pose height (which is 50 mm under the key); a shallow demo end pulls out only to that height. Without this, a return from the stop pose itself was refused (IK at -47 mm) and shallow ends would dip below the stop pose.
- Dry run (record_handoff.sh --dry-run; real cell mesh + out/handoff_state.json, no hardware): returns from timeline frames 42-49 all plan clean (frame 48/49: 50 mm out, 51 mesh checks).
- Tests: robot/tests/test_handoff_return.py (8: carried tube, straight/monotonic/orientation-held retract on the real chain, stop-height cap, drift refusal, contact refusal, pace + gripper untouched, resist = hold) and lerobot_env plugin +4 (return both arms, refusals move nothing, session r/s/d + 3 min-cap episodes in a real LeRobotDataset). Main 1855 passed, plugin 44 passed.
- Next (owner present): replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live ..., then record_handoff.sh; first return supervised.

2026-11-15 owner-approved reset helper: robot/reset_start.sh (show state, typed YES that the tube is out of the grippers or hand-held, hold_tube release --gripper-off on both arms (each asks its own YES), open any non-open gripper via gripper_ends --action open --no-tube, then loop: owner folds by hand + read-only scripts/start_check.py until READY). start_check.py is read-only (torque, gripper band, each joint within 10 deg of clamped frame 0); tests/test_start_check.py. reset_start.sh itself not yet run live. A start_check run at ~21:10Z read READY (all 12 torque 0, both grippers open, left/right within 10 deg).

2026-11-15 21:37Z FIRST live record_handoff session (owner): cycle to frame 42 OK (21:36:59Z), connect + adopt OK with camera, episode 0 started 21:38:26, catch-up done in 7 s (122 mesh checks, 5.5 ms mean), followed ~47 s with 0 collision holds / 0 clamps. Owner: pressing q could not stop the recording (by design q was idle-only; s/d were the episode keys). Ended with Ctrl-C: hold, then released joints 1-5 of both arms; dataset insert_left_20261115T213741Z has 0 episodes (unfinished episode discarded). No automatic return ran. FIX: q while recording now = save the episode and quit (no return; arms hold, then the release prompt); help + on-screen prompt updated; test added. Also: start_check.py gets --live (read-only; dry-run without it, as the script smoke tests require); reset_start.sh uses --live. Main 1860 passed, plugin 45 passed.

2026-11-15 21:56-22:02Z FIRST SUCCESSFUL live rapid-fire session (owner): reset_start (both grippers torque off; right still read locked 2036 but cycle started with it open 1557, i.e. it was opened in between), cycle to frame 42 OK (worst path 33/30 then 16/5 ticks), record_handoff with camera: 2 episodes SAVED (583 + 1060 frames, 37 s + 68 s), datasets/insert_left_20261115T215810Z (2 episodes, 1643 frames, wrist video 4 MB). Both automatic returns ran clean: ep0 left straight out 50 mm (worst track 19 ticks) + joint line (18); ep1 out 43 mm (stop-height cap) (16) + line (13). Right arm drift < 10 ticks both times, so no right leg was needed. 0 collision holds, 0 clamps all session; ended idle with q, released joints 1-5 of both arms.
OBSERVATION (data quality, to raise with the owner): episode 1 catch-up lasted 25 s (1011 mesh checks): the leader kept moving faster than 5 deg/s, so the follower stayed rate-limited and the recorded ACTION (leader) differed from what the arm did for that time. Options: move the leader to the follower before r (owner's plan), a faster catch-up (max 10 deg/s), or start recording only after catch-up. Also the 3 s settle wait ran out on the first retract (final error 13 > 10 ticks): harmless, adds ~3 s per return.

2026-11-15 owner decisions: (1) option 3: record only after the catch-up. record_handoff: after r the arm catches up (not recorded; log recording_start with catchup_s), then frames are added; the 3 min cap counts from recording_start; an episode stopped during catch-up has nothing to save = discarded. (2) First-return 3 s settle pause OK as is. (3) Retract never below the stop-pose height: CONFIRMED.
FOUND + FIXED (data): in the first dataset (insert_left_20261115T215810Z) the saved ACTION for elbow_flex / wrist_flex (and some shoulder_lift) is 360 deg off observation.state (leader map wraps to another 4096-tick turn; the follower used the nearest representative, so motion was correct but the saved numbers were not). Wrapped |action-state| p99 was 2/37/23/15/5 deg (the 37/23 = the old recorded catch-up). Now record_handoff saves the action unwrapped to the state's turn (unwrap_to). The two saved episodes need the same unwrap before training (or re-record). Tests: plugin 47 (catch-up not recorded + action within 2 deg of state; unwrap_to).

2026-11-15 owner: RE-RECORD (the 2 episodes of insert_left_20261115T215810Z have the 360 deg action offset; do not train on them). Added live wrist view to record_handoff: MJPEG at http://localhost:8765 (127.0.0.1 only) serving the frames the recorder already grabs (never opens the camera: one-camera invariant); --preview-port 0 = off. The lerobot env's OpenCV is headless (no window support) and rerun is not installed, hence the browser. Plugin tests 48.

2026-11-16 owner report: the live view shows split images (halves of two frames). MEASURED in the recorded videos too, so not a viewer bug: datasets/insert_left_20261116T015012Z (2 episodes, 2102 frames) has ~352 frames that look torn (one neighbour's content above a row, the other's below; seam rows vary 40-420), insert_left_20261115T222125Z ~162 of 1761. Also exact duplicate frames (e.g. frames 2 = 3). The existing corrupt-frame filter (libjpeg warning / SOI-EOI in V4L2CameraTransport) does not catch these. Likely cause (UNVERIFIED): OpenCV V4L2 MJPEG decode into a reused buffer on a partial frame from the shared USB 2.0 link. Detector used: /tmp/tear2.py heuristic (not committed). NEXT (fresh session): confirm with a camera-only capture (no arms moving) that saves raw frames, then reject torn frames at grab (keep last good, like corrupt frames) and report the rate; decide with the owner whether the 2 datasets are usable or re-record.
2026-11-16 owner request: start pose wrist_roll of BOTH arms +2 deg counter-clockwise viewed from the side opposite the cables. Not done yet: needs (1) the tick sign of 'CCW from opposite the cables' measured (owner turns each wrist slightly CCW by hand, torque off, read-only status before/after), (2) regenerating the timeline (the plan holds roll at 0 for both arms; inchworm_plan measured_fold_pose sets q[4]=0) and re-running the preflights, (3) the handoff stop pose changes, so episodes recorded before the change are at the old roll (re-record after).

2026-11-16 03:15Z CAMERA (measured, read-only, no motion): the earlier 'torn frame' finding was WRONG (heuristic false positives). A stricter check finds 0 torn frames in both datasets. The real problem: the left wrist camera delivers only ~7-8 DIFFERENT images per second. cv2 cap.read() returns 30-60 frames/s, but 73-76 % are exact repeats (repeat runs of 3-4). Raw OpenCV, bypassing our code: 640x480 7.8 distinct/s, 1280x720 7.5/s; auto exposure 313 -> 6.5/s; manual 100 -> 8.5/s; manual 30 -> 8.2/s. So it is not exposure, resolution or our pipeline; the camera/usbip link gives ~8 new images/s (gap 183 ms p50). camera_probe reports '30.9 fps' because it counts reads, not new images. The split image in the browser view is probably the browser drawing a partly received MJPEG part (unconfirmed). Episodes so far = 30 Hz joints + ~8 Hz images (repeats). Ask the owner before more probes (rule 8). Options: check the camera at 640x480 in the Windows Camera app (rate there = whether usbip/WSL is the limit); different dock/port; accept 8 Hz.
2026-11-16 OWNER DECISIONS: (1) the start must NOT require precise placement: 'if I'm off, it should just go to the right starting position'. This replaces the 10 deg approach cap and covers a present pose slightly OUTSIDE the soft limits (live abort 02:44Z: right wrist_flex 902 ticks = -1.6751 rad vs soft limit -1.6581; only the grippers locked, no arm motion). (2) Start-pose wrist_roll (servo 5) of BOTH arms 2 deg counter-clockwise viewed from the side opposite the wires. doc-1: servo 5 cable exits down; a higher reading = clockwise viewed with the cable exiting away from the viewer, so CCW = LOWER reading: -2 deg = -23 ticks, q4 = -0.0349 rad (wrist_roll_sign +1 both arms). PLAN (next session): (a) inchworm_plan: start roll -2 deg both arms (measured_fold_pose q[4]; hold_pan_roll keeps it), regenerate out/inchworm/timeline.jsonl, rerun the preflights, check the stop pose. (b) replay_cycle approach: no degree cap; slow (<= 5 deg/s), mesh- and EEPROM-checked straight line; a start joint outside its soft limit is allowed only when it moves INWARD to an inside goal (goal limits unchanged); tests. (c) start_check.py READY to match. Re-record after (a).

2026-11-16 19:30Z DONE (dry-run/tests only, not yet live): (1) wrist_roll start -2 deg (owner: 2 deg CCW from the side opposite the wires) on both arms: inchworm_plan.START_ROLL_RAD; timeline regenerated (only wrist_roll changed, -0.0349 rad in every frame), planner OK 0 violations, demo-start dry-run: preflight 0, fluid preflight 0, simulation OK. New ticks frame 0 / stop frame 42: wrist_roll L 2062->2039, R 2055->2032 (other joints unchanged). (2) Automatic start adjustment from ANY hand placement: APPROACH_MAX_DEG removed. cycle_replay.approach_plan: one arm at a time (left first, else right first), straight joint line, mesh-checked at >=1 sample/deg, goal must be inside the soft limits; a start joint outside its soft limit is allowed (MotionGuard.check_trajectory inward_start, used only by move_to_start); slow APPROACH_TICKS_S=57 (~5 deg/s, unverified placeholder). ORDER CHANGED in replay_cycle --tube-in-grippers: grippers EMPTY during the adjustment, then a YES prompt to put the tube keys in, then the locks (the arms never drag the tube; also no lock before a possibly refused move). start_check.py: no degree window; READY = torque off + grippers open + approach_plan clear; prints the degrees to move and any outside-limit joints. 02:44Z abort pose plans fine. Tests: main 1863 passed, plugin 48 passed. Previous recordings start from the old roll: re-record. First live run of the new adjustment: owner supervises.

2026-11-17 owner: 3 deg CCW total, by MOVING THE ZERO. A -3 deg plan roll was refused by the planner (start grip 3.42 deg off the CAD middle point vs the 3.0 deg START_GRIP_TOL; -2.5 deg = 2.99 deg). So hardware.yaml wrist_roll_zero_offset_deg -3 deg on both arms (L 181.187->178.187, R 180.615->177.615; roll 0 = tick 2027 L / 2021 R), the fold_allowance roll centres re-expressed (+0.0524 rad), and inchworm_plan START_ROLL_RAD back to 0. Joints 1-4 of the timeline are identical to the original; the start/stop wrist_roll ticks are 34-35 lower than the original. Soft-limit ticks are now 582..3472 L, 576..3466 R (inside the EEPROM limits). The leader map works in ticks, so it is unchanged. Planner OK, demo-start and full-cycle dry-runs: preflight 0, fluid 0, simulation OK. Main suite 1863 passed, plugin 48 passed. Live 01:28Z run (before this change): the automatic start adjustment worked, left arm first, then the tube, locks and the handoff stop at frame 42.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
Rewritten 2026-10-24 with owner approval (decision-1, plan items A-C). The earlier version was 'Leaderless demonstration capture'.
---
<!-- COMMENTS:END -->
