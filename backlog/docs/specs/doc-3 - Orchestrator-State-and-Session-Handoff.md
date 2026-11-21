---
id: doc-3
title: Orchestrator State and Session Handoff
type: guide
created_date: '2026-10-20 21:37'
updated_date: '2026-11-21 15:29'
---
Live orchestrator state: what is in flight, what is blocked, what is uncertain, and how to resume cold. Read this **first** when picking up this project in a fresh session, then doc-1 (hardware truth) and doc-2 (change-control rules).

**Keep this current.** Update it via `backlog doc update doc-3` whenever a blocker clears, a task finishes, an estimate changes, or a decision is made. A stale doc-3 is worse than none, because it invites confident action on wrong information. If the Last verified date below is old, trust task statuses and `git log` over this prose and re-verify before acting.

**OWNER NEW PLAN 2026-11-21 (supersedes the AA-39 full test flow; owner authorised task updates).** Retrain BOTH arms as SEPARATE policies from two FROZEN training positions saved in `robot/config/training_poses.json` (commit e8b6ee9; loader `inchworm.training_poses`, test fails on drift). **These positions must not change after training unless the owner says so.** `train_left` = cycle frame 42 (right LOCKED holding, left open ~50 mm under its point): L [1972,2197,2184,1169,1988] R [2063,1730,1941,1865,1981]. `train_right` = frame 67 (left LOCKED at its gait pose = frame 49, right open under its point): L [1972,2337,1775,1438,1988] R [2063,2261,2140,1135,1981].
- **AA-54 recording (two programs, one per arm):** arms go to the arm's training position (no tube), owner adds the tube and inserts the HOLDING gripper's key, code locks it (AA-17), then rapid-fire teleop recording. Before each episode the holding arm moves the tube so the attachment point is rotated <= 2 deg (any axis) and translated <= 20 mm per axis (owner correction: NOT a per-joint jitter). 50 episodes per arm: 10 nominal + 40 perturbed. The right arm needs `leader_map_right.json` fitted first.
- **AA-55 run sequence:** frames 0..42 -> left policy -> left lock + look -> right unlock + out of the key -> left carries the tube to its train_right pose -> right to its train_right start -> right policy (own checkpoint, no joint-offset map) -> right lock + look -> both home.
- **Status (2026-11-21 ~06:45Z):** AA-54 recordings DONE live: `datasets/insert_left_v2_20261121T050907Z` (50 ep = 10 nominal + 40 perturbed, 23022 frames, seed 37722, perturbations.json) and `datasets/insert_right_v2_20261121T055710Z` (50 ep, 13137 frames). `leader_map_right.json` fitted (owner approved 6.5 deg fit tolerance: pan 5.54, roll 6.0 deg). Launcher checks the leader answers before moving (the first run failed because the leader was unpowered). **AA-25 training RUNNING** in the background since 06:31Z: `robot/out/train/run_v2_both.sh` (left, then right automatically; ACT, 20000 steps, batch 8, n_action_steps 20; ~2.2 step/s = ~2.4 h per arm on the RTX 3050 Ti after the owner fixed laptop power mode). Monitor: `cat robot/out/train/logs/run_v2_both.log` ('exit 0' per arm), `tail -c 300 robot/out/train/logs/<dataset>.log`. Resume if interrupted: `lerobot-train --resume=true --config_path=../out/train/<dataset>/checkpoints/last/pretrained_model/train_config.json`. Robot: torque off (end of right recording; tube state as the owner left it). AA-55 (2026-11-21 ~07:10Z): launcher `lerobot_env/crawl_two_policy.py/.sh` + `lerobot_env/tests/test_crawl_two_policy.py` BUILT (6 tests pass on the sim with a stand-in policy; finish refs = dataset demo-end medians; details in AA-55 notes). NOT yet: CPU dry run with the real 005000 checkpoints (waits for the right one), full suite, commit. AA-54 ACs 3,5 can now be checked from the live logs. `tests/test_constants.py` forbids bare 40/67/80 in src.
- **START HERE / next action (2026-11-21 15:00Z):** training DONE (both exit 0, 020000 checkpoints); AA-55 launcher committed e0d8e74 (suites 1960 + 99 pass). Dry run with the real checkpoints stops safely at the left lock check (9.4 / 18.5 deg vs 8 deg). OWNER DECISION PENDING: the 8 deg lock check vs one demo-end median rejects 11/50 left and 26/50 right demo ends (perturbed tube placements) - widen it, or rely on the from-actual-pose plan checks. Then: owner-supervised first live `bash robot/lerobot_env/crawl_two_policy.sh` (AA-55 notes).

**Previous state (2026-11-21 00:45Z), before the new plan:**  2026-11-21 00:45Z. **ROBOT STATE:** joints 1-5 of BOTH arms torque off (quit at 00:15:31Z); right gripper LOCKED (2036, tube presumably in it), left open. Start the next live session with `bash ~/893/robot/reset_start.sh`. Windows camera bridge server must be running in PowerShell (`py -3.12 camera_bridge_server.py --connect 172.26.80.62:50853`; it was running, PID 66604). **2026-11-21 00:11Z FIRST LIVE `full_policy_test.sh`: the LEFT policy FAILED** (3 attempts, owner r/r/q). The policy barely moved and went the wrong way (elbow 193->199 deg; demos 193->170; the 11-20 03:00Z run 193->167). Ruled out: right-arm placement (= training within 3 ticks), camera dead/black (29 new images/s, mean 128), camera swap (owner covered the left camera; only busid 9-4 changed), speed cap (commanded = target). Suspects left: the scene in view (tube in the right gripper, DexArm, lighting) or policy brittleness. Proposed next: replay to frame 42, then compare left-camera frames against training frames at the same pose. **OWNER 2026-11-21: wrist_roll zero moved 1 deg more CCW on both arms (6.5 deg in total; L 174.687, R 174.115; left roll soft lower limit tightened to -2.2036 rad).** Dry-run replay OK; NOT live. The policy was trained at the old roll (1 deg off). Details in AA-39 notes. **Resume with:** "Read doc-3 and AA-39 notes; decide with the owner on the left-policy failure (camera-frame comparison at frame 42, or re-record/fine-tune at the new roll) before another full_policy_test.sh."

**START HERE (fresh orchestrator session).** Read this section, then `backlog task view <ID> --plain` only for the task you pick up. Do not read old session transcripts. Session rules: see AGENTS.md "Session and cost discipline".

### Owner decisions (recorded, do not revisit)
0. Cameras (2026-11-06): all three cameras withhold frames with a libjpeg corruption warning (scanner 1280x800 flagged frames are ~99% grey; the owner chose to exclude flagged arm frames too). Damaged frames never raise or stop a run: `CameraCapture.grab_valid()` skips them (used by level_scan, the LeRobot frame source and teleop capture). Cost: only the skipped frames (~8-17% on the arms).
1. E-stop HOLDS in place.
2. The tube advances AWAY from the DexArm (`cycle_runner.advance_physical: away_from_dexarm`; live preflight checks the sign against AA-16 calibration).
3. Cura uses Rotrics' official settings (AA-34 Done). No tube fixture design. Keep the one-camera rule.
4. **The owner cell URDF `CAD/robot_urdf_new/` is THE robot model** (both SO-101s with custom gripper and camera mounts, DexArm + laser camera, base mount `baseforreal`, tube `tube_1`). Adding limits is fine; nothing else to rebuild.
5. The table is NOT a collision object. "Go with the real fold": model contacts at the measured fold are accepted only within 0.05 rad of it. Wrist roll: our 0 = cell URDF -127 deg. Tube = sections with one attachment point each (3 now, +1 per printed section).
6. Owner prefers the official Feetech route for servo firmware.
7. DexArm collision: keep the mesh cell check (owner asked about a simple box; rejected with numbers: the fold reaches Y 67 mm, the DexArm base extends to Y 99 mm, and the tube crosses any box).
8. (2026-11-09) Both arms rebuilt with powered STS3215 1/345 on every joint (same model as the first set). A third, dedicated encoder-only **leader arm** exists for teleop (AA-44); it changes decision-1's cross-arm plan. Ask the owner before reworking AA-23 around it.

### Owner decisions 2026-11-15: demo start with the tube (AA-23/AA-44; full text in both tasks' notes)
- Route A: the real cycle (`replay_cycle.py`) carries the tube to the left-regrip handoff and STOPS; right arm extended and LOCKED on the tube (pan 0, lift ~-69, elbow ~-8, wrist_flex ~-11, roll 0 deg; tube ~170 mm up). Tube available; attachment point = simulation.
- The left arm starts where the LEADER is: slow catch-up (~5 deg/s proposed) of the powered left arm to the leader, replacing the 10 deg start gate on this path, with the mesh collision check ON (tube where the cycle left it).
- CONFIRMED by the owner 2026-11-15: stop frame 42; end of session = support tube + right arm, joints of both arms off, right gripper stays LOCKED (re-lock / finish the cycle = separate task).
- **BUILT 2026-11-15 (sim-tested, dry-run OK, NOT live):**
  - `cd robot && uv run python scripts/replay_cycle.py --stop-after-frame 42 [--tube-in-grippers]` (add `--live --allow-unverified-soft-limits "..."`) plays frames 0..42 fluidly and stops. Both arms stay holding after exit. It writes `robot/out/handoff_state.json`. `--tube-in-grippers` models the tube in every preflight, skips the fold press, and never switches a gripper off.
  - The LeRobot follower `--robot.tube_support=handoff` (WITHOUT `--robot.no_collision_check`) uses `inchworm.handoff`. It has no torque cut. It refuses unless both arms hold at the stop pose (+-34 ticks, UNVERIFIED placeholder), the right gripper is in its locked band and the left is not. Catch-up runs at `--robot.catchup_deg_s` (5), with a mesh check per goal (tube where the cycle left it; ~3 ms). A hit = hold. At the end, joints 1-5 of BOTH arms go off and the grippers are untouched. Launcher `robot/lerobot_env/teleop_handoff.sh`.
  - **Owner decision 2026-11-15:** no DexArm collision check on the demo-start path; the owner keeps the DexArm out of the way (the tube preflight checks SO-101 pairs only; the YES prompt reminds). Launcher: `bash robot/lerobot_env/teleop_handoff.sh`. No-tube dry-run of `--stop-after-frame 42` re-run 2026-11-15: preflight 0, fluid 0, sim OK, both arms holding.
  - **2026-11-15 18:30-18:38Z no-tube LIVE rehearsal OK** (AA-23 notes): stop worst path error 17/3 ticks; hold after exit measured (<= 6 ticks drift); catch-up 2 s, mesh check mean 8.9 ms; release of both arms OK. lerobot_env now installs `inchworm-robot[model]` (fcl was missing; the gate refused safely).
  - Start-up gotchas: every servo incl. both grippers must be torque OFF before replay_cycle (`hold_tube.py release --gripper-off`); each joint within 10 deg of the fold start pose (place by hand).
  - **2026-11-15 WITH tube:** 19:28Z abort ~4 s after YES on ServoChecksumError (servo 2 0x7B->0xFB, then right servo 5 on the hold; single-bit-like; 2nd ever, first 2026-11-12, also servo 2), nothing moved, released. 19:31Z rerun OK: both grippers locked on the keys (L 2027, R 2029), carry worst path error 36/34 ticks, left unlocked, stop at frame 42 (present within 13 ticks). If checksum aborts recur, the owner chose rerun; offered options: check cabling/power, or a bounded read retry (needs owner approval).
  - 19:34Z `teleop_handoff.sh` WITH tube OK (see AA-23 notes).
  - Owner confirmed open points (a)-(d) + 3 min episode cap; `record_handoff.py` built (AA-23 notes 'BUILT rapid-fire'). **Next:** first live recording session, first return supervised.

### In flight: LeRobot plugins (AA-44 / AA-23): written + sim-tested 2026-11-15, NOT run on hardware
- `robot/lerobot_plugins/lerobot_robot_inchworm/` (robot `inchworm_follower`, teleoperator `inchworm_leader`) is installed editable in `robot/lerobot_env`. It does NOT use LeRobot's Feetech bus, so it never writes servo memory and no LeRobot calibration file is needed.
- Follower: refuses before opening any port without `--robot.no_collision_check`, `--robot.allow_unverified_soft_limits` and `--robot.tube_support` (`no_tube` | `locked_arm`; locked_arm = a read-only check that the other gripper is torque-on in its locked band and its joints are torque on).
  - Then follower_preflight, then a YES prompt. Torque goes on only at the first action that passes the 10 deg start gate.
  - Stop file `robot/out/STOP_TELEOP` = Ctrl-C. Disconnect = HOLD, Enter, torque off joints 1-5. The gripper is never in action or observation.
  - Wrist cam via the arbiter: grey copied to 3 channels (LeRobot 0.6.1 rejects (H,W,1) uint8; tell the owner). Log `calibration_data/teleop_lerobot_<date>.jsonl`.
- Tests: `cd robot/lerobot_env && uv run pytest tests/` (18 plugin tests incl. LeRobot's teleop_loop and real record() to a local video dataset).
- **2026-11-15 16:03Z live run OK** (owner: 'it worked'; log `calibration_data/teleop_lerobot_2026-11-15.jsonl`, hold then release, no errors). Launch with `bash robot/lerobot_env/teleop_left.sh` (pasted multi-line commands broke; extra flags pass through, e.g. `--robot.camera=true`).
- **16:08Z camera run OK** (V4L2 fix; ~33.5 ms loop). Tools ready, not run live: `bash robot/lerobot_env/record_left.sh` (local only, `robot/datasets/<DATASET>_<UTC>/`; keys n/r/q) and `cd robot && uv run python scripts/hold_tube.py status|lock|release --arm right --live` (lock needs `--tube-in-gripper` or `--no-tube`; the arm keeps holding after the script exits; `TUBE=locked_arm bash record_left.sh` then checks it).
- **Next:** record no_tube; hold_tube rehearsal; then record with the tube. Older plan text: camera run, then `lerobot-record` no_tube, then locked_arm (no tool yet locks and holds the right arm during recording; build it first). Original command, for reference: from `robot/lerobot_env`, run with the cell as for teleop_live (no tube, other arm torque-off):
  `uv run lerobot-teleoperate --robot.type=inchworm_follower --robot.id=left --robot.camera=false --robot.tube_support=no_tube --robot.no_collision_check="owner 2026-11-13: first teleop without mesh check" --robot.allow_unverified_soft_limits="owner <date>: <reason>" --teleop.type=inchworm_leader --teleop.id=leader --fps=30`
  Then with the camera, then `lerobot-record ... --dataset.push_to_hub=false --dataset.repo_id=local/<name> --dataset.single_task="..."` with the right arm LOCKED (`--robot.tube_support=locked_arm`; needs `--robot.follower=left`).
- Not checked: the read-only `scripts/teleop_live.py --check` start gate is the quickest pre-check; LeRobot's own loop calls get_observation before the first action, so a refused start gate raises after connect (nothing energised).

### In flight: AA-44 leader teleop (2026-11-15) — ACs #2-#4 checked; #1 open
- **Live teleop WORKS (owner-supervised).** The left follower tracks the leader on joints 1-5 at about 30 Hz. Owner: every joint moved the correct direction (AC #4 checked). `leader_map_left.json` is verified (all signs +1).
- Run it from `robot/`: `uv run python scripts/teleop_live.py --live --allow-unverified-soft-limits "owner <date>: <reason>" --no-collision-check "owner 2026-11-13: first teleop without mesh check"`.
  - Defaults are the owner's choice from 6 tuning runs: speed cap 250 ticks/s, no smoothing (`--smooth 1`), acceleration 100.
  - `--check` is read-only. Ctrl-C = HOLD, then the owner releases.
  - The log date is the laptop's local date (`teleop_live_2026-11-14.jsonl` holds the 2026-11-15Z runs).
- **Parked (owner: "good enough for now"):** fix for "it seems to only move one joint at a time" on branch `aa-44-coordinated-speed` (commit c653de7, pushed, NOT run live). Cause in the log: on fast leader moves each lagging joint ran at the 250 cap on its own, so short moves finished first. The fix scales each joint's Goal_Speed by its share of the largest remaining distance; the new test fails on main and passes on the branch. To resume: merge the branch, run the suite, then one supervised live run.
- Not done: AA-44 AC #1 (doc-1 entry; gearing is not needed per the owner), finalization. The right-arm leader map is not fitted.
- **Next (fresh session): LeRobot plugins.**
  - Follower robot = teleop_live core + left wrist cam via the arbiter, mono (H,W,1).
  - Leader teleoperator = the leader map.
  - A LeRobot calibration file from the current follower EEPROM.
  - Then `lerobot-teleoperate`, then `lerobot-record` with the right arm LOCKED holding the tube.
- Open question for the owner: hardware.yaml `servo_tick_limits` for shoulder_pan/lift differ from the live EEPROM on both arms (e.g. left pan yaml 674..2527, EEPROM 672..3340). Teleop clamps to the tightest of soft, EEPROM and yaml.

### Owner decisions 2026-11-13: teleop and leader (AA-44 / AA-23), recorded in both tasks' notes
- Leader calibration: software sign/offset fit from 2-3 matched hand poses (leader and follower both read-only, no EEPROM writes). Joint 6 excluded.
- First live teleop WITHOUT the mesh collision check (per-run flag, logged); joint, soft and EEPROM limits stay enforced. Measured cost if re-enabled: 2.2 ms median, 18 ms p95 per pose.
- lerobot-calibrate may rewrite servo memory (owner OK). Plan: generate the LeRobot calibration file from the current follower EEPROM, so the values it writes are identical (zeros, fold and the AA-45 timeline depend on them).
- During demos the other arm holds the tube, LOCKED and torque on (`locked_arm`).
- Handoff proposal (awaiting owner OK): the cycle pauses at the end of each regrip's "move under target" (pre-insertion pose, 50 mm standoff). Teleop or the policy takes the moving arm, then code locks and re-plans from the actual pose. Torque stays on throughout.

### Done: AA-52 touching fold (2026-11-13)
- The method was delegated by the owner ("use your best judgement") and accepted after the run ("worked great"). The replay stream keeps the elbow cap (`FOLD_ELBOW_CAP_TICKS` L 3081 / R 3036). At the start (after the final adjustment) and at the end, `ReplayRunner.fold_press` drives ONLY the elbows at 100 ticks/s with torque limit 150/1000 toward contact + 15 (L 3135 / R 3090) and holds; then the owner answers a y/n prompt (logged). `fold_unpress` returns to the cap and restores torque 500 before the stream starts. Warnings (> 20 ticks short of contact) are logged only. `--no-fold-press` disables it.
- Live 2026-11-13 03:47Z: both presses settled in 0.8 s at L 3123 / R 3078 (3 ticks past the hand-measured contact), elbow load 104, current about 5 raw; owner y twice; torque released without errors.
- Handoff AA-51 is still 'active' in the human queue, although its work is done (AA-50 runs). The owner can finish it in the handoff tab; then `human_verify` it with the AA-50 evidence. Do not mark it verified otherwise.

### Done: AA-50 full-cycle movement test (2026-11-13)
- Run: `cd robot && uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits "owner 2026-11-13: AA-50 fluid replay"` (arms roughly in the fold, no tube). Without `--live` it is a dry-run (preflight + simulation). Log: `calibration_data/replay_cycle_<date>.jsonl`.
- Default = fluid: the whole path, both arms, is checked against the soft limits, EEPROM limits and mesh before the single YES prompt; then it streams at 50 Hz (sync-write position + speed) at animation tempo, slowed only where a joint passes 650 ticks/s (about 18 s of motion). `--tempo` scales the speed; `--step-by-step` is the old checked frame-by-frame mode.
- **OWNER DECISIONS 2026-11-13 (do not revisit):**
  - No per-point checks or automatic aborts once a motion is confirmed; readings (temperature, current, lag) are only logged. Only Ctrl-C, the stop file or a gripper-lock failure stop the run (hold, then release).
  - Per-run waiver for the unverified soft limits (`--allow-unverified-soft-limits`); `hardware.yaml` stays `unverified: true`.
  - The lower arm may touch its own shoulder at any pose. The robot makes the final adjustment to the start pose itself if each joint is within 10 deg. The owner dislikes needless refusals: ask before adding checks that block intended motion.
- Hardware note: single temperature reads of 70-150 C appear while moving (at rest the servos read 28-31 C). These are bogus readings, so do not act on them. One checksum error was also seen once.
### Gripper lock = 45-degree position lock (OWNER DECISION 2026-11-12; VERIFIED AA-49 2026-11-13)
- The locking piece slips on its screw at both ends (deliberate clutch), so no stall marks the lock. Lock = slow drive (150 ticks/s, torque 300) to open end + 512 ticks: L 2038, R 2041; LOCKED within 12 ticks; stopping short = FAULT + torque off. AA-49: key-in trials settle at L 2032-2034, R 2035, all in band; the owner confirmed by eye the key locked every time. `hardware.yaml gripper_lock.arm_*.unverified: false`. AA-43 and AA-17 Done. AA-41 is superseded (owner may decline it). Grippers' rest state after AA-49 was not reported (probably open, torque off).
### In flight: AA-43 (servo swap + recalibration of joints 4-6)
- **2026-11-12 late, AC #4 remainder:** hardware.yaml right gripper tick limits 1563 -> 1523 (was stale vs EEPROM); wrist zeros/signs and tick limits now cite their new-servo evidence (AA-47 landmark within 1-3 deg, owner-observed powered moves). `gripper_lock_probe.py` reworked for the new servos (150 ticks/s, settled-only classification, torque 50..500, restores speed/accel/torque, tests). **Proposed AA-49** (owner-supervised key-in/no-key lock trials, both arms, ~30 min) replaces AA-41's stale steps. **Next:** owner accepts and runs AA-49 (`accept AA-49`); then the agent proposes locked band + lock_torque_limit_raw per arm for owner approval, then checks AC #4. AA-41 stays deferred (ask the owner to decline it once AA-49 is accepted).
- **2026-11-12 AA-48 steps 5-9 done + gripper ends:** all powered moves of IDs 4-6 on both arms REACHED; owner confirmed every direction (doc-1 convention holds for both wrist_rolls and the right wrist_flex). New `GripperLockSM.close_empty` / `measure_end` + `scripts/gripper_ends.py`. Gripper ends measured (doc-1). Right gripper Min_Position_Limit 1563 -> 1523 (EEPROM, backup + read-back) and right open band -> 1504..1589 ticks (owner approved). **AA-43 AC #3, #5 checked; #4 open** (gripper locked band needs the key = AA-41; servo_calibration.unverified and the fold re-measure). AA-48 handoff verified 2026-11-12. State now: torque off on all 12; both grippers closed (L 2526, R 2499).
- **2026-11-12 AA-48 step 5 aborted, cause found:** left wrist_roll powered move hit a ServoTimeoutError on the goal read-back (reg 0x2A) in `set_torque_enable`. Read-only bus check: 300/300 reads per left servo, no timeouts. Supervised probe (`scripts/autoenable_probe.py`): the goal write itself sets Torque_Enable=1 (no motion, goal == present). Fix: one retry of the read-back on a timeout; torque OFF on any failure after the goal write (tests). Torque off on both arms, read back. **Next:** owner reruns AA-48 step 5 (left wrist_roll), then steps 6-9.
- Done: IDs assigned by the owner with `lerobot-setup-motors`; read-only census `robot/calibration_data/servo_census_2026-11-08T20-15-27.json` (12/12 at IDs 1-6, fw 3.9, error 0, current 0). AC #1, #2 checked.
- **Owner answers (2026-11-09):** IDs 1-3 on both arms are UNCHANGED (keep their 2026-10-25 offsets/limits and hardware.yaml calibration). Gripper caps are to be re-applied (done).
- **Grippers (ID 6) DONE 2026-11-09, both arms, torque off, nothing moved:** the owner cannot sweep them by hand (powered servo), both rest at the full clockwise stop = the old `closed_empty` stop. So the closed stop was homed to read the old closed_empty tick (left offset 1207 -> 2521, right offset -1914 -> 2494) and Min/Max_Position_Limit set to the previously written values (left 1521..2531, right 1563..2504). LeRobot gripper caps re-applied: Max_Torque_Limit 500, Protection_Current 250, Overload_Torque 25, P 16, D 32. Read back; nothing else changed. **ASSUMPTION, not measured:** the open end sits at the old open tick (L 1531, R 1576), so the hardware.yaml gripper bands stay valid. Verify with one supervised powered move before relying on them.
- **IDs 4 (wrist_flex) and 5 (wrist_roll) DONE 2026-11-09, both arms:** homed at mid-travel, swept, limits written with 30-tick margin and read back. Left: flex 831..3226, roll 550..3573. Right: flex 797..3191, roll 533..3577. hardware.yaml updated (tick limits, mid-range zeros, wrist_roll soft limit capped at +-2.2165 rad = the cell URDF's modelled range). `test_cell_collision._measured_fold` pins the OLD wrist zeros for the 2026-10-25 fold ticks; re-measure the fold on the new servos and drop the pin.
- **Powered move + direction check, 2026-11-10:** `scripts/powered_move_check.py` ran the LEFT wrist_flex +/-5 deg out-and-back live (log `calibration_data/powered_move_2026-11-09.jsonl`, all legs REACHED within 6 ticks, peak load ~80, 31 C). The owner watched: ticks DOWN = tip moves AWAY from the DexArm, ticks UP = TOWARD it. This matches the cell model (sign +1; render `scripts/view_joint_move.py`, orientation/dark parts/DexArm side confirmed by the owner). doc-1 convention holds (opposite the cable: ticks DOWN = counter-clockwise). The owner noted the viewer's REST pose did not look like the real arm's starting pose, so absolute zero offsets (wrists set at mid-range) are NOT verified. NOT yet checked: right wrist_flex, both wrist_rolls, grippers, joints 2-3 after the rebuild. Read-only tool for these: `scripts/direction_check.py --arm X --live` (you move each joint clockwise by hand, it reads ticks).
- **AA-43 still open:** AC #3 (gripper limits are owner-assumed old values, not measured), AC #4 (joint 4-5 signs and the gripper open end not re-verified on hardware), AC #5 (first supervised small powered move of IDs 4-6, both arms; dry-run first, owner supervising). `servo_calibration.unverified` is still false from the old servos: re-verify signs with a read-only move check before powered motion.
- **Torque/drive settings vs LeRobot (read 2026-11-09):** LeRobot so_follower.configure() sets P=16, I=0, D=32 on every joint and, for the gripper only, Max_Torque 500, Protection_Current 250, Overload_Torque 25; it leaves the other joints at factory Max_Torque 1000, Protection_Current 310, Overload 80, and writes Acceleration 254. Ours now: gripper = LeRobot exactly; joints 1-5 = factory (Max_Torque 1000, current 310, overload 80, P 32, D 32). The only difference on joints 1-5 is P 32 vs 16 (and Acceleration register 0 vs 254). Not changed; decision pending with the owner (stiffness for holding the tube vs LeRobot-like smoothness for the learned policy).
- **Leader arm (AA-44), 2026-11-09:** bound, attached (`/dev/ttyACM3`, role `arm_leader`, `arms.leader` in hardware.yaml), IDs 1-6 assigned by the owner, read-only scan OK (`scripts/scan_leader.py`: all six answer, fw 3.10, 214 Hz, 0 errors). discover/attach_usb find it (8/8). AC #1 needs the owner's gearing; AC #4 not started. **Software ready (commit 25a2458):** `home_servos.py` and `record_joint_ranges.py` accept `--arm leader` (tolerate ONLY error bit 0x08, writes read back; dry-run by default). **Next (owner at the arm, torque irrelevant, encoder-only):** half-turn-home the leader with `cd robot && uv run python scripts/home_servos.py homing --arm leader --ids 1,2,3,4,5 --reading ID=TICKS ...` (dry-run first, then live) at a pose matching the followers' zero pose, record ranges, then the leader-to-follower mapping at low speed. Ask the owner before reworking AA-23 around the leader (decision 8).

### Done 2026-11-12: AA-45 (inchworm cycle animation). Owner: 'AA-45 cycle looks good' (final GIF: measured-fold start, 87 frames, 0 violations). Still revisitable: mesh margin 2.9 mm (global) and GRIP_REACH_TOL 2.7 mm. Future: check the animation on the real robot (pose_snapshot --plan-start, then replay without/with the tube). History below.
- `scripts/inchworm_animation.py [--animate --every 3] [--lift-mm N] [--first-carry-mm N]` + `src/inchworm/inchworm_plan.py`, tests `tests/test_inchworm_plan.py`. Outputs (gitignored): `robot/out/inchworm/{timeline.jsonl,report.txt,cycle.gif}`. No test frame needed any more (owner).
- **Owner refined cycle (2026-11-11, in AA-45 notes):** the tube moves only through the locked arms. Lift it away from both arms, carry it right (away from the DexArm), left regrip, right regrip, lower, arms back to start. Joints 2-4 do the motion: no joint 1, and the elbow must be used.
- **Now:** lift 170 mm, carry half a pitch, regrip standoff = approach.standoff_m (50 mm). Both values are model-chosen, not measured. CellState has `tube_transform` (rigid 4x4) so the lifted tube is checked where it is.
- **Result on the placeholder:** 87 frames, 11 violations. Left pan 2.5 deg, right pan 13.8 deg, elbows ~118 deg. Remaining:
  - start/end within ~3 mm of the shoulder (placeholder pose);
  - right gripper misses by up to 3.6 mm on 3 carry frames and 2.9 mm on 1 regrip frame. Likely cause: the CAD tube axis is ~1 deg off the arm plane, and the working-pose grip spacing is 182.4 mm against a 183.676 mm pitch.
- **Owner 2026-11-11:** "the animation looks correct". AC #1-#6 are checked. The task stays In Progress for two new owner points:
  - the tube placement in the URDF is good enough (insertion is learned);
  - on return to the start, the two arms should match each other.
- **Done since:** the matching start (one shared pose for both arms, about 2 mm grip residual, the cycle ends exactly in it). Frames are now checked against the configured soft limits.
- **Zeros applied 2026-11-11 (owner approved):** shoulder_lift zero L 214.893 / R 220.518 (AA-47 landmark); lift soft limits re-derived (lower = URDF -100 deg, upper 0.85 L / 0.65 R from the EEPROM). folded_transit, the fold waypoints and the fold allowance now use the fold hand-placed 2026-11-11 on the new servos. AA-45 re-plan: 14 violations (none soft-limit): 10 regrip reach misses of 1-2.7 mm and 4 start/end near misses (lower arm 3 mm from the shoulder, margin 5 mm). Previously planned: fix those two (e.g. a smaller regrip standoff or a different retract path; a printing pose slightly more open), then the owner reviews the GIF.
- **Owner decision 2026-11-12 (REVISITABLE, see AA-45 notes):** tolerances changed instead. mesh `margin_m` 5 mm -> 2.9 mm (GLOBAL, hardware.yaml) and `GRIP_REACH_TOL_M` 1.0 -> 2.7 mm. Re-plan: 0 violations. To revert: restore 5 mm and open the start pose elbow ~1-2 deg; restore 1 mm and exempt only the regrip transit frames. **Next for AA-45:** owner reviews `robot/out/inchworm/cycle.gif`.
- **2026-11-12 elbow zero corrected (owner approved):** AA-47 landmark ticks (L 182.900, R 179.033 deg; was mid-range 177.1 / 173.672). Fold render now matches (owner 'yes'). Real fold elbow 91.4 deg = the model's contact (91.7), so the model is NOT conservative there; owner kept the 2.9 mm margin anyway.
- **2026-11-12 AA-45 start = measured fold (owner):** both arms the same, pan 0 and roll 0 held in every frame of this movement (pan stays a free joint elsewhere; the learned insertion may pan). The tube is re-placed onto the grippers (stops if > 5 mm off the CAD point or the axis turns > 2 deg). 0 violations. Owner confirmed the GIF 2026-11-12; task Done.
- **AA-40 DONE 2026-11-12.** AC6 reduced to the fold (owner). `motion_guard.collision.mesh.unverified: false` (owner). Caveat: model compared with reality only near the fold; the first supervised slow replay (no tube) checks the rest. Live still gated by unverified soft limits, AA-43, and unwired cell-state sources in fold/cycle/approach/jog.
- **Verification plan (owner asked how the animation will match reality):**
  1. Static pose match: `scripts/pose_snapshot.py --label X --live`. Read-only, torque off. It renders the reading and logs the owner's y/n. This also covers AA-40 #6 and most of AA-43 #4.
  2. Start-pose check before motion: `inchworm.start_pose_check`, or `--plan-start out/inchworm/timeline.jsonl`.
  3. Dry-run, then a supervised slow replay without the tube.
  4. Then with the tube.

### nono sandbox profile: draft COMPLETE (2026-11-17), owner to promote
- The draft `~/.config/nono/profile-drafts/inchworm-893.json` is valid. Versus before, it adds rw `/tmp` (stop files, camera locks, pytest tmp), rw `/dev/shm`, `listen_port [0]`, `open_port [8765]` (record preview) and `open_port_range [[32768,60999]]` (the test that binds port 0).
- Under it: the main suite rc 0, the lerobot_env suite rc 0, a real git commit OK, worktrees, backlog and `pi --version` OK, and `touch /tmp/inchworm_stop_move` OK.
- **Owner decision 2026-11-17: push from OUTSIDE the sandbox** (no SSH/credentials in the profile). Under nono the orchestrator commits, and the owner pushes (`git -C ~/893 push origin HEAD`).
- Launch:
  1. Outside the sandbox: `cd ~/893/robot && uv run python scripts/attach_usb.py` (devices must exist before start).
  2. Once: `nono profile promote inchworm-893`.
  3. Run `nono run -p inchworm-893 -- pi`; suggested alias `pi893`.
- Untested: serial/camera devices inside the sandbox. The first hardware motion under nono is a supervised dry-run.

### LeRobot pipeline (2026-11-17)
- **Recording: GO** once the owner has checked the wrist-camera rate (~8-12 new frames/s; Windows Camera app).
- Dataset: v3.0, 30 fps, `observation.state` 5 joints, `action` 5 joints, `observation.images.wrist` 480x640x3 (grey copied to 3 channels). The gripper is excluded by design.
- **Fixed (commit 0103784):**
  - lerobot_env has the `training` extra (accelerate).
  - The AA-26 LeRobotBackend is on the 0.6.1 API with processors.
  - The checkpoint check handles 0.6.1.
  - New dataset-replay dry-run in `rollout.py`.
  - `inspect_dataset.py` reads v3.0 (episodes, new-image rate, 360 deg check).
  - The recorder logs `loop_hz`/`overruns`/`save_s`.
- Train: `cd robot/lerobot_env && uv run lerobot-train --policy.type=act --dataset.repo_id=local/<name> --dataset.root=../datasets/<name> --output_dir=../out/train/<name> --policy.push_to_hub=false --wandb.enable=false --policy.device=cuda --steps=20000`. Consider `--policy.n_action_steps` 10-20.
- Replay dry-run: `uv run python ../scripts/rollout.py --checkpoint ../out/train/<name>/checkpoints/last/pretrained_model --dataset ../datasets/<name> --episode 0 --device cuda --cycle-count 1`.
- **Still open:** live wiring of `rollout.py --live` (AA-26 notes list 4 items) and per-episode success labels.
- Episode plan: 10 clean episodes, then a smoke train, then the replay dry-run, then 50 and later 80-100 in about 5 variation groups.

### Blocker
- None for software. Hardware progress waits on the owner for the AA-43 steps above.

### What is built (software, tested, not yet run live)
- **Cell model + SO-101 mesh collision (AA-40, 5/6 AC):** `system_model/urdf.py`, `collision.py` (python-fcl, fail closed), `MotionGuard.mesh` layer, `scripts/view_pose.py` on the cell URDF, `tests/test_cell_collision.py`. AA-40 Done 2026-11-12: mesh model verified by the owner (`unverified: false`).
- **DexArm cell check (2026-10-28):** `system_model/dexarm_kinematics.py` (firmware IK/FK from Rotrics' Marlin_For_DexArm source, URDF mapping 0.31 mm max), `system_model/dexarm_guard.py`, wired into `DexArmDriver.move_absolute/move_relative/print_move/home`; `dexarm.open_live_dexarm()` fails closed without SO-101 poses and tube state; `so101_pose_reader.py` (read-only, tolerates 0x08 only on IDs 4-6); `level_scan.py` / `calibrate_scanner.py --live` require `--tube`.

### Next actions (pick one per session; ask the owner which)
1. **Supervised DexArm check** (owner at the robot): attach arm ports, read real SO-101 poses, dry-run queries including one expected refusal, one 5 mm move, compare reported vs model. Also explain the unexplained Z drop (31.05 -> 9.35 mm between two reads 2026-10-28, nothing commanded).
2. **AA-13 DexArm repeatability** by scanner camera + chessboard, scale checked with the owner's digital calipers (method approved in principle; write the script and dry-run first; AA-30 intrinsics can come first).
3. **AA-30 scanner intrinsics** (owner deferred 2026-11-06, wants to do it soon). Software pre-flight DONE 2026-11-09: live paths fixed and tested, `robot/assets/chessboard_9x6_25mm.pdf` generated (`scripts/make_chessboard_pdf.py`). Owner session: print the PDF at 100 % ("actual size"), check one square (25 mm) and the 100 mm bar with calipers, mount flat, good light, then `cd robot && uv run python scripts/calibrate_scanner.py --live --step intrinsics --n-frames 20` (about 20 board poses; the script re-prompts if the board is not detected). Needs the scanner camera attached (`scripts/attach_usb.py`); no arm motion, laser off. Optional: re-measure arm-camera fps with the room lit.
4. **Software follow-ups:** wire `other_arm_pose_source` / `tube_source` / `dexarm_pose_source` into fold/cycle/approach/jog (live refuses until wired); re-derive `folded_transit` and the fold waypoints from the measured fold (the hardware.yaml placeholder is in contact in the model).
5. **AA-43** (in flight, see above), then **AA-44** leader arm.

### Human-handoff queue
- AA-41 (gripper lock probe): deferred; needs a fresh procedure after AA-43. The old 0x08 / encoder-only diagnosis no longer applies.
- AA-42 (FD re-flash): moot. On 2026-11-11 the owner checked with FD at 1000000 baud (FD had fallen back to 115200): all 6 left servos answer, all on 3.9. That is the expected factory firmware of the replacement IDs 4-6 (census 2026-11-09); the earlier 3.10 was the removed servo. Owner to decline it (`/human decline AA-42`); do not mark it verified.
- AA-46: superseded by AA-47 (zero landmark check added). AA-42: declined by the owner 2026-11-11. Walkthrough tab: if the auto-launch dies, start it by hand with `tools/human-handoff/open-walkthrough.sh`.

### Not committed on purpose
- `CAD/exports/wrist_gripper.stl` deleted and `CAD/exports/gripper_update.stl` added by the owner (superseded by `CAD/robot_urdf_new`). Ask before committing either.

### Lessons (keep applying)
- Ask the owner after two failed probes of a hardware anomaly (the 2026-10-24 "0 C, ~930 current" reading was the encoder-only servos and cost several sessions).
- Sub-agents get ONE narrow task, an early deliverable and a time cap; the AA-40 stage-2 workers explored for 30-45 min each and wrote nothing.
- Cost to date about $428 (2026-11-03); most of it was long sessions re-reading 300k+ token contexts. Restart instead of continuing.

## 1. Resume in 60 seconds

```bash
cd /home/stevotimp/893
pi                         # then /new, and: "Read doc-3 and continue with next action N"
# pi -r                    # browse past sessions only if you must recover something
```

Session transcripts live in `~/.pi/agent/sessions/--home-stevotimp-893--/` and are **machine-local, not in git**. They are not a system of record: compaction summarizes and discards older turns, and sub-agents cannot read them. Everything that matters is in the repository instead:

| Source | What it holds |
|---|---|
| `doc-1` | Hardware and system truth. Never assume a hardware fact; take it from here. |
| `doc-2` | Change-control rules. Read before implementing anything. |
| `doc-3` (this) | Live state, blockers, uncertainty, next action. |
| `backlog/tasks/` | 33 tasks, ~230 acceptance criteria, dependencies. |
| `AGENTS.md` | Loads the critical rules into any agent session automatically. |
| `backlog/config.yml` | Project-wide Definition of Done, inherited by every new task. |

Ground truth commands, which outrank this document:

```bash
backlog task list --plain            # real status of every task
backlog task list --status "To Do" --plain
backlog task view AA-4 --plain       # full criteria for one task
git log --oneline -5
```

## 2. Where the project stands

**Software for every subsystem exists and is tested against mocks.** Done: AA-4, 5, 6, 7, 8, 12, 15, 20, 22, 28, 33, 34, 37, 40. Every other started task has its software ACs checked with test evidence; only hardware-, owner- or model-dependent ACs remain open. Not started (they need hardware or a trained model end to end): AA-13, 25, 27, 35, 36, 38, 39.

Modules in `robot/src/inchworm/`:
- **Devices:** discovery (cameras by usbipd BusId via vhci), PySerialTransport, servo_bus, dexarm, laser, camera_arbiter, camera_capture.
- **Motion:** kinematics (vendored SO-101 URDF), trajectory, joint_calibration, motion_guard, homing, torque, estop, fold, approach.
- **Gripper:** gripper_lock (the sole token holder), policy_actions.
- **Cycle:** frames, cycle, recovery, supervisor, policy_contract, policy_runner, seating.
- **Vision and leveling:** laser_line, scanner_calibration, leveling, print_frame, gcode_intake, calibration.
- **Data:** teleop_capture, lerobot_adapter, plus a separate `robot/lerobot_env/` (Python 3.12, lerobot 0.6.1, CUDA works; ACT training step about 1.7 GB at 480x640 bs8).

**Review lessons (keep applying):** agent self-audits and Haiku reviews repeatedly passed real safety bugs:
- camera lock per device
- laser max-on not enforced
- DexArm bounds fail-open and a raw-motion regex hole
- a laser that never switched on during scans because of a mock mismatch
- arm-frame clearance computed in the wrong frame
- fail-open on missing readings in four places
- live script paths calling non-existent APIs
- synthetic calibration committed into config/
- AC checks lost in worktree patches, plus an agent adding pseudo-ACs

The orchestrator must independently verify every safety-relevant change, and must re-check AC state after applying any worktree patch.

## 2b. Hardware session 1 results (2026-10-24, owner supervised)

Done:
- Servo scan: both arms have 6x STS3215 (model 777) at IDs 1-6, firmware 3.9/3.10; full joint read about 200 Hz (AA-8 Done).
- Relay and laser: LCUS-1 at 9600 verified on hardware (AA-29 #1).
- **Servo EEPROM written (owner-approved):**
  - half-turn homing offsets and per-joint Min/Max_Position_Limit on both arms, all read back;
  - backups in `robot/calibration_data/eeprom_backup_*`;
  - right shoulder_pan capped to the left arm's span (owner decision).
- Joint zeros = mid-range (URDF so101_new_calib convention).
- Direction test: higher reading = clockwise (cable-away view).

Bugs found ON HARDWARE and fixed:
- model register (3, not 0);
- serial read blocking for the full timeout (811 ms to 5 ms per read);
- SYNC_READ reply collisions;
- live scripts silently using the mock backend;
- **torque enable would have slammed every joint to the stale goal 0 deg** (now goal := present, verified, before enable);
- range recorder not wrap-aware.

Open from session 1:
1. Joint signs vs the URDF +axis: an assumed +1, servo_calibration kept `unverified: true`, so live motion is blocked. Next step (no hardware): derive the per-joint mapping from URDF/CAD, then one owner check ("move the tip toward the DexArm").
2. Combined-joint safe envelope (owner: limits depend on all joints together). The collision keep-out geometry is still placeholders; it must come from CAD and measurements before motion near the limits.
3. The servo temperature reads 0 C and the current reads about 930 raw at rest: confirm plausibility before AA-10 fault thresholds rely on them.
4. Remaining checklist steps: gripper bands (AA-17), the first powered jog, named poses and fold, DexArm, cameras, calibration.

## 3. Owner hardware session checklist (about 2-3 h, dry-run first, owner supervising every first live motion)

Order matters: each step unblocks the next. All scripts are in `robot/scripts/`, dry-run by default, `--live` explicit.
1. **Attach and discover:** `uv run python scripts/attach_usb.py` then `uv run python -m inchworm.discover`. Expect 7/7 attached.
   - **After any power-off or reboot:** usbipd *binds* persist (no admin step is needed again), but devices must be **re-attached** to WSL each time, so run attach_usb.py. Linux node numbers (`/dev/ttyACM*`, `/dev/video*`) and WSL vhci ports change on every attach; the code never relies on them. Arms, DexArm and scanner camera are identified by USB serial; the relay by its unique VID:PID; the two arm cameras by **Windows BusId = physical dock port** (9-4 left, 10-2 right), now cross-checked against VID:PID. BusIds stay the same as long as the dock is plugged into the **same laptop USB port** and each cable stays in its **same dock port**. If the dock moves to another laptop port, the BusIds change: run `usbipd list`, update `windows_busid` for cam_left/cam_right in hardware.yaml, and re-confirm which camera is which.
2. **Relay protocol (AA-29 #1):** `uv run python scripts/laser_probe.py --live`. Confirm that LCUS-1 `A0 01 01 A2` / `A0 01 00 A1` and 9600 baud switch the laser. Record whether the relay reports state (AA-29 #5).
3. **Servo IDs (AA-8 #2, #4):** `uv run python scripts/scan_servos.py --live --persist`, then `--benchmark` (>= 20 Hz).
4. **Joint limits and calibration (AA-9 #1):** `uv run python scripts/measure_limits.py --live --arm left` / `--arm right` (torque-off manual sweep). Also measure per-joint zero offset and sign, then set `unverified: false`.
5. **Gripper bands (AA-17 #1):** `uv run python scripts/gripper_bands.py --live --arm arm_left` / `arm_right`.
6. **Left/right confirmation:** `uv run python scripts/jog.py --arm left --delta 0.005,0,0 --live` (speed-scale 0.2 default, through GuardedArm). It refuses until steps 3-4 populate servo IDs, calibration and limits.
7. **Named poses (AA-9 #4, AA-19 #1/#4):** neutral and folded_transit, then `scripts/fold_cycle.py --live --cycles 10`.
8. **DexArm (AA-12 bounds, AA-13):** confirm 115200 baud and homing; measure the real workspace box (replace `unverified: true`); then the AA-13 repeatability suite.
9. **Cameras (AA-14 #4, AA-38):** `scripts/camera_probe.py --live --role ...` for each role, then the concurrency measurement.
10. **Calibration (AA-16, AA-30, AA-11 #3):** `scripts/calibrate_arm_base.py`, `hand_eye.py`, `tube_axis.py`, `calibrate_scanner.py --live`, `fk_verify.py --live`.
11. **Approach and dry cycle (AA-18 #4, AA-21 #7):** `scripts/approach_repeatability.py --live --n 10`, `scripts/dry_cycle.py --live --cycles 5` (no tube).
12. **Then:** tube fixture build (`robot/docs/tube_fixture.md`), seating traces (AA-24), teleop demos (AA-23), training (AA-25), rollout (AA-26), AA-27/35/36.

## 4. Pending owner decisions

None open as of 2026-10-27 (all six were answered; see the top of this doc). Still owed from the owner at the hardware: accept and run AA-41; the AA-40 #6 render comparison; confirm the installed nozzle is the stock 0.4 mm and the filament 1.75 mm PLA (AA-34 doc §10).

Software follow-ups: wire cell-state sources into callers; re-derive the fold waypoints from the measured fold. AA-40 #6 render comparison is ready (the old DexArm misplacement is gone: the cell URDF is used directly).

## 5. Decisions already made (do not silently revisit)

Each of these came from an explicit owner answer or a verified observation. Changing one is a scope change per doc-2 section 4.

- **The chassis is fixed; the tube moves.** This inverts the usual mobile-robot convention. Every frame computation must respect it.
- **The gripper latch and jaw share one servo ID** (owner's custom gripper). Lock and unlock are *position bands* of a single servo, not separate actuators. Locking is commanded by code and never by the learned policy, per the owner's explicit instruction.
- **Demonstrations use cross-arm teleoperation with the tube held by a fixture** (decision-1, owner 2026-10-24). A torque-off leader never holds the tube, and the gripper is never teleoperated. Start with one arm; AA-39 decides on the second.
- **Both arms hold the tube at the start of every cycle and whenever printing** (owner, 2026-10-24).
- **Commit and push automatically** after each verified unit of work (AGENTS.md rule 7). Never force-push.
- **Control host is WSL2** with usbipd attach, chosen for LeRobot support.
- **Slicing is done by the owner in Cura**, and a correct DexArm profile must be installed there first. AA-34 owns that boundary; do not assume a profile exists.
- **The real crawl cycle depends on learned insertion** (AA-27 depends on AA-26), per the owner's observation that crawl cannot really happen until insertion works. A tube-free dry rehearsal (AA-21) comes first.
- **DexArm repeatability is measured before laser calibration** (AA-30 depends on AA-13), so arm positioning error is not silently absorbed into the optical calibration.
- **Printing is deferred.** The near-term goal is crawl, regrip, fold and auto-level. AA-34 and AA-35 integrate printing later.

## 6. Honest uncertainty

Stated plainly so a fresh session does not mistake an estimate for a commitment.

- **Overall: roughly 45 to 90 agent sessions plus 25 to 50 hours of owner hardware time; 6 to 10 weeks hands-on, 3 to 5 months at weekend pace.** Hardware tasks cannot be parallelized across sub-agents: one robot, one supervising human.
- **Owner demonstration time (added 2026-10-24):** about 6-15 extra hours. The first dataset is roughly 50-100 episodes per arm (LeRobot guidance: 50 or more, about 10 per variation), about 3-8 h. Then 2-3 HIL correction rounds at about 1-2 h each. Any change to the camera mount, attachment geometry or approach pose can invalidate the recorded data.
- **M3 (learned insertion) is the least predictable phase**, estimated at 10 to 25 sessions. No leader arms makes demo capture awkward, and 4 GB of VRAM constrains architecture. If the first policy does not generalize across the coarse-approach offset range, the recollect, retrain, re-evaluate loop costs days per iteration. This is the number least worth trusting.
- **AA-13 could force a redesign of M5.** If DexArm repeatability cannot hold roughly 0.1 mm at the scan standoff with the print head and scanner payload attached, leveling precision is capped. Deliberately sequenced early so this surfaces in week 2 rather than week 8.
- **AA-16 calibration quality propagates into M3.** Loose transforms widen the capture range the policy must absorb.
- **AA-31 line extraction on a glossy, curved, ridged printed surface is harder than on a flat coupon.** A method that passes on a test coupon and fails on the real tube would look fine in review and then produce quietly wrong conclusions.
- **Estimates for m-0 through m-2 are defensible. From m-3 onward they are a forecast.**

## 7. Stability model, and its limit

doc-2 holds the rules. The mechanisms in place: criteria written before code, doc-1 referenced rather than copied, a project-wide Definition of Done every task inherits, dependency ordering, mock backends so regressions appear as failing tests instead of crashes, the safety boundary built and tested before the model exists (AA-20), a dry rehearsal before anything touches the tube (AA-21), and recorded baselines in AA-13, AA-18 and AA-21 so later drift is detectable.

**The limit, stated honestly:** the Definition of Done and doc-2 are strong defaults, not enforcement. A determined agent can still check a box it has not earned. The real backstop is the orchestrator reviewing `git diff` for unapproved changes to acceptance criteria, doc-1 numbers, or safety checks. A pre-commit hook that fails on modified `AC:` lines without an approval marker would make drift mechanically hard rather than merely discouraged; it has been proposed to the owner but is **not** implemented.

## 8. Repository hygiene notes

- `.parley.env` contains a live API key and is gitignored. This repo pushes to a GitHub org remote, so never stage it. Verified absent from `origin/main`.
- `*Zone.Identifier` files are gitignored; one that had already been committed was untracked in `98840c7`.
- `robot/sessions/`, `robot/out/`, `robot/datasets/` and `robot/checkpoints/` are pre-ignored so AA-7 logs, camera frames and policy checkpoints cannot be committed by accident.
- Do not commit photographs, session transcripts, or secrets as task evidence. Keep frames local unless the owner authorizes publication.
