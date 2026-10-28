---
id: doc-3
title: Orchestrator State and Session Handoff
type: guide
created_date: '2026-10-20 21:37'
updated_date: '2026-10-28 22:46'
---
Live orchestrator state: what is in flight, what is blocked, what is uncertain, and how to resume cold. Read this **first** when picking up this project in a fresh session, then doc-1 (hardware truth) and doc-2 (change-control rules).

**Keep this current.** Update it via `backlog doc update doc-3` whenever a blocker clears, a task finishes, an estimate changes, or a decision is made. A stale doc-3 is worse than none, because it invites confident action on wrong information. If the Last verified date below is old, trust task statuses and `git log` over this prose and re-verify before acting.

**Last verified:** 2026-10-27 (session 3), AA-40 stage 2 commit. Full suite **1617 passed**.

**Owner decisions 2026-10-27 (recorded, do not revisit):** (1) e-stop HOLDS in place. (2) The tube advances to the right, AWAY from the DexArm (`cycle_runner.advance_physical: away_from_dexarm`; live preflight checks the sign against AA-16 calibration). (3) Cura = Rotrics' official settings (AA-34 Done). (4) No tube fixture design. (5) Gripper lock torque via human task **AA-41** (`scripts/gripper_lock_probe.py`; in progress with the owner). (6) Keep the one-camera rule. Session 3: (7) the owner cell URDF `CAD/robot_urdf_new/` is THE robot model (arms, grippers, camera mounts, DexArm + laser camera, base mount, tube); adding limits is fine, nothing else to rebuild. (8) The table is NOT a collision object. (9) "Go with the real fold": model contacts at the measured fold are model error, accepted only near that pose. (10) Wrist roll: our 0 = cell URDF -127 deg. (11) Tube = sections with one attachment point each (3 now, +1 per printed section).

**Current focus: AA-40 #6 (owner render comparison), then wiring cell state into callers.** AA-40 stage 2 is in: `system_model/urdf.py` + `collision.py` (python-fcl, fail closed), `MotionGuard.mesh` layer, `scripts/view_pose.py` on the cell URDF, `tests/test_cell_collision.py`. Live motion stays blocked: `motion_guard.collision.mesh.unverified: true` until the owner compares renders (`cd robot && uv run python scripts/view_pose.py --presets --tube-pitches 0`, PNGs in robot/out/poses).
- Next software: wire `other_arm_pose_source`, `tube_source` (advance in m) and `dexarm_pose_source` into fold/cycle/approach/jog (live refuses until wired); re-derive `folded_transit` and the fold waypoints from the measured fold (the hardware.yaml placeholder is in contact in the model).
- Why stage 2 took three tries: two sub-agents explored without writing; the third orchestrator attempt crashed on an API error (a tool call without a result). Lesson: do model work directly, in small committed slices.
- **BLOCKER, ROOT CAUSE FOUND (owner 2026-10-27):** servos ID 4-6 (wrist_flex, wrist_roll, gripper) on BOTH arms are **encoder-only leader-arm servos with no motor drive** (doc-1 s3). They read position but cannot move; that explains the zeroed drive EEPROM, 0 C, ~950 current and 0x08 flag. Owner will buy 6 powered servos and rebuild both arms: **AA-43** (swap + recalibrate joints 4-6; AA-17 and AA-41 depend on it). Until then, no powered wrist/gripper motion; joints 1-3 are powered and fine. AA-42 (FD re-flash) is moot; the owner closes it in the walkthrough tab. AA-41 is deferred and gets a fresh procedure after AA-43. EEPROM writes made to IDs 4-6 on 2026-10-27 are harmless (backups in calibration_data/eeprom_*2026-10-27T23-*). Lesson: the 2026-10-24 observation '0 C, ~930 current on some servos' was this hardware difference and should have been chased then.
- Session 2 facts still valid: servo_calibration verified; EEPROM limits widened; left pan unobstructed (240 deg); measured poses in `robot/calibration_data/poses_2026-10-25.jsonl` (the stored pan q is stale; recompute from ticks, as view_pose.py does).

## 1. Resume in 60 seconds

```bash
cd /home/stevotimp/893
pi -c                      # continue the most recent session for this folder
# or: pi -r                # browse past sessions for this folder
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

**Software for every subsystem exists and is tested against mocks.** Done: AA-4, 5, 6, 7, 8, 12, 15, 20, 22, 28, 33, 34, 37. Every other started task has its software ACs checked with test evidence; only hardware-, owner- or model-dependent ACs remain open. Not started (they need hardware or a trained model end to end): AA-13, 25, 27, 35, 36, 38, 39.

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
