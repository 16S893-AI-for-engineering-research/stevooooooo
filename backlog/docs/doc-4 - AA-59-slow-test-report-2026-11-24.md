---
id: doc-4
title: AA-59 slow-test report (2026-11-24)
type: other
created_date: '2026-11-24 04:40'
updated_date: '2026-11-24 04:40'
---
# AA-59 report: slow / redundant test candidates (REPORT ONLY)

Scope: report only (AGENTS.md rule 3). No test file was changed, skipped, marked or deleted.
No full suite was re-run by this agent. The owner decides what, if anything, is removed.

Sources:
- `/tmp/aa59/robot.txt`: robot suite with `--durations=60`, run while the machine was loaded by
  parallel jobs. Per-test seconds there are inflated (see the laser note below); the ranking is still useful.
- Earlier totals today: robot 1960 tests in 435 s; lerobot_env 109 tests in 899 s.
- `/tmp/suite_robot.txt` / `/tmp/suite_lerobot.txt` (orchestrator's fresh sequential run): the robot
  one had no durations section yet when this was written, and the lerobot one did not exist yet.
- lerobot_env single-file timings: see section 1b (status given there).
- `/tmp/aa59/imp.txt` (`-X importtime`): `import trimesh` costs about 1.9 s cumulative per process.

## 1a. Robot suite: slowest tests

Columns: **seq** = fresh sequential run by the orchestrator (`/tmp/suite_robot.txt`, `--durations=80`,
EXIT=0); **loaded** = `/tmp/aa59/robot.txt` (`--durations=60`, run under parallel load). Use seq.

| seq s | loaded s | test | what it does |
|---|---|---|---|
| 75.35 | 76.49 | tests/test_inchworm_plan.py::test_matching_start_pose_and_return | full `plan_cycle(start_from_fold=False)` (IK matching start, full step count, mesh checks) |
| 72.16 | 70.56 | tests/test_inchworm_plan.py::test_per_arm_start_reaches_each_point | full `plan_cycle(start_from_fold=False, matching_start=False, steps=2)` |
| 32.58 | 45.73 | tests/test_jog_live.py::TestLiveVerifiedWiresGuardedArm::test_writes_go_through_guarded_arm_only | in-process `jog.main(--live)` on a fake backend; spies GuardedArm, asserts no gripper write |
| 28.11 | 31.11 | tests/test_jog_live.py::TestLiveVerifiedWiresGuardedArm::test_no_direct_bus_writes_bypass_guard | in-process `jog.main(--live)`, asserts all writes go through GuardedArm |
| 13.06 | 14.22 | tests/test_script_imports.py::test_script_dry_run_default_exits_zero[dry_cycle.py] | subprocess bare invocation |
| 11.97 | - | tests/test_perturb.py::test_v6_full_schedule_counts_variation_and_spread | full v6 perturbation schedule |
| 11.78 | 31.45 (timed out, FAILED) | tests/test_kinematics.py::TestJogDryRun::test_jog_dry_run_delta | subprocess `jog.py --dry-run --delta` |
| 11.61 | 4.77 | tests/test_direction_check.py::test_view_joint_move_renders_and_states_the_sense | render |
| 10.98 | 20.65 | tests/test_script_imports.py::test_script_dry_run_default_exits_zero[view_pose.py] | subprocess, real mesh load + render |
| 10.82 | 15.94 | tests/test_cell_collision.py::test_viewer_renders_trajectory_and_reports_contacts | in-process view_pose: 3 PNG renders + presets check |
| 10.59 | 17.27 | tests/test_script_imports.py::test_script_dry_run_default_exits_zero[fold_cycle.py] | subprocess |
| 9.83 | 30.45 | tests/test_motion_guard.py::TestJogAA9Integration::test_jog_dry_run_with_guard | subprocess `jog.py` dry run, asserts "AA-9 guard active" |
| 9.41 | 22.39 | tests/test_kinematics.py::TestJogDryRun::test_jog_no_gripper_command_in_output | subprocess `jog.py --dry-run`, greps output for gripper_channel |
| 9.18 | 14.95 | tests/test_script_imports.py::test_script_dry_run_default_exits_zero[approach_repeatability.py] | subprocess |
| 8.83 (setup) | 7.50 | tests/test_approach.py::test_plan_produces_trajectory | fixture: planner + real mesh |
| 8.75 | 30.16 (timed out, FAILED) | tests/test_kinematics.py::TestJogDryRun::test_jog_dry_run_abs | subprocess `jog.py --dry-run --abs` |
| 7.56 | 13.38 | tests/test_servo_bus.py::TestGripperCapability::test_no_token_import_outside_aa17 | source text scan; the time is probably the first-import cost it happens to pay, not the scan |
| 5.43 (setup) | 82.16 (setup, other test id) | tests/test_laser.py fixtures (`load_config()`) | **not a real cost.** test_laser.py re-timed alone: 5.8 s wall for the whole file, slowest item 0.35 s. The 82 s in the loaded run was a stall under load. Nothing to remove |
| 4.11 | 14.99 | tests/test_system_model.py::test_scene_zero_builds | first `SystemModel()` real-mesh load |
| 4.08 | 5.66 | tests/test_inchworm_plan.py::test_fold_start_holds_pan_at_zero_and_roll_at_zero | default plan_cycle (fold start) |
| 3.94 | 4.65 | tests/test_inchworm_plan.py::test_plan_cycle_produces_labelled_timeline | default plan_cycle |
| 3.52 | 5.39 | tests/test_two_policy.py::test_left_and_right_to_train_right_pass_the_checks | |
| 3.37 | 3.23 | tests/test_cycle_replay.py::test_simulated_replay_runs_the_sequence_slowly | |
| 3.15 | 2.92 | tests/test_handoff.py::test_simulated_stop_leaves_both_arms_holding_right_locked | |
| <3 each | | ~56 more in the top 80 (cycle_replay, perturb, handoff_return, camera_arbiter, script smoke tests 0.4-2.9 s) | |

Sums (seq run): top 80 = 426.9 s of the ~435 s suite; i.e. ~1880 tests take < 10 s together. The two
non-default planner tests = **147.5 s (34%)**. The six jog tests (2 live-path + 4 subprocess) = **100.9 s (23%)**.
Script bare-invocation subprocesses in the top 80: 10 ids, 49.3 s. `test_script_help_exits_zero[*]` are each
< 0.43 s in the seq run (not a real cost; the loaded run showed 1-3.6 s).

Note: `TestJogDryRun::test_jog_dry_run_delta` / `_abs` FAILED in the loaded run on their 30 s subprocess
timeout (9-12 s unloaded). They are flaky under load. Not changed here (rule 3).

## 1b. lerobot_env suite (109 tests, 899 s)

Per-file timing (`uv run pytest tests/<file> -q -p no:cacheprovider --durations=20`) was started for
test_crawl_two_policy.py, test_policy_handoff.py, test_full_policy_test.py, output in
`/tmp/aa59/lerobot_files.txt`. Status at the time of writing: see section 6 (appended at the end).

Structure (from the source, not timing): almost all of the cost is end-to-end dry runs on simulated
servos with the real cell mesh and the real timeline, each loaded per test:
- test_crawl_two_policy.py: 15 tests; 13 call `_run()` = a full `crawl_two_policy.main(--dry-run)`
  (load_config + load_timeline + clamp_to_eeprom + mesh-checked moves at 8-10 deg/s simulated, `--max-s 30`).
- test_policy_handoff.py: 22 tests; most run `policy_handoff.main` on simulated servos with the mesh check.
- test_full_policy_test.py (AA-39): 5 tests, each a full `full_policy_test.main(--dry-run)`
  (`_state42` reloads the config and timeline per test).
- test_record_train.py: 11 tests (one bash subprocess); test_policy_test_sh.py: 4 bash subprocesses of the AA-39 launcher.

## 2. FLAGGED candidates (owner decides; nothing was removed)

Legend: KEEP = safety invariant, or the only cover. CANDIDATE = could go if the owner agrees; the
covering test is named.

| # | test id | s (loaded) | reason | invariant covered | still covered by | verdict |
|---|---|---|---|---|---|---|
| F1 | robot tests/test_kinematics.py::TestJogDryRun::test_jog_dry_run_delta | 11.8 seq (31.5 loaded, timed out) | duplicate: the same jog dry-run path in a subprocess; the in-process `tests/test_jog_live.py::TestDryRunUnchanged::test_dry_run_delta_exits_zero` runs `jog.main(["--arm","left","--delta","0.01,0.0,0.0"])` | dry-run is the default (rule 5) | test_jog_live.py::TestDryRunUnchanged::test_dry_run_delta_exits_zero, ::test_dry_run_explicit_flag_exits_zero, ::test_dry_run_never_opens_backend | CANDIDATE. Loses only "jog.py runs as a real `python script` process", which test_motion_guard::test_jog_dry_run_with_guard (F3) also checks |
| F2 | robot tests/test_kinematics.py::TestJogDryRun::test_jog_dry_run_abs | 8.8 seq (30.2 loaded, timed out) | duplicate subprocess of the in-process abs dry-run | dry-run default; --abs reachable | test_jog_live.py::TestDryRunUnchanged::test_dry_run_abs_exits_zero | CANDIDATE |
| F3 | robot tests/test_motion_guard.py::TestJogAA9Integration::test_jog_dry_run_with_guard | 9.8 seq | subprocess jog dry run; asserts "AA-9 guard active" in stdout | collision guard wired into jog (limits/collision) | partly: test_jog_live.py::TestLiveVerifiedWiresGuardedArm::* prove writes go through GuardedArm (the live path). No in-process test asserts the guard banner in the dry-run path | **KEEP** (collision/limits). Keep ONE jog subprocess test; this is the one to keep if F1/F2 go |
| F4 | robot tests/test_kinematics.py::TestJogDryRun::test_jog_no_gripper_command_in_output | 9.4 seq | weak subprocess output-grep for "gripper_channel" | gripper only via AA-17 | stronger: test_jog_live.py::TestLiveVerifiedWiresGuardedArm::test_writes_go_through_guarded_arm_only (spy asserts no gripper bus write, 5-vector writes only) and test_servo_bus.py::TestGripperCapability::* (token gate) | KEEP unless the owner accepts the named equivalents; they are stronger than this test |
| F5 | robot tests/test_jog_live.py::TestLiveVerifiedWiresGuardedArm::test_writes_go_through_guarded_arm_only and ::test_no_direct_bus_writes_bypass_guard | 32.6 + 28.1 seq | two full live-path `jog.main(--live)` runs on the same fake backend; they overlap (both spy the GuardedArm) | gripper never written by jog; all writes through the guard (limits/collision) | each other (could be one test with both assertions) | **KEEP** (safety). Merging the two into one run would save ~30 s without losing an assertion. That is a test change, so it needs owner approval |
| F6 | robot tests/test_inchworm_plan.py::test_matching_start_pose_and_return | 75.4 seq | non-default planner mode: `start_from_fold=False` (IK matching start, owner 2026-11-11). Since 2026-11-12 the default is the measured fold; no script, src or lerobot_env caller passes `start_from_fold=False` (grep) | planner returns to the start pose; IK residual < 3 mm | default fold path: test_fold_start_holds_pan_at_zero_and_roll_at_zero, test_plan_cycle_produces_labelled_timeline. Frame validation (collision, both-unlocked, soft limits) is covered by test_colliding_frame_fails, test_both_unlocked_frame_fails, test_soft_limit_violation_is_reported | CANDIDATE (obsolete mode). Alternative with nothing lost: run it with `steps=2` like F7 (a test change; owner) |
| F7 | robot tests/test_inchworm_plan.py::test_per_arm_start_reaches_each_point | 72.2 seq | non-default mode (`matching_start=False`, `start_from_fold=False`); no caller uses it | per-arm IK reaches each point within 1 mm | none for this mode | CANDIDATE (obsolete mode) only if the owner confirms the per-arm IK mode is dead; otherwise KEEP |
| F8 | robot tests/test_script_imports.py::test_script_dry_run_default_exits_zero[view_pose.py] | 11.0 seq | subprocess re-loads the mesh and renders; the in-process test_cell_collision.py::test_viewer_renders_trajectory_and_reports_contacts already runs view_pose.main | dry-run default | test_cell_collision::test_viewer_renders_trajectory_and_reports_contacts (covers rendering, not the bare default invocation) | CANDIDATE (weak). The point of this parametrized test is "every script's bare invocation is a dry run" (rule 5); removing one id weakens that guarantee for view_pose |
| F9 | robot test_script_dry_run_default_exits_zero[fold_cycle.py / approach_repeatability.py / dry_cycle.py] | 10.6 / 9.2 / 13.1 seq | subprocess smoke runs of whole dry cycles | dry-run default for scripts that move arms | the in-process tests of the modules behind them (test_cycle_replay, test_approach) cover the logic, not the script's default flag | **KEEP** (rule 5, dry-run default of motion scripts). Speed up instead (section 4) |
| F10 | robot test_script_help_exits_zero[*] (~40 ids, 0.7-3.6 s each, ~50 s total) | < 0.43 each seq (only 1 id in the top 80) | `--help` subprocess per script | none safety-related | test_script_inchworm_imports_resolve covers imports | NOT worth removing: the seq run shows them cheap (the loaded run inflated them). Withdrawn as a candidate |
| F11 | lerobot_env tests/test_full_policy_test.py (5 tests, AA-39) | per-file timing in section 6 | the AA-39 single-policy full test. AA-55/56 `crawl_two_policy.py` is the current flow (doc-3 START HERE 2026-11-24) and has its own end-to-end tests. BUT crawl_two_policy.py imports `full_policy_test` and reuses `UI, Stop, policy_phase, lock_and_look, hold_and_release, open_live_buses, _digest, ph_stop_file, RELEASE_HINT, J5, TIMELINE`, so the module is NOT dead | lock-cycle order; "left key not in -> nothing unlocks" (never both unlocked); quit -> locks nothing; lock far from demo end stops before the right unlocks | test_crawl_two_policy.py: test_full_crawl_phase_order_and_gripper_sequence, test_owner_says_the_left_key_is_not_in_the_right_never_unlocks, test_unlock_right_refused_unless_the_left_reads_locked, test_quit_in_the_right_policy_keeps_both_locked_left, test_lock_far_from_the_finish_ref_is_logged_and_the_crawl_continues | CANDIDATE: test_full_test_left_policy_lock_cycle_right_policy_lock, test_quit_after_the_left_attempt_locks_nothing, test_refuses_a_state_that_is_not_frame_42 (the crawl tests exercise the same shared functions). **KEEP** test_owner_says_the_left_key_is_not_in_nothing_unlocks and test_lock_far_from_the_demo_end_stops_before_the_right_unlocks unless the owner accepts the named crawl equivalents (never both unlocked). Note the crawl "lock far" test asserts "logged and continues", a DIFFERENT behaviour from the AA-39 "stops before the right unlocks", so it is not an equivalent for the AA-39 script; it is one only if full_policy_test.sh is retired as a launcher |
| F12 | lerobot_env tests/test_policy_test_sh.py (4 tests, AA-39 launcher `policy_test.sh`) | small (bash) | AA-39 launcher, superseded as the operator entry point by crawl_two_policy.sh | "policy never starts without a good replay" | none for this launcher | CANDIDATE only if the owner retires policy_test.sh / full_policy_test.sh; otherwise KEEP |
| F13 | lerobot_env tests/test_crawl_two_policy.py autonomous variants (test_autonomous_runs_the_original_order..., test_autonomous_regrip_first_runs_to_home..., test_autonomous_original_order_left_far..., test_autonomous_far_at_the_lock_timer_retries_once_then_holds) | per-file timing in section 6 | each is a whole simulated crawl; the phase order overlaps with test_full_crawl_phase_order_and_gripper_sequence | autonomous order, both-locked holds, stop file | partly each other | **KEEP**: AA-56 is the current live flow, and each test asserts a different branch (prompted vs autonomous, original vs regrip-first, retry). Speed up via fixtures/parallel instead |

Not flagged (KEEP, safety or unique): tests/test_laser.py (laser off on every exit path; 5.8 s alone),
test_servo_bus.py::TestGripperCapability (AA-17 gate), test_inchworm_plan.py::test_both_unlocked_frame_fails /
test_colliding_frame_fails / test_soft_limit_violation_is_reported, test_handoff*.py, test_cycle_replay obstruction/stop
tests, test_policy_handoff.py (speed cap, jump guard, mesh hit, gripper key refused, limit clamps), test_record_train
"never both open with tube", camera-arbiter tests (test_lerobot_adapter / test_camera_bridge, one camera).

## 3. Estimated time saved if every CANDIDATE were removed

Robot (435 s sequential; seq-run seconds): F1 11.8 + F2 8.8 + F6 75.4 + F7 72.2 + F8 11.0 = **179 s,
i.e. 435 s -> ~255 s (-41%)**. F6+F7 alone (the obsolete non-default planner modes) = 147.6 s. Without F7
(the less certain one): ~107 s. F1/F2 also remove the two tests that failed under load.

lerobot_env (899 s): F11 candidates (3 of 5 AA-39 tests) + F12. Each lerobot end-to-end dry run averages
about 8 s (899 s / 109 tests; the e2e files are well above the mean). Estimate **~40-90 s if the AA-39 runs cost like the mean, up to ~2.5-5 min if they cost like the crawl runs (see section 6)**; not yet measured.
Removal is a poor lever here: the cost sits in current-flow (AA-55/56) and safety tests. Use section 4.

## 4. Speedups that remove nothing

| option | estimate | notes |
|---|---|---|
| pytest-xdist `-n auto` (8 CPUs here; xdist is NOT installed yet: `ModuleNotFoundError: xdist`) | robot 435 s -> ~90-130 s (the floor is the slowest single test, ~45-75 s, unless F6/F7 are made cheaper). lerobot 899 s -> ~150-250 s (109 mostly independent e2e tests; floor = the longest crawl test) | needs a dev-group dependency and checks that tests do not share fixed paths: lerobot tests already monkeypatch STOP_FILE / OUT_DIR into tmp_path, which is good; `fpt.SIM["buses"]` is per-process module state, fine under xdist. The loaded run showed jog subprocesses hitting their 30 s timeout, so `-n auto` can make the subprocess tests flaky; `-n 4` is safer |
| session-scoped fixtures | robot: ~20-40 s (share one `SystemModel`/mesh + `CellCollisionChecker` + `load_config()` across test_system_model, test_cell_collision, test_approach, test_inchworm_plan (already module-scoped `cfg`/`checker`/`kin`), test_direction_check). lerobot: `_state42` in test_crawl_two_policy / test_full_policy_test reloads config + timeline + `clamp_to_eeprom` per test; a session fixture for the clamped timeline and the mesh checker could save ~1-3 s per e2e test, ~60-150 s overall | only valid where the objects are read-only; config objects that tests monkeypatch must stay per test |
| in-process instead of subprocess for script smoke tests | each subprocess pays ~2 s Python + trimesh import (`-X importtime`: trimesh 1.9 s) plus the mesh load; ~60 script subprocesses -> ~60-120 s robot | changes tests; loses the "real process" check, so keep one subprocess per motion script |
| `slow` marker, excluded by default (`addopts = -m "not slow"`), full run (`-m ""`) before every commit (AGENTS.md rule 7 keeps requiring the full `uv run pytest`) | inner-loop robot run: 435 s -> ~150-200 s marking the top ~20 robot tests; lerobot: mark the e2e files -> under 60 s inner loop | needs care: rule 7 says tests pass via `cd robot && uv run pytest` before a commit; if the default excludes slow tests, the commit gate must be changed to the full run explicitly (owner decision). Never mark the safety tests as excluded from the commit gate |
| cheaper parameters (no test removed, assertions kept): F6 with `steps=2`, `--max-s`/speed in lerobot e2e runs, merging F5's two live jog runs | robot ~70-100 s; lerobot unknown until per-test durations exist | these ARE test edits; owner approval needed under rule 3 |

Biggest single lever per suite: robot = xdist + making F6/F7 cheaper (or removing them if the modes are
obsolete); lerobot = xdist + a session fixture for the clamped timeline/mesh.

## 5. Evidence and limits

- Robot numbers come from a run under parallel load; the 82 s laser setup and the 13 s token scan are
  stalls (test_laser.py re-timed alone: 5.8 s wall for the file, slowest 0.35 s). Use the fresh
  `/tmp/suite_robot.txt` durations to confirm the absolute numbers.
- lerobot per-test numbers: section 6.
- No test file edited; `git status` should show only this report as new.

## 6. lerobot_env timing status (final)

- `/tmp/suite_lerobot.txt` (orchestrator's sequential `--durations=80` run, started ~23:24) was still
  running when this report closed (about 9 dots, no durations section). **Its durations section is the
  authoritative per-test list for lerobot_env; paste it into section 1b when it finishes.**
- My per-file run of `tests/test_crawl_two_policy.py` (started 23:17) had finished only **8 of 15 tests
  after ~15 min** and I stopped it at 23:32 (the time cap). For 8 of those 15 min it competed with the
  orchestrator's full lerobot run, so the numbers are inflated. Rough estimate: **~1-2 min per crawl test
  (~60 s or more uncontended)**, so this one file is probably **~10-15 min of the 899 s suite**, i.e.
  most of it. test_policy_handoff.py and test_full_policy_test.py were not timed (the loop was stopped
  before they started). No `--durations` numbers exist from me for lerobot_env, so AC#1 is only partly
  met for that suite until the orchestrator's run finishes.
- I stopped only my own per-file pytest processes. The orchestrator's full lerobot run was not touched,
  but the 23:24-23:32 part of it competed with my run and may be slower than normal.

Effect on sections 3 and 4 for lerobot_env: if test_crawl_two_policy.py really is ~60 s per test, the lever
is the per-test cost of the simulated crawl (wall-clock simulated motion at 8-10 deg/s with a mesh check per
step, `--max-s 30`), not the test count. Options that remove nothing, all needing owner approval as test
edits: (a) a simulated clock / faster sim speed in the dry-run harness, (b) a session fixture for the clamped
timeline + mesh checker, (c) xdist `-n 4` (15 crawl tests spread over 4 workers -> ~4 min). Removal of the
F11 AA-39 candidates saves only the AA-39 file's share (probably ~5 runs x ~30-60 s = ~2.5-5 min), and that
estimate has to be confirmed with the orchestrator's durations.


## 1b. lerobot_env durations (orchestrator sequential run 2026-11-24, EXIT 0; top 30)

```
179.75s call     lerobot_env/tests/test_crawl_two_policy.py::test_autonomous_original_order_left_far_from_its_finish_ref_is_logged_and_continues
103.60s call     lerobot_env/tests/test_crawl_two_policy.py::test_full_crawl_phase_order_and_gripper_sequence
90.26s call     lerobot_env/tests/test_crawl_two_policy.py::test_autonomous_runs_the_original_order_with_no_prompt_but_the_final_enter
88.07s call     lerobot_env/tests/test_crawl_two_policy.py::test_lock_far_from_the_finish_ref_is_logged_and_the_crawl_continues
67.30s call     lerobot_env/tests/test_crawl_two_policy.py::test_quit_in_the_right_policy_keeps_both_locked_left
64.73s call     lerobot_env/tests/test_crawl_two_policy.py::test_autonomous_regrip_first_runs_to_home_with_no_prompt_but_the_final_enter
58.06s call     lerobot_env/tests/test_crawl_two_policy.py::test_autonomous_far_at_the_lock_timer_retries_once_then_holds
47.34s call     lerobot_env/tests/test_crawl_two_policy.py::test_autonomous_stop_file_holds_both_arms
35.69s call     lerobot_env/tests/test_full_policy_test.py::test_full_test_left_policy_lock_cycle_right_policy_lock
34.89s call     lerobot_env/tests/test_crawl_two_policy.py::test_regrip_first_left_locked_far_from_its_finish_ref_never_moves_both_locked
27.33s call     lerobot_env/tests/test_full_policy_test.py::test_owner_says_the_left_key_is_not_in_nothing_unlocks
19.40s call     lerobot_env/tests/test_crawl_two_policy.py::test_owner_says_the_left_key_is_not_in_the_right_never_unlocks
18.39s call     lerobot_env/tests/test_policy_runner_lerobot.py::test_real_trained_checkpoint_loads_and_steps
17.58s call     lerobot_env/tests/test_full_policy_test.py::test_quit_after_the_left_attempt_locks_nothing
16.30s call     lerobot_env/tests/test_full_policy_test.py::test_lock_far_from_the_demo_end_stops_before_the_right_unlocks
12.49s call     lerobot_env/tests/test_eval_offline.py::test_evaluate_reports_per_joint_errors_and_hold_baseline
5.11s call     lerobot_env/tests/test_record_train.py::test_train_session_follows_the_schedule_and_repeats_a_discarded_placement
3.54s setup    lerobot_env/tests/test_inspect_dataset_v3.py::test_lerobot_v3_report
2.09s call     lerobot_env/tests/test_policy_test_sh.py::test_policy_never_starts_without_a_good_replay[state2-4]
2.01s setup    lerobot_env/tests/test_policy_runner_lerobot.py::test_loader_accepts_lerobot_061_config
1.55s call     lerobot_env/tests/test_inchworm_plugin.py::test_lerobot_record_end_to_end
1.40s call     lerobot_env/tests/test_policy_test_sh.py::test_arm_is_required
1.35s call     lerobot_env/tests/test_inchworm_plugin.py::test_lerobot_teleop_loop
1.26s call     lerobot_env/tests/test_inchworm_plugin.py::test_session_records_from_r_including_the_catch_up_with_the_sent_goal
1.25s call     lerobot_env/tests/test_policy_test_sh.py::test_policy_never_starts_without_a_good_replay[state1-0]
1.16s call     lerobot_env/tests/test_policy_test_sh.py::test_left_stops_at_42
1.14s call     lerobot_env/tests/test_policy_test_sh.py::test_policy_never_starts_without_a_good_replay[None-0]
1.13s call     lerobot_env/tests/test_policy_test_sh.py::test_right_is_disabled
0.91s call     lerobot_env/tests/test_policy_runner_lerobot.py::test_supervisor_dry_run_replay_never_commands_gripper
0.85s call     lerobot_env/tests/test_policy_runner_lerobot.py::test_backend_recorded_frame_gives_five_joint_targets
```
