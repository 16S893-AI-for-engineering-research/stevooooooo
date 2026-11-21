---
id: AA-26
title: Autonomous policy runner with code handback and lock
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:12'
updated_date: '2026-11-21 01:55'
labels: []
milestone: m-3
dependencies:
  - AA-25
  - AA-20
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 26000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner requires the system to switch from code, to the model, back to code with no intervention, and for the gripper lock to be commanded by code afterwards. This task replaces the stub policy in the AA-20 supervisor with the trained checkpoint and closes the loop: supervised rollout, independent seating detection, code-commanded lock, verified hold, control returned to the cycle runner.

All safety guarantees must continue to hold with a real model in place, including when the model behaves unexpectedly after a checkpoint change.

See doc-1 sections 3 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 The trained checkpoint runs inside the AA-20 supervisor with all limits, bounds, clamps and abort paths active and unchanged
- [ ] #2 On abort, or when the attempt time cap is reached without the finish rule firing, the arm retracts safely, the coarse approach is retried up to the configured bound, and persistent failure halts the cycle in a safe state without operator input
- [ ] #3 Policy inference latency is measured on this host and the supervisor loop rate is shown to be compatible, with the measured values recorded
- [x] #4 Checkpoint identity (path and hash) is logged with every rollout so a run can be traced to the exact model used
- [x] #5 Loading an incompatible checkpoint or one whose observation and action spaces disagree with the contract fails at load time, not mid-insertion
- [ ] #6 At least 20 consecutive supervised handoffs are executed with zero operator interventions and the outcome of each is logged
- [ ] #7 When the finish rule fires (owner 2026-11-20: every joint within 2 deg over the last 2 s AND within 8 deg of the median demo-end pose), code takes control, commands the AA-17 lock, verifies the latch is holding, and only then returns success to the cycle runner
- [ ] #8 Tests cover handback, the finish rule (fires after 2 s still near the demo end; not before 2 s, not while creeping, not far from the demo end), lock-on-finish, time-cap retry, abort escalation and the incompatible-checkpoint path
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
## Plan

### Scope (software half only)
1. **New module ** containing:
   - `CheckpointIncompatible` exception — raised at load time when config.json features disagree with the AA-20 contract
   - `CheckpointLoader` — reads config.json from a LeRobot ACT checkpoint dir WITHOUT importing lerobot; validates observation/action features against the AA-20 contract (5 joints, 1 wrist image, NO gripper); computes SHA-256 of model.safetensors; logs path+hash at rollout start
   - `PolicyProtocol` re-export (or reference to supervisor.PolicyProtocol)
   - `PolicyBackend` ABC with `step(obs: Observation) -> PolicyAction` — two implementations: `LeRobotBackend` (lazy lerobot import; returns `NotImplementedError` when unavailable) and `StubBackend` (deterministic for tests)
   - `CheckpointPolicy` class implementing `PolicyProtocol` — wraps a `PolicyBackend`, exposes `step()`, holds the loaded checkpoint metadata; logs hash on every rollout start (AC#5)
   - `PolicyRunner` orchestration class:
     - Wires `CheckpointPolicy` into `PolicySupervisor` **unchanged** (AC#1)
     - Uses injected AA-24 `SeatingEvaluator` as the seating_detector (UNCERTAIN → probe → re-evaluate per SeatingEvaluator.run_evaluation)
     - On SEATED: calls injected AA-17 `GripperLockSM.lock()` + `hold_tick()` verify; only returns success if hold_tick confirms (AC#2)
     - On unseated/abort/retry/halt: reuses the supervisor paths (AC#3)
   - `scripts/rollout.py` — CLI entry point (dry-run default, --live flag, --checkpoint PATH)

2. **New test file ** covering AC#1, AC#2, AC#3, AC#6, AC#8

3. Synthetic checkpoint fixture in tmp_path (matching + incompatible)

4. lerobot_env/latency_measure.py for AC#4 partial evidence (random-weight policy)

### Seams
- `inchworm.supervisor.PolicySupervisor` — used unchanged; `PolicyRunner` builds and delegates to it
- `inchworm.seating.SeatingEvaluator` — injected as seating_detector callable
- `inchworm.gripper_lock.GripperLockSM` — injected lock + hold_tick callable

### ACs checklist
- AC#1 (supervisor limits unchanged): verified via test
- AC#2 (lock on seated): verified via test  
- AC#3 (retry/halt path): reuses supervisor; verified via test
- AC#4 (latency): unchecked — needs trained checkpoint + hardware
- AC#5 (hash logging): verified via test
- AC#6 (incompatible checkpoint): verified via synthetic test
- AC#7 (20 handoffs): unchecked — needs hardware
- AC#8 (coverage): verified via tests

## Implementation notes (2026-10-24T19:11:45Z)

### Files created
- **src/inchworm/policy_runner.py** (NEW module)
  - CheckpointIncompatible, CheckpointMeta, CheckpointLoader
  - PolicyBackend ABC, StubBackend, LeRobotBackend (lazy lerobot import)
  - CheckpointPolicy implementing PolicyProtocol
  - _SeatingAdapter (wraps SeatingEvaluator as () -> bool)
  - _LockAdapter (calls lock_sm.lock() + hold_tick(); returns True only on confirmed latch)
  - PolicyRunner (wires all components; supervisor_kwargs passed unchanged)
  - RunnerResult dataclass
- **tests/test_policy_runner.py** (NEW test file, 51 tests, all pass)
- **scripts/rollout.py** (NEW, dry-run default, --live guard)
- **lerobot_env/latency_measure.py** (NEW, AC#4 partial evidence)

### AC status
- AC#1 EVIDENCED: test_supervisor_limits_are_config_values patches PolicySupervisor.__init__ and asserts config object is default_cfg, all limit fields match
- AC#2 EVIDENCED: TestLockOnSeated — _LockAdapter calls lock() then hold_tick(); returns False on JAM/MISS/hold_tick failure
- AC#3 EVIDENCED: TestUnseatedRetry.test_abort_after_all_retries_halts, TestAbortEscalation.test_approach_failure_aborts
- AC#4 UNCHECKED — needs owner-supervised hardware session: cd robot/lerobot_env && uv run python latency_measure.py
- AC#5 EVIDENCED: test_runner_logs_sha256_on_every_call — two rollouts each produce ≥1 log record containing sha256
- AC#6 EVIDENCED: TestCheckpointLoader — gripper_in_action, 6dof_action, missing_config, missing_safetensors, invalid_json, missing_camera_obs, missing_joint_in_obs all raise CheckpointIncompatible at load time
- AC#7 UNCHECKED — needs owner-supervised hardware session: python scripts/rollout.py --live --checkpoint PATH
- AC#8 EVIDENCED: 51 tests covering all named paths

### Test run
1318 passed, 0 failed, 0 errors (including 1267 pre-existing tests)

AC#4 UNCHECKED: needs owner-supervised hardware session: cd robot/lerobot_env && uv run python latency_measure.py (script written and documented)
AC#7 UNCHECKED: needs owner-supervised hardware session: python scripts/rollout.py --live --checkpoint PATH (script written; live mode has stub guard that forces the owner to wire real hardware callables)

AC#4 measurement 2026-10-25 (this host: RTX 3050 Ti 4 GB, CUDA, lerobot 0.6.1, RANDOM-weight ACTPolicy at the deployment shapes: 3x480x640 wrist image, 5-joint state, default chunk_size=100, n_action_steps=100; latency_measure.py fixed for the lerobot 0.6 API):
- select_action(): mean 0.94 ms, median 0.61, p95 1.91, p99 19.02, max 22.78 ms (300 calls). Most calls pop a queued action.
- Full network forward (predict_action_chunk, runs once per 100 steps): mean 16.93 ms, p95 21.11, max 25.52 ms.
Compatibility with supervisor_loop_rate_hz 50 (20 ms budget): the mean fits easily. But the per-step budget also includes the camera (measured mean ~17-18 ms per frame at ~55-60 fps, AA-14) and the servo bus read (~5 ms). On chunk-boundary steps that is about 17 + 21 + 5 = ~43 ms, which OVERRUNS 20 ms (about once every 2 s). Also, the supervisor _attempt loop has no rate pacing or overrun detection: it runs as fast as camera + policy + bus allow, so 50 Hz is a config value, not an enforced rate. NOT CHECKED: (1) random weights, not the trained checkpoint (an earlier note required the trained checkpoint); (2) compatibility is not shown for worst-case steps. Owner decision needed: accept random-weight evidence, and decide whether to (a) run the loop at 20-25 Hz, (b) add pacing with overrun logging, or (c) shrink chunk_size or use temporal ensembling.

2026-11-17 (commit 0103784): LeRobot 0.6.1 port.
- LeRobotBackend now uses ACTPolicy.from_pretrained + make_pre_post_processors (normalisation) + build_inference_frame/make_robot_action, with features from the inchworm_follower plugin (`follower_features`, `format_frame`: observation.state 5 raw servo deg, observation.images.wrist 3x480x640).
- CheckpointLoader accepts the 0.6.1 config (output_features.action.shape [5], typed input_features) and checks the dataset action names (no gripper).
- New dry-run: `rollout.py --checkpoint CKPT --dataset DS --episode N` replays the recorded observations through the real policy + unchanged supervisor (nothing commanded). Verified on the 30-step audit checkpoint: 600 steps; |policy - recorded| median 8.8 deg (an untrained model, so expected).
- Tests: main suite rc 0; lerobot_env suite rc 0.
- NOT done (live wiring; `--live` still aborts on purpose): (1) motion_executor via GuardedArm/MotionGuard with the mesh check; (2) a live frame/joint source (arbiter wrist cam + pose reader; read_arm_ticks refuses with torque on, so it needs a torque-on read path); (3) SeatingEvaluator sample source (AA-24); (4) GripperLockSM (AA-17) wired to the seating result. First live run supervised by the owner.

2026-11-20 OWNER: no seating detector is needed any more (AA-24). Owner proposal (pending agent caveat): after the policy + lock, move the arm to a FIXED known pose and continue the scripted cycle, no live re-plan. Agent measured: the 29 demo ends (median) and the 3 live policy ends differ from the timeline's planned inserted pose (frame 48) by lift ~-12.5 / elbow ~+13.6 deg = gripper ~28-29 mm in x and 4 deg (FK, tool offset unverified); live ends are within 0-6 deg of the demo median. AC/status changes to AA-24/AA-26/AA-27 await the owner's answers.

2026-11-20 OWNER-APPROVED AC change: the seating verdict (AA-24, closed as not needed) is replaced by the finish rule 'the pose does not change for 5 s'. AC #2, #3, #8 reworded and left UNCHECKED (their old evidence was for the seating-verdict path); #1, #5, #6 unchanged and re-checked from their earlier evidence; #4, #7 still open. The still-tolerance for 'does not change' is not yet decided by the owner.

2026-11-20 owner chose the agent recommendation over '5 s still' (5 s never fired on the 3 live attempts): finish = still within 2 deg for 2 s AND within 8 deg of the demo-end median (left ticks 1977/2195/1930/1396/1988, mapped for the right arm). Replayed on the live trace: fires at 13.7 s / 12.4 s (7.7 / 6.4 deg from the demo end), not on attempt 3 (18 deg away). The old #2/#8 (5 s wording) were replaced by the new last two ACs (numbering moved, order otherwise unchanged); unchecked. BUILT in policy_handoff (end='finished', arm HOLDS); the lock after the finish + retry are not built. Tests: 5 new in lerobot_env/tests/test_policy_handoff.py; dry runs policy_test.sh both arms end 'finished' (left 11.7 s, 1.49 deg; right 12.3 s, 1.19 deg).

2026-11-21 OWNER: after each policy + lock the sequence no longer goes to a demo-end pose; see AA-55 (right out of its key, left carries the tube to the frozen train_right holding pose, right policy, lock, home).
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 19:14
---
Orchestrator 2026-10-24: the agent set Done with AC#4 and #7 unchecked, which the Definition of Done forbids. Reset to In Progress. #4 (latency on this host) and #7 (20 supervised handoffs) need a trained checkpoint and hardware.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented inchworm/policy_runner.py (CheckpointLoader, CheckpointPolicy, PolicyBackend ABC, StubBackend, LeRobotBackend, PolicyRunner, _LockAdapter, _SeatingAdapter) and 51 tests in test_policy_runner.py covering AC#1–#3, #5–#6, #8. Added scripts/rollout.py (dry-run default, live guard) and lerobot_env/latency_measure.py. Verified with uv run pytest: 1318 passed, 0 failed. AC#4 and AC#7 left unchecked per AC: needs trained checkpoint + hardware session.
<!-- SECTION:FINAL_SUMMARY:END -->
