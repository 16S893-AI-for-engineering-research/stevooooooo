---
id: AA-57
title: >-
  Policy rollout without the per-step mesh collision check (owner: the check
  slows the loop below the trained rate)
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-24 03:32'
updated_date: '2026-11-24 04:40'
labels:
  - safety
  - policy
dependencies: []
priority: high
ordinal: 57000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner 2026-11-24: while a learned policy drives an arm (inchworm_follower tube_support=handoff, PolicyAttempt in lerobot_env/policy_handoff.py, used by full_policy_test.py and crawl_two_policy.py), the per-goal cell-mesh check (~6-9 ms/step, measured 2026-11-23; loop 26-27.5 Hz instead of the 30 fps the demos were recorded at) slows the arm to a speed it was not trained at. The owner explicitly approves removing the mesh check DURING policy steps only. It is an owner-approved exception to the collision-check invariant, so it must be logged. Kept: joint/soft/EEPROM limits, the jump guard, the max-deg-s speed cap, stop files/Ctrl-C hold, the start handoff checks, and the mesh check on every planned move (catch-up start gate, returns, crawl phases, home).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 During policy steps no mesh collision check runs; limits, jump guard, speed cap and stop handling still apply (tests)
- [x] #2 Every non-policy motion (handoff catch-up/start, handoff_return, move_holding, crawl planned phases, home) is still mesh-checked (tests)
- [x] #3 The run log records that the mesh check was skipped during policy steps, citing the owner approval of 2026-11-24
- [ ] #4 The measured loop rate in a dry run is at least as high as the commanded fps, or the remaining limiter is identified with numbers
- [ ] #5 First live run owner-supervised and its loop rate recorded in the notes
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
2026-11-24: the first sub-agent timed out (30 min) after writing the change and tests. Orchestrator reviewed and applied it to main (uncommitted until the suites pass): InchwormFollowerConfig.policy_no_mesh_check (owner reason; refused unless tube_support=handoff); _handoff_goal returns the goal without the mesh check once catching_up is False; the catch-up and every step until it ends stay checked; connect logs mesh_check_skipped_policy with the reason. policy_handoff.POLICY_NO_MESH_CHECK is passed by policy_handoff.py, full_policy_test.py and crawl_two_policy.py; record_handoff and record_train are unchanged (a test asserts this). Tests: tests/test_policy_handoff.py 25 passed (+9 new: skip after catch-up, limits clamp, jump guard + speed cap + stop file, catch-up still mesh-checked with a hold, return still mesh-checked, teleop default checks every goal, refused outside handoff, runner wiring). Profile dry run (agent): with the check off, _handoff_goal 0.1 ms mean; policy.step p50 8.7 ms, p95 50 ms, max 170 ms; after-run attempt 308 steps / 16.4 s = 18.8 Hz with 103 overruns. Inference spikes, not the mesh check, limit the dry run. The after dry run stopped at the train_right gate (right shoulder_lift -52 ticks; max 34). Under investigation by a follow-up agent (likely sim tracking under CPU load; the right arm is not policy-driven there).

2026-11-24 commit 423049b. Full suites before commit: robot EXIT 0 (sequential), lerobot_env EXIT 0. ACs #1-#3 by tests/test_policy_handoff.py (25 passed incl. 9 new). Follow-up investigation (sub-agent, AGENT_REPORT_AA-57b): (1) the after-run train_right gate stop was NOT caused by AA-57. The right_to_train_right planned stream ran late under CPU load (3.64 s for a 2.59 s plan, 89 vs ~126 loops); the stream does not catch up after a host stall, so the arm ended short (lift -52 ticks) and the gate refused (the safe outcome). A repro with the change passed the gate (exit 0). Live risk: a host stall > ~0.3 s in a planned phase stops at the gate (owner decision whether the stream should catch up). (2) Loop rate with the mesh check off: 27.3-27.7 Hz uncontended (repro under 1 suite: left 24.7 Hz / 95 overruns, right 27.3 Hz / 20 overruns; infer p50 9-12 ms, p95 39-79, max ~195). Remaining limiter = ACT chunk inference every 20th step (n_action_steps=20) at ~80-96 ms vs the 33 ms budget, giving ~28 Hz max. Camera replay 1.1 ms, send_action 0.1 ms. AC #4 open (the rate is still below 30 fps; limiter identified with numbers). Proposed, not implemented: compute the next chunk in the background before the queue empties; log the device and chunk vs step inference times. AC #5 (live) open.
<!-- SECTION:NOTES:END -->
