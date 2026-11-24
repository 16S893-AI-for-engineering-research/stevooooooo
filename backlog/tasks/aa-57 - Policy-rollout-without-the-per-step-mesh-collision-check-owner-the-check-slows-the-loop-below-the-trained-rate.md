---
id: AA-57
title: >-
  Policy rollout without the per-step mesh collision check (owner: the check
  slows the loop below the trained rate)
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 03:32'
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
- [ ] #1 During policy steps no mesh collision check runs; limits, jump guard, speed cap and stop handling still apply (tests)
- [ ] #2 Every non-policy motion (handoff catch-up/start, handoff_return, move_holding, crawl planned phases, home) is still mesh-checked (tests)
- [ ] #3 The run log records that the mesh check was skipped during policy steps, citing the owner approval of 2026-11-24
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
