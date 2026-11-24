---
id: AA-61
title: Planned streams catch up after a host stall instead of ending short
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 04:51'
labels:
  - motion
  - safety
dependencies: []
priority: medium
ordinal: 61000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
AA-57 investigation 2026-11-24: cycle_replay's planned stream sets each servo speed from the plan, not from the lag. After a host stall (CPU load) the arm ends short of the last frame. Example: right_to_train_right ended 52 ticks off and the train_right gate refused, which is safe but stops the crawl. The owner asked the orchestrator to use its best judgment. Chosen approach: if the arm is still short after the stream, finish the REMAINING part of the already-preflighted plan at the planned tempo (no new path), then wait for arrival. The arrival tolerance and the gates are unchanged.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 After a stream, if any joint is outside the arrival tolerance, the remainder of the same preflighted plan is re-streamed from the nearest planned point; any bridge from the present pose to that point is limits- and mesh-checked; at most one catch-up per segment, and the stop file / Ctrl-C still hold
- [ ] #2 Arrival tolerances, the training-pose gates and the speed ceiling are unchanged
- [ ] #3 Tests with a simulated stall (e.g. 300-600 ms) show the arm ends within tolerance; without a stall, behaviour and timing are unchanged
- [ ] #4 Dry run of the autonomous crawl under CPU load reaches the train_right gate cleanly; the first live use is owner-supervised
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
