---
id: AA-61
title: Planned streams catch up after a host stall instead of ending short
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 04:51'
updated_date: '2026-11-24 06:36'
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
- [x] #1 After a stream, if any joint is outside the arrival tolerance, the remainder of the same preflighted plan is re-streamed from the nearest planned point; any bridge from the present pose to that point is limits- and mesh-checked; at most one catch-up per segment, and the stop file / Ctrl-C still hold
- [x] #2 Arrival tolerances, the training-pose gates and the speed ceiling are unchanged
- [x] #3 Tests with a simulated stall (e.g. 300-600 ms) show the arm ends within tolerance; without a stall, behaviour and timing are unchanged
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-24 commit a1d3a3d (sub-agent; reviewed and applied by the orchestrator). cycle_replay._stream_segment = _stream_path(seg, 0) + settle wait + at most one _catch_up when any joint is still outside REACH_TOL_TICKS. _catch_up joins the planned 20 ms stream sample nearest the present pose (worst-joint ticks, both arms). The bridge is a joint line checked by catch_up_bridge_problems (soft + EEPROM limits, the cell mesh checker at >=1 sample/deg, the tube where the plan puts it; fails closed, refused = no motion, logged), run slowly at APPROACH_TICKS_S = 57 ticks/s; then the rest of the plan streams at the planned tempo. Logged stream_catch_up start/done/refused with lag ticks. Tests: stall 0.3/0.45/0.6 s left the arms 66-146 ticks short, ending within tolerance after one catch-up; no stall = identical sync-writes (same sha256 of 204 writes vs the old code); stop file during the catch-up -> ReplayAborted; never two catch-ups; bridge limit + mesh checked. test_cycle_replay 33 passed 3 skipped; full suites pass. Agent dry run (no load): crawl_done rc 0, gates clean, 0 catch-ups. Under 12 busy-loop CPU hogs, the LEFT POLICY hit its time cap twice (11.2 then 8.4 deg) and the run stopped holding before any later stream, so AC #4 is not shown: the policy under CPU load, not the stream, failed first (it ties in with AA-57 inference speed). AC #4 open; live use owner-supervised.
<!-- SECTION:NOTES:END -->
