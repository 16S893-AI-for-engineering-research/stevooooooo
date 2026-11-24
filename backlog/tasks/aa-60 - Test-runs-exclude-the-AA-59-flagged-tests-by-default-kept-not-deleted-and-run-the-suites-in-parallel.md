---
id: AA-60
title: >-
  Test runs: exclude the AA-59 flagged tests by default (kept, not deleted) and
  run the suites in parallel
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 04:51'
labels:
  - tests
dependencies: []
priority: medium
ordinal: 60000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner 2026-11-24 after the AA-59 report (doc-4): the suites take too long (robot 435 s, lerobot_env 899 s). The owner approved (1) not running the tests flagged as removal candidates in doc-4 (F1, F2, F6, F7, F8, the 3 F11 candidates, F12). Keep them in the repo, out of the default run, and runnable on demand, because the owner may want them back. (2) Running the suites in parallel (pytest-xdist). This is an owner-approved exception to AGENTS.md rule 3 for exactly the doc-4 candidates; KEEP tests stay in the default run.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A 'flagged' pytest marker excludes exactly the doc-4 removal candidates from the default run in both suites; no test file or test is deleted; one documented command runs them
- [ ] #2 Both suites run in parallel via pytest-xdist by default, with no shared-path collisions (a parallel run gives the same pass/fail as a sequential one)
- [ ] #3 Measured wall time of both default runs before and after is recorded in the notes
- [ ] #4 AGENTS.md rule 7 and doc-3 name the default commit-gate commands and the command for the flagged tests
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
