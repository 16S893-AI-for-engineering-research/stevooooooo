---
id: AA-59
title: Flag slow or redundant tests for removal (report only)
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-24 03:33'
labels:
  - tests
dependencies: []
priority: medium
ordinal: 59000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner 2026-11-24: the suites take far too long (lerobot_env 109 tests = 15 min; robot 1960 tests = 7 min; measured 2026-11-24). The owner wants candidates for removal flagged. Per AGENTS.md rule 3, no test is deleted or skipped without the owner's approval, so this task only produces a report for the owner to decide on.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A report (backlog doc or task notes) lists the slowest tests of both suites with measured durations (pytest --durations)
- [ ] #2 Each flagged test has a reason (redundant with test X, duplicate end-to-end path, obsolete feature), the invariant it covers, and whether another test still covers it
- [ ] #3 Safety-invariant tests (one camera, never both unlocked, gripper only via AA-17, laser off, limits) are marked keep unless an equivalent test is named
- [ ] #4 The estimated time saved per suite if all flagged tests were removed, plus faster alternatives to removal (shared fixtures, a slow marker excluded by default, parallel runs)
- [ ] #5 No test file is changed in this task
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
