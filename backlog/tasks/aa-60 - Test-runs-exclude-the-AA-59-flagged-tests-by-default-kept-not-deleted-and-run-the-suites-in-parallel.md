---
id: AA-60
title: >-
  Test runs: exclude the AA-59 flagged tests by default (kept, not deleted) and
  run the suites in parallel
status: Done
assignee:
  - '@agent'
created_date: '2026-11-24 04:51'
updated_date: '2026-11-24 06:36'
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
- [x] #1 A 'flagged' pytest marker excludes exactly the doc-4 removal candidates from the default run in both suites; no test file or test is deleted; one documented command runs them
- [x] #2 Both suites run in parallel via pytest-xdist by default, with no shared-path collisions (a parallel run gives the same pass/fail as a sequential one)
- [x] #3 Measured wall time of both default runs before and after is recorded in the notes
- [x] #4 AGENTS.md rule 7 and doc-3 name the default commit-gate commands and the command for the flagged tests
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
2026-11-24 commit 0474133 (sub-agent timed out at 30 min with the change done; the orchestrator verified and finished it). Marked flagged exactly: robot F1, F2 (TestJogDryRun delta/abs), F6, F7 (test_inchworm_plan), F8 (dry-run default [view_pose.py] only); lerobot F11 (3 of 5 in test_full_policy_test.py), F12 (test_policy_test_sh.py). Nothing deleted. Parallel: robot -n 4 --dist loadgroup, with the script-subprocess tests (test_script_imports, TestJogDryRun, TestJogAA9Integration, TestMeasureLimitsDryRun, TestEstopCLI, test_fault_inject_hw) in one xdist_group so they do not starve each other; BLAS threads capped to 1 in tests/conftest.py; test_script_imports also accepts 'from pkg import submodule' that has not been imported yet (an order dependence exposed by xdist; the check is not weakened: the name must still resolve). The 30 s timeout is unchanged. lerobot_env: -n 4 loadfile timed out under external load, so -n 3 --dist load. Measured (orchestrator, quiet machine): robot default 1968 passed in 137 s and 91 s (before: 435 s sequential); robot -m flagged 5 passed in 75 s; lerobot default 117 passed in 532 s and 519 s at -n 3 (701 s at -n 2; before: 899 s); lerobot -m flagged 9 passed in 73 s. The lerobot suite is bounded by the longest crawl test (~158 s) and by CPU: 10 simulated crawls at real motion speed. Residual risk: under external CPU load (dry runs, other sessions) the subprocess tests can hit their 30 s timeout, so do not run the gates alongside heavy jobs (AGENTS.md rule 7).
<!-- SECTION:NOTES:END -->
