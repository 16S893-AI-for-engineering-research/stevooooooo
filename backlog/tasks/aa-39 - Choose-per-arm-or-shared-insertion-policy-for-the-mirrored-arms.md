---
id: AA-39
title: Choose per-arm or shared insertion policy for the mirrored arms
status: To Do
assignee: []
created_date: '2026-10-24 16:07'
labels: []
milestone: m-3
dependencies:
  - AA-25
type: task
ordinal: 39000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The left and right SO-101 arms insert from mirrored sides of the tube. The owner approved starting with one arm's policy (AA-23/AA-25) and adding the other later. Afterwards a choice is needed: train a second, separate policy, or one shared policy on mirrored data with an arm-identity input. That choice drives the owner's demonstration burden, since a second policy roughly doubles it.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Once the first arm's policy passes AA-25, a written comparison of a separate second policy versus a shared mirrored policy states the expected demo count, the risk and the recommendation
- [ ] #2 The owner's choice is recorded as a decision record, and follow-up capture and training tasks for the second arm are created to match
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
