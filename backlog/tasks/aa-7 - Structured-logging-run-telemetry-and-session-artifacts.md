---
id: AA-7
title: 'Structured logging, run telemetry and session artifacts'
status: To Do
assignee: []
created_date: '2026-10-20 21:08'
labels: []
milestone: m-0
dependencies:
  - AA-4
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 7000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The system is meant to run unattended through repeated crawl, insert, lock, fold, level and print cycles. When a cycle fails at 2 am there is no operator watching, so the only way to diagnose it is the record left behind. Debugging servo-level faults, policy aborts and leveling outliers after the fact requires timestamped, machine-readable traces from the very first driver, which is why this precedes hardware bring-up.

Keep artifacts local by default: camera frames and session traces are not to be committed to the repository.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A logging module provides structured (JSON-lines) event records with monotonic timestamps, logical device role, subsystem and severity, plus human-readable console output
- [ ] #2 Each run writes a session directory containing the event log, the resolved hardware config snapshot and any captured artifacts, under a gitignored path
- [ ] #3 Motion commands, gripper lock and unlock transitions, policy handoffs, aborts and retries each emit a distinct typed event with the fields needed to reconstruct the cycle
- [ ] #4 A summarizer renders one session into a readable per-cycle timeline (durations, outcomes, failures) from the event log alone
- [ ] #5 Session artifact paths are gitignored and no frames or traces are committed by this task
- [ ] #6 Tests cover event schema stability and the summarizer output for a synthetic session
<!-- AC:END -->
