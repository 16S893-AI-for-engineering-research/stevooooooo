---
id: AA-26
title: Autonomous policy runner with code handback and lock
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
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
- [ ] #1 The trained checkpoint runs inside the AA-20 supervisor with all limits, bounds, clamps and abort paths active and unchanged
- [ ] #2 On a seated verdict, code takes control, commands the AA-17 lock, verifies the latch is holding, and only then returns success to the cycle runner
- [ ] #3 On abort or an unseated verdict, the arm retracts safely, the coarse approach is retried up to the configured bound, and persistent failure halts the cycle in a safe state without operator input
- [ ] #4 Policy inference latency is measured on this host and the supervisor loop rate is shown to be compatible, with the measured values recorded
- [ ] #5 Checkpoint identity (path and hash) is logged with every rollout so a run can be traced to the exact model used
- [ ] #6 Loading an incompatible checkpoint or one whose observation and action spaces disagree with the contract fails at load time, not mid-insertion
- [ ] #7 At least 20 consecutive supervised handoffs are executed with zero operator interventions and the outcome of each is logged
- [ ] #8 Tests cover handback, lock-on-success, unseated retry, abort escalation and the incompatible-checkpoint path
<!-- AC:END -->
