---
id: AA-25
title: Train and evaluate the insertion policy
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
labels: []
milestone: m-3
dependencies:
  - AA-23
  - AA-24
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 25000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
This is the learned component the owner specified: final insertion into the attachment point, bounded by code on both sides. Training must fit a 4 GB GPU, so architecture, image resolution and batch size are constrained, and the evaluation has to be against the real capture range produced by the coarse approach rather than against a convenient subset.

The acceptance bar is behavioural: the policy must succeed often enough, and fail safely enough, that the supervised autonomous cycle is viable. A policy that succeeds frequently but occasionally drives hard into the structure is worse than a slightly less successful but gentle one.

See doc-1 sections 7 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A training pipeline trains a LeRobot policy on the captured dataset within the 4 GB VRAM budget, with the chosen architecture, resolution, batch size and hyperparameters recorded
- [ ] #2 Training is reproducible from a pinned config and a recorded dataset version, and produces a versioned checkpoint artifact outside version control
- [ ] #3 Offline evaluation reports held-out action error, and online evaluation reports insertion success rate over at least 30 real attempts spanning the measured coarse-approach offset range
- [ ] #4 Success is judged by the independent AA-24 seating detector, not by the policy or by visual impression
- [ ] #5 Failure modes are characterized and reported (for example miss direction, jam, oscillation, timeout) with counts
- [ ] #6 Maximum observed contact force or servo load during attempts is recorded, and runs that exceed a documented safe load are counted as unsafe failures regardless of outcome
- [ ] #7 A go or no-go recommendation for autonomous use is stated with the measured success rate, the unsafe-failure count and the retry count needed to reach an acceptable per-cycle success probability
<!-- AC:END -->
