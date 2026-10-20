---
id: AA-24
title: Insertion seating detection independent of the policy
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
labels: []
milestone: m-3
dependencies:
  - AA-8
  - AA-17
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 24000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Code, not the model, decides when insertion has succeeded, because the lock must only be commanded once the gripper is actually seated in the attachment point. A false positive locks into thin air and the next advance drops the tube; a false negative wastes retries. The detector therefore needs to be a separate, independently testable component rather than a side output of the policy.

The primary signal candidates are gripper-servo position and current or load from the shared servo, plus joint-load signatures on the arm; vision is a fallback if those prove insufficient. Which signals suffice is an open question in doc-1 section 10 and must be answered with measured data.

See doc-1 sections 3, 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A detector returns seated, not-seated or uncertain from sensor signals available during insertion, with the signals and thresholds documented
- [ ] #2 Thresholds are derived from recorded data covering at least 20 genuine seatings and 20 near-miss or jam cases, and the dataset used is retained
- [ ] #3 False-positive and false-negative rates are measured on held-out cases and reported, with false positives treated as the more serious failure and explicitly minimized
- [ ] #4 The detector does not consume policy internals or policy confidence and can be evaluated standalone from a recorded trace
- [ ] #5 The uncertain verdict triggers a documented disambiguation step (for example a gentle probe move or a vision check) rather than defaulting to seated
- [ ] #6 Detection runs within a latency budget compatible with the supervisor loop rate, measured and recorded
- [ ] #7 Tests cover each verdict path and threshold-boundary behaviour against recorded traces
<!-- AC:END -->
