---
id: AA-35
title: Autonomous multi-cycle print-crawl loop
status: To Do
assignee: []
created_date: '2026-10-20 21:14'
labels: []
milestone: m-6
dependencies:
  - AA-28
  - AA-33
  - AA-34
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 35000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The end goal: the system repeats print, crawl, regrip, fold, level, print with no human in the loop, continuously extending the tube. Every subsystem exists by this point, so this task is about orchestration, invariants across subsystem boundaries, and surviving long enough to be useful. The USB single-camera rule, the always-one-arm-holding rule and the code-owned lock must all hold across every transition, including during recovery.

Unattended operation also means bounded resource use: session artifacts, logs and frames accumulate, and a run that fills the disk at cycle 30 has failed.

See doc-1 sections 1, 6 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A single entry point runs N consecutive print-crawl cycles unattended, with each cycle executing crawl, regrip with learned insertion, code-commanded lock, fold, laser leveling and segment print in the documented order
- [ ] #2 The single-camera USB invariant, the always-one-arm-holding invariant and the code-only-gripper invariant are asserted at every phase transition, and a violation halts the run
- [ ] #3 At least 5 consecutive cycles complete with zero operator interventions, and the intervention count is reported
- [ ] #4 Accumulated geometric error (tube advance, leveling residual and print-frame drift) is measured per cycle across the run and reported as a trend rather than a single value
- [ ] #5 Any unrecoverable fault halts the run in a safe state with the tube supported, the laser off and the nozzle retracted, and the halt reason is recorded
- [ ] #6 A run can be stopped and resumed, with resume either continuing correctly from persisted state or refusing with a clear reason
- [ ] #7 Session artifact and log growth per cycle is measured and bounded by a documented retention policy so a long run cannot exhaust disk space
- [ ] #8 A per-run report summarizes cycles attempted, cycles succeeded, faults, interventions and the error trend
<!-- AC:END -->
