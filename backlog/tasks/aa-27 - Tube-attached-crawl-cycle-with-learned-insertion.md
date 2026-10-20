---
id: AA-27
title: Tube-attached crawl cycle with learned insertion
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
labels: []
milestone: m-4
dependencies:
  - AA-21
  - AA-26
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 27000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
This is the first time the full cycle runs on the real structure: one arm holds the tube while the other unlocks, retracts, coarse-approaches the next attachment point, inserts under the learned policy, locks under code control, and both arms fold. Because the chassis is fixed and the tube moves, the net result of each cycle is the tube advancing one 183.676 mm pitch.

The critical new risk relative to the dry cycle (AA-21) is that the tube is load-bearing: at no point may both grippers be unlocked, and any fault must leave the tube supported. That invariant deserves explicit enforcement rather than careful sequencing alone.

See doc-1 sections 1, 2 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A full cycle advances the tube exactly one 183.676 mm pitch with both arms ending locked and folded, verified by measurement over at least 3 consecutive cycles
- [ ] #2 An always-one-arm-holding invariant is enforced structurally: any command sequence that would unlock both grippers is rejected before execution, verified by a test
- [ ] #3 Measured advance error per cycle and accumulated error over the consecutive cycles are reported per axis
- [ ] #4 Any fault during the cycle leaves the tube supported by at least one locked gripper, and the resulting state is logged and documented as recoverable
- [ ] #5 Lattice bookkeeping and tube pose in the frame model agree with the measured physical advance after each cycle, and disagreement beyond a configured tolerance halts the run
- [ ] #6 Cycle time and per-phase durations are recorded and compared against the dry-cycle baseline from AA-21
- [ ] #7 The run is executed with zero operator intervention between cycles and the intervention count is reported
<!-- AC:END -->
