---
id: AA-28
title: Crawl cycle fault recovery and tube-state verification
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
labels: []
milestone: m-4
dependencies:
  - AA-27
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 28000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
For unattended operation the interesting cases are not the successful cycles but the ones that go wrong: a missed insertion, a latch that does not hold, a servo fault mid-advance, a power-cycle during a hold, or a tube that has shifted relative to the model. Recovery has to be automatic where it is safe to be automatic, and must stop cleanly where it is not, with enough recorded state that the system can be resumed rather than re-homed blindly.

This task hardens the cycle produced by AA-27 and is what makes the long unattended run in M6 credible.

See doc-1 sections 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A verification step confirms, before and after each cycle, that each arm is locked into the lattice index the model believes it holds, using measurable signals
- [ ] #2 Each enumerated fault (insertion failure after retries, lock verification failure, mid-advance servo fault, unexpected tube displacement, loss of USB device) has a defined automatic response or a defined safe halt, and the mapping is documented
- [ ] #3 Recovery never requires both grippers to be unlocked simultaneously and never commands motion from an unverified state
- [ ] #4 Interrupted-cycle state is persisted such that a resume either continues correctly or refuses with a clear reason, and a stale or inconsistent state is always refused
- [ ] #5 Each fault path is exercised at least once by deliberate injection on hardware or on mocks where hardware injection is unsafe, with the method recorded per fault
- [ ] #6 A post-run report lists every fault encountered, the response taken and whether it succeeded
- [ ] #7 Tests cover state verification, the resume-refusal path and each injected fault response
<!-- AC:END -->
