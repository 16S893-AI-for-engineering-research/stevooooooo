---
id: AA-36
title: Prototype acceptance test and operations runbook
status: To Do
assignee: []
created_date: '2026-10-20 21:14'
labels: []
milestone: m-6
dependencies:
  - AA-35
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: task
ordinal: 36000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The prototype needs one agreed definition of working, plus the written procedure to reproduce it after the project has been idle or hardware has been disturbed. Without this, every future change is validated ad hoc and regressions in a mechanically sensitive system go unnoticed until something breaks physically.

The runbook should cover cold start from a powered-off machine (including usbipd attach), the order calibrations must be re-run after disturbance, and what to check when the system halts.

See doc-1 sections 7, 8 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 An acceptance test defines the pass criteria for the prototype (cycle count, insertion success rate, advance accuracy, leveling residual, zero interventions) and is runnable as a single documented procedure
- [ ] #2 The acceptance test is executed once end to end and the results are recorded, with each criterion marked pass or fail
- [ ] #3 A runbook documents cold start from power-off including device attach, preflight checks, normal start, stop, emergency stop and recovery from each documented halt reason
- [ ] #4 The runbook states which calibrations must be re-run after which kinds of disturbance, with the commands for each
- [ ] #5 A troubleshooting section maps the observable symptoms of the known failure modes to their diagnostic steps
- [ ] #6 Known limitations and open risks are listed explicitly, including anything deferred from earlier tasks
- [ ] #7 The runbook is verified by following it from a cold start without consulting any other document, and gaps found are fixed
<!-- AC:END -->
