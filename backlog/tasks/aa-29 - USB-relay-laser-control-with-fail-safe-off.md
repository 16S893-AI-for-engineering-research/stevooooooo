---
id: AA-29
title: USB relay laser control with fail-safe off
status: To Do
assignee: []
created_date: '2026-10-20 21:13'
labels: []
milestone: m-5
dependencies:
  - AA-5
  - AA-7
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 29000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The line laser is powered through a USB serial relay module, which makes laser state software-controlled. That is convenient for automation and dangerous if a crash leaves the laser energized: it is a visible-light line laser aimed across the workspace. The control layer therefore needs guaranteed-off semantics on every exit path, including exceptions, emergency stop and process death, before any scanning work builds on it.

The relay protocol must be determined empirically from the device and documented, since these modules vary.

See doc-1 section 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The relay serial protocol is determined and documented, and a driver turns the laser on, turns it off and reports the commanded state by logical role from config
- [ ] #2 Laser-on is exposed as a context manager that turns the laser off on normal exit and on exception
- [ ] #3 Emergency stop (AA-10) turns the laser off, verified by a test
- [ ] #4 Process exit, including SIGINT and SIGTERM, turns the laser off, and the residual risk if the process is SIGKILLed is documented with a mitigation
- [ ] #5 Commanded state is tracked and a mismatch between commanded and reported relay state raises a typed error rather than being ignored
- [ ] #6 Cumulative laser-on time per session is logged, and a configured maximum continuous-on duration is enforced
- [ ] #7 Tests cover on, off, exception-path off, state-mismatch detection and the duration limit against the mock backend
<!-- AC:END -->
