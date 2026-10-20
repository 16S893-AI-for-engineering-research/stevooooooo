---
id: AA-12
title: DexArm G-code driver and connection management
status: To Do
assignee: []
created_date: '2026-10-20 21:10'
labels: []
milestone: m-1
dependencies:
  - AA-5
  - AA-7
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 12000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The DexArm carries both the print head and the laser scanner, so leveling and printing both depend on being able to command and query it reliably. It speaks a Marlin-style G-code dialect over USB serial with its own Rotrics quirks, and nothing has been driven from this host yet. A thin, well-tested driver is needed before any scan-path or print integration work.

The driver must know when a move has actually finished: the leveling scan needs the arm to be settled at each sample pose, and asynchronous G-code acknowledgement is a classic source of silent misalignment.

See doc-1 sections 3, 7 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Driver connects to the DexArm by logical role, performs handshake, reports firmware identity, and recovers from a disconnect with a clear typed error rather than hanging
- [ ] #2 Driver supports homing, absolute and relative linear moves in mm, feedrate control, position query and a documented safe-park command
- [ ] #3 Move completion is positively confirmed (blocking acknowledgement or position query convergence) so callers can rely on the arm being settled before capturing a frame
- [ ] #4 A command queue serializes access so concurrent callers cannot interleave G-code lines, and malformed or rejected commands raise typed errors including the firmware response
- [ ] #5 Workspace bounds for the DexArm are configured and commanded moves outside them are rejected before transmission
- [ ] #6 A mock DexArm backend emulates acknowledgements and position queries, and tests cover handshake, move confirmation, bounds rejection and error mapping
<!-- AC:END -->
