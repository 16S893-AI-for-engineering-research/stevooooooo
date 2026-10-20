---
id: AA-10
title: 'Safe homing, torque management and emergency stop'
status: To Do
assignee: []
created_date: '2026-10-20 21:09'
labels: []
milestone: m-1
dependencies:
  - AA-9
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 10000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Serial bus servos hold no absolute calibration reference the software can trust across power cycles, and an unattended system that cannot reliably reach a known state cannot be restarted autonomously after a fault. The system also needs one unambiguous way to stop everything: a hung policy, a jammed insertion or a bad trajectory must be stoppable without physically unplugging the dock.

Homing must be gentle: the arms may start in the folded pose, gripping the tube, or in an unknown pose, so the homing routine must reason about the current state before moving and must not yank against a locked gripper.

See doc-1 sections 3, 8 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A homing routine brings each arm from an arbitrary safe starting pose to the named neutral pose using velocity- and current-limited motion, and refuses to move if the measured state is inconsistent with its assumptions
- [ ] #2 Homing explicitly handles the case where the gripper is in the locked band and still engaged with an attachment point, by unlocking first or aborting with a clear reason rather than pulling against the latch
- [ ] #3 Torque enable/disable is centrally managed so arms can be safely de-energized and re-energized without losing the known pose, and a power-cycle recovery path is documented
- [ ] #4 An emergency stop entry point halts all commanded motion on both arms and the DexArm, turns the laser relay off, and leaves the system in a documented, recoverable state
- [ ] #5 Emergency stop is callable programmatically, from the command line and from a signal handler (SIGINT/SIGTERM), and is exercised by a test against the mock backend
- [ ] #6 Overcurrent, overtemperature and unreachable-servo conditions trigger the same stop path and log a typed fault event
<!-- AC:END -->
