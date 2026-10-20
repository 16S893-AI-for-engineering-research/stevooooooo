---
id: AA-8
title: SO-101 servo bus driver and per-arm servo ID map
status: To Do
assignee: []
created_date: '2026-10-20 21:09'
labels: []
milestone: m-1
dependencies:
  - AA-5
  - AA-7
references:
  - pictures/robot.png
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 8000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner has assembled both SO-101 arms and assigned servo IDs, but nothing has been commanded in software yet. Every later task (homing, IK, crawl, policy rollout) depends on a trustworthy low-level channel to the Feetech serial-bus servos, and on knowing which ID is which joint on which arm. Getting this wrong means commanding the wrong joint on the wrong arm.

Treat this as the lowest control layer: read and write position, read current/load/temperature/voltage, set torque enable, and nothing about kinematics. The custom gripper shares one servo for jaw and lock (doc-1 section 3), so the gripper servo must be readable at high enough rate to later detect seating and latch hold.

See doc-1 sections 3, 7 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A driver opens either arm bus by logical role and performs ping, read position, read current/load, read temperature/voltage, write goal position and torque enable/disable on an addressed servo
- [ ] #2 A discovery routine scans the bus and reports every responding servo ID with model and current position, and the resulting per-arm joint-to-ID map is persisted in the hardware config rather than hardcoded
- [ ] #3 Driver surfaces checksum errors, timeouts and out-of-range responses as distinct typed errors and never returns a silently stale or partial reading
- [ ] #4 Readback rate sufficient for closed-loop supervision (at least 20 Hz for a full joint state read on one arm) is measured and recorded in the task notes
- [ ] #5 All position values are exposed in degrees at the driver interface with raw-tick conversion isolated in one place
- [ ] #6 A mock servo bus backend supports the full interface so higher layers are testable with no hardware, and tests cover both the mock and the error-mapping paths
<!-- AC:END -->
