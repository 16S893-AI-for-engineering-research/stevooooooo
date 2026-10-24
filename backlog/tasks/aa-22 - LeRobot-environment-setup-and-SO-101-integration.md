---
id: AA-22
title: LeRobot environment setup and SO-101 integration
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
updated_date: '2026-10-24 16:07'
labels: []
milestone: m-3
dependencies:
  - AA-8
  - AA-14
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 22000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The insertion policy will be a LeRobot model, but LeRobot is not installed on this host and the available GPU is a 4 GB RTX 3050 Laptop with 7 GB of system RAM. Establishing a working, pinned environment and confirming that LeRobot can address this specific SO-101 build (already-assigned servo IDs, custom single-servo gripper, mono OV9281 cameras, single-camera USB limit) is a prerequisite for any data collection, and is where version and VRAM surprises should surface rather than mid-training.

Integrate through the existing drivers and arbiter rather than letting LeRobot open devices directly, so the single-camera invariant and gripper protection are preserved.

See doc-1 sections 6, 7 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 LeRobot is installed in a reproducible pinned environment under the project and a smoke test imports it and reports the resolved versions and CUDA availability
- [ ] #2 A robot adapter exposes this build to LeRobot using the project drivers, with the correct joint-to-servo map, mono single-channel camera observations and the chassis-fixed frame convention
- [ ] #3 Camera access from the LeRobot path goes through the USB arbiter so only one camera streams at a time, verified by a test
- [ ] #4 The gripper servo is excluded from the LeRobot action space at the adapter level, not merely by convention
- [ ] #5 Observation and action space definitions match the handoff contract from AA-20, and a mismatch raises an error at construction time
- [ ] #6 A VRAM and memory feasibility note records measured usage for a minimal forward and backward pass at the intended image resolution on the 4 GB GPU, with recommended resolution and batch size
- [ ] #7 Documentation in robot/README.md explains how to recreate the environment from scratch
- [ ] #8 LeRobot runs in a separate uv-managed Python >= 3.12 environment, since LeRobot 0.6.2 requires 3.12 and the host default is 3.10, and the robot package remains importable from it
- [ ] #9 Camera observations for each arm are resolved through AA-5 discovery (usbipd BusId), never by OpenCV index or by-id, because both arm cameras share USB serial 5423534724
<!-- AC:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
ACs #8 and #9 added 2026-10-24 with owner approval (plan item D). The adapter's 5-joint gripper-free action space is already covered by AC #4.
---
<!-- COMMENTS:END -->
