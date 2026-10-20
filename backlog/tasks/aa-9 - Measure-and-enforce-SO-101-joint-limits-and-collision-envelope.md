---
id: AA-9
title: Measure and enforce SO-101 joint limits and collision envelope
status: To Do
assignee: []
created_date: '2026-10-20 21:09'
labels: []
milestone: m-1
dependencies:
  - AA-8
references:
  - pictures/robot.png
  - CAD/Assembly 1.step
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 9000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The arms are mounted on a fixed chassis in a crowded configuration (see pictures/robot.png): they cannot reach everywhere their nominal workspace suggests, and the folded transit pose is tight. Before any trajectory code runs, software must know the real per-joint travel and which poses collide with the chassis, the other arm, the DexArm or the tube. Without this, the first automated motion risks bending hardware.

Limits must be derived from the as-built machine and recorded as data in the hardware config, then enforced at a layer that every motion path goes through, so no later task can bypass them.

See doc-1 sections 2, 3 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Per-joint soft minimum and maximum travel for both arms are recorded in the hardware config with the method and date of measurement noted
- [ ] #2 A validator rejects any commanded joint target outside soft limits before it reaches the servo bus, and the rejection path is unit-tested
- [ ] #3 A documented collision model (at minimum conservative keep-out zones for chassis, opposite arm, DexArm and tube volume) rejects unsafe poses and unsafe straight-line joint interpolations, not just unsafe endpoints
- [ ] #4 Both the folded transit pose and a safe neutral pose are defined, pass validation and are stored as named poses in config
- [ ] #5 Validation is enforced inside the motion layer so that bypassing it requires an explicit documented override flag
- [ ] #6 Tests cover in-limit acceptance, out-of-limit rejection, and a trajectory whose endpoints are legal but whose interpolation passes through a keep-out zone
<!-- AC:END -->
