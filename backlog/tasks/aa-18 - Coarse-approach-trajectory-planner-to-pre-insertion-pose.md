---
id: AA-18
title: Coarse approach trajectory planner to pre-insertion pose
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
labels: []
milestone: m-2
dependencies:
  - AA-15
  - AA-16
  - AA-9
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 18000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The learned policy is responsible only for final insertion; code must deliver the gripper to a consistent pre-insertion pose near the target attachment point, repeatably and without collisions, in a workspace where the arms cannot move freely. The approach must also respect that the opposite arm is simultaneously holding the tube, so the moving arm has to route around the tube and its partner.

A consistent pre-insertion pose matters beyond safety: the policy will be trained from that pose, so variation here directly becomes variation the policy must absorb. The handoff pose definition belongs to this task.

See doc-1 sections 2, 3 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A planner produces a collision-free, velocity- and acceleration-limited trajectory from the current pose to a pre-insertion pose for a given attachment-point lattice index
- [ ] #2 The pre-insertion pose is defined relative to the target attachment point with documented standoff and orientation, and is the single definition later used for policy training and rollout
- [ ] #3 Planning accounts for the opposite arm currently holding the tube and for the tube volume itself, and rejects targets whose approach cannot be routed safely
- [ ] #4 Repeatability of the achieved pre-insertion pose is measured over at least 10 approaches and reported per axis, since this variation bounds the required policy capture range
- [ ] #5 A dry-run mode prints and optionally visualizes the planned trajectory without commanding motion
- [ ] #6 Failure to reach the pre-insertion pose within tolerance aborts with a typed error and leaves the arm in a safe, recoverable pose rather than mid-trajectory
- [ ] #7 Tests cover planning success, unreachable and unsafe rejections, limit compliance and dry-run output
<!-- AC:END -->
