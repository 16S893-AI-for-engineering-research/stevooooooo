---
id: AA-11
title: SO-101 forward and inverse kinematics with Cartesian jog
status: To Do
assignee: []
created_date: '2026-10-20 21:10'
labels: []
milestone: m-1
dependencies:
  - AA-9
references:
  - CAD/Assembly 1.step
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 11000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Coarse approach, pre-insertion poses, folded poses and policy action framing are all naturally expressed in Cartesian space relative to the chassis, not in joint angles. A verified FK/IK layer with a measured link model is therefore a prerequisite for the crawl stage, and a Cartesian jog tool is the practical way the owner and later tasks will probe reachability of real attachment-point locations.

Derive link lengths and offsets from the CAD assembly and verify them against the physical arm; a kinematic model that disagrees with the hardware by several millimeters will make the learned insertion job much harder.

See doc-1 sections 2, 3 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Forward kinematics maps joint angles to end-effector pose in a documented chassis-fixed frame, with link parameters taken from CAD and recorded in config
- [ ] #2 Inverse kinematics returns joint solutions for reachable targets, reports unreachable targets explicitly, and selects among multiple solutions by a documented deterministic rule that respects joint limits
- [ ] #3 FK/IK round-trip accuracy is measured in simulation (sub-millimeter) and verified against at least five measured physical poses per arm with the residuals recorded in task notes
- [ ] #4 A Cartesian jog CLI moves a chosen arm by a commanded delta or to an absolute pose, honouring joint limits and the collision envelope, with a dry-run mode that prints the plan without moving
- [ ] #5 Joint-space trajectory generation produces time-parameterized, velocity- and acceleration-limited motion rather than step commands
- [ ] #6 Tests cover FK/IK round trips, unreachable-target handling, limit-respecting solution choice and dry-run output
<!-- AC:END -->
