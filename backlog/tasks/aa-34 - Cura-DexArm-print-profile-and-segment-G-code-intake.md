---
id: AA-34
title: Cura DexArm print profile and segment G-code intake
status: To Do
assignee: []
created_date: '2026-10-20 21:14'
labels: []
milestone: m-6
dependencies:
  - AA-12
  - AA-33
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 34000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Slicing is done by the owner in Cura, and a correct DexArm machine profile has to exist there first. For the agent side, what matters is a defined intake: a sliced segment G-code file must be validated against the DexArm workspace and the current leveled print frame before it is streamed, because a segment sliced for the wrong origin or with absolute Z from a previous hop would crash the nozzle into the part.

This task owns the boundary between externally sliced G-code and the robot, including the request to the owner for a slice when one is needed.

See doc-1 sections 3 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A documented intake accepts a sliced segment G-code file and validates it for dialect compatibility, workspace bounds, feedrate and temperature limits, and expected coordinate mode before any streaming
- [ ] #2 Validation rejects a segment whose coordinate origin or Z reference is inconsistent with the current leveled print frame, naming the specific inconsistency
- [ ] #3 The leveling correction from AA-33 is applied to the segment transform rather than requiring a re-slice per hop, and the applied transform is logged with the segment
- [ ] #4 Streaming to the DexArm reports progress, confirms completion, and aborts safely on firmware error with the nozzle retracted and the laser off
- [ ] #5 Required Cura profile settings for this DexArm and this nozzle and material are documented in the repository so a slice can be reproduced
- [ ] #6 A dry-run mode parses and validates a segment and reports the planned bounding box without commanding motion
- [ ] #7 Tests cover validation acceptance, each rejection reason and dry-run output on sample G-code files
<!-- AC:END -->
