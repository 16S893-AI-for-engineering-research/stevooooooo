---
id: AA-19
title: Folded transit pose and fold/unfold sequencing
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
labels: []
milestone: m-2
dependencies:
  - AA-11
  - AA-9
references:
  - pictures/robot.png
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 19000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
After each regrip both arms must return to the folded pose visible in pictures/robot.png. The fold serves two purposes: it brings the part as close as possible to the DexArm print head, and it is the pose from which the system expects repeatable geometry for leveling and printing. Because the folded configuration is tight, the order in which joints move matters, and a naive simultaneous interpolation can collide.

The fold is also the pose whose repeatability the leveling step effectively depends on, so its achieved repeatability needs to be measured rather than assumed.

See doc-1 sections 1, 2 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The folded transit pose for both arms is defined in config, matches the configuration shown in pictures/robot.png, and passes joint-limit and collision validation
- [ ] #2 Fold and unfold sequences execute in a documented joint order with intermediate waypoints that are individually collision-checked, not a single simultaneous interpolation
- [ ] #3 Folding is only permitted when both grippers report locked state, and attempting to fold otherwise is rejected with a clear reason
- [ ] #4 Achieved folded-pose repeatability is measured over at least 10 fold cycles and reported per joint and at the end effector
- [ ] #5 An unfold path returns both arms to a state from which a coarse approach can begin, without passing through the keep-out zones
- [ ] #6 Tests cover sequencing order, the lock precondition rejection and collision validation of all intermediate waypoints
<!-- AC:END -->
