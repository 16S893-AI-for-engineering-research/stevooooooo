---
id: AA-23
title: Leaderless demonstration capture tooling for insertion
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
labels: []
milestone: m-3
dependencies:
  - AA-22
  - AA-18
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 23000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
There are no leader arms, so the usual LeRobot teleoperation workflow is unavailable and demonstrations must be produced another way. Options include keyboard or gamepad Cartesian jogging of the follower, gravity-compensated hand guiding with torque reduced and joint states recorded, or scripted randomized insertions seeded from a known-good pose. The choice materially affects data quality, so it must be evaluated and recorded as a decision rather than assumed.

Every demonstration must start from the same pre-insertion pose definition used at rollout (AA-18), or the policy will be trained outside the distribution it will actually see.

See doc-1 sections 3, 4, 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A demonstration capture mode is implemented and the chosen leaderless method (jog, hand-guide or scripted) is justified in task notes against the alternatives considered
- [ ] #2 Each episode records synchronized wrist-camera frames, full joint state, timestamps and the starting pre-insertion pose, in a LeRobot-compatible dataset format
- [ ] #3 Episodes always begin from the AA-18 pre-insertion pose definition, with controlled randomization of the starting offset whose range is recorded in the dataset metadata
- [ ] #4 Episode success or failure is labeled, and failed or aborted episodes are retained and distinguishable rather than discarded
- [ ] #5 The gripper servo is never commanded by the capture path, and lock and unlock remain under the AA-17 state machine
- [ ] #6 Only the active wrist camera streams during capture, enforced via the arbiter
- [ ] #7 A dataset inspection tool reports episode count, duration distribution, success rate and starting-offset coverage, and flags under-covered regions of the offset range
<!-- AC:END -->
