---
id: AA-23
title: Cross-arm teleoperated demonstration capture with tube fixture
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
updated_date: '2026-10-24 16:07'
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
The insertion policy is learned from human demonstrations, and the owner will record many of them. Per decision-1, demonstrations use cross-arm teleoperation. One SO-101 follower, with torque off, is moved by hand as the leader, and the other follower mirrors it and performs the insertion. LeRobot's so101_leader only reads Present_Position with torque disabled, so a follower can serve as a leader. But followers are geared 1/345 on every joint, unlike a purpose-built leader, so backdrive effort must be measured.

Both arms normally hold the tube (doc-1 s2), and a torque-off leader cannot hold anything. So during recording the tube, or a representative attachment-point section, is supported by a fixture. The fixture must reproduce the real attachment-point pose relative to the inserting arm and its wrist camera, or the policy will be trained outside the distribution it sees at rollout. Joint 6 is the shared jaw/lock servo and must never be driven by teleop.

Every demonstration must start from the same pre-insertion pose definition used at rollout (AA-18). Start with one arm, then add the other (owner-approved).

See decision-1 and doc-1 sections 2, 3, 4, 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A cross-arm teleoperation mode maps the torque-off leader arm's joints 1-5 to the follower's, with per-joint sign and offset handling for mirrored mounting, and the mapping is verified at low speed under owner supervision with the result logged
- [ ] #2 Joint 6 (shared jaw/lock servo) is excluded from the teleop mapping and the recorded action space at the code level, and lock and unlock remain under the AA-17 state machine, verified by a test
- [ ] #3 Recording refuses to start unless the tube is supported by the fixture or by a locked, torque-enabled arm; a torque-off arm is never permitted to be the tube holder, verified by a test
- [ ] #4 A tube fixture is specified (design file or documented build) and validated by comparing wrist-camera frames and follower joint poses at the pre-insertion pose in the fixture against the real crawl configuration, with the measured difference recorded
- [ ] #5 Leader backdrive effort and teleop tracking latency are measured and recorded, with a stated judgement of whether demo smoothness is acceptable
- [ ] #6 Each episode records synchronized wrist-camera frames, follower joint state, leader joint state, timestamps and the starting pre-insertion pose, in a LeRobot-compatible dataset stored locally only (never pushed to the Hub)
- [ ] #7 Episodes always begin from the AA-18 pre-insertion pose definition, with controlled randomization of the starting offset whose range is recorded in the dataset metadata
- [ ] #8 Episode success or failure is labeled, and failed or aborted episodes are retained and distinguishable rather than discarded
- [ ] #9 Only the active wrist camera streams during capture, enforced via the arbiter
- [ ] #10 A dataset inspection tool reports episode count, duration distribution, success rate and starting-offset coverage, and flags under-covered regions of the offset range
<!-- AC:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
Rewritten 2026-10-24 with owner approval (decision-1, plan items A-C). The earlier version was 'Leaderless demonstration capture'.
---
<!-- COMMENTS:END -->
