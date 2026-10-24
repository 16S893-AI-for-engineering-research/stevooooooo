---
id: AA-23
title: Cross-arm teleoperated demonstration capture with tube fixture
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:12'
updated_date: '2026-10-24 18:49'
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
- [x] #2 Joint 6 (shared jaw/lock servo) is excluded from the teleop mapping and the recorded action space at the code level, and lock and unlock remain under the AA-17 state machine, verified by a test
- [x] #3 Recording refuses to start unless the tube is supported by the fixture or by a locked, torque-enabled arm; a torque-off arm is never permitted to be the tube holder, verified by a test
- [ ] #4 A tube fixture is specified (design file or documented build) and validated by comparing wrist-camera frames and follower joint poses at the pre-insertion pose in the fixture against the real crawl configuration, with the measured difference recorded
- [ ] #5 Leader backdrive effort and teleop tracking latency are measured and recorded, with a stated judgement of whether demo smoothness is acceptable
- [x] #6 Each episode records synchronized wrist-camera frames, follower joint state, leader joint state, timestamps and the starting pre-insertion pose, in a LeRobot-compatible dataset stored locally only (never pushed to the Hub)
- [x] #7 Episodes always begin from the AA-18 pre-insertion pose definition, with controlled randomization of the starting offset whose range is recorded in the dataset metadata
- [x] #8 Episode success or failure is labeled, and failed or aborted episodes are retained and distinguishable rather than discarded
- [x] #9 Only the active wrist camera streams during capture, enforced via the arbiter
- [x] #10 A dataset inspection tool reports episode count, duration distribution, success rate and starting-offset coverage, and flags under-covered regions of the offset range
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Software implementation plan:
1. Add TeleopCaptureConfig to config.py (Optional, default None) with per-joint sign/offset mirror map, start-offset range, tube-support attestation field.
2. Add teleop_capture block to config/hardware.yaml inside # --- AA-23 --- markers.
3. Implement inchworm/teleop_capture.py: TELEOP_JOINTS (joints 1-5 only; joint 6 excluded), TubeSupportSource enum, precondition_check(), MirrorMapper, LeRobotWriter (local only, lerobot import with fallback), EpisodeRecorder (leader+follower state, wrist frames via arbiter, labels, seed+offset metadata), TeleopSession.
4. scripts/teleop.py: dry-run default, --live --speed-scale flags.
5. scripts/inspect_dataset.py: episode count, duration distribution, success rate, offset coverage histogram.
6. tests/test_teleop_capture.py: mock bus + camera, synthetic dataset, inspector output.
7. robot/docs/tube_fixture.md: fixture spec (owner deliverable, no invented dimensions).
ACs #1, #4, #5 need hardware; left unchecked with commands listed.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
AC#1 (low-speed mapping verification): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2

AC#4 (fixture validation): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --validate-fixture
  See also robot/docs/tube_fixture.md (owner deliverable for dimensions).

AC#5 (backdrive effort and latency): needs owner-supervised hardware session:
  cd robot && uv run python scripts/teleop.py --live --measure-latency

AC#2 checked: 26 tests in tests/test_teleop_capture.py pass; gripper excluded at code level (TELEOP_JOINTS has no gripper key; MirrorMapper and EpisodeRecorder raise GripperInTeleopError if gripper key appears).

AC#3 checked: 9 precondition tests cover all refusal paths.

AC#6 checked: dataset written locally; info.json contains hub_push=False.

AC#7 checked: seed and start_offset_deg in every EpisodeMeta.

AC#8 checked: all three labels (success/failure/aborted) retained and distinguishable.

AC#9 checked: mock arbiter verified called with follower camera role only; leader camera role never acquired.

AC#10 checked: inspector reports episode count, duration distribution, success rate, per-joint offset coverage histogram with under-covered bins flagged. Output pasted in test_teleop_capture.py::test_inspect_dataset_runs.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
Rewritten 2026-10-24 with owner approval (decision-1, plan items A-C). The earlier version was 'Leaderless demonstration capture'.
---
<!-- COMMENTS:END -->
