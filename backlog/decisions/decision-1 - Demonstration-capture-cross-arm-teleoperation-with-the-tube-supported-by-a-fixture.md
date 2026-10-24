---
id: decision-1
title: >-
  Demonstration capture: cross-arm teleoperation with the tube supported by a
  fixture
date: '2026-10-24 16:06'
status: accepted
---
## Context

Owner decision, 2026-10-24 (doc-2 s4). doc-1 previously said "No leader arms exist ... all demonstration data must be collected without a leader". The owner pointed out that each SO-101 follower can act as the leader for the other.

Verified facts, from LeRobot 0.6.2 source (`src/lerobot/teleoperators/so_leader/so_leader.py`, main as of 2026-10-23, and `docs/source/so101.mdx`):
- `so101_leader` only does `sync_read("Present_Position")` with torque disabled, on the same 6 STS3215 motor IDs a follower uses. Nothing in it requires leader-specific hardware.
- A purpose-built leader uses lighter gearing (pan 1/191, shoulder-lift 1/345, elbow 1/191, wrist-flex, wrist-roll and gripper 1/147). A follower uses 1/345 on all six joints, so it is stiffer to backdrive by hand.
- Joint 6 on this robot is the shared jaw/lock servo, so a stock leader-to-follower mapping would command the lock. That is forbidden (AGENTS.md invariant 4).

Owner constraint: both arms hold the tube at the start of a cycle and whenever printing. A torque-off leader cannot hold the tube, so cross-arm teleop cannot happen in the live crawl pose. Options considered: (a) cross-arm with a fixture, (b) hand-guide the inserting arm while the other stays locked, (c) keyboard or gamepad jog, (d) buy or build a dedicated leader. The owner chose (a).

## Decision

Demonstrations are captured by **cross-arm teleoperation**: one SO-101, torque off, is the leader and the other is the follower, which performs the insertion. During recording, the tube (or a representative attachment-point section) is held by a **fixture**, never by the leader arm.

Rules:
1. The leader arm never carries the tube while its torque is off. Recording refuses to start unless the tube is supported by the fixture or by a locked, torqued arm.
2. The gripper (joint 6) is excluded from the teleop mapping and from the action space. Lock and unlock remain AA-17 state-machine commands only.
3. Joint mapping between the arms handles mirrored mounting (per-joint sign and offset), and it is verified at low speed under owner supervision before any recording.
4. The fixture must reproduce the real attachment-point geometry and pose relative to the inserting arm and its wrist camera closely enough that rollout observations match training. This is checked by comparing camera frames from the fixture and from the real crawl pose.

## Consequences

- AA-23 is rewritten for cross-arm capture with a fixture. A fixture design and validation step is added, and the fixture is a physical owner deliverable.
- Distribution shift between the fixture and the real crawl is the main risk. If AA-25 online success in the real crawl lags fixture success, fall back to option (b) or (c) for HIL correction rounds in the real pose.
- Follower backdrive stiffness may degrade demo smoothness. AA-23 measures it.
- Both arms need demonstrations eventually (mirrored sides). Start with one arm (owner-approved item H), then add the other.

