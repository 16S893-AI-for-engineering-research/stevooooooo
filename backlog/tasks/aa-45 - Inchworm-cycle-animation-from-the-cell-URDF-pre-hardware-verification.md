---
id: AA-45
title: Inchworm cycle animation from the cell URDF (pre-hardware verification)
status: To Do
assignee:
  - '@agent'
created_date: '2026-11-10 20:28'
labels:
  - simulation
  - verification
milestone: m-2
dependencies: []
priority: high
type: feature
ordinal: 45000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner cannot trust the agents' text descriptions of physical motion; rendered images from the cell URDF are what confirm behaviour (devlog 2026-11-10). Before any live crawl, the owner wants to watch the proposed inchworm motion animated on the real cell model (CAD/robot_urdf_new). Today only the approach phase produces joint trajectories; the advance is bookkeeping only (FrameModel.complete_advance) and no code describes how the arms physically carry the tube. Owner-specified cycle (2026-11-10): start with both arms locked in the printing pose; both arms (locked) reposition the tube along its axis away from the DexArm until the next attachment point is reachable by the left arm; the left arm (nearest the DexArm) unlocks and regrips the attachment point one pitch closer to the DexArm; the right arm then unlocks and regrips the point the left arm just left; extra both-arm repositions are allowed between regrips if a target is out of reach; only after both regrips do both arms fold back to the printing pose (net tube advance one pitch, 183.676 mm, away from the DexArm). Insertion is assumed perfect (straight-line pre-insertion to seated), standing in for the learned policy. No printing or laser. The printing pose is the hardware.yaml placeholder until the owner re-provides it. Single side view: DexArm, left arm, right arm left to right, camera on the +x (cable) side; owner confirms the camera side from one test frame.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A planner produces a timeline (JSONL) of both arms' joint angles, gripper lock states and tube pose for the owner-specified cycle, using the existing approach planner, frame model, joint limits and mesh collision guard
- [ ] #2 During every both-arms-locked segment, the two grippers keep a constant relative pose along the tube (rigid-tube constraint), checked per frame within a stated tolerance
- [ ] #3 Every frame is checked for mesh collision, joint limits and the gripper invariants (never both unlocked with the tube attached, at most one unlocked); any violation fails the run with the frame and reason, and tests show a colliding frame and a both-unlocked frame each fail
- [ ] #4 Any extra reposition the planner inserts is logged with its reason; if an owner-specified regrip target does not exist on the modelled tube the run stops and reports instead of inventing one
- [ ] #5 An MP4 or GIF renders the cycle in one side view (DexArm, left, right from left to right, +x cable side) with the step name, per-gripper lock state and tube advance overlaid, and contacting parts drawn red; the placeholder printing pose is labelled as such
- [ ] #6 The owner has watched the animation and confirmed the motion matches the intended inchworm cycle
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 Acceptance criteria are each verified by concrete evidence (test output, measurement, or a logged hardware run) and not by code inspection or intent
- [ ] #2 Acceptance criteria and task description were NOT altered to match the implementation; any scope or criterion change was approved by the owner first and is recorded in the task
- [ ] #3 Hardware facts used came from doc-1; any corrected measurement was written back to doc-1 via `backlog doc update doc-1` in the same task
- [ ] #4 Tests pass via `uv run pytest` from robot/, including mock-backend tests that run with no hardware attached
- [ ] #5 Safety invariants still hold and are covered by a test: single-camera USB exclusivity, always-one-arm-holding, gripper servo commanded only by the AA-17 state machine, laser off on every exit path
- [ ] #6 Implementation notes record measured values, deviations from plan, and anything deferred
<!-- DOD:END -->
