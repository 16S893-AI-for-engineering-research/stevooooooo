---
id: AA-39
title: Choose per-arm or shared insertion policy for the mirrored arms
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-24 16:07'
updated_date: '2026-11-20 13:20'
labels: []
milestone: m-3
dependencies:
  - AA-25
type: task
ordinal: 39000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The left and right SO-101 arms insert from mirrored sides of the tube. The owner approved starting with one arm's policy (AA-23/AA-25) and adding the other later. Afterwards a choice is needed: train a second, separate policy, or one shared policy on mirrored data with an arm-identity input. That choice drives the owner's demonstration burden, since a second policy roughly doubles it.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Once the first arm's policy passes AA-25, a written comparison of a separate second policy versus a shared mirrored policy states the expected demo count, the risk and the recommendation
- [ ] #2 The owner's choice is recorded as a decision record, and follow-up capture and training tasks for the second arm are created to match
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

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Owner 2026-11-20: try the LEFT ACT policy on the RIGHT arm before any retraining; merge replay_cycle + policy_handoff into one launcher.
Facts: doc-1 says the arms are mounted the same way (not mirrored) and all joint signs are +1 on both arms. Timeline: the right pre-insertion pose (frame 67) = the left one (frame 42) = [0,-21.8,9.0,-75.5,0] deg, and inserted 73 = 48. The policy works in raw servo degrees, so the per-arm zero offsets differ (pan 173.320 L vs 181.318 R, lift 214.893 vs 220.518, elbow 182.900 vs 179.033, wrist_flex 178.286 vs 175.254, roll 175.687 vs 175.115).
1. policy_handoff --follower right: observation right raw deg -> left-equivalent (deg - zero_R + zero_L, per joint, sign +1 both, asserted from hardware.yaml; refuse if any sign differs), policy, action back (deg - zero_L + zero_R). Plugin follower=right (handoff gate, mesh, limits, return already arm-generic; check that return/handoff_return work with follower=right, the other arm = left, LOCKED). Tests: round-trip mapping, sign refusal, right-arm sim attempt.
2. replay_cycle --stop-after-frame 67 --tube-in-grippers: check that it accepts frame 67 (right open, left LOCKED) and writes handoff_state for the right arm; dry-run.
3. ONE launcher (owner): robot/lerobot_env/policy_test.sh --arm left|right [--dry-run]: runs replay_cycle to the arm's stop frame (42 left / 67 right) and then the policy session in the same terminal. Decide the mechanism: subprocess call of scripts/replay_cycle.py in the main env, then policy_handoff (keeps each env's deps), or import inside lerobot_env (it installs inchworm-robot[model]). The YES prompts of both stay. If the replay fails, the policy step never starts.
4. Dry runs: both arms (simulated servos + real cell mesh). The right arm sees the camera frames of a left episode in its dry run (a mapping check only).
5. Owner-supervised first live right-arm trial with the same guards (5 deg/s, 10 deg jump, 20 s). Report the result; then the AA-39 comparison (AC1) and the owner's decision (AC2).
Risks: the right wrist camera sees a different scene (the left arm holding, background, lighting); real key offsets; camera mount differences. If it fails: 10-20 right demos (fit leader_map_right first), then fine-tune the existing checkpoint on left+right.
<!-- SECTION:PLAN:END -->
