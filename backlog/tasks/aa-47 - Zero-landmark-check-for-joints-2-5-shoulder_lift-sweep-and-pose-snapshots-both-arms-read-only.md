---
id: AA-47
title: >-
  Zero landmark check for joints 2-5, shoulder_lift sweep and pose snapshots
  (both arms, read-only)
status: To Do
assignee: []
created_date: '2026-11-11 23:53'
updated_date: '2026-11-11 23:54'
labels:
  - human-action
  - human-key-e88fb3cd4d5b820a
dependencies: []
references:
  - AA-45
ordinal: 47000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-45

Why only you: The model's joint angles depend on the servo zero offsets. For shoulder_lift, elbow_flex, wrist_flex and wrist_roll these offsets are only assumed: they are the midpoints of the swept ranges and were never checked against a landmark. Evidence that they are wrong: the hand-posed fold is physically fine, yet the model puts the lower arm inside the shoulder, and the shoulder sweep spans 142-153 deg against 200 deg in the URDF. Only someone at the robot can place the real arms in a reference pose. The owner asked for the zeros to be re-measured.

Alternatives considered: Software only can reach nothing more. The existing data are the midpoint zeros, one landmark (pan, pointing at the DexArm) and the hand-posed fold. These give the arms' agreement with each other, not absolute angles. Fitting the zeros to the fold from model contacts is underdetermined and would just encode the same mesh assumptions. The agent does everything else: renders (scripts/view_zero_pose.py), read-only reads (read_pose.py, record_joint_ranges.py, pose_snapshot.py, all refuse with torque on), computing the new offsets, the hardware.yaml/soft-limit updates with owner approval, and re-planning. This supersedes AA-46, which lacked the zero check.

Unlocks: (1) calibration_data/poses_2026-11-11.jsonl has a 'zero_landmark' reading for each arm, with the arm hand-placed in the URDF zero pose shown in out/zero_{left,right}.png. From it the agent computes the zero corrections for shoulder_lift, elbow_flex, wrist_flex and wrist_roll. (2) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json hold a shoulder_lift sweep to both end stops, with a note on what stops the fold side. (3) pose_snapshots.jsonl has a fold snapshot with the owner's y/n on whether the render matches. That y/n is re-checked by the agent after the zero correction.

Estimated active time: 35 minutes

Preparation: Robot powered as in earlier sessions, both arms limp (torque off). No tube, DexArm not needed. Hand within reach of the arm power switch. A phone spirit-level/inclinometer app helps but is optional. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; DexArm MISSING is fine). Open the images out/zero_left.png and out/zero_right.png; each shows that arm at URDF zero from four sides. Model facts from the URDF (not measured): at zero the forearm (elbow to wrist) is horizontal and points away from the DexArm end, the wrist and gripper continue straight in line with the forearm, and the upper arm leans about 14 deg off vertical. Nothing in this procedure moves a servo or writes to it.

## Procedure

1. Open out/zero_left.png. By hand, place the LEFT arm in that pose: pan pointing at the DexArm as before, upper arm as in the picture, forearm horizontal (check with the phone level on the forearm if you have one), wrist and gripper straight in line with the forearm, gripper jaws oriented as in the picture. Hold or prop it there.
   - Expected: The real left arm looks like the render from all four views.
   - Verify: Compare the arm with each of the four views in out/zero_left.png. If you used a level, the forearm reads within about 2 deg of horizontal.
   - Stop if: Stop if the pose cannot be reached without force, or if the render shows a joint bent the opposite way to what is physically possible. Describe what you see instead.

2. While the left arm is held in that pose, run: uv run python scripts/read_pose.py --live --arms left --label zero_landmark --out calibration_data/poses_2026-11-11.jsonl
   - Expected: It prints the left arm's ticks and joint angles and saves one line. Joints far from 0 deg are expected: that difference is what we are measuring.
   - Verify: The output says it saved a line to calibration_data/poses_2026-11-11.jsonl with label zero_landmark.
   - Stop if: Stop if it prints REFUSED (torque on) or a read error.

3. Repeat steps 1-2 for the RIGHT arm with out/zero_right.png and --arms right.
   - Expected: Same as for the left arm, with a second zero_landmark line.
   - Verify: Two zero_landmark lines exist (left, right).
   - Stop if: Same as steps 1-2.

4. Run: uv run python scripts/record_joint_ranges.py --arm left --live --max-seconds 90 --out calibration_data/joint_ranges_left_lift_2026-11-11.json. While it records, move ONLY the left shoulder_lift by hand: slowly to the fold-side end as far as it goes without forcing (3 times), then once to the other end without forcing.
   - Expected: It prints RECORDING and status lines, then a JSON result after 90 s and 'saved ...json'.
   - Verify: The JSON has shoulder_lift lo and hi values and wrapped is false.
   - Stop if: Stop if REFUSED, any joint feels powered, or reaching an end needs force or makes something creak. Note where it stopped.

5. Note what stopped the left shoulder on the fold side: a hard stop in the joint, the lower arm touching the shoulder bracket, a cable, or the chassis.
   - Expected: One short note.
   - Verify: The note is given to the orchestrator with the results.
   - Stop if: If unsure, describe what you saw.

6. Repeat steps 4-5 for the right arm: uv run python scripts/record_joint_ranges.py --arm right --live --max-seconds 90 --out calibration_data/joint_ranges_right_lift_2026-11-11.json
   - Expected: Same as the left arm, saved to the right-arm file, plus a note.
   - Verify: The JSON has shoulder_lift lo/hi, wrapped false, and there is a note.
   - Stop if: Same as step 4.

7. Put both arms by hand into the usual fold/rest pose, then run: uv run python scripts/pose_snapshot.py --label fold --live. Open the PNG it saves (out/pose_snapshots/) and compare it with the real arms.
   - Expected: It prints both arms' angles, saves a PNG and asks whether the render matches.
   - Verify: Answer y, or n plus what differs (expected: may be off, since the zeros are not yet corrected). It logs to calibration_data/pose_snapshots.jsonl.
   - Stop if: Stop if REFUSED.

8. Leave the arms limp and tell the orchestrator you are done, with the notes from steps 5 and 6, and anything that was hard to match in steps 1-3.
   - Expected: Files present: poses_2026-11-11.jsonl (2 zero_landmark lines), two joint_ranges_*_lift files, one fold snapshot line.
   - Verify: The orchestrator checks the files, computes the zero corrections and re-renders the fold for comparison.
   - Stop if: If a file is missing, report which step failed.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Zero landmark check for joints 2-5, shoulder_lift sweep and pose snapshots (both arms, read-only)",
  "parent": "AA-45",
  "reason": "The model's joint angles depend on the servo zero offsets. For shoulder_lift, elbow_flex, wrist_flex and wrist_roll these offsets are only assumed: they are the midpoints of the swept ranges and were never checked against a landmark. Evidence that they are wrong: the hand-posed fold is physically fine, yet the model puts the lower arm inside the shoulder, and the shoulder sweep spans 142-153 deg against 200 deg in the URDF. Only someone at the robot can place the real arms in a reference pose. The owner asked for the zeros to be re-measured.",
  "alternatives": "Software only can reach nothing more. The existing data are the midpoint zeros, one landmark (pan, pointing at the DexArm) and the hand-posed fold. These give the arms' agreement with each other, not absolute angles. Fitting the zeros to the fold from model contacts is underdetermined and would just encode the same mesh assumptions. The agent does everything else: renders (scripts/view_zero_pose.py), read-only reads (read_pose.py, record_joint_ranges.py, pose_snapshot.py, all refuse with torque on), computing the new offsets, the hardware.yaml/soft-limit updates with owner approval, and re-planning. This supersedes AA-46, which lacked the zero check.",
  "outcome": "(1) calibration_data/poses_2026-11-11.jsonl has a 'zero_landmark' reading for each arm, with the arm hand-placed in the URDF zero pose shown in out/zero_{left,right}.png. From it the agent computes the zero corrections for shoulder_lift, elbow_flex, wrist_flex and wrist_roll. (2) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json hold a shoulder_lift sweep to both end stops, with a note on what stops the fold side. (3) pose_snapshots.jsonl has a fold snapshot with the owner's y/n on whether the render matches. That y/n is re-checked by the agent after the zero correction.",
  "preparation": "Robot powered as in earlier sessions, both arms limp (torque off). No tube, DexArm not needed. Hand within reach of the arm power switch. A phone spirit-level/inclinometer app helps but is optional. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; DexArm MISSING is fine). Open the images out/zero_left.png and out/zero_right.png; each shows that arm at URDF zero from four sides. Model facts from the URDF (not measured): at zero the forearm (elbow to wrist) is horizontal and points away from the DexArm end, the wrist and gripper continue straight in line with the forearm, and the upper arm leans about 14 deg off vertical. Nothing in this procedure moves a servo or writes to it.",
  "category": "physical-access",
  "minutes": 35,
  "steps": [
    {
      "action": "Open out/zero_left.png. By hand, place the LEFT arm in that pose: pan pointing at the DexArm as before, upper arm as in the picture, forearm horizontal (check with the phone level on the forearm if you have one), wrist and gripper straight in line with the forearm, gripper jaws oriented as in the picture. Hold or prop it there.",
      "expected": "The real left arm looks like the render from all four views.",
      "verification": "Compare the arm with each of the four views in out/zero_left.png. If you used a level, the forearm reads within about 2 deg of horizontal.",
      "stop": "Stop if the pose cannot be reached without force, or if the render shows a joint bent the opposite way to what is physically possible. Describe what you see instead."
    },
    {
      "action": "While the left arm is held in that pose, run: uv run python scripts/read_pose.py --live --arms left --label zero_landmark --out calibration_data/poses_2026-11-11.jsonl",
      "expected": "It prints the left arm's ticks and joint angles and saves one line. Joints far from 0 deg are expected: that difference is what we are measuring.",
      "verification": "The output says it saved a line to calibration_data/poses_2026-11-11.jsonl with label zero_landmark.",
      "stop": "Stop if it prints REFUSED (torque on) or a read error."
    },
    {
      "action": "Repeat steps 1-2 for the RIGHT arm with out/zero_right.png and --arms right.",
      "expected": "Same as for the left arm, with a second zero_landmark line.",
      "verification": "Two zero_landmark lines exist (left, right).",
      "stop": "Same as steps 1-2."
    },
    {
      "action": "Run: uv run python scripts/record_joint_ranges.py --arm left --live --max-seconds 90 --out calibration_data/joint_ranges_left_lift_2026-11-11.json. While it records, move ONLY the left shoulder_lift by hand: slowly to the fold-side end as far as it goes without forcing (3 times), then once to the other end without forcing.",
      "expected": "It prints RECORDING and status lines, then a JSON result after 90 s and 'saved ...json'.",
      "verification": "The JSON has shoulder_lift lo and hi values and wrapped is false.",
      "stop": "Stop if REFUSED, any joint feels powered, or reaching an end needs force or makes something creak. Note where it stopped."
    },
    {
      "action": "Note what stopped the left shoulder on the fold side: a hard stop in the joint, the lower arm touching the shoulder bracket, a cable, or the chassis.",
      "expected": "One short note.",
      "verification": "The note is given to the orchestrator with the results.",
      "stop": "If unsure, describe what you saw."
    },
    {
      "action": "Repeat steps 4-5 for the right arm: uv run python scripts/record_joint_ranges.py --arm right --live --max-seconds 90 --out calibration_data/joint_ranges_right_lift_2026-11-11.json",
      "expected": "Same as the left arm, saved to the right-arm file, plus a note.",
      "verification": "The JSON has shoulder_lift lo/hi, wrapped false, and there is a note.",
      "stop": "Same as step 4."
    },
    {
      "action": "Put both arms by hand into the usual fold/rest pose, then run: uv run python scripts/pose_snapshot.py --label fold --live. Open the PNG it saves (out/pose_snapshots/) and compare it with the real arms.",
      "expected": "It prints both arms' angles, saves a PNG and asks whether the render matches.",
      "verification": "Answer y, or n plus what differs (expected: may be off, since the zeros are not yet corrected). It logs to calibration_data/pose_snapshots.jsonl.",
      "stop": "Stop if REFUSED."
    },
    {
      "action": "Leave the arms limp and tell the orchestrator you are done, with the notes from steps 5 and 6, and anything that was hard to match in steps 1-3.",
      "expected": "Files present: poses_2026-11-11.jsonl (2 zero_landmark lines), two joint_ranges_*_lift files, one fold snapshot line.",
      "verification": "The orchestrator checks the files, computes the zero corrections and re-renders the fold for comparison.",
      "stop": "If a file is missing, report which step failed."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 (1) calibration_data/poses_2026-11-11.jsonl has a 'zero_landmark' reading for each arm, with the arm hand-placed in the URDF zero pose shown in out/zero_{left,right}.png. From it the agent computes the zero corrections for shoulder_lift, elbow_flex, wrist_flex and wrist_roll. (2) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json hold a shoulder_lift sweep to both end stops, with a note on what stops the fold side. (3) pose_snapshots.jsonl has a fold snapshot with the owner's y/n on whether the render matches. That y/n is re-checked by the agent after the zero correction.
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
<!-- human-handoff-state:v1 -->
```json
{
  "version": 1,
  "key": "e88fb3cd4d5b820ab5de0843189476004e23751a0995a903aa739672dccda318",
  "phase": "proposed",
  "createdAt": "2026-11-11T23:54:00.365Z",
  "origin": "01a1cb2b-5f23-70da-906d-dcbf45d889dd",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
