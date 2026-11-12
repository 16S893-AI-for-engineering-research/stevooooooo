---
id: AA-47
title: >-
  Zero landmark check for joints 2-5, shoulder_lift sweep and pose snapshots
  (both arms, read-only)
status: Done
assignee:
  - '@human'
created_date: '2026-11-11 23:53'
updated_date: '2026-11-12 00:36'
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
- [x] #1 (1) calibration_data/poses_2026-11-11.jsonl has a 'zero_landmark' reading for each arm, with the arm hand-placed in the URDF zero pose shown in out/zero_{left,right}.png. From it the agent computes the zero corrections for shoulder_lift, elbow_flex, wrist_flex and wrist_roll. (2) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json hold a shoulder_lift sweep to both end stops, with a note on what stops the fold side. (3) pose_snapshots.jsonl has a fold snapshot with the owner's y/n on whether the render matches. That y/n is re-checked by the agent after the zero correction.
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
Follow the user-accepted procedure one step at a time, record evidence and stop on deviations. Submit the outcome for orchestrator verification; do not complete the originating project task.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
<!-- human-handoff-state:v1 -->
```json
{
  "version": 1,
  "key": "e88fb3cd4d5b820ab5de0843189476004e23751a0995a903aa739672dccda318",
  "phase": "verified",
  "createdAt": "2026-11-11T23:54:00.365Z",
  "origin": "01a1cb2b-5f23-70da-906d-dcbf45d889dd",
  "step": 8,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 done (self-reported). The owner reports the left arm hand-placed in the URDF zero pose from out/zero_left.png, \"matched by eye\". No level was used (it is optional), so forearm horizontality is judged by eye only, with no measured angle; the ±2° criterion is unverified. The agent could not independently see the four render views clearly or the real arm; there is no photo. The owner did not report any joint needing force or bending the wrong way. Arm is held in the pose for the step 2 reading.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation: \"matched by eye\"",
      "advanced": true,
      "at": "2026-11-12T00:03:08.776Z"
    },
    {
      "step": 2,
      "note": "Step 2 done. The agent read calibration_data/poses_2026-11-11.jsonl: one line, arm left, label zero_landmark, utc 2026-11-12T00:03:52Z. ticks [1968, 2445, 2081, 2044, 2034, 2516]. q_deg pan -0.3, shoulder_lift 27.9, elbow_flex 5.8, wrist_flex 1.4, wrist_roll -2.4. gripper_ticks 2516. soft_limit_violations is empty. No REFUSED or read error. Observation for the orchestrator: shoulder_lift is the large outlier at 27.9 deg, about 2x the 14 deg upper-arm lean the URDF zero pose specifies. It may be a real zero-offset error or how the upper arm was placed. The pose was matched by eye only (step 1), so the agent does not interpret it further. Elbow, wrist and roll are within about 6 deg. The pan reading of -0.3 deg agrees with the existing pan landmark.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/poses_2026-11-11.jsonl (1 line, left zero_landmark)",
      "advanced": true,
      "at": "2026-11-12T00:04:38.480Z"
    },
    {
      "step": 3,
      "note": "Step 3 NOT verified: no right-arm reading was saved. The agent read calibration_data/poses_2026-11-11.jsonl, which now has 2 lines, BOTH arm \"left\". New line 2: label zero_landmark, utc 2026-11-12T00:06:52Z, ticks [2651, 1255, 3116, 2179, 2034, 2516], q_deg pan 59.7, shoulder_lift -76.7, elbow_flex 96.8, wrist_flex 13.2, wrist_roll -2.4. It is very likely the LEFT arm, read again in its released or resting pose, not the zero pose. Evidence: the arm field is left, and the wrist_roll (2034) and gripper (2516) ticks are identical to the line-1 left reading. The likely cause is the command being run with --arms left instead of --arms right (unconfirmed). FOR THE ORCHESTRATOR: line 2 is NOT a valid zero landmark and must be excluded. Only line 1 (00:03:52Z) is the real left zero_landmark. The agent cannot edit files, and the owner was not asked to hand-edit. Correction to the step-2 note: the 27.9 deg shoulder_lift reading should not be compared with the URDF's 14 deg upper-arm lean. The 14 deg is the arm's geometric angle at zero; the 27.9 deg is the deviation from zero. Not advanced. The owner was asked to re-pose the right arm and re-run with --arms right.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/poses_2026-11-11.jsonl (2 lines, both arm=left)",
      "advanced": false,
      "at": "2026-11-12T00:07:29.724Z"
    },
    {
      "step": 3,
      "note": "Step 3 done. The agent read calibration_data/poses_2026-11-11.jsonl, which now has 3 lines. Line 3 is new: arm right, label zero_landmark, utc 2026-11-12T00:09:53Z, ticks [2083, 2509, 2037, 2006, 2025, 2493], q_deg pan 1.8, shoulder_lift 33.4, elbow_flex 5.4, wrist_flex 1.1, wrist_roll -2.6. gripper_ticks 2493. soft_limit_violations is empty. No REFUSED or read error. The owner's terminal output matches the file. An earlier attempt failed with an argparse error ('--out expected one argument', command truncated); nothing was read or saved by it. Valid zero landmarks: line 1 (left, 00:03:52Z) and line 3 (right, 00:09:53Z). Line 2 (left, 00:06:52Z) is spurious and must be excluded, as already noted. The right-arm pose match was not explicitly confirmed by the owner (by eye or level): self-report is missing, no photo. The values are consistent with a deliberate zero pose: pan, elbow, wrist and roll are all within ~6 deg of 0. Observation for the orchestrator, not interpreted further: both arms show shoulder_lift well off zero in a similar direction (left 27.9, right 33.4 deg), while the other joints are near 0.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/poses_2026-11-11.jsonl (3 lines); owner-pasted read_pose.py terminal output",
      "advanced": true,
      "at": "2026-11-12T00:10:25.262Z"
    },
    {
      "step": 4,
      "note": "Step 4 done. The agent read calibration_data/joint_ranges_left_lift_2026-11-11.json (recorded 2026-11-12T00:15:46Z, 487 samples, so it probably ended early via the stop file). This overwrote the AA-46 file; the old values were min 1224 / max 3647. shoulder_lift (servo 2): start 1416, min_ticks 1224 (107.58 servo deg), max_ticks 3651 (320.89 servo deg), span 213.31 deg, wrapped false, crosses_seam false. Verification met: lo and hi are present and wrapped is false. Observation: the fold-side minimum of 1224 is IDENTICAL to the earlier AA-46 sweep (1224) and the max is within 4 ticks (3651 vs 3647). The limit repeated exactly across two separate sweeps, which suggests a repeatable physical stop rather than a random hand stop. That is not confirmed; the cause is still to come from the owner in step 5. Torque is off, so EEPROM limits do not constrain hand motion. DEVIATION: as before, shoulder_pan moved 89.7 deg (1944-2964) and elbow_flex 141.8 deg (1508-3121) during a sweep meant to be shoulder_lift only. Wrist and gripper did not move. No REFUSED was reported.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/joint_ranges_left_lift_2026-11-11.json (recorded 2026-11-12T00:15:46Z)",
      "advanced": true,
      "at": "2026-11-12T00:16:15.654Z"
    },
    {
      "step": 5,
      "note": "Step 5 note received but ambiguous. The owner reports, for the left shoulder_lift sweep: \"bracket on one side and hard stop on the other\". The owner did not say which end was which. The likely reading is fold side (min 1224) = lower arm touching the shoulder bracket, and the other end (max 3651) = hard stop, but this is NOT confirmed and the agent did not assume it. The fold-side cause is the one the outcome needs, so the owner was asked which end touched the bracket. If the fold side is the bracket, the 1224 limit is set by contact between parts, not by the servo's own range; how far the lower arm can fold may then depend on the elbow angle (the elbow moved 142 deg during this sweep). Not advanced pending that clarification.",
      "type": "self-reported",
      "reference": "Owner verbal reply in conversation: \"bracket on one side and hard stop on the other\"",
      "advanced": false,
      "at": "2026-11-12T00:18:07.362Z"
    },
    {
      "step": 5,
      "note": "Step 5 done. The owner clarified the left shoulder_lift stops. Fold side (min_ticks 1224): the lower arm hits the shoulder bracket. Other end (max_ticks 3651): a hard stop. So the fold-side limit is set by parts touching, not by the servo's own range. It may depend on the elbow angle at the moment of contact; the elbow was not held fixed during the sweep (moved 142 deg), and its angle at contact was not recorded. For the orchestrator: since 1224 repeated exactly over two sweeps, the contact point looks repeatable for the elbow position the owner used. Self-reported only, no photo.",
      "type": "self-reported",
      "reference": "Owner verbal reply in conversation: \"fold = bracket: the lower arm hit the shoulder bracket when folded, and the hard stop was at the other end\"",
      "advanced": true,
      "at": "2026-11-12T00:19:31.490Z"
    },
    {
      "step": 6,
      "note": "Step 6 done. The agent read calibration_data/joint_ranges_right_lift_2026-11-11.json (recorded 2026-11-12T00:20:24Z, 451 samples, so it probably ended early via the stop file). This overwrote the AA-46 file; the old values were min 1295 / max 3719. shoulder_lift (servo 2): start 1484, min_ticks 1291 (113.47 servo deg), max_ticks 3717 (326.69 servo deg), span 213.22 deg, wrapped false, crosses_seam false. Verification met: lo/hi present, wrapped false, and a note was given. The owner's note is \"same as other arm\", taken as: fold side (1291) = lower arm hits the shoulder bracket, other end (3717) = hard stop. This is self-reported, with no photo. Repeatability: within 4 ticks of the earlier AA-46 sweep (1295/3719). Both arms' spans match within 0.1 deg (213.3 vs 213.2). For the orchestrator: right 1291 is 8 ticks short of the current right soft limit 1283, and ~199 ticks (~17.5 deg) short of the ~1092 printing-pose target. The left fold contact is 1224. DEVIATION: as before, shoulder_pan moved 86.9 deg (2036-3025) and elbow_flex 139.6 deg (1488-3076); the elbow angle at bracket contact was not recorded. Wrist and gripper did not move. No REFUSED was reported.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/joint_ranges_right_lift_2026-11-11.json (recorded 2026-11-12T00:20:24Z); owner verbal note \"saved and same as other arm\"",
      "advanced": true,
      "at": "2026-11-12T00:20:54.294Z"
    },
    {
      "step": 7,
      "note": "Step 7 done. The agent read calibration_data/pose_snapshots.jsonl, which has 2 fold lines. The new line is time 2026-11-11T19-23-05: the script timestamps in local time, and this matches about 00:23Z, just after step 6. The earlier line, 18-57-01 local (~23:57Z), is from the AA-46 walkthrough and is not part of this task. New line q_deg: left pan 0.53, lift -61.92, elbow 97.21, wrist_flex -90.13, roll -2.68; right pan 0.79, lift -55.81, elbow 96.59, wrist_flex -90.18, roll -2.81. The owner's answer is logged in the file: \"n it does not. the shoulder lift (servo 2 is wrong) everything else looks good\". The model contacts include \"left_lower_arm intersects left_shoulder\" and \"right_lower_arm intersects right_shoulder\", matching the problem described in the task reason. The agent opened the PNG at out/pose_snapshots/fold_2026-11-11T19-23-05.png, but the render-to-real comparison is the owner's: the agent cannot see the real arms. No REFUSED. Expected result met: a fold snapshot with y/n. The owner's mismatch points only at shoulder_lift, consistent with the zero_landmark readings, where shoulder_lift reads +27.9 (left) / +33.4 (right) deg at the URDF zero pose while the other joints are within ~6 deg.",
      "type": "automated",
      "reference": "read of /home/stevotimp/893/robot/calibration_data/pose_snapshots.jsonl (line 2, time 2026-11-11T19-23-05); image read of /home/stevotimp/893/robot/out/pose_snapshots/fold_2026-11-11T19-23-05.png",
      "advanced": true,
      "at": "2026-11-12T00:25:06.821Z"
    },
    {
      "step": 8,
      "note": "Step 8 done (self-reported). The owner confirms both arms are left limp. The agent checked that all expected files are present.\n- poses_2026-11-11.jsonl has 2 valid zero_landmark lines: line 1 left at 00:03:52Z, line 3 right at 00:09:53Z. Line 2 (left, 00:06:52Z) is spurious and must be excluded.\n- joint_ranges_left_lift_2026-11-11.json: 1224-3651.\n- joint_ranges_right_lift_2026-11-11.json: 1291-3717.\n- pose_snapshots.jsonl has the fold line 2026-11-11T19-23-05 with the owner's answer \"n, shoulder lift (servo 2) wrong, everything else good\".\nNotes for the orchestrator: on both arms the fold side is stopped by the bracket and the other end by a hard stop. The zero poses were matched by eye; the right-arm match was not explicitly confirmed. Elbow and pan moved during the sweeps. The AA-46 joint_ranges files were overwritten. Nothing was written to any servo. Open items: AA-46 is superseded and still active; AA-42 is stopped, and BusId 9-3 maps to arm_right. The walkthrough is complete and goes to the orchestrator for verification via /walk finish. This is not completion of AA-45.",
      "type": "self-reported",
      "reference": "Owner verbal confirmation \"arms are limp\"; prior reads of poses_2026-11-11.jsonl, joint_ranges_{left,right}_lift_2026-11-11.json, pose_snapshots.jsonl",
      "advanced": true,
      "at": "2026-11-12T00:28:39.954Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-11-11T23:59:08.129Z",
      "hash": "5decf4e7004dd299f98d8119594f5062c56236c89707205e67da290b37ead13b"
    }
  ],
  "acceptedHash": "5decf4e7004dd299f98d8119594f5062c56236c89707205e67da290b37ead13b",
  "acceptedAt": "2026-11-11T23:59:08.129Z",
  "summary": "good",
  "completedAt": "2026-11-12T00:35:58.680Z",
  "verifiedAt": "2026-11-12T00:36:19.035Z",
  "verification": "Orchestrator check of every file the procedure was meant to produce.\n- poses_2026-11-11.jsonl: valid zero_landmark lines for left (ticks [1968,2445,2081,2044,2034]) and right ([2083,2509,2037,2006,2025]). The left line from 00:06:52Z is a spurious re-read and is excluded.\n- joint_ranges_{left,right}_lift_2026-11-11.json: shoulder_lift 1224..3651 (left) and 1291..3717 (right), wrapped false. These repeat the AA-46 sweeps within 4 ticks. Owner's note: the fold side stops on the shoulder bracket, the other end on a hard stop.\n- pose_snapshots.jsonl: fold snapshot, owner answer 'n, shoulder lift wrong, everything else good'.\nIndependent cross-check computed by the agent: the landmark shoulder_lift zeros (214.89 deg left, 220.52 deg right) put the swept stops at -107.3/+106.0 (left) and -107.1/+106.2 (right). The old midpoint zeros gave -79/+134 and -74/+140. That symmetry, across both arms within 0.2 deg, independently supports the landmark zeros. The other joints' landmark differences are 1-6 deg.\nI re-rendered the fold with the corrected zero (out/pose_snapshots/fold_corrected_lift.png); its shape matches the owner's description.\nResidual: the zero poses were matched by eye with no level, and the right-arm match was not explicitly confirmed. Applying the zeros awaits the owner's approval under AA-45."
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Orchestrator check of every file the procedure was meant to produce.
- poses_2026-11-11.jsonl: valid zero_landmark lines for left (ticks [1968,2445,2081,2044,2034]) and right ([2083,2509,2037,2006,2025]). The left line from 00:06:52Z is a spurious re-read and is excluded.
- joint_ranges_{left,right}_lift_2026-11-11.json: shoulder_lift 1224..3651 (left) and 1291..3717 (right), wrapped false. These repeat the AA-46 sweeps within 4 ticks. Owner's note: the fold side stops on the shoulder bracket, the other end on a hard stop.
- pose_snapshots.jsonl: fold snapshot, owner answer 'n, shoulder lift wrong, everything else good'.
Independent cross-check computed by the agent: the landmark shoulder_lift zeros (214.89 deg left, 220.52 deg right) put the swept stops at -107.3/+106.0 (left) and -107.1/+106.2 (right). The old midpoint zeros gave -79/+134 and -74/+140. That symmetry, across both arms within 0.2 deg, independently supports the landmark zeros. The other joints' landmark differences are 1-6 deg.
I re-rendered the fold with the corrected zero (out/pose_snapshots/fold_corrected_lift.png); its shape matches the owner's description.
Residual: the zero poses were matched by eye with no level, and the right-arm match was not explicitly confirmed. Applying the zeros awaits the owner's approval under AA-45.
<!-- SECTION:FINAL_SUMMARY:END -->
