---
id: AA-46
title: 'Re-measure shoulder_lift travel (both arms) and take pose snapshots, read-only'
status: To Do
assignee: []
created_date: '2026-11-11 23:06'
updated_date: '2026-11-11 23:06'
labels:
  - human-action
  - human-key-b9a975d02991ba9c
dependencies: []
references:
  - AA-45
ordinal: 46000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-45

Why only you: The shoulder_lift travel can only be found by moving the real arms by hand to their real end stops, with the robot powered but limp. Comparing the model renders with the real arms also needs someone looking at the robot. AA-45 planning shows the printing pose needs shoulder_lift near -91 deg, about raw tick 1092. The EEPROM/soft limits stop at -80.1 deg (left, tick 1216) and -74.4 deg (right, tick 1283). The owner chose option (b): re-measure rather than raise the printing pose.

Alternatives considered: The software is ready and read-only: scripts/record_joint_ranges.py and scripts/pose_snapshot.py both refuse if torque is on, and both were dry-run checked today. The existing data cannot answer the question. The 2026-10-24 sweep and the 2026-10-25 widen-to-observed record only where the hand stopped that day, not whether -91 deg is mechanically reachable. A model or URDF check cannot tell what the physical stop is. The agent will do the analysis and the EEPROM limit write afterwards, with the owner's approval of the numbers, not in this procedure.

Unlocks: The outcome has two parts.
(1) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json, each with a shoulder_lift min tick from a deliberate sweep to the fold-side end stop, plus the owner's note of what physically stops it (hard stop, lower arm touching the shoulder, cable, or chassis).
(2) Two or more lines in calibration_data/pose_snapshots.jsonl, each with the owner's y/n on whether the render matches the real arms. One is the fold pose; one is a hand-posed approximation of the printing start pose, compared with the plan's first frame.

Estimated active time: 25 minutes

Preparation: The robot is powered as in earlier sessions, with both arms limp (torque off). No tube attached, and the DexArm not needed. Hand within reach of the arm power switch. In a WSL terminal, cd ~/893/robot and run: uv run python scripts/attach_usb.py. The arm ports must attach; the DexArm may show MISSING, which is fine. Run every command from ~/893/robot. Nothing in this procedure commands motion or writes to the servos.

## Procedure

1. Run: uv run python scripts/record_joint_ranges.py --arm left --live --max-seconds 90 --out calibration_data/joint_ranges_left_lift_2026-11-11.json. While it records, move ONLY the left shoulder_lift joint (servo 2, the shoulder pitch) by hand. Fold it slowly toward the folded/rest side as far as it will go without forcing, three times. Then sweep it once to the other end, without forcing.
   - Expected: It prints RECORDING and status lines about every few seconds, then a JSON result after 90 s, and 'saved ...json'.
   - Verify: In the JSON, shoulder_lift has a 'lo' value, ideally at or below about 1100 ticks if -91 deg is reachable. wrapped is false.
   - Stop if: Stop if it prints REFUSED (torque on), if any joint feels stiff or powered, or if reaching the stop needs force or makes a part creak or bend. Then note where it stopped and do not push further.

2. Write down what physically stopped the left arm on the fold side: a hard stop in the servo/joint, the lower arm touching the shoulder bracket, a cable pulling, or the arm touching the chassis.
   - Expected: One short note, e.g. 'lower arm touches shoulder bracket'.
   - Verify: The note is given to the orchestrator with the results.
   - Stop if: Stop if you are not sure; describe what you saw instead.

3. Repeat step 1 for the right arm: uv run python scripts/record_joint_ranges.py --arm right --live --max-seconds 90 --out calibration_data/joint_ranges_right_lift_2026-11-11.json, sweeping only the right shoulder_lift the same way. Then note what stopped it, as in step 2.
   - Expected: Same as step 1, saved to the right-arm file.
   - Verify: The JSON shoulder_lift 'lo' value is present, wrapped is false, and there is a note on what stopped it.
   - Stop if: Same stop conditions as step 1.

4. Put both arms by hand into their usual fold/rest pose, then run: uv run python scripts/pose_snapshot.py --label fold --live. Open the PNG it saves, under out/pose_snapshots/, and compare it with the real arms.
   - Expected: It prints both arms' joint angles in degrees, saves a PNG of the model in that pose, then asks whether the render matches.
   - Verify: Answer y if the model arms bend the same way on the same sides as the real ones; otherwise n plus what differs. The script logs it to calibration_data/pose_snapshots.jsonl.
   - Stop if: Stop if it prints REFUSED, or if the PNG shows an arm bent the opposite way (that would be a calibration sign error); answer n and describe it.

5. Pose both arms by hand to look like the printing start pose: both the same; shoulder about 91 deg back toward the fold; elbow about 90 deg; wrist about -89 deg; pan and roll near zero. Get as close as you reasonably can. Run: uv run python scripts/pose_snapshot.py --label start --live --plan-start out/inchworm/timeline.jsonl.
   - Expected: It prints a start-pose check with the per-joint difference from the plan's first frame, OK or MISMATCH (MISMATCH is fine and expected for a hand pose), saves a PNG, and asks whether it matches.
   - Verify: Answer whether the render matches the real arms; the line is logged. The orchestrator reads the per-joint differences from the log.
   - Stop if: Stop if reaching that shoulder angle needs force (the result from steps 1-3 then already answers the question); snapshot the closest pose you can reach instead.

6. Leave the arms limp in the fold pose and tell the orchestrator you are done, with the two notes from steps 2 and 3.
   - Expected: Two new joint_ranges_*_lift_2026-11-11.json files and at least two new snapshot lines exist.
   - Verify: The orchestrator checks the files and the ticks.
   - Stop if: Stop if either file is missing; report which step failed.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Re-measure shoulder_lift travel (both arms) and take pose snapshots, read-only",
  "parent": "AA-45",
  "reason": "The shoulder_lift travel can only be found by moving the real arms by hand to their real end stops, with the robot powered but limp. Comparing the model renders with the real arms also needs someone looking at the robot. AA-45 planning shows the printing pose needs shoulder_lift near -91 deg, about raw tick 1092. The EEPROM/soft limits stop at -80.1 deg (left, tick 1216) and -74.4 deg (right, tick 1283). The owner chose option (b): re-measure rather than raise the printing pose.",
  "alternatives": "The software is ready and read-only: scripts/record_joint_ranges.py and scripts/pose_snapshot.py both refuse if torque is on, and both were dry-run checked today. The existing data cannot answer the question. The 2026-10-24 sweep and the 2026-10-25 widen-to-observed record only where the hand stopped that day, not whether -91 deg is mechanically reachable. A model or URDF check cannot tell what the physical stop is. The agent will do the analysis and the EEPROM limit write afterwards, with the owner's approval of the numbers, not in this procedure.",
  "outcome": "The outcome has two parts.\n(1) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json, each with a shoulder_lift min tick from a deliberate sweep to the fold-side end stop, plus the owner's note of what physically stops it (hard stop, lower arm touching the shoulder, cable, or chassis).\n(2) Two or more lines in calibration_data/pose_snapshots.jsonl, each with the owner's y/n on whether the render matches the real arms. One is the fold pose; one is a hand-posed approximation of the printing start pose, compared with the plan's first frame.",
  "preparation": "The robot is powered as in earlier sessions, with both arms limp (torque off). No tube attached, and the DexArm not needed. Hand within reach of the arm power switch. In a WSL terminal, cd ~/893/robot and run: uv run python scripts/attach_usb.py. The arm ports must attach; the DexArm may show MISSING, which is fine. Run every command from ~/893/robot. Nothing in this procedure commands motion or writes to the servos.",
  "category": "physical-access",
  "minutes": 25,
  "steps": [
    {
      "action": "Run: uv run python scripts/record_joint_ranges.py --arm left --live --max-seconds 90 --out calibration_data/joint_ranges_left_lift_2026-11-11.json. While it records, move ONLY the left shoulder_lift joint (servo 2, the shoulder pitch) by hand. Fold it slowly toward the folded/rest side as far as it will go without forcing, three times. Then sweep it once to the other end, without forcing.",
      "expected": "It prints RECORDING and status lines about every few seconds, then a JSON result after 90 s, and 'saved ...json'.",
      "verification": "In the JSON, shoulder_lift has a 'lo' value, ideally at or below about 1100 ticks if -91 deg is reachable. wrapped is false.",
      "stop": "Stop if it prints REFUSED (torque on), if any joint feels stiff or powered, or if reaching the stop needs force or makes a part creak or bend. Then note where it stopped and do not push further."
    },
    {
      "action": "Write down what physically stopped the left arm on the fold side: a hard stop in the servo/joint, the lower arm touching the shoulder bracket, a cable pulling, or the arm touching the chassis.",
      "expected": "One short note, e.g. 'lower arm touches shoulder bracket'.",
      "verification": "The note is given to the orchestrator with the results.",
      "stop": "Stop if you are not sure; describe what you saw instead."
    },
    {
      "action": "Repeat step 1 for the right arm: uv run python scripts/record_joint_ranges.py --arm right --live --max-seconds 90 --out calibration_data/joint_ranges_right_lift_2026-11-11.json, sweeping only the right shoulder_lift the same way. Then note what stopped it, as in step 2.",
      "expected": "Same as step 1, saved to the right-arm file.",
      "verification": "The JSON shoulder_lift 'lo' value is present, wrapped is false, and there is a note on what stopped it.",
      "stop": "Same stop conditions as step 1."
    },
    {
      "action": "Put both arms by hand into their usual fold/rest pose, then run: uv run python scripts/pose_snapshot.py --label fold --live. Open the PNG it saves, under out/pose_snapshots/, and compare it with the real arms.",
      "expected": "It prints both arms' joint angles in degrees, saves a PNG of the model in that pose, then asks whether the render matches.",
      "verification": "Answer y if the model arms bend the same way on the same sides as the real ones; otherwise n plus what differs. The script logs it to calibration_data/pose_snapshots.jsonl.",
      "stop": "Stop if it prints REFUSED, or if the PNG shows an arm bent the opposite way (that would be a calibration sign error); answer n and describe it."
    },
    {
      "action": "Pose both arms by hand to look like the printing start pose: both the same; shoulder about 91 deg back toward the fold; elbow about 90 deg; wrist about -89 deg; pan and roll near zero. Get as close as you reasonably can. Run: uv run python scripts/pose_snapshot.py --label start --live --plan-start out/inchworm/timeline.jsonl.",
      "expected": "It prints a start-pose check with the per-joint difference from the plan's first frame, OK or MISMATCH (MISMATCH is fine and expected for a hand pose), saves a PNG, and asks whether it matches.",
      "verification": "Answer whether the render matches the real arms; the line is logged. The orchestrator reads the per-joint differences from the log.",
      "stop": "Stop if reaching that shoulder angle needs force (the result from steps 1-3 then already answers the question); snapshot the closest pose you can reach instead."
    },
    {
      "action": "Leave the arms limp in the fold pose and tell the orchestrator you are done, with the two notes from steps 2 and 3.",
      "expected": "Two new joint_ranges_*_lift_2026-11-11.json files and at least two new snapshot lines exist.",
      "verification": "The orchestrator checks the files and the ticks.",
      "stop": "Stop if either file is missing; report which step failed."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The outcome has two parts.
(1) calibration_data/joint_ranges_{left,right}_lift_2026-11-11.json, each with a shoulder_lift min tick from a deliberate sweep to the fold-side end stop, plus the owner's note of what physically stops it (hard stop, lower arm touching the shoulder, cable, or chassis).
(2) Two or more lines in calibration_data/pose_snapshots.jsonl, each with the owner's y/n on whether the render matches the real arms. One is the fold pose; one is a hand-posed approximation of the printing start pose, compared with the plan's first frame.
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
  "key": "b9a975d02991ba9c472ec76332a48349ccdd36d56798f7edd84131943363f2f2",
  "phase": "proposed",
  "createdAt": "2026-11-11T23:06:14.558Z",
  "origin": "01a1cb2b-5f23-70da-906d-dcbf45d889dd",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
