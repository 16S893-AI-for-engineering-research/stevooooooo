---
id: AA-51
title: Supervised live full-cycle replay without the tube (AA-50)
status: To Do
assignee: []
created_date: '2026-11-12 21:48'
updated_date: '2026-11-12 21:48'
labels:
  - human-action
  - human-key-56aeb9883b7721f9
dependencies: []
references:
  - AA-50
ordinal: 51000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-50

Why only you: This is the first live two-arm, multi-joint motion on the new servos. AGENTS.md rule 5 requires the owner to supervise the first live execution of a new path. Only someone at the robot can place the arms in the fold, confirm there is no tube, keep a hand on the power switch, support the arms before torque is released, and report what the motion looked and sounded like.

Alternatives considered: Done by the agent: scripts/replay_cycle.py with dry-run, preflight (0 problems on the real timeline, both arms together, no tube) and a simulated full cycle (85 moves, 6 gripper actions, worst error 20 ticks). Every arm goal goes through GuardedArm. The owner already approved waiving the unverified-soft-limit gate for this test; the limit values stay enforced. Nothing more can be tested without powered motion at the robot.

Unlocks: calibration_data/replay_cycle_<date>.jsonl holds a run that ends in 'done' and 'released' (or an abort with its reason), plus the owner's observations: did both arms follow the cycle smoothly, did each gripper open and turn about 45 degrees at its regrip, and was there any contact, grinding or alarming sound.

Estimated active time: 20 minutes

Preparation: Robot powered as usual; both arms torque-off and placed by hand in the fold (the printing start pose, as in the last pose snapshot); no tube anywhere in the cell; DexArm parked where it normally sits; hand within reach of the power switch; WSL terminal in /home/stevotimp/893/robot. Run uv run python scripts/attach_usb.py first if the arms are not attached to WSL.

## Procedure

1. Attach the USB devices: uv run python scripts/attach_usb.py
   - Expected: Both arm serial ports are attached to WSL.
   - Verify: The script reports both arms attached without errors.
   - Stop if: Any device missing or an error: stop and report the output.

2. Dry-run: uv run python scripts/replay_cycle.py --skip-sim
   - Expected: Lists 4 'adjusted' elbow lines, 'preflight: cached pass' or '0 problem(s)', the phase list with left UNLOCK/LOCK at frames 25/49 and right at 50/74, then DRY-RUN. Nothing moves.
   - Verify: Last line starts with DRY-RUN; no motion.
   - Stop if: REFUSED, any preflight problem, or any motion.

3. With the arms torque-off in the fold and no tube in the cell, start the live run: uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits "owner 2026-11-13: AA-50 no-tube replay"
   - Expected: It prints the start-pose check with every joint inside 3 deg, then the CONFIRM list.
   - Verify: The start-pose summary shows no failing joint, and the CONFIRM prompt appears.
   - Stop if: REFUSED (torque on, drive config, start pose off): do not force it; report the printed line. You may re-place the arms by hand and retry once.

4. Check the CONFIRM list (no tube, DexArm parked, hands and cables clear, you at the power switch), then type YES. Watch both grippers.
   - Expected: Each gripper opens to its open end and then turns slowly about 45 degrees and stops; two gripper_lock lines print with state LOCKED. Then the arms make a small move to frame 0.
   - Verify: Two gripper_lock lines with LOCKED; the arms barely move to frame 0.
   - Stop if: Any grinding, buzzing over ~2 s, an arm moving fast or toward something: press Ctrl-C (or switch power off if it does not stop). Report what happened.

5. At each 'next phase' prompt, look that the cell is still clear and press Enter. There are 4 or 5 phases: lift, carry right, left regrip, right regrip, lower and return. Type q at a prompt to stop early.
   - Expected: The arms move slowly and smoothly through each phase; at each regrip that arm's gripper opens, the arm moves, and the gripper turns about 45 degrees to lock again. Frame lines print 'reached'.
   - Verify: Frame lines keep printing; gripper_unlock/gripper_lock lines appear at the regrips.
   - Stop if: Contact between arms, with the chassis or the DexArm, jerky motion, grinding, or an ABORTED line: press Ctrl-C if it is still moving; when it says 'Holding', support both arms and press Enter. Then report.

6. At 'CYCLE COMPLETE' (or after an abort), support both arms with your hands and press Enter.
   - Expected: 'torque released. Log: calibration_data/replay_cycle_<date>.jsonl'. The arms go limp in your hands.
   - Verify: The log path line prints and the arms are limp.
   - Stop if: An arm stays stiff or a WARNING about the torque-off sweep: switch the power off and report.

7. Tell the orchestrator how it went: did both arms follow the cycle smoothly, did each gripper unlock and relock about 45 degrees, any contact, sounds, hot servos, or anything that looked wrong.
   - Expected: Your observations are recorded.
   - Verify: The orchestrator reads the JSONL log (start, frames, gripper events, done/aborted, released).
   - Stop if: If anything is unclear, say so instead of guessing.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Supervised live full-cycle replay without the tube (AA-50)",
  "parent": "AA-50",
  "reason": "This is the first live two-arm, multi-joint motion on the new servos. AGENTS.md rule 5 requires the owner to supervise the first live execution of a new path. Only someone at the robot can place the arms in the fold, confirm there is no tube, keep a hand on the power switch, support the arms before torque is released, and report what the motion looked and sounded like.",
  "alternatives": "Done by the agent: scripts/replay_cycle.py with dry-run, preflight (0 problems on the real timeline, both arms together, no tube) and a simulated full cycle (85 moves, 6 gripper actions, worst error 20 ticks). Every arm goal goes through GuardedArm. The owner already approved waiving the unverified-soft-limit gate for this test; the limit values stay enforced. Nothing more can be tested without powered motion at the robot.",
  "outcome": "calibration_data/replay_cycle_<date>.jsonl holds a run that ends in 'done' and 'released' (or an abort with its reason), plus the owner's observations: did both arms follow the cycle smoothly, did each gripper open and turn about 45 degrees at its regrip, and was there any contact, grinding or alarming sound.",
  "preparation": "Robot powered as usual; both arms torque-off and placed by hand in the fold (the printing start pose, as in the last pose snapshot); no tube anywhere in the cell; DexArm parked where it normally sits; hand within reach of the power switch; WSL terminal in /home/stevotimp/893/robot. Run uv run python scripts/attach_usb.py first if the arms are not attached to WSL.",
  "category": "physical-access",
  "minutes": 20,
  "steps": [
    {
      "action": "Attach the USB devices: uv run python scripts/attach_usb.py",
      "expected": "Both arm serial ports are attached to WSL.",
      "verification": "The script reports both arms attached without errors.",
      "stop": "Any device missing or an error: stop and report the output."
    },
    {
      "action": "Dry-run: uv run python scripts/replay_cycle.py --skip-sim",
      "expected": "Lists 4 'adjusted' elbow lines, 'preflight: cached pass' or '0 problem(s)', the phase list with left UNLOCK/LOCK at frames 25/49 and right at 50/74, then DRY-RUN. Nothing moves.",
      "verification": "Last line starts with DRY-RUN; no motion.",
      "stop": "REFUSED, any preflight problem, or any motion."
    },
    {
      "action": "With the arms torque-off in the fold and no tube in the cell, start the live run: uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits \"owner 2026-11-13: AA-50 no-tube replay\"",
      "expected": "It prints the start-pose check with every joint inside 3 deg, then the CONFIRM list.",
      "verification": "The start-pose summary shows no failing joint, and the CONFIRM prompt appears.",
      "stop": "REFUSED (torque on, drive config, start pose off): do not force it; report the printed line. You may re-place the arms by hand and retry once."
    },
    {
      "action": "Check the CONFIRM list (no tube, DexArm parked, hands and cables clear, you at the power switch), then type YES. Watch both grippers.",
      "expected": "Each gripper opens to its open end and then turns slowly about 45 degrees and stops; two gripper_lock lines print with state LOCKED. Then the arms make a small move to frame 0.",
      "verification": "Two gripper_lock lines with LOCKED; the arms barely move to frame 0.",
      "stop": "Any grinding, buzzing over ~2 s, an arm moving fast or toward something: press Ctrl-C (or switch power off if it does not stop). Report what happened."
    },
    {
      "action": "At each 'next phase' prompt, look that the cell is still clear and press Enter. There are 4 or 5 phases: lift, carry right, left regrip, right regrip, lower and return. Type q at a prompt to stop early.",
      "expected": "The arms move slowly and smoothly through each phase; at each regrip that arm's gripper opens, the arm moves, and the gripper turns about 45 degrees to lock again. Frame lines print 'reached'.",
      "verification": "Frame lines keep printing; gripper_unlock/gripper_lock lines appear at the regrips.",
      "stop": "Contact between arms, with the chassis or the DexArm, jerky motion, grinding, or an ABORTED line: press Ctrl-C if it is still moving; when it says 'Holding', support both arms and press Enter. Then report."
    },
    {
      "action": "At 'CYCLE COMPLETE' (or after an abort), support both arms with your hands and press Enter.",
      "expected": "'torque released. Log: calibration_data/replay_cycle_<date>.jsonl'. The arms go limp in your hands.",
      "verification": "The log path line prints and the arms are limp.",
      "stop": "An arm stays stiff or a WARNING about the torque-off sweep: switch the power off and report."
    },
    {
      "action": "Tell the orchestrator how it went: did both arms follow the cycle smoothly, did each gripper unlock and relock about 45 degrees, any contact, sounds, hot servos, or anything that looked wrong.",
      "expected": "Your observations are recorded.",
      "verification": "The orchestrator reads the JSONL log (start, frames, gripper events, done/aborted, released).",
      "stop": "If anything is unclear, say so instead of guessing."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 calibration_data/replay_cycle_<date>.jsonl holds a run that ends in 'done' and 'released' (or an abort with its reason), plus the owner's observations: did both arms follow the cycle smoothly, did each gripper open and turn about 45 degrees at its regrip, and was there any contact, grinding or alarming sound.
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
  "key": "56aeb9883b7721f9ef1fb18840c1b55b0e96568fab5e904ac17fae201eef5845",
  "phase": "proposed",
  "createdAt": "2026-11-12T21:48:39.439Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
