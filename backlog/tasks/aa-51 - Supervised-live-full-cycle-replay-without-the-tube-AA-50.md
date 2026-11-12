---
id: AA-51
title: Supervised live full-cycle replay without the tube (AA-50)
status: In Progress
assignee:
  - '@human'
created_date: '2026-11-12 21:48'
updated_date: '2026-11-12 22:24'
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

Preparation: Robot powered as usual; both arms torque-off and placed by hand roughly in the fold (the printing start pose); within about 10 degrees per joint is enough, the robot makes the final adjustment itself; no tube anywhere in the cell; DexArm parked where it normally sits; hand within reach of the power switch; WSL terminal in /home/stevotimp/893/robot. Run uv run python scripts/attach_usb.py first if the arms are not attached to WSL.

## Procedure

1. Attach the USB devices: uv run python scripts/attach_usb.py
   - Expected: Both arm serial ports are attached to WSL.
   - Verify: The script reports both arms attached without errors.
   - Stop if: Any device missing or an error: stop and report the output.

2. Dry-run: uv run python scripts/replay_cycle.py --skip-sim
   - Expected: Lists 4 'adjusted' elbow lines, 'preflight: cached pass' or '0 problem(s)', the phase list with left UNLOCK/LOCK at frames 25/49 and right at 50/74, then DRY-RUN. Nothing moves.
   - Verify: Last line starts with DRY-RUN; no motion.
   - Stop if: REFUSED, any preflight problem, or any motion.

3. With the arms torque-off and roughly in the fold (no tube in the cell), start the live run: uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits "owner 2026-11-13: AA-50 no-tube replay"
   - Expected: It prints 'Start pose: the robot will make the final adjustment itself' with the degrees each joint will move (each within 10), then the CONFIRM list. Nothing has moved yet.
   - Verify: The degrees-to-move lines are small (a few degrees), and the CONFIRM prompt appears.
   - Stop if: REFUSED (torque on, drive config, a joint more than 10 degrees from the start pose, or the adjustment touching something in the model): do not force it; report the printed line. For a joint too far off, you may move that arm closer by hand and retry once.

4. Check the CONFIRM list (no tube, DexArm parked, hands and cables clear, you at the power switch), then type YES. Watch both grippers.
   - Expected: Each gripper opens to its open end and then turns slowly about 45 degrees and stops; two gripper_lock lines print with state LOCKED. Then both arms slowly make the final adjustment to the start pose (a few degrees).
   - Verify: Two gripper_lock lines with LOCKED; the arms move only a few degrees to the start pose.
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
  "preparation": "Robot powered as usual; both arms torque-off and placed by hand roughly in the fold (the printing start pose); within about 10 degrees per joint is enough, the robot makes the final adjustment itself; no tube anywhere in the cell; DexArm parked where it normally sits; hand within reach of the power switch; WSL terminal in /home/stevotimp/893/robot. Run uv run python scripts/attach_usb.py first if the arms are not attached to WSL.",
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
      "action": "With the arms torque-off and roughly in the fold (no tube in the cell), start the live run: uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits \"owner 2026-11-13: AA-50 no-tube replay\"",
      "expected": "It prints 'Start pose: the robot will make the final adjustment itself' with the degrees each joint will move (each within 10), then the CONFIRM list. Nothing has moved yet.",
      "verification": "The degrees-to-move lines are small (a few degrees), and the CONFIRM prompt appears.",
      "stop": "REFUSED (torque on, drive config, a joint more than 10 degrees from the start pose, or the adjustment touching something in the model): do not force it; report the printed line. For a joint too far off, you may move that arm closer by hand and retry once."
    },
    {
      "action": "Check the CONFIRM list (no tube, DexArm parked, hands and cables clear, you at the power switch), then type YES. Watch both grippers.",
      "expected": "Each gripper opens to its open end and then turns slowly about 45 degrees and stops; two gripper_lock lines print with state LOCKED. Then both arms slowly make the final adjustment to the start pose (a few degrees).",
      "verification": "Two gripper_lock lines with LOCKED; the arms move only a few degrees to the start pose.",
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
  "key": "56aeb9883b7721f9ef1fb18840c1b55b0e96568fab5e904ac17fae201eef5845",
  "phase": "paused",
  "createdAt": "2026-11-12T21:48:39.439Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 2,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 (attach USB) done for the arms. The owner pasted the output of `uv run python scripts/attach_usb.py`. arm_left: OK, already attached (BusId 10-4). arm_right: OK, already attached (BusId 9-3). Others OK: arm_leader, relay, cam_left, cam_scanner, and cam_right newly attached (10-2). dexarm: MISSING (\"not found in usbipd device list\", VID:PID 0483:5740). The script reported 7/8 devices OK. The step's expected result and verification concern only the two arm ports, which are attached with no errors. The step's stop line says \"Any device missing\", and I read it as meaning the arm devices. This is an interpretation. The DexArm is not visible to WSL (unplugged, off, or not enumerated on Windows is unknown). The procedure says the DexArm should be parked where it normally sits; I do not know whether the replay script needs a DexArm connection. The step 2 dry-run will show that without moving anything. The agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/attach_usb.py",
      "advanced": true,
      "at": "2026-11-12T22:10:09.197Z"
    },
    {
      "step": 2,
      "note": "Step 2 (dry-run) done. Owner pasted the output of `uv run python scripts/replay_cycle.py --skip-sim`. 4 'adjusted' elbow_flex lines: frame 0 and frame 86, left 3120->3081 (-3.4 deg, EEPROM [939, 3091]) and right 3076->3036 (-3.5 deg, EEPROM [906, 3046]). \"preflight: 0 problem(s) in 89 s\" (a fresh preflight, not the cached one; both are accepted). 87 frames, arm motion ~41 s at 150 ticks/s plus guard checks. Phase list: frame 1 lift tube, frame 13 carry tube right, frame 25 left regrip with left gripper UNLOCK at 25 and LOCK at 49, frame 50 right regrip with right gripper UNLOCK at 50 and LOCK at 74, frame 75 lower tube/arms return to start. That is 5 phases, and the step allows 4 or 5. Abort hint printed: Ctrl-C or touch /tmp/inchworm_stop_move. Last line: \"DRY-RUN: nothing opened or moved. Add --live with the owner present.\" No REFUSED. No DexArm complaint appeared, so the MISSING DexArm from step 1 did not block the dry-run; whether the live run needs it is still unknown. The owner did not explicitly say nothing moved; the script states it. The agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/replay_cycle.py --skip-sim (dry-run)",
      "advanced": true,
      "at": "2026-11-12T22:19:22.838Z"
    },
    {
      "step": 3,
      "note": "Step 3 (live start) REFUSED, not advanced. Owner pasted the output of the live command. Preflight: cached pass (2026-11-12T22:12:03Z). Start-pose check (tol 3.0 deg per joint): MISMATCH. Five joints off (measured minus planned): left elbow_flex +91.4 vs +87.9 (+3.5), left wrist_roll -5.2 vs 0.0 (-5.2), right shoulder_lift -92.6 vs -89.5 (-3.1), right elbow_flex +91.2 vs +87.8 (+3.4), right wrist_roll -3.3 vs 0.0 (-3.3). Other joints within tolerance. The script logged an event \"start\" with start_pose_ok false (left ticks [1979,1402,3121,1014,2002], right ticks [2075,1455,3075,970,2017], gripper ticks left 1567, right 1556) and printed \"REFUSED: the arms are not at the plan's start pose (move them by hand, torque off)\". The CONFIRM list never appeared, nothing was opened and no arm moved or was torqued (the agent did not observe the robot; the script refused before the CONFIRM prompt). Agent's inference, not verified: the two elbow ticks (left 3121, right 3075) match the last fold snapshot, but the plan start was adjusted to 3081 / 3036 to stay inside the EEPROM limits (3091 / 3046), 3.4 to 3.5 deg away, just past the 3.0 deg tolerance. If so, re-placing by hand may not bring the elbows within tolerance and the cause is in the plan or tolerance, not in the owner's placement. The procedure allows one retry after re-placing the arms by hand. Owner decision pending: retry once or pause for the orchestrator.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/replay_cycle.py --live --allow-unverified-soft-limits (REFUSED, start-pose MISMATCH)",
      "advanced": false,
      "at": "2026-11-12T22:20:48.574Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-11-12T22:09:08.899Z",
      "hash": "eb37e669de424949fb34e0ad3a6fd492bfb387d0fc805debfdf6610788f97c94"
    }
  ],
  "acceptedHash": "eb37e669de424949fb34e0ad3a6fd492bfb387d0fc805debfdf6610788f97c94",
  "acceptedAt": "2026-11-12T22:09:08.899Z"
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
