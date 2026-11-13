---
id: AA-51
title: Supervised live full-cycle replay without the tube (AA-50)
status: In Progress
assignee:
  - '@human'
created_date: '2026-11-12 21:48'
updated_date: '2026-11-13 03:22'
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
  "phase": "active",
  "createdAt": "2026-11-12T21:48:39.439Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 6,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 (attach USB, revised AA-51) done. The owner pasted the output of `uv run python scripts/attach_usb.py`. arm_left: OK, already attached (BusId 10-4). arm_right: OK, already attached (BusId 9-3). All other devices OK as well: arm_leader, relay, cam_left, cam_right, cam_scanner, and dexarm newly attached (BusId 11-3). The script reported 8/8 devices OK with no errors. The DexArm, which was MISSING in the earlier attempt, is now attached. The agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/attach_usb.py",
      "advanced": true,
      "at": "2026-11-12T22:35:57.536Z"
    },
    {
      "step": 2,
      "note": "Step 2 (dry-run, revised AA-51) done. The owner pasted the output of `uv run python scripts/replay_cycle.py --skip-sim`. 4 'adjusted' elbow_flex lines: frame 0 and frame 86, left 3120->3081 (-3.4 deg, EEPROM [939, 3091]) and right 3076->3036 (-3.5 deg, EEPROM [906, 3046]). \"preflight: 0 problem(s) in 87 s\". 87 frames, arm motion ~41 s at 150 ticks/s plus guard checks. Phase list: frame 1 lift tube, frame 13 carry tube right, frame 25 left regrip with left gripper UNLOCK at 25 and LOCK at 49, frame 50 right regrip with right gripper UNLOCK at 50 and LOCK at 74, frame 75 lower tube/arms return to start. Abort hint: Ctrl-C or touch /tmp/inchworm_stop_move. Last line: \"DRY-RUN: nothing opened or moved. Add --live with the owner present.\" No REFUSED. All values match the step's expected output. The owner did not explicitly say nothing moved; the script states it. The agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/replay_cycle.py --skip-sim (dry-run)",
      "advanced": true,
      "at": "2026-11-12T22:38:59.921Z"
    },
    {
      "step": 3,
      "note": "Step 3 (live start, revised AA-51) REFUSED, not advanced. The owner pasted the output of the live command. Preflight: cached pass (2026-11-12T22:37:38Z). The script logged a \"start\" event at 2026-11-12T22:39:24Z. Measured ticks: left [1979,1402,3121,1014,2002], right [2075,1455,3075,970,2017]; gripper ticks left 1567, right 1556. These are the same ticks as in the earlier refused attempt, so the arms were not moved between attempts. diff_to_start_deg (measured vs plan): left [+0.6, -2.2, +3.5, +1.0, -5.2], right [+1.1, -3.1, +3.4, +0.2, -3.3]. Every joint is well within the new 10 degree limit, so the placement itself is not the problem. The event listed \"approach_problems\": [\"adjustment move (t=0.00): right_lower_arm within 0.1 mm of right_shoulder\"]. The script then printed \"REFUSED: the robot will not adjust to the start pose from here: adjustment move (t=0.00): right_lower_arm within 0.1 mm of right_shoulder\". The \"Start pose: the robot will make the final adjustment itself\" lines and the CONFIRM list did not appear. Nothing was opened or moved and no torque was enabled, per the script's refusal before the CONFIRM prompt (the agent did not observe the robot). The refused item is the model's collision check on the adjustment move, which the step lists as a stop condition. Agent's reading, not verified: the t=0.00 flag means the model already sees the right lower arm touching the right shoulder at the measured pose, so the cause is in the model or the right-arm geometry at the fold, not a joint that is too far off. The step allows a retry only for a joint too far off; that does not apply here. Needs orchestrator input.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of scripts/replay_cycle.py --live --allow-unverified-soft-limits (REFUSED, approach problem: right_lower_arm within 0.1 mm of right_shoulder)",
      "advanced": false,
      "at": "2026-11-12T22:39:49.703Z"
    },
    {
      "step": 3,
      "note": "Step 3 (live start, revised AA-51) recorded from the log, on the owner's instruction (\"all of these steps were finished\"). Not owner-observed and not from a pasted terminal output. I read calibration_data/replay_cycle_2026-11-12.jsonl (693 lines). The earlier refusal (right_lower_arm within 0.1 mm of right_shoulder at t=0.00) did not recur. The start event at 2026-11-13T00:23:01Z has \"approach_problems\": [] and diff_to_start_deg (left [+0.6,-2.2,+3.5,+1.0,-5.2], right [+1.1,-3.1,+3.4,+0.2,-3.3]), all within the 10 deg limit. The same measured ticks as the refused attempts, so the arms were not moved. The run went on to log gripper locks and frames, so a CONFIRM prompt must have been answered, but the CONFIRM text and the \"degrees to move\" lines are not in the log. The log shows nothing about the code change that cleared the collision flag. Not verified: the printed start-pose lines, and what was typed at CONFIRM.",
      "type": "automated",
      "reference": "robot/calibration_data/replay_cycle_2026-11-12.jsonl, start event 2026-11-13T00:23:01Z",
      "advanced": true,
      "at": "2026-11-13T03:21:31.677Z"
    },
    {
      "step": 4,
      "note": "Step 4 (CONFIRM, YES, grippers lock, move to frame 0) recorded from the log, on the owner's instruction. Not owner-observed. In replay_cycle_2026-11-12.jsonl, the run started 2026-11-13T00:23:01Z logs gripper_lock left LOCKED ticks 2026 (00:23:15Z) and right LOCKED ticks 2031 (00:23:20Z), then frame 0 at 00:23:24Z with error_ticks left [2,-3,10,2,-2] and right [4,-12,18,2,-9] (elbow errors of 10 and 18 ticks are the largest). Later runs show the same pattern (left 2026-2029, right 2029-2031). The goals are 2038/2041 (about 45 deg past open) and the lock ticks are 7-15 short; I did not check that against the script's arrival tolerance. Not in the log and not verified: that the YES prompt was answered, the CONFIRM list, the gripper opening before the turn, any noise or grinding, whether anything moved fast.",
      "type": "automated",
      "reference": "robot/calibration_data/replay_cycle_2026-11-12.jsonl, events 2026-11-13T00:23:15Z to 00:23:24Z",
      "advanced": true,
      "at": "2026-11-13T03:21:50.436Z"
    },
    {
      "step": 5,
      "note": "Step 5 (phases and regrips) recorded from the log, on the owner's instruction. Not owner-observed. The run started 2026-11-13T00:23:01Z logs frames 0 to 86 (frame 49 and 74 have no frame line; the lock events sit at those points). Left regrip: gripper_unlock left ticks 1608 (00:24:56Z), gripper_lock left LOCKED ticks 2028 (00:25:41Z). Right regrip: gripper_unlock right ticks 1589 (00:25:49Z), gripper_lock right LOCKED ticks 2031 (00:26:37Z). Frame 86 final ticks left [1973,1428,3083,1003,2060] and right [2065,1506,3029,966,2053]; the worst frame errors in this run are around 20 ticks. A second full run (start 00:51:59Z) shows the same pattern and ends in done at 00:59:48Z. No \"aborted\" event in these two runs. There is a gap of about 3.5 minutes between the right lock (00:55:32Z) and frame 75 (00:59:08Z) in the second run; the log does not say why. Other runs in the same file are different modes and abort on temperature: 'fluid tempo 0.5' (AA-50 fluid replay) with aborts \"left wrist_flex at 70 C\", \"left elbow_flex at 84 C\", \"right shoulder_lift at 76 C\", \"left shoulder_lift at 70 C\" and \"78 C\", and one \"ServoChecksumError: servo 2\" abort at 00:50:50Z. Later 'fluid' segments log max_temp_read_c values up to 150 C (left, 02:28:07Z), 128 and 109 on the left; whether these are real or bad reads is not known. These belong to other work, but they are in the same file. Not verified: smoothness, noise, contact, whether the owner pressed Enter at each prompt.",
      "type": "automated",
      "reference": "robot/calibration_data/replay_cycle_2026-11-12.jsonl, runs started 2026-11-13T00:23:01Z and 00:51:59Z (and aborted runs later in the file)",
      "advanced": true,
      "at": "2026-11-13T03:22:07.923Z"
    },
    {
      "step": 6,
      "note": "Step 6 (CYCLE COMPLETE, torque released) recorded from the log, on the owner's instruction. Not owner-observed. The first full run ends with {\"event\":\"done\"} at 2026-11-13T00:27:31Z and {\"event\":\"released\",\"errors\":{}} at 00:27:39Z. The second full run ends done at 00:59:48Z and released at 01:01:31Z, errors {} (about 1 min 43 s between done and released, which fits the owner supporting the arms before pressing Enter). Later runs also end in done then released with errors {}: 01:56:20Z/01:56:54Z, 02:28:07Z/02:28:13Z, 02:51:03Z/02:51:11Z, 02:52:32Z/02:54:54Z and 02:58:24Z/02:58:30Z. No torque-off sweep WARNING shows in the log (released errors are {} each time). Not in the log and not verified: the printed 'torque released. Log: ...' line, and that the arms went limp in the owner's hands.",
      "type": "automated",
      "reference": "robot/calibration_data/replay_cycle_2026-11-12.jsonl, done and released events at 00:27:31Z/00:27:39Z and 00:59:48Z/01:01:31Z",
      "advanced": true,
      "at": "2026-11-13T03:22:13.931Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-11-12T22:09:08.899Z",
      "hash": "eb37e669de424949fb34e0ad3a6fd492bfb387d0fc805debfdf6610788f97c94"
    },
    {
      "action": "accept",
      "at": "2026-11-12T22:35:08.754Z",
      "hash": "ff0e74c277109b038724b178aefb87b3a123ed98a8dfd113af59e0c2e2680779"
    }
  ],
  "acceptedHash": "ff0e74c277109b038724b178aefb87b3a123ed98a8dfd113af59e0c2e2680779",
  "acceptedAt": "2026-11-12T22:35:08.754Z"
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
