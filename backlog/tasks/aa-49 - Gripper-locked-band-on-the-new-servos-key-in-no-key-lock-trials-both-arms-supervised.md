---
id: AA-49
title: >-
  Gripper 45-degree position lock: owner confirms the key locks (both arms,
  supervised)
status: Done
assignee:
  - '@human'
created_date: '2026-11-12 19:13'
updated_date: '2026-11-12 22:09'
labels:
  - human-action
  - human-key-d9694f3656b1cc6f
dependencies: []
references:
  - AA-43
ordinal: 49000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-43

Why only you: AA-43 AC #4 needs the gripper locked band on the new servos. Owner finding 2026-11-12 (left arm, steps 1-3 of the first version): the locking piece slips on its screw at either end of its travel (a deliberate clutch), so the servo never stalls on the key and the key-stop method cannot find the locked angle. Owner decision: lock = a 45 degree turn past the open end; the owner verifies by eye that the key is locked. Only someone at the robot can insert the key and judge whether it is locked.

Alternatives considered: Software is done and tested: GripperLockSM has a position lock (lock_mode: position). It drives slowly (150 ticks/s, torque limit 300) to open end + 512 ticks (45 deg): left 1526 -> 2038, right 1529 -> 2041. It calls the gripper locked when the servo arrives within 12 ticks. Stopping short means an obstruction: FAULT and torque off. The servo cannot tell whether the key is seated, so the owner's eye is the check. Already logged in gripper_lock_probe_new_servos.jsonl: left no-key x3 and left key-in x3 at torque 300 (old key-stop method, slipping).

Unlocks: calibration_data/gripper_lock_probe_new_servos.jsonl gains 3 key-in position-lock trials per arm at torque 300, each settling at the 45-degree goal (in_locked_band), with the owner saying for each arm whether the key was locked every time. The agent then records the locked band (goal +-25 ticks) as owner-verified for AA-43 AC #4.

Estimated active time: 15 minutes

Preparation: Robot powered as in the 2026-11-12 gripper-ends session. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; others may be MISSING). No tube attached. Both arms limp (torque off) in their rest pose; the grippers are currently closed (L 2526, R 2499 ticks) and the probe opens them first. The locking key at hand. A hand within reach of the arm power switch. Abort at any time with the power switch or Ctrl-C (cleanup switches torque off). Run every command from ~/893/robot. About 30 minutes.

## Procedure

1. Dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key
   - Expected: Prints a plan for arm_left: lock mode position, lock goal 179.12 deg (2038 ticks), locked band [176.92, 181.32] deg, 150 ticks/s, then DRY-RUN. Nothing moves.
   - Verify: No servo motion; exit without error.
   - Stop if: Any motion, an error, or a different lock goal.

2. Left, key IN: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt insert the key into the open gripper, take your hands away, press Enter. While the gripper is closed on the key, look: is the key locked? Do not pull on it.
   - Expected: Per trial the gripper turns slowly about 45 degrees and stops; the key is locked; then it reopens. SUMMARY: 3/3 (in_locked_band).
   - Verify: SUMMARY line, and your look at the key in each trial.
   - Stop if: The key is not locked at the 45 degree position, grinding or buzzing for more than ~2 s, a hot servo, any other joint moving, a REFUSED or WARNING line, or short_of_locked_band / not_settled. Stop and report what you saw.

3. Right arm: dry-run, then the step 2 command with --arm right. Its plan shows lock goal 179.38 deg (2041 ticks), locked band [177.19, 181.58] deg.
   - Expected: Same as for the left arm: 3/3 in_locked_band with the key locked each time.
   - Verify: SUMMARY line and your look at the key.
   - Stop if: Same as step 2.

4. Remove the key. If you want the grippers closed at rest: uv run python scripts/gripper_ends.py --arm left --action close --no-tube --live, then the same with --arm right.
   - Expected: Each gripper closes slowly; RESULT: OK; torque off afterwards.
   - Verify: RESULT: OK and torque_after all 0.
   - Stop if: RESULT: NOT OK, a FAULT, or anything in the jaws.

5. Tell the orchestrator: for each arm, was the key locked in all 3 trials at the 45 degree position? Anything you saw or heard.
   - Expected: The log has 6 new key-in lines (3 per arm).
   - Verify: The orchestrator reads the log and checks counts, positions and settled flags.
   - Stop if: If a run failed or a file is missing, report which step.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Gripper 45-degree position lock: owner confirms the key locks (both arms, supervised)",
  "parent": "AA-43",
  "reason": "AA-43 AC #4 needs the gripper locked band on the new servos. Owner finding 2026-11-12 (left arm, steps 1-3 of the first version): the locking piece slips on its screw at either end of its travel (a deliberate clutch), so the servo never stalls on the key and the key-stop method cannot find the locked angle. Owner decision: lock = a 45 degree turn past the open end; the owner verifies by eye that the key is locked. Only someone at the robot can insert the key and judge whether it is locked.",
  "alternatives": "Software is done and tested: GripperLockSM has a position lock (lock_mode: position). It drives slowly (150 ticks/s, torque limit 300) to open end + 512 ticks (45 deg): left 1526 -> 2038, right 1529 -> 2041. It calls the gripper locked when the servo arrives within 12 ticks. Stopping short means an obstruction: FAULT and torque off. The servo cannot tell whether the key is seated, so the owner's eye is the check. Already logged in gripper_lock_probe_new_servos.jsonl: left no-key x3 and left key-in x3 at torque 300 (old key-stop method, slipping).",
  "outcome": "calibration_data/gripper_lock_probe_new_servos.jsonl gains 3 key-in position-lock trials per arm at torque 300, each settling at the 45-degree goal (in_locked_band), with the owner saying for each arm whether the key was locked every time. The agent then records the locked band (goal +-25 ticks) as owner-verified for AA-43 AC #4.",
  "preparation": "Robot powered as in the 2026-11-12 gripper-ends session. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; others may be MISSING). No tube attached. Both arms limp (torque off) in their rest pose; the grippers are currently closed (L 2526, R 2499 ticks) and the probe opens them first. The locking key at hand. A hand within reach of the arm power switch. Abort at any time with the power switch or Ctrl-C (cleanup switches torque off). Run every command from ~/893/robot. About 30 minutes.",
  "category": "physical-access",
  "minutes": 15,
  "steps": [
    {
      "action": "Dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key",
      "expected": "Prints a plan for arm_left: lock mode position, lock goal 179.12 deg (2038 ticks), locked band [176.92, 181.32] deg, 150 ticks/s, then DRY-RUN. Nothing moves.",
      "verification": "No servo motion; exit without error.",
      "stop": "Any motion, an error, or a different lock goal."
    },
    {
      "action": "Left, key IN: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt insert the key into the open gripper, take your hands away, press Enter. While the gripper is closed on the key, look: is the key locked? Do not pull on it.",
      "expected": "Per trial the gripper turns slowly about 45 degrees and stops; the key is locked; then it reopens. SUMMARY: 3/3 (in_locked_band).",
      "verification": "SUMMARY line, and your look at the key in each trial.",
      "stop": "The key is not locked at the 45 degree position, grinding or buzzing for more than ~2 s, a hot servo, any other joint moving, a REFUSED or WARNING line, or short_of_locked_band / not_settled. Stop and report what you saw."
    },
    {
      "action": "Right arm: dry-run, then the step 2 command with --arm right. Its plan shows lock goal 179.38 deg (2041 ticks), locked band [177.19, 181.58] deg.",
      "expected": "Same as for the left arm: 3/3 in_locked_band with the key locked each time.",
      "verification": "SUMMARY line and your look at the key.",
      "stop": "Same as step 2."
    },
    {
      "action": "Remove the key. If you want the grippers closed at rest: uv run python scripts/gripper_ends.py --arm left --action close --no-tube --live, then the same with --arm right.",
      "expected": "Each gripper closes slowly; RESULT: OK; torque off afterwards.",
      "verification": "RESULT: OK and torque_after all 0.",
      "stop": "RESULT: NOT OK, a FAULT, or anything in the jaws."
    },
    {
      "action": "Tell the orchestrator: for each arm, was the key locked in all 3 trials at the 45 degree position? Anything you saw or heard.",
      "expected": "The log has 6 new key-in lines (3 per arm).",
      "verification": "The orchestrator reads the log and checks counts, positions and settled flags.",
      "stop": "If a run failed or a file is missing, report which step."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 robot/calibration_data/gripper_lock_probe_new_servos.jsonl holds, for each arm: 3 no-key lock trials at torque 300 (settled position, load), and key-in trials at increasing torque (300, then 400, then 500 if needed) until 3/3 settle with the key seated, with the owner's word on whether the key was really locked. From this the agent proposes the locked band and lock_torque_limit_raw per arm, for owner approval (AA-43 AC #4, AA-17 AC #1).
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
  "key": "d9694f3656b1cc6fcfab655b8b37a8ea0197c21c1991efbca3c45e456901c91d",
  "phase": "verified",
  "createdAt": "2026-11-12T19:13:16.087Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 5,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 (left dry-run, revised AA-49) done. Owner pasted the terminal output of `uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key`. It printed: \"Plan: 3 trial(s) on arm_left, torque_limit_raw=300, key=in; lock mode position, lock goal 179.12 deg (2038 ticks), locked band [176.92, 181.32] deg; 150 ticks/s, torque OFF afterwards.\" and then \"DRY-RUN: nothing opened or moved.\" Mode, goal, ticks, band, speed and torque all match the step's expected values, and the goal is not the old 191.07 deg. The command ended with no error. The owner did not explicitly say there was no motion. The script states that nothing moved, and the agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm left --torque 300 --key (dry-run)",
      "advanced": true,
      "at": "2026-11-12T22:01:13.756Z"
    },
    {
      "step": 2,
      "note": "Step 2 (left, key IN, position lock, torque 300) done. Owner pasted the terminal output. 3 trials at 22:02:32Z, 22:02:40Z, 22:02:49Z, all sm_outcome LATCHED, classification in_locked_band, settled true. Position 178.59 deg (2032 ticks) in every trial; goal 179.12 deg (2038); locked band [176.92, 181.32] deg. The position is 6 ticks short of the goal, inside the 12-tick arrival tolerance. peak_abs_load 48/52/60, load_raw -36, current_raw 1/1/0, temperature 32 C. SUMMARY: \"3/3 trials as expected for key=in at torque 300\". No REFUSED or WARNING lines. The owner said \"went perfectly\". That is a general statement. The owner did not say trial by trial that the key was locked, and did not mention noise, heat or other-joint motion. The agent treats \"went perfectly\" as the owner's self-reported OK but does not treat it as a per-trial, per-arm statement that the key was locked. The final per-arm statement is still requested in step 5. The JSON file was not read, and the overtravel_goal_deg 191.07 field in each line appears to be a leftover field from the old method (not investigated). The evidence is the pasted output.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl, plus owner comment \"went perfectly\"",
      "advanced": true,
      "at": "2026-11-12T22:03:23.166Z"
    },
    {
      "step": 3,
      "note": "Step 3 (right arm) output received, not yet advanced. The owner pasted both the dry-run and the live run. Dry-run: plan for arm_right, position mode, lock goal 179.38 deg (2041 ticks), locked band [177.19, 181.58] deg, 150 ticks/s, torque 300. All values match the step. Nothing moved. Live run: 3 trials at 22:04:27Z, 22:04:38Z, 22:04:50Z, all sm_outcome LATCHED, classification in_locked_band, settled true. Position 178.86 deg (2035 ticks) every trial, 6 ticks short of the goal, within the 12-tick tolerance. peak_abs_load 56/52/56, load_raw -36, current_raw 0/1/0, temperature 32 C. SUMMARY: \"3/3 trials as expected for key=in at torque 300\". No REFUSED or WARNING lines. The step's verification also needs the owner's look at the key, and the owner has not said whether the key was locked in the right-arm trials, or mentioned noise, heat or other-joint motion. Holding the advance until the owner gives that. The overtravel_goal_deg 189.84 in each line looks like a leftover field from the old method; not investigated. The JSON file was not read.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm right --torque 300 --key (dry-run) and --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl",
      "advanced": false,
      "at": "2026-11-12T22:05:19.124Z"
    },
    {
      "step": 3,
      "note": "Step 3 (right arm, key IN, position lock, torque 300) done. Evidence is the owner-pasted terminal output (dry-run matched: goal 179.38 deg / 2041 ticks, band [177.19, 181.58]). Live: 3 trials, all LATCHED, in_locked_band, settled true, 178.86 deg (2035 ticks) each, SUMMARY 3/3, 32 C, no REFUSED or WARNING lines. Owner then answered my question about whether the key was locked each time and about noise, heat and other-joint motion with \"worked great\". Self-reported and general: not a trial-by-trial statement, and it does not mention noise, heat or other joints explicitly. The agent reads it as the owner's OK for the right arm, but the explicit per-arm statement is still asked for in step 5. The agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm right --torque 300 --key (dry-run and --live --trials 3), plus owner reply \"worked great\"",
      "advanced": true,
      "at": "2026-11-12T22:05:31.882Z"
    },
    {
      "step": 4,
      "note": "Step 4 skipped by the owner's choice. The owner said \"skip this\" when offered the optional gripper close (gripper_ends.py --action close for each arm). The step makes closing optional (\"If you want the grippers closed at rest\"), so no RESULT/torque_after output exists and none was expected. Not reported by the owner: whether the key was taken out of both grippers, and the state the grippers were left in (open or closed). After the last probe trial the script ends with torque OFF per its plan line, but the agent did not observe the grippers. Both are asked about together with the step 5 report.",
      "type": "self-reported",
      "reference": "Owner reply in conversation: \"skip this\"",
      "advanced": true,
      "at": "2026-11-12T22:07:08.101Z"
    },
    {
      "step": 5,
      "note": "Step 5 (owner report). Self-reported, not observed by the agent. I asked four questions: (1) left arm, key locked in all 3 trials; (2) right arm, the same; (3) anything unusual (noise, heat, other-joint motion); (4) key out and the state of the grippers. The owner replied \"everything is right for all 4 questions\". The agent reads this as: key locked in all 3 trials on both arms, nothing unusual, key out. Caveats: the reply is a blanket statement, not a trial-by-trial one. Question 4 asked for the gripper state (open, closed or limp), and \"right\" does not say which. The gripper state is therefore not recorded. The earlier \"went perfectly\" (left) and \"worked great\" (right) are consistent with it. Log evidence from the pasted output (the JSON file was not read): 6 key-in position-lock trials, all LATCHED, in_locked_band, settled true. Left 178.59 deg (2032 ticks) against goal 2038, band [176.92, 181.32]. Right 178.86 deg (2035 ticks) against goal 2041, band [177.19, 181.58]. Both at torque 300, 32 C. The step 4 close was skipped by choice. For the orchestrator to check: the 6 new lines in gripper_lock_probe_new_servos.jsonl; the overtravel_goal_deg field (191.07 left, 189.84 right) in the new lines looks left over from the old method.",
      "type": "self-reported",
      "reference": "Owner reply in conversation: \"everything is right for all 4 questions\"; owner-pasted output of the left and right --live --trials 3 runs",
      "advanced": true,
      "at": "2026-11-12T22:08:11.099Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-11-12T19:27:00.997Z",
      "hash": "d052f14d2197f3414652807514a5308f47934659308b8cfdcd2a60ce9d9b4ff7"
    },
    {
      "action": "accept",
      "at": "2026-11-12T21:59:31.523Z",
      "hash": "8d4340ed88fb0e0db917c69d0df672b44f496601827f084606cd8acc8348900c"
    }
  ],
  "acceptedHash": "8d4340ed88fb0e0db917c69d0df672b44f496601827f084606cd8acc8348900c",
  "acceptedAt": "2026-11-12T21:59:31.523Z",
  "summary": "went well. everything worked as expected",
  "completedAt": "2026-11-12T22:08:50.665Z",
  "verifiedAt": "2026-11-12T22:09:27.718Z",
  "verification": "Read calibration_data/gripper_lock_probe_new_servos.jsonl directly (not only the pasted output). There are 9 new key-in position-lock trials at torque_limit_raw 300, all with sm_outcome LATCHED, classification in_locked_band and settled true. Left 22:02:32-22:02:49: 2032 ticks (178.59 deg) x3 against goal 2038, band [176.92, 181.32]. Right 22:04:27-22:04:50: 2035 ticks (178.86 deg) x3 against goal 2041, band [177.19, 181.58]. An extra left run 22:06:45-22:07:15, not mentioned in the walkthrough: 2034 ticks x3, also in band. Each trial stopped 4-6 ticks short of the goal, inside the 12-tick tolerance. Peak |load| 40-60, current 0-1 raw, 32-33 C. That a key was locked each time is owner self-report ('everything is right for all 4 questions'); software cannot observe it, by design. The resting state of the grippers was not reported. The overtravel_goal_deg field in these lines is left over from the old method (the probe's log field), not a behaviour problem."
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Read calibration_data/gripper_lock_probe_new_servos.jsonl directly (not only the pasted output). There are 9 new key-in position-lock trials at torque_limit_raw 300, all with sm_outcome LATCHED, classification in_locked_band and settled true. Left 22:02:32-22:02:49: 2032 ticks (178.59 deg) x3 against goal 2038, band [176.92, 181.32]. Right 22:04:27-22:04:50: 2035 ticks (178.86 deg) x3 against goal 2041, band [177.19, 181.58]. An extra left run 22:06:45-22:07:15, not mentioned in the walkthrough: 2034 ticks x3, also in band. Each trial stopped 4-6 ticks short of the goal, inside the 12-tick tolerance. Peak |load| 40-60, current 0-1 raw, 32-33 C. That a key was locked each time is owner self-report ('everything is right for all 4 questions'); software cannot observe it, by design. The resting state of the grippers was not reported. The overtravel_goal_deg field in these lines is left over from the old method (the probe's log field), not a behaviour problem.
<!-- SECTION:FINAL_SUMMARY:END -->
