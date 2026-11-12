---
id: AA-49
title: >-
  Gripper 45-degree position lock: owner confirms the key locks (both arms,
  supervised)
status: In Progress
assignee:
  - '@human'
created_date: '2026-11-12 19:13'
updated_date: '2026-11-12 20:46'
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
- [ ] #1 robot/calibration_data/gripper_lock_probe_new_servos.jsonl holds, for each arm: 3 no-key lock trials at torque 300 (settled position, load), and key-in trials at increasing torque (300, then 400, then 500 if needed) until 3/3 settle with the key seated, with the owner's word on whether the key was really locked. From this the agent proposes the locked band and lock_torque_limit_raw per arm, for owner approval (AA-43 AC #4, AA-17 AC #1).
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
  "phase": "paused",
  "createdAt": "2026-11-12T19:13:16.087Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 2,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 (left dry-run) done. The owner pasted the terminal output of `uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --no-key`. It printed: \"Plan: 3 trial(s) on arm_left, torque_limit_raw=300, key=OUT; lock goal 191.07 deg, locked band [156.09, 160.58] deg; 150 ticks/s, torque OFF afterwards.\" and then \"DRY-RUN: nothing opened or moved.\" All values match the expected plan and the command ended without an error. The owner did not explicitly say there was no noise or motion. The script says nothing moved in dry-run mode, and the agent did not observe the robot.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm left --torque 300 --no-key (dry-run)",
      "advanced": true,
      "at": "2026-11-12T19:27:57.896Z"
    },
    {
      "step": 2,
      "note": "Step 2 (left, NO key, torque 300) done. Owner pasted the terminal output. 3 trials at 19:29:09Z, 19:29:23Z, 19:29:37Z, all classified overtravel_past_locked_band with settled true. Position 190.81 deg (2171 ticks) in every trial; goal 191.07 deg; locked band [156.09, 160.58] deg. peak_abs_load 72/68/72, load_raw -24, current_raw 0/0/1, speed_raw 0, temperature 32 C. The state machine reported \"LockVerificationFailed: LockVerdict.UNVERIFIED\" in all three trials, which is what the agent expects with no key present, but the agent did not check that against the code. SUMMARY: \"3/3 trials as expected for key=OUT at torque 300\". No REFUSED or WARNING lines. The owner did not say whether there was grinding, buzzing, heat or other-joint motion, so those stop conditions are unreported rather than confirmed absent. The JSON file itself was not read; the evidence is the pasted output.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm left --torque 300 --no-key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl",
      "advanced": true,
      "at": "2026-11-12T19:30:00.683Z"
    },
    {
      "step": 3,
      "note": "Step 3 (left, key IN, torque 300) NOT passed: SUMMARY 0/3 as expected for key=in. The owner pasted the terminal output. Three trials at 19:31:18Z, 19:31:41Z, 19:31:58Z, all classified overtravel_past_locked_band, settled true. Position 190.37 deg (2166 ticks) every trial, against the locked band [156.09, 160.58] deg and the overtravel goal 191.07 deg. peak_abs_load 84/84/80, load_raw -44, current_raw 1, temperature 32 C. No REFUSED or WARNING lines.\\n\\nThe key-in position (190.37 deg, 2166) is almost the same as the no-key position from step 2 (190.81 deg, 2171). The jaw closed essentially to the overtravel goal with the key reportedly in, about 30 deg past the locked band. The only visible effect of the key is a slightly higher load (-44 vs -24). The agent has not established why. Possible explanations the data cannot separate: the key was not in the jaw path, was not seated, or was squeezed out or deformed. The owner did not say whether the key was in the jaw when it closed, whether it was seated or ejected, or whether there was any noise, heat or deformation, so those stop conditions are unreported. The JSON file was not read; the evidence is the pasted output.\\n\\nStep 4 would raise torque to 400 and 500. With the jaw already reaching the goal at 300, more torque looks unlikely to explain the missing stop at the band. The agent is holding step 4 until the owner describes what the key did.",
      "type": "automated",
      "reference": "Owner-pasted terminal output of gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl; compared with step 2 output",
      "advanced": false,
      "at": "2026-11-12T19:32:37.197Z"
    },
    {
      "step": 3,
      "note": "Owner's account of the step 3 key-in trials (left arm, torque 300), self-reported. Key was inserted in the jaw. Owner's hypothesis: the torque at 300 was stronger than the screw friction holding the locking piece, so the locking piece spun after it had locked. This would explain why the jaw ended at 190.37 deg (2166), about as far as the no-key position (190.81 deg, 2171). The hypothesis is the owner's, not confirmed. The data is consistent with it but cannot prove it: the key-in load (-44) is only slightly above no-key (-24). Not reported: noise, heat, key condition, and whether the locking piece or its screw is now in a different state. Consequence: the procedure's step 4 raises torque to 400/500, which goes the wrong way if this is right, so step 4 is held. The agent did not suggest a lower torque, because the procedure has no lower-torque trials and the script's torque range is set by the orchestrator. Needs orchestrator input: revised procedure, e.g. lower-torque trials or a torque below the screw-friction limit, and a check of the locking piece. Not advanced.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation: \"The key was inserted. I think the torque was stronger than the screw friction that is holding the locking piece. This causes it to spin after its already locked.\"",
      "advanced": false,
      "at": "2026-11-12T19:34:50.568Z"
    },
    {
      "step": 3,
      "note": "Owner's hand check of the left locking piece after the step 3 key-in trials (self-reported, no measurement). Quote: \"its stiff but I can spin it if I twist hard without moving the servo\". So the piece turns on its own screw under hand torque without the servo horn turning. This is consistent with the owner's hypothesis that the piece can slip on its screw friction, but it does not show that the servo's torque at limit 300 is what made it slip. The slip torque was not measured, and the 300 servo torque is not known in matching units, so the agent cannot say whether a lower torque would stay below the friction. Not reported: noise, heat, key condition, screw loose or tight, or any change since before the trials. Step 3 is still not passed and step 4 stays held. Needs orchestrator input: whether to measure or adjust the friction (e.g. screw tension) and what torque to try, before any further key-in trials.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation: \"its stiff but I can spin it if I twist hard without moving the servo\"",
      "advanced": false,
      "at": "2026-11-12T19:37:10.248Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-11-12T19:27:00.997Z",
      "hash": "d052f14d2197f3414652807514a5308f47934659308b8cfdcd2a60ce9d9b4ff7"
    }
  ],
  "acceptedHash": "d052f14d2197f3414652807514a5308f47934659308b8cfdcd2a60ce9d9b4ff7",
  "acceptedAt": "2026-11-12T19:27:00.997Z"
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
