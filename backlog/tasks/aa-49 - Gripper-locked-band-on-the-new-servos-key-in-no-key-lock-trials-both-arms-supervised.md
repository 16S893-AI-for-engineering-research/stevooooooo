---
id: AA-49
title: >-
  Gripper locked band on the new servos: key-in / no-key lock trials (both arms,
  supervised)
status: To Do
assignee: []
created_date: '2026-11-12 19:13'
updated_date: '2026-11-12 19:13'
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

Why only you: AA-43 AC #4 requires the gripper bands to be re-measured on the new servos. The locked band can only be measured with the real locking key inserted into the gripper by hand, between powered trials, while someone watches for binding and keeps a hand at the power switch. These are the first powered lock motions on the new servos, so the owner must supervise them (AGENTS.md rule 5).

Alternatives considered: Software is done and dry-run tested. scripts/gripper_lock_probe.py commands the gripper only through the AA-17 state machine. It now moves at 150 ticks/s, acceleration 20 and the candidate torque limit (the profile that drove both new grippers end to end on 2026-11-12), all set before torque is enabled. It classifies only settled positions, caps torque at the EEPROM 500, and switches torque off and restores the RAM registers on every exit. The 2026-10-25 locked samples (L 1801-1802, R 1792-1801 ticks) come from the old servos with a different horn offset. AA-41's procedure is stale (old torque range, old full-speed lock path). Mocks cannot tell where the key seats or how much torque the jaw needs.

Unlocks: robot/calibration_data/gripper_lock_probe_new_servos.jsonl holds, for each arm: 3 no-key lock trials at torque 300 (settled position, load), and key-in trials at increasing torque (300, then 400, then 500 if needed) until 3/3 settle with the key seated, with the owner's word on whether the key was really locked. From this the agent proposes the locked band and lock_torque_limit_raw per arm, for owner approval (AA-43 AC #4, AA-17 AC #1).

Estimated active time: 30 minutes

Preparation: Robot powered as in the 2026-11-12 gripper-ends session. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; others may be MISSING). No tube attached. Both arms limp (torque off) in their rest pose; the grippers are currently closed (L 2526, R 2499 ticks) and the probe opens them first. The locking key at hand. A hand within reach of the arm power switch. Abort at any time with the power switch or Ctrl-C (cleanup switches torque off). Run every command from ~/893/robot. About 30 minutes.

## Procedure

1. Dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --no-key
   - Expected: Prints a plan for arm_left at torque 300, lock goal 191.07 deg, locked band [156.09, 160.58] deg, 150 ticks/s, then DRY-RUN. Nothing moves.
   - Verify: No servo noise or motion; exit without error.
   - Stop if: Any motion, or an error instead of the plan.

2. Left, NO key: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --no-key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt make sure the gripper is empty, hands clear, press Enter.
   - Expected: Per trial the gripper opens slowly, then closes slowly past where the key would sit and stops, then reopens. Ends with SUMMARY: 3/3 (classification overtravel_past_locked_band).
   - Verify: SUMMARY line, plus one JSON line per trial with settled true.
   - Stop if: Grinding, buzzing for more than ~2 s, a hot servo, any OTHER joint moving, a REFUSED or WARNING line, or a classification of short_of_locked_band / in_locked_band / not_settled. Stop and report it; do not raise the torque on your own.

3. Left, key IN: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt insert the key into the open gripper as in the 2026-10-25 session, take your hands away, press Enter. While the gripper is closed on the key, look and listen: is the key seated and held? Do not pull on it.
   - Expected: Per trial the gripper closes onto the key, stops with the key seated, then reopens. SUMMARY: 3/3 (in_locked_band).
   - Verify: SUMMARY line; your own look at whether the key was seated each time.
   - Stop if: Same as step 2, or the key looks forced or deformed, or the key is NOT seated but the script says in_locked_band (tell the orchestrator).

4. If step 3 was below 3/3 and the key was not forced, repeat step 3 with --torque 400, then with --torque 500 if still below 3/3. Do not go above 500 (the script refuses anyway).
   - Expected: At some torque all 3 key-in trials report in_locked_band with the key seated.
   - Verify: SUMMARY: 3/3 at that torque.
   - Stop if: Still below 3/3 at 500, or the key looks forced or deformed. Stop and report.

5. If the passing torque from steps 3-4 is above 300, run the no-key check at that torque: the step 2 command with --torque <passing torque>.
   - Expected: SUMMARY: 3/3 overtravel_past_locked_band.
   - Verify: SUMMARY line.
   - Stop if: Same as step 2.

6. Right arm: repeat steps 1-5 with --arm right. Its plan shows lock goal 189.84 deg and locked band [155.3, 160.49] deg.
   - Expected: Same outcomes as for the left arm.
   - Verify: SUMMARY lines for no-key and key-in on the right arm.
   - Stop if: Same stop conditions as steps 1-5.

7. Both grippers end OPEN with torque off. If you want them closed at rest, for each arm (key removed): uv run python scripts/gripper_ends.py --arm left --action close --no-tube --live, then the same with --arm right.
   - Expected: Each gripper closes slowly to its closed position; RESULT: OK; torque off afterwards.
   - Verify: RESULT: OK and torque_after all 0 in the printed line.
   - Stop if: RESULT: NOT OK, a FAULT, or anything in the jaws.

8. Tell the orchestrator you are done: the passing key-in torque per arm, whether the key was seated in every key-in trial, and anything you saw or heard (stalls, noise, heat).
   - Expected: calibration_data/gripper_lock_probe_new_servos.jsonl has at least 12 lines covering both arms.
   - Verify: The orchestrator reads the log and checks the counts, settled flags and positions.
   - Stop if: If a run failed or a file is missing, report which step.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Gripper locked band on the new servos: key-in / no-key lock trials (both arms, supervised)",
  "parent": "AA-43",
  "reason": "AA-43 AC #4 requires the gripper bands to be re-measured on the new servos. The locked band can only be measured with the real locking key inserted into the gripper by hand, between powered trials, while someone watches for binding and keeps a hand at the power switch. These are the first powered lock motions on the new servos, so the owner must supervise them (AGENTS.md rule 5).",
  "alternatives": "Software is done and dry-run tested. scripts/gripper_lock_probe.py commands the gripper only through the AA-17 state machine. It now moves at 150 ticks/s, acceleration 20 and the candidate torque limit (the profile that drove both new grippers end to end on 2026-11-12), all set before torque is enabled. It classifies only settled positions, caps torque at the EEPROM 500, and switches torque off and restores the RAM registers on every exit. The 2026-10-25 locked samples (L 1801-1802, R 1792-1801 ticks) come from the old servos with a different horn offset. AA-41's procedure is stale (old torque range, old full-speed lock path). Mocks cannot tell where the key seats or how much torque the jaw needs.",
  "outcome": "robot/calibration_data/gripper_lock_probe_new_servos.jsonl holds, for each arm: 3 no-key lock trials at torque 300 (settled position, load), and key-in trials at increasing torque (300, then 400, then 500 if needed) until 3/3 settle with the key seated, with the owner's word on whether the key was really locked. From this the agent proposes the locked band and lock_torque_limit_raw per arm, for owner approval (AA-43 AC #4, AA-17 AC #1).",
  "preparation": "Robot powered as in the 2026-11-12 gripper-ends session. In a WSL terminal: cd ~/893/robot && uv run python scripts/attach_usb.py (arm ports must attach; others may be MISSING). No tube attached. Both arms limp (torque off) in their rest pose; the grippers are currently closed (L 2526, R 2499 ticks) and the probe opens them first. The locking key at hand. A hand within reach of the arm power switch. Abort at any time with the power switch or Ctrl-C (cleanup switches torque off). Run every command from ~/893/robot. About 30 minutes.",
  "category": "physical-access",
  "minutes": 30,
  "steps": [
    {
      "action": "Dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --no-key",
      "expected": "Prints a plan for arm_left at torque 300, lock goal 191.07 deg, locked band [156.09, 160.58] deg, 150 ticks/s, then DRY-RUN. Nothing moves.",
      "verification": "No servo noise or motion; exit without error.",
      "stop": "Any motion, or an error instead of the plan."
    },
    {
      "action": "Left, NO key: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --no-key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt make sure the gripper is empty, hands clear, press Enter.",
      "expected": "Per trial the gripper opens slowly, then closes slowly past where the key would sit and stops, then reopens. Ends with SUMMARY: 3/3 (classification overtravel_past_locked_band).",
      "verification": "SUMMARY line, plus one JSON line per trial with settled true.",
      "stop": "Grinding, buzzing for more than ~2 s, a hot servo, any OTHER joint moving, a REFUSED or WARNING line, or a classification of short_of_locked_band / in_locked_band / not_settled. Stop and report it; do not raise the torque on your own."
    },
    {
      "action": "Left, key IN: uv run python scripts/gripper_lock_probe.py --arm left --torque 300 --key --live --trials 3 --out calibration_data/gripper_lock_probe_new_servos.jsonl. At each prompt insert the key into the open gripper as in the 2026-10-25 session, take your hands away, press Enter. While the gripper is closed on the key, look and listen: is the key seated and held? Do not pull on it.",
      "expected": "Per trial the gripper closes onto the key, stops with the key seated, then reopens. SUMMARY: 3/3 (in_locked_band).",
      "verification": "SUMMARY line; your own look at whether the key was seated each time.",
      "stop": "Same as step 2, or the key looks forced or deformed, or the key is NOT seated but the script says in_locked_band (tell the orchestrator)."
    },
    {
      "action": "If step 3 was below 3/3 and the key was not forced, repeat step 3 with --torque 400, then with --torque 500 if still below 3/3. Do not go above 500 (the script refuses anyway).",
      "expected": "At some torque all 3 key-in trials report in_locked_band with the key seated.",
      "verification": "SUMMARY: 3/3 at that torque.",
      "stop": "Still below 3/3 at 500, or the key looks forced or deformed. Stop and report."
    },
    {
      "action": "If the passing torque from steps 3-4 is above 300, run the no-key check at that torque: the step 2 command with --torque <passing torque>.",
      "expected": "SUMMARY: 3/3 overtravel_past_locked_band.",
      "verification": "SUMMARY line.",
      "stop": "Same as step 2."
    },
    {
      "action": "Right arm: repeat steps 1-5 with --arm right. Its plan shows lock goal 189.84 deg and locked band [155.3, 160.49] deg.",
      "expected": "Same outcomes as for the left arm.",
      "verification": "SUMMARY lines for no-key and key-in on the right arm.",
      "stop": "Same stop conditions as steps 1-5."
    },
    {
      "action": "Both grippers end OPEN with torque off. If you want them closed at rest, for each arm (key removed): uv run python scripts/gripper_ends.py --arm left --action close --no-tube --live, then the same with --arm right.",
      "expected": "Each gripper closes slowly to its closed position; RESULT: OK; torque off afterwards.",
      "verification": "RESULT: OK and torque_after all 0 in the printed line.",
      "stop": "RESULT: NOT OK, a FAULT, or anything in the jaws."
    },
    {
      "action": "Tell the orchestrator you are done: the passing key-in torque per arm, whether the key was seated in every key-in trial, and anything you saw or heard (stalls, noise, heat).",
      "expected": "calibration_data/gripper_lock_probe_new_servos.jsonl has at least 12 lines covering both arms.",
      "verification": "The orchestrator reads the log and checks the counts, settled flags and positions.",
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
<!-- human-handoff-state:v1 -->
```json
{
  "version": 1,
  "key": "d9694f3656b1cc6fcfab655b8b37a8ea0197c21c1991efbca3c45e456901c91d",
  "phase": "proposed",
  "createdAt": "2026-11-12T19:13:16.087Z",
  "origin": "01a1d101-b8e5-7496-a5a0-f4e66bdd6498",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
