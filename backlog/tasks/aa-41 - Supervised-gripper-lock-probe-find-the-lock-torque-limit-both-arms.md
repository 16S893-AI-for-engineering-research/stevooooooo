---
id: AA-41
title: 'Supervised gripper lock probe: find the lock torque limit (both arms)'
status: To Do
assignee: []
created_date: '2026-10-27 20:23'
updated_date: '2026-10-27 20:23'
labels:
  - human-action
  - human-key-0bb0fd9f151c3229
dependencies: []
references:
  - AA-17
ordinal: 41000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-17

Why only you: The lock torque limit can only be found by energising the real gripper servo with the real locking key inserted and removed by hand, while a person watches and listens for binding or stalling. Nobody else can insert the key, and the first powered gripper motion must be owner-supervised (AGENTS.md rule 5).

Alternatives considered: Software is done: the lock path, a bounded settle wait before readback (fixes the false-MISS finding), and a dry-run-default probe script (scripts/gripper_lock_probe.py) that commands the gripper only through the AA-17 state machine, refuses if any arm joint has torque on, and restores torque limit 1000 and torque off on every exit. Mocks cannot tell how much torque seats this key, and the earlier torque-off sampling session gave positions only, not stall behaviour under a torque limit.

Unlocks: A JSON-lines log robot/calibration_data/gripper_lock_probe_2026-10-27.jsonl for each arm holding: the lowest tested torque at which 3/3 key-in trials settle inside the locked band, plus 3 no-key trials at that torque that run past the band. The agent then sets lock_torque_limit_raw and the latch load/current thresholds from the data.

Estimated active time: 30 minutes

Preparation: Robot powered as in the 2026-10-25 band session, USB devices attached to WSL (cd ~/893/robot && uv run python scripts/attach_usb.py). No tube attached. Both arms resting limp (torque off) in their rest/fold pose. The locking key at hand. Hand within reach of the arm power switch. Run every command from ~/893/robot in a WSL terminal.

## Procedure

1. Run the dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 150 --key
   - Expected: Prints a plan with lock goal 191.07 deg and locked band [156.09, 160.58] deg, then DRY-RUN. Nothing moves.
   - Verify: No servo noise or motion; exit without error.
   - Stop if: Any motion, or an error instead of the plan.

2. Left arm, key in, low torque: uv run python scripts/gripper_lock_probe.py --arm left --torque 150 --key --live --trials 3 --out calibration_data/gripper_lock_probe_2026-10-27.jsonl. At each prompt, insert the key into the open gripper, take your hands away, press Enter.
   - Expected: Per trial the gripper opens, closes onto the key and stops, then opens again. Ends with SUMMARY: n/3.
   - Verify: Each trial prints a JSON line; classification is in_locked_band when the key seated.
   - Stop if: Grinding, buzzing for more than ~2 s, a hot servo, any OTHER joint moving, a WARNING cleanup line, or a refusal message. Power the arm off if it stays energised.

3. If the SUMMARY was below 3/3, repeat step 2 with --torque 250, then 350 if still below 3/3. Do not go above 350.
   - Expected: At some torque, all 3 trials report in_locked_band.
   - Verify: SUMMARY: 3/3 trials as expected.
   - Stop if: Still below 3/3 at 350, or the key looks forced or deformed. Stop and report instead.

4. Left arm, NO key, at the torque that passed: uv run python scripts/gripper_lock_probe.py --arm left --torque <T> --no-key --live --trials 3 --out calibration_data/gripper_lock_probe_2026-10-27.jsonl. Keep the gripper empty and hands clear at each prompt.
   - Expected: The gripper closes past the key position and stops near the overtravel goal, then reopens.
   - Verify: SUMMARY: 3/3; classification overtravel_past_locked_band.
   - Stop if: The gripper jams, hits something, or reports in_locked_band with no key.

5. Repeat steps 2-4 for the right arm (--arm right); its locked band is [155.30, 160.49] deg and its lock goal 189.84 deg.
   - Expected: Same outcomes as for the left arm.
   - Verify: SUMMARY lines 3/3 for key-in and no-key on the right arm.
   - Stop if: Same stop conditions as steps 2-4.

6. Leave both arms powered or off as you prefer, then tell the orchestrator the passing torque for each arm, plus anything you saw or heard.
   - Expected: The log file has 12 or more lines covering both arms.
   - Verify: The orchestrator reads calibration_data/gripper_lock_probe_2026-10-27.jsonl.
   - Stop if: The file is missing or empty.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Supervised gripper lock probe: find the lock torque limit (both arms)",
  "parent": "AA-17",
  "reason": "The lock torque limit can only be found by energising the real gripper servo with the real locking key inserted and removed by hand, while a person watches and listens for binding or stalling. Nobody else can insert the key, and the first powered gripper motion must be owner-supervised (AGENTS.md rule 5).",
  "alternatives": "Software is done: the lock path, a bounded settle wait before readback (fixes the false-MISS finding), and a dry-run-default probe script (scripts/gripper_lock_probe.py) that commands the gripper only through the AA-17 state machine, refuses if any arm joint has torque on, and restores torque limit 1000 and torque off on every exit. Mocks cannot tell how much torque seats this key, and the earlier torque-off sampling session gave positions only, not stall behaviour under a torque limit.",
  "outcome": "A JSON-lines log robot/calibration_data/gripper_lock_probe_2026-10-27.jsonl for each arm holding: the lowest tested torque at which 3/3 key-in trials settle inside the locked band, plus 3 no-key trials at that torque that run past the band. The agent then sets lock_torque_limit_raw and the latch load/current thresholds from the data.",
  "preparation": "Robot powered as in the 2026-10-25 band session, USB devices attached to WSL (cd ~/893/robot && uv run python scripts/attach_usb.py). No tube attached. Both arms resting limp (torque off) in their rest/fold pose. The locking key at hand. Hand within reach of the arm power switch. Run every command from ~/893/robot in a WSL terminal.",
  "category": "physical-access",
  "minutes": 30,
  "steps": [
    {
      "action": "Run the dry-run: uv run python scripts/gripper_lock_probe.py --arm left --torque 150 --key",
      "expected": "Prints a plan with lock goal 191.07 deg and locked band [156.09, 160.58] deg, then DRY-RUN. Nothing moves.",
      "verification": "No servo noise or motion; exit without error.",
      "stop": "Any motion, or an error instead of the plan."
    },
    {
      "action": "Left arm, key in, low torque: uv run python scripts/gripper_lock_probe.py --arm left --torque 150 --key --live --trials 3 --out calibration_data/gripper_lock_probe_2026-10-27.jsonl. At each prompt, insert the key into the open gripper, take your hands away, press Enter.",
      "expected": "Per trial the gripper opens, closes onto the key and stops, then opens again. Ends with SUMMARY: n/3.",
      "verification": "Each trial prints a JSON line; classification is in_locked_band when the key seated.",
      "stop": "Grinding, buzzing for more than ~2 s, a hot servo, any OTHER joint moving, a WARNING cleanup line, or a refusal message. Power the arm off if it stays energised."
    },
    {
      "action": "If the SUMMARY was below 3/3, repeat step 2 with --torque 250, then 350 if still below 3/3. Do not go above 350.",
      "expected": "At some torque, all 3 trials report in_locked_band.",
      "verification": "SUMMARY: 3/3 trials as expected.",
      "stop": "Still below 3/3 at 350, or the key looks forced or deformed. Stop and report instead."
    },
    {
      "action": "Left arm, NO key, at the torque that passed: uv run python scripts/gripper_lock_probe.py --arm left --torque <T> --no-key --live --trials 3 --out calibration_data/gripper_lock_probe_2026-10-27.jsonl. Keep the gripper empty and hands clear at each prompt.",
      "expected": "The gripper closes past the key position and stops near the overtravel goal, then reopens.",
      "verification": "SUMMARY: 3/3; classification overtravel_past_locked_band.",
      "stop": "The gripper jams, hits something, or reports in_locked_band with no key."
    },
    {
      "action": "Repeat steps 2-4 for the right arm (--arm right); its locked band is [155.30, 160.49] deg and its lock goal 189.84 deg.",
      "expected": "Same outcomes as for the left arm.",
      "verification": "SUMMARY lines 3/3 for key-in and no-key on the right arm.",
      "stop": "Same stop conditions as steps 2-4."
    },
    {
      "action": "Leave both arms powered or off as you prefer, then tell the orchestrator the passing torque for each arm, plus anything you saw or heard.",
      "expected": "The log file has 12 or more lines covering both arms.",
      "verification": "The orchestrator reads calibration_data/gripper_lock_probe_2026-10-27.jsonl.",
      "stop": "The file is missing or empty."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A JSON-lines log robot/calibration_data/gripper_lock_probe_2026-10-27.jsonl for each arm holding: the lowest tested torque at which 3/3 key-in trials settle inside the locked band, plus 3 no-key trials at that torque that run past the band. The agent then sets lock_torque_limit_raw and the latch load/current thresholds from the data.
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
  "key": "0bb0fd9f151c322977c1e18a0e1b8ae54336a755edd3dfab34ba499c59ea4e23",
  "phase": "proposed",
  "createdAt": "2026-10-27T20:23:10.960Z",
  "origin": "01a17eda-abe6-765f-b277-eb0952f11682",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
