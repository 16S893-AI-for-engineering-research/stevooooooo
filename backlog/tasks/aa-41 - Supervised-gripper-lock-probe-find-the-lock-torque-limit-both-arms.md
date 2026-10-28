---
id: AA-41
title: 'Supervised gripper lock probe: find the lock torque limit (both arms)'
status: In Progress
assignee:
  - '@human'
created_date: '2026-10-27 20:23'
updated_date: '2026-10-28 13:35'
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
  "key": "0bb0fd9f151c322977c1e18a0e1b8ae54336a755edd3dfab34ba499c59ea4e23",
  "phase": "paused",
  "createdAt": "2026-10-27T20:23:10.960Z",
  "origin": "01a17eda-abe6-765f-b277-eb0952f11682",
  "step": 1,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 dry-run executed: `gripper_lock_probe.py --arm left --torque 150 --key`. Pasted terminal output matches expected plan exactly (arm_left, torque_limit_raw=150, key=in, lock goal 191.07 deg, locked band [156.09, 160.58] deg) and ended with DRY-RUN line, no port opened, no motion. Clean exit, no error.",
      "type": "self-reported",
      "reference": "User-pasted stdout of scripts/gripper_lock_probe.py dry run (conversation transcript)",
      "advanced": true,
      "at": "2026-10-27T23:45:06.685Z"
    },
    {
      "step": 2,
      "note": "Step 2 live run at torque 150, left arm, key in: SUMMARY 0/3. Owner reports the servo did not move at all during any trial. All 3 JSON lines: sm_outcome \"LockVerificationFailed: LockVerdict.UNVERIFIED\", classification short_of_locked_band, position 135.44 deg (ticks 1541) on trials 1-2 and 139.48 deg (1587) on trial 3, load_raw 0, speed_raw 0, temperature_c 0, current_raw 1001 constant. No WARNING cleanup line and no refusal message. Deviation: zero motion plus zero load/speed and an implausible temperature_c of 0 suggests the gripper was never actually driven (torque enable / comms / telemetry path) rather than insufficient torque, so step 3's torque escalation to 250/350 was NOT attempted. Instruction sequence paused pending orchestrator diagnosis.",
      "type": "self-reported",
      "reference": "User-pasted stdout of scripts/gripper_lock_probe.py --live run, 3 JSON-lines + SUMMARY (conversation transcript); log file calibration_data/gripper_lock_probe_2026-10-27.jsonl",
      "advanced": false,
      "at": "2026-10-27T23:48:01.037Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-10-27T23:39:58.714Z",
      "hash": "aa4d7da9bda03c27e0d30d08002810b875956d2baf240aa5d4ed69079c6be79b"
    }
  ],
  "acceptedHash": "aa4d7da9bda03c27e0d30d08002810b875956d2baf240aa5d4ed69079c6be79b",
  "acceptedAt": "2026-10-27T23:39:58.714Z"
}
```
<!-- /human-handoff-state -->

2026-10-27 orchestrator DIAGNOSIS of the AA-41 step-2 result (zero motion at torque 150, UNVERIFIED x3, load/speed/temp 0, current 1001). Read-only register dump of all 12 servos (no torque, no writes): on BOTH arms, IDs 4-6 (wrist_flex, wrist_roll, gripper; firmware 3.10) have their drive configuration ZEROED in EEPROM: Max_Torque_Limit(16)=0, P(21)=0, D(22)=0, Protection_Current(28)=0, Protection_Torque(34)=0, Overload_Torque(36)=0, Max_Temp(13)=0, Unloading_Condition(19)=1, LED_Alarm(20)=1; Torque_Limit(48) 0 or 1000. IDs 1-3 (firmware 3.9) are normal (Max_Torque 1000, P 32, D 32, Prot_Current 310, Max_Temp 70). Temperature reads 0 C and current ~1000 constant on exactly those servos, so the earlier AA-8 '0 C / ~930 raw at rest' observation was this, not a telemetry bug. Conclusion: with Max_Torque_Limit=0 and P=0 the servo cannot produce torque, so torque enable + goal writes produced no motion. Not insufficient torque, not wiring (comms work: position reads track, goal register holds the commanded value), not the AA-17 code. Unknown: whether 3.10 firmware stores these elsewhere or they were zeroed by an earlier write; our tools wrote only homing offset, min/max position limits, torque enable/limit, goal, speed and acceleration (all writes of addresses 9-12, 31-32, 40-48). This also means wrist_flex and wrist_roll on both arms cannot move under power: any powered motion of joints 4-5 would silently fail. Safeguard added: gripper_lock_probe.py now reads the drive registers and REFUSES (rc 5) if Max_Torque/P/Protection_Current are 0 (test test_drive_config_faults_flags_zeroed_eeprom; verified live: refuses on arm_left). Steps 3+ remain paused. Renewed procedure needs an owner decision: restore those EEPROM values on IDs 4-6 (proposal: copy IDs 1-3 values: Max_Torque 1000, P 32, D 32, I 0, Protection_Current 310, Prot_Torque 20, Prot_Time 200, Overload_Torque 80, Max_Temp 70, Unloading 44, LED 47; for the gripper LeRobot's default caps: Max_Torque 500, Protection_Current 250, Overload_Torque 25), with an EEPROM backup and read-back, torque off, and a tiny supervised test move.

2026-10-27 LIVE EEPROM drive-config restore (owner-approved proposal), torque OFF throughout, nothing moved. scripts/restore_drive_config.py (tests/test_restore_drive_config.py): full register dump 0-70 of all 6 servos saved before and after (calibration_data/eeprom_backup_{left,right}_2026-10-27T23-01-*_pre_drive_restore.json, eeprom_after_*), EEPROM unlock/write/relock per servo, every value read back. Both arms OK: IDs 4-5 = ID 3 values (Max_Torque 1000, P 32, D 32, I 0, Protection_Current 310, Max_Temp 70, Unloading 44, LED 47, Prot_Torque 20, Prot_Time 200, Overload 80, ...); ID 6 = same with LeRobot gripper caps (Max_Torque 500, Protection_Current 250, Overload 25, P 16, D 32). No other register changed. NEW FINDING right after: IDs 4-6 on both arms now return status error 0x08 (OverEle = overcurrent, Feetech SDK ERRBIT_OVERELE) on every read, torque off. Cause: their present_current reads a constant ~1000 and temperature 0 even with torque off (bogus sensor/feedback on firmware 3.10; IDs 1-3 read current 0 and 30-32 C). Before the restore, Unloading/LED condition was 1 (voltage only) so no flag; now 44 includes overcurrent (bit 8), so the servo flags overcurrent permanently and will unload torque. Effect now: our driver raises ServoStatusError on any read of IDs 4-6 (read_pose/read_all_joint_states fail); powered motion of IDs 4-6 would be unloaded. STOPPED, no further writes. Owner decision needed: (a) set Unloading/LED condition on IDs 4-6 to exclude overcurrent+overheat (e.g. 36 / 39, keeping voltage and overload), since those sensors read bogus on 3.10; (b) revert IDs 4-6 fully to the backup; or (c) investigate the 3.10 current/temperature feedback first (e.g. Feetech FD software).

2026-10-27 owner decision (a) applied LIVE, torque off, nothing moved: IDs 4-6 on both arms Unloading_Condition 44->33 (voltage 1 + overload 32; overcurrent 8 and overheat 4 masked), LED_Alarm 47->35 (voltage, angle, overload). (Correction: an earlier note proposed 36/39; those values were wrong bit sums.) Read back OK, nothing else changed; backups calibration_data/eeprom_*2026-10-27T23-32*. restore_drive_config.py --fw310-mask uses a read path that tolerates ONLY error bit 0x08 (checksum and other bits still raise), script-local. RESULT: status register (65) still reads 8 and every reply still carries error 0x08 on IDs 4-6, so the main driver still raises on reads. Either the flag is latched until power-off, or the servo reports it regardless of the unloading mask. Next: owner power-cycles both arms (12 V off/on), then a read-only status check. If the flag persists, owner decision: let the driver accept 0x08 on the firmware-3.10 servos (would weaken a fault check, so not done without approval).

2026-10-27 after the owner's 12 V power cycle of both arms (USB re-attached with attach_usb.py, 7/7): read-only check. IDs 1-3 both arms: error 0, status 0, temp 30-32 C, current 0. IDs 4-6 both arms (fw 3.10): error 0x08 / status 8 PERSISTS, temp 0 C, current 948-953 raw with torque OFF (was ~1000 before the cycle). Restored EEPROM survived the cycle (maxT 1000/500, P/D, prot current, unloading/LED 33/35). Conclusion: the OverEle flag is reported independently of the unloading mask and is driven by the bogus current reading; it is not latched. Also: these servos have Max_Voltage 16.0 V vs 14.0 V on IDs 1-3, so IDs 4-6 are probably a different STS3215 hardware variant. Next needs an owner decision: let the driver accept error bit 0x08 ONLY on firmware-3.10 servos 4-6 (all other bits still raise), so they can be read and a supervised small move can test whether they move under torque at all.

2026-10-27 web research (no definitive answer): (1) firmware 3.10 is the current STS3215 image on Feetech's server (komatachi-robotics/feetech-flash pins SCServo21-GD32-TTL-250306.bin as 3.10 for model key 9.3); 3.9->3.10 updates are common, so IDs 4-6 were probably updated at some point. (2) The 0o8o0-blip clean-room STS3215 firmware computes Present_Current as ADC minus a power-on warm-up baseline; a bad baseline gives a constant false idle current and false overcurrent, matching our ~950 at rest. In that firmware the OverEle flag is cleared only when Protection_Current is 0 and it also limits PWM output when measured current exceeds Protection_Current, so the false reading could block torque. Not confirmed for factory 3.10. Owner decision: take the official Feetech route: re-flash 3.10 on IDs 4-6 of both arms with FD (Windows). Proposed human task AA-42; the owner declined a servo-1 test move for now.
<!-- SECTION:NOTES:END -->
