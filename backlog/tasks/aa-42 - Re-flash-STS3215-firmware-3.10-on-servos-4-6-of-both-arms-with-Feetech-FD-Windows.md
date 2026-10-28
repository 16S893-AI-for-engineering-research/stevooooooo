---
id: AA-42
title: >-
  Re-flash STS3215 firmware 3.10 on servos 4-6 of both arms with Feetech FD
  (Windows)
status: To Do
assignee: []
created_date: '2026-10-28 13:35'
updated_date: '2026-10-28 13:35'
labels:
  - human-action
  - human-key-8d8f686b9ed669e1
dependencies: []
references:
  - AA-41
ordinal: 42000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-41

Why only you: Feetech's FD firmware tool runs only on Windows and is operated interactively. Flashing a servo carries a real risk of bricking it if power or USB is interrupted, so the owner must be at the robot to control power and cabling during each ~3 s update, and the owner chose the official Feetech route over an agent-run Linux flasher (2026-10-27). The agent cannot drive the Windows GUI or physically control power.

Alternatives considered: (1) komatachi-robotics/feetech-flash (Linux, same bytes as FD, sha256-pinned 3.10 image) could be run by the agent from WSL, but the owner explicitly chose the official Feetech FD route for safety. (2) Masking the error bit in our driver, or setting Protection_Current to 0, was considered; it would hide the fault rather than fix the bogus current baseline, and both are on hold. (3) A test move of servo 1 was proposed and deferred by the owner in favour of this. Everything else (backups, verification reads, diagnosis) the agent already did and will do afterwards.

Unlocks: Servos 4, 5 and 6 on both SO-101 arms re-flashed with the official Feetech STS3215 firmware via FD Online upgrade, each confirmed by FD as firmware 3.10 afterwards, with the dock reattached to WSL so the agent can re-read all 12 servos (current/temperature/error status and EEPROM).

Estimated active time: 35 minutes

Preparation: Windows laptop with Feetech FD installed (FD 1.9.8.3 from https://www.feetechrc.com/software.html, as in the LeRobot Feetech firmware guide) and internet access for FD's Online button. Robot USB dock connected to the SAME laptop USB port as before (BusIds must not change); both arms' 12 V supply available. Arms resting limp in a safe pose (torque is off on all servos). No other program may use the arm COM ports: the agent will not touch the arms during this task. Do NOT change any servo setting in FD other than the firmware upgrade (no ID, baud, offset, limit or parameter edits). Never cut 12 V or unplug USB while an upgrade is running.

## Procedure

1. Reconnect the robot USB dock to the same laptop USB port as before, with the arms' 12 V supply ON. Then, in a Windows PowerShell, run: usbipd detach --busid 10-4 and usbipd detach --busid 9-3 (left and right arm adapters), so Windows owns the two arm COM ports.
   - Expected: Both detach commands finish without error (a 'not attached' message is also fine).
   - Verify: Windows Device Manager > Ports (COM & LPT) shows two 'USB-Enhanced-SERIAL CH343' ports (previously COM3 and COM5).
   - Stop if: Stop if the CH343 ports do not appear, or if usbipd list shows different BusIds than 10-4 / 9-3 for the arm adapters.

2. Open Feetech FD. Select the COM port of the LEFT arm (start with the lower-numbered CH343 port; if the wrong arm answers, swap later), baud 1000000, click Open, then Search.
   - Expected: Six servos are found, IDs 1-6.
   - Verify: FD lists IDs 1, 2, 3, 4, 5, 6. Click IDs 1 and 4 and note the firmware version FD shows (expected 3.9 for ID 1 and 3.10 for ID 4).
   - Stop if: Stop if fewer or more than 6 servos appear, any ID is duplicated, or FD reports a communication error.

3. Stop the search. Select ID 4, open the Upgrade tab, click Online.
   - Expected: FD shows an available firmware image for this servo (expected version 3.10).
   - Verify: The firmware box names a file and version; write down the version shown.
   - Stop if: Stop if FD offers a version other than 3.10 or finds no image, and report what it shows. Do not upgrade to an unexpected version.

4. With ID 4 still selected, click Upgrade. Do not touch power, USB or the arm until it finishes.
   - Expected: The progress bar completes and FD reports success.
   - Verify: After FD refreshes, ID 4 is still listed and shows firmware 3.10.
   - Stop if: Stop immediately if the upgrade reports failure, the servo disappears from the list, or the progress stalls for more than a minute. Do NOT cut power; leave everything connected and report.

5. Repeat the Upgrade (Online, then Upgrade) for ID 5, then ID 6 on the left arm, one at a time.
   - Expected: Each completes successfully.
   - Verify: IDs 5 and 6 still appear and show 3.10. A fresh Search still finds exactly IDs 1-6.
   - Stop if: Same as the previous step; stop at the first failure.

6. Do not upgrade IDs 1-3. Close the port in FD. Select the RIGHT arm's COM port (the other CH343), Open, Search, and repeat the upgrade for IDs 4, 5 and 6, one at a time.
   - Expected: Six servos found on the right arm; each of IDs 4-6 upgrades successfully.
   - Verify: Right-arm IDs 4-6 show 3.10 after upgrade; a fresh Search finds exactly IDs 1-6.
   - Stop if: Stop at the first failure, missing servo or unexpected version, as above.

7. Close the port in FD and quit FD. Switch the arms' 12 V supply off for 10 s, then back on. Tell the orchestrator in the main Pi session that the firmware step is done; the agent re-attaches the USB devices to WSL itself (scripts/attach_usb.py).
   - Expected: FD is closed so WSL can claim the ports again.
   - Verify: Report the versions FD showed per servo and any warnings. The agent then reads all 12 servos (error status, current, temperature, EEPROM) to check the result.
   - Stop if: Stop if FD will not close or the ports stay in use.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Re-flash STS3215 firmware 3.10 on servos 4-6 of both arms with Feetech FD (Windows)",
  "parent": "AA-41",
  "reason": "Feetech's FD firmware tool runs only on Windows and is operated interactively. Flashing a servo carries a real risk of bricking it if power or USB is interrupted, so the owner must be at the robot to control power and cabling during each ~3 s update, and the owner chose the official Feetech route over an agent-run Linux flasher (2026-10-27). The agent cannot drive the Windows GUI or physically control power.",
  "alternatives": "(1) komatachi-robotics/feetech-flash (Linux, same bytes as FD, sha256-pinned 3.10 image) could be run by the agent from WSL, but the owner explicitly chose the official Feetech FD route for safety. (2) Masking the error bit in our driver, or setting Protection_Current to 0, was considered; it would hide the fault rather than fix the bogus current baseline, and both are on hold. (3) A test move of servo 1 was proposed and deferred by the owner in favour of this. Everything else (backups, verification reads, diagnosis) the agent already did and will do afterwards.",
  "outcome": "Servos 4, 5 and 6 on both SO-101 arms re-flashed with the official Feetech STS3215 firmware via FD Online upgrade, each confirmed by FD as firmware 3.10 afterwards, with the dock reattached to WSL so the agent can re-read all 12 servos (current/temperature/error status and EEPROM).",
  "preparation": "Windows laptop with Feetech FD installed (FD 1.9.8.3 from https://www.feetechrc.com/software.html, as in the LeRobot Feetech firmware guide) and internet access for FD's Online button. Robot USB dock connected to the SAME laptop USB port as before (BusIds must not change); both arms' 12 V supply available. Arms resting limp in a safe pose (torque is off on all servos). No other program may use the arm COM ports: the agent will not touch the arms during this task. Do NOT change any servo setting in FD other than the firmware upgrade (no ID, baud, offset, limit or parameter edits). Never cut 12 V or unplug USB while an upgrade is running.",
  "category": "physical-access",
  "minutes": 35,
  "steps": [
    {
      "action": "Reconnect the robot USB dock to the same laptop USB port as before, with the arms' 12 V supply ON. Then, in a Windows PowerShell, run: usbipd detach --busid 10-4 and usbipd detach --busid 9-3 (left and right arm adapters), so Windows owns the two arm COM ports.",
      "expected": "Both detach commands finish without error (a 'not attached' message is also fine).",
      "verification": "Windows Device Manager > Ports (COM & LPT) shows two 'USB-Enhanced-SERIAL CH343' ports (previously COM3 and COM5).",
      "stop": "Stop if the CH343 ports do not appear, or if usbipd list shows different BusIds than 10-4 / 9-3 for the arm adapters."
    },
    {
      "action": "Open Feetech FD. Select the COM port of the LEFT arm (start with the lower-numbered CH343 port; if the wrong arm answers, swap later), baud 1000000, click Open, then Search.",
      "expected": "Six servos are found, IDs 1-6.",
      "verification": "FD lists IDs 1, 2, 3, 4, 5, 6. Click IDs 1 and 4 and note the firmware version FD shows (expected 3.9 for ID 1 and 3.10 for ID 4).",
      "stop": "Stop if fewer or more than 6 servos appear, any ID is duplicated, or FD reports a communication error."
    },
    {
      "action": "Stop the search. Select ID 4, open the Upgrade tab, click Online.",
      "expected": "FD shows an available firmware image for this servo (expected version 3.10).",
      "verification": "The firmware box names a file and version; write down the version shown.",
      "stop": "Stop if FD offers a version other than 3.10 or finds no image, and report what it shows. Do not upgrade to an unexpected version."
    },
    {
      "action": "With ID 4 still selected, click Upgrade. Do not touch power, USB or the arm until it finishes.",
      "expected": "The progress bar completes and FD reports success.",
      "verification": "After FD refreshes, ID 4 is still listed and shows firmware 3.10.",
      "stop": "Stop immediately if the upgrade reports failure, the servo disappears from the list, or the progress stalls for more than a minute. Do NOT cut power; leave everything connected and report."
    },
    {
      "action": "Repeat the Upgrade (Online, then Upgrade) for ID 5, then ID 6 on the left arm, one at a time.",
      "expected": "Each completes successfully.",
      "verification": "IDs 5 and 6 still appear and show 3.10. A fresh Search still finds exactly IDs 1-6.",
      "stop": "Same as the previous step; stop at the first failure."
    },
    {
      "action": "Do not upgrade IDs 1-3. Close the port in FD. Select the RIGHT arm's COM port (the other CH343), Open, Search, and repeat the upgrade for IDs 4, 5 and 6, one at a time.",
      "expected": "Six servos found on the right arm; each of IDs 4-6 upgrades successfully.",
      "verification": "Right-arm IDs 4-6 show 3.10 after upgrade; a fresh Search finds exactly IDs 1-6.",
      "stop": "Stop at the first failure, missing servo or unexpected version, as above."
    },
    {
      "action": "Close the port in FD and quit FD. Switch the arms' 12 V supply off for 10 s, then back on. Tell the orchestrator in the main Pi session that the firmware step is done; the agent re-attaches the USB devices to WSL itself (scripts/attach_usb.py).",
      "expected": "FD is closed so WSL can claim the ports again.",
      "verification": "Report the versions FD showed per servo and any warnings. The agent then reads all 12 servos (error status, current, temperature, EEPROM) to check the result.",
      "stop": "Stop if FD will not close or the ports stay in use."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Servos 4, 5 and 6 on both SO-101 arms re-flashed with the official Feetech STS3215 firmware via FD Online upgrade, each confirmed by FD as firmware 3.10 afterwards, with the dock reattached to WSL so the agent can re-read all 12 servos (current/temperature/error status and EEPROM).
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
  "key": "8d8f686b9ed669e1e8cdc5b0ee8e97d3c7e0b6b08c8c35a337431bcc64d03189",
  "phase": "proposed",
  "createdAt": "2026-10-28T13:35:30.642Z",
  "origin": "01a17fcd-ad6a-7474-9ab6-a7b139b28469",
  "step": 0,
  "evidence": [],
  "history": []
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
