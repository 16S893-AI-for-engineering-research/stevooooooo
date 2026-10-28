---
id: AA-42
title: >-
  Re-flash STS3215 firmware 3.10 on servos 4-6 of both arms with Feetech FD
  (Windows)
status: In Progress
assignee:
  - '@human'
created_date: '2026-10-28 13:35'
updated_date: '2026-10-28 22:46'
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
  "key": "8d8f686b9ed669e1e8cdc5b0ee8e97d3c7e0b6b08c8c35a337431bcc64d03189",
  "phase": "active",
  "createdAt": "2026-10-28T13:35:30.642Z",
  "origin": "01a17fcd-ad6a-7474-9ab6-a7b139b28469",
  "step": 3,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 verified via pasted `usbipd list` output. Both arm adapters present at the expected BusIds with state \"Shared\" (i.e. not attached to WSL, so Windows owns them): 9-3 = 1a86:55d3 USB-Enhanced-SERIAL CH343 (COM3) and 10-4 = 1a86:55d3 CH343 (COM5). Same COM numbers (COM3/COM5) as the previous session and no BusId change, so the step-1 stop conditions do not apply. Noted discrepancy in the procedure text for later steps: the procedure labels 10-4 = left and 9-3 = right, which makes the LEFT arm COM5 and the RIGHT arm COM3, contradicting step 2's \"start with the lower-numbered port for the LEFT arm\". Scope is unaffected because IDs 4-6 are to be flashed on both arms; ports will be referred to by COM/BusId rather than by arm label to avoid mislabelling. Other shared devices (CH340 COM7, cameras, COM6) are unrelated to the arm buses. Not independently verified in Device Manager; usbipd's own COM assignment accepted as equivalent evidence.",
      "type": "self-reported",
      "reference": "User-pasted PowerShell output of `usbipd list` (conversation transcript), Connected section listing BusIds 9-3 (COM3) and 10-4 (COM5) as Shared",
      "advanced": true,
      "at": "2026-10-28T21:41:38.377Z"
    },
    {
      "step": 2,
      "note": "Step 2 partially done. Owner opened Feetech FD on COM3 (BusId 9-3) at baud 1000000 and reports \"worked, i see 6 servos\". Servo count matches the expected result and no communication error or duplicate-ID problem was reported. Not advanced: the step's stated verification also requires reading the firmware version of ID 1 (expected 3.9) and ID 4 (expected 3.10), which establishes the pre-flash baseline; owner has not reported those versions yet, nor the explicit ID list. Requested both before moving to the Upgrade tab. No write operations performed.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation transcript (\"worked, i see 6 servos\"); no FD screenshot or version readout supplied",
      "advanced": false,
      "at": "2026-10-28T21:42:47.926Z"
    },
    {
      "step": 2,
      "note": "Step 2 complete. FD open on COM3 (BusId 9-3) at baud 1000000; Search found 6 servos. Baseline firmware read as FD's split main/secondary fields: ID 1 main 3 / secondary 9 = 3.9, ID 4 main 3 / secondary 10 = 3.10. Both match the procedure's expected baseline, so no stop condition. Owner did not transcribe the full ID list digit by digit beyond \"6 servos\" and reported no duplicate or comms error; accepted as matching IDs 1-6. Observation for later interpretation: ID 4 already reports 3.10 pre-flash, so the re-flash rewrites the same official version rather than changing the version string; a post-flash reading of 3.10 is therefore NOT by itself evidence the bogus current baseline is fixed. The decisive check remains the agent's post-flash read of current/temperature/error status/EEPROM on all 12 servos. No writes performed in this step.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation transcript: FD firmware main version 3 for ID 1 and ID 4, secondary versions 9 and 10 respectively; 6 servos found on COM3",
      "advanced": true,
      "at": "2026-10-28T21:44:44.948Z"
    },
    {
      "step": 3,
      "note": "Step 3 complete. Search stopped, ID 4 selected on COM3 (BusId 9-3), Upgrade tab opened, Online clicked. Owner reports FD offers firmware version 3.10, matching the procedure's expected version, so the stop condition (unexpected version or no image found) does not apply and the upgrade is cleared to proceed. Owner reported the version only and did not transcribe the image file name, so the file identity is unverified beyond FD's version string; the version was the decisive safety gate. No Upgrade click performed in this step; no servo settings written.",
      "type": "self-reported",
      "reference": "Owner verbal report in conversation transcript: FD Online firmware box shows version \"3.10\" for ID 4; file name not transcribed",
      "advanced": true,
      "at": "2026-10-28T21:46:50.935Z"
    },
    {
      "step": 4,
      "note": "Step 4 action done, verification incomplete. Owner clicked Upgrade for ID 4 on COM3 (BusId 9-3); progress bar reached 100% but FD showed no completion popup. Owner confirms FD 1.9.8.3 has no status text field in the Upgrade tab, so absence of a popup is treated as unremarkable UI behaviour rather than evidence of failure. ID 4 answers after the flash (rules out a bricked servo, the main risk). Owner subsequently reports all three servos (IDs 4, 5 and 6) were reflashed, i.e. the step-5 actions were performed too, but has supplied NO post-flash firmware readback for any of the three and no fresh ID-list confirmation. Not advanced: both step 4 and step 5 verify on \"ID still listed and shows firmware 3.10\" (FD main 3 / secondary 10). Requested main/secondary versions for IDs 4, 5, 6 plus a fresh Search count. Separately, owner asked to test-move the servos from FD; declined as out of scope (the procedure's preparation forbids any FD write other than the firmware upgrade, and FD jog controls write goal position and enable torque; a servo-1 test move was previously deferred by the owner). Directed to the orchestrator for renewed acceptance if a powered move test is wanted. No motion commanded, no settings written.",
      "type": "self-reported",
      "reference": "Owner verbal reports in conversation transcript: progress reached 100%, no popup, no status text in FD, \"id4 answers\", \"all three servos were reflashed\"; no version readback or screenshot supplied",
      "advanced": false,
      "at": "2026-10-28T21:51:49.096Z"
    }
  ],
  "history": [
    {
      "action": "accept",
      "at": "2026-10-28T16:11:53.923Z",
      "hash": "3419bdcae0a19a3637bb37ad5716a18e3c3da7d716854723dc41e03d29e821b0"
    }
  ],
  "acceptedHash": "3419bdcae0a19a3637bb37ad5716a18e3c3da7d716854723dc41e03d29e821b0",
  "acceptedAt": "2026-10-28T16:11:53.926Z"
}
```
<!-- /human-handoff-state -->

ROOT CAUSE (owner, 2026-10-27): servos ID 4-6 on both arms are encoder-only leader-arm servos with no motor drive. That explains the zeroed drive EEPROM, temperature 0 C, constant ~950 current and the 0x08 flag; nothing was fried and no software fault. Powered wrist/gripper motion is impossible until they are replaced (AA-43). The EEPROM values written 2026-10-27 to IDs 4-6 are harmless on these parts (backups in calibration_data/eeprom_*2026-10-27T23-*); they are moot once the servos are swapped. The re-flash is therefore moot: it cannot add a motor driver. Left-arm IDs 4-6 were re-flashed (3.10 over 3.10; ID 4 answered afterwards) before this was known. The owner should close this handoff from the walkthrough tab; the orchestrator will not mark it verified.
<!-- SECTION:NOTES:END -->
