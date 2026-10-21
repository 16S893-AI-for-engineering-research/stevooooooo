---
id: AA-37
title: Connect robot USB dock and admin-bind all devices for WSL
status: In Progress
assignee:
  - '@human'
created_date: '2026-10-20 21:15'
updated_date: '2026-10-21 01:57'
labels:
  - human-action
  - human-key-82c2d849e0ff6e02
dependencies: []
references:
  - AA-5
ordinal: 37000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Originating task: AA-5

Why only you: Two separate things only you can do. (1) Physical: `usbipd state` shows both SO-101 CH343 adapters as persisted but with no BusId, meaning they are not currently plugged in, and the DexArm, the three OV9281 cameras and the USB relay do not appear at all. Someone has to physically connect and power the dock. (2) Permission: `usbipd bind` requires an elevated Windows administrator console. I verified this directly, not by assumption: `usbipd attach --wsl --busid 2-2` returned "Device is not shared; run 'usbipd bind --busid 2-2' as administrator first". I cannot elevate from this session. Bind is one-time per device and persists, so this is not a recurring request.

Alternatives considered: I ran `usbipd state`, `usbipd list` and `usbipd --version` (5.3.0) from WSL via powershell.exe, which works for read-only queries, and I attempted a real attach, which failed on the permission check rather than on tooling. I checked WSL for `/dev/ttyUSB*`, `/dev/ttyACM*` and `/dev/video*`: none exist. I captured the stable identities I need for the role map without your help (CH343 VID_1A86:PID_55D3, serials 5B91030298 for COM3 and 5B91045044 for COM5), so I am not asking you to look those up. Non-admin `usbipd attach` is blocked by design, and there is no authorized way for me to elevate or to plug in hardware. All software-only work (AA-4 scaffold, AA-6 arbiter, AA-7 logging, and every mock-backend test) proceeds without this, so I am only asking for the part that genuinely gates hardware bring-up.

Unlocks: All robot USB devices physically connected and bound in usbipd, so that from WSL I can attach them and see the expected /dev/ttyUSB* or /dev/ttyACM* and /dev/video* nodes. Specifically: both SO-101 arm buses, the Rotrics DexArm, the three OV9281 cameras, and the USB relay each appear in `usbipd state` with a BusId and a PersistedGuid. Also: a note of which CH343 serial (5B91030298 or 5B91045044) is the left arm versus the right arm, since I cannot determine that without moving hardware.

Estimated active time: 20 minutes

Preparation: Windows host with usbipd-win 5.3.0 already installed (confirmed present). The robot's centralized USB dock and its USB 2.0 uplink cable to the computer. An elevated Windows Terminal or PowerShell window ("Run as administrator"). Arms should be powered but clear of obstructions in case a servo twitches on connection; do not have the tube clamped in the grippers for this task. Nothing is commanded to move by this procedure.

## Procedure

1. Physically connect the robot's centralized USB dock to the computer's USB port and apply power to the dock, the two SO-101 arms, and the DexArm. Leave the grippers empty and not engaged with the tube.
   - Expected: Dock power indicator is on; Windows plays the device-connect sound or shows new devices.
   - Verify: In a normal (non-admin) PowerShell run: usbipd list . Both 'USB-Enhanced-SERIAL CH343' entries should now show a BusId value instead of appearing only under Persisted.
   - Stop if: Stop if a servo moves on its own, if anything audibly binds or strains, or if the dock does not power up. Report what you saw instead of continuing.

2. Open an elevated console: right-click Windows Terminal or PowerShell and choose 'Run as administrator'.
   - Expected: A console window whose title bar includes 'Administrator'.
   - Verify: Run: usbipd --version . It should print 5.3.0-... without a permission error.
   - Stop if: Stop if the elevation prompt is refused or you do not have an administrator account on this machine.

3. List all connected devices and record the full table: run usbipd list and copy the entire 'Connected' section, including BusId, VID:PID and DEVICE text for every row.
   - Expected: A table that includes the two CH343 serial adapters, plus rows for the DexArm, the cameras and the USB relay. Some rows may have generic names such as a composite USB device or a UVC camera name.
   - Verify: Confirm at least six robot-related rows are present beyond the built-in Surface cameras, Bluetooth and the USB input device. Paste the table into the task notes.
   - Stop if: Stop and report the table as-is if fewer devices appear than expected, for example if the cameras or the relay are missing. Do not start unplugging and replugging other devices to hunt for them.

4. Bind each robot device by its BusId, one command per device, using the BusIds you recorded: usbipd bind --busid <BusId> . Do this for both CH343 adapters, the DexArm, each of the three cameras, and the relay. Do not bind the Surface Camera Front, the Intel Wireless Bluetooth, or the USB Input Device rows.
   - Expected: Each command completes with no output, which indicates success.
   - Verify: Run usbipd state and confirm each robot device now has both a BusId and a non-null PersistedGuid.
   - Stop if: Stop if any bind command returns an error, and report the exact device and error text. Do not retry more than once per device, and do not use --force.

5. Identify which arm is which: note the CH343 serial number shown for each arm bus (5B91030298 corresponds to COM3 and 5B91045044 corresponds to COM5 from the earlier scan), and physically trace each adapter's cable to determine whether it goes to the left or the right SO-101 arm as seen in pictures/robot.png.
   - Expected: A clear statement such as 'serial 5B91030298 is the left arm, serial 5B91045044 is the right arm'.
   - Verify: Trace each cable end to end by hand, or temporarily unplug one arm bus, run usbipd list, and see which serial disappears, then plug it back in.
   - Stop if: Stop if the cables cannot be traced without disassembling anything, or if you have to disconnect the DexArm or any camera to follow a cable. Report the ambiguity rather than guessing, since a wrong mapping could drive a trajectory into the chassis.

6. Leave the dock connected and powered, and report back the usbipd list table from step 3 plus the left/right arm mapping from step 5.
   - Expected: Devices remain connected and bound, ready for me to run the attach step from WSL myself.
   - Verify: Final check: usbipd state shows a PersistedGuid for every robot device.
   - Stop if: Stop if you need to power the robot down before I can run the attach; tell me so, and I will schedule the attach for when it is powered again.

## Machine-readable procedure (keep in sync when revising)

```human-procedure
{
  "title": "Connect robot USB dock and admin-bind all devices for WSL",
  "parent": "AA-5",
  "reason": "Two separate things only you can do. (1) Physical: `usbipd state` shows both SO-101 CH343 adapters as persisted but with no BusId, meaning they are not currently plugged in, and the DexArm, the three OV9281 cameras and the USB relay do not appear at all. Someone has to physically connect and power the dock. (2) Permission: `usbipd bind` requires an elevated Windows administrator console. I verified this directly, not by assumption: `usbipd attach --wsl --busid 2-2` returned \"Device is not shared; run 'usbipd bind --busid 2-2' as administrator first\". I cannot elevate from this session. Bind is one-time per device and persists, so this is not a recurring request.",
  "alternatives": "I ran `usbipd state`, `usbipd list` and `usbipd --version` (5.3.0) from WSL via powershell.exe, which works for read-only queries, and I attempted a real attach, which failed on the permission check rather than on tooling. I checked WSL for `/dev/ttyUSB*`, `/dev/ttyACM*` and `/dev/video*`: none exist. I captured the stable identities I need for the role map without your help (CH343 VID_1A86:PID_55D3, serials 5B91030298 for COM3 and 5B91045044 for COM5), so I am not asking you to look those up. Non-admin `usbipd attach` is blocked by design, and there is no authorized way for me to elevate or to plug in hardware. All software-only work (AA-4 scaffold, AA-6 arbiter, AA-7 logging, and every mock-backend test) proceeds without this, so I am only asking for the part that genuinely gates hardware bring-up.",
  "outcome": "All robot USB devices physically connected and bound in usbipd, so that from WSL I can attach them and see the expected /dev/ttyUSB* or /dev/ttyACM* and /dev/video* nodes. Specifically: both SO-101 arm buses, the Rotrics DexArm, the three OV9281 cameras, and the USB relay each appear in `usbipd state` with a BusId and a PersistedGuid. Also: a note of which CH343 serial (5B91030298 or 5B91045044) is the left arm versus the right arm, since I cannot determine that without moving hardware.",
  "preparation": "Windows host with usbipd-win 5.3.0 already installed (confirmed present). The robot's centralized USB dock and its USB 2.0 uplink cable to the computer. An elevated Windows Terminal or PowerShell window (\"Run as administrator\"). Arms should be powered but clear of obstructions in case a servo twitches on connection; do not have the tube clamped in the grippers for this task. Nothing is commanded to move by this procedure.",
  "category": "permission",
  "minutes": 20,
  "steps": [
    {
      "action": "Physically connect the robot's centralized USB dock to the computer's USB port and apply power to the dock, the two SO-101 arms, and the DexArm. Leave the grippers empty and not engaged with the tube.",
      "expected": "Dock power indicator is on; Windows plays the device-connect sound or shows new devices.",
      "verification": "In a normal (non-admin) PowerShell run: usbipd list . Both 'USB-Enhanced-SERIAL CH343' entries should now show a BusId value instead of appearing only under Persisted.",
      "stop": "Stop if a servo moves on its own, if anything audibly binds or strains, or if the dock does not power up. Report what you saw instead of continuing."
    },
    {
      "action": "Open an elevated console: right-click Windows Terminal or PowerShell and choose 'Run as administrator'.",
      "expected": "A console window whose title bar includes 'Administrator'.",
      "verification": "Run: usbipd --version . It should print 5.3.0-... without a permission error.",
      "stop": "Stop if the elevation prompt is refused or you do not have an administrator account on this machine."
    },
    {
      "action": "List all connected devices and record the full table: run usbipd list and copy the entire 'Connected' section, including BusId, VID:PID and DEVICE text for every row.",
      "expected": "A table that includes the two CH343 serial adapters, plus rows for the DexArm, the cameras and the USB relay. Some rows may have generic names such as a composite USB device or a UVC camera name.",
      "verification": "Confirm at least six robot-related rows are present beyond the built-in Surface cameras, Bluetooth and the USB input device. Paste the table into the task notes.",
      "stop": "Stop and report the table as-is if fewer devices appear than expected, for example if the cameras or the relay are missing. Do not start unplugging and replugging other devices to hunt for them."
    },
    {
      "action": "Bind each robot device by its BusId, one command per device, using the BusIds you recorded: usbipd bind --busid <BusId> . Do this for both CH343 adapters, the DexArm, each of the three cameras, and the relay. Do not bind the Surface Camera Front, the Intel Wireless Bluetooth, or the USB Input Device rows.",
      "expected": "Each command completes with no output, which indicates success.",
      "verification": "Run usbipd state and confirm each robot device now has both a BusId and a non-null PersistedGuid.",
      "stop": "Stop if any bind command returns an error, and report the exact device and error text. Do not retry more than once per device, and do not use --force."
    },
    {
      "action": "Identify which arm is which: note the CH343 serial number shown for each arm bus (5B91030298 corresponds to COM3 and 5B91045044 corresponds to COM5 from the earlier scan), and physically trace each adapter's cable to determine whether it goes to the left or the right SO-101 arm as seen in pictures/robot.png.",
      "expected": "A clear statement such as 'serial 5B91030298 is the left arm, serial 5B91045044 is the right arm'.",
      "verification": "Trace each cable end to end by hand, or temporarily unplug one arm bus, run usbipd list, and see which serial disappears, then plug it back in.",
      "stop": "Stop if the cables cannot be traced without disassembling anything, or if you have to disconnect the DexArm or any camera to follow a cable. Report the ambiguity rather than guessing, since a wrong mapping could drive a trajectory into the chassis."
    },
    {
      "action": "Leave the dock connected and powered, and report back the usbipd list table from step 3 plus the left/right arm mapping from step 5.",
      "expected": "Devices remain connected and bound, ready for me to run the attach step from WSL myself.",
      "verification": "Final check: usbipd state shows a PersistedGuid for every robot device.",
      "stop": "Stop if you need to power the robot down before I can run the attach; tell me so, and I will schedule the attach for when it is powered again."
    }
  ]
}
```
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 All robot USB devices physically connected and bound in usbipd, so that from WSL I can attach them and see the expected /dev/ttyUSB* or /dev/ttyACM* and /dev/video* nodes. Specifically: both SO-101 arm buses, the Rotrics DexArm, the three OV9281 cameras, and the USB relay each appear in `usbipd state` with a BusId and a PersistedGuid. Also: a note of which CH343 serial (5B91030298 or 5B91045044) is the left arm versus the right arm, since I cannot determine that without moving hardware.
<!-- AC:END -->

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
  "key": "82c2d849e0ff6e02d332c0d99ab9f781728389e46288ceb0eb17c7a990db591d",
  "phase": "paused",
  "createdAt": "2026-10-20T21:15:28.287Z",
  "origin": "01a15b09-5f50-755d-bad8-87a487c943cc",
  "step": 0,
  "evidence": [],
  "history": [
    {
      "action": "accept",
      "at": "2026-10-21T01:22:46.619Z",
      "hash": "38e9a3b39fdd071a20ee3d62e8b5808223cc3cba6224d3a0e819f2f9d82b5fd6"
    },
    {
      "action": "accept",
      "at": "2026-10-21T01:52:29.799Z",
      "hash": "38e9a3b39fdd071a20ee3d62e8b5808223cc3cba6224d3a0e819f2f9d82b5fd6"
    }
  ],
  "acceptedHash": "38e9a3b39fdd071a20ee3d62e8b5808223cc3cba6224d3a0e819f2f9d82b5fd6",
  "acceptedAt": "2026-10-21T01:22:46.622Z"
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->
