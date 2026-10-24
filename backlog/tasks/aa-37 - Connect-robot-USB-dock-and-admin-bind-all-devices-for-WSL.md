---
id: AA-37
title: Connect robot USB dock and admin-bind all devices for WSL
status: Done
assignee:
  - '@human'
created_date: '2026-10-20 21:15'
updated_date: '2026-10-24 14:49'
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
- [x] #1 All robot USB devices physically connected and bound in usbipd, so that from WSL I can attach them and see the expected /dev/ttyUSB* or /dev/ttyACM* and /dev/video* nodes. Specifically: both SO-101 arm buses, the Rotrics DexArm, the three OV9281 cameras, and the USB relay each appear in `usbipd state` with a BusId and a PersistedGuid. Also: a note of which CH343 serial (5B91030298 or 5B91045044) is the left arm versus the right arm, since I cannot determine that without moving hardware.
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
  "phase": "verified",
  "createdAt": "2026-10-20T21:15:28.287Z",
  "origin": "01a15b09-5f50-755d-bad8-87a487c943cc",
  "step": 6,
  "evidence": [
    {
      "step": 1,
      "note": "Step 1 verification NOT met. User pasted non-admin `usbipd list` output. Connected: 2-2 045e:0c1e Surface Camera Front/IR (Not shared), 2-10 8087:0029 Intel Wireless Bluetooth (Not shared), 4-5 0bda:1100 USB Input Device (Not shared), 7-2 1a86:55d3 USB-Enhanced-SERIAL CH343 (COM3) STATE=Shared, 8-3 0483:5740 USB Serial Device (COM6) (Not shared). Persisted only: 94285d9f-33cf-4d6c-b488-6282fbe7f102 USB-Enhanced-SERIAL CH343 (COM5).\n\nFindings: only ONE of the two CH343 arm adapters is present (COM3 at 7-2); COM5 is still persisted-with-no-BusId, i.e. not connected. 7-2 already shows Shared, so that device appears to have been bound at some earlier point. A new row 8-3 0483:5740 (STMicroelectronics VID) appeared and is plausibly the DexArm but is unconfirmed. The three OV9281 cameras and the USB relay do not appear at all. Step 1 requires both CH343 entries to show a BusId, so not advancing. Awaiting dock power-indicator state and whether all dock downstream cables are seated.",
      "type": "self-reported",
      "reference": "User-pasted `usbipd list` output in walkthrough conversation (AA-37 step 1)",
      "advanced": false,
      "at": "2026-10-23T02:24:00.328Z"
    },
    {
      "step": 1,
      "note": "Step 1 still NOT verified, but materially closer. Second user-pasted non-admin `usbipd list` Connected section:\n2-2 045e:0c1e Surface Camera Front/IR (Not shared) [built-in]\n2-10 8087:0029 Intel Wireless Bluetooth (Not shared) [built-in]\n4-5 0bda:1100 USB Input Device (Not shared) [built-in]\n7-1 1a86:7523 USB-SERIAL CH340 (COM4) (Not shared) [NEW - plausibly USB relay, unconfirmed]\n7-2 1a86:55d3 USB-Enhanced-SERIAL CH343 (COM3) STATE=Shared [arm bus A, already bound]\n7-3 32e6:9251 openaicam (Not shared) [NEW - camera]\n8-3 0483:5740 USB Serial Device (COM6) (Not shared) [plausibly DexArm, unconfirmed]\n8-4 0c45:6366 Arducam OV9281 USB Camera (Not shared) [NEW - camera]\nPersisted only: 94285d9f-33cf-4d6c-b488-6282fbe7f102 USB-Enhanced-SERIAL CH343 (COM5).\n\nFive robot-related rows now enumerate (was 1-2). Remaining gaps: (a) the second SO-101 arm adapter, CH343 COM5 / serial 5B91045044, is STILL persisted-with-no-BusId i.e. not connected - this is exactly what step 1's verification requires; (b) only two camera rows are present (7-3 openaicam, 8-4 Arducam OV9281) against three expected OV9281 cameras, and the two camera rows report different VID:PIDs (32e6:9251 vs 0c45:6366) so the expected camera set may differ from the procedure's assumption. Not advancing: step 1 requires both CH343 entries to show a BusId. No unplug/replug hunting performed. Asked user whether the second SO-101 arm is powered and its USB cable seated in the dock.",
      "type": "self-reported",
      "reference": "Second user-pasted `usbipd list` output in walkthrough conversation (AA-37 step 1)",
      "advanced": false,
      "at": "2026-10-23T02:26:45.113Z"
    },
    {
      "step": 1,
      "note": "Step 1 VERIFIED (recorded on resume; evidence was supplied before the task was paused). Third user-pasted non-admin `usbipd list` Connected section shows BOTH CH343 arm adapters with BusIds, which is step 1's stated verification:\n2-2 045e:0c1e Surface Camera Front/IR (Not shared) [built-in]\n2-10 8087:0029 Intel Wireless Bluetooth (Not shared) [built-in]\n4-5 0bda:1100 USB Input Device (Not shared) [built-in]\n9-2 1a86:7523 USB-SERIAL CH340 (COM7) (Not shared) [relay candidate, unconfirmed]\n9-3 1a86:55d3 CH343 (COM3) STATE=Shared [arm bus A]\n9-4 32e6:9251 openaicam (Not shared) [camera 1]\n10-2 32e6:9251 openaicam (Not shared) [camera 2]\n10-4 1a86:55d3 CH343 (COM5) STATE=Shared [arm bus B]\n11-3 0483:5740 USB Serial Device (COM6) (Not shared) [DexArm candidate, unconfirmed]\n11-4 0c45:6366 Arducam OV9281 USB Camera (Not shared) [camera 3]\n\nSeven robot-related rows, exceeding step 3's \"at least six\" threshold. All expected device classes now enumerate: 2 arm buses, 3 cameras, 1 relay candidate, 1 DexArm candidate.\n\nCarry-forward observations:\n(a) BusIds MOVED between captures (arm A was 7-2, now 9-3; hubs renumbered 7/8 -> 9/10/11). BusIds are not stable across reconnects, so step 4 must bind from a FRESH list taken in the elevated console, not from any earlier numbers.\n(b) BOTH CH343 adapters already report STATE=Shared, including COM5 which was never bound in this session. Those two binds may be no-ops; binds actually outstanding are the relay, the three cameras and the DexArm.\n(c) Camera set does NOT match the procedure's \"three OV9281\" assumption: two are 32e6:9251 \"openaicam\", one is 0c45:6366 \"Arducam OV9281\". Flagged for orchestrator review as a possible inventory discrepancy; not treated as a blocker since three camera devices are present as the outcome requires.\n(d) User reports all arms and cameras connected. No servo motion, binding or strain reported, so no step 1 stop condition triggered. Earlier missing-device condition resolved after user intervention; root cause NOT established (power-budget and cable hypotheses were discussed but never confirmed).\n(e) Persisted section was not included in the third paste.\nEvidence is user-pasted console text, not independently verified by me.",
      "type": "self-reported",
      "reference": "Third user-pasted `usbipd list` Connected table in walkthrough conversation (AA-37 step 1)",
      "advanced": true,
      "at": "2026-10-24T13:56:35.326Z"
    },
    {
      "step": 2,
      "note": "Step 2 VERIFIED. User opened an elevated console and `usbipd --version` printed without a permission error:\n5.3.0-54+Branch.master.Sha.aa3db8b82c4cb5071fd31bc54211606c70886912.aa3db8b82c4cb5071fd31bc54211606c70886912\n\nMatches the expected 5.3.0-... version string and confirms usbipd-win 5.3.0 is on PATH in the elevated session. Note: `usbipd --version` succeeds in a non-elevated console too, so this output alone does not independently prove elevation; the \"Administrator\" title bar was user-observed and not separately evidenced. Elevation will be definitively confirmed at step 4, where a non-elevated `usbipd bind` would fail with a permission error. No stop condition triggered (no refused UAC prompt, no missing admin account reported). Elevated window to remain open for steps 3 and 4.",
      "type": "self-reported",
      "reference": "User-pasted `usbipd --version` output in walkthrough conversation (AA-37 step 2)",
      "advanced": true,
      "at": "2026-10-24T13:58:14.883Z"
    },
    {
      "step": 3,
      "note": "Step 3 VERIFIED. Fresh `usbipd list` captured in the elevated console. Full Connected section:\nBUSID  VID:PID    DEVICE                                          STATE\n2-2    045e:0c1e  Surface Camera Front, Surface IR Camera Front   Not shared   [built-in, DO NOT BIND]\n2-10   8087:0029  Intel(R) Wireless Bluetooth(R)                  Not shared   [built-in, DO NOT BIND]\n4-5    0bda:1100  USB Input Device                                Not shared   [built-in, DO NOT BIND]\n9-2    1a86:7523  USB-SERIAL CH340 (COM7)                         Not shared   [relay candidate]\n9-3    1a86:55d3  USB-Enhanced-SERIAL CH343 (COM3)                Shared       [arm bus A, already shared]\n9-4    32e6:9251  openaicam                                       Not shared   [camera 1]\n10-2   32e6:9251  openaicam                                       Not shared   [camera 2]\n10-4   1a86:55d3  USB-Enhanced-SERIAL CH343 (COM5)                Shared       [arm bus B, already shared]\n11-3   0483:5740  USB Serial Device (COM6)                        Not shared   [DexArm candidate]\n11-4   0c45:6366  Arducam OV9281 USB Camera                       Not shared   [camera 3]\nPersisted: header present with NO rows.\n\nSeven robot-related rows beyond the three built-ins, exceeding the \"at least six\" threshold, so step 3's verification is satisfied. BusIds are IDENTICAL to the third step-1 capture (9-2, 9-3, 9-4, 10-2, 10-4, 11-3, 11-4), so the topology has been stable across both captures and these numbers are safe to use for step 4 binds.\n\nNotes:\n(a) The empty Persisted section is consistent rather than contradictory: usbipd lists bound-but-currently-disconnected devices under Persisted, and both bound CH343 adapters are now connected, so they appear in Connected with STATE=Shared instead. The step 4 verification via `usbipd state` will be the authoritative check for BusId + non-null PersistedGuid per device.\n(b) Binds outstanding for step 4: 9-2, 9-4, 10-2, 11-3, 11-4 (five devices). 9-3 and 10-4 already show Shared and will be skipped rather than re-bound, since binding an already-shared device is unnecessary and the procedure forbids --force.\n(c) Role labels for 9-2 (relay) and 11-3 (DexArm) remain inferred from VID:PID only and are still unconfirmed by physical tracing.\n(d) Camera inventory discrepancy persists: two 32e6:9251 \"openaicam\" plus one 0c45:6366 \"Arducam OV9281\", versus the procedure's \"three OV9281\" assumption. Three camera devices are present, satisfying the outcome's count, but the device identities differ from the task description. Flagged for orchestrator review.\nEvidence is user-pasted console text, not independently verified by me.",
      "type": "self-reported",
      "reference": "User-pasted elevated `usbipd list` Connected + Persisted output in walkthrough conversation (AA-37 step 3)",
      "advanced": true,
      "at": "2026-10-24T13:59:19.121Z"
    },
    {
      "step": 4,
      "note": "Step 4 VERIFIED. User ran the five binds (9-2, 9-4, 10-2, 11-3, 11-4) in the elevated console and pasted full `usbipd state` JSON. All seven robot devices now have BOTH a BusId and a non-null PersistedGuid, which is step 4's stated verification:\n9-3  CH343 (COM3)      VID_1A86&PID_55D3\\5B91030298     guid 2f00f798-5407-4c13-b96e-3d119241835b  [arm bus A]\n10-4 CH343 (COM5)      VID_1A86&PID_55D3\\5B91045044     guid 94285d9f-33cf-4d6c-b488-6282fbe7f102  [arm bus B]\n9-2  USB-SERIAL CH340  VID_1A86&PID_7523\\7&165AD49D&0&2 guid 824c18df-2ce9-4f3e-8465-cb157c24e559  [relay]\n9-4  openaicam         VID_32E6&PID_9251\\5423534724     guid 3da964d5-c232-49d0-901a-53b41883b7cc  [camera 1]\n10-2 openaicam         VID_32E6&PID_9251\\7&69F6256&0&2  guid 39288756-bae8-4c07-bab9-542538063315  [camera 2]\n11-3 USB Serial Device VID_0483&PID_5740\\306133643438   guid 681c79d1-54e6-4a31-b175-c32ee82d7ca8  [DexArm]\n11-4 Arducam OV9281    VID_0C45&PID_6366\\UC762          guid 0ffed26c-f9a7-476d-b65c-b675b341641b  [camera 3]\nCorrectly NOT bound (PersistedGuid null, as instructed): 2-2 Surface Camera, 2-10 Intel Bluetooth, 4-5 USB Input Device.\nIsForced is false on every device, confirming no --force was used. ClientIPAddress and StubInstanceId are null everywhere, i.e. bound/shared but not yet attached to WSL - expected at this stage.\n\nThis also retroactively proves step 2's elevation: `usbipd bind` cannot succeed without administrator rights, and five binds succeeded.\n\nThe task outcome's device-identity requirement is now met, and arm serials are confirmed against BusIds: serial 5B91030298 = COM3 = BusId 9-3; serial 5B91045044 = COM5 = BusId 10-4. This matches the serial/COM pairing asserted in the procedure description.\n\nNEW observation for orchestrator (affects stable device naming downstream, not this task's pass/fail): only three of the seven robot devices expose a real unique serial in their InstanceId - 9-3 (5B91030298), 10-4 (5B91045044), 9-4 (5423534724), 11-3 (306133643438) and 11-4 (UC762). Two do NOT: 9-2 relay (7&165AD49D&0&2) and 10-2 camera 2 (7&69F6256&0&2) use hub-port-derived instance IDs, so those two are identified POSITIONALLY and their identity will change if moved to a different hub port. Note also that 11-4's \"UC762\" is a model string shared across Arducam units rather than a per-unit serial. Implication: udev rules keyed on serial will work for the two arms and camera 1, but the relay and camera 2 will need by-path or port-pinned identification, and camera 3 may collide if a second identical Arducam is ever added. Not a blocker for AA-37.\nEvidence is user-pasted console JSON, not independently verified by me.",
      "type": "self-reported",
      "reference": "User-pasted elevated `usbipd state` JSON output in walkthrough conversation (AA-37 step 4)",
      "advanced": true,
      "at": "2026-10-24T14:02:02.481Z"
    },
    {
      "step": 5,
      "note": "CORRECTION to the step 4 note (recorded here because the checkpoint has already moved to step 5; step 4's pass result is unaffected). The step 4 note says \"only three of the seven robot devices expose a real unique serial\" and then lists five devices. That count is wrong. Corrected: FIVE of seven InstanceIds end in a serial-like string: 9-3 (5B91030298), 10-4 (5B91045044), 9-4 (5423534724), 11-3 (306133643438), 11-4 (UC762). TWO use Windows hub-port-derived instance IDs (the \"7&...&0&2\" form), which means the device reports no USB serial: 9-2 relay (7&165AD49D&0&2) and 10-2 camera 2 (7&69F6256&0&2). Their identity is positional and will change if they move to a different port. Also correcting an overstatement: the note said 11-4's \"UC762\" is a model string shared across Arducam units. That is an unverified guess. It may be a per-unit serial. Only one Arducam is present, so this cannot be checked here. Step 5 not yet started.",
      "type": "self-reported",
      "reference": "Correction of my own AA-37 step 4 checkpoint note, based on the same user-pasted `usbipd state` JSON",
      "advanced": false,
      "at": "2026-10-24T14:02:16.117Z"
    },
    {
      "step": 5,
      "note": "Step 5 mapping REPORTED by user, NOT yet advanced. User statement: \"left arm is com 5. right arm is com3. 9-4 is left camera and 10-2 is right camera for the arm.\"\nUsing the step 4 usbipd state identities, this gives:\n- LEFT SO-101 arm = COM5 = BusId 10-4 = CH343 serial 5B91045044\n- RIGHT SO-101 arm = COM3 = BusId 9-3 = CH343 serial 5B91030298\n- Left arm camera = BusId 9-4 = openaicam 32e6:9251, InstanceId serial 5423534724\n- Right arm camera = BusId 10-2 = openaicam 32e6:9251, InstanceId 7&69F6256&0&2 (hub-port-derived, no USB serial, so this identity is positional)\nInferred by elimination, not stated by the user: 11-4 Arducam OV9281 (UC762) is the third camera, and its role was not given.\n\nNot advancing yet, for two reasons. (1) Step 5 requires either an end-to-end cable trace or an unplug test, and the user has not said which method they used. (2) The user has not confirmed that left/right is relative to the pictures/robot.png viewpoint, which the procedure specifies. A swapped mapping is a stated collision risk. The camera mapping goes beyond the procedure scope and is recorded as extra information. Note that 10-2's identity will change if that camera moves to a different hub port.",
      "type": "self-reported",
      "reference": "User chat statement of arm/camera mapping in walkthrough conversation (AA-37 step 5), cross-referenced against step 4 user-pasted `usbipd state` JSON",
      "advanced": false,
      "at": "2026-10-24T14:09:50.481Z"
    },
    {
      "step": 5,
      "note": "Step 5 method now REPORTED by user: \"Traced the cables by hand. Unplugged one at a time. Saw which went away.\" So both verification methods were used: hand trace plus a one-at-a-time unplug test. That satisfies the method part of step 5's verification. Mapping as reported earlier: LEFT arm = COM5 = serial 5B91045044; RIGHT arm = COM3 = serial 5B91030298.\n\nStill NOT advancing, for two reasons.\n(1) The user wrote \"Here is the most recent list:\" but no list was included in the message. Since devices were unplugged and replugged, BusIds may have changed. Post-replug evidence that both arm adapters re-enumerated is not yet in hand.\n(2) The viewpoint question is still unanswered: whether left/right is as seen in pictures/robot.png, or from the robot's point of view, or from the operator's. A swapped mapping is the step's stated collision risk.\nAsked the user to re-paste the list and confirm the viewpoint. Camera 11-4's role is still not given (optional).",
      "type": "self-reported",
      "reference": "User chat message in walkthrough conversation (AA-37 step 5) stating trace+unplug method; referenced usbipd list was not attached",
      "advanced": false,
      "at": "2026-10-24T14:11:20.164Z"
    },
    {
      "step": 5,
      "note": "Post-replug `usbipd list` pasted by user after the step 5 hand-trace and one-at-a-time unplug test. Connected: 9-2 CH340 (COM7) Shared; 9-3 CH343 (COM3) Shared; 9-4 openaicam Shared; 10-2 openaicam Shared; 10-4 CH343 (COM5) Shared; 11-3 USB Serial Device (COM6) Shared; 11-4 Arducam OV9281 Shared. Built-ins 2-2, 2-10 and 4-5 are Not shared, as intended. Persisted section is empty.\n\nBoth arm adapters re-enumerated at the SAME BusIds as before (COM3 on 9-3, COM5 on 10-4), and the binds survived the replug. All seven robot devices now show STATE=Shared. The unplug test did not disturb the topology.\n\nStep 5 verification method is satisfied: the user traced the cables by hand and also ran the one-at-a-time unplug test. Mapping reported: LEFT arm = COM5 = serial 5B91045044 (10-4); RIGHT arm = COM3 = serial 5B91030298 (9-3).\n\nStill not advancing for one remaining reason: the user has not yet confirmed the left/right frame of reference. The procedure specifies \"as seen in pictures/robot.png\". I could not interpret that image myself. Asked once more, as a one-word answer.",
      "type": "self-reported",
      "reference": "User-pasted post-replug `usbipd list` output in walkthrough conversation (AA-37 step 5)",
      "advanced": false,
      "at": "2026-10-24T14:11:31.617Z"
    },
    {
      "step": 5,
      "note": "User gave the role of camera 11-4 (answering the earlier optional question): \"The arducam OV9281 is the camera for the 3rd 3d printer head arm to be used with the laser.\" Recorded: BusId 11-4, Arducam OV9281 0c45:6366, InstanceId UC762, PersistedGuid 0ffed26c-f9a7-476d-b65c-b675b341641b = camera for the third arm (the 3D-printer-head arm, presumably the DexArm at 11-3, used with the laser). The user did not explicitly say this third arm is the DexArm, so that link is my inference. The laser is noted only as context. This procedure commands nothing, and no laser use is in scope.\n\nFull camera role map, all user-reported: 9-4 openaicam (serial 5423534724) = left SO-101 arm camera; 10-2 openaicam (port-derived ID, positional) = right SO-101 arm camera; 11-4 Arducam OV9281 = third-arm/laser camera.\n\nStep 5 still NOT advanced. The user has not yet answered the frame-of-reference question: whether left/right is as seen in pictures/robot.png, as seen standing in front of the robot, or from the robot's own point of view. The procedure requires the robot.png frame, and a mirrored mapping is the step's stated collision risk. Asked again.",
      "type": "self-reported",
      "reference": "User chat message in walkthrough conversation (AA-37 step 5) giving camera 11-4's role",
      "advanced": false,
      "at": "2026-10-24T14:11:59.458Z"
    },
    {
      "step": 5,
      "note": "Step 5 VERIFIED. User confirmed the frame of reference: \"i used same view as the image\", meaning left/right is as seen in pictures/robot.png, which is the frame the procedure requires.\n\nFinal arm mapping (user-reported, robot.png viewpoint):\n- LEFT SO-101 arm = COM5 = CH343 serial 5B91045044 = BusId 10-4 = PersistedGuid 94285d9f-33cf-4d6c-b488-6282fbe7f102\n- RIGHT SO-101 arm = COM3 = CH343 serial 5B91030298 = BusId 9-3 = PersistedGuid 2f00f798-5407-4c13-b96e-3d119241835b\nMethod: the user traced each cable by hand AND ran a one-at-a-time unplug test, watching which COM port disappeared. Both of the step's verification methods were used. The post-replug usbipd list showed both adapters back on the same BusIds, still Shared.\n\nCamera roles (user-reported, beyond procedure scope): 9-4 openaicam (serial 5423534724) = left arm camera; 10-2 openaicam (port-derived ID 7&69F6256&0&2, positional) = right arm camera; 11-4 Arducam OV9281 (UC762) = third-arm camera, for the 3D-printer-head arm used with the laser. My inference, not confirmed by the user: the third arm is the DexArm at 11-3.\n\nUncertainty: I could not interpret pictures/robot.png myself, so the viewpoint match rests on the user's statement alone. The mapping is self-reported and has not been independently verified. The orchestrator should confirm it with a low-speed, low-torque motion test before any real trajectory runs.",
      "type": "self-reported",
      "reference": "User chat confirmation of the robot.png viewpoint, plus the earlier trace/unplug report and post-replug usbipd list, in the walkthrough conversation (AA-37 step 5)",
      "advanced": true,
      "at": "2026-10-24T14:12:52.070Z"
    },
    {
      "step": 6,
      "note": "Step 6 VERIFIED. User pasted a final `usbipd state` JSON. The opening '{\"Devices\": [' and the first brace were cut off in the paste, but all ten device entries are present. All seven robot devices have a BusId and a non-null PersistedGuid, and every GUID matches the step 4 capture exactly:\n9-3  CH343 (COM3)  serial 5B91030298  2f00f798-5407-4c13-b96e-3d119241835b  RIGHT SO-101 arm\n10-4 CH343 (COM5)  serial 5B91045044  94285d9f-33cf-4d6c-b488-6282fbe7f102  LEFT SO-101 arm\n9-2  CH340 (COM7)  7&165AD49D&0&2     824c18df-2ce9-4f3e-8465-cb157c24e559  relay (role inferred from VID:PID, not confirmed)\n9-4  openaicam     serial 5423534724  3da964d5-c232-49d0-901a-53b41883b7cc  left arm camera\n10-2 openaicam     7&69F6256&0&2      39288756-bae8-4c07-bab9-542538063315  right arm camera (positional ID)\n11-3 USB Serial (COM6) 0483:5740 306133643438  681c79d1-54e6-4a31-b175-c32ee82d7ca8  DexArm (role inferred from VID:PID, not confirmed)\n11-4 Arducam OV9281 UC762             0ffed26c-f9a7-476d-b65c-b675b341641b  third-arm / laser camera\nBuilt-ins 2-2, 2-10 and 4-5 have PersistedGuid null, as intended (not bound). IsForced is false everywhere. ClientIPAddress is null everywhere, meaning nothing is attached to WSL yet, which is expected because the orchestrator runs the attach. The bind records survived the step 5 unplug/replug.\nOpen item: the user did not say whether the robot will stay powered for the WSL attach. That is not a step 6 stop condition unless they need to power down. Asked in the closing summary.",
      "type": "self-reported",
      "reference": "User-pasted final elevated `usbipd state` JSON in walkthrough conversation (AA-37 step 6)",
      "advanced": true,
      "at": "2026-10-24T14:14:25.903Z"
    }
  ],
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
  "acceptedAt": "2026-10-21T01:22:46.622Z",
  "summary": "no deviations",
  "completedAt": "2026-10-24T14:47:26.318Z",
  "verifiedAt": "2026-10-24T14:49:05.224Z",
  "verification": "I checked this on my own from WSL on 2026-10-24, after the walkthrough reported ready. Non-admin `usbipd list` shows all 7 robot devices as Attached: 9-2 CH340, 9-3 and 10-4 CH343, 9-4 and 10-2 openaicam, 11-3 0483:5740, 11-4 Arducam. The three built-ins (2-2, 2-10, 4-5) are Not shared, as intended. In WSL, /dev/serial/by-id shows 5B91030298->ttyACM0, 5B91045044->ttyACM1, STM32F407ZG 306133643438->ttyACM2 (DexArm), 1a86 CH340->ttyUSB0 (relay), plus /dev/video0 through /dev/video5 (3 UVC cameras). That proves the binds worked: attach would fail without them. The user belongs to dialout and video. Still uncertain: the left/right arm mapping (left=5B91045044/COM5, right=5B91030298/COM3) rests on the user's hand trace and unplug test only, and must be confirmed with a supervised low-speed motion test before any trajectory runs. The relay and DexArm roles are inferred from VID:PID. The camera models don't match doc-1 (two openaicam 32e6:9251 against the specified Waveshare OV9281)."
}
```
<!-- /human-handoff-state -->
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
I checked this on my own from WSL on 2026-10-24, after the walkthrough reported ready. Non-admin `usbipd list` shows all 7 robot devices as Attached: 9-2 CH340, 9-3 and 10-4 CH343, 9-4 and 10-2 openaicam, 11-3 0483:5740, 11-4 Arducam. The three built-ins (2-2, 2-10, 4-5) are Not shared, as intended. In WSL, /dev/serial/by-id shows 5B91030298->ttyACM0, 5B91045044->ttyACM1, STM32F407ZG 306133643438->ttyACM2 (DexArm), 1a86 CH340->ttyUSB0 (relay), plus /dev/video0 through /dev/video5 (3 UVC cameras). That proves the binds worked: attach would fail without them. The user belongs to dialout and video. Still uncertain: the left/right arm mapping (left=5B91045044/COM5, right=5B91030298/COM3) rests on the user's hand trace and unplug test only, and must be confirmed with a supervised low-speed motion test before any trajectory runs. The relay and DexArm roles are inferred from VID:PID. The camera models don't match doc-1 (two openaicam 32e6:9251 against the specified Waveshare OV9281).
<!-- SECTION:FINAL_SUMMARY:END -->
