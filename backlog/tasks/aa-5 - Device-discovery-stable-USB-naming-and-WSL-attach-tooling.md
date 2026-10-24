---
id: AA-5
title: 'Device discovery, stable USB naming and WSL attach tooling'
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:08'
updated_date: '2026-10-24 15:44'
labels: []
milestone: m-0
dependencies:
  - AA-4
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 5000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Devices reach WSL only through usbipd-win, and Linux device nodes are assigned in enumeration order, so /dev/ttyACM0 may be either arm on any given boot. Confusing the two arms could drive a trajectory into the chassis. All five USB devices (2 arm buses, 3 cameras) plus the relay must be identified by stable properties rather than node order.

At planning time usbipd list showed both CH343 serial adapters as persisted but not attached, and WSL had no /dev/ttyUSB*, /dev/ttyACM* or /dev/video* nodes, so this tooling must handle the not-attached case gracefully and tell the operator exactly what to run.

See doc-1 sections 3, 4, 6 and 7.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A discovery command reports every expected device with its stable identity (serial number, USB path or by-id symlink) and maps it to its logical role (arm_left, arm_right, cam_left, cam_right, cam_scanner, relay, dexarm)
- [x] #2 Logical-role to device resolution is driven by the hardware config and never by raw enumeration index; swapping USB port order does not change role assignment
- [x] #3 udev rules or an equivalent documented mechanism give persistent by-id or by-role symlinks for the serial devices and cameras
- [x] #4 A scripted helper attaches all required devices to WSL via usbipd and reports per-device success or failure, exiting nonzero if any required device is missing
- [x] #5 When a device is absent or unattached, the error names the device role and prints the exact remediation command instead of failing with a bare file-not-found
- [x] #6 Discovery output is also available as machine-readable JSON for use by later tooling
- [x] #7 Tests cover role resolution and the missing-device error path using fixtures, with no hardware attached
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
## Slice A (this agent): ACs #1, #2, #5, #6, #7

1. Add inchworm/discovery.py: a DeviceDiscovery class that reads the HardwareConfig and builds a role→DeviceIdentity map; never uses enumeration index. Implements scan_by_id() which reads /dev/serial/by-id and /dev/video/by-id symlinks, and resolve_role(role) which returns the tty/video path or raises DeviceNotFoundError/DeviceNotAttachedError with role name + exact remediation command (usbipd attach ...).

2. Add CLI entry point inchworm/cli/discover.py (or top-level discover_devices.py) callable via 'python -m inchworm.discover' (or similar). Human-readable table output by default; --json flag emits machine-readable JSON list with role/status/path/identity fields.

3. DeviceNotFoundError and DeviceNotAttachedError each carry the role name and a remediation_command string (e.g. 'usbipd bind/attach ...' or 'ls /dev/serial/by-id').

4. Add tests/test_discovery.py covering:
   - Role resolution from config (no hardware): fixtures that mock /dev/serial/by-id. Roles map correctly regardless of port order (swapped ACM0/ACM1 scenario).
   - Missing device error names the role and contains the remediation command string.
   - Unattached device error (by-id file exists but /dev/ttyACM* target doesn't) similarly names role + remediation.
   - --json output shape.
   - Human-readable output contains all expected roles.
   - swapped port order test: right arm appears before left arm in the by-id scan but roles resolve correctly.

5. Wire up a 'inchworm-discover' console_scripts entry point in pyproject.toml.

## Slice B (separate agent): ACs #3, #4

6. udev rules file at robot/udev/99-inchworm.rules: persistent by-id/by-role symlinks for serial devices and cameras (generated from hardware.yaml).
7. WSL attach helper script robot/scripts/attach_usb.sh: iterates all required devices, calls usbipd attach --wsl for each, reports per-device success/failure, exits nonzero if any required device is missing.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Slice A (ACs #1, #2, #5, #6, #7) implemented 2026-10-24.

Files added:
- robot/src/inchworm/discovery.py: DeviceDiscovery logic, discover(), require_device(), format_human(), format_json(), DeviceNotFoundError, DeviceNotAttachedError
- robot/src/inchworm/discover.py: CLI entry point (python -m inchworm.discover [--json] [--config PATH])
- robot/tests/test_discovery.py: 24 fixture-based tests covering all AC items in this slice
- robot/pyproject.toml: added [project.scripts] inchworm-discover console_scripts entry

Live discovery run output (read-only, no bytes written to devices):
- arm_left → /dev/ttyACM1 (5B91045044 serial, attached via ttyACM1 not ACM0)
- arm_right → /dev/ttyACM0 (5B91030298 serial, swapped port order)
- dexarm → /dev/ttyACM2, relay → /dev/ttyUSB0, cam_scanner → /dev/video0
- cam_left/cam_right show NO_UDEV_RULE (linux_by_id: null in config; Slice B to address)

Test run: 179 passed (155 original + 24 new). All pass with no hardware.

Slice B (ACs #3 and #4) implemented 2026-10-24.

Files added:
- robot/udev/99-inchworm.rules: udev rules giving stable /dev/inchworm_* symlinks for 6 serial/video devices (arm_left, arm_right, dexarm, relay, cam_left, cam_scanner). cam_right symlink commented out pending physical port-path confirmation.
- robot/udev/INSTALL.md: install instructions, expected output table, troubleshooting, WSL note.
- robot/scripts/attach_usb.py: Python WSL attach helper. Calls powershell.exe -NoProfile -Command 'usbipd state', parses JSON, resolves BusIds by VID:PID+serial (or VID:PID alone for port-path devices). Calls 'usbipd attach --wsl --busid X' per device. Reports per-device [OK]/[MISSING]/[FAIL]/[DRY-RUN]. Exits nonzero if any device is missing or fails.
- robot/scripts/README.md: usage docs for attach_usb.py.
- robot/tests/test_attach_usb.py: 22 fixture-based tests; no hardware.

Live run output (2026-10-24):
  [OK]      arm_left: already attached (BusId 10-4)
  [OK]      arm_right: already attached (BusId 9-3)
  [OK]      dexarm: already attached (BusId 11-3)
  [OK]      relay: already attached (BusId 9-2)
  [OK]      cam_left: attached BusId 9-4
  [OK]      cam_right: attached BusId 10-2
  [OK]      cam_scanner: already attached (BusId 11-4)
  7/7 devices OK
  Exit 0.

Test run: 201 passed (179 from Slice A + 22 new). All pass with no hardware.

KEY DESIGN CORRECTION applied 2026-10-24 (continued worker):

relay and cam_right identify_by changed from port_path to vid_pid (new IdentifyBy enum value).
- relay: identify_by vid_pid, resolved via linux_by_id (usb-1a86_USB_Serial-if00-port0) in /dev/serial/by-id. Single CH340 1a86:7523 check unchanged.
- cam_right: identify_by vid_pid, no linux_by_id (no USB serial), falls back to linux_by_path for dev_node lookup. Ambiguity check fixed: _cam_left_entries_in_by_id now uses '-video-index0' in name (substring) instead of endswith(), catching duplicate entries like 'usb-openaicam_openaicam_5423534724-video-index0-dup'.
- udev 99-inchworm.rules: cam_right rule added using ATTRS{serial}!="5423534724", ATTR{index}=="0", no KERNELS== port-path pin. Relay rule already VID:PID only. Comment block explains both rules rely on at most one matching device.
- INSTALL.md updated with cam_right row, identity strategy section, and 7-device expected output.
- test_discovery.py: test_roles_resolve_to_correct_by_id_names updated (relay by_id_name now _RELAY_BY_ID not _RELAY_BY_PATH).
- test_config.py: relay and cam_right identify_by assertions updated to VID_PID.

Full test run: 222 passed, 0 failed (uv run pytest).

Live human discovery output (no devices attached in this WSL session):
All 7 roles: absent, remediation shows usbipd busid commands.
relay: by-id=usb-1a86_USB_Serial-if00-port0 (resolved via by-id not by-path).
cam_right: by-id=platform-vhci_hcd.0-usb-0:6:1.0-video-index0 (by-path fallback, informational).

Live JSON discovery output: 7-element array with role/status/by_id_name/dev_node/vid_pid/usb_serial/remediation fields.

USBIP_BUSID RESOLUTION (AA-5 continuation, 2026-10-24):

KEY FINDING: Both 32e6:9251 cameras (Waveshare OV9281) report the SAME USB serial 5423534724 to Linux. /dev/v4l/by-id is ambiguous; the usb-openaicam_openaicam_5423534724-video-index0 symlink pointed at whichever camera enumerated last. udev cannot distinguish them. The only stable distinguisher is the Windows usbipd BusId (physical dock port): cam_left=9-4, cam_right=10-2.

CORRECTION TO doc-1: both openaicam cameras share serial 5423534724. (Orchestrator to update doc-1 per instructions.)

CHANGES MADE:
- inchworm/vhci.py (new): parses /sys/devices/platform/vhci_hcd.0/status, maps BusId -> local sysfs device -> /dev/videoN (index 0). Real vhci sample verified against live system (2026-10-24).
- inchworm/config.py: added IdentifyBy.USBIP_BUSID enum value and windows_busid field (identity key for positional dock-port devices). Validator requires windows_busid when identify_by=usbip_busid.
- config/hardware.yaml: cam_left and cam_right changed to identify_by: usbip_busid with windows_busid: '9-4' and '10-2' respectively. usb_serial set to null (shared serial cannot be used as identity). linux_by_id/linux_by_path set to null. Comment documents POSITIONAL requirement.
- inchworm/discovery.py: added usbip_busid resolution path (_probe_usbip_busid). Added global invariant: DuplicateDeviceNodeError if two roles resolve to same /dev node; DuplicateUsbDeviceError if two roles share same USB identity key. Removed CamRightAmbiguousError as active check (kept for backwards compat). Relay VID_PID path unchanged.
- udev/99-inchworm.rules: removed cam_left/cam_right rules (udev cannot distinguish them; both cameras have serial 5423534724). Added NOTE explaining mechanism (discovery via usbip_busid satisfies AC#3 'equivalent documented mechanism'). cam_scanner rule retained (unique serial UC762).
- udev/INSTALL.md: updated to document camera role resolution mechanism, removed cam_left/cam_right from rule table, added BusId-to-node table and troubleshooting for cameras.
- tests/test_vhci.py (new): 24 tests for vhci parser (real sample), _busid_to_dev_hex, resolve_busid_to_video_node, BusIdNotFoundError, VhciParseError, enumeration-order invariance, colliding-serial non-issue.
- tests/test_discovery.py: updated for usbip_busid resolution; added cam_left/cam_right vhci fixture tests, DuplicateDeviceNodeError and DuplicateUsbDeviceError invariant tests, missing-BusId NOT_ATTACHED test.
- tests/test_config.py: updated assertions for cam_left/cam_right (now identify_by=USBIP_BUSID, usb_serial=null, windows_busid present).

LIVE DISCOVERY (read-only, 2026-10-24):
  arm_left   attached node=/dev/ttyACM0  by-id=usb-1a86_USB_Single_Serial_5B91045044-if00
  arm_right  attached node=/dev/ttyACM1  by-id=usb-1a86_USB_Single_Serial_5B91030298-if00
  dexarm     attached node=/dev/ttyACM2  by-id=usb-STMicroelectronics_STM32F407ZG...-if00
  relay      attached node=/dev/ttyUSB0  by-id=usb-1a86_USB_Serial-if00-port0
  cam_left   attached node=/dev/video0   by-id=9-4  (BusId 9-4 -> local 1-5 -> /dev/video0)
  cam_right  attached node=/dev/video2   by-id=10-2 (BusId 10-2 -> local 1-6 -> /dev/video2)
  cam_scanner attached node=/dev/video4  by-id=usb-Arducam_..._UC762-video-index0

cam_left=/dev/video0 (local 1-5), cam_right=/dev/video2 (local 1-6) as specified.
No two roles share a dev node (duplicate invariant: pass).

TEST RUN: uv run pytest: 252 passed, 0 failed (222 pre-existing + 24 test_vhci + 6 new test_discovery).

Orchestrator live check 2026-10-24 ~11:45 EDT. I detached all three cameras and re-attached them in REVERSED order (10-2, then 11-4, then 9-4). The vhci local ports changed: 10-2 went to 1-5, 11-4 to 1-6, 9-4 to 1-7. Discovery followed the BusId correctly: cam_left=/dev/video4 (ID_PATH vhci usb-0:7, which is BusId 9-4), cam_right=/dev/video0, cam_scanner=/dev/video2. Before the fix, the same input labelled one node as both cameras. 252 tests pass. Follow-up observed: the first 'usbipd attach' for 10-2 twice returned without the device appearing in vhci status. A retry succeeded. attach_usb.py does not verify presence after attaching (no vhci check), so it can report OK falsely. Logged here and not fixed in AA-5.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @agent
created: 2026-10-24 14:42
---
Slice A complete. ACs #1, #2, #5, #6, #7 implemented and tested. ACs #3 (udev rules) and #4 (WSL attach helper) are Slice B and remain unchecked — they require hardware-adjacent scripts and cannot be verified without a Windows host. cam_left and cam_right have linux_by_id: null in config; Slice B should set their by-id names when writing udev rules.
---

author: @orchestrator
created: 2026-10-24 14:58
---
Reopened by orchestrator 2026-10-24 after Sonnet review e59fe9ef. (1) Task was Done with ACs #1,#2,#5,#6,#7 unchecked; they need evidence recorded. (2) AC#3 is not met: the cam_right rule is commented out, and cam_left's linux_by_id is null, so discovery can't resolve it even though a udev rule exists. (3) Safety: the relay rule matches CH340 VID:PID only. The relay switches the laser, so its identity must be pinned (e.g. port path or by-path) or flagged ambiguous, and discovery must refuse if more than one CH340 is present. (4) test_all_present_all_attached never covers cam_left in the attached state.
---

author: @orchestrator
created: 2026-10-24 15:34
---
Reopened by orchestrator 2026-10-24 11:35 EDT. MISIDENTIFICATION, verified live: BOTH openaicam 32e6:9251 cameras report USB serial 5423534724 to Linux (/sys/bus/usb/devices/1-5 and 1-6 both serial=5423534724; udevadm ID_SERIAL is identical). Windows gave the second one the port-derived instance ID 7&69F6256&0&2 because the serials collide, not because it lacks one. So /dev/v4l/by-id/usb-openaicam_openaicam_5423534724-video-index0 points at whichever camera enumerated last. Right now it points at /dev/video2, which /sys/devices/platform/vhci_hcd.0/status shows is remote BusId 10-2 = the RIGHT camera. Discovery reported cam_left=/dev/video2 and cam_right=/dev/video2: the same node, labelled as both cameras. Correct identity source: the Windows BusId, which is the physical dock port. Map it in WSL through vhci_hcd.0/status, where the dev column = busnum<<16|devnum of the remote device (00090004 -> 9-4, 000a0002 -> 10-2) and the port column gives the local 1-(port+1). The udev serial rules for the cameras are invalid. Discovery must also refuse when two roles resolve to the same node.
---

author: @orchestrator
created: 2026-10-24 15:44
---
Orchestrator follow-up (not blocking AA-5): attach_usb.py should confirm each device appears in /sys/devices/platform/vhci_hcd.0/status after attach and retry once, because usbipd attach was observed to succeed silently without the device appearing. Fold this into AA-14 or a small follow-up.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Camera role resolution redesigned around usbip_busid (physical dock port) to handle the discovered fact that both OV9281 cameras share USB serial 5423534724. Added IdentifyBy.USBIP_BUSID and inchworm/vhci.py (vhci_hcd status parser). cam_left and cam_right in hardware.yaml now use identify_by: usbip_busid with windows_busid: '9-4' / '10-2'. Discovery resolves BusId -> local sysfs device -> /dev/videoN (index 0). Invalid udev rules for cam_left/cam_right removed (udev cannot distinguish them); replaced with NOTE and discovery-as-mechanism documentation. Global duplicate invariant added: DuplicateDeviceNodeError and DuplicateUsbDeviceError raised if two roles collide. Live discovery shows cam_left=/dev/video0, cam_right=/dev/video2. 252 tests pass.
<!-- SECTION:FINAL_SUMMARY:END -->
