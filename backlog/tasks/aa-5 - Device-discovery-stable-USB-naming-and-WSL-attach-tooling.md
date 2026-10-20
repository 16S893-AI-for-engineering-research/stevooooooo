---
id: AA-5
title: 'Device discovery, stable USB naming and WSL attach tooling'
status: To Do
assignee: []
created_date: '2026-10-20 21:08'
updated_date: '2026-10-20 21:08'
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
- [ ] #1 A discovery command reports every expected device with its stable identity (serial number, USB path or by-id symlink) and maps it to its logical role (arm_left, arm_right, cam_left, cam_right, cam_scanner, relay, dexarm)
- [ ] #2 Logical-role to device resolution is driven by the hardware config and never by raw enumeration index; swapping USB port order does not change role assignment
- [ ] #3 udev rules or an equivalent documented mechanism give persistent by-id or by-role symlinks for the serial devices and cameras
- [ ] #4 A scripted helper attaches all required devices to WSL via usbipd and reports per-device success or failure, exiting nonzero if any required device is missing
- [ ] #5 When a device is absent or unattached, the error names the device role and prints the exact remediation command instead of failing with a bare file-not-found
- [ ] #6 Discovery output is also available as machine-readable JSON for use by later tooling
- [ ] #7 Tests cover role resolution and the missing-device error path using fixtures, with no hardware attached
<!-- AC:END -->
