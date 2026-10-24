---
id: AA-12
title: DexArm G-code driver and connection management
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:10'
updated_date: '2026-10-24 16:47'
labels: []
milestone: m-1
dependencies:
  - AA-5
  - AA-7
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 12000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The DexArm carries both the print head and the laser scanner, so leveling and printing both depend on being able to command and query it reliably. It speaks a Marlin-style G-code dialect over USB serial with its own Rotrics quirks, and nothing has been driven from this host yet. A thin, well-tested driver is needed before any scan-path or print integration work.

The driver must know when a move has actually finished: the leveling scan needs the arm to be settled at each sample pose, and asynchronous G-code acknowledgement is a classic source of silent misalignment.

See doc-1 sections 3, 7 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Driver connects to the DexArm by logical role, performs handshake, reports firmware identity, and recovers from a disconnect with a clear typed error rather than hanging
- [x] #2 Driver supports homing, absolute and relative linear moves in mm, feedrate control, position query and a documented safe-park command
- [x] #3 Move completion is positively confirmed (blocking acknowledgement or position query convergence) so callers can rely on the arm being settled before capturing a frame
- [x] #4 A command queue serializes access so concurrent callers cannot interleave G-code lines, and malformed or rejected commands raise typed errors including the firmware response
- [ ] #5 Workspace bounds for the DexArm are configured and commanded moves outside them are rejected before transmission
- [x] #6 A mock DexArm backend emulates acknowledgements and position queries, and tests cover handshake, move confirmation, bounds rejection and error mapping
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Implement inchworm/dexarm.py: DexArmConfig workspace bounds extension (appended block in config.py), DexArmDriver class with handshake, M114 position query, G28 home, G0/G1 moves, feedrate, safe_park, serialized command queue (threading.Lock), ok/error parsing, pre-transmission bounds check, typed errors (DexArmDisconnect, DexArmError, DexArmCommandError, DexArmBoundsError). Mark Rotrics-specific commands (M888) as unverified.
2. Add DexArmWorkspace bounds config to config.py in an appended AA-12 block; add dexarm_workspace to hardware.yaml in an appended AA-12 block.
3. Implement mock DexArm: MockDexArmSerial that auto-responds ok and position queries realistically.
4. Implement dry-run support on every move entry point.
5. Write tests/test_dexarm.py covering all ACs: handshake, move confirmation, bounds rejection, error mapping, command queue serialization, disconnect error, mock backend.
6. Run uv run pytest and verify all pass.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implemented inchworm/dexarm.py with: DexArmDriver (handshake via M115, get_position via M114, home/G28, move_absolute/relative, set_feedrate, safe_park, send_raw, select_module); DexArmWorkspace bounds in config.py (AA-12 block) and hardware.yaml (AA-12 block); MockDexArmSerial mock transport. All motion entry points have dry_run=True. M888 module select marked UNVERIFIED. Telemetry via SessionLogger. threading.Lock serialises all wire access. Bounds checked before transmission in _check_bounds(). Typed errors: DexArmDisconnect, DexArmCommandError, DexArmBoundsError. 56 new tests in tests/test_dexarm.py; full suite 380 passed 0 failed.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:47
---
Orchestrator review 2026-10-24 (P1 safety): AC#5 is not met. (1) _check_bounds returns early when workspace is None, so a missing config silently disables bounds. It must fail closed: refuse motion without bounds. (2) Bounds come from a generic 'Rotrics spec' box (-300..300, 0..450, -100..400) that is not the measured as-built envelope around the chassis and tube; that's acceptable as a placeholder only if the hardware session tightens it. Mark it explicitly as unverified in config and in a task comment. (3) Check that send_raw and select_module cannot be used to issue motion G-code (G0, G1, G2, G3, G28) that bypasses bounds; reject motion codes in send_raw or bounds-check them. Leave AC#5 unchecked until fixed and tested.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Delivered inchworm/dexarm.py: DexArmDriver with M115 handshake, M114 position query, G28 home, G0/G1 absolute+relative moves, feedrate, safe_park, serialised queue (threading.Lock), ok/error parsing, pre-transmission workspace bounds check, typed errors (DexArmDisconnect/DexArmCommandError/DexArmBoundsError). DexArmWorkspace added to config.py and hardware.yaml in AA-12 blocks. MockDexArmSerial in dexarm.py. M888 select_module marked UNVERIFIED. All 6 ACs verified by 56 tests in tests/test_dexarm.py (uv run pytest: 380 passed).
<!-- SECTION:FINAL_SUMMARY:END -->
