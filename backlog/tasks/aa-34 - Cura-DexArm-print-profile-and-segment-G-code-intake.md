---
id: AA-34
title: Cura DexArm print profile and segment G-code intake
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:14'
updated_date: '2026-10-24 19:15'
labels: []
milestone: m-6
dependencies:
  - AA-12
  - AA-33
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 34000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Slicing is done by the owner in Cura, and a correct DexArm machine profile has to exist there first. For the agent side, what matters is a defined intake: a sliced segment G-code file must be validated against the DexArm workspace and the current leveled print frame before it is streamed, because a segment sliced for the wrong origin or with absolute Z from a previous hop would crash the nozzle into the part.

This task owns the boundary between externally sliced G-code and the robot, including the request to the owner for a slice when one is needed.

See doc-1 sections 3 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A documented intake accepts a sliced segment G-code file and validates it for dialect compatibility, workspace bounds, feedrate and temperature limits, and expected coordinate mode before any streaming
- [x] #2 Validation rejects a segment whose coordinate origin or Z reference is inconsistent with the current leveled print frame, naming the specific inconsistency
- [x] #3 The leveling correction from AA-33 is applied to the segment transform rather than requiring a re-slice per hop, and the applied transform is logged with the segment
- [x] #4 Streaming to the DexArm reports progress, confirms completion, and aborts safely on firmware error with the nozzle retracted and the laser off
- [ ] #5 Required Cura profile settings for this DexArm and this nozzle and material are documented in the repository so a slice can be reproduced
- [x] #6 A dry-run mode parses and validates a segment and reports the planned bounding box without commanding motion
- [x] #7 Tests cover validation acceptance, each rejection reason and dry-run output on sample G-code files
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
AC#1 checked: parse_segment validates dialect (G28 forbidden, unrecognised opcodes), workspace bounds, feedrate, temperature limits, and coordinate mode tracking. Test output: 62 passed in 0.40s (uv run pytest tests/test_gcode_intake.py).
AC#2 checked: GCodeOriginError raised with reason='z_origin_mismatch', naming first_layer_z, frame_z0, tolerance. Test: TestValidateSegment::test_z_origin_mismatch passes.
AC#3 checked: PrintFrameTransform protocol defined; validate_segment applies frame.apply() before workspace check; frame_id logged via debug logging. Tests: test_frame_transform_applied_before_bounds, test_frame_id_is_logged, test_dry_run_with_named_frame all pass.
AC#4 checked: stream_segment catches DexArmCommandError and general exceptions; calls move_relative for retract and laser.off() on abort; raises GCodeStreamAborted. Tests: test_abort_retracts_nozzle, test_abort_turns_laser_off pass.
AC#5 UNCHECKED: docs/cura_dexarm_profile.md created with required Cura settings, machine size from DexArm workspace config, Marlin flavour, start/end G-code constraints (no G28 mid-segment, absolute/relative E). Nozzle and material values marked 'owner-to-confirm'. Needs owner-supervised hardware session to confirm nozzle type, material, temperatures, speed limits.
AC#6 checked: dry_run_segment parses and validates, prints bounding box and estimated time without commanding motion. Tests: TestDryRunSegment suite passes.
AC#7 checked: fixture files under robot/tests/data/: simple_segment.gcode, out_of_bounds.gcode, g28_mid_segment.gcode, temp_exceeded.gcode, feedrate_exceeded.gcode, bed_temp_exceeded.gcode. 62 tests covering all rejection reasons and dry-run output.

Orchestrator verification 2026-10-24: `uv run pytest -q tests/test_gcode_intake.py` -> 62 passed. TestValidateSegment covers dialect, bounds, feedrate, temperature, mode and origin rejections with named reasons; TestDryRunSegment (8) covers bounding box and estimate; fixtures in tests/data. Checked #1, #2, #6, #7. #3 and #4 stay unchecked per the orchestrator comment (E axis dropped in streaming, print frame not applied to streamed coordinates, start position assumed 0,0,0, fragile abort retract). #5 needs owner nozzle and material confirmation.

AA-34 streaming fix applied (agent 2026-10-24):
1. DexArmDriver.print_move(x,y,z,e=None,f=None,*,dry_run) added in dexarm.py # --- AA-34 --- block: bounds-checked G1 carrying E/F, fail-closed (_check_bounds before any send).
2. stream_segment rewritten in gcode_intake.py # --- AA-34 --- block: (a) queries start position via driver.get_position() (M114), seeding XYZ state; falls back to (0,0,0) with a warning; (b) applies frame.apply(xyz) to every G0/G1 XYZ before calling print_move, so AA-33 leveling correction reaches every arm move; frame_id logged at stream start; (c) tracks M82/M83 extrusion mode, passes E through in both absolute and relative modes; (d) rejects G2/G3 arcs with GCodeArcRejected(reason='arc_not_linearised') — arcs accepted at parse/validate time only.
3. Abort path improved: retract via move_relative; if that raises, falls back to driver.quickstop() (M410) and logs; laser off() always attempted; GCodeStreamAborted now carries retract_ok and quickstop_attempted attributes.
4. docs/cura_dexarm_profile.md updated: G2/G3 arc streaming rejection and Cura configuration note added.
Tests added: test_retract_fails_then_quickstop, test_e_passthrough_absolute_mode, test_e_passthrough_relative_mode, test_no_e_in_move_passes_none, test_frame_transform_applied_to_streamed_moves, test_start_position_queried_via_m114, test_start_position_assumed_zero_when_get_position_unavailable, test_arc_g2_rejected_at_stream_time, test_arc_g3_rejected_at_stream_time, test_identity_frame_no_shift_in_streaming.
Test evidence: uv run pytest tests/test_gcode_intake.py -> 73 passed; uv run pytest -> 1278 passed.

Orchestrator verification 2026-10-24 after the fix lane: DexArmDriver.print_move is bounds-checked before transmit and carries E/F; stream_segment applies frame.apply() to every streamed XYZ (test_frame_transform_applied_to_streamed_moves), tracks M82/M83 (test_e_passthrough_absolute_mode/relative_mode), rejects G2/G3 at stream time (documented in cura_dexarm_profile.md), and on abort retracts with an M410 quickstop fallback (test_retract_fails_then_quickstop) with the laser off always attempted. Orchestrator hardening: a failed M114 start-position query now REFUSES in live mode instead of assuming (0,0,0) (test updated). `uv run pytest -q tests/test_gcode_intake.py` all pass; full suite 1392 passed. AC#5 stays unchecked: owner nozzle and material confirmation.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 19:01
---
Orchestrator review 2026-10-24 (P0 functional): stream_segment (1) DROPS the extruder axis: motion lines are sent as DexArmDriver.move_absolute(x,y,z) with no E, so a print would extrude nothing; G2/G3 arcs are also flattened to straight moves. (2) Does NOT apply the AA-33 print-frame correction to streamed coordinates (only at validation), so the leveling correction never reaches the arm. (3) Assumes the start position is (0,0,0) instead of querying it. (4) The abort retract via move_relative depends on M114 and bounds and can fail exactly when the firmware is in error. Fix: add a bounded 'print move' API on the DexArm driver (in an AA-34 block) that carries E/F and is bounds-checked AFTER the frame transform, query the start position, convert or reject arcs explicitly, and test the abort retract when move_relative fails (fall back to raw 'G91\nG1 Z+r' through a documented emergency path or the M410 quickstop). AC#3 and AC#4 are unchecked until then.
---
<!-- COMMENTS:END -->
