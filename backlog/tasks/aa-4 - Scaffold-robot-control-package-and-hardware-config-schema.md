---
id: AA-4
title: Scaffold robot control package and hardware config schema
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:08'
updated_date: '2026-10-24 14:32'
labels: []
milestone: m-0
dependencies: []
references:
  - pictures/robot.png
  - sim/pyproject.toml
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 4000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
All robot work needs one shared home, one dependency environment and one declarative description of the hardware before any driver code is written. Without it every later task invents its own serial-port constants, pitch numbers and calibration file format, and nothing composes.

Mirror the existing `sim/` package conventions (uv-managed, `src/` layout, pytest). The hardware description must be data, not code, so calibration results produced by later tasks can be written back without touching logic. A mock/simulated backend is mandatory because most development happens with no USB devices attached to WSL.

See doc-1 sections 7 and 8 for the host environment and software conventions.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A uv-managed Python package exists at robot/ with pyproject.toml, src/ layout and tests/, and uv run pytest passes from robot/
- [x] #2 A versioned config schema (e.g. robot/config/hardware.yaml plus a typed loader) describes both SO-101 arms, the DexArm, all three cameras, the USB relay and the laser, including per-device identity fields, and rejects unknown or missing required fields with a clear error
- [x] #3 Attachment pitch 183.676 mm, laser design values (40 deg, 67 mm offset, 80 mm standoff, 0.1 mm/px) and the single-camera USB constraint are defined once in config or constants and imported everywhere else, with no duplicated literals in other modules
- [x] #4 A driver backend abstraction exists with a mock backend selectable by config or env var, so arm, camera, relay and DexArm code can be exercised with zero hardware attached
- [x] #5 Units convention (mm and degrees at interface boundaries) is documented in robot/README.md and enforced by naming or type annotations
- [x] #6 Tests cover config load success, config validation failure and mock-backend selection
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Create robot/ uv package (pyproject mirroring sim/: hatchling, src/inchworm_robot, tests/, pytest dev extra; deps pydantic v2 + pyyaml).
2. robot/config/hardware.yaml (schema_version 1): arms left/right (CH343 VID:PID + USB serial + by-id), dexarm (0483:5740 + serial), cameras left/right/scanner (VID:PID, serial or null + positional note, model observed), relay (CH340 1a86:7523, no serial, path-identified), laser (design geometry), geometry (attachment pitch), usb (max_concurrent_camera_streams=1), backend.
3. Typed loader (pydantic, extra=forbid, strict required fields) raising ConfigError with field path; schema_version check.
4. constants.py loads pitch/laser/USB constraint once from config; other modules import from there.
5. Driver backend abstraction: Backend protocol + MockBackend + HardwareBackend placeholder that refuses to open devices in AA-4; selection by config backend field overridden by ROBOT_BACKEND env var.
6. Units: README documents mm/deg at boundaries; field names suffixed _mm/_deg and typed aliases Millimeters/Degrees.
7. Tests: config load success, validation failures (unknown field, missing field, bad version), mock backend selection by config and env var, no-duplicated-literal test scanning src for 183.676 etc.
8. uv run pytest from robot/.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Verification run 2026-10-24: uv run pytest from robot/ — 44 passed in 0.33s. Output:

============================= test session starts ==============================
platform linux -- Python 3.12.15, pytest-9.1.1, pluggy-1.7.0
rootdir: /home/stevotimp/893/robot
configfile: pyproject.toml
testpaths: tests
collected 44 items

tests/test_backends.py .................                                 [ 38%]
tests/test_config.py .......................                             [ 90%]
tests/test_constants.py ....                                             [100%]

============================== 44 passed in 0.33s ==============================

AC#1: pyproject.toml (hatchling+uv), src/inchworm/ package, tests/ dir; uv run pytest passes (44/44).
AC#2: robot/config/hardware.yaml schema_version:1 covers both arms, dexarm, 3 cameras, relay, laser; config.py typed loader raises ConfigError on unknown/missing/wrong-type/bad-version.
AC#3: attachment_pitch_mm:183.676, laser design values (40 deg, 67 mm, 80 mm, 0.1 mm/px), max_concurrent_camera_streams:1 all in hardware.yaml; constants.py imports from config; test_no_duplicated_design_literals_in_src scans src/ and passes.
AC#4: Backend ABC + MockBackend + HardwareBackend (refuses to open); selectable by config backend field or INCHWORM_BACKEND env var; 17 backend tests pass.
AC#5: units.py defines Millimeters/Degrees/MillimetersPerPixel; README.md documents mm/deg convention; test_dimensional_fields_carry_unit_suffix enforces _mm/_deg/_mm_per_px suffixes on all float schema fields.
AC#6: test_load_default_config_success, test_missing_required_field_rejected (8 parametrised), test_unknown_top_level_field_rejected, test_unknown_nested_field_rejected, test_unsupported_schema_version_rejected; test_mock_backend_selected_by_config, test_env_var_overrides_config_to_mock, test_env_var_overrides_config_to_hardware.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
robot/ package fully implemented and verified. uv run pytest from robot/ passes 44/44 tests (test_backends.py 17, test_config.py 23, test_constants.py 4). All 6 ACs evidenced: (1) pyproject.toml+src layout+tests; (2) hardware.yaml schema_version:1 + typed pydantic loader with ConfigError; (3) design literals in config only, constants.py imports them, src scan passes; (4) MockBackend/HardwareBackend/env-var selection; (5) units.py aliases + README mm/deg convention + suffix enforcement test; (6) config load success, validation failure, mock-backend selection tests.
<!-- SECTION:FINAL_SUMMARY:END -->
