---
id: AA-4
title: Scaffold robot control package and hardware config schema
status: To Do
assignee: []
created_date: '2026-10-20 21:08'
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
- [ ] #1 A uv-managed Python package exists at robot/ with pyproject.toml, src/ layout and tests/, and uv run pytest passes from robot/
- [ ] #2 A versioned config schema (e.g. robot/config/hardware.yaml plus a typed loader) describes both SO-101 arms, the DexArm, all three cameras, the USB relay and the laser, including per-device identity fields, and rejects unknown or missing required fields with a clear error
- [ ] #3 Attachment pitch 183.676 mm, laser design values (40 deg, 67 mm offset, 80 mm standoff, 0.1 mm/px) and the single-camera USB constraint are defined once in config or constants and imported everywhere else, with no duplicated literals in other modules
- [ ] #4 A driver backend abstraction exists with a mock backend selectable by config or env var, so arm, camera, relay and DexArm code can be exercised with zero hardware attached
- [ ] #5 Units convention (mm and degrees at interface boundaries) is documented in robot/README.md and enforced by naming or type annotations
- [ ] #6 Tests cover config load success, config validation failure and mock-backend selection
<!-- AC:END -->
