---
id: AA-32
title: Scan pattern and surface fit producing a leveling correction
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:13'
updated_date: '2026-10-24 18:39'
labels: []
milestone: m-5
dependencies:
  - AA-31
  - AA-12
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 32000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Individual height profiles have to become one actionable correction: the DexArm sweeps the scanner across the end of the printed tube, the profiles are stitched into a surface, and a fit yields the Z offset and the two tilt angles that the print frame needs. The tube is curved and its end face is the print interface, so the fit must represent that geometry rather than assuming a flat bed.

The output must also be trustworthy enough to act on autonomously, which means the fit has to report its own quality and refuse to emit a correction when the data does not support one.

See doc-1 sections 1, 3 and 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A scan pattern moves the DexArm through a documented path at the calibrated standoff, capturing profiles with the arm settled at each sample, and completes within a recorded duration
- [ ] #2 Profiles are stitched using DexArm position feedback into a single point set in the print frame
- [ ] #3 A geometric fit appropriate to the printed tube end (documented and justified, for example plane plus tube-axis or cylinder fit) yields a Z offset and two tilt angles with residuals and an estimated uncertainty
- [ ] #4 The fit rejects outliers and refuses to emit a correction when coverage, residual or uncertainty thresholds are not met, returning a typed failure instead
- [ ] #5 Repeatability of the computed correction is measured over at least 10 scans of an undisturbed part and reported, and meets or explains its relationship to a 0.2 mm Z target
- [ ] #6 A known tilt and a known Z shift introduced deliberately are recovered to within a reported tolerance
- [ ] #7 The laser is on only during capture and off for the rest of the sequence, enforced via AA-29
- [ ] #8 Tests cover stitching, fitting on synthetic surfaces with known parameters, outlier rejection and the refusal path
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete (software half).

CHECKED ACs (with evidence):
- AC#2: stitching via test_leveling.py::TestStitchProfiles — 23 tests pass (pytest output).
- AC#3: plane fit Z+rx+ry with residuals and covariance uncertainty — test_known_tilt_z_recovery: z_err=0.0004mm (true=80mm), rx_err=0.0022deg (true=5°), ry_err=0.0116deg (true=3°). Justified plane fit model in leveling.py docstring.
- AC#4: LevelingRefused on insufficient coverage, high residual, high uncertainty, no inliers — tests pass.
- AC#7: laser on only during capture — TestRunScanLaserEnforcement: laser off between stops AND on exception paths confirmed in tests.
- AC#8: synthetic stitching, known-tilt recovery, outlier rejection (200/230 correct), coverage refusal, laser enforcement — 23 tests pass.

UNCHECKED ACs (need owner-supervised hardware session):
- AC#1: scan duration on hardware — needs owner-supervised hardware session: cd robot && uv run python scripts/level_scan.py --live
- AC#5: 10-scan repeatability — needs owner-supervised hardware session: cd robot && uv run python scripts/level_scan.py --live --repeats 10
- AC#6: deliberate shift recovered on hardware — needs owner-supervised hardware session: cd robot && uv run python scripts/level_scan.py --live (with deliberate Z shift in fixture)

AC#6 judgment: the synthetic tilt/Z recovery test satisfies the computation correctness (z_err=0.0004mm, rx_err=0.002deg). However AC#6 explicitly says 'introduced deliberately' which implies a real hardware fixture change; therefore kept unchecked for the hardware portion.

Files: src/inchworm/leveling.py (NEW), tests/test_leveling.py (NEW), scripts/level_scan.py (NEW), src/inchworm/config.py (AA-32 markers appended), config/hardware.yaml (AA-32 markers appended).

Orchestrator review 2026-10-24 (P0 functional/safety): run_scan used 'with laser:' and assumed __enter__ switches the laser on. The real LaserDriver.__enter__ does nothing; only __exit__ switches it off. So real scans would have captured with the laser DARK, and the mock laser hid this by turning on in __enter__. Fixed: run_scan calls laser.on() explicitly inside the context (live only). The mock now mirrors the real driver, the interleave test runs the live path, and the new test test_run_scan_real_laser_driver_on_during_capture_off_after uses the real LaserDriver and checks the relay bytes (on during each of 4 captures, OFF last). 1065 passed.
<!-- SECTION:NOTES:END -->
