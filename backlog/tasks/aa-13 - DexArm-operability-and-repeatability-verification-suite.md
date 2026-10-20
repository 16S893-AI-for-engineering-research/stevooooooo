---
id: AA-13
title: DexArm operability and repeatability verification suite
status: To Do
assignee: []
created_date: '2026-10-20 21:10'
labels: []
milestone: m-1
dependencies:
  - AA-12
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: task
ordinal: 13000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Everything downstream of leveling assumes the DexArm can return to a commanded pose accurately and repeatably, because the laser triangulation result is only as good as the pose at which each frame was captured. The DexArm is a delta-style desktop arm whose accuracy at the edges of its envelope and under the added mass of the print head plus scanner assembly is unknown on this build. Measuring it now prevents attributing DexArm positioning error to the laser calibration later.

This is a measurement and reporting task: it produces numbers and a verdict, not new motion features. Measurements may use a dial indicator, calipers or repeated laser-based observations, and the method used must be recorded.

See doc-1 sections 3, 5 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A repeatable test script exercises homing, a set of poses spanning the usable envelope (including the scan standoff region around 80 mm from the part), and return-to-reference, logging commanded versus measured position each time
- [ ] #2 Unidirectional and bidirectional repeatability at the reference pose are quantified over at least 20 cycles and reported with mean, standard deviation and worst case per axis
- [ ] #3 Positioning error is characterized at a minimum of five poses across the envelope and the results are recorded in the task notes and in a report artifact
- [ ] #4 Repeatability is measured both with and without the print-head plus scanner payload attached, and any sag or payload-dependent offset is quantified
- [ ] #5 Thermal or warm-up drift is checked by repeating the reference pose measurement after a documented continuous-motion interval
- [ ] #6 The report states explicitly whether measured repeatability is within the 0.1 mm per pixel leveling sensitivity budget, and if not, names the limiting axis and recommends a mitigation
- [ ] #7 Measurement method, fixturing and instrument used are documented well enough to repeat the suite after any mechanical change
<!-- AC:END -->
