---
id: AA-14
title: Mono camera capture pipeline for OV9281 devices
status: To Do
assignee: []
created_date: '2026-10-20 21:10'
labels: []
milestone: m-1
dependencies:
  - AA-6
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 14000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Three OV9281 mono global-shutter cameras serve two very different jobs: wrist observation for the learned insertion policy and laser-line capture for leveling. Both need deterministic, timestamped frames with manual exposure control, and both must respect the single-camera USB rule. Laser-line extraction in particular needs manual exposure and gain, since auto-exposure will bloom the line or lose it against ambient light.

Build one capture layer both consumers share so exposure handling, timestamping and arbiter compliance are not reimplemented twice.

See doc-1 sections 4 and 6.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Capture opens any configured camera by logical role through the USB arbiter, and streaming without holding the arbiter lock is impossible by construction
- [ ] #2 Resolution, frame rate, exposure and gain are explicitly settable, auto-exposure and auto-gain can be disabled, and the applied settings are read back and logged
- [ ] #3 Frames are delivered as single-channel arrays with a monotonic capture timestamp and a frame counter, and dropped or corrupt frames are detected and reported rather than silently skipped
- [ ] #4 Achieved frame rate and latency are measured for each camera role at the settings used downstream and recorded in task notes
- [ ] #5 A snapshot and a short-recording CLI exist for any camera role, writing to the gitignored session artifact directory
- [ ] #6 Switching between two camera roles in sequence fully closes the first device before opening the second, verified by a test
- [ ] #7 Tests cover arbiter enforcement, settings read-back, frame metadata and the sequential-switch path against the mock backend
<!-- AC:END -->
