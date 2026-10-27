---
id: AA-38
title: Measure concurrent camera streaming on the USB 3.2 dock
status: To Do
assignee: []
created_date: '2026-10-24 16:07'
updated_date: '2026-10-27 20:06'
labels: []
milestone: m-1
dependencies:
  - AA-14
type: task
ordinal: 38000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner replaced the USB 2.0 dock with a USB 3.2 dock and asked whether cameras could now stream at once. All three cameras are USB 2.0 High-Speed UVC devices (doc-1 s4), so their traffic shares the dock's 480 Mbps USB 2.0 path, and usbip adds overhead. Per-camera isochronous reservations from the descriptors: openaicam up to 2x1024 B per microframe (~131 Mbps), Arducam up to 3x1024 B (~197 Mbps). Two arm cameras may fit; all three are marginal. The single-camera invariant (AGENTS.md 4, doc-1 s6) stays until this measurement and an explicit owner decision say otherwise. This task only measures; it does not relax the arbiter.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 With the arbiter bypassed only inside an explicit, owner-supervised test harness, two-camera and three-camera concurrent capture through WSL/usbipd is measured at the intended resolutions and formats, recording delivered FPS, dropped or corrupt frames and bus errors over at least 5 minutes per combination
- [ ] #2 Single-camera baselines for each camera are measured with the same method for comparison
- [ ] #3 Serial control latency for both arms is measured during the heaviest passing camera combination and compared against the single-camera baseline
- [ ] #4 Results and a recommendation are written to doc-1 s6; any relaxation of the single-camera invariant is recorded as a decision record approved by the owner before the arbiter is changed
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 Acceptance criteria are each verified by concrete evidence (test output, measurement, or a logged hardware run) and not by code inspection or intent
- [ ] #2 Acceptance criteria and task description were NOT altered to match the implementation; any scope or criterion change was approved by the owner first and is recorded in the task
- [ ] #3 Hardware facts used came from doc-1; any corrected measurement was written back to doc-1 via `backlog doc update doc-1` in the same task
- [ ] #4 Tests pass via `uv run pytest` from robot/, including mock-backend tests that run with no hardware attached
- [ ] #5 Safety invariants still hold and are covered by a test: single-camera USB exclusivity, always-one-arm-holding, gripper servo commanded only by the AA-17 state machine, laser off on every exit path
- [ ] #6 Implementation notes record measured values, deviations from plan, and anything deferred
<!-- DOD:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Owner 2026-10-27: keep the single-camera rule for now, even after the dock change. The arbiter stays unchanged; this task remains a measurement task only.
<!-- SECTION:NOTES:END -->
