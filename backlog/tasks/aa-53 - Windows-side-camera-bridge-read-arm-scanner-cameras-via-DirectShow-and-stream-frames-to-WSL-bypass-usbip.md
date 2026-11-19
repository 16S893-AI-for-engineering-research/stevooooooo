---
id: AA-53
title: >-
  Windows-side camera bridge: read arm/scanner cameras via DirectShow and stream
  frames to WSL (bypass usbip)
status: To Do
assignee: []
created_date: '2026-11-19 18:56'
updated_date: '2026-11-19 19:01'
labels:
  - camera
  - hardware
dependencies: []
priority: high
ordinal: 53000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Measured 2026-11-19: over usbip the arm cameras deliver only ~5-8 new images/s (new image every ~192 ms, rest exact duplicates; plus ~15% truncated JPEG frames). In Windows (DirectShow, same dock/cable) the same cameras deliver 60 new images/s, 0 duplicates (C:\Users\steph\inchworm_diag\win_report.json). Fix: a Windows program reads cameras via DirectShow and streams frames to the WSL robot code over loopback; servos and DexArm stay on usbip. Single-camera invariant (AGENTS.md 4) must hold on both sides. Step 1 is a throwaway prototype gate (>=30 new images/s in WSL); step 2 only proceeds if it passes.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Prototype gate: a Windows DirectShow reader streaming the left arm camera to WSL delivers >= 30 new (non-identical) images/s in WSL over >= 10 s, measured and logged
- [ ] #2 Windows camera program kept in the repo (robot/windows/), DirectShow only, binds to loopback/WSL-reachable address only, serves at most one camera stream at a time
- [ ] #3 Robot-side bridge camera backend in robot/src/inchworm/ selectable in config/hardware.yaml (bridge vs usbip); CameraArbiter single-camera rule, corrupt-frame handling and recorder/LeRobot paths unchanged above it
- [ ] #4 Camera identity (cam_left, cam_right, scanner) mapped by stable Windows device path/port, not index or serial; documented in doc-1
- [ ] #5 Tests with a fake bridge server cover frames, reconnect/failure, and refusal of a second concurrent stream; both suites pass (robot and robot/lerobot_env)
- [ ] #6 Live measurement through the production backend: left wrist camera >= 25 new images/s in a 30 fps recording loop, report saved
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
Step 1 prototype PASSED 2026-11-19 (agent run, Opus 5.5): Windows DirectShow -> TCP -> WSL, 640x480, JPEG q90 re-encode (~16 kB/frame). dshow1 and dshow3 (the two arm cameras) each 60.3 new images/s over 12 s, 0 of ~724 identical consecutive frames, 0 decode failures, 0 seq gaps. WSL NAT networking (WSL 172.26.80.62, Windows 172.26.80.1): Windows->WSL inbound listen timed out (likely firewall); reverse direction (WSL listens, Windows server connects out) worked. Windows CPU 24-30% of one core, WSL 8-10%. Open: DirectShow index for cam_left unresolved; camera's own JPEG not passed through (OpenCV returned decoded frames); latency unmeasured (clock offset). Scripts: C:\Users\steph\inchworm_diag\proto_bridge_server.py, proto_bridge_client.py (copy). LeRobot 0.6.1 has a ZMQCamera + image_server (binds tcp://*, re-encodes, no single-camera limit) - reuse its message format or approach where sensible.
<!-- SECTION:NOTES:END -->
