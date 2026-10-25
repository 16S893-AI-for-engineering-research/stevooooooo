---
id: AA-14
title: Mono camera capture pipeline for OV9281 devices
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:10'
updated_date: '2026-10-25 19:36'
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
- [x] #1 Capture opens any configured camera by logical role through the USB arbiter, and streaming without holding the arbiter lock is impossible by construction
- [x] #2 Resolution, frame rate, exposure and gain are explicitly settable, auto-exposure and auto-gain can be disabled, and the applied settings are read back and logged
- [ ] #3 Frames are delivered as single-channel arrays with a monotonic capture timestamp and a frame counter, and dropped or corrupt frames are detected and reported rather than silently skipped
- [x] #4 Achieved frame rate and latency are measured for each camera role at the settings used downstream and recorded in task notes
- [x] #5 A snapshot and a short-recording CLI exist for any camera role, writing to the gitignored session artifact directory
- [x] #6 Switching between two camera roles in sequence fully closes the first device before opening the second, verified by a test
- [x] #7 Tests cover arbiter enforcement, settings read-back, frame metadata and the sequential-switch path against the mock backend
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Add CameraCapture class in new module src/inchworm/camera_capture.py:
   - Constructible ONLY inside the CameraArbiter context (accepts CameraTransport + metadata)
   - Settings: resolution (width/height), fps, exposure, gain, auto_exposure, auto_gain
   - Reads back applied settings and logs via AA-7 SessionLogger telemetry
   - Frame delivery: FrameResult(data=np.ndarray[H,W] uint8 grayscale, ts_monotonic, counter)
   - Drop/corrupt detection: decode failures flagged, counter gaps flagged
   - Arm cameras (32e6:9251) output MJPEG only; scanner (0c45:6366) outputs MJPEG/YUY2
   - HardwareBackend opens real V4L2 devices via OpenCV with CAP_V4L2 backend
   - Uses device node resolved by AA-5 discovery (never OpenCV index)
2. Extend MockCameraTransport in backends.py to produce synthetic MJPEG frames (encoded as real MJPEG bytes via cv2 imencode or a pure-python fallback) with configurable settings read-back, frame counter, monotonic timestamps, configurable decode failures for drop/corrupt testing.
3. Add CameraCapture to HardwareBackend._open_camera as a V4L2 OpenCV transport.
4. Add opencv-python-headless to pyproject.toml in AA-14 markers.
5. Add sequential-switch test (close before open).
6. Create scripts/camera_probe.py (dry-run default, --live flag) for AC#4.
7. Write tests/test_camera_capture.py covering ACs #1,#2,#3,#5,#6,#7.
8. Session artifact dir: robot/sessions/ (already gitignored).
9. CLI snapshot/record commands in scripts/camera_probe.py (also AC#5).
10. Leave AC#4 unchecked with hardware comment.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete (software half):
- New module: src/inchworm/camera_capture.py — CameraCapture, open_capture(), FrameResult, CameraSettings, AppliedSettings, ArbiterLockRequiredError, CaptureError, _decode_to_gray()
- New module: src/inchworm/camera_cli.py — 'snapshot' and 'record' subcommands, dry-run by default
- New script: scripts/camera_probe.py — FPS/latency measurement, dry-run default, --live flag
- backends.py (AA-14 markers): CameraTransport.apply_settings() / read_frame_with_counter() defaults; MockMjpegCameraTransport (MJPEG synthesis, settings read-back, counter, skip/corrupt hooks); MockMjpegBackend; V4L2CameraTransport (hardware path, AA-5 discovery)
- pyproject.toml (AA-14 markers): opencv-python-headless>=4.8, numpy>=1.24; inchworm-camera entry point
- tests/test_camera_capture.py: 48 tests, all pass (509 total pass)
- Hardware notes recorded: arm cameras (32e6:9251) MJPEG-only; scanner (0c45:6366) MJPEG+YUY2; no native Y8/GREY — grayscale produced in software
- AC#4 unchecked: needs owner-supervised hardware session: uv run python scripts/camera_probe.py --live --role camera_left_arm --frames 120

Orchestrator hardening 2026-10-24: V4L2CameraTransport.open() refuses to open unless CameraArbiter.acquire() holds the global lock for that device (camera_arbiter.global_lock_held_for). That closes a bypass where scripts could stream without the lock. Test: test_v4l2_transport_refuses_open_without_global_lock. 583 passed.

AC#4 measurement 2026-10-25 (live, one camera at a time via the arbiter, scripts/camera_probe.py --live --frames 300, 640x480 requested = downstream teleop/policy setting, robot/config hardware.yaml teleop.camera_frame_*):
- camera_left_arm: applied 640x480@60, achieved 59.85 fps, latency mean 16.74 / p95 31.42 / max 41.59 ms, dropped 0.
- camera_right_arm: applied 640x480@60, achieved 54.62 fps, latency mean 18.35 / p95 32.81 / max 63.94 ms, dropped 0.
- camera_scanner 640x480: applied @100, achieved 101.49 fps, latency mean 9.88 / p95 19.64 / max 28.40 ms.
- camera_scanner 1280x800 (full sensor; scanner downstream resolution not yet fixed by AA-30/31): achieved 59.30 fps, latency mean 16.84 / p95 29.84 / max 48.54 ms.
FIX found during measurement: without an explicit MJPG FOURCC the scanner fell back to YUYV, ignored the size and ran at 1280x720@~10.7 fps. backends.py now requests MJPG before the size.
DATA-QUALITY FINDING (follow-up, not hidden): libjpeg printed 'Corrupt JPEG data: premature end of data segment' (truncated frames) for left-arm 36/300 and scanner-1280x800 203/300; right-arm and scanner-640 had 0. The probe's 'corrupt' counter reported 0, so it does NOT detect truncated-but-decodable frames. Likely USB/IP bandwidth on 1280x800 MJPEG. It needs truncated-frame detection before scanner use (AA-30/31).

AC#3 UNCHECKED 2026-10-25 by the orchestrator. Live evidence contradicts it: truncated MJPEG frames ('premature end of data segment') decode without error and are NOT counted as corrupt (left arm 36/300, scanner 1280x800 203/300). AC#3 was previously checked on mock evidence only. Fix needed: detect truncated JPEG (missing EOI marker 0xFFD9 / libjpeg warning) and report it as corrupt; then re-measure.

Truncated-frame detection attempt 2026-10-25: added an SOI/EOI check in _decode_to_gray (kept, with a unit test; it catches truncated JPEG bytes when the transport supplies raw JPEG). The raw passthrough via CAP_PROP_CONVERT_RGB=0 was REVERTED: on the openaicam arm cameras OpenCV then returns misaligned buffers (no SOI/EOI, every frame unreadable). So hardware truncated frames are still undetected and AC#3 stays unchecked. Next options: read V4L2 directly (v4l2 mmap buffer bytesused), or detect them via the libjpeg warning or the grey-tail pattern. Live probe after the revert: L 59.4 fps, R 53.3, scanner 99.7, reported corrupt 0 (as before).
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @agent
created: 2026-10-24 17:41
---
AC#4 unchecked — needs owner-supervised hardware session: uv run python scripts/camera_probe.py --live --role camera_left_arm --frames 120 (repeat for camera_right_arm and camera_scanner). Record achieved FPS, mean/p95/max latency as task notes.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented the camera capture pipeline (software half). New modules: camera_capture.py (CameraCapture, open_capture, FrameResult, settings/read-back, drop/corrupt detection via MJPEG decode), camera_cli.py (snapshot/record CLI writing to robot/sessions/). New script: scripts/camera_probe.py (dry-run default, --live flag for AC#4). backends.py extended (AA-14 markers): CameraTransport.apply_settings()/read_frame_with_counter() defaults; MockMjpegCameraTransport with synthetic MJPEG, configurable counter gaps and corrupt bytes; MockMjpegBackend; V4L2CameraTransport (hardware path via AA-5 discovery). pyproject.toml: opencv-python-headless, numpy, inchworm-camera entry point. 48 new tests all pass (509 total). AC#1-3,5-7 verified by test output; AC#4 stays unchecked — needs owner-supervised hardware session (see comment).
<!-- SECTION:FINAL_SUMMARY:END -->
