---
id: AA-14
title: Mono camera capture pipeline for OV9281 devices
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:10'
updated_date: '2026-11-06 03:56'
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
- [x] #3 Frames are delivered as single-channel arrays with a monotonic capture timestamp and a frame counter, and dropped or corrupt frames are detected and reported rather than silently skipped
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

AC#3 fix (owner chose option 2, 2026-11-05): detect libjpeg corruption warnings ('Corrupt JPEG data: premature end of data segment') by capturing OS fd 2 around cap.read() in V4L2CameraTransport; flag the frame corrupt, deliver an empty array, log the reason; probe --save-samples writes flagged/clean PNGs for owner visual confirmation. Live re-run required before checking AC#3.

Revision (owner, 2026-11-06): only the scanner withholds frames with a libjpeg warning; arm cameras deliver them and log the warning. Damaged scanner frames are skipped, never raised: CameraCapture.grab_valid() retries up to 10 frames and returns the last result without raising; level_scan live capture uses it.
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

AC#3 option 2 implemented 2026-11-05 (software, mock-tested; NOT yet live): backends.py _StderrCapture (dup2 fd 2 to a temp file around cap.read(), restores fd 2, re-emits captured text so nothing is swallowed) + jpeg_corruption_warning(); V4L2CameraTransport sets last_integrity_error / last_rejected_frame and returns b'' -> CameraCapture.grab() reports corrupt=True, data 0x0, camera_frame_corrupt event with payload.reason. camera_probe.py: per-reason corrupt counts in the report, --save-samples N writes <role>_corrupt_*.png (rejected frame, evidence only) and <role>_clean_*.png. Tests: 6 new in test_camera_capture.py (fake cap writes the warning to fd 2 like libjpeg), full suite passes. Note: the opencv 5.0.0.93 wheel contains the libjpeg string 'Corrupt JPEG data: premature end of data segment'; imdecode on truncated bytes in this build returns None silently, so the warning path can only be proven on hardware. AC#3 stays UNCHECKED until: cameras attached via usbipd, then for each role: cd robot && uv run python scripts/camera_probe.py --live --role <role> --frames 300 --save-samples 5 (scanner also --width 1280 --height 800). Pass = corrupt count roughly matches the libjpeg lines printed (expected L ~36/300, scanner 1280x800 ~203/300), flagged PNGs show the grey tail and clean PNGs do not (owner visual check).

Live run 2026-11-06 (agent ran it; cameras attached via usbipd 9-4/10-2/11-4; probe --live --frames 300 --save-samples 5): left arm corrupt 39/300 measured (41 libjpeg lines incl. warm-up, 41 corrupt events: 1:1), right arm 48/300 (49/49), dropped 0. Warning detection on hardware WORKS (every libjpeg line became a corrupt event; fd-2 capture overhead 0.07 ms/frame). BUT the visual check FAILED to distinguish: flagged and clean PNGs look identical, and BOTH have the same flat value-128 band in the last 8 rows (rows 472-479) with no larger grey tail. So the 'premature end' frames are missing at most the last MCU row, and clean frames show the same band; meaning unclear (sensor/camera artifact, or every frame short by one MCU row while libjpeg only warns on some). Achieved fps was 20.6 / 20.5 (was 59.9 / 54.6 on 2026-10-25) with auto exposure on (exposure read-back 313), dark scene (mean 33); the fd-2 capture is not the cause. Scanner not yet run. AC#3 NOT checked: stopped to ask the owner (two analysis probes did not explain the band). Artifacts: robot/sessions/20261106_030126_e8a6, 20261106_030142_d8be.

Owner 2026-11-06: viewed both arm cameras in the Windows Camera app; they 'look good' and the grey band at the bottom of the frame is a real object in view (so the 8 flat rows in the saved PNGs are scene, not missing data). Room was dark during the probe run (explains auto-exposure ~31 ms and ~20 fps). Scanner camera released to Windows (usbipd detach 11-4) for the owner to view.

Live cross-check 2026-11-06 (owner: all three cameras look good in the Windows Camera app). Per-frame comparison of the libjpeg flag against the decoded image: scanner 1280x800 flagged 64/300, every flagged frame almost entirely flat grey (784-792 of 800 rows), clean frames 0 flat rows: real damage, detection correct. Scanner 640x480: 0 flagged. Arm cameras 640x480: flagged 25/300 (L) and 40/300 (R), but flagged frames show only the normal 8-row band (a real object in view per the owner): the warning fires on frames that look fine, so rejecting them discards good frames (false positives for the arm cameras). Arm 'clean' frames sometimes show large flat areas (dark room, flat-row measure unreliable; not investigated). Asked the owner whether to keep rejecting on the arm cameras. AC#3 not checked.

Owner decision 2026-11-06: withhold damaged frames for the DexArm (scanner) camera only; do not throw an error or stop. Implemented: backends._REJECT_CORRUPT_DEVICES={'camera_scanner'}, V4L2CameraTransport.reject_corrupt; arm cameras deliver warned frames (corrupt=False) and emit camera_frame_integrity_warning (INFO); CameraCapture.grab_valid(max_attempts=10) skips damaged frames and never raises; scripts/level_scan.py live capture_fn now uses grab_valid (returns None, scan continues, instead of the old raw imdecode, which raised cv2.error on empty bytes). camera_probe reports 'withheld' and 'warned, kept'. 4 new tests; full suite 1650 passed. LIVE 2026-11-06 (dark room, auto exposure), per-frame check of flag against the decoded image: scanner 1280x800 clean 263 (0 flat rows), withheld 37 (all 792/800 rows flat grey); left arm clean 249, warned-kept 51 (normal 8-row band only); right arm clean 271, warned-kept 29. Scanner grab_valid x100 live: 0 corrupt returned, 116 frames read, shape 800x1280, no exception.

Owner decision 2026-11-06 (revision): arm cameras must also exclude bad frames. _REJECT_CORRUPT_DEVICES now covers camera_scanner, camera_left_arm, camera_right_arm. lerobot_adapter frame source and teleop_capture wrist frame now use grab_valid() (teleop previously did raw imdecode of empty bytes). Live 2026-11-06 (dark room, auto exposure, 640x480): left arm grab x200: 15 corrupt, 20.0 fps; grab_valid x200: 0 corrupt returned, 236 reads, 16.7 good fps. Right arm: 33 corrupt, 18.8 fps; grab_valid: 0 corrupt, 226 reads, 16.9 good fps. Overhead = only the skipped frames (~8-17 %), detection itself 0.07 ms/frame. Full suite 1651 passed.
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
Camera capture pipeline complete. AC#3 closed 2026-11-06: OpenCV hides truncated MJPEG frames, so the V4L2 transport captures libjpeg's 'Corrupt JPEG data' warning (fd 2 around cap.read(), re-emitted, 0.07 ms/frame). Owner decision: the scanner (DexArm) camera withholds warned frames (live: all 37 withheld 1280x800 frames were 99% flat grey, 263 clean frames had none); arm cameras deliver warned frames and only log them (owner visual check: they look fine). Damaged frames never raise or stop a run: CameraCapture.grab_valid() skips them (live: 100 calls, 0 corrupt returned, no exception); level_scan uses it. Verified by 10 new tests, full suite 1650 passed, and live per-frame runs on all three cameras (task notes).
<!-- SECTION:FINAL_SUMMARY:END -->
