---
id: AA-6
title: USB bandwidth arbiter enforcing single-camera streaming
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:08'
updated_date: '2026-10-24 14:55'
labels: []
milestone: m-0
dependencies:
  - AA-5
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 6000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
All arms and cameras share one USB 2.0 uplink through a single dock, so only one camera may transfer data at a time. This is a hard system invariant: if two cameras stream concurrently, frames drop or the bus stalls, which can corrupt a laser scan or an insertion policy observation mid-motion. The constraint must be enforced structurally so no later task can violate it by accident.

The autonomy loop will switch cameras repeatedly (wrist camera during insertion, scanner camera during leveling), so acquisition must be cheap, reliable and must fully close the previous device rather than just pausing it.

See doc-1 section 6.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A single process-wide and cross-process arbiter grants exclusive camera access; a second acquisition attempt while a camera is held either blocks with a timeout or fails explicitly, never silently proceeds
- [x] #2 Acquisition is exposed as a context manager (or equivalent) that opens the device on enter and fully closes and releases it on exit, including on exception
- [x] #3 Releasing a camera verifiably closes the underlying device handle rather than pausing the stream
- [x] #4 A crashed or killed holder does not permanently deadlock the arbiter; stale locks are detected and recovered with a logged warning
- [x] #5 Arm and relay serial traffic is explicitly permitted concurrently with one camera stream and is not gated by the arbiter
- [x] #6 Tests prove mutual exclusion, timeout behaviour, exception-path release and stale-lock recovery against the mock backend
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Create robot/src/inchworm/camera_arbiter.py: cross-process exclusive camera arbiter using fcntl LOCK_EX on a per-device lock file under /tmp. Each lock file stores JSON with pid and timestamp. On acquisition: try LOCK_EX|LOCK_NB; on EAGAIN poll up to timeout seconds (default 5.0) then raise CameraArbiterError. Stale lock (dead PID) detected via os.kill(pid, 0): recover with logged WARNING. Context manager: __enter__ acquires, __exit__ releases and closes the CameraTransport. Underlying device handle fully closed (transport.close()) on every exit path including exception.
2. CameraSession context manager: wraps Backend.open_camera + arbiter, returns the live CameraTransport. Fully closes transport on __exit__.
3. add robot/tests/test_camera_arbiter.py: uses MockBackend only. Tests: mutual exclusion (second acquire raises), timeout raises CameraArbiterError, exception-path release (transport closed), stale-lock recovery (mock dead PID, expect WARNING). Does NOT open real cameras.
4. Record plan, run tests, append notes.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete (ACs #1–#4, no AC checks yet).
Added robot/src/inchworm/camera_arbiter.py: CameraArbiter, CameraSession, CameraArbiterError, CameraArbiterTimeoutError.
Added robot/tests/test_camera_arbiter.py: 11 tests covering mutual exclusion, timeout, context manager (normal + exception), transport closed on exit, stale-lock recovery with WARNING, live-lock not falsely recovered.
All 212 tests pass: uv run pytest (212 passed in 1.00s).
AC #5 (serial traffic not gated) and AC #6 (full cross-process proof) are out of this slice.

AC #5 verified: test_ac5_serial_open_while_camera_locked opens arm_left, arm_right, relay, dexarm serial concurrently while camera_left_arm lock is held — all pass. test_ac5_camera_arbiter_rejects_serial_device confirms arbiter refuses serial-class devices with BackendError. AC #6 verified: 4 cross-process (multiprocessing fork) tests added and passing — test_ac6_cross_process_mutual_exclusion (second process denied), test_ac6_cross_process_timeout (CameraArbiterTimeoutError after ~0.5s), test_ac6_cross_process_exception_path_release (exception in body → lock released, handle closed, re-acquire succeeds), test_ac6_cross_process_stale_lock_recovery_after_sigkill (SIGKILL'd holder → WARNING with dead PID, re-acquire succeeds). Full suite: 218 passed in 2.04s (uv run pytest).

Reopened fix (2026-10-24): Enforced ONE global cross-process camera lock (_global_camera.lock) in camera_arbiter.py. Any camera acquisition — regardless of device name — must hold this single lock, directly enforcing doc-1 s6. Per-device lock files removed; owner record now includes device name. Replaced test_ac1_different_cameras_are_independent (which wrongly asserted two cameras could be open simultaneously) with test_ac1_different_cameras_are_mutually_exclusive (holding left prevents right, timeout) and test_ac1_different_cameras_sequential_allowed (different cameras work sequentially). Added test_ac6_cross_process_different_device_mutual_exclusion (cross-process proof: different device still denied). Fixed _get_dead_pid: now uses subprocess.run([sys.executable, '-c', '...']) instead of os.fork(). Updated _worker_hold_then_raise to report transport.is_open via result_q after __exit__; test_ac6_cross_process_exception_path_release asserts result['is_open_after_exit'] is False. DEFAULT_TIMEOUT_S kept as 5.0 module literal — no fitting timeout field exists in config/hardware.yaml or constants.py; adding one solely for this value is not warranted. Full suite: uv run pytest → 220 passed in 2.53s. Camera arbiter: uv run pytest -v tests/test_camera_arbiter.py → 19 passed in 1.66s.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 14:52
---
Reopened by orchestrator 2026-10-24. Sonnet review (b32239b8) found camera_arbiter.py locks per device (<device>.lock), and test_ac1_different_cameras_are_independent asserts that camera_left_arm and camera_right_arm can be open at the same time. That breaks AC#1 (exclusive when *a* camera is held), doc-1 s6 and AGENTS.md invariant 4 (one camera streaming at a time). AC#1 and #6 unchecked until one global cross-process camera lock is enforced and tested.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Enforced ONE global camera lock (_global_camera.lock) in camera_arbiter.py — any camera acquire holds this single file lock, blocking all other cameras (doc-1 s6). Replaced wrong test_ac1_different_cameras_are_independent with two correct tests: mutual exclusion across different devices (in-process + cross-process) and sequential access. Fixed _get_dead_pid to use subprocess, not os.fork(). Cross-process exception test now verifies transport.is_open via Queue. DEFAULT_TIMEOUT_S retained as module literal (no config field exists). All 6 ACs verified: 220 tests pass (uv run pytest), including 19 camera-arbiter tests.
<!-- SECTION:FINAL_SUMMARY:END -->
