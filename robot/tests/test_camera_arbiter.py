"""Tests for inchworm.camera_arbiter — AA-6 ACs #1–#6.

All tests use MockBackend; no real hardware is accessed.
"""

from __future__ import annotations

import logging
import multiprocessing
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from inchworm.backends import BackendError, MockBackend, create_backend
from inchworm.camera_arbiter import (
    CameraArbiter,
    CameraArbiterError,
    CameraArbiterTimeoutError,
    CameraSession,
)
from inchworm.config import load_config


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def config():
    return load_config()


@pytest.fixture()
def backend(config):
    return create_backend(config)


@pytest.fixture()
def lock_dir(tmp_path):
    d = tmp_path / "cam_locks"
    d.mkdir()
    return d


@pytest.fixture()
def arbiter(backend, lock_dir):
    return CameraArbiter(backend, lock_dir=lock_dir)


# ---------------------------------------------------------------------------
# AC #1 — single-process exclusive access: second acquire blocks/fails
# ---------------------------------------------------------------------------


def test_ac1_second_acquire_raises_timeout(arbiter):
    """While one session holds the lock, a second acquire must fail, not silently proceed."""
    session1 = arbiter.acquire("camera_left_arm", timeout_s=0.5)
    session1.__enter__()
    try:
        with pytest.raises(CameraArbiterTimeoutError, match="camera_left_arm"):
            # timeout_s=0 → fail immediately; must not silently succeed.
            arbiter.acquire("camera_left_arm", timeout_s=0.0).__enter__()
    finally:
        session1.__exit__(None, None, None)


def test_ac1_lock_released_allows_second_acquire(arbiter):
    """After the first session is released, a second acquire must succeed."""
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam1:
        assert cam1.is_open

    # Now the lock is free — this must succeed without timeout.
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam2:
        assert cam2.is_open


def test_ac1_different_cameras_are_mutually_exclusive(arbiter):
    """Holding camera_left_arm must prevent camera_right_arm from being acquired.

    The global lock enforces the doc-1 s6 invariant: only one camera may
    stream at a time, regardless of which device is requested.
    """
    session1 = arbiter.acquire("camera_left_arm", timeout_s=1.0)
    session1.__enter__()
    try:
        with pytest.raises(CameraArbiterTimeoutError):
            # Different device, but the global lock is already held → must fail.
            arbiter.acquire("camera_right_arm", timeout_s=0.0).__enter__()
    finally:
        session1.__exit__(None, None, None)


def test_ac1_different_cameras_sequential_allowed(arbiter):
    """After releasing one camera, a DIFFERENT camera can be acquired.

    Ensures the global lock is properly released (not stuck on device name).
    """
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam_l:
        assert cam_l.is_open
    # lock released — now a different device must be acquirable
    with arbiter.acquire("camera_right_arm", timeout_s=1.0) as cam_r:
        assert cam_r.is_open


# ---------------------------------------------------------------------------
# AC #2 — context manager: open on enter, fully closed on exit (normal path)
# ---------------------------------------------------------------------------


def test_ac2_transport_open_inside_context(arbiter, backend):
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        assert cam.is_open
        frame = cam.read_frame()
        assert len(frame) > 0


def test_ac2_transport_closed_after_context_exit(arbiter, backend):
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        pass  # exit immediately
    assert not cam.is_open


# ---------------------------------------------------------------------------
# AC #3 — releasing closes the handle (not just pausing the stream)
# ---------------------------------------------------------------------------


def test_ac3_close_called_on_transport_on_exit(arbiter, backend):
    """Transport.close() must be called on context exit; subsequent reads must raise."""
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        pass
    # MockCameraTransport raises BackendError if read_frame() called while closed.
    with pytest.raises(BackendError, match="closed"):
        cam.read_frame()


def test_ac3_transport_not_open_after_release(arbiter, backend):
    """is_open must return False after the session exits."""
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        transport_ref = cam
    assert not transport_ref.is_open


# ---------------------------------------------------------------------------
# AC #2 (exception path) / AC #3 — closed on exception, lock released
# ---------------------------------------------------------------------------


def test_ac2_transport_closed_on_exception(arbiter, backend):
    """Transport must be fully closed even when the body raises an exception."""
    cam_ref = None
    with pytest.raises(RuntimeError, match="intentional"):
        with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
            cam_ref = cam
            raise RuntimeError("intentional")
    assert cam_ref is not None
    assert not cam_ref.is_open


def test_lock_released_after_exception_allows_reacquire(arbiter):
    """After an exception exits the context, the lock must be released."""
    with pytest.raises(RuntimeError):
        with arbiter.acquire("camera_left_arm", timeout_s=1.0):
            raise RuntimeError("intentional")

    # Lock must be free now.
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        assert cam.is_open


# ---------------------------------------------------------------------------
# AC #4 — stale lock recovery: dead PID does not permanently deadlock
# ---------------------------------------------------------------------------


def test_ac4_stale_lock_recovered_with_warning(arbiter, lock_dir, caplog):
    """A lock file claiming a dead PID must be recovered with a logged WARNING."""
    # Write a fake lock file with a PID that definitely does not exist.
    dead_pid = _get_dead_pid()
    import json as _json

    from inchworm.camera_arbiter import _GLOBAL_LOCK_FILENAME

    lock_path = lock_dir / _GLOBAL_LOCK_FILENAME
    lock_path.write_bytes(
        _json.dumps({"pid": dead_pid, "device": "camera_left_arm", "ts": time.time()}).encode()
    )

    with caplog.at_level(logging.WARNING, logger="inchworm.camera_arbiter"):
        with arbiter.acquire("camera_left_arm", timeout_s=2.0) as cam:
            assert cam.is_open

    # Warning must mention the dead PID.
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert any(str(dead_pid) in r.message for r in warnings), (
        f"Expected WARNING mentioning pid {dead_pid}; got: {[r.message for r in warnings]}"
    )


def test_ac4_no_stale_recovery_for_live_pid(arbiter, lock_dir):
    """A lock held by a live PID must NOT be silently recovered — it must timeout."""
    import fcntl as _fcntl
    import json as _json

    from inchworm.camera_arbiter import _GLOBAL_LOCK_FILENAME

    lock_path = lock_dir / _GLOBAL_LOCK_FILENAME
    fh = lock_path.open("a+b")
    _fcntl.flock(fh, _fcntl.LOCK_EX)
    record = {"pid": os.getpid(), "device": "camera_left_arm", "ts": time.time()}
    fh.seek(0)
    fh.write(_json.dumps(record).encode())
    fh.truncate()
    fh.flush()
    try:
        with pytest.raises(CameraArbiterTimeoutError):
            arbiter.acquire("camera_left_arm", timeout_s=0.1).__enter__()
    finally:
        _fcntl.flock(fh, _fcntl.LOCK_UN)
        fh.close()


# ---------------------------------------------------------------------------
# AC #5 — arm and relay serial traffic is NOT gated by the camera arbiter
# ---------------------------------------------------------------------------


def test_ac5_serial_open_while_camera_locked(arbiter, backend):
    """ARM and relay serial devices can be opened concurrently with a live camera lock.

    The arbiter only gates camera devices; serial transports are opened via
    backend.open_serial() which has no interaction with CameraArbiter.
    """
    with arbiter.acquire("camera_left_arm", timeout_s=1.0) as cam:
        assert cam.is_open
        # All serial devices must open freely while the camera lock is held.
        arm_left = backend.open_serial("arm_left")
        arm_right = backend.open_serial("arm_right")
        relay = backend.open_serial("relay")
        dexarm = backend.open_serial("dexarm")
        assert arm_left.is_open
        assert arm_right.is_open
        assert relay.is_open
        assert dexarm.is_open
        # Write to serial while camera lock held — must not raise or block.
        arm_left.write(b"\xff")
        relay.write(b"\x01")
        arm_left.close()
        arm_right.close()
        relay.close()
        dexarm.close()


def test_ac5_camera_arbiter_rejects_serial_device():
    """CameraArbiter.acquire() must refuse a serial-class device name.

    The arbiter enforces per-camera exclusion; calling it on a serial device
    is a programming error that surfaces as BackendError, not silent success.
    """
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    arb = CameraArbiter(be)
    # "arm_left" is a serial device; the backend lookup inside acquire() must
    # raise BackendError because it is not a camera device.
    with pytest.raises(BackendError):
        arb.acquire("arm_left", timeout_s=0.5).__enter__()


# ---------------------------------------------------------------------------
# AC #6 — cross-process (multiprocessing) rigorous proofs
# ---------------------------------------------------------------------------
#
# Each test forks a worker process using the "fork" start method so it
# inherits the lock_dir path.  Workers communicate results via
# multiprocessing.Queue.


_MP_CTX = multiprocessing.get_context("fork")


# -- helpers -----------------------------------------------------------------


def _worker_hold_lock(lock_dir_str: str, device: str, ready_q, done_q) -> None:
    """Child: acquire lock, signal ready, wait for done, then exit cleanly."""
    from inchworm.backends import create_backend
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    arb = CameraArbiter(be, lock_dir=Path(lock_dir_str))
    with arb.acquire(device, timeout_s=5.0):
        ready_q.put("ready")
        done_q.get(timeout=10)  # blocks until parent signals done


def _worker_hold_then_raise(
    lock_dir_str: str, device: str, ready_q, done_q, result_q
) -> None:
    """Child: acquire lock, signal ready, raise inside the context manager, exit.

    After __exit__ completes, puts (is_open_after_exit,) into result_q so the
    parent can verify the transport was closed.
    """
    from inchworm.backends import create_backend
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    arb = CameraArbiter(be, lock_dir=Path(lock_dir_str))
    transport_ref = None
    try:
        with arb.acquire(device, timeout_s=5.0) as transport:
            transport_ref = transport
            ready_q.put("ready")
            done_q.get(timeout=10)  # wait for signal before raising
            raise RuntimeError("intentional cross-process exception")
    except RuntimeError:
        pass  # expected: context manager must have released the lock
    # Report whether the transport was closed after __exit__
    is_open = transport_ref.is_open if transport_ref is not None else None
    result_q.put({"is_open_after_exit": is_open})
    ready_q.put("released")  # signal that release is complete


# -- tests -------------------------------------------------------------------


def test_ac6_cross_process_mutual_exclusion(lock_dir):
    """Process A holds the camera lock; Process B must be denied, not silently admitted."""
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    device = "camera_left_arm"

    ready_q: multiprocessing.Queue = _MP_CTX.Queue()
    done_q: multiprocessing.Queue = _MP_CTX.Queue()

    proc = _MP_CTX.Process(
        target=_worker_hold_lock,
        args=(str(lock_dir), device, ready_q, done_q),
        daemon=True,
    )
    proc.start()
    try:
        assert ready_q.get(timeout=5) == "ready"  # A holds the lock

        arb_b = CameraArbiter(be, lock_dir=lock_dir)
        with pytest.raises(CameraArbiterTimeoutError, match=device):
            arb_b.acquire(device, timeout_s=0.3).__enter__()
    finally:
        done_q.put("done")
        proc.join(timeout=5)


def test_ac6_cross_process_different_device_mutual_exclusion(lock_dir):
    """Process A holds camera_left_arm; Process B's attempt on camera_right_arm must fail.

    Proves the global lock prevents ANY second camera, not just the same device.
    """
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)

    ready_q: multiprocessing.Queue = _MP_CTX.Queue()
    done_q: multiprocessing.Queue = _MP_CTX.Queue()

    proc = _MP_CTX.Process(
        target=_worker_hold_lock,
        args=(str(lock_dir), "camera_left_arm", ready_q, done_q),
        daemon=True,
    )
    proc.start()
    try:
        assert ready_q.get(timeout=5) == "ready"  # A holds camera_left_arm

        arb_b = CameraArbiter(be, lock_dir=lock_dir)
        with pytest.raises(CameraArbiterTimeoutError):
            # Different device — still must fail: global lock is taken.
            arb_b.acquire("camera_right_arm", timeout_s=0.3).__enter__()
    finally:
        done_q.put("done")
        proc.join(timeout=5)


def test_ac6_cross_process_timeout(lock_dir):
    """Process A holds the lock indefinitely; Process B's acquire must raise CameraArbiterTimeoutError."""
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    device = "camera_left_arm"

    ready_q: multiprocessing.Queue = _MP_CTX.Queue()
    done_q: multiprocessing.Queue = _MP_CTX.Queue()

    proc = _MP_CTX.Process(
        target=_worker_hold_lock,
        args=(str(lock_dir), device, ready_q, done_q),
        daemon=True,
    )
    proc.start()
    try:
        assert ready_q.get(timeout=5) == "ready"

        arb_b = CameraArbiter(be, lock_dir=lock_dir)
        t0 = time.monotonic()
        with pytest.raises(CameraArbiterTimeoutError):
            arb_b.acquire(device, timeout_s=0.5).__enter__()
        elapsed = time.monotonic() - t0
        # Must have waited approximately the timeout (within 0.5s slack).
        assert elapsed >= 0.4, f"timed out too early: {elapsed:.3f}s"
    finally:
        done_q.put("done")
        proc.join(timeout=5)


def test_ac6_cross_process_exception_path_release(lock_dir):
    """Process A acquires, the body raises, __exit__ releases; Process B can then acquire.

    Also verifies (via Queue) that the transport is_open is False after __exit__,
    proving the handle was closed, not merely that the lock was released.
    """
    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    device = "camera_left_arm"

    ready_q: multiprocessing.Queue = _MP_CTX.Queue()
    done_q: multiprocessing.Queue = _MP_CTX.Queue()
    result_q: multiprocessing.Queue = _MP_CTX.Queue()

    proc = _MP_CTX.Process(
        target=_worker_hold_then_raise,
        args=(str(lock_dir), device, ready_q, done_q, result_q),
        daemon=True,
    )
    proc.start()
    try:
        assert ready_q.get(timeout=5) == "ready"  # A holds lock
        done_q.put("raise")  # tell A to raise inside the context

        # Collect transport.is_open reported by the child after __exit__
        result = result_q.get(timeout=5)
        assert result["is_open_after_exit"] is False, (
            f"Expected transport.is_open == False after __exit__, got: {result['is_open_after_exit']}"
        )

        assert ready_q.get(timeout=5) == "released"  # A's __exit__ ran

        # Lock must be free now; B can acquire it.
        arb_b = CameraArbiter(be, lock_dir=lock_dir)
        with arb_b.acquire(device, timeout_s=1.0) as cam:
            assert cam.is_open  # handle is open (not just silently acquired)
    finally:
        proc.join(timeout=5)


def test_ac6_cross_process_stale_lock_recovery_after_sigkill(lock_dir, caplog):
    """Process A acquires the lock, is SIGKILL'd; Process B must recover with a WARNING.

    SIGKILL prevents any cleanup; this is the hardest stale-lock scenario.
    The arbiter must detect the dead PID and recover instead of blocking.
    """
    import signal

    from inchworm.config import load_config

    cfg = load_config()
    be = create_backend(cfg)
    device = "camera_left_arm"

    ready_q: multiprocessing.Queue = _MP_CTX.Queue()
    done_q: multiprocessing.Queue = _MP_CTX.Queue()

    proc = _MP_CTX.Process(
        target=_worker_hold_lock,
        args=(str(lock_dir), device, ready_q, done_q),
        daemon=True,
    )
    proc.start()
    killed_pid = proc.pid
    try:
        assert ready_q.get(timeout=5) == "ready"  # A holds the lock

        # SIGKILL — no __exit__, no lock release.
        os.kill(killed_pid, signal.SIGKILL)
        proc.join(timeout=3)
        assert proc.exitcode == -signal.SIGKILL, (
            f"Expected SIGKILL exit code, got {proc.exitcode}"
        )

        # OS advisory lock is released by the kernel on process death.
        # The lock file still contains A's PID record — arbiter must recover it.
        arb_b = CameraArbiter(be, lock_dir=lock_dir)
        with caplog.at_level(logging.WARNING, logger="inchworm.camera_arbiter"):
            with arb_b.acquire(device, timeout_s=2.0) as cam:
                assert cam.is_open

        warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
        assert any(str(killed_pid) in r.message for r in warnings), (
            f"Expected WARNING mentioning killed pid {killed_pid}; "
            f"got: {[r.message for r in warnings]}"
        )
    finally:
        # Ensure proc is dead and done_q is drained (belt-and-suspenders).
        try:
            os.kill(killed_pid, 0)  # process still alive?
            os.kill(killed_pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.join(timeout=2)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _get_dead_pid() -> int:
    """Return a PID that is guaranteed not to exist in the current OS.

    Uses subprocess (not os.fork) so there is no fork-in-test ambiguity and
    the helper works correctly even inside pytest's worker processes.
    """
    result = subprocess.run(
        [sys.executable, "-c", "import os; print(os.getpid())"],
        capture_output=True,
        text=True,
        check=True,
    )
    pid = int(result.stdout.strip())
    # The subprocess has already exited; its PID is now dead.
    return pid
