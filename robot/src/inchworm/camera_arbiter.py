"""Cross-process exclusive camera arbiter (doc-1 s6 / AA-6).

Only one camera may transfer data at a time on the shared USB 2.0 uplink.
This module enforces that invariant structurally via :class:`CameraArbiter`
and the :func:`acquire_camera` context manager.

Implementation:
- One system-wide ``fcntl`` LOCK_EX file lock (``_global_camera.lock``) under
  *lock_dir* (default ``/tmp/inchworm_cam_locks``) guards ALL camera
  acquisitions.  Any process that holds this lock is the sole active camera
  holder, regardless of which device it has opened.  This directly enforces
  the doc-1 s6 / AGENTS.md invariant: only one camera streaming at a time.
- The global lock file contains a JSON record
  ``{"pid": <int>, "device": <str>, "ts": <float>}`` written by the holder.
- Per-device lock files are *not* used; the global lock is the sole guard.
- Contention: poll until *timeout_s* elapses, then raise
  :class:`CameraArbiterTimeoutError`.
- Stale lock (holder PID is dead): atomically recover by re-locking and
  logging a WARNING.
- The :meth:`CameraSession.__exit__` always closes the underlying
  :class:`~inchworm.backends.CameraTransport`, even if the body raised.

``DEFAULT_TIMEOUT_S`` is kept as a module-level constant (5.0 s).  There is
no ``timeout`` field in ``config/hardware.yaml`` or ``constants.py``; adding
one solely for this value is not warranted.

Usage::

    arbiter = CameraArbiter(backend)
    with arbiter.acquire("camera_left_arm", timeout_s=5.0) as cam:
        frame = cam.read_frame()
    # cam is closed here, global lock released
"""

from __future__ import annotations

import fcntl
import json
import logging
import os
import time
from pathlib import Path
from types import TracebackType
from typing import Optional, Type

from inchworm.backends import Backend, CameraTransport

log = logging.getLogger(__name__)

DEFAULT_LOCK_DIR = Path("/tmp/inchworm_cam_locks")  # noqa: S108 — tmp is intentional
DEFAULT_TIMEOUT_S: float = 5.0
_POLL_INTERVAL_S: float = 0.05

# Name of the single global lock file shared by every camera acquisition.
_GLOBAL_LOCK_FILENAME = "_global_camera.lock"


class CameraArbiterError(RuntimeError):
    """Base error for arbiter failures."""


class CameraArbiterTimeoutError(CameraArbiterError):
    """Raised when camera lock cannot be acquired within the timeout."""


class CameraSession:
    """Context manager: holds the exclusive global camera lock and the live transport.

    Do not create directly; use :meth:`CameraArbiter.acquire`.
    """

    def __init__(
        self,
        *,
        device: str,
        transport: CameraTransport,
        lock_fh,  # open file handle used for fcntl lock
        lock_path: Path,
    ) -> None:
        self._device = device
        self._transport = transport
        self._lock_fh = lock_fh
        self._lock_path = lock_path
        self._entered = False

    # -----------------------------------------------------------------
    # Context-manager protocol
    # -----------------------------------------------------------------

    def __enter__(self) -> CameraTransport:
        self._entered = True
        return self._transport

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool:
        """Release the global lock and fully close the camera transport."""
        try:
            if self._transport.is_open:
                self._transport.close()
        finally:
            try:
                fcntl.flock(self._lock_fh, fcntl.LOCK_UN)
            finally:
                try:
                    self._lock_fh.close()
                except OSError:
                    pass
        return False  # never suppress exceptions


class CameraArbiter:
    """Process-wide and cross-process exclusive camera arbiter.

    Enforces the doc-1 s6 invariant: **only one camera may stream at a time**,
    system-wide.  Any call to :meth:`acquire` — regardless of which device is
    requested — must hold the single global lock
    ``<lock_dir>/_global_camera.lock`` before the camera is opened.  A second
    concurrent acquisition of *any* camera device will block until the first
    holder releases, or raise :class:`CameraArbiterTimeoutError`.

    One arbiter instance may be shared across threads in the same process
    (the cross-process guarantee is provided by the OS-level file lock).

    Parameters
    ----------
    backend:
        The :class:`~inchworm.backends.Backend` used to open cameras.
    lock_dir:
        Directory where the global camera lock file is created.  Defaults to
        ``/tmp/inchworm_cam_locks``.
    """

    def __init__(
        self,
        backend: Backend,
        *,
        lock_dir: Path = DEFAULT_LOCK_DIR,
    ) -> None:
        self._backend = backend
        self._lock_dir = lock_dir
        self._lock_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def acquire(
        self,
        device: str,
        *,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> CameraSession:
        """Acquire the global exclusive camera lock for *device* and return a
        :class:`CameraSession`.

        The same global lock is contended by every camera device; only one
        camera can be open at any time across all processes.

        Call inside a ``with`` statement::

            with arbiter.acquire("camera_left_arm") as cam:
                frame = cam.read_frame()

        Raises
        ------
        CameraArbiterTimeoutError
            If the lock cannot be acquired within *timeout_s* seconds.
        CameraArbiterError
            For unexpected locking errors.
        """
        lock_path = self._lock_dir / _GLOBAL_LOCK_FILENAME
        lock_fh = self._acquire_lock(device, lock_path, timeout_s)
        # Global lock is now held; open the camera.
        try:
            transport = self._backend.open_camera(device)
        except Exception:
            # Release the lock before propagating; transport not opened.
            try:
                fcntl.flock(lock_fh, fcntl.LOCK_UN)
            finally:
                lock_fh.close()
            raise
        return CameraSession(
            device=device,
            transport=transport,
            lock_fh=lock_fh,
            lock_path=lock_path,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _write_owner_record(self, fh, device: str) -> None:
        """Overwrite the lock file with the current PID, device and timestamp."""
        record = {"pid": os.getpid(), "device": device, "ts": time.time()}
        fh.seek(0)
        fh.write(json.dumps(record).encode())
        fh.truncate()
        fh.flush()

    def _read_owner_record(self, path: Path) -> Optional[dict]:
        """Return the parsed owner record, or None if unreadable/invalid."""
        try:
            raw = path.read_bytes()
            return json.loads(raw) if raw.strip() else None
        except (OSError, json.JSONDecodeError):
            return None

    def _pid_is_alive(self, pid: int) -> bool:
        """Return True if *pid* exists in the OS process table."""
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # Process exists but we cannot signal it — treat as alive.
            return True

    def _acquire_lock(self, device: str, lock_path: Path, timeout_s: float):
        """Block-poll for LOCK_EX on the global lock up to *timeout_s*; recover stale locks.

        Stale-lock detection works in two modes:

        1. **No contention** (LOCK_EX succeeds immediately): read the previous
           owner record; if the recorded PID is dead, log a WARNING (the process
           had exited and the OS released the advisory lock automatically).
        2. **Contention** (LOCK_EX would block): read the owner record to check
           whether the holder PID is still alive.  If the PID is dead, unlink
           the lock file and retry immediately; otherwise poll until deadline.

        Returns the open file handle with the global lock held.
        """
        deadline = time.monotonic() + timeout_s
        first_contention = True

        while True:
            fh = lock_path.open("a+b")  # create if absent; open for read+write
            try:
                fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
                # Lock acquired — check for a previously-stale record.
                prev_record = self._read_owner_record(lock_path)
                if prev_record is not None:
                    prev_pid = prev_record.get("pid")
                    if isinstance(prev_pid, int) and prev_pid != os.getpid() and not self._pid_is_alive(prev_pid):
                        log.warning(
                            "global camera lock: recovered stale lock from dead pid %d "
                            "(was holding device %r)",
                            prev_pid,
                            prev_record.get("device", "<unknown>"),
                        )
                self._write_owner_record(fh, device)
                log.debug("global camera lock acquired for device %r by pid %d", device, os.getpid())
                return fh
            except BlockingIOError:
                fh.close()
                # Check whether the holder PID is dead (stale lock under contention).
                if first_contention:
                    first_contention = False
                    record = self._read_owner_record(lock_path)
                    if record is not None:
                        pid = record.get("pid")
                        if isinstance(pid, int) and not self._pid_is_alive(pid):
                            log.warning(
                                "global camera lock: stale lock from dead pid %d "
                                "(was holding device %r) detected; recovering",
                                pid,
                                record.get("device", "<unknown>"),
                            )
                            try:
                                lock_path.unlink(missing_ok=True)
                            except OSError:
                                pass
                            continue

                # Holder is alive (or unknown) — poll or timeout.
                if time.monotonic() >= deadline:
                    raise CameraArbiterTimeoutError(
                        f"camera {device!r}: could not acquire exclusive global camera lock "
                        f"within {timeout_s:.1f}s"
                    )
                time.sleep(_POLL_INTERVAL_S)
            except OSError as exc:
                fh.close()
                raise CameraArbiterError(
                    f"camera {device!r}: unexpected locking error: {exc}"
                ) from exc
