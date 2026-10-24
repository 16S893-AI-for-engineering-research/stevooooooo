"""USB relay laser driver for the inchworm print-crawl robot (AA-29).

The line laser is switched by a USB serial relay module (CH340, 1a86:7523).
This module provides:

* :class:`RelayProtocol` – pluggable ABC for relay wire encodings.
* :class:`Lcus1Protocol` – **UNVERIFIED** candidate default using the LCUS-1
  byte encoding (on: A0 01 01 A2, off: A0 01 00 A1). Marked unverified until
  confirmed with a live relay on hardware.
* :class:`LaserDriver` – fail-safe driver that wraps any protocol; provides
  a context manager, atexit, SIGINT/SIGTERM handlers, max-continuous-on
  enforcement, commanded-state tracking, and AA-7 telemetry.

Device resolution
-----------------
:func:`open_laser` resolves the relay device via
:func:`inchworm.discovery.require_device` (AA-5). If more than one
CH340 1a86:7523 device is present, discovery raises
:class:`~inchworm.discovery.RelayAmbiguousError` before any serial port is
opened.  The hardware backend (real ports) is intentionally NOT used in the
mock path; tests must supply a :class:`~inchworm.backends.MockSerialTransport`.

Safety invariants
-----------------
* **Fail-safe off**: the relay is commanded OFF on every exit path: normal
  context-manager exit, exception, SIGINT, SIGTERM, and atexit.
* **SIGKILL residual risk**: if the process is SIGKILLed (kill -9) none of the
  Python exit handlers run, and the relay remains in its last commanded state.
  Mitigation: use an external hardware watchdog timer (e.g. a mechanical timer
  outlet or a host-side watchdog script that detects process death and cuts
  USB power to the relay) so the laser cannot remain energised beyond the
  watchdog timeout. Document this in the operational runbook.
* **Max-continuous-on**: :meth:`LaserDriver.on` enforces a configured
  maximum single-on duration from ``config.laser.max_continuous_on_s``.
  Two complementary mechanisms:

  1. *Pre-check*: if ``duration_s`` is supplied and exceeds the limit,
     :class:`LaserDurationExceededError` is raised before energising the relay.
  2. *Active watchdog*: a :class:`threading.Timer` is started when the relay is
     energised (with or without ``duration_s``). If ``off()`` is not called
     before the limit elapses, the watchdog calls ``off()`` automatically, logs
     a ``laser_max_on_cutoff`` WARNING event, and resets internal state so that
     the next ``on()`` call works normally (no explicit re-arming needed; the
     watchdog firing is equivalent to a manual ``off()``).

  ``off()`` and ``close()`` always cancel the watchdog before it fires.

Commanded-vs-reported state
---------------------------
The CH340 LCUS-1 relay module does NOT echo state back over serial. There is no
reliable way to read the relay's current physical state from the host side.
:class:`LaserStateMismatchError` is therefore reserved for a future protocol
variant that can verify state (e.g. via a GPIO sense line). AC#5 cannot be
fully verified on LCUS-1 hardware; see :class:`LaserDriver.commanded_on`.

AA-10 emergency stop hook
--------------------------
:func:`register_estop_hook` wires the driver's :meth:`LaserDriver.estop` into
the AA-10 e-stop bus once AA-10 exists. The hook is provided and tested here
using a local no-op bus; the AC remains unchecked until AA-10 is implemented.
"""

from __future__ import annotations

import atexit
import logging
import signal
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from inchworm.backends import SerialTransport
    from inchworm.config import HardwareConfig
    from inchworm.telemetry import SessionLogger

_LOG = logging.getLogger("inchworm.laser")

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class LaserStateMismatchError(RuntimeError):
    """Raised when commanded and reported relay state disagree.

    NOTE: The LCUS-1 protocol cannot report relay state; this error is
    reserved for future protocol variants that include state readback (e.g.
    a GPIO sense wire). See module docstring for details.
    """

    def __init__(self, commanded: bool, reported: bool) -> None:
        self.commanded = commanded
        self.reported = reported
        super().__init__(
            f"Relay state mismatch: commanded={'ON' if commanded else 'OFF'}, "
            f"reported={'ON' if reported else 'OFF'}. "
            "Check wiring and relay firmware."
        )


class LaserDurationExceededError(RuntimeError):
    """Raised when a requested on-period would exceed ``max_continuous_on_s``."""

    def __init__(self, requested_s: float, max_s: float) -> None:
        self.requested_s = requested_s
        self.max_s = max_s
        super().__init__(
            f"Laser on-duration {requested_s:.1f}s would exceed configured "
            f"max_continuous_on_s={max_s:.1f}s. Reduce scan time or raise the limit."
        )


class LaserAlreadyOnError(RuntimeError):
    """Raised when :meth:`LaserDriver.on` is called while the laser is already on."""


class LaserNotOpenError(RuntimeError):
    """Raised when :meth:`LaserDriver.on` or :meth:`LaserDriver.off` is called on a closed driver."""


# ---------------------------------------------------------------------------
# Pluggable relay protocol
# ---------------------------------------------------------------------------


class RelayProtocol(ABC):
    """Wire-encoding strategy for a single-channel USB relay module.

    Subclass this to add a new protocol variant; pass the instance to
    :class:`LaserDriver`. The default is :class:`Lcus1Protocol`.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Short human-readable protocol name for logging."""

    @abstractmethod
    def encode_on(self) -> bytes:
        """Return the bytes to send to close (energise) the relay."""

    @abstractmethod
    def encode_off(self) -> bytes:
        """Return the bytes to send to open (de-energise) the relay."""

    def can_report_state(self) -> bool:
        """Return True if this protocol can read back the relay state.

        Override and return True if the protocol includes a state-query
        command, then override :meth:`decode_state` as well.
        """
        return False

    def decode_state(self, response: bytes) -> bool:
        """Parse a relay state response into a bool (True = ON).

        Only called when :meth:`can_report_state` returns True.
        Raises :exc:`NotImplementedError` by default.
        """
        raise NotImplementedError(
            f"Protocol {self.name!r} does not implement state readback."
        )


class Lcus1Protocol(RelayProtocol):
    """LCUS-1 USB relay wire encoding.

    **VERIFIED 2026-10-24 on this robot's relay** (CH340 1a86:7523, 9600 8N1):
    owner-supervised ``laser_probe.py --live --on-seconds 1``. The relay
    clicked and its LED lit, the line laser came on for about 1 s, and both went
    off on the OFF command (on: ``A0 01 01 A2``, off: ``A0 01 00 A1``).

    The checksum byte is ``A0 + channel + state``. Channel is hard-coded to
    ``0x01`` (single relay, channel 1).

    The LCUS-1 does NOT echo state; :meth:`can_report_state` returns ``False``.
    """

    name = "lcus1"

    # Channel 1 byte sequences
    _ON: bytes = bytes([0xA0, 0x01, 0x01, 0xA2])
    _OFF: bytes = bytes([0xA0, 0x01, 0x00, 0xA1])

    def encode_on(self) -> bytes:
        return self._ON

    def encode_off(self) -> bytes:
        return self._OFF

    def can_report_state(self) -> bool:
        return False


# ---------------------------------------------------------------------------
# E-stop hook registry (AA-10 integration seam)
# ---------------------------------------------------------------------------

# List of registered e-stop callbacks. AA-10 populates this list.
_ESTOP_HOOKS: List[Callable[[], None]] = []


def register_estop_hook(callback: Callable[[], None]) -> None:
    """Register a zero-argument callable that is called on emergency stop.

    This is the integration seam for AA-10. Once AA-10 is implemented it calls
    this function to wire in laser shutdown. AC#3 (e-stop turns laser off)
    stays unchecked until AA-10 exists and is tested end-to-end.

    Multiple callbacks are supported (last-in, first-called).
    """
    _ESTOP_HOOKS.append(callback)


def trigger_estop() -> None:
    """Fire all registered e-stop hooks (in registration order).

    This exists for testing the hook mechanism without AA-10.
    """
    for hook in list(_ESTOP_HOOKS):
        try:
            hook()
        except Exception:
            _LOG.exception("e-stop hook raised; continuing with remaining hooks")


# ---------------------------------------------------------------------------
# LaserDriver
# ---------------------------------------------------------------------------


class LaserDriver:
    """Fail-safe laser driver.

    Parameters
    ----------
    transport:
        Byte-level serial transport (from :class:`~inchworm.backends.Backend`).
        On the mock backend, pass a :class:`~inchworm.backends.MockSerialTransport`.
    config:
        Validated :class:`~inchworm.config.HardwareConfig`; used to read
        ``laser.max_continuous_on_s`` and the role name.
    protocol:
        Wire-encoding strategy. Defaults to :class:`Lcus1Protocol` (UNVERIFIED).
    logger:
        AA-7 session logger. If ``None``, events are emitted to stdlib logging
        only (no JSONL session file).

    Safety
    ------
    * Context-manager ``__exit__`` always calls :meth:`off`.
    * An atexit handler and SIGINT/SIGTERM handlers call :meth:`off`.
    * :meth:`on` raises :class:`LaserDurationExceededError` before energising
      if the requested duration would exceed ``max_continuous_on_s``.
    * SIGKILL is NOT catchable; see module docstring for the watchdog mitigation.
    """

    def __init__(
        self,
        transport: "SerialTransport",
        config: "HardwareConfig",
        *,
        protocol: Optional[RelayProtocol] = None,
        logger: Optional["SessionLogger"] = None,
    ) -> None:
        self._transport = transport
        self._max_on_s: float = config.laser.max_continuous_on_s
        self._protocol: RelayProtocol = protocol or Lcus1Protocol()
        self._logger = logger
        self._commanded_on: bool = False
        self._on_start_time: Optional[float] = None
        self._cumulative_on_s: float = 0.0
        self._closed: bool = False
        self._lock = threading.Lock()
        self._watchdog: Optional[threading.Timer] = None

        # Register atexit for fail-safe off.
        atexit.register(self._atexit_off)

        # Register signal handlers (replacing any existing handlers temporarily).
        self._prev_sigint = signal.signal(signal.SIGINT, self._signal_handler)
        self._prev_sigterm = signal.signal(signal.SIGTERM, self._signal_handler)

        # Register e-stop hook.
        register_estop_hook(self.estop)

        self._log("INFO", "laser_driver_opened", {
            "protocol": self._protocol.name,
            "max_continuous_on_s": self._max_on_s,
        })

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def commanded_on(self) -> bool:
        """True if the driver last commanded the relay ON.

        This reflects the last *commanded* state, NOT a hardware readback.
        The LCUS-1 protocol cannot read relay state; no mismatch detection is
        possible for this hardware. :class:`LaserStateMismatchError` is typed
        for future protocol variants that include state readback.
        """
        return self._commanded_on

    @property
    def cumulative_on_s(self) -> float:
        """Total laser-on time (seconds) accumulated in this session."""
        with self._lock:
            running = 0.0
            if self._commanded_on and self._on_start_time is not None:
                running = time.monotonic() - self._on_start_time
            return self._cumulative_on_s + running

    def on(self, duration_s: Optional[float] = None) -> None:
        """Command the relay ON (laser energised).

        Parameters
        ----------
        duration_s:
            If supplied, verify that this on-period would NOT exceed
            ``max_continuous_on_s`` *before* energising the relay.
            Raises :class:`LaserDurationExceededError` without opening the
            relay if the duration is too long.

        Raises
        ------
        LaserNotOpenError
            If the driver has been closed.
        LaserAlreadyOnError
            If the laser is already on.
        LaserDurationExceededError
            If ``duration_s`` exceeds the configured maximum.
        """
        with self._lock:
            self._require_open()
            if self._commanded_on:
                raise LaserAlreadyOnError("Laser is already ON; call off() first.")
            if duration_s is not None and duration_s > self._max_on_s:
                raise LaserDurationExceededError(
                    requested_s=duration_s, max_s=self._max_on_s
                )
            self._transport.write(self._protocol.encode_on())
            self._commanded_on = True
            self._on_start_time = time.monotonic()
            # Start the active watchdog; it fires off() if the caller never does.
            self._watchdog = threading.Timer(self._max_on_s, self._watchdog_cutoff)
            self._watchdog.daemon = True
            self._watchdog.start()

        self._log("INFO", "laser_on", {
            "protocol": self._protocol.name,
            "duration_s": duration_s,
        })

    def off(self) -> None:
        """Command the relay OFF (laser de-energised).

        Safe to call even if already off (no-op when not on and driver is open).
        Safe to call after :meth:`close` (will attempt to send off command if
        transport is still open).

        Always cancels the active watchdog timer, if any.
        """
        # Cancel the watchdog *before* acquiring the lock so the watchdog
        # callback (which also calls off()) cannot deadlock: if the watchdog
        # fires and calls off(), it will acquire the lock normally from its
        # own thread; cancelling here is a best-effort early stop.
        watchdog = self._watchdog
        if watchdog is not None:
            watchdog.cancel()
            self._watchdog = None

        with self._lock:
            was_on = self._commanded_on
            on_start = self._on_start_time
            self._commanded_on = False
            self._on_start_time = None
            # Clear watchdog ref inside lock as well (harmless double-clear).
            self._watchdog = None

            if self._transport.is_open:
                self._transport.write(self._protocol.encode_off())

            if was_on and on_start is not None:
                elapsed = time.monotonic() - on_start
                self._cumulative_on_s += elapsed
            else:
                elapsed = 0.0

        self._log("INFO", "laser_off", {
            "protocol": self._protocol.name,
            "on_duration_s": round(elapsed, 4),
            "cumulative_on_s": round(self._cumulative_on_s, 4),
        })

    def estop(self) -> None:
        """Emergency stop: immediately command the relay OFF.

        This is the callback registered with :func:`register_estop_hook` for
        AA-10 integration. AC#3 stays unchecked until AA-10 is implemented
        and verified in an end-to-end test.
        """
        self._log("WARNING", "laser_estop", {"reason": "emergency_stop"})
        self.off()

    def close(self) -> None:
        """Close the driver: command OFF then close the transport.

        Safe to call multiple times.
        """
        if self._closed:
            return
        self._closed = True
        self.off()
        if self._transport.is_open:
            self._transport.close()
        self._log("INFO", "laser_driver_closed", {
            "cumulative_on_s": round(self._cumulative_on_s, 4),
        })

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "LaserDriver":
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        """Always commands OFF regardless of whether an exception occurred."""
        self.off()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _watchdog_cutoff(self) -> None:
        """Called by the watchdog timer when max_continuous_on_s elapses.

        Executed on the timer's daemon thread. Logs a WARNING then calls
        off() to de-energise the relay. After this, the next on() call works
        normally – no explicit re-arming is required.
        """
        self._log("WARNING", "laser_max_on_cutoff", {
            "max_continuous_on_s": self._max_on_s,
            "reason": "watchdog_elapsed",
        })
        try:
            self.off()
        except Exception:
            pass

    def _require_open(self) -> None:
        if self._closed:
            raise LaserNotOpenError("LaserDriver has been closed.")

    def _atexit_off(self) -> None:
        """atexit handler: command relay OFF if process exits normally."""
        try:
            self.off()
        except Exception:
            pass

    def _signal_handler(self, signum: int, frame: object) -> None:
        """SIGINT/SIGTERM handler: command relay OFF then chain to previous handler."""
        try:
            self.off()
        except Exception:
            pass
        # Chain to previously installed handler (e.g. KeyboardInterrupt for SIGINT).
        prev = self._prev_sigint if signum == signal.SIGINT else self._prev_sigterm
        if callable(prev):
            prev(signum, frame)  # type: ignore[arg-type]
        elif prev == signal.SIG_DFL:
            signal.signal(signum, signal.SIG_DFL)
            signal.raise_signal(signum)

    def _log(self, severity: str, event_type: str, payload: dict) -> None:
        if self._logger is not None:
            self._logger.emit(
                role="relay",
                subsystem="laser",
                event_type=event_type,
                severity=severity,  # type: ignore[arg-type]
                payload=payload,
            )
        else:
            _LOG.log(
                logging.getLevelName(severity),
                "laser %s %r",
                event_type,
                payload,
            )


# ---------------------------------------------------------------------------
# open_laser factory
# ---------------------------------------------------------------------------


def open_laser(
    config: "HardwareConfig",
    backend,  # inchworm.backends.Backend
    *,
    protocol: Optional[RelayProtocol] = None,
    logger: Optional["SessionLogger"] = None,
) -> LaserDriver:
    """Open the laser driver, resolving the relay device through the backend.

    On the mock backend, this opens a mock serial transport. On the hardware
    backend, the transport is opened via :func:`inchworm.discovery.require_device`
    (AA-5) which raises :class:`~inchworm.discovery.RelayAmbiguousError` if
    more than one CH340 1a86:7523 device is present.

    Parameters
    ----------
    config:
        Loaded :class:`~inchworm.config.HardwareConfig`.
    backend:
        Active :class:`~inchworm.backends.Backend` instance. Use mock backend
        for testing and dry-run.
    protocol:
        Wire encoding; defaults to :class:`Lcus1Protocol` (**UNVERIFIED**).
    logger:
        Optional AA-7 session logger for structured event records.
    """
    transport = backend.open_serial("relay")
    return LaserDriver(transport, config, protocol=protocol, logger=logger)
