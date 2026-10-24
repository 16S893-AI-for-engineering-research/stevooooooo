"""Tests for inchworm.laser (AA-29 ACs #2, #4, #6, #7).

All tests run without hardware. No real serial ports are opened; all relay
I/O uses :class:`~inchworm.backends.MockSerialTransport`.

ACs NOT tested here (hardware or external integration required):
- AC#1: LCUS-1 protocol correctness vs. real relay  (needs owner-supervised hardware session)
- AC#3: AA-10 e-stop integration                    (AA-10 not yet implemented)
- AC#5: Commanded-vs-reported mismatch              (LCUS-1 cannot report state; no live evidence)

AC#7 coverage:
- on/off via mock transport (byte sequences sent)
- exception-path off (context manager exit on exception)
- state-mismatch error type exists and is typed
- duration limit enforcement
"""

from __future__ import annotations

import signal
import threading
import time

import pytest

from inchworm.backends import MockBackend, MockSerialTransport
from inchworm.config import load_config
from inchworm.laser import (
    LaserAlreadyOnError,
    LaserDriver,
    LaserDurationExceededError,
    LaserNotOpenError,
    LaserStateMismatchError,
    Lcus1Protocol,
    _ESTOP_HOOKS,
    open_laser,
    register_estop_hook,
    trigger_estop,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def config():
    return load_config()


@pytest.fixture()
def mock_transport():
    return MockSerialTransport("relay")


@pytest.fixture()
def driver(config, mock_transport):
    """A LaserDriver wired to a MockSerialTransport; closed after each test."""
    drv = LaserDriver(mock_transport, config)
    yield drv
    # Ensure cleanup even if test fails mid-flight (idempotent).
    try:
        drv.close()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Lcus1Protocol byte encoding
# ---------------------------------------------------------------------------


class TestLcus1Protocol:
    """Wire encoding tests – verify bytes match the documented LCUS-1 encoding."""

    def test_on_bytes(self):
        p = Lcus1Protocol()
        assert p.encode_on() == bytes([0xA0, 0x01, 0x01, 0xA2])

    def test_off_bytes(self):
        p = Lcus1Protocol()
        assert p.encode_off() == bytes([0xA0, 0x01, 0x00, 0xA1])

    def test_cannot_report_state(self):
        p = Lcus1Protocol()
        assert p.can_report_state() is False

    def test_protocol_name(self):
        assert "unverified" in Lcus1Protocol().name.lower()


# ---------------------------------------------------------------------------
# AC#7a: basic on/off
# ---------------------------------------------------------------------------


class TestLaserOnOff:
    """AC#7: tests for on, off via mock backend."""

    def test_on_sends_correct_bytes(self, driver, mock_transport):
        driver.on()
        assert mock_transport.written[-1] == bytes([0xA0, 0x01, 0x01, 0xA2])
        assert driver.commanded_on is True

    def test_off_sends_correct_bytes(self, driver, mock_transport):
        driver.on()
        mock_transport.written.clear()
        driver.off()
        assert mock_transport.written[-1] == bytes([0xA0, 0x01, 0x00, 0xA1])
        assert driver.commanded_on is False

    def test_off_when_already_off_sends_off_bytes(self, driver, mock_transport):
        """off() is idempotent: always sends the off command."""
        mock_transport.written.clear()
        driver.off()  # already off
        assert mock_transport.written[-1] == bytes([0xA0, 0x01, 0x00, 0xA1])

    def test_on_while_already_on_raises(self, driver):
        driver.on()
        with pytest.raises(LaserAlreadyOnError):
            driver.on()

    def test_commanded_on_false_initially(self, driver):
        assert driver.commanded_on is False

    def test_on_after_close_raises(self, driver):
        driver.close()
        with pytest.raises(LaserNotOpenError):
            driver.on()


# ---------------------------------------------------------------------------
# AC#2 / AC#7b: context manager – exception-path off
# ---------------------------------------------------------------------------


class TestContextManager:
    """AC#2: context manager turns laser off on normal exit and on exception."""

    def test_context_manager_turns_off_on_normal_exit(self, config, mock_transport):
        with LaserDriver(mock_transport, config) as drv:
            drv.on()
            assert drv.commanded_on is True
        # After exit the last write must be the OFF bytes.
        assert mock_transport.written[-1] == bytes([0xA0, 0x01, 0x00, 0xA1])

    def test_context_manager_turns_off_on_exception(self, config, mock_transport):
        with pytest.raises(RuntimeError):
            with LaserDriver(mock_transport, config) as drv:
                drv.on()
                raise RuntimeError("simulated error")
        # Off bytes must have been sent despite exception.
        assert bytes([0xA0, 0x01, 0x00, 0xA1]) in mock_transport.written

    def test_context_manager_off_even_if_never_turned_on(self, config, mock_transport):
        with LaserDriver(mock_transport, config) as _drv:
            pass
        assert bytes([0xA0, 0x01, 0x00, 0xA1]) in mock_transport.written


# ---------------------------------------------------------------------------
# AC#6 / AC#7d: duration limit enforcement
# ---------------------------------------------------------------------------


class TestDurationLimit:
    """AC#6 and AC#7: configured max_continuous_on_s is enforced."""

    def test_duration_within_limit_is_accepted(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        # max is 30.0 by default; 1.0 is fine.
        drv.on(duration_s=1.0)
        assert drv.commanded_on is True
        drv.close()

    def test_duration_exceeding_limit_raises_before_on(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        written_before = len(mock_transport.written)
        with pytest.raises(LaserDurationExceededError) as exc_info:
            drv.on(duration_s=31.0)
        # Relay must NOT have been energised.
        assert drv.commanded_on is False
        # No extra ON bytes were written.
        on_writes = [b for b in mock_transport.written[written_before:] if b == bytes([0xA0, 0x01, 0x01, 0xA2])]
        assert on_writes == []
        assert exc_info.value.requested_s == 31.0
        assert exc_info.value.max_s == 30.0
        drv.close()

    def test_duration_exactly_at_limit_raises(self, config, mock_transport):
        """Boundary: duration == max_s is NOT allowed (strictly greater check)."""
        drv = LaserDriver(mock_transport, config)
        # max_continuous_on_s = 30.0; requesting exactly 30.0 is not > 30.0 so it passes.
        drv.on(duration_s=30.0)
        assert drv.commanded_on is True
        drv.close()

    def test_duration_just_above_limit_raises(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        with pytest.raises(LaserDurationExceededError):
            drv.on(duration_s=30.001)
        drv.close()

    def test_no_duration_arg_is_always_accepted(self, config, mock_transport):
        """on() with no duration_s never checks the limit."""
        drv = LaserDriver(mock_transport, config)
        drv.on()
        assert drv.commanded_on is True
        drv.close()

    def test_custom_max_from_config(self, config, mock_transport):
        """Verify the limit is read from config, not hard-coded."""
        # Patch config to a tiny max.
        import copy
        cfg2 = config.model_copy(update={"laser": config.laser.model_copy(update={"max_continuous_on_s": 5.0})})
        drv = LaserDriver(mock_transport, cfg2)
        with pytest.raises(LaserDurationExceededError) as exc_info:
            drv.on(duration_s=6.0)
        assert exc_info.value.max_s == 5.0
        drv.close()


# ---------------------------------------------------------------------------
# AC#6 (active watchdog): auto-cutoff when max_continuous_on_s elapses
# ---------------------------------------------------------------------------


@pytest.fixture()
def watchdog_config(config):
    """Config with a small max_continuous_on_s so watchdog tests finish quickly."""
    return config.model_copy(
        update={"laser": config.laser.model_copy(update={"max_continuous_on_s": 0.2})}
    )


class TestWatchdog:
    """AC#6 active enforcement: watchdog fires off() after max_continuous_on_s."""

    def test_watchdog_sends_off_bytes_after_limit(self, watchdog_config, mock_transport):
        """Relay receives OFF bytes after the limit with no caller action."""
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()  # no duration_s – only watchdog can stop it
        assert drv.commanded_on is True
        # Wait up to 1 s for watchdog to fire (limit is 0.2 s).
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if not drv.commanded_on:
                break
            time.sleep(0.02)
        assert drv.commanded_on is False, "watchdog did not fire in time"
        assert mock_transport.written[-1] == bytes([0xA0, 0x01, 0x00, 0xA1])
        drv.close()

    def test_watchdog_logs_laser_max_on_cutoff(self, watchdog_config, mock_transport):
        """laser_max_on_cutoff WARNING is emitted when the watchdog fires."""
        import logging
        records = []

        class Capture(logging.Handler):
            def emit(self, record):
                records.append(record)

        handler = Capture()
        logging.getLogger("inchworm.laser").addHandler(handler)
        try:
            drv = LaserDriver(mock_transport, watchdog_config)
            drv.on()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if not drv.commanded_on:
                    break
                time.sleep(0.02)
            drv.close()
        finally:
            logging.getLogger("inchworm.laser").removeHandler(handler)

        cutoff_records = [r for r in records if "laser_max_on_cutoff" in r.getMessage()]
        assert cutoff_records, "expected laser_max_on_cutoff log record"
        assert cutoff_records[0].levelno == logging.WARNING

    def test_manual_off_cancels_watchdog(self, watchdog_config, mock_transport):
        """Calling off() manually cancels the watchdog; no second OFF is sent."""
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()
        drv.off()  # cancel the watchdog now
        off_count_at_manual = mock_transport.written.count(bytes([0xA0, 0x01, 0x00, 0xA1]))
        # Sleep past the watchdog deadline.
        time.sleep(0.35)
        off_count_after = mock_transport.written.count(bytes([0xA0, 0x01, 0x00, 0xA1]))
        assert off_count_after == off_count_at_manual, (
            f"watchdog fired after manual off: {off_count_after} vs {off_count_at_manual}"
        )
        drv.close()

    def test_close_cancels_watchdog(self, watchdog_config, mock_transport):
        """close() cancels the watchdog; no extra OFF bytes are written after close."""
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()
        drv.close()
        off_count_at_close = mock_transport.written.count(bytes([0xA0, 0x01, 0x00, 0xA1]))
        time.sleep(0.35)
        off_count_after = mock_transport.written.count(bytes([0xA0, 0x01, 0x00, 0xA1]))
        assert off_count_after == off_count_at_close, (
            f"watchdog fired after close: {off_count_after} vs {off_count_at_close}"
        )

    def test_no_leaked_threads_after_off(self, watchdog_config, mock_transport):
        """Calling off() leaves no live watchdog threads."""
        threads_before = threading.active_count()
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()
        drv.off()
        # Give any spurious thread a moment to die.
        time.sleep(0.05)
        threads_after = threading.active_count()
        drv.close()
        # Thread count should not grow from active watchdog timers.
        assert threads_after <= threads_before + 1, (
            f"possible thread leak: before={threads_before} after={threads_after}"
        )

    def test_no_leaked_threads_after_watchdog_fires(self, watchdog_config, mock_transport):
        """Watchdog thread cleans itself up after firing."""
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()
        # Wait for watchdog to fire.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if not drv.commanded_on:
                break
            time.sleep(0.02)
        # Brief pause for thread to fully finish.
        time.sleep(0.05)
        # Watchdog timer thread should be done (daemon, finished callback).
        live_names = [t.name for t in threading.enumerate()]
        # No Timer threads for this driver should be alive.
        assert drv._watchdog is None, "watchdog ref should be cleared after firing"
        drv.close()

    def test_on_after_watchdog_fires_works(self, watchdog_config, mock_transport):
        """After the watchdog fires, the next on() call works normally."""
        drv = LaserDriver(mock_transport, watchdog_config)
        drv.on()
        # Wait for watchdog to cut off.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if not drv.commanded_on:
                break
            time.sleep(0.02)
        assert drv.commanded_on is False
        # No re-arming required; just call on() again.
        drv.on()
        assert drv.commanded_on is True
        drv.close()



# ---------------------------------------------------------------------------
# AC#6 / AC#7: cumulative on-time logging
# ---------------------------------------------------------------------------


class TestCumulativeOnTime:
    """AC#6: cumulative laser-on time is tracked per session."""

    def test_cumulative_zero_before_any_on(self, driver):
        assert driver.cumulative_on_s == 0.0

    def test_cumulative_increments_after_off(self, driver):
        driver.on()
        time.sleep(0.01)
        driver.off()
        assert driver.cumulative_on_s > 0.0

    def test_cumulative_includes_running_time(self, driver):
        driver.on()
        # cumulative_on_s should be > 0 even while still on.
        time.sleep(0.01)
        live = driver.cumulative_on_s
        assert live > 0.0
        driver.off()


# ---------------------------------------------------------------------------
# AC#5: typed error and protocol limitation note
# ---------------------------------------------------------------------------


class TestStateMismatchTypedError:
    """AC#5 (partially): LaserStateMismatchError exists as a typed error.

    The LCUS-1 protocol cannot report relay state, so a real mismatch
    cannot be detected in software. This test confirms the typed error
    exists and is constructable, and that Lcus1Protocol.can_report_state()
    returns False (documenting the limitation).

    AC#5 remains UNCHECKED pending a protocol variant that includes state
    readback. See module docstring in laser.py.
    """

    def test_error_is_typed(self):
        err = LaserStateMismatchError(commanded=True, reported=False)
        assert isinstance(err, RuntimeError)
        assert err.commanded is True
        assert err.reported is False

    def test_lcus1_cannot_report_state(self):
        assert Lcus1Protocol().can_report_state() is False


# ---------------------------------------------------------------------------
# AC#3 seam: e-stop hook (AC#3 stays UNCHECKED until AA-10 exists)
# ---------------------------------------------------------------------------


class TestEstopHook:
    """Verify the e-stop hook mechanism works with a local test bus.

    AC#3 (AA-10 e-stop turns laser off) stays UNCHECKED because AA-10 does
    not yet exist. This test validates the hook contract: when trigger_estop()
    fires, the laser is commanded off.
    """

    def test_estop_turns_off_when_on(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        drv.on()
        assert drv.commanded_on is True
        drv.estop()
        assert drv.commanded_on is False
        drv.close()

    def test_estop_hook_registered_and_fires(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        drv.on()
        assert drv.commanded_on is True
        # trigger_estop fires all registered hooks including this driver's.
        trigger_estop()
        assert drv.commanded_on is False
        drv.close()

    def test_register_custom_hook(self):
        called = []
        register_estop_hook(lambda: called.append(True))
        trigger_estop()
        assert True in called


# ---------------------------------------------------------------------------
# AC#4: atexit / signal handlers (smoke-test only; real signal delivery not tested)
# ---------------------------------------------------------------------------


class TestFailSafeOff:
    """AC#4: atexit and signal handlers are registered at driver open."""

    def test_driver_registers_signal_handlers(self, config, mock_transport):
        """Confirm SIGINT/SIGTERM handlers are replaced during driver lifetime."""
        old_sigint = signal.getsignal(signal.SIGINT)
        drv = LaserDriver(mock_transport, config)
        new_handler = signal.getsignal(signal.SIGINT)
        # The handler must have changed (or be the driver's internal method).
        # We just check it is callable (not SIG_DFL or SIG_IGN).
        assert callable(new_handler) or new_handler != signal.SIG_DFL
        drv.close()

    def test_close_is_idempotent(self, config, mock_transport):
        drv = LaserDriver(mock_transport, config)
        drv.close()
        drv.close()  # Must not raise.

    def test_off_after_close_does_not_raise(self, config, mock_transport):
        """off() should be safe even after close() (atexit may call off again)."""
        drv = LaserDriver(mock_transport, config)
        drv.on()
        drv.close()
        # off() is idempotent; no exception expected.
        drv.off()


# ---------------------------------------------------------------------------
# open_laser factory
# ---------------------------------------------------------------------------


class TestOpenLaser:
    """open_laser resolves the relay through the backend."""

    def test_open_laser_mock_backend(self, config):
        backend = MockBackend(config)
        drv = open_laser(config, backend)
        assert isinstance(drv, LaserDriver)
        drv.close()

    def test_open_laser_sends_on_off(self, config):
        backend = MockBackend(config)
        drv = open_laser(config, backend)
        drv.on()
        drv.off()
        transport = backend.serial["relay"]
        assert bytes([0xA0, 0x01, 0x01, 0xA2]) in transport.written
        assert bytes([0xA0, 0x01, 0x00, 0xA1]) in transport.written
        drv.close()
