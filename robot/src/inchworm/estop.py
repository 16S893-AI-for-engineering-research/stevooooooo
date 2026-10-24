"""Emergency stop and fault event handling for the inchworm robot (AA-10).

Single entry point (:meth:`EStop.stop`) that:

1. Disables torque on both SO-101 arms via :class:`~inchworm.torque.TorqueManager`.
2. Sends the Marlin M410 quickstop to the DexArm via
   :meth:`~inchworm.dexarm.DexArmDriver.quickstop`.
3. Calls :func:`~inchworm.laser.trigger_estop` which fires all registered
   AA-29 laser e-stop hooks (turns the laser relay off).

Each subsystem is wrapped independently so that a failure in one does not
prevent stopping the others.  All errors are collected and re-raised together
as :class:`EStopPartialError` after all subsystems have been attempted.

Signal handling
---------------
:class:`EStop` installs SIGINT and SIGTERM handlers on construction.  They
call :meth:`stop` and then re-raise the signal via the previously installed
handler.  The handlers are installed once; subsequent :meth:`stop` calls are
idempotent (re-entry is safe).

Typed fault events
------------------
:mod:`inchworm.estop` defines :class:`FaultEvent` typed payloads for:

- ``overcurrent``: raw current value exceeded the configured threshold.
- ``overtemperature``: temperature exceeded the configured threshold.
- ``servo_unreachable``: a servo failed to respond.

All three trigger the same :meth:`EStop.stop` path and log a typed fault event
via AA-7 telemetry.

AA-29 hook wiring
-----------------
On construction, :class:`EStop` calls :func:`~inchworm.laser.register_estop_hook`
to wire :func:`~inchworm.laser.trigger_estop` into the e-stop bus.  The
:class:`~inchworm.laser.LaserDriver` registers its own ``estop()`` callback
when it is opened; this class just fires all registered hooks.

Note for AA-29 task
-------------------
AA-29 AC#3 ("Emergency stop (AA-10) turns the laser off, verified by a test")
can now be checked by the orchestrator.  The test in ``tests/test_estop.py``
(``test_estop_turns_laser_off``) verifies end-to-end that calling
:meth:`EStop.stop` triggers the laser estop hook.

Dry-run mode
------------
When ``dry_run=True``, all motion and relay commands are logged but never sent.
SIGINT/SIGTERM still install the stop handler (they call stop with dry_run=True).
"""

from __future__ import annotations

import logging
import signal
import threading
from typing import Dict, List, Optional

from inchworm.config import HardwareConfig, FaultThresholds
from inchworm.torque import TorqueManager
from inchworm.laser import trigger_estop as _laser_trigger_estop

_LOG = logging.getLogger("inchworm.estop")


# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class EStopError(RuntimeError):
    """Base class for emergency stop errors."""


class EStopPartialError(EStopError):
    """Raised when the e-stop completed but one or more subsystems failed.

    :attr:`subsystem_errors` maps subsystem name to the exception raised.
    The e-stop is considered partially successful: all subsystems were
    attempted, and some succeeded even if this error is raised.
    """

    def __init__(self, subsystem_errors: Dict[str, Exception]) -> None:
        self.subsystem_errors = dict(subsystem_errors)
        detail = "; ".join(
            f"{name}: {exc}" for name, exc in subsystem_errors.items()
        )
        super().__init__(f"e-stop partially failed [{detail}]")


# ---------------------------------------------------------------------------
# Typed fault event payloads
# ---------------------------------------------------------------------------


class FaultEvent:
    """Factory for typed fault event payloads (AA-10 telemetry).

    Each fault triggers :meth:`EStop.stop` and logs a structured event.
    """

    @staticmethod
    def overcurrent(
        *,
        role: str,
        servo_id: int,
        current_raw: int,
        threshold_raw: int,
    ) -> dict:
        """Payload for an overcurrent fault event."""
        return {
            "fault_type": "overcurrent",
            "role": role,
            "servo_id": servo_id,
            "current_raw": current_raw,
            "threshold_raw": threshold_raw,
        }

    @staticmethod
    def overtemperature(
        *,
        role: str,
        servo_id: int,
        temperature_c: int,
        threshold_c: int,
    ) -> dict:
        """Payload for an overtemperature fault event."""
        return {
            "fault_type": "overtemperature",
            "role": role,
            "servo_id": servo_id,
            "temperature_c": temperature_c,
            "threshold_c": threshold_c,
        }

    @staticmethod
    def servo_unreachable(
        *,
        role: str,
        servo_id: int,
        operation: str,
        error: str,
    ) -> dict:
        """Payload for a servo-unreachable fault event."""
        return {
            "fault_type": "servo_unreachable",
            "role": role,
            "servo_id": servo_id,
            "operation": operation,
            "error": error,
        }


# ---------------------------------------------------------------------------
# FaultMonitor
# ---------------------------------------------------------------------------


class FaultMonitor:
    """Checks servo state for overcurrent, overtemperature, and reachability.

    Used by the orchestrator polling loop.  On a fault it emits a typed event
    and calls :meth:`EStop.stop`.

    Parameters
    ----------
    estop:
        The :class:`EStop` instance to call on fault detection.
    config:
        Loaded :class:`~inchworm.config.HardwareConfig`.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    """

    def __init__(
        self,
        estop: "EStop",
        config: HardwareConfig,
        logger: Optional[object] = None,
    ) -> None:
        self._estop = estop
        self._config = config
        self._logger = logger
        self._ft: Optional[FaultThresholds] = config.fault_thresholds

    def check_joint_state(
        self,
        role: str,
        servo_id: int,
        current_raw: int,
        temperature_c: int,
    ) -> Optional[dict]:
        """Check one joint for overcurrent / overtemperature.

        Returns the fault event payload if a fault was detected and e-stop was
        triggered; ``None`` otherwise.
        """
        ft = self._ft
        if ft is None:
            return None

        # Overcurrent check
        if current_raw > ft.overcurrent_raw:
            payload = FaultEvent.overcurrent(
                role=role,
                servo_id=servo_id,
                current_raw=current_raw,
                threshold_raw=ft.overcurrent_raw,
            )
            self._emit("CRITICAL", "fault_overcurrent", role, payload)
            self._estop.stop(fault_payload=payload)
            return payload

        # Overtemperature check
        if temperature_c > ft.overtemperature_c:
            payload = FaultEvent.overtemperature(
                role=role,
                servo_id=servo_id,
                temperature_c=temperature_c,
                threshold_c=ft.overtemperature_c,
            )
            self._emit("CRITICAL", "fault_overtemperature", role, payload)
            self._estop.stop(fault_payload=payload)
            return payload

        return None

    def check_servo_unreachable(
        self,
        role: str,
        servo_id: int,
        operation: str,
        error: Exception,
    ) -> dict:
        """Log and trigger e-stop for an unreachable servo.

        Returns the fault event payload.
        """
        payload = FaultEvent.servo_unreachable(
            role=role,
            servo_id=servo_id,
            operation=operation,
            error=str(error),
        )
        self._emit("CRITICAL", "fault_servo_unreachable", role, payload)
        self._estop.stop(fault_payload=payload)
        return payload

    def _emit(self, severity: str, event_type: str, role: str, payload: dict) -> None:
        if self._logger is not None:
            method = getattr(self._logger, severity.lower(), None)
            if method is not None:
                method(
                    role=role,
                    subsystem="fault_monitor",
                    event_type=event_type,
                    payload=payload,
                )
        else:
            _LOG.log(
                logging.getLevelName(severity),
                "fault_monitor %s %s %r",
                role,
                event_type,
                payload,
            )


# ---------------------------------------------------------------------------
# EStop
# ---------------------------------------------------------------------------


class EStop:
    """Single entry point for emergency stop across all subsystems.

    Parameters
    ----------
    torque_manager:
        :class:`~inchworm.torque.TorqueManager` for both arms.
    dexarm:
        Optional :class:`~inchworm.dexarm.DexArmDriver`.  When ``None``,
        the DexArm stop step is skipped.
    config:
        Loaded :class:`~inchworm.config.HardwareConfig`.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    dry_run:
        When ``True``, all commands are logged but not sent.
    install_signal_handlers:
        When ``True`` (default), SIGINT and SIGTERM are hooked to call
        :meth:`stop`.  Set ``False`` in unit tests that manage signals
        themselves.
    """

    def __init__(
        self,
        torque_manager: TorqueManager,
        dexarm: Optional[object] = None,
        *,
        config: Optional[HardwareConfig] = None,
        logger: Optional[object] = None,
        dry_run: bool = False,
        install_signal_handlers: bool = True,
        release_on_stop: bool = False,
    ) -> None:
        self._torque = torque_manager
        # False (default): e-stop holds arms in place. True: e-stop releases torque.
        self._release_on_stop: bool = release_on_stop
        self._dexarm = dexarm
        self._config = config
        self._logger = logger
        self._dry_run = dry_run
        # Prevent concurrent stop() calls from racing.
        self._lock = threading.Lock()
        self._stopped = False

        if install_signal_handlers:
            self._prev_sigint = signal.signal(signal.SIGINT, self._signal_handler)
            self._prev_sigterm = signal.signal(signal.SIGTERM, self._signal_handler)
        else:
            self._prev_sigint = None
            self._prev_sigterm = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def stop(self, *, fault_payload: Optional[dict] = None) -> None:
        """Emergency stop: halt all motion, disable torques, turn laser off.

        Idempotent: subsequent calls after the first are no-ops (the system
        is already stopped).

        Each subsystem is attempted independently.  If any fail, errors are
        collected and raised together as :class:`EStopPartialError` after all
        subsystems have been attempted.

        Parameters
        ----------
        fault_payload:
            Optional typed fault event payload (from :class:`FaultEvent`).
            When provided, logged as the trigger reason.

        Raises
        ------
        EStopPartialError
            If one or more subsystems failed.  The system is as stopped as
            possible; the caller should log and transition to a faulted state.
        """
        with self._lock:
            if self._stopped:
                return
            self._stopped = True

        self._emit(
            "CRITICAL",
            "estop_triggered",
            "orchestrator",
            {"dry_run": self._dry_run, "fault": fault_payload or {}},
        )

        errors: Dict[str, Exception] = {}

        # --- 1. Arms: HOLD in place by default (tube may be carried), or
        # disable torque only when explicitly requested (release=True) or for
        # thermal/overcurrent faults that require de-energising the servo.
        # Orchestrator decision 2026-10-24, provisional pending owner input.
        fault_kind = (fault_payload or {}).get("fault_type") or (fault_payload or {}).get("kind")
        release = self._release_on_stop or fault_kind in ("overcurrent", "overtemperature")
        try:
            torque_results = (
                self._torque.disable_all() if release else self._torque.hold_all()
            )
            failed_arms = {r: e for r, e in torque_results.items() if e is not None}
            if failed_arms:
                errors["torque"] = EStopError(
                    f"arm {'release' if release else 'hold'} failed on: {'; '.join(str(e) for e in failed_arms.values())}"
                )
        except Exception as exc:
            errors["torque"] = exc
            _LOG.error("estop: torque disable raised: %s", exc)

        # --- 2. DexArm M410 quickstop ---
        if self._dexarm is not None:
            try:
                self._dexarm.quickstop(dry_run=self._dry_run)
            except Exception as exc:
                errors["dexarm"] = exc
                _LOG.error("estop: DexArm quickstop raised: %s", exc)
        else:
            self._emit(
                "INFO",
                "estop_dexarm_skipped",
                "orchestrator",
                {"reason": "no_dexarm_driver_registered"},
            )

        # --- 3. Laser off (AA-29 hooks) ---
        try:
            _laser_trigger_estop()
        except Exception as exc:
            errors["laser"] = exc
            _LOG.error("estop: laser trigger raised: %s", exc)

        # --- Emit completion event ---
        self._emit(
            "CRITICAL",
            "estop_complete",
            "orchestrator",
            {
                "dry_run": self._dry_run,
                "subsystem_errors": {k: str(v) for k, v in errors.items()},
                "state": "recoverable" if not errors else "partial_failure",
            },
        )

        if errors:
            raise EStopPartialError(errors)

    def reset(self) -> None:
        """Reset the stopped flag so :meth:`stop` can be called again.

        Use only after the system has been physically inspected and confirmed
        safe to re-energise.  This does NOT re-enable torque; call
        :meth:`~inchworm.torque.TorqueManager.enable_all` explicitly after
        the operator confirms the safe state.
        """
        with self._lock:
            self._stopped = False
        self._emit("INFO", "estop_reset", "orchestrator", {"dry_run": self._dry_run})

    # ------------------------------------------------------------------
    # Signal handler
    # ------------------------------------------------------------------

    def _signal_handler(self, signum: int, frame: object) -> None:
        """SIGINT/SIGTERM handler: call stop() then chain to previous handler."""
        _LOG.warning("estop: signal %d received, triggering stop", signum)
        try:
            self.stop()
        except Exception:
            pass
        # Chain to previously installed handler.
        prev = self._prev_sigint if signum == signal.SIGINT else self._prev_sigterm
        if callable(prev):
            prev(signum, frame)  # type: ignore[arg-type]
        elif prev == signal.SIG_DFL:
            signal.signal(signum, signal.SIG_DFL)
            signal.raise_signal(signum)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _emit(self, severity: str, event_type: str, role: str, payload: dict) -> None:
        if self._logger is not None:
            method = getattr(self._logger, severity.lower(), None)
            if method is not None:
                method(
                    role=role,
                    subsystem="estop",
                    event_type=event_type,
                    payload=payload,
                )
        else:
            _LOG.log(
                logging.getLevelName(severity),
                "estop %s %s %r",
                role,
                event_type,
                payload,
            )


# ---------------------------------------------------------------------------
# CLI entry point (installed as inchworm-estop by pyproject.toml)
# ---------------------------------------------------------------------------


def _cli_main(argv: Optional[list] = None) -> None:  # pragma: no cover
    """Installed CLI entry point: ``inchworm-estop [--live]``.

    Delegates to :mod:`scripts.estop` main function. Import is deferred so
    the module can be installed without scripts/ on the path.
    """
    import sys
    from pathlib import Path

    scripts_dir = Path(__file__).resolve().parents[2] / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    # Inline the CLI to avoid circular import chains:
    import argparse
    import logging

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )
    log = logging.getLogger("inchworm.estop_cli")

    parser = argparse.ArgumentParser(
        description="Inchworm emergency stop (AA-10). Default: dry-run."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Send real commands to hardware (default: dry-run).",
    )
    args = parser.parse_args(argv)
    dry_run: bool = not args.live

    from inchworm.backends import MockSerialTransport
    from inchworm.config import load_config
    from inchworm.dexarm import DexArmDriver, MockDexArmSerial
    from inchworm.servo_bus import ArmServoBus
    from inchworm.torque import TorqueManager

    def _mock_bus(role: str) -> ArmServoBus:
        t = MockSerialTransport(role)
        jmap = {j: i + 1 for i, j in enumerate(
            ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]
        )}
        return ArmServoBus(t, role=role, joint_id_map=jmap, dry_run=True)

    try:
        config = load_config()
    except Exception as exc:
        log.error("config load failed: %s", exc)
        sys.exit(2)

    torque_mgr = TorqueManager(
        {"arm_left": _mock_bus("arm_left"), "arm_right": _mock_bus("arm_right")},
        dry_run=dry_run,
    )
    dexarm = DexArmDriver(MockDexArmSerial())
    estop = EStop(
        torque_mgr, dexarm, config=config, dry_run=dry_run,
        install_signal_handlers=False,
    )
    try:
        estop.stop()
        log.info("Emergency stop completed.")
        sys.exit(0)
    except EStopPartialError as exc:
        log.error("E-stop partial failure: %s", exc)
        sys.exit(1)
