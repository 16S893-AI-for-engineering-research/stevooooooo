"""Structured event logger and per-run session directory management.

Every event is a JSON-lines record written to the session log file **and**
emitted as a human-readable message to a stdlib ``logging`` logger (so that
tests and interactive runs see console output without touching files).

Usage::

    from inchworm.session import new_session
    from inchworm.config import load_config

    config = load_config()
    with new_session(config) as ctx:
        ctx.logger.info(
            role="arm_left",
            subsystem="servo_bus",
            event_type="servo_torque_enabled",
            payload={"servo_id": 1},
        )

The session directory layout (under ``robot/sessions/<timestamp>_<hex>/``)::

    event_log.jsonl        – JSON-lines, one EventRecord per line
    config_snapshot.yaml   – resolved HardwareConfig at session open time
    artifacts/             – caller-managed captured files (frames, etc.)
"""

from __future__ import annotations

import json
import logging
import time
from datetime import datetime, timezone
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

# ---------------------------------------------------------------------------
# Event record schema
# ---------------------------------------------------------------------------

Severity = Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]

_SEVERITY_TO_LOGGING: dict[str, int] = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}


class EventRecord(BaseModel):
    """A single structured event emitted during a robot session.

    Constraint: ``ts_monotonic`` is seconds since the epoch of
    ``time.monotonic_ns()`` — not wall-clock zero — so it is suitable for
    measuring durations within a session but not for absolute timestamps.
    Use ``ts_iso`` for absolute correlation.
    """

    model_config = ConfigDict(frozen=True)

    # Monotonic time in fractional seconds (nanosecond resolution).
    ts_monotonic: float = Field(
        description="time.monotonic_ns() / 1e9 at event creation"
    )
    # ISO 8601 UTC wall-clock time.
    ts_iso: str = Field(
        description="UTC wall-clock time in ISO 8601 format"
    )
    # Logical device role (e.g. 'arm_left', 'dexarm', 'relay', 'orchestrator').
    role: str = Field(description="logical device or component role")
    # Coarse subsystem within the role (e.g. 'servo_bus', 'gripper', 'policy').
    subsystem: str = Field(description="subsystem within the role")
    # Severity level matching Python logging levels.
    severity: Severity = Field(description="log severity")
    # Machine-readable event classifier (e.g. 'servo_torque_enabled').
    event_type: str = Field(description="machine-readable event classifier")
    # Arbitrary key/value pairs specific to the event_type.
    payload: dict[str, Any] = Field(
        default_factory=dict,
        description="event-type-specific structured fields",
    )


# ---------------------------------------------------------------------------
# Session logger
# ---------------------------------------------------------------------------

_LOG = logging.getLogger("inchworm.telemetry")


class SessionLogger:
    """Writes :class:`EventRecord` objects to a JSON-lines file and the console.

    Parameters
    ----------
    log_path:
        Path to the JSON-lines file (created/truncated on construction).
    console_logger:
        stdlib Logger used for human-readable output; defaults to
        ``logging.getLogger("inchworm.telemetry")``.
    """

    def __init__(
        self,
        log_path: Any,  # pathlib.Path or str
        console_logger: logging.Logger | None = None,
    ) -> None:
        from pathlib import Path

        self._path = Path(log_path)
        self._console = console_logger or _LOG
        self._fh = self._path.open("w", encoding="utf-8", buffering=1)  # line-buffered

    # ------------------------------------------------------------------
    # Public emit methods
    # ------------------------------------------------------------------

    def emit(
        self,
        *,
        role: str,
        subsystem: str,
        event_type: str,
        severity: Severity = "INFO",
        payload: dict[str, Any] | None = None,
    ) -> EventRecord:
        """Create, record, and return an :class:`EventRecord`."""
        record = EventRecord(
            ts_monotonic=time.monotonic_ns() / 1e9,
            ts_iso=datetime.now(tz=timezone.utc).isoformat(),
            role=role,
            subsystem=subsystem,
            severity=severity,
            event_type=event_type,
            payload=payload or {},
        )
        self._write(record)
        return record

    def debug(self, *, role: str, subsystem: str, event_type: str, payload: dict[str, Any] | None = None) -> EventRecord:
        return self.emit(role=role, subsystem=subsystem, event_type=event_type, severity="DEBUG", payload=payload)

    def info(self, *, role: str, subsystem: str, event_type: str, payload: dict[str, Any] | None = None) -> EventRecord:
        return self.emit(role=role, subsystem=subsystem, event_type=event_type, severity="INFO", payload=payload)

    def warning(self, *, role: str, subsystem: str, event_type: str, payload: dict[str, Any] | None = None) -> EventRecord:
        return self.emit(role=role, subsystem=subsystem, event_type=event_type, severity="WARNING", payload=payload)

    def error(self, *, role: str, subsystem: str, event_type: str, payload: dict[str, Any] | None = None) -> EventRecord:
        return self.emit(role=role, subsystem=subsystem, event_type=event_type, severity="ERROR", payload=payload)

    def critical(self, *, role: str, subsystem: str, event_type: str, payload: dict[str, Any] | None = None) -> EventRecord:
        return self.emit(role=role, subsystem=subsystem, event_type=event_type, severity="CRITICAL", payload=payload)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Flush and close the underlying file handle."""
        if not self._fh.closed:
            self._fh.flush()
            self._fh.close()

    def __enter__(self) -> "SessionLogger":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _write(self, record: EventRecord) -> None:
        line = json.dumps(record.model_dump(), ensure_ascii=False)
        self._fh.write(line + "\n")
        # Human-readable console output
        msg = (
            f"[{record.ts_iso}] {record.severity:8s} "
            f"{record.role}/{record.subsystem} {record.event_type} {record.payload!r}"
        )
        self._console.log(_SEVERITY_TO_LOGGING[record.severity], msg)
