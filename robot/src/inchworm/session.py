"""Per-run session directory management.

A *session* is a single operator or autonomous run.  Calling
:func:`new_session` allocates a timestamped directory under
``robot/sessions/`` (already gitignored), writes a config snapshot, creates
the artifacts sub-directory, and returns a :class:`SessionContext` that holds
the :class:`~inchworm.telemetry.SessionLogger` for that run.

The ``robot/sessions/`` root is gitignored; no session file is ever committed.

Typical usage::

    from inchworm.session import new_session
    from inchworm.config import load_config

    with new_session(load_config()) as ctx:
        ctx.logger.info(
            role="orchestrator",
            subsystem="lifecycle",
            event_type="session_opened",
        )
        # … do work …
        artifact = ctx.artifact_dir / "frame_001.png"
"""

from __future__ import annotations

import secrets
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml

if TYPE_CHECKING:
    from inchworm.config import HardwareConfig

from inchworm.telemetry import SessionLogger

# ---------------------------------------------------------------------------
# Default session root (already gitignored via robot/sessions/ in .gitignore)
# ---------------------------------------------------------------------------

_DEFAULT_SESSION_ROOT = Path(__file__).resolve().parents[2] / "sessions"


# ---------------------------------------------------------------------------
# SessionContext
# ---------------------------------------------------------------------------


class SessionContext:
    """Holds all per-run paths and the event logger for one session.

    Attributes
    ----------
    session_dir:
        Root directory for this run (e.g. ``robot/sessions/20261024_153000_a3f1/``).
    log_path:
        Path to the JSON-lines event log (``event_log.jsonl`` inside session_dir).
    config_snapshot_path:
        Path to the resolved config snapshot (``config_snapshot.yaml``).
    artifact_dir:
        Sub-directory for captured artifacts (frames, datasets, etc.).
    logger:
        :class:`~inchworm.telemetry.SessionLogger` bound to this session.
    """

    def __init__(
        self,
        session_dir: Path,
        log_path: Path,
        config_snapshot_path: Path,
        artifact_dir: Path,
        logger: SessionLogger,
    ) -> None:
        self.session_dir = session_dir
        self.log_path = log_path
        self.config_snapshot_path = config_snapshot_path
        self.artifact_dir = artifact_dir
        self.logger = logger

    def close(self) -> None:
        """Flush and close the session logger."""
        self.logger.close()

    def __enter__(self) -> "SessionContext":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def new_session(
    config: "HardwareConfig",
    *,
    session_root: Path | None = None,
) -> SessionContext:
    """Create a new session directory and return a :class:`SessionContext`.

    Parameters
    ----------
    config:
        The resolved :class:`~inchworm.config.HardwareConfig` for this run.
        It is serialised to ``config_snapshot.yaml`` inside the session dir.
    session_root:
        Override for the parent of all session dirs.  Defaults to
        ``robot/sessions/`` (gitignored).  Useful in tests via ``tmp_path``.

    Returns
    -------
    SessionContext
        Open session; call :meth:`~SessionContext.close` (or use as a context
        manager) when the run ends.
    """
    root = session_root if session_root is not None else _DEFAULT_SESSION_ROOT
    ts = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
    suffix = secrets.token_hex(2)  # 4 hex chars, e.g. "a3f1"
    session_dir = root / f"{ts}_{suffix}"
    session_dir.mkdir(parents=True, exist_ok=False)

    artifact_dir = session_dir / "artifacts"
    artifact_dir.mkdir()

    # Write config snapshot as YAML so it is human-readable and round-trippable.
    config_snapshot_path = session_dir / "config_snapshot.yaml"
    _write_config_snapshot(config, config_snapshot_path)

    log_path = session_dir / "event_log.jsonl"
    logger = SessionLogger(log_path)

    return SessionContext(
        session_dir=session_dir,
        log_path=log_path,
        config_snapshot_path=config_snapshot_path,
        artifact_dir=artifact_dir,
        logger=logger,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _write_config_snapshot(config: "HardwareConfig", path: Path) -> None:
    """Serialise *config* to *path* as YAML.

    We serialise via Pydantic's ``model_dump()`` so enums become their string
    values and the output round-trips through :func:`~inchworm.config.parse_config`.
    """
    data = config.model_dump(mode="json")
    path.write_text(
        yaml.dump(data, default_flow_style=False, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
