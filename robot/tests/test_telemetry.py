"""Tests for inchworm.telemetry and inchworm.session (AA-7 ACs #1, #2, #5).

All tests run without hardware.  No /dev/ttyACM*, cameras, or network I/O.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

import pytest
import yaml

from inchworm.config import load_config
from inchworm.session import SessionContext, new_session
from inchworm.telemetry import EventRecord, SessionLogger


# ---------------------------------------------------------------------------
# AC #1 helpers – EventRecord schema
# ---------------------------------------------------------------------------


class TestEventRecordSchema:
    """AC #1: JSON-lines event records with required fields and correct types."""

    def test_required_fields_present(self):
        """EventRecord must have ts_monotonic, ts_iso, role, subsystem, severity, event_type, payload."""
        field_names = set(EventRecord.model_fields.keys())
        assert {"ts_monotonic", "ts_iso", "role", "subsystem", "severity", "event_type", "payload"}.issubset(
            field_names
        )

    def test_ts_monotonic_is_float(self):
        r = EventRecord(
            ts_monotonic=1.0,
            ts_iso="2026-01-01T00:00:00+00:00",
            role="arm_left",
            subsystem="servo_bus",
            severity="INFO",
            event_type="test_event",
        )
        assert isinstance(r.ts_monotonic, float)

    def test_severity_rejects_invalid(self):
        import pydantic

        with pytest.raises(pydantic.ValidationError):
            EventRecord(
                ts_monotonic=1.0,
                ts_iso="2026-01-01T00:00:00+00:00",
                role="arm_left",
                subsystem="servo_bus",
                severity="TRACE",  # not a valid Severity
                event_type="test_event",
            )

    def test_payload_defaults_to_empty_dict(self):
        r = EventRecord(
            ts_monotonic=1.0,
            ts_iso="2026-01-01T00:00:00+00:00",
            role="arm_left",
            subsystem="servo_bus",
            severity="DEBUG",
            event_type="test_event",
        )
        assert r.payload == {}

    def test_all_severity_literals_accepted(self):
        for sev in ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"):
            r = EventRecord(
                ts_monotonic=1.0,
                ts_iso="2026-01-01T00:00:00+00:00",
                role="orchestrator",
                subsystem="lifecycle",
                severity=sev,  # type: ignore[arg-type]
                event_type="test_event",
            )
            assert r.severity == sev

    def test_model_dump_round_trips_via_json(self):
        r = EventRecord(
            ts_monotonic=42.5,
            ts_iso="2026-10-24T15:30:00+00:00",
            role="dexarm",
            subsystem="gcode",
            severity="WARNING",
            event_type="response_timeout",
            payload={"attempt": 2, "cmd": "G1 X10"},
        )
        dumped = r.model_dump()
        parsed_back = json.loads(json.dumps(dumped))
        assert parsed_back["role"] == "dexarm"
        assert parsed_back["payload"]["attempt"] == 2
        assert parsed_back["severity"] == "WARNING"


# ---------------------------------------------------------------------------
# AC #1 – SessionLogger writes JSON-lines + console output
# ---------------------------------------------------------------------------


class TestSessionLogger:
    """AC #1: SessionLogger writes JSON-lines and emits console messages."""

    def test_writes_jsonlines(self, tmp_path):
        log_file = tmp_path / "events.jsonl"
        with SessionLogger(log_file) as logger:
            logger.info(role="arm_left", subsystem="servo_bus", event_type="torque_on", payload={"id": 1})
            logger.warning(role="dexarm", subsystem="gcode", event_type="retry", payload={"n": 1})
            logger.error(role="relay", subsystem="laser", event_type="failsafe_triggered")

        lines = log_file.read_text(encoding="utf-8").splitlines()
        assert len(lines) == 3

        first = json.loads(lines[0])
        assert first["role"] == "arm_left"
        assert first["subsystem"] == "servo_bus"
        assert first["severity"] == "INFO"
        assert first["event_type"] == "torque_on"
        assert first["payload"] == {"id": 1}
        assert isinstance(first["ts_monotonic"], float)
        assert "ts_iso" in first

        second = json.loads(lines[1])
        assert second["severity"] == "WARNING"

        third = json.loads(lines[2])
        assert third["severity"] == "ERROR"

    def test_console_output_uses_logging(self, tmp_path, caplog):
        log_file = tmp_path / "events.jsonl"
        with caplog.at_level(logging.DEBUG, logger="inchworm.telemetry"):
            with SessionLogger(log_file) as logger:
                logger.debug(role="arm_right", subsystem="servo_bus", event_type="ping")
        assert any("ping" in r.message for r in caplog.records)

    def test_all_convenience_methods_emit_correct_severity(self, tmp_path):
        log_file = tmp_path / "sev.jsonl"
        with SessionLogger(log_file) as logger:
            logger.debug(role="r", subsystem="s", event_type="e")
            logger.info(role="r", subsystem="s", event_type="e")
            logger.warning(role="r", subsystem="s", event_type="e")
            logger.error(role="r", subsystem="s", event_type="e")
            logger.critical(role="r", subsystem="s", event_type="e")

        severities = [json.loads(line)["severity"] for line in log_file.read_text().splitlines()]
        assert severities == ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]

    def test_emit_returns_event_record(self, tmp_path):
        log_file = tmp_path / "e.jsonl"
        with SessionLogger(log_file) as logger:
            rec = logger.emit(
                role="orchestrator",
                subsystem="lifecycle",
                event_type="session_opened",
                severity="INFO",
            )
        assert isinstance(rec, EventRecord)
        assert rec.event_type == "session_opened"

    def test_monotonic_timestamps_are_monotonically_non_decreasing(self, tmp_path):
        log_file = tmp_path / "mono.jsonl"
        with SessionLogger(log_file) as logger:
            for i in range(5):
                logger.info(role="r", subsystem="s", event_type=f"ev{i}")

        ts_values = [json.loads(line)["ts_monotonic"] for line in log_file.read_text().splitlines()]
        for a, b in zip(ts_values, ts_values[1:]):
            assert b >= a, f"monotonic regression: {a} -> {b}"

    def test_file_closed_after_context_exit(self, tmp_path):
        log_file = tmp_path / "closed.jsonl"
        logger = SessionLogger(log_file)
        logger.info(role="r", subsystem="s", event_type="e")
        logger.close()
        assert logger._fh.closed

    def test_explicit_emit_with_all_fields(self, tmp_path):
        log_file = tmp_path / "full.jsonl"
        with SessionLogger(log_file) as logger:
            logger.emit(
                role="arm_left",
                subsystem="gripper",
                event_type="lock_transition",
                severity="INFO",
                payload={"from": "unlocked", "to": "locked"},
            )
        data = json.loads(log_file.read_text().splitlines()[0])
        assert data["payload"]["from"] == "unlocked"


# ---------------------------------------------------------------------------
# AC #2 – Session directory structure
# ---------------------------------------------------------------------------


class TestSessionDirectory:
    """AC #2: new_session creates the expected directory layout."""

    def test_session_dir_created_under_root(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            assert ctx.session_dir.exists()
            assert ctx.session_dir.parent == tmp_path

    def test_session_dir_name_format(self, tmp_path):
        import re

        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            name = ctx.session_dir.name
        # Expected: YYYYMMDD_HHMMSS_<4hexchars>
        assert re.fullmatch(r"\d{8}_\d{6}_[0-9a-f]{4}", name), f"Unexpected name: {name!r}"

    def test_event_log_created(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            assert ctx.log_path.name == "event_log.jsonl"
            assert ctx.log_path.parent == ctx.session_dir
            # File exists (may be empty if no events emitted yet)
            assert ctx.log_path.exists()

    def test_config_snapshot_written(self, tmp_path):
        from inchworm.config import parse_config

        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            snap_path = ctx.config_snapshot_path
        assert snap_path.exists()
        # Must be valid YAML that round-trips to a HardwareConfig
        loaded = yaml.safe_load(snap_path.read_text(encoding="utf-8"))
        round_tripped = parse_config(loaded)
        assert round_tripped.schema_version == config.schema_version
        assert round_tripped.backend == config.backend

    def test_config_snapshot_preserves_device_identities(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            snap_path = ctx.config_snapshot_path

        from inchworm.config import parse_config

        loaded = yaml.safe_load(snap_path.read_text(encoding="utf-8"))
        rt = parse_config(loaded)
        assert rt.arms.left.identity.usb_serial == config.arms.left.identity.usb_serial
        assert rt.arms.right.identity.usb_serial == config.arms.right.identity.usb_serial

    def test_artifact_dir_created(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            assert ctx.artifact_dir.exists()
            assert ctx.artifact_dir.is_dir()
            assert ctx.artifact_dir.name == "artifacts"
            assert ctx.artifact_dir.parent == ctx.session_dir

    def test_session_logger_bound_and_writes(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            ctx.logger.info(
                role="orchestrator",
                subsystem="lifecycle",
                event_type="session_opened",
            )
        lines = ctx.log_path.read_text(encoding="utf-8").splitlines()
        assert len(lines) == 1
        assert json.loads(lines[0])["event_type"] == "session_opened"

    def test_context_manager_closes_logger(self, tmp_path):
        config = load_config()
        with new_session(config, session_root=tmp_path) as ctx:
            pass
        assert ctx.logger._fh.closed

    def test_successive_sessions_get_unique_dirs(self, tmp_path):
        import time

        config = load_config()
        dirs = []
        for _ in range(3):
            with new_session(config, session_root=tmp_path) as ctx:
                dirs.append(ctx.session_dir)
            time.sleep(0.001)  # ensure timestamp progression where possible
        assert len(set(dirs)) == 3, "Session dirs must be unique"

    def test_default_session_root_is_under_robot(self):
        """Default session root resolves to robot/sessions/ (gitignored)."""
        from inchworm.session import _DEFAULT_SESSION_ROOT

        # Must be named 'sessions' and live inside the robot package tree.
        assert _DEFAULT_SESSION_ROOT.name == "sessions"
        # Parent should be robot/ (two levels up from src/inchworm/)
        assert _DEFAULT_SESSION_ROOT.parent.name == "robot"


# ---------------------------------------------------------------------------
# AC #5 – Gitignore coverage
# ---------------------------------------------------------------------------


class TestGitignore:
    """AC #5: session artifact paths are gitignored."""

    # tests/test_telemetry.py -> tests -> robot -> 893 (repo root)
    _REPO_ROOT = Path(__file__).resolve().parents[2]

    def test_gitignore_covers_sessions(self):
        """robot/sessions/ must appear in the repo .gitignore."""
        gitignore = self._REPO_ROOT / ".gitignore"
        assert gitignore.exists(), ".gitignore not found at repo root"
        content = gitignore.read_text(encoding="utf-8")
        assert "robot/sessions/" in content, (
            "robot/sessions/ not found in .gitignore — session artifacts could be committed"
        )

    def test_gitignore_covers_robot_out(self):
        """robot/out/ must also be gitignored (artifacts sink)."""
        content = (self._REPO_ROOT / ".gitignore").read_text(encoding="utf-8")
        assert "robot/out/" in content

    def test_default_session_root_matches_gitignored_path(self):
        """The runtime default session root must resolve to the gitignored path."""
        from inchworm.session import _DEFAULT_SESSION_ROOT

        repo_root = self._REPO_ROOT
        gitignore = (repo_root / ".gitignore").read_text(encoding="utf-8")
        # sessions/ is under robot/, and robot/sessions/ is in .gitignore
        expected_relative = "robot/sessions/"
        assert expected_relative in gitignore
        # _DEFAULT_SESSION_ROOT is an absolute path; its relative form (from repo root)
        # should match the gitignored pattern.
        rel = _DEFAULT_SESSION_ROOT.relative_to(repo_root)
        assert str(rel).replace("\\", "/") + "/" == expected_relative
