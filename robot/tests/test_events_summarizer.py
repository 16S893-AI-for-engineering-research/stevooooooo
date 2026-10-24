"""Tests for AA-7 ACs #3, #4, #6.

AC #3 – Typed events: motion commands, gripper lock/unlock, policy handoffs,
         aborts and retries each emit a distinct typed event with the fields
         needed to reconstruct the cycle.
AC #4 – Summariser: renders a per-cycle timeline from the event log alone.
AC #6 – Schema stability and summariser tests on a synthetic session.

All tests run without hardware.  No /dev/ttyACM*, cameras, or network I/O.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from inchworm import events as ev
from inchworm.summarizer import (
    CycleSummary,
    SessionTimeline,
    format_timeline,
    summarize_session,
)
from inchworm.telemetry import SessionLogger


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _write_log(tmp_path: Path, records: list[dict[str, Any]]) -> Path:
    """Write *records* as a JSON-lines file and return the path."""
    log = tmp_path / "event_log.jsonl"
    with log.open("w", encoding="utf-8") as fh:
        for r in records:
            fh.write(json.dumps(r) + "\n")
    return log


def _emit_all(logger: SessionLogger, kwargs_list: list[dict[str, Any]]) -> None:
    """Emit each kwargs dict through *logger*."""
    for kw in kwargs_list:
        logger.emit(**kw)


# ---------------------------------------------------------------------------
# AC #3 – typed event constructors: schema and field checks
# ---------------------------------------------------------------------------


class TestTypedEventMotion:
    """motion_command() emits the correct event_type and required payload fields."""

    def test_event_type_is_motion_command_sent(self):
        kw = ev.motion_command(
            role="arm_left",
            target_type="joint_angles",
            target={"shoulder": 10.0, "elbow": -5.0},
        )
        assert kw["event_type"] == "motion_command_sent"

    def test_role_propagated_to_top_level(self):
        kw = ev.motion_command(
            role="dexarm",
            target_type="gcode",
            target={"cmd": "G1 X10 F300"},
        )
        assert kw["role"] == "dexarm"

    def test_payload_has_required_fields(self):
        kw = ev.motion_command(
            role="arm_right",
            target_type="cartesian_pose",
            target={"x": 0.1, "y": 0.2, "z": 0.3},
        )
        p = kw["payload"]
        assert "role" in p
        assert "target_type" in p
        assert "target" in p
        assert "speed_fraction" in p
        assert "dry_run" in p

    def test_payload_target_type_values(self):
        for tt in ("joint_angles", "cartesian_pose", "gcode"):
            kw = ev.motion_command(role="arm_left", target_type=tt, target={})  # type: ignore[arg-type]
            assert kw["payload"]["target_type"] == tt

    def test_dry_run_flag_defaults_false(self):
        kw = ev.motion_command(role="arm_left", target_type="joint_angles", target={})
        assert kw["payload"]["dry_run"] is False

    def test_dry_run_flag_can_be_set(self):
        kw = ev.motion_command(
            role="arm_left", target_type="joint_angles", target={}, dry_run=True
        )
        assert kw["payload"]["dry_run"] is True

    def test_speed_fraction_optional(self):
        kw = ev.motion_command(role="arm_left", target_type="joint_angles", target={})
        assert kw["payload"]["speed_fraction"] is None

    def test_speed_fraction_set(self):
        kw = ev.motion_command(
            role="arm_left",
            target_type="joint_angles",
            target={},
            speed_fraction=0.5,
        )
        assert kw["payload"]["speed_fraction"] == 0.5

    def test_severity_default_info(self):
        kw = ev.motion_command(role="arm_left", target_type="joint_angles", target={})
        assert kw["severity"] == "INFO"

    def test_payload_is_json_serialisable(self):
        kw = ev.motion_command(
            role="arm_left",
            target_type="joint_angles",
            target={"shoulder": 10.0},
            speed_fraction=0.8,
        )
        # Must not raise
        json.dumps(kw["payload"])

    def test_round_trip_via_session_logger(self, tmp_path):
        log = tmp_path / "e.jsonl"
        with SessionLogger(log) as logger:
            logger.emit(
                **ev.motion_command(
                    role="arm_left",
                    target_type="joint_angles",
                    target={"shoulder": 5.0},
                )
            )
        data = json.loads(log.read_text().strip())
        assert data["event_type"] == "motion_command_sent"
        assert data["payload"]["target"]["shoulder"] == 5.0


class TestTypedEventGripper:
    """gripper_lock() / gripper_unlock() emit the correct typed events."""

    def test_gripper_lock_event_type(self):
        kw = ev.gripper_lock(role="arm_left")
        assert kw["event_type"] == "gripper_lock"

    def test_gripper_unlock_event_type(self):
        kw = ev.gripper_unlock(role="arm_right")
        assert kw["event_type"] == "gripper_unlock"

    def test_gripper_lock_to_state(self):
        kw = ev.gripper_lock(role="arm_left")
        assert kw["payload"]["to_state"] == "locked"

    def test_gripper_unlock_to_state(self):
        kw = ev.gripper_unlock(role="arm_right")
        assert kw["payload"]["to_state"] == "unlocked"

    def test_gripper_lock_from_state_default_unlocked(self):
        kw = ev.gripper_lock(role="arm_left")
        assert kw["payload"]["from_state"] == "unlocked"

    def test_gripper_unlock_from_state_default_locked(self):
        kw = ev.gripper_unlock(role="arm_right")
        assert kw["payload"]["from_state"] == "locked"

    def test_gripper_payload_required_fields(self):
        kw = ev.gripper_lock(role="arm_left")
        p = kw["payload"]
        for f in ("role", "from_state", "to_state", "servo_id"):
            assert f in p, f"missing field: {f}"

    def test_gripper_servo_id_optional(self):
        kw = ev.gripper_lock(role="arm_left")
        assert kw["payload"]["servo_id"] is None

    def test_gripper_servo_id_set(self):
        kw = ev.gripper_lock(role="arm_left", servo_id=3)
        assert kw["payload"]["servo_id"] == 3

    def test_gripper_severity_info(self):
        assert ev.gripper_lock(role="arm_left")["severity"] == "INFO"
        assert ev.gripper_unlock(role="arm_right")["severity"] == "INFO"

    def test_gripper_from_state_unknown(self):
        kw = ev.gripper_lock(role="arm_left", from_state="unknown")
        assert kw["payload"]["from_state"] == "unknown"


class TestTypedEventPolicy:
    """policy_handoff_start() / policy_handoff_end() emit correct typed events."""

    def test_handoff_start_event_type(self):
        kw = ev.policy_handoff_start(policy_name="insertion_v1", cycle_index=0)
        assert kw["event_type"] == "policy_handoff_start"

    def test_handoff_end_event_type(self):
        kw = ev.policy_handoff_end(policy_name="insertion_v1", cycle_index=0)
        assert kw["event_type"] == "policy_handoff_end"

    def test_handoff_start_direction(self):
        kw = ev.policy_handoff_start(policy_name="insertion_v1", cycle_index=0)
        assert kw["payload"]["direction"] == "code_to_policy"

    def test_handoff_end_direction(self):
        kw = ev.policy_handoff_end(policy_name="insertion_v1", cycle_index=0)
        assert kw["payload"]["direction"] == "policy_to_code"

    def test_payload_required_fields(self):
        kw = ev.policy_handoff_start(policy_name="insertion_v1", cycle_index=2)
        p = kw["payload"]
        for f in ("direction", "policy_name", "cycle_index", "reason"):
            assert f in p

    def test_reason_optional_start(self):
        kw = ev.policy_handoff_start(policy_name="insertion_v1", cycle_index=0)
        assert kw["payload"]["reason"] is None

    def test_reason_end_can_be_set(self):
        kw = ev.policy_handoff_end(
            policy_name="insertion_v1", cycle_index=0, reason="completed"
        )
        assert kw["payload"]["reason"] == "completed"

    def test_cycle_index_in_payload(self):
        kw = ev.policy_handoff_start(policy_name="p", cycle_index=7)
        assert kw["payload"]["cycle_index"] == 7


class TestTypedEventAbort:
    """cycle_abort() emits the correct typed event."""

    def test_event_type(self):
        kw = ev.cycle_abort(cycle_index=0, reason="joint_stall", subsystem="gripper")
        assert kw["event_type"] == "cycle_abort"

    def test_severity_error(self):
        kw = ev.cycle_abort(cycle_index=0, reason="joint_stall", subsystem="gripper")
        assert kw["severity"] == "ERROR"

    def test_required_payload_fields(self):
        kw = ev.cycle_abort(cycle_index=1, reason="timeout", subsystem="servo_bus")
        p = kw["payload"]
        for f in ("cycle_index", "reason", "detail", "subsystem"):
            assert f in p

    def test_cycle_index(self):
        kw = ev.cycle_abort(cycle_index=3, reason="r", subsystem="s")
        assert kw["payload"]["cycle_index"] == 3

    def test_reason_field(self):
        kw = ev.cycle_abort(cycle_index=0, reason="insertion_failed", subsystem="policy")
        assert kw["payload"]["reason"] == "insertion_failed"

    def test_detail_optional(self):
        kw = ev.cycle_abort(cycle_index=0, reason="r", subsystem="s")
        assert kw["payload"]["detail"] is None

    def test_detail_set(self):
        kw = ev.cycle_abort(
            cycle_index=0, reason="r", subsystem="s", detail="servo id 3 timeout"
        )
        assert kw["payload"]["detail"] == "servo id 3 timeout"


class TestTypedEventRetry:
    """cycle_retry() emits the correct typed event."""

    def test_event_type(self):
        kw = ev.cycle_retry(
            cycle_index=0, attempt=1, max_attempts=3, reason="timeout", subsystem="gcode"
        )
        assert kw["event_type"] == "cycle_retry"

    def test_severity_warning(self):
        kw = ev.cycle_retry(
            cycle_index=0, attempt=1, max_attempts=3, reason="r", subsystem="s"
        )
        assert kw["severity"] == "WARNING"

    def test_required_payload_fields(self):
        kw = ev.cycle_retry(
            cycle_index=0, attempt=2, max_attempts=5, reason="r", subsystem="s"
        )
        p = kw["payload"]
        for f in ("cycle_index", "attempt", "max_attempts", "reason", "subsystem"):
            assert f in p

    def test_attempt_number(self):
        kw = ev.cycle_retry(
            cycle_index=0, attempt=2, max_attempts=3, reason="r", subsystem="s"
        )
        assert kw["payload"]["attempt"] == 2
        assert kw["payload"]["max_attempts"] == 3


class TestTypedEventCycleBookends:
    """cycle_start() / cycle_end() emit the correct typed events."""

    def test_cycle_start_event_type(self):
        assert ev.cycle_start(cycle_index=0)["event_type"] == "cycle_start"

    def test_cycle_end_event_type(self):
        assert ev.cycle_end(cycle_index=0)["event_type"] == "cycle_end"

    def test_cycle_index_payload(self):
        assert ev.cycle_start(cycle_index=5)["payload"]["cycle_index"] == 5

    def test_phase_optional(self):
        kw = ev.cycle_start(cycle_index=0)
        assert kw["payload"]["phase"] is None

    def test_phase_set(self):
        kw = ev.cycle_start(cycle_index=0, phase="approach")
        assert kw["payload"]["phase"] == "approach"

    def test_all_event_types_are_distinct(self):
        """AC #3: every typed event has a unique event_type string."""
        event_types = [
            ev.motion_command(role="r", target_type="joint_angles", target={})["event_type"],
            ev.gripper_lock(role="r")["event_type"],
            ev.gripper_unlock(role="r")["event_type"],
            ev.policy_handoff_start(policy_name="p", cycle_index=0)["event_type"],
            ev.policy_handoff_end(policy_name="p", cycle_index=0)["event_type"],
            ev.cycle_abort(cycle_index=0, reason="r", subsystem="s")["event_type"],
            ev.cycle_retry(cycle_index=0, attempt=1, max_attempts=2, reason="r", subsystem="s")["event_type"],
            ev.cycle_start(cycle_index=0)["event_type"],
            ev.cycle_end(cycle_index=0)["event_type"],
        ]
        assert len(set(event_types)) == len(event_types), (
            f"Duplicate event_type values: {event_types}"
        )


# ---------------------------------------------------------------------------
# AC #4 + AC #6 – summarizer on synthetic sessions
# ---------------------------------------------------------------------------


def _build_synthetic_log(
    tmp_path: Path, *, include_abort: bool = False, n_cycles: int = 2
) -> Path:
    """Build a synthetic event_log.jsonl and return its path.

    Cycle 0: start → motion × 2 → gripper_lock → policy_handoff_start →
             policy_handoff_end → gripper_unlock → end
    Cycle 1: start → motion × 1 → retry → abort   (when include_abort=True)
             or start → motion × 1 → end           (otherwise)
    """
    log = tmp_path / "event_log.jsonl"
    ts = 100.0  # monotonic start

    def rec(event_type: str, payload: dict, cycle_idx: int | None = None) -> dict:
        nonlocal ts
        p = dict(payload)
        if cycle_idx is not None:
            p.setdefault("cycle_index", cycle_idx)
        r = {
            "ts_monotonic": ts,
            "ts_iso": f"2026-10-24T15:00:{ts:.3f}+00:00",
            "role": "orchestrator",
            "subsystem": "lifecycle",
            "severity": "INFO",
            "event_type": event_type,
            "payload": p,
        }
        ts += 1.0
        return r

    records: list[dict] = []

    # Cycle 0
    records.append(rec("cycle_start", {"cycle_index": 0}))
    records.append(rec("motion_command_sent", {"role": "arm_left", "target_type": "joint_angles", "target": {}, "speed_fraction": None, "dry_run": False, "cycle_index": 0}))
    records.append(rec("motion_command_sent", {"role": "arm_right", "target_type": "cartesian_pose", "target": {}, "speed_fraction": None, "dry_run": False, "cycle_index": 0}))
    records.append(rec("gripper_lock", {"role": "arm_left", "from_state": "unlocked", "to_state": "locked", "servo_id": None, "cycle_index": 0}))
    records.append(rec("policy_handoff_start", {"direction": "code_to_policy", "policy_name": "insertion_v1", "cycle_index": 0, "reason": None}))
    records.append(rec("policy_handoff_end", {"direction": "policy_to_code", "policy_name": "insertion_v1", "cycle_index": 0, "reason": "completed"}))
    records.append(rec("gripper_unlock", {"role": "arm_left", "from_state": "locked", "to_state": "unlocked", "servo_id": None, "cycle_index": 0}))
    records.append(rec("cycle_end", {"cycle_index": 0}))

    if n_cycles >= 2:
        # Cycle 1
        records.append(rec("cycle_start", {"cycle_index": 1}))
        records.append(rec("motion_command_sent", {"role": "dexarm", "target_type": "gcode", "target": {"cmd": "G1 X10"}, "speed_fraction": None, "dry_run": False, "cycle_index": 1}))
        if include_abort:
            records.append(rec("cycle_retry", {"cycle_index": 1, "attempt": 1, "max_attempts": 3, "reason": "joint_stall", "subsystem": "servo_bus"}))
            records.append(rec("cycle_abort", {"cycle_index": 1, "reason": "joint_stall", "detail": "servo 3 timeout", "subsystem": "servo_bus"}))
        else:
            records.append(rec("cycle_end", {"cycle_index": 1}))

    log.write_text(
        "\n".join(json.dumps(r) for r in records) + "\n",
        encoding="utf-8",
    )
    return log


class TestSummarizerBasic:
    """AC #4 + AC #6: summarizer returns correct structure for simple cases."""

    def test_total_events_counted(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        # cycle 0: 8 events; cycle 1: 3 events = 11
        assert tl.total_events == 11

    def test_two_cycles_found(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        assert len(tl.cycles) == 2

    def test_cycle_0_completed(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        c0 = tl.get_cycle(0)
        assert c0 is not None
        assert c0.outcome == "completed"

    def test_cycle_0_motion_count(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        assert tl.get_cycle(0).motion_count == 2

    def test_cycle_0_gripper_transitions(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        c0 = tl.get_cycle(0)
        assert len(c0.gripper_transitions) == 2
        assert c0.gripper_transitions[0]["event_type"] == "gripper_lock"
        assert c0.gripper_transitions[1]["event_type"] == "gripper_unlock"

    def test_cycle_0_policy_handoffs(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        c0 = tl.get_cycle(0)
        assert len(c0.policy_handoffs) == 2
        directions = [p["direction"] for p in c0.policy_handoffs]
        assert "code_to_policy" in directions
        assert "policy_to_code" in directions

    def test_cycle_0_duration_positive(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        c0 = tl.get_cycle(0)
        assert c0.duration_s is not None
        assert c0.duration_s > 0.0

    def test_cycle_1_end_to_end(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        c1 = tl.get_cycle(1)
        assert c1 is not None
        assert c1.outcome == "completed"
        assert c1.motion_count == 1

    def test_cycles_sorted_by_index(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        indices = [c.cycle_index for c in tl.cycles]
        assert indices == sorted(indices)

    def test_empty_log(self, tmp_path):
        log = tmp_path / "event_log.jsonl"
        log.write_text("", encoding="utf-8")
        tl = summarize_session(log)
        assert tl.total_events == 0
        assert tl.cycles == []

    def test_partial_log_no_crash(self, tmp_path):
        """A truncated line must not crash the summariser."""
        log = tmp_path / "event_log.jsonl"
        log.write_text(
            json.dumps({"event_type": "cycle_start", "payload": {"cycle_index": 0},
                        "ts_monotonic": 1.0, "ts_iso": "x", "role": "o", "subsystem": "l", "severity": "INFO"})
            + "\n{broken json\n",
            encoding="utf-8",
        )
        tl = summarize_session(log)
        assert tl.total_events == 1  # only the valid line


class TestSummarizerAbort:
    """AC #4 + AC #6: abort and retry events are surfaced correctly."""

    def test_aborted_cycle_outcome(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        c1 = tl.get_cycle(1)
        assert c1.outcome == "aborted"

    def test_abort_reason_captured(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        assert tl.get_cycle(1).abort_reason == "joint_stall"

    def test_retry_recorded(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        c1 = tl.get_cycle(1)
        assert len(c1.retries) == 1
        assert c1.retries[0]["attempt"] == 1
        assert c1.retries[0]["max_attempts"] == 3

    def test_abort_does_not_corrupt_cycle_0(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        assert tl.get_cycle(0).outcome == "completed"


class TestSummarizerFormatter:
    """AC #4: format_timeline produces the required output sections."""

    def test_output_is_string(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        assert isinstance(format_timeline(tl), str)

    def test_contains_cycle_header(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "Cycle 0" in out
        assert "Cycle 1" in out

    def test_contains_outcome_labels(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "completed" in out

    def test_abort_label_in_aborted_session(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "aborted" in out
        assert "ABORT" in out
        assert "joint_stall" in out

    def test_contains_motion_count(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "motion:2" in out  # cycle 0 has 2 motions

    def test_contains_gripper_summary(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "gripper:2" in out  # cycle 0 has 2 gripper events

    def test_contains_policy_summary(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "policy:2" in out  # cycle 0 has 2 policy events

    def test_contains_duration(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        # Duration is rendered as "X.XXX s"
        assert " s" in out

    def test_header_line_present(self, tmp_path):
        log = _build_synthetic_log(tmp_path, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "Session timeline" in out

    def test_retry_count_in_output(self, tmp_path):
        log = _build_synthetic_log(tmp_path, include_abort=True, n_cycles=2)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "retry:1" in out

    def test_single_cycle_word_form(self, tmp_path):
        """Formatter uses 'cycle' (not 'cycles') when there is exactly one."""
        log = _build_synthetic_log(tmp_path, n_cycles=1)
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "1 cycle," in out

    def test_orphan_events_reported(self, tmp_path):
        """Events without cycle_index appear in orphan section."""
        log = tmp_path / "event_log.jsonl"
        records = [
            {
                "ts_monotonic": 1.0, "ts_iso": "x", "role": "o", "subsystem": "l",
                "severity": "INFO", "event_type": "session_opened", "payload": {},
            }
        ]
        log.write_text("\n".join(json.dumps(r) for r in records) + "\n")
        tl = summarize_session(log)
        out = format_timeline(tl)
        assert "Orphan" in out


# ---------------------------------------------------------------------------
# AC #6 – schema stability: payload model field invariants
# ---------------------------------------------------------------------------


class TestPayloadSchemaStability:
    """AC #6: payload models have stable required fields; renaming breaks this test."""

    def test_motion_payload_fields(self):
        kw = ev.motion_command(role="r", target_type="joint_angles", target={})
        p = kw["payload"]
        required = {"role", "target_type", "target", "speed_fraction", "dry_run"}
        assert required == set(p.keys()), (
            f"motion_command payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_gripper_lock_payload_fields(self):
        kw = ev.gripper_lock(role="r")
        p = kw["payload"]
        required = {"role", "from_state", "to_state", "servo_id"}
        assert required == set(p.keys()), (
            f"gripper_lock payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_gripper_unlock_payload_fields(self):
        kw = ev.gripper_unlock(role="r")
        p = kw["payload"]
        required = {"role", "from_state", "to_state", "servo_id"}
        assert required == set(p.keys()), (
            f"gripper_unlock payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_policy_handoff_start_payload_fields(self):
        kw = ev.policy_handoff_start(policy_name="p", cycle_index=0)
        p = kw["payload"]
        required = {"direction", "policy_name", "cycle_index", "reason"}
        assert required == set(p.keys()), (
            f"policy_handoff_start payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_policy_handoff_end_payload_fields(self):
        kw = ev.policy_handoff_end(policy_name="p", cycle_index=0)
        p = kw["payload"]
        required = {"direction", "policy_name", "cycle_index", "reason"}
        assert required == set(p.keys()), (
            f"policy_handoff_end payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_abort_payload_fields(self):
        kw = ev.cycle_abort(cycle_index=0, reason="r", subsystem="s")
        p = kw["payload"]
        required = {"cycle_index", "reason", "detail", "subsystem"}
        assert required == set(p.keys()), (
            f"cycle_abort payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_retry_payload_fields(self):
        kw = ev.cycle_retry(cycle_index=0, attempt=1, max_attempts=3, reason="r", subsystem="s")
        p = kw["payload"]
        required = {"cycle_index", "attempt", "max_attempts", "reason", "subsystem"}
        assert required == set(p.keys()), (
            f"cycle_retry payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_cycle_start_payload_fields(self):
        kw = ev.cycle_start(cycle_index=0)
        p = kw["payload"]
        required = {"cycle_index", "phase"}
        assert required == set(p.keys()), (
            f"cycle_start payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_cycle_end_payload_fields(self):
        kw = ev.cycle_end(cycle_index=0)
        p = kw["payload"]
        required = {"cycle_index", "phase"}
        assert required == set(p.keys()), (
            f"cycle_end payload fields changed: expected {required}, got {set(p.keys())}"
        )

    def test_event_record_fields_stable(self):
        """EventRecord schema is the foundation; its fields must never silently change."""
        from inchworm.telemetry import EventRecord

        required = {"ts_monotonic", "ts_iso", "role", "subsystem", "severity", "event_type", "payload"}
        actual = set(EventRecord.model_fields.keys())
        assert required.issubset(actual), (
            f"EventRecord fields missing: {required - actual}"
        )

    def test_all_payloads_json_serialisable(self):
        """Every event constructor must produce a JSON-serialisable payload."""
        kwarg_list = [
            ev.motion_command(role="r", target_type="joint_angles", target={}),
            ev.gripper_lock(role="r"),
            ev.gripper_unlock(role="r"),
            ev.policy_handoff_start(policy_name="p", cycle_index=0),
            ev.policy_handoff_end(policy_name="p", cycle_index=0),
            ev.cycle_abort(cycle_index=0, reason="r", subsystem="s"),
            ev.cycle_retry(cycle_index=0, attempt=1, max_attempts=2, reason="r", subsystem="s"),
            ev.cycle_start(cycle_index=0),
            ev.cycle_end(cycle_index=0),
        ]
        for kw in kwarg_list:
            try:
                json.dumps(kw["payload"])
            except TypeError as exc:
                pytest.fail(
                    f"Payload for {kw['event_type']} is not JSON-serialisable: {exc}"
                )
