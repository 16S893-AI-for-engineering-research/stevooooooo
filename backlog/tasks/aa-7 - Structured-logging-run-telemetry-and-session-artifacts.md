---
id: AA-7
title: 'Structured logging, run telemetry and session artifacts'
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:08'
updated_date: '2026-10-24 14:38'
labels: []
milestone: m-0
dependencies:
  - AA-4
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 7000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The system is meant to run unattended through repeated crawl, insert, lock, fold, level and print cycles. When a cycle fails at 2 am there is no operator watching, so the only way to diagnose it is the record left behind. Debugging servo-level faults, policy aborts and leveling outliers after the fact requires timestamped, machine-readable traces from the very first driver, which is why this precedes hardware bring-up.

Keep artifacts local by default: camera frames and session traces are not to be committed to the repository.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A logging module provides structured (JSON-lines) event records with monotonic timestamps, logical device role, subsystem and severity, plus human-readable console output
- [x] #2 Each run writes a session directory containing the event log, the resolved hardware config snapshot and any captured artifacts, under a gitignored path
- [x] #3 Motion commands, gripper lock and unlock transitions, policy handoffs, aborts and retries each emit a distinct typed event with the fields needed to reconstruct the cycle
- [x] #4 A summarizer renders one session into a readable per-cycle timeline (durations, outcomes, failures) from the event log alone
- [x] #5 Session artifact paths are gitignored and no frames or traces are committed by this task
- [x] #6 Tests cover event schema stability and the summarizer output for a synthetic session
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
## Plan for AA-7 ACs #1, #2, #5 (worker slice)

### AC #1 – JSON-lines event logger
- Create robot/src/inchworm/logging.py (module name: inchworm.logging avoided, use telemetry.py)
  - EventRecord: Pydantic model with: ts_monotonic (float), ts_iso (str), role (str), subsystem (str), severity (Literal[DEBUG,INFO,WARNING,ERROR,CRITICAL]), event_type (str), payload (dict)
  - SessionLogger class: writes JSON-lines to session dir, also emits human-readable console output via stdlib logging
  - Monotonic timestamp via time.monotonic_ns() (nanoseconds) normalised to seconds; wall-clock ISO via datetime.utcnow()

### AC #2 – Per-run session directory
- Create robot/src/inchworm/session.py
  - new_session(config: HardwareConfig) -> SessionContext: creates timestamped dir under robot/sessions/<YYYYMMDD_HHMMSS_<random4hex>>/
  - Writes resolved config snapshot as config_snapshot.yaml on open
  - Returns context object with .log_path, .artifact_dir, .logger (SessionLogger)
  - SessionContext is a context manager (close flushes/closes the log file)

### AC #5 – Gitignore verification
- robot/sessions/ is already in .gitignore (verified). Test asserts the pattern is present.

### Tests (robot/tests/test_telemetry.py)
- test_event_record_schema_fields: instantiate EventRecord, check all fields present and types correct
- test_session_logger_writes_jsonlines: write 3 events, read back, parse JSON, assert fields
- test_session_dir_created_under_sessions: new_session creates dir under robot/sessions/
- test_config_snapshot_written: config_snapshot.yaml exists and round-trips to HardwareConfig
- test_gitignore_covers_sessions: .gitignore contains 'robot/sessions/'
- All tests use tmp_path or a temp session root, no hardware touched
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
## Progress – worker slice (ACs #1, #2, #5)

Implemented:
- **robot/src/inchworm/telemetry.py**: EventRecord (Pydantic, frozen, JSON-serialisable) with fields ts_monotonic, ts_iso, role, subsystem, severity (DEBUG/INFO/WARNING/ERROR/CRITICAL), event_type, payload. SessionLogger writes JSON-lines (line-buffered) + emits to stdlib logging logger 'inchworm.telemetry'. Convenience methods: debug/info/warning/error/critical.
- **robot/src/inchworm/session.py**: new_session(config, *, session_root) creates robot/sessions/<YYYYMMDD_HHMMSS_<4hex>>/ with event_log.jsonl, config_snapshot.yaml (round-trippable via parse_config), and artifacts/ sub-dir. SessionContext is a context manager that closes the logger on exit. Default session root is robot/sessions/ (already gitignored).
- **robot/tests/test_telemetry.py**: 26 tests covering EventRecord schema stability, SessionLogger JSON-lines output, console logging, monotonic timestamps, severity convenience methods, session dir creation/naming/contents, config snapshot round-trip, device identity preservation, gitignore coverage.

Test run: 70 passed, 0 failed (uv run pytest)

ACs not yet checked (will be done in finalisation with pasted output as evidence):
- AC #1, #2, #5 implementation complete; evidence will be pasted at finalisation.
- AC #3, #4, #6 are out of scope for this worker slice.

## Progress – ACs #3, #4, #6 + finalization (all ACs)

Added robot/src/inchworm/events.py – typed event constructor module (AC #3):
- motion_command(): event_type='motion_command_sent', payload fields: role/target_type/target/speed_fraction/dry_run
- gripper_lock(): event_type='gripper_lock', payload: role/from_state/to_state/servo_id
- gripper_unlock(): event_type='gripper_unlock', payload: role/from_state/to_state/servo_id
- policy_handoff_start(): event_type='policy_handoff_start', payload: direction/policy_name/cycle_index/reason
- policy_handoff_end(): event_type='policy_handoff_end', payload: direction/policy_name/cycle_index/reason
- cycle_abort(): event_type='cycle_abort', payload: cycle_index/reason/detail/subsystem (severity ERROR)
- cycle_retry(): event_type='cycle_retry', payload: cycle_index/attempt/max_attempts/reason/subsystem (severity WARNING)
- cycle_start()/cycle_end(): bookend events, payload: cycle_index/phase
- All 9 event_type strings are distinct (verified by test_all_event_types_are_distinct)
- All payload models are frozen Pydantic; all payloads are JSON-serialisable

Added robot/src/inchworm/summarizer.py – per-cycle timeline summarizer (AC #4):
- summarize_session(log_path) -> SessionTimeline: parses JSON-lines, groups by cycle_index
- CycleSummary: outcome/duration_s/motion_count/gripper_transitions/policy_handoffs/retries/abort_reason
- format_timeline(timeline) -> str: renders header + per-cycle lines with durations, outcomes, event counts
- Handles truncated logs and orphan events (no cycle_index) gracefully

Added robot/tests/test_events_summarizer.py – tests for ACs #3, #4, #6 (85 tests):
- TestTypedEventMotion (11 tests): schema, field values, round-trip via SessionLogger
- TestTypedEventGripper (10 tests): lock/unlock event types, to_state/from_state, servo_id
- TestTypedEventPolicy (8 tests): handoff directions, payload fields, cycle_index
- TestTypedEventAbort (7 tests): event_type, severity ERROR, required fields
- TestTypedEventRetry (5 tests): event_type, severity WARNING, attempt/max_attempts
- TestTypedEventCycleBookends (6 tests): cycle_start/end event types, all 9 event_types distinct
- TestSummarizerBasic (11 tests): total_events, cycles, outcome, motion_count, gripper, policy, duration
- TestSummarizerAbort (4 tests): aborted outcome, abort_reason, retries, cycle isolation
- TestSummarizerFormatter (11 tests): header, cycle headers, labels, motion/gripper/policy counts, duration
- TestPayloadSchemaStability (11 tests): every payload model has exact expected field set

Test run (full suite): 155 passed in 0.50s
- tests/test_backends.py: 17 passed
- tests/test_config.py: 23 passed
- tests/test_constants.py: 4 passed
- tests/test_events_summarizer.py: 85 passed
- tests/test_telemetry.py: 26 passed

ACs #1, #2, #5: fully evidenced by test_telemetry.py (26 tests, all pass)
ACs #3, #4, #6: fully evidenced by test_events_summarizer.py (85 tests, all pass)
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented structured logging, session management, typed events, and a session summarizer for AA-7. Files added: robot/src/inchworm/telemetry.py (EventRecord schema + SessionLogger), robot/src/inchworm/session.py (new_session, SessionContext, config snapshot), robot/src/inchworm/events.py (9 typed event constructors with frozen Pydantic payload models), robot/src/inchworm/summarizer.py (summarize_session + format_timeline). Tests: 155 passed (0 failed) across 5 test files — test_telemetry.py (26 tests, ACs #1/#2/#5) and test_events_summarizer.py (85 tests, ACs #3/#4/#6). Evidence: 'uv run pytest' output pasted in implementation notes. No hardware touched.
<!-- SECTION:FINAL_SUMMARY:END -->
