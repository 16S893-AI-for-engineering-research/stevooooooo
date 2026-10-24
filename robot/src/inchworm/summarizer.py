"""Session summariser: renders a per-cycle timeline from a JSON-lines event log.

Usage::

    from inchworm.summarizer import summarize_session, format_timeline

    timeline = summarize_session(Path("robot/sessions/20261024_153000_a3f1/event_log.jsonl"))
    print(format_timeline(timeline))

The summariser reads the event log produced by :mod:`inchworm.telemetry` and
groups events by crawl cycle using ``cycle_start`` / ``cycle_end`` bookends.
It aggregates per-cycle: duration, motion count, gripper transitions, policy
handoffs, retries, and the terminal outcome (completed / aborted).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# Data classes for the timeline
# ---------------------------------------------------------------------------


@dataclass
class CycleSummary:
    """Per-cycle aggregated information derived from the event log."""

    cycle_index: int
    # Wall-clock ISO timestamp of cycle_start (None if no cycle_start seen).
    start_ts_iso: str | None = None
    # Monotonic seconds at cycle_start.
    start_ts_monotonic: float | None = None
    # Wall-clock ISO timestamp of cycle_end or cycle_abort.
    end_ts_iso: str | None = None
    # Monotonic seconds at cycle_end or cycle_abort.
    end_ts_monotonic: float | None = None
    # "completed", "aborted", or "in_progress" if no terminal event seen.
    outcome: str = "in_progress"
    # Abort reason string if outcome == "aborted".
    abort_reason: str | None = None
    # Number of motion_command_sent events in this cycle.
    motion_count: int = 0
    # Gripper transitions: list of {"event_type", "role", "from_state", "to_state"}.
    gripper_transitions: list[dict[str, Any]] = field(default_factory=list)
    # Policy handoffs: list of {"event_type", "direction", "policy_name"}.
    policy_handoffs: list[dict[str, Any]] = field(default_factory=list)
    # Retry events: list of {"attempt", "max_attempts", "reason", "subsystem"}.
    retries: list[dict[str, Any]] = field(default_factory=list)

    @property
    def duration_s(self) -> float | None:
        """Elapsed seconds between cycle_start and cycle_end/abort, or None."""
        if self.start_ts_monotonic is not None and self.end_ts_monotonic is not None:
            return round(self.end_ts_monotonic - self.start_ts_monotonic, 3)
        return None


@dataclass
class SessionTimeline:
    """Timeline for one session, composed of ordered :class:`CycleSummary` objects."""

    # Total number of raw event records parsed.
    total_events: int = 0
    # All cycles seen, ordered by first appearance.
    cycles: list[CycleSummary] = field(default_factory=list)
    # Events that don't belong to any cycle (before first cycle_start or between cycles).
    orphan_event_types: list[str] = field(default_factory=list)

    def get_cycle(self, index: int) -> CycleSummary | None:
        """Return the :class:`CycleSummary` for *index*, or None."""
        for c in self.cycles:
            if c.cycle_index == index:
                return c
        return None


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

# Event types that belong to the motion family.
_MOTION_EVENT = "motion_command_sent"
_GRIPPER_EVENTS = {"gripper_lock", "gripper_unlock"}
_POLICY_EVENTS = {"policy_handoff_start", "policy_handoff_end"}
_RETRY_EVENT = "cycle_retry"
_ABORT_EVENT = "cycle_abort"
_CYCLE_START = "cycle_start"
_CYCLE_END = "cycle_end"


def _get_cycle_index(record: dict[str, Any]) -> int | None:
    """Extract the cycle_index from a record's payload, returning None if absent."""
    payload = record.get("payload", {})
    val = payload.get("cycle_index")
    if val is None:
        return None
    try:
        return int(val)
    except (TypeError, ValueError):
        return None


def summarize_session(log_path: Path) -> SessionTimeline:
    """Parse *log_path* (JSON-lines event log) and return a :class:`SessionTimeline`.

    Lines that cannot be parsed as JSON are silently skipped so that truncated
    logs still yield a partial timeline.
    """
    timeline = SessionTimeline()
    # Map cycle_index -> CycleSummary for fast lookup while streaming.
    index_map: dict[int, CycleSummary] = {}

    raw_lines = log_path.read_text(encoding="utf-8").splitlines()

    for raw in raw_lines:
        raw = raw.strip()
        if not raw:
            continue
        try:
            record: dict[str, Any] = json.loads(raw)
        except json.JSONDecodeError:
            continue

        timeline.total_events += 1
        etype = record.get("event_type", "")
        payload = record.get("payload", {})
        ts_mono = record.get("ts_monotonic")
        ts_iso = record.get("ts_iso", "")

        cycle_idx = _get_cycle_index(record)

        # ------------------------------------------------------------------
        # Route to cycle summary or orphan bucket
        # ------------------------------------------------------------------
        if cycle_idx is None:
            timeline.orphan_event_types.append(etype)
            continue

        if cycle_idx not in index_map:
            cs = CycleSummary(cycle_index=cycle_idx)
            index_map[cycle_idx] = cs
            timeline.cycles.append(cs)
        cs = index_map[cycle_idx]

        if etype == _CYCLE_START:
            cs.start_ts_iso = ts_iso
            cs.start_ts_monotonic = float(ts_mono) if ts_mono is not None else None

        elif etype == _CYCLE_END:
            cs.end_ts_iso = ts_iso
            cs.end_ts_monotonic = float(ts_mono) if ts_mono is not None else None
            if cs.outcome == "in_progress":
                cs.outcome = "completed"

        elif etype == _ABORT_EVENT:
            cs.end_ts_iso = ts_iso
            cs.end_ts_monotonic = float(ts_mono) if ts_mono is not None else None
            cs.outcome = "aborted"
            cs.abort_reason = payload.get("reason")

        elif etype == _MOTION_EVENT:
            cs.motion_count += 1

        elif etype in _GRIPPER_EVENTS:
            cs.gripper_transitions.append(
                {
                    "event_type": etype,
                    "role": payload.get("role", record.get("role", "")),
                    "from_state": payload.get("from_state"),
                    "to_state": payload.get("to_state"),
                }
            )

        elif etype in _POLICY_EVENTS:
            cs.policy_handoffs.append(
                {
                    "event_type": etype,
                    "direction": payload.get("direction"),
                    "policy_name": payload.get("policy_name", ""),
                }
            )

        elif etype == _RETRY_EVENT:
            cs.retries.append(
                {
                    "attempt": payload.get("attempt"),
                    "max_attempts": payload.get("max_attempts"),
                    "reason": payload.get("reason"),
                    "subsystem": payload.get("subsystem"),
                }
            )

    # Sort cycles by cycle_index for deterministic output.
    timeline.cycles.sort(key=lambda c: c.cycle_index)
    return timeline


# ---------------------------------------------------------------------------
# Formatter
# ---------------------------------------------------------------------------


def format_timeline(timeline: SessionTimeline) -> str:
    """Render *timeline* as a human-readable multi-line string.

    Example output::

        Session timeline  (3 cycles, 142 events total)
        ─────────────────────────────────────────────
        Cycle 0  completed   4.321 s  | motion:3 gripper:2 policy:2 retry:0
          gripper  arm_left  unlocked → locked
          gripper  arm_right unlocked → locked
          policy   code_to_policy  insertion_v1
          policy   policy_to_code  insertion_v1  completed
        ─────────────────────────────────────────────
        Cycle 1  aborted    1.012 s  | motion:1 gripper:0 policy:1 retry:1
          retry    attempt 1/3  gripper  joint_stall
          policy   code_to_policy  insertion_v1
          ABORT    joint_stall  (gripper: servo timeout)
        ─────────────────────────────────────────────
        Orphan events: 2
    """
    lines: list[str] = []
    sep = "─" * 53

    cycle_word = "cycle" if len(timeline.cycles) == 1 else "cycles"
    lines.append(
        f"Session timeline  ({len(timeline.cycles)} {cycle_word}, "
        f"{timeline.total_events} events total)"
    )
    lines.append(sep)

    for cs in timeline.cycles:
        dur = f"{cs.duration_s:.3f} s" if cs.duration_s is not None else "  ?.??? s"
        outcome_tag = {
            "completed": "completed",
            "aborted": "aborted  ",
            "in_progress": "in_progress",
        }.get(cs.outcome, cs.outcome)
        header = (
            f"Cycle {cs.cycle_index}  {outcome_tag}  {dur}"
            f"  | motion:{cs.motion_count}"
            f" gripper:{len(cs.gripper_transitions)}"
            f" policy:{len(cs.policy_handoffs)}"
            f" retry:{len(cs.retries)}"
        )
        lines.append(header)

        for g in cs.gripper_transitions:
            lines.append(
                f"  gripper  {g['role']:<12s}  "
                f"{g['from_state']} → {g['to_state']}"
            )

        for p in cs.policy_handoffs:
            lines.append(
                f"  policy   {p['direction']:<18s}  {p['policy_name']}"
                + (f"  {p.get('reason','')}" if p.get("reason") else "")
            )

        for r in cs.retries:
            lines.append(
                f"  retry    attempt {r['attempt']}/{r['max_attempts']}"
                f"  {r['subsystem']}  {r['reason']}"
            )

        if cs.outcome == "aborted" and cs.abort_reason:
            lines.append(f"  ABORT    {cs.abort_reason}")

        lines.append(sep)

    if timeline.orphan_event_types:
        lines.append(f"Orphan events: {len(timeline.orphan_event_types)}")

    return "\n".join(lines)
