#!/usr/bin/env python3
"""Generate a privacy-preserving aggregate chart from Pi session records.

The output contains only daily totals for sessions whose saved cwd is this repo:
user-message entries, tool-result entries, and recorded usage cost. It does not
copy prompts, tool output, model IDs, session IDs, or file paths into the site.
"""
from __future__ import annotations

import json
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape

REPO = Path(__file__).resolve().parents[1]
SESSIONS = Path.home() / ".pi" / "agent" / "sessions" / ("--" + str(REPO).strip("/").replace("/", "-") + "--")
OUTPUT = REPO / "assets" / "session-activity.svg"


def tick_max(value: float, steps: int = 4) -> float:
    if value <= 0:
        return 1
    magnitude = 10 ** math.floor(math.log10(value))
    step = magnitude
    for factor in (1, 2, 5, 10):
        candidate = factor * magnitude
        if value / candidate <= steps:
            step = candidate
            break
    return math.ceil(value / step) * step


HANDOFF = Path.home() / ".pi" / "agent" / "human-handoff" / "sessions"
KINDS = ("orchestrator", "subagent", "guide")
COLORS = {"orchestrator": "#ff7a45", "subagent": "#3ec1b4", "guide": "#ffc857", "messages": "#ff5c8a", "tools": "#3ec1b4"}
LABELS = {"orchestrator": "Main (orchestrator) sessions", "subagent": "Sub-agent workers", "guide": "Walkthrough guide"}
BG, PLOT, GRID, INK, DIM = "#120f24", "#17132e", "#2a2450", "#f3e9d2", "#b9b0c9"


def session_files():
    """Yield (kind, path) for every persisted session that belongs to this repo."""
    for path in sorted(SESSIONS.glob("*.jsonl")):
        yield "orchestrator", path
    for path in sorted(SESSIONS.glob("*/*/run-*/session.jsonl")):
        yield "subagent", path
    for path in sorted(HANDOFF.glob("*.jsonl")):
        yield "guide", path


def collect() -> dict[str, dict[str, float]]:
    totals: dict[str, dict[str, float]] = defaultdict(lambda: {"sessions": 0, "messages": 0, "tools": 0, "cost": 0.0, **{k: 0.0 for k in KINDS}})
    for kind, path in session_files():
        try:
            records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        except (OSError, json.JSONDecodeError):
            continue
        if not records:
            continue
        cwd = records[0].get("cwd", "")
        # Sub-agents run in isolated worktrees; their transcripts are stored under this repo's session directory.
        if kind != "subagent" and cwd != str(REPO):
            continue
        day = records[0].get("timestamp", "")[:10]
        if not day:
            continue
        total = totals[day]
        total["sessions"] += 1
        for record in records:
            message = record.get("message", {}) if record.get("type") == "message" else {}
            role = message.get("role")
            if role == "user" and kind == "orchestrator":
                total["messages"] += 1
            elif role == "toolResult":
                total["tools"] += 1
            cost = (message.get("usage") or {}).get("cost", {})
            cost = cost.get("total", 0) if isinstance(cost, dict) else 0
            if isinstance(cost, (int, float)):
                total["cost"] += cost
                total[kind] += cost
    return dict(sorted(totals.items()))


def text(x: float, y: float, value: str, size: int = 14, fill: str = "#b9b0c9", anchor: str = "start") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" font-family="Space Grotesk, Segoe UI, sans-serif" font-size="{size}" text-anchor="{anchor}">{escape(value)}</text>'


def chart(totals: dict[str, dict[str, float]]) -> str:
    width, height, left, right = 1100, 860, 82, 34
    plot_w = width - left - right
    days = list(totals)
    activity_max = tick_max(max((max(row["messages"], row["tools"]) for row in totals.values()), default=1))
    cost_max = tick_max(max((row["cost"] for row in totals.values()), default=1))
    top_y, top_h = 112, 300
    bottom_y, bottom_h = 548, 200
    group_w = plot_w / max(len(days), 1)
    grand = sum(row["cost"] for row in totals.values())
    today = datetime.now().strftime("%-m/%-d/%y")
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        f'<title id="title">pi agent session activity through {today}</title>',
        '<desc id="desc">Daily totals from all persisted pi sessions for this repository, including sub-agent workers and the walkthrough guide. The first chart compares user messages and agent tool calls; the second shows recorded model cost in US dollars, split by session type.</desc>',
        f'<rect width="100%" height="100%" rx="18" fill="{BG}"/>',
        text(width / 2, 44, "Agent activity and cost by day", 28, INK, "middle"),
        text(width / 2, 72, f"{sum(row['sessions'] for row in totals.values()):.0f} sessions through {today} \u00b7 total recorded cost ${grand:,.0f}", 15, DIM, "middle"),
    ]
    for y, h, maximum, label, money in [(top_y, top_h, activity_max, "Count", False), (bottom_y, bottom_h, cost_max, "Model cost (USD)", True)]:
        parts.append(f'<rect x="{left}" y="{y}" width="{plot_w}" height="{h}" rx="8" fill="{PLOT}" stroke="{GRID}"/>')
        for i in range(5):
            value = maximum * i / 4
            py = y + h - h * i / 4
            parts.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left + plot_w}" y2="{py:.1f}" stroke="{GRID}" stroke-width="1" stroke-dasharray="3 5"/>')
            parts.append(text(left - 10, py + 5, f"${value:.0f}" if money else f"{value:.0f}", 13, DIM, "end"))
        parts.append(f'<text x="25" y="{y + h / 2:.1f}" transform="rotate(-90 25 {y + h / 2:.1f})" fill="{DIM}" font-family="Space Grotesk, Segoe UI, sans-serif" font-size="15" text-anchor="middle">{label}</text>')
    for x, key, label in [(left + 18, "messages", "Owner messages"), (left + 190, "tools", "Agent tool calls")]:
        parts.extend([f'<rect x="{x}" y="{top_y + 16}" width="16" height="16" rx="4" fill="{COLORS[key]}"/>', text(x + 24, top_y + 29, label, 14, INK)])
    lx = left + 18
    for kind in KINDS:
        parts.extend([f'<rect x="{lx}" y="{bottom_y + 16}" width="16" height="16" rx="4" fill="{COLORS[kind]}"/>', text(lx + 24, bottom_y + 29, LABELS[kind], 14, INK)])
        lx += 250
    for index, day in enumerate(days):
        row = totals[day]
        center = left + group_w * (index + .5)
        bar_w = min(26, group_w * .3)
        for offset, value, key in [(-bar_w * .6, row["messages"], "messages"), (bar_w * .6, row["tools"], "tools")]:
            h = top_h * value / activity_max
            x, y = center + offset - bar_w / 2, top_y + top_h - h
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" rx="2" fill="{COLORS[key]}"/>')
            if value >= 50:
                parts.append(text(x + bar_w / 2, y - 6, f"{value:.0f}", 11, INK, "middle"))
        label = datetime.strptime(day, "%Y-%m-%d").strftime("%-m/%-d")
        parts.append(text(center, top_y + top_h + 22, label, 12, DIM, "middle"))
        w = min(40, group_w * .6)
        y = bottom_y + bottom_h
        for kind in KINDS:
            seg = bottom_h * row[kind] / cost_max
            if seg > 0:
                y -= seg
                parts.append(f'<rect x="{center - w / 2:.1f}" y="{y:.1f}" width="{w:.1f}" height="{seg:.1f}" fill="{COLORS[kind]}"/>')
        if row["cost"] >= 1:
            parts.append(text(center, y - 7, f"${row['cost']:.0f}", 12, INK, "middle"))
        parts.append(text(center, bottom_y + bottom_h + 22, label, 12, DIM, "middle"))
    parts.extend([
        text(width / 2, 808, "Sessions are dated by the day they started. Sub-agent cost is dated by its own start day. $0 days had little or no paid model usage.", 12, DIM, "middle"),
        text(width / 2, 830, f"Generated {today} from local pi session metadata only; prompt and tool contents are not included.", 12, DIM, "middle"),
        "</svg>",
    ])
    return "\n".join(parts)


if __name__ == "__main__":
    totals = collect()
    if not totals:
        raise SystemExit(f"No persisted Pi sessions found for {REPO}")
    OUTPUT.write_text(chart(totals), encoding="utf-8")
    print(json.dumps({"sessions": sum(row["sessions"] for row in totals.values()), "days": totals, "output": str(OUTPUT)}, indent=2))
