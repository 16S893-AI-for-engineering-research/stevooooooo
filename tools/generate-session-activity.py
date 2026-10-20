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


def collect() -> dict[str, dict[str, float]]:
    totals: dict[str, dict[str, float]] = defaultdict(lambda: {"sessions": 0, "messages": 0, "tools": 0, "cost": 0.0})
    for path in sorted(SESSIONS.glob("*.jsonl")):
        try:
            records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        except (OSError, json.JSONDecodeError):
            continue
        if not records or records[0].get("cwd") != str(REPO):
            continue
        day = records[0].get("timestamp", "")[:10]
        if not day:
            continue
        total = totals[day]
        total["sessions"] += 1
        for record in records:
            message = record.get("message", {}) if record.get("type") == "message" else {}
            role = message.get("role")
            if role == "user":
                total["messages"] += 1
            elif role == "toolResult":
                total["tools"] += 1
            cost = (message.get("usage") or {}).get("cost", {}).get("total", 0)
            if isinstance(cost, (int, float)):
                total["cost"] += cost
    return dict(sorted(totals.items()))


def text(x: float, y: float, value: str, size: int = 14, fill: str = "#b8b8d4", anchor: str = "start") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" font-family="Space Grotesk, Segoe UI, sans-serif" font-size="{size}" text-anchor="{anchor}">{escape(value)}</text>'


def chart(totals: dict[str, dict[str, float]]) -> str:
    width, height, left, right = 1100, 860, 82, 34
    plot_w = width - left - right
    days = list(totals)
    activity_max = tick_max(max((max(row["messages"], row["tools"]) for row in totals.values()), default=1))
    cost_max = tick_max(max((row["cost"] for row in totals.values()), default=1))
    top_y, top_h = 92, 330
    bottom_y, bottom_h = 548, 190
    group_w = plot_w / max(len(days), 1)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">', '<title id="title">pi agent session activity through 2026-10-20</title>', '<desc id="desc">Daily totals from all persisted pi sessions associated with this repository. The first chart compares user messages and tool calls; the second chart shows recorded model cost in United States dollars.</desc>', '<rect width="100%" height="100%" fill="#070712"/>', text(width / 2, 42, "pi Agent Session Activity", 30, "#e8e8f5", "middle"), text(width / 2, 68, f"All {sum(row['sessions'] for row in totals.values())} persisted repository sessions through 10/20/26", 15, "#a9a9c7", "middle")]
    for y, h, maximum, label, money in [(top_y, top_h, activity_max, "Count", False), (bottom_y, bottom_h, cost_max, "Recorded model cost (USD)", True)]:
        parts.append(f'<rect x="{left}" y="{y}" width="{plot_w}" height="{h}" fill="#080817" stroke="#503d9c"/>')
        for i in range(5):
            value = maximum * i / 4
            py = y + h - h * i / 4
            parts.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left + plot_w}" y2="{py:.1f}" stroke="#211a45" stroke-width="1"/>')
            label_value = f"${value:.0f}" if money else f"{value:.0f}"
            parts.append(text(left - 10, py + 5, label_value, 13, "#b8b8d4", "end"))
        parts.append(f'<text x="25" y="{y + h / 2:.1f}" transform="rotate(-90 25 {y + h / 2:.1f})" fill="#d9d9eb" font-family="Space Grotesk, Segoe UI, sans-serif" font-size="15" text-anchor="middle">{label}</text>')
    legend_y = top_y + 20
    for x, color, label in [(left + 18, "#4cc9f0", "User messages"), (left + 205, "#7c5cff", "Tool calls")]:
        parts.extend([f'<rect x="{x}" y="{legend_y}" width="17" height="17" fill="{color}"/>', text(x + 25, legend_y + 14, label, 15, "#e8e8f5")])
    for index, day in enumerate(days):
        row = totals[day]
        center = left + group_w * (index + .5)
        bar_w = min(32, group_w * .22)
        for offset, value, color in [(-bar_w * .65, row["messages"], "#4cc9f0"), (bar_w * .65, row["tools"], "#7c5cff")]:
            h = top_h * value / activity_max
            x, y = center + offset - bar_w / 2, top_y + top_h - h
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" fill="{color}"/>')
            if value:
                parts.append(text(x + bar_w / 2, y - 7, f"{value:.0f}", 12, "#e8e8f5", "middle"))
        label = datetime.strptime(day, "%Y-%m-%d").strftime("%-m/%-d")
        parts.extend([text(center, top_y + top_h + 23, label, 13, "#d9d9eb", "middle"), text(center, top_y + top_h + 42, f"{row['sessions']:.0f} session{'s' if row['sessions'] != 1 else ''}", 11, "#a9a9c7", "middle")])
        cost_h = bottom_h * row["cost"] / cost_max
        x, y = center - min(52, group_w * .48) / 2, bottom_y + bottom_h - cost_h
        w = min(52, group_w * .48)
        parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{cost_h:.1f}" fill="#f0a84c"/>')
        parts.append(text(center, y - 7, f"${row['cost']:.2f}", 12, "#e8e8f5", "middle"))
        parts.append(text(center, bottom_y + bottom_h + 23, label, 13, "#d9d9eb", "middle"))
    parts.extend([text(width / 2, 806, "Tool calls are completed tool-result entries. Cost is the sum of recorded model usage; $0.00 may reflect zero-priced or unavailable usage data.", 12, "#a9a9c7", "middle"), text(width / 2, 830, "Generated from local Pi session metadata on 10/20/26; prompt and tool contents are not included.", 12, "#a9a9c7", "middle"), '</svg>'])
    return "\n".join(parts)


if __name__ == "__main__":
    totals = collect()
    if not totals:
        raise SystemExit(f"No persisted Pi sessions found for {REPO}")
    OUTPUT.write_text(chart(totals), encoding="utf-8")
    print(json.dumps({"sessions": sum(row["sessions"] for row in totals.values()), "days": totals, "output": str(OUTPUT)}, indent=2))
