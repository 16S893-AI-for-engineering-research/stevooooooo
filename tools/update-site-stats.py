#!/usr/bin/env python3
"""Refresh the numeric facts and harness lists on the static site.

Every value lives between markers in the HTML, for example
``<!-- stats:tests -->1,863<!-- /stats -->``; only the text between the
markers is rewritten, so rerunning is idempotent for unchanged inputs.

Sources: ``backlog task list --json`` (tasks, acceptance criteria, milestones),
``uv run pytest --co`` in robot/, robot/lerobot_env/ and sim/ (test counts per group), the Pi session records via
generate-session-activity.py (cost, sessions; also rewrites the SVG chart), and
the .pi/skills, .pi/prompts and .pi/extensions directories (harness lists).
"""
from __future__ import annotations

import html
import importlib.util
import json
import re
import subprocess
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PAGES = ("index.html", "project.html", "devlog.html")
REGION = re.compile(r"(<!-- stats:([\w-]+) -->)(.*?)(<!-- /stats -->)", re.DOTALL)
MILESTONES = ("m-0", "m-1", "m-2", "m-3", "m-4", "m-5", "m-6")

# One-line summaries shown on the Project page; entries without one fall back to their own description.
SUMMARIES = {
    "clarify-before-build": "Before building anything visual or physical, ask clarifying questions, restate the plan and wait for my confirmation.",
    "human-handoff": "Only for steps that truly need my hands, identity or judgment: write a short checked procedure that I accept, then verify the result.",
    "handoff": "Wrap up a session: task notes, doc-3 update, tests, commit and push, then a one-line \u201cresume with\u201d for the next session.",
    "task": "Start a fresh session on one Backlog task: read doc-3 START HERE and that task only, restate the open criteria, wait for my go-ahead.",
    "human-handoff-extension": "Adds the human_request, human_queue, human_checkpoint and human_verify tools and the /human command; accepted procedures run one at a time in a separate walkthrough tab.",
}


def fmt_pct(part: float, whole: float) -> str:
    return f"{100 * part / whole:.1f}".rstrip("0").rstrip(".") if whole else "0"


def frontmatter(path: Path) -> dict[str, str]:
    match = re.match(r"---\n(.*?)\n---", path.read_text(encoding="utf-8"), re.DOTALL)
    fields = {}
    for line in (match.group(1).splitlines() if match else []):
        key, _, value = line.partition(":")
        fields[key.strip()] = value.strip().strip('"')
    return fields


def first_sentence(text: str) -> str:
    return re.split(r"(?<=\.)\s", text, maxsplit=1)[0]


def task_stats() -> dict[str, str]:
    data = json.loads(subprocess.run(["backlog", "task", "list", "--json"], cwd=REPO, check=True, capture_output=True, text=True).stdout)
    tasks = data["tasks"]
    stats: dict[str, str] = {}

    def summarize(subset):
        return (len(subset), sum(t["status"] == "Done" for t in subset), sum(t["status"] == "In Progress" for t in subset),
                sum(t["status"] == "To Do" for t in subset), sum(t["acceptanceCriteriaCompleted"] for t in subset),
                sum(t["acceptanceCriteriaCount"] for t in subset))

    total, done, doing, todo, ac_done, ac_total = summarize(tasks)
    stats.update({"tasks": str(total), "tasks-done": str(done), "ac-total": f"{ac_total:,}", "ac-done": f"{ac_done:,}"})
    m_total, m_done, m_doing, m_todo, m_ac_done, m_ac_total = summarize([t for t in tasks if t["milestone"]])
    stats["ms-pct"] = f"{round(100 * m_ac_done / m_ac_total)}%"
    stats["ms-ac-total"] = str(m_ac_total)
    stats["ms-done"], stats["ms-doing"], stats["ms-todo"] = str(m_done), str(m_doing), str(m_todo)
    stats["overall-bar"] = (
        f'<div class="progress lg" role="img" aria-label="{m_done} of {m_total} milestone tasks done, {m_doing} in progress, {m_todo} to do">'
        f'<span class="done" style="width:{fmt_pct(m_done, m_total)}%"></span>'
        f'<span class="doing" style="width:{fmt_pct(m_doing, m_total)}%"></span>'
        f'<span class="todo" style="width:{fmt_pct(m_todo, m_total)}%"></span></div>'
    )
    for milestone in MILESTONES:
        n, d, p, _, a, c = summarize([t for t in tasks if t["milestone"] == milestone])
        spans = "".join(f'<span class="{cls}" style="width:{fmt_pct(v, n)}%"></span>' for cls, v in (("done", d), ("doing", p)) if v)
        stats[f"bar-{milestone}"] = f'<div class="progress">{spans}</div>'
        stats[f"count-{milestone}"] = f"<b>{d}/{n}</b> tasks<br>{a}/{c} criteria"
    return stats


# Test groups shown on the Project page ("How it's tested"). Every test file of every suite must be listed
# here, so a new file fails the refresh instead of silently missing from the chart.
TEST_SUITES = {  # suite -> (directory, pytest command)
    "robot": ("robot", ["uv", "run", "pytest", "-q", "--co", "-p", "no:warnings"]),
    "lerobot": ("robot/lerobot_env", ["uv", "run", "pytest", "-q", "--co", "-p", "no:warnings"]),
    "sim": ("sim", ["uv", "run", "--extra", "dev", "pytest", "-q", "--co", "-p", "no:warnings"]),
}
TEST_GROUPS = {
    "safety": ("pink", "gripper_lock motion_guard cell_collision dexarm_guard estop torque homing laser camera_arbiter "
               "ast_guard constants start_pose_check start_check"),
    "drivers": ("orange", "servo_bus dexarm camera_capture camera_bridge discovery attach_usb vhci backends config camera_rate_diag"),
    "geometry": ("mustard", "frames kinematics approach fold cycle_replay inchworm_plan system_model calibration jog_live"),
    "crawl": ("teal", "supervisor cycle recovery seating handoff handoff_return resume_cycle hold_tube fault_inject_hw two_policy"),
    "learning": ("violet", "lerobot_adapter policy_runner teleop_capture teleop_live leader_mapping perturb training_poses "
                 "arm_transfer lerobot:*"),
    "printing": ("blue", "gcode_intake print_frame laser_line leveling scanner_calibration calibrate_scanner_live"),
    "tooling": ("grey", "script_imports events_summarizer telemetry home_servos direction_check powered_move_check "
                "restore_drive_config record_joint_ranges gripper_empty_moves gripper_lock_probe"),
    "sim": ("sky", "sim:*"),
}


def test_stats() -> dict[str, str]:
    per_file: dict[tuple[str, str], int] = {}
    for suite, (folder, cmd) in TEST_SUITES.items():
        out = subprocess.run(cmd, cwd=REPO / folder, check=True, capture_output=True, text=True).stdout
        for m in re.finditer(r"^(?:\S+/)?tests/test_(\w+)\.py: (\d+)$", out, re.MULTILINE):
            per_file[(suite, m.group(1))] = int(m.group(2))
        if not any(key[0] == suite for key in per_file):
            raise SystemExit(f"pytest collection reported no tests in {folder}")
    groups = {name: 0 for name in TEST_GROUPS}
    for (suite, name), n in per_file.items():
        owners = [g for g, (_, files) in TEST_GROUPS.items()
                  if f"{suite}:*" in files.split() or (suite == "robot" and name in files.split())]
        if len(owners) != 1:
            raise SystemExit(f"test file {suite}:test_{name}.py needs exactly one TEST_GROUPS entry (found {owners})")
        groups[owners[0]] += n
    robot = sum(n for (suite, _), n in per_file.items() if suite == "robot")
    everything = sum(per_file.values())
    stats = {"tests": f"{robot:,}", "tests-floor": f"{robot // 100 * 100:,}", "tests-all": f"{everything:,}",
             "tests-files": str(len(per_file))}
    for suite in TEST_SUITES:
        stats[f"tests-suite-{suite}"] = f"{sum(n for (s, _), n in per_file.items() if s == suite):,}"
    for name, n in groups.items():
        stats[f"tests-group-{name}"] = f"{n:,}"
    stats["tests-bar"] = "".join(
        f'<span class="tg-{name}" style="width:{fmt_pct(n, everything)}%" title="{n:,} tests"></span>' for name, n in groups.items())
    return stats


def cost_stats() -> dict[str, str]:
    spec = importlib.util.spec_from_file_location("session_activity", REPO / "tools" / "generate-session-activity.py")
    activity = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(activity)
    totals = activity.collect()
    if not totals:
        raise SystemExit("No persisted Pi sessions found")
    activity.OUTPUT.write_text(activity.chart(totals), encoding="utf-8")
    grand = sum(row["cost"] for row in totals.values())
    parts = {kind: sum(row[kind] for row in totals.values()) for kind in activity.KINDS}
    first, last = (datetime.strptime(day, "%Y-%m-%d") for day in (min(totals), max(totals)))
    widths = {kind: round(100 * value / grand) for kind, value in parts.items()}
    colors = {"orchestrator": "orange", "subagent": "teal", "guide": "mustard"}
    return {
        "cost": f"${grand:,.0f}",
        "sessions": str(sum(row["sessions"] for row in totals.values())),
        "cost-range": f"{first:%-m/%-d} to {last:%-m/%-d/%y}",
        "cost-main": f"${parts['orchestrator']:,.0f}",
        "cost-sub": f"${parts['subagent']:,.0f}",
        "cost-guide": f"${parts['guide']:,.0f}",
        "cost-bar": "".join(f'<span style="width:{widths[k]}%; background:var(--{colors[k]})"></span>' for k in activity.KINDS),
    }


def harness_stats() -> dict[str, str]:
    def item(name: str, label: str, fallback: str) -> str:
        return f"<li><strong>{html.escape(label)}</strong>: {html.escape(SUMMARIES.get(name, fallback))}</li>"

    pi = REPO / ".pi"
    skills = [item(p.parent.name, p.parent.name, first_sentence(frontmatter(p).get("description", "")))
              for p in sorted(pi.glob("skills/*/SKILL.md"))]
    prompts = [item(p.stem, f"/{p.stem}", frontmatter(p).get("description", "")) for p in sorted(pi.glob("prompts/*.md"))]
    extensions = [item(f"{p.name}-extension", f"{p.name} (project)", "Project extension.") for p in sorted(pi.glob("extensions/*")) if p.is_dir()]
    return {"harness-skills": "".join(skills), "harness-prompts": "".join(prompts), "harness-extensions": "".join(extensions)}


def main() -> None:
    stats = {**task_stats(), **test_stats(), **cost_stats(), **harness_stats(), "updated": datetime.now().strftime("%b %-d, %Y")}
    unknown = set()
    for page in PAGES:
        path = REPO / page
        source = path.read_text(encoding="utf-8")

        def replace(match: re.Match) -> str:
            key = match.group(2)
            if key not in stats:
                unknown.add(key)
                return match.group(0)
            return match.group(1) + stats[key] + match.group(4)

        updated = REGION.sub(replace, source)
        if updated != source:
            path.write_text(updated, encoding="utf-8")
            print(f"updated {page}")
    if unknown:
        raise SystemExit(f"Unknown stats markers: {sorted(unknown)}")
    print(json.dumps({k: v for k, v in stats.items() if "<" not in v}, indent=2))


if __name__ == "__main__":
    main()
