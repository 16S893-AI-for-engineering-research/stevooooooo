
<!-- BACKLOG.MD GUIDELINES START -->
<!-- backlog.md-instructions-version: 1.53.0 -->
<CRITICAL_INSTRUCTION>

## Backlog.md Workflow

This project uses Backlog.md for task and project management.

**At the beginning of each conversation in this project, run `backlog instructions overview` before answering or taking action. Re-read it only if you have not read it yet in the current conversation.**

Use the overview to decide whether to search, read, create, or update Backlog tasks.

Before task lifecycle actions, read the matching detailed guide:
- `backlog instructions task-creation` before creating or splitting tasks
- `backlog instructions task-execution` before planning, changing status or assignee, adding a plan or implementation notes, or implementing task work
- `backlog instructions task-finalization` before checking acceptance criteria, writing final summaries, or moving tasks to terminal statuses

Use `backlog <command> --help` before running unfamiliar commands. Help shows options, fields, and examples.

Do not edit Backlog task, draft, document, decision, or milestone markdown files directly. Use the `backlog` CLI so metadata, relationships, and history stay consistent.

</CRITICAL_INSTRUCTION>
<!-- BACKLOG.MD GUIDELINES END -->

<CRITICAL_INSTRUCTION>

## Robot project: read before implementing any task

This repository contains the inchworm print-crawl robot (two SO-101 arms + a Rotrics DexArm on a fixed chassis, moving a printed tube). Before implementing, planning, or finalizing any robot task, read these three specs in order:

- `backlog/docs/specs/doc-3 - Orchestrator-State-and-Session-Handoff.md` — **read this first.** Live state: what is in flight, what is blocked, what is uncertain, and the next action. If its "Last verified" date looks stale, trust `backlog task list --plain` and `git log` over its prose.
- `backlog/docs/specs/doc-1 - Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md` — the single source of truth for hardware, geometry, and system constraints. Take hardware facts from here; never assume them. If you measure a contradicting value, correct doc-1 via `backlog doc update doc-1` in the same task.
- `backlog/docs/specs/doc-2 - Agent-Working-Agreement-—-Requirements-Stability-and-Change-Control.md` — change-control rules.

Non-negotiable without explicit owner approval:

1. **Do not edit acceptance criteria or task descriptions to match what you built.** If a criterion looks wrong or unachievable, stop and ask; record the question as a task comment.
2. **Do not check an acceptance criterion without evidence** (test output, measurement, or a logged run). Code inspection is not evidence.
3. **Do not weaken a tolerance, limit, or safety check, and do not delete or skip a test, to make something pass.**
4. **Safety invariants always hold:** one camera streaming at a time (shared USB 2.0); never both grippers unlocked while the tube is attached; the gripper servo is commanded only by the AA-17 state machine and never by the learned policy; the laser is off on every exit path; joint limits and collision checks are enforced inside the motion layer.
5. **Dry-run first.** Use dry-run mode before any first powered motion on a new path, and ask the owner to supervise the first live execution.
6. **Keep doc-3 current.** When you finish a task, clear a blocker, or make a decision, update `doc-3` via `backlog doc update doc-3` in the same session. A stale doc-3 invites confident action on wrong information.
7. **Commit regularly, without asking (owner standing instruction, 2026-10-24).** The orchestrator commits to the local branch after each verified unit of work (a task finalized or reviewed, a doc correction, a fix). Before every commit: tests pass (`cd robot && uv run pytest`), and the staged files contain no `.parley.env`, secrets, `.venv`, caches, session artifacts, frames, datasets or checkpoints. Sub-agents do not commit; the orchestrator reviews and commits their work. **Push to `origin` automatically after each commit** (owner standing instruction, 2026-10-24). Never force-push or rewrite published history; if a push is rejected, stop and report instead of forcing.

</CRITICAL_INSTRUCTION>
