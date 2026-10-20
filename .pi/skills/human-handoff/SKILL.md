---
name: human-handoff
description: Use when agent work genuinely requires the user's physical access, identity, authorization, or personal judgment. Minimize human-only work, propose verifiable Backlog procedures for explicit acceptance, and coordinate a separate walkthrough tab. Never use to offload routine agent work.
---

# Minimal human-only handoffs

The main Pi session is the orchestrator. It owns necessity checks, proposal quality, result verification, and resumption. Read this before calling human_request.

## Eligibility gate

1. Identify the existing originating Backlog task and the necessary outcome.
2. Explain why only the human can achieve it: physical access, identity, permission, or personal judgment.
3. Investigate available, authorized tools and reasonable alternatives within a bounded effort. A failed tool attempt, missing package, or routine implementation choice is not proof the human is needed. Never bypass restrictions or seek extra access merely to avoid asking.
4. Reduce the request to the smallest human intervention. Do research, commands, preparation, and follow-up yourself when possible. Bundle closely related physical actions; avoid repeated trips and duplicate tasks.
5. Use human_queue to check pending, declined, or deferred requests. Respect those decisions; do not recreate requests or nag.
6. Write 1–20 numbered steps, each with action, expected result, verification, and stop condition. Include prerequisites, approximate active time, and the result that unlocks agent work. Do not invent hardware-specific facts.

## Proposal and acceptance

- Use human_request; it creates a human-action task through the Backlog CLI. It does NOT accept it.
- Tell the user the returned ID and estimated effort once. The user can type `accept AA-N` or use `/human` to review. A user-facing confirmation binds acceptance to the exact procedure.
- Never call tools, issue synthetic commands, write runtime files, or edit Backlog metadata to pretend the user accepted. Approval is an explicit human UI action.
- Continue independent work while waiting. Accepted tasks are queued in one Walkthrough tab; do not launch ad hoc guides or ask the user to repeat instructions in another session.
- For procedure revisions, pause first and regenerate the displayed procedure and its machine-readable block consistently through the CLI. Any title, description, or acceptance-criterion change invalidates the prior acceptance. Require a new user review; never expand scope silently.

## Results

The guide stores checkpoints/evidence in Backlog and submits results as ready, not Done. The orchestrator receives the outcome in its session (or can request `/human results` after returning).

- Inspect evidence; distinguish user self-report from independent checks.
- Run any available automated verification.
- Use human_verify only with concrete verification evidence. This completes only the human handoff, not the originating project task.
- If results are inadequate, explain the discrepancy and ask whether the user wants a revised/new task rather than quietly assigning extra work.
- Do not commit photographs, session transcripts, or secrets as incidental task evidence. Keep photos local unless the user authorizes publication. Task notes may be version-controlled.

## UI commands

Main session: `/human`, `/human accept ID`, `/human defer ID`, `/human decline ID`, `/human status`, `/human open`, `/human setup`, `/human retry`, `/human results`.

Guide tab: `/walk next`, `/walk pause`, `/walk queue`, `/walk finish`. The guide is a restricted interactive Pi session, not an unattended subagent. The tab is launched only through the configured Windows Terminal/WSL launcher.
