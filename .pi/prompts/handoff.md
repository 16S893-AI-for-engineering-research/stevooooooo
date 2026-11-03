---
description: Wrap up the session so the next one can start fresh
---
Prepare this session for a restart (AGENTS.md rule 8):
1. Append implementation notes (what was done, measured values, deviations) to every Backlog task touched this session.
2. Update doc-3 via `backlog doc update doc-3`: Last verified line (date, commit, test count), blockers, and the "Next actions" list.
3. Run `cd robot && uv run pytest`; it must pass.
4. Commit and push, checking the staged files for secrets, .venv, caches, sessions, frames, datasets and checkpoints.
5. Reply in at most 5 lines: what changed, what is next, and the exact "resume with:" line to type after /new.
