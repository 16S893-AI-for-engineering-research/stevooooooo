---
id: AA-2
title: Build human-only task handoffs and a singleton walkthrough tab
status: Done
assignee:
  - '@pi'
created_date: '2026-10-20 00:10'
updated_date: '2026-10-20 00:27'
labels: []
dependencies: []
type: feature
ordinal: 2000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Provide a minimal-human-intervention workflow in the main Pi session. Human-only requests must be accepted by the user, recorded in Backlog, and guided step by step by a separate persistent Pi conversation in one Windows Terminal tab under WSL.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Human requests require justification, alternatives, expected outcome and numbered verifiable steps; duplicate proposals are suppressed.
- [x] #2 Only explicit user interaction accepts a procedure; editing its scope invalidates approval.
- [x] #3 Accepted tasks queue into one managed walkthrough instance, with restart/resume and next-task behavior.
- [x] #4 The guide records step checkpoints and evidence references in Backlog, pauses on uncertainty, and returns results for orchestrator verification.
- [x] #5 Automated tests cover transitions, acceptance, locking, recovery and launcher argument construction; usage and limitations are documented.
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Implement a project-local extension, policy skill and restricted guide profile. Use the Backlog CLI as the task store, private local runtime metadata and OS flock locks for serialization/single-instance launch. Add approval dialogs, user-input routing, durable completion notifications, and tests with an isolated Backlog project and mocked terminal launcher.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implemented project-local Pi extension (.pi/extensions/human-handoff/index.ts), human-handoff policy skill, foreground walkthrough profile, Backlog-backed state engine, and Windows Terminal/WSL singleton launcher. Task acceptance is only through an interactive confirmation; synthetic input is ignored, headless approval is refused, and changed procedure fingerprints invalidate acceptance. Guide tool access is limited to read/queue/checkpoint. Native Pi sessions persist privately; outcomes remain in Backlog and are delivered to the originating orchestrator as follow-up evidence.

Verification: 14/14 Node tests passed, including isolated real Backlog CLI round trip, actual extension loading with mocked Pi UI, canceled/confirmed approval, scope changes, deduplication, step progression/pause/resume/next, result delivery, guide tool restrictions, flock serialization and launch failure/retry. Pi RPC command discovery recognized /human and /walk without model calls. A short-lived real wt.exe -> wsl.exe -> Ubuntu probe succeeded. The actual guide runner was also exercised under a PTY: interactive Pi reached ready state with empty queue, runner exec-replaced itself with Pi, singleton lock was held while alive and released at shutdown. No real human tasks or LLM walkthrough calls were created during tests.

Limitations documented in tools/human-handoff/README.md: one managed guide per WSL user/state directory, explicit WT target 0 defaults to most recently used window, launcher cannot guarantee focus of existing tab, custom Backlog statuses need adaptation, normal workflow guard is not a hostile-process sandbox, and full live photo/hardware walkthrough remains a user acceptance check. Guide disables unrelated extensions including Parley to keep the role restricted; main Pi session is unchanged. Global Pi settings and unrelated files were not modified. Activate with /reload; no commit/push performed for this resource work.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Built the human-only task handoff workflow: explicit approval, Backlog procedures/checkpoints/evidence, one reusable Windows Terminal walkthrough tab, restricted guide profile, and orchestrator verification/resumption. Verified with 14 passing automated tests, real Backlog persistence and Pi extension discovery, Windows Terminal/WSL launch smoke test, and actual interactive guide startup/lock lifecycle under a PTY. Documentation includes usage, privacy, scope, and remaining manual end-to-end acceptance checks.
<!-- SECTION:FINAL_SUMMARY:END -->
