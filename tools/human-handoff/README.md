# Human-only task handoffs for Pi + Windows Terminal / WSL

The main Pi session stays the orchestrator. It proposes only necessary human-only work; the user explicitly accepts a versioned procedure. One dedicated **interactive Pi tab** walks through accepted work. Backlog stores task/procedure/progress/outcome records; a private user directory stores only launcher, project-discovery, and session runtime data.

## Activate

From this trusted repository in Pi, run:

```text
/reload
/human status
```

The extension is auto-discovered at `.pi/extensions/human-handoff/index.ts`. No npm install or global Pi settings change is required. The current running agent will not gain new tools until `/reload` or restart.

Requirements: Pi 0.85.1-compatible extension APIs; Node >=22.15 (tested with 22.23.2); Backlog CLI with schema-version-1 JSON; WSL Linux with `flock`; Windows Terminal `wt.exe` and `wsl.exe` accessible on PATH. Current project uses Backlog statuses `To Do`, `In Progress`, `Done`; these names are presently required.

## Main-session workflow

Ask Pi normally to manage the project. The `human-handoff` skill and `human_request` tool require an existing originating task, human-only reason, alternatives considered, effort estimate, preparation, outcome, and steps with expected results, verification, and stop conditions.

Examples of eligible intervention: physical access, identity verification, permissions, personal judgment. Failed debugging, research, commands the agent can execute, and routine implementation decisions are not eligible merely because the agent wants help. The gate requires structured justification but does not mechanically prove necessity; the orchestrator must exercise judgment.

After a proposal:

```text
accept AA-3
```

This exact phrase is recognized only from interactive user input. A dialog shows the full readable procedure and requires your confirmation. Ambiguous `accept it` opens a picker. You can also use:

| Command | Action |
|---|---|
| `/human` | Pick a request and Accept / Defer / Decline |
| `/human accept ID` | Review and explicitly accept the current procedure |
| `/human defer ID` | Defer without repeated reminders |
| `/human decline ID` | Decline; equivalent proposals are deduplicated |
| `/human status` | See local requests and whether the guide process is running |
| `/human open` | Open/reuse the Walkthrough tab without changing task acceptance |
| `/human retry` | Explicitly retry a launch that never appeared |
| `/human setup` | Configure Windows Terminal window target and WSL distribution |
| `/human results` | Bring ready outcomes into the current orchestrator conversation |

The first launch asks permission to use Windows Terminal **window target 0**, meaning its most recently used window. This is not guaranteed to be the window that contains the main Pi session. Set a window ID/name with `/human setup` when needed. The default distro is the current `WSL_DISTRO_NAME` (`Ubuntu` in the development environment).

Accepting records the exact title/description/acceptance-criterion fingerprint and then opens/queues the task. Canceling terminal setup or a launch failure does **not** silently unaccept the task: it remains queued.

## Walkthrough tab

The launcher runs a separate interactive Pi conversation with `tools/human-handoff/walkthrough.md` as its appended role prompt. That markdown file is a **foreground profile**, not a pi-subagents background-agent definition.

The guide has only `read`, `human_queue`, and `human_checkpoint` tools. Other discovered extensions, skills, prompt templates, and project context files are disabled in that guide process to avoid conflicting behavior. This also means Parley and other extension-specific recording are not active there; native Pi session persistence is active. The main orchestrator retains its normal extensions. The guide initially uses the orchestrator's selected provider/model when available.

| Command | Action |
|---|---|
| `/walk next` | Start the next accepted task, or resume the current paused task |
| `/walk pause` | Save progress and pause the current procedure |
| `/walk queue` | See accepted/active/paused tasks, with project paths |
| `/walk finish` | After all steps, enter outcome/deviations and explicitly submit for verification |
| `/compact` | Pi's native compaction; Backlog checkpoints remain authoritative |

The first accepted task starts when the newly launched guide is ready. New arrivals never replace an active step. After finishing, a dialog offers the next accepted task in the **same tab**. A paused task is resumed before another task starts; it is not silently abandoned. Closing the tab preserves progress; on restart an unfinished task is paused and requires `/walk next` before more physical instructions.

### Photos and evidence

Attach photos using Pi's native image-input support, or provide their WSL-readable file paths (for example `/mnt/c/Users/.../Pictures/label.jpg`). The guide can read an image even if Windows Terminal does not display an inline preview. The selected model must support image input; otherwise use a vision-capable model or a text observation, and never pretend an image was inspected.

Evidence is labeled `self-reported`, `photo`, `measurement`, or `automated`. A photo/measurement/test reference must identify actual evidence. Checkpoints store references and notes, **not image files**. Prefer durable local photo paths if you need evidence after compaction. Do not paste tokens/passwords, include private details unnecessarily, or commit photos without explicit permission. Backlog notes may be committed to Git.

## Completion and notification

Human workflow phases are stored separately from ordinary Backlog status:

```text
proposed → accepted → active ⇄ paused → ready → verified
        ↘ deferred / declined
```

- Proposed/accepted requests use `To Do`; active work uses `In Progress`.
- `ready` means the walkthrough outcome is available, not verified or complete.
- The orchestrator reviews evidence/runs available checks, then uses `human_verify` to mark this handoff `Done`.
- No operation completes the originating project task automatically.
- The main session checks for outcomes every 10 seconds. For the originating session it queues a follow-up turn; other sessions show the outcome without starting an autonomous turn. Closing the orchestrator does not lose results: use `/human results` on return.
- Outcome notices are deduplicated per Pi session. The durable task record remains authoritative if a notification is missed.

## Storage, locking, and trust

All task mutations use the **Backlog CLI**, never direct task-markdown edits. Procedure data lives in the task description; state/checkpoints live in a delimited JSON block in implementation notes. Unrelated notes are preserved. Changing title, description, or criteria invalidates acceptance; the next operation refuses to continue until reviewed again. Displayed steps must agree with the structured procedure. When revising, use the CLI to regenerate both consistently; do not modify only one representation.

Private runtime defaults to `~/.pi/agent/human-handoff/` (mode 0700 directory, 0600 JSON files):

- `projects.json`: registered project roots; guide queue can include several explicitly registered projects.
- `current.json`: current task identity, not a second progress store.
- `config.json`: terminal window/distro preferences.
- `launch.json`: launch state and executable locations; no copied API credentials.
- `sessions/`: Pi's private walkthrough transcripts, including submitted images.
- `operations.lock`: kernel flock serializing extension writers.
- `guide.lock`: kernel flock held for the full guide-process lifetime. The launcher replaces its runner with Pi via `execve`, avoiding an orphaned guide if a wrapper crashes. The lock, not a stale `launch.json` status, determines whether the guide is alive.

One managed guide process is allowed **per WSL user/state directory**. Multiple project sessions share that lock and accepted queue. It does not coordinate other distributions, Windows users, machines, or manually launched unmanaged Pi sessions. A pending launch is not blindly retried: use `/human retry` after checking the terminal. The process lock prevents duplicate active guides even if two launch requests reach Windows Terminal; an unusual delayed/retried request could still leave an empty/closed duplicate tab.

The guide starts using the repository associated with its launch; queued tasks include explicit project paths, so file references must be absolute when crossing projects. Installed code/profile paths must remain available. Missing projects, corrupt state, or CLI failures surface as errors rather than being silently dropped.

**This is a workflow guard, not a hostile-process security sandbox.** A process running under your OS account with shell/filesystem access can modify tasks, runtime files, or the extension. The normal tool API offers no acceptance action, synthetic input is not treated as acceptance, and all approval commands require a live user dialog. Do not concurrently edit handoff state through unrelated CLI processes; `flock` coordinates this extension's writers, not external programs.

## Tests

From the repository root:

```bash
node --test tools/human-handoff/tests/*.test.mjs
```

Tests use temporary directories, isolated Backlog projects, a mocked terminal executable, and a mocked Pi UI. They cover justification/schema checks, synthetic-input rejection, canceled/confirmed approval, scope drift, duplicate suppression, checkpoints, resume/next, verification, notes preservation, locking, launch failure/retry, and guide tool restrictions. Extension tests use the installed Pi package's own loader/dependencies; no LLM calls are made.

A real Windows Terminal → Ubuntu launch was also smoke-tested with a short-lived marker-writing process. This is not a claim that a full human/photo/hardware walkthrough has been manually validated.
