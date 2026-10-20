---
id: doc-2
title: Agent Working Agreement — Requirements Stability and Change Control
type: guide
created_date: '2026-10-20 21:22'
updated_date: '2026-10-20 21:23'
---
Rules every agent (including sub-agents) must follow when executing tasks on this project. The orchestrator enforces these at review time. Read with doc-1, which holds the hardware facts.

## 1. The core failure mode we are preventing

The characteristic way an agent-built robotics project rots is **silent requirement drift**: an agent cannot satisfy an acceptance criterion, so it edits the criterion, widens a tolerance, deletes a test, stubs a safety check "temporarily", or marks a criterion done because the code *looks* right. Each individual change seems reasonable. Fifteen tasks later the system no longer does what was specified, and nobody can point to when that changed.

In this project that failure mode is not merely untidy: the arms share a crowded workspace, the tube is load-bearing mid-cycle, and a line laser is software-switched. Drift in a safety invariant can bend hardware.

## 2. Hard rules

1. **Never edit an acceptance criterion or task description to match what you built.** If a criterion is wrong, unachievable, or ambiguous, stop and raise it with the owner. Record the question as a task comment. Do not proceed on your own interpretation of a criterion you believe is wrong.
2. **Never mark an acceptance criterion checked without evidence.** Evidence is test output, a measurement, or a logged hardware run. Code presence, grep results, and "the implementation does this" are not evidence. This is Definition of Done item 1 and it is checked at review.
3. **Never weaken a tolerance, limit, or safety check to make something pass.** Widening a limit is a scope change and needs owner approval. A failing test is information, not an obstacle.
4. **Never delete, skip, or `xfail` an existing test to get green.** If a test is genuinely wrong, say so explicitly in the task and get agreement.
5. **Do not expand scope.** Build what the criteria ask for. If you notice adjacent work that should happen, propose it; do not silently fold it in. Per the Backlog finalization guide, do not create or start follow-up tasks without owner approval.
6. **Hardware facts come from doc-1, not from your own assumption.** If you measure a value that contradicts doc-1, update doc-1 via `backlog doc update doc-1` in the same task and say so in your notes. Never let a per-task copy of a number diverge.
7. **Stay inside your task's dependencies.** If your task needs something a dependency was supposed to deliver and it is missing or broken, stop and report. Do not reimplement a neighbouring task's responsibility inline.
8. **Hardware is not a sandbox.** Before any first powered motion on a new path, run the dry-run mode, then ask the owner to supervise the first real execution. Do not improvise a motion test to unblock yourself.

## 3. The safety invariants (never negotiable without owner approval)

These are system-level and cross-cutting. Any task may be rejected for breaking one, even if its own criteria pass:

- **Single-camera USB exclusivity.** One camera streaming at a time, enforced through the AA-6 arbiter. Everything shares one USB 2.0 uplink.
- **Always one arm holding.** No command sequence may leave both grippers unlocked while the tube is attached.
- **Code owns the gripper.** The gripper servo is commanded only by the AA-17 state machine. The learned policy must be structurally unable to touch it, not merely discouraged.
- **Laser fail-safe off.** The laser is off on every exit path, including exception, emergency stop and signal.
- **Limits are enforced below the caller.** Joint limits and the collision envelope are checked inside the motion layer; bypassing requires an explicit, documented override.

Every task's Definition of Done includes a check that these still hold and are covered by a test.

## 4. How a change to requirements is made legitimately

Drift is bad; *change* is normal and expected on a prototype. The difference is visibility.

1. State the problem, the options, and your recommendation to the owner.
2. Get an explicit decision.
3. Record it: for anything architectural or anything that alters a safety invariant, create a decision record via `backlog decision create`, so the reasoning survives.
4. Update the affected task criteria and doc-1 through the `backlog` CLI.
5. Then implement.

A criterion changed this way is fine. A criterion changed quietly during implementation is not.

## 5. Stability mechanisms already in place

- **Specification before code.** All 33 tasks carry explicit, testable acceptance criteria written before implementation, so "done" is not negotiated after the fact.
- **One source of hardware truth.** doc-1 is referenced by tasks instead of copied, so a corrected measurement propagates.
- **Project-wide Definition of Done.** Configured in `backlog/config.yml`; every new task inherits it, including the no-criteria-rewriting and evidence rules.
- **Dependency ordering.** Tasks declare dependencies, so an agent cannot start work whose foundations do not exist.
- **Mock backends everywhere.** Logic is testable with zero hardware attached, so a regression surfaces as a failing test rather than as a crash into the chassis.
- **Structured telemetry from the start (AA-7).** Unattended failures are reconstructable from logs rather than from guesswork.
- **Safety boundary built before the model (AA-20).** The supervisor, limits and abort paths are tested against a stub policy, so the learned component is swapped in behind an already-proven boundary.
- **Dry rehearsal before live (AA-21).** The motion skeleton is shaken out in free air before the tube is ever at risk.
- **Regression baselines.** AA-21, AA-13 and AA-18 each record measured baselines (cycle timing, DexArm repeatability, approach repeatability), so later drift is detectable rather than invisible.

## 6. Review gate the orchestrator applies

A task is not accepted until:

- Each checked acceptance criterion names its evidence.
- `git diff` shows no unapproved edit to any task's criteria, to doc-1 numbers, or to a safety check.
- The test suite passes, including mock-backend tests.
- Measured values are recorded in implementation notes, and anything deferred is stated explicitly rather than left implied.
