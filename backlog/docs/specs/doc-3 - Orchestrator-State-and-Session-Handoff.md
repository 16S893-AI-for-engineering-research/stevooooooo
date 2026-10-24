---
id: doc-3
title: Orchestrator State and Session Handoff
type: guide
created_date: '2026-10-20 21:37'
updated_date: '2026-10-24 17:14'
---
Live orchestrator state: what is in flight, what is blocked, what is uncertain, and how to resume cold. Read this **first** when picking up this project in a fresh session, then doc-1 (hardware truth) and doc-2 (change-control rules).

**Keep this current.** Update it via `backlog doc update doc-3` whenever a blocker clears, a task finishes, an estimate changes, or a decision is made. A stale doc-3 is worse than none, because it invites confident action on wrong information. If the Last verified date below is old, trust task statuses and `git log` over this prose and re-verify before acting.

**Last verified:** 2026-10-24 by the orchestrator, at the commit that first adds `robot/`. 252 tests pass. All 7 devices are attached and resolve via discovery.

## 1. Resume in 60 seconds

```bash
cd /home/stevotimp/893
pi -c                      # continue the most recent session for this folder
# or: pi -r                # browse past sessions for this folder
```

Session transcripts live in `~/.pi/agent/sessions/--home-stevotimp-893--/` and are **machine-local, not in git**. They are not a system of record: compaction summarizes and discards older turns, and sub-agents cannot read them. Everything that matters is in the repository instead:

| Source | What it holds |
|---|---|
| `doc-1` | Hardware and system truth. Never assume a hardware fact; take it from here. |
| `doc-2` | Change-control rules. Read before implementing anything. |
| `doc-3` (this) | Live state, blockers, uncertainty, next action. |
| `backlog/tasks/` | 33 tasks, ~230 acceptance criteria, dependencies. |
| `AGENTS.md` | Loads the critical rules into any agent session automatically. |
| `backlog/config.yml` | Project-wide Definition of Done, inherited by every new task. |

Ground truth commands, which outrank this document:

```bash
backlog task list --plain            # real status of every task
backlog task list --status "To Do" --plain
backlog task view AA-4 --plain       # full criteria for one task
git log --oneline -5
```

## 2. Where the project stands

- **Done:** AA-4, AA-5, AA-6, AA-7, AA-37, and AA-12 (DexArm G-code driver; mock-verified only, with real-device behaviour exercised by AA-13; the workspace box is marked `unverified: true`).
- **Software half done, hardware ACs pending:** AA-8 (servo bus: ACs #2 live ID scan and #4 20 Hz benchmark; run `scripts/scan_servos.py --live`), AA-29 (laser relay: #1 protocol is unverified LCUS-1, so run `scripts/laser_probe.py --live`; #3 waits on AA-10 e-stop; #5 because LCUS-1 cannot report state).
- **Lessons recorded:** agent self-audits passed three real safety bugs (camera lock per device, bounds fail-open and the raw-motion regex, laser max-on not enforced). The orchestrator must independently check every safety-relevant change.
- Parallel lanes run in managed worktrees. The orchestrator applies the returned patches with `git apply --3way` and runs the full suite before committing.
- Package: `robot/src/inchworm/`, tests in `robot/tests/` (461 pass), config in `robot/config/hardware.yaml`.

## 3. Blocked right now / pending owner input

- Left/right arm mapping is still owner-reported. It needs a supervised low-speed motion test before any trajectory runs.
- The tube fixture for demonstrations (decision-1) is a physical owner deliverable under AA-23 AC#4.
- The single-camera invariant stands until AA-38 has measured concurrency and the owner decides.

## 4. Next action

Software halves (mock backend, no hardware I/O) of **AA-8** (SO-101 servo bus), **AA-12** (DexArm G-code) and **AA-29** (relay laser fail-safe), one at a time, each with a Sonnet review. Their hardware ACs stay unchecked until an owner-supervised session, with dry runs first.

## 5. Decisions already made (do not silently revisit)

Each of these came from an explicit owner answer or a verified observation. Changing one is a scope change per doc-2 section 4.

- **The chassis is fixed; the tube moves.** This inverts the usual mobile-robot convention. Every frame computation must respect it.
- **The gripper latch and jaw share one servo ID** (owner's custom gripper). Lock and unlock are *position bands* of a single servo, not separate actuators. Locking is commanded by code and never by the learned policy, per the owner's explicit instruction.
- **Demonstrations use cross-arm teleoperation with the tube held by a fixture** (decision-1, owner 2026-10-24). A torque-off leader never holds the tube, and the gripper is never teleoperated. Start with one arm; AA-39 decides on the second.
- **Both arms hold the tube at the start of every cycle and whenever printing** (owner, 2026-10-24).
- **Commit and push automatically** after each verified unit of work (AGENTS.md rule 7). Never force-push.
- **Control host is WSL2** with usbipd attach, chosen for LeRobot support.
- **Slicing is done by the owner in Cura**, and a correct DexArm profile must be installed there first. AA-34 owns that boundary; do not assume a profile exists.
- **The real crawl cycle depends on learned insertion** (AA-27 depends on AA-26), per the owner's observation that crawl cannot really happen until insertion works. A tube-free dry rehearsal (AA-21) comes first.
- **DexArm repeatability is measured before laser calibration** (AA-30 depends on AA-13), so arm positioning error is not silently absorbed into the optical calibration.
- **Printing is deferred.** The near-term goal is crawl, regrip, fold and auto-level. AA-34 and AA-35 integrate printing later.

## 6. Honest uncertainty

Stated plainly so a fresh session does not mistake an estimate for a commitment.

- **Overall: roughly 45 to 90 agent sessions plus 25 to 50 hours of owner hardware time; 6 to 10 weeks hands-on, 3 to 5 months at weekend pace.** Hardware tasks cannot be parallelized across sub-agents: one robot, one supervising human.
- **Owner demonstration time (added 2026-10-24):** about 6-15 extra hours. The first dataset is roughly 50-100 episodes per arm (LeRobot guidance: 50 or more, about 10 per variation), about 3-8 h. Then 2-3 HIL correction rounds at about 1-2 h each. Any change to the camera mount, attachment geometry or approach pose can invalidate the recorded data.
- **M3 (learned insertion) is the least predictable phase**, estimated at 10 to 25 sessions. No leader arms makes demo capture awkward, and 4 GB of VRAM constrains architecture. If the first policy does not generalize across the coarse-approach offset range, the recollect, retrain, re-evaluate loop costs days per iteration. This is the number least worth trusting.
- **AA-13 could force a redesign of M5.** If DexArm repeatability cannot hold roughly 0.1 mm at the scan standoff with the print head and scanner payload attached, leveling precision is capped. Deliberately sequenced early so this surfaces in week 2 rather than week 8.
- **AA-16 calibration quality propagates into M3.** Loose transforms widen the capture range the policy must absorb.
- **AA-31 line extraction on a glossy, curved, ridged printed surface is harder than on a flat coupon.** A method that passes on a test coupon and fails on the real tube would look fine in review and then produce quietly wrong conclusions.
- **Estimates for m-0 through m-2 are defensible. From m-3 onward they are a forecast.**

## 7. Stability model, and its limit

doc-2 holds the rules. The mechanisms in place: criteria written before code, doc-1 referenced rather than copied, a project-wide Definition of Done every task inherits, dependency ordering, mock backends so regressions appear as failing tests instead of crashes, the safety boundary built and tested before the model exists (AA-20), a dry rehearsal before anything touches the tube (AA-21), and recorded baselines in AA-13, AA-18 and AA-21 so later drift is detectable.

**The limit, stated honestly:** the Definition of Done and doc-2 are strong defaults, not enforcement. A determined agent can still check a box it has not earned. The real backstop is the orchestrator reviewing `git diff` for unapproved changes to acceptance criteria, doc-1 numbers, or safety checks. A pre-commit hook that fails on modified `AC:` lines without an approval marker would make drift mechanically hard rather than merely discouraged; it has been proposed to the owner but is **not** implemented.

## 8. Repository hygiene notes

- `.parley.env` contains a live API key and is gitignored. This repo pushes to a GitHub org remote, so never stage it. Verified absent from `origin/main`.
- `*Zone.Identifier` files are gitignored; one that had already been committed was untracked in `98840c7`.
- `robot/sessions/`, `robot/out/`, `robot/datasets/` and `robot/checkpoints/` are pre-ignored so AA-7 logs, camera frames and policy checkpoints cannot be committed by accident.
- Do not commit photographs, session transcripts, or secrets as task evidence. Keep frames local unless the owner authorizes publication.
