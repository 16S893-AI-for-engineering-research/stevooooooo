---
id: AA-52
title: 'Touching fold: rest each lower arm on its shoulder at the printing pose'
status: Done
assignee:
  - '@orchestrator'
created_date: '2026-11-13 03:10'
updated_date: '2026-11-13 03:48'
labels:
  - hardware
dependencies:
  - AA-50
references:
  - >-
    backlog/tasks/aa-50 -
    Full-cycle-movement-test-on-the-real-arms-replay-the-AA-45-inchworm-cycle-without-the-tube-grippers-moving.md
priority: medium
ordinal: 52000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Owner request 2026-11-13: the folded (printing) pose should have each lower arm resting on its own shoulder, so the arm is supported while printing and not held only by motor torque. Prepared: the model allows lower-arm/shoulder contact at any pose (mesh.always_allowed_pairs); the elbow EEPROM Max was raised +60 (L 3151, R 3106; backups in calibration_data). The replay still caps the fold elbow at L 3081 / R 3036 (cycle_replay.FOLD_ELBOW_CAP_TICKS). The hand-placed touching fold measured elbow L 3120-3121, R 3075-3076. OPEN OWNER DECISION before any code: how to press into the contact (proposed: drive the elbow slowly into contact at a capped torque, e.g. about 15%, like the gripper lock, with the owner confirming by eye).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 The owner has decided how the elbow presses into the shoulder contact (force cap, speed, how it is confirmed), recorded in this task
- [x] #2 At the start and end of the replay both lower arms rest on their shoulders as decided, shown in a logged live run with the owner's observation
- [x] #3 Contact is limited as decided (capped torque or position) and the run log shows elbow load and current at the fold
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [x] #1 Acceptance criteria are each verified by concrete evidence (test output, measurement, or a logged hardware run) and not by code inspection or intent
- [x] #2 Acceptance criteria and task description were NOT altered to match the implementation; any scope or criterion change was approved by the owner first and is recorded in the task
- [x] #3 Hardware facts used came from doc-1; any corrected measurement was written back to doc-1 via `backlog doc update doc-1` in the same task
- [x] #4 Tests pass via `uv run pytest` from robot/, including mock-backend tests that run with no hardware attached
- [x] #5 Safety invariants still hold and are covered by a test: single-camera USB exclusivity, always-one-arm-holding, gripper servo commanded only by the AA-17 state machine, laser off on every exit path
- [x] #6 Implementation notes record measured values, deviations from plan, and anything deferred
<!-- DOD:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Chosen (owner delegated 2026-11-13):
- The streamed path stays capped at the old fold elbow (L 3081 / R 3036). At the start pose (after the final adjustment) and at the end of the cycle, a separate FOLD PRESS step moves only the elbows.
- Press: elbow torque limit 150/1000 (15%), Goal_Speed 100 ticks/s (~9 deg/s), acceleration 20, goal = hand-measured contact + 15 ticks (L 3135, R 3090; both inside EEPROM Max - 10). The servo stalls at the contact; the 15% cap limits the force (below Overload_Torque 80%, so no overload trip).
- Settle: wait until the elbow stops moving (max 4 s), hold 2 s, log elbow position, load, current, temperature, and ticks short of the measured contact (L 3120, R 3075).
- No automatic abort (owner dislikes needless refusals): ending > 20 ticks short of the measured contact is LOGGED as a warning only.
- Leaving the start press: elbow goal back to the cap tick first, then torque limit back to 500, wait for arrival, then the stream begins.
- Confirmation by eye: at the end, before torque release, the script asks the owner 'did both lower arms rest on their shoulders at the start and now? y/n'; logged.
- Offline preflight of the press move (elbow cap -> press goal, both arms together): soft limits, EEPROM, cell mesh (lower arm/shoulder contact allowed).
- --no-fold-press keeps the old behaviour.
- Tests against simulated servos (contact emulated as a stall); dry-run; then one owner-supervised live run.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-13 software slice (not run live):
- cycle_replay: FOLD_CONTACT_TICKS (L 3120, R 3075 measured), FOLD_PRESS_GOAL_TICKS (L 3135, R 3090), FOLD_PRESS_TORQUE_LIMIT 150, FOLD_PRESS_SPEED_TICKS_S 100; fold_press_frame, fold_press_preflight, ReplayRunner.fold_press / fold_unpress. Readings logged only; >20 ticks short of contact = warning, never an abort.
- replay_cycle.py: press on by default at start (after the final adjustment) and end, owner y/n prompt logged after each press; --no-fold-press disables. Preflight of both press moves (soft limits, EEPROM, mesh) OK on the real timeline.
- replay_sim: one-sided contact stop (blocks only travel past the tick). Dry-run: presses end at 3120/3075 (0 short), cycle worst error 20 ticks.
- Tests: 4 new in tests/test_cycle_replay.py; full suite passes.
- Next: owner-supervised live run: cd robot && uv run python scripts/replay_cycle.py --live --allow-unverified-soft-limits "owner 2026-11-13: AA-52 touching fold". AC #1 is the owner's to accept (decision was delegated); AC #2/#3 need that run.

2026-11-13 03:46-03:48Z live run (owner present), log robot/calibration_data/replay_cycle_2026-11-12.jsonl:
- Start: approach_problems [], every joint within 0.3 deg of frame 0.
- Fold press START: settle 0.82 s; left elbow 3123, right 3078 (3 ticks past the measured contact, steady over 38 readings); elbow load 104 both, current 5 / 4.9 raw; owner y.
- Unpress back to the cap: 0.48 s, elbow error 17 / 15 ticks (inside the 20-tick reach tolerance). The full fluid cycle (4 segments) followed with no read errors.
- Fold press END: settle 0.81 s; left 3123, right 3078; load 104, current 5.1 / 4.8 raw; owner y. Torque released with no errors.
- Temperature reads of 47-65 C appeared during motion; these are the known bogus single reads (at rest 30-35 C). Logged only.
- Owner: 'worked great'.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-11-13 03:23
---
Owner 2026-11-13 (asked how to press into the contact): 'use your best judgement. just let me know what you choose'. Decision delegated to the orchestrator; choice recorded in the plan.
---

author: @orchestrator
created: 2026-11-13 03:48
---
Owner 2026-11-13 after the live run: 'worked great'. Taken as acceptance of the delegated press method (15% elbow torque, 100 ticks/s, goal = contact + 15, confirmed by eye).
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Touching fold: at the start and end of the replay each elbow alone presses into its shoulder at 15% torque and 100 ticks/s, aiming at contact + 15 ticks; readings logged, warnings never abort; owner y/n prompt. Verified by tests, a dry-run and a live supervised run (both arms rested at 3 ticks past the measured contact, load 104, current about 5 raw, owner y both times, owner: 'worked great').
<!-- SECTION:FINAL_SUMMARY:END -->
