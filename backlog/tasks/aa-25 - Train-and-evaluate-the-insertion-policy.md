---
id: AA-25
title: Train and evaluate the insertion policy
status: To Do
assignee: []
created_date: '2026-10-20 21:12'
updated_date: '2026-11-20 02:55'
labels: []
milestone: m-3
dependencies:
  - AA-23
  - AA-24
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 25000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
This is the learned component the owner specified: final insertion into the attachment point, bounded by code on both sides. Training must fit a 4 GB GPU, so architecture, image resolution and batch size are constrained, and the evaluation has to be against the real capture range produced by the coarse approach rather than against a convenient subset.

The acceptance bar is behavioural: the policy must succeed often enough, and fail safely enough, that the supervised autonomous cycle is viable. A policy that succeeds frequently but occasionally drives hard into the structure is worse than a slightly less successful but gentle one.

See doc-1 sections 7 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A training pipeline trains a LeRobot policy on the captured dataset within the 4 GB VRAM budget, with the chosen architecture, resolution, batch size and hyperparameters recorded
- [ ] #2 Training is reproducible from a pinned config and a recorded dataset version, and produces a versioned checkpoint artifact outside version control
- [ ] #3 Offline evaluation reports held-out action error, and online evaluation reports insertion success rate over at least 30 real attempts spanning the measured coarse-approach offset range
- [ ] #4 Success is judged by the independent AA-24 seating detector, not by the policy or by visual impression
- [ ] #5 Failure modes are characterized and reported (for example miss direction, jam, oscillation, timeout) with counts
- [ ] #6 Maximum observed contact force or servo load during attempts is recorded, and runs that exceed a documented safe load are counted as unsafe failures regardless of outcome
- [ ] #7 A go or no-go recommendation for autonomous use is stated with the measured success rate, the unsafe-failure count and the retry count needed to reach an acceptable per-cycle success probability
- [ ] #8 At least one human-in-the-loop correction round (policy rollout with owner takeover recorded, e.g. lerobot-rollout --strategy.type=dagger or equivalent, then fine-tune) is run, and success before and after each round is reported; up to 3 rounds are budgeted
- [ ] #9 Local training memory feasibility on the 4 GB GPU is measured first; if it is infeasible, training runs on a documented cloud target (Colab or HF Jobs) with a private, non-pushed dataset or explicit owner approval to upload, and the cost is recorded
- [ ] #10 Online success is reported separately for the fixture setup and for the real crawl configuration, so fixture-to-crawl distribution shift is measured
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-19 first ACT run, dataset insert_left_20261119T221003Z (29 ep; train 0-25, held out 26-28), 20k steps bs 8 chunk 100 n_action_steps 20, 480x640, resnet18, local GPU 2.1-2.9 GB, ~3 h 20 min, final l1 0.091 (script out/train/run_insert_left_20261119T221003Z.sh). OFFLINE held-out eval (robot/lerobot_env/eval_offline.py; results out/train/insert_left_20261119T221003Z/eval/heldout_<step>.json), mean |pred - recorded action| raw servo deg over the executed 20-step chunk, vs a hold-still baseline 0.917: 5k 1.024, 10k 0.930, 15k 0.883, 20k 0.797 (end pose 20k 0.819; 10k 1.065). Training ep 0 at 20k: 0.559 vs hold 0.617. Per joint the largest errors are elbow_flex and wrist_flex (end pose 1-3 deg). Verdict: 20k > 10k; only modestly better than holding still; weak first policy. rollout.py dataset-replay dry-run (20k, eps 26-28, max-steps = episode length): rc 0, 0 clamps, nothing commanded, median |policy - recorded| 3.0/2.2/1.6 deg, outcome HALT (stub seating never seats; expected). No AC checked (online evaluation needs hardware + AA-24).

2026-11-20 owner: go straight to a physical test. Built robot/lerobot_env/policy_handoff.py/.sh (+ tests/test_policy_handoff.py, 6 tests): ACT checkpoint (default 20k) drives the left arm through inchworm_follower tube_support=handoff (handoff checks + YES, mesh gate per goal, limits, stop file, joints 1-5 only, release prompt). Agent placeholders UNVERIFIED: speed cap 5 deg/s, jump guard 10 deg (stop+hold), 20 s cap. No software success judgement (AA-24 not wired). Dry run (simulated servos at the handoff stop, real checkpoint + cell mesh, ep 26 frames): 20 s, 0 holds, inference p50 8 ms p95 34-64 ms, ~6% loop overruns; arm moved elbow -280 / wrist_flex +233 ticks, demo-like. Trace robot/out/rollout/policy_dry_*.jsonl. First live run: owner-supervised.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:07
---
ACs #8-#10 added 2026-10-24 with owner approval (plan item E, decision-1). Start with one arm's policy (item H).
---
<!-- COMMENTS:END -->
