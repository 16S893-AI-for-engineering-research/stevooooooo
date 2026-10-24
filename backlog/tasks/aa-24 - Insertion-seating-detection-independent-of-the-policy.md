---
id: AA-24
title: Insertion seating detection independent of the policy
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:12'
updated_date: '2026-10-24 18:28'
labels: []
milestone: m-3
dependencies:
  - AA-8
  - AA-17
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 24000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Code, not the model, decides when insertion has succeeded, because the lock must only be commanded once the gripper is actually seated in the attachment point. A false positive locks into thin air and the next advance drops the tube; a false negative wastes retries. The detector therefore needs to be a separate, independently testable component rather than a side output of the policy.

The primary signal candidates are gripper-servo position and current or load from the shared servo, plus joint-load signatures on the arm; vision is a fallback if those prove insufficient. Which signals suffice is an open question in doc-1 section 10 and must be answered with measured data.

See doc-1 sections 3, 9 and 10.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A detector returns seated, not-seated or uncertain from sensor signals available during insertion, with the signals and thresholds documented
- [ ] #2 Thresholds are derived from recorded data covering at least 20 genuine seatings and 20 near-miss or jam cases, and the dataset used is retained
- [ ] #3 False-positive and false-negative rates are measured on held-out cases and reported, with false positives treated as the more serious failure and explicitly minimized
- [x] #4 The detector does not consume policy internals or policy confidence and can be evaluated standalone from a recorded trace
- [x] #5 The uncertain verdict triggers a documented disambiguation step (for example a gentle probe move or a vision check) rather than defaulting to seated
- [x] #6 Detection runs within a latency budget compatible with the supervisor loop rate, measured and recorded
- [x] #7 Tests cover each verdict path and threshold-boundary behaviour against recorded traces
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Design seating.py: JointStateSample datatype, SeatingSamples trace type, SeatingVerdict enum (SEATED/NOT_SEATED/UNCERTAIN), ProbeRequest object.
2. Implement SeatingDetector class: evaluate_trace(trace) -> (verdict, features); supervisor-compatible __call__ wrapping a stream of JointState samples.
3. ThresholdConfig in config.py: Optional fields with null defaults (position_band, load_rise, settle; all Optional[...] = None). Add seating_detector block to HardwareConfig.
4. UNCERTAIN triggers ProbeRequest with probe_move_m and re-evaluates after executor runs it (injected GuardedArm executor in probe path).
5. ThresholdFitter: given labelled JSON-lines trace file, fits thresholds minimising FP subject to FN cap; reports FP/FN on held-out split.
6. Record script: scripts/record_seating_traces.py (dry-run default, --live flag) that reads JointState samples from bus and writes JSON-lines.
7. Tests (tests/test_seating.py): synthetic seated/jam traces (40+ each), verify FP/FN, each verdict path, threshold-boundary, latency measurement vs supervisor loop rate.
8. Add seating_detector config to hardware.yaml with null thresholds. Add SeatingDetectorConfig to config.py under AA-24 marker.
9. Finalize: paste test output, check software ACs; leave hardware ACs (#2,#3) unchecked with note.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete (software half).

seating.py: JointStateSample, SeatingSamples, SeatingVerdict (SEATED/NOT_SEATED/UNCERTAIN), ProbeRequest, SeatingDetectorConfig, SeatingDetector, SeatingEvaluator, ThresholdFitter, extract_features.
config.py: SeatingDetectorConfigSchema added under # --- AA-24 --- markers; Optional field seating_detector added to HardwareConfig.
hardware.yaml: seating_detector block appended with all null thresholds (fail-closed).
scripts/record_seating_traces.py: dry-run default; --live flag for owner-supervised recording.
tests/test_seating.py: 57 tests, all pass.

Latency measured: mean 0.007 ms (full config), 0.004 ms (null config) vs 20 ms budget (50 Hz supervisor loop). 2800x under budget.
Synthetic FP/FN on 50 seated + 50 not_seated: FP=0.000, FN=0.000 (best_threshold=47 raw units). Pipeline validated; real data needs hardware session.

AC#2 unchecked — needs owner-supervised hardware session:
  python scripts/record_seating_traces.py record --label seated --out dataset.jsonl --live  (≥20 runs)
  python scripts/record_seating_traces.py record --label not_seated --out dataset.jsonl --live  (≥20 runs)
AC#3 unchecked — needs real recorded dataset:
  python scripts/record_seating_traces.py fit dataset.jsonl
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented inchworm/seating.py (AA-24 software half): SeatingDetector callable (AA-20 supervisor-compatible), SeatingVerdict SEATED/NOT_SEATED/UNCERTAIN, ProbeRequest for UNCERTAIN disambiguation, SeatingEvaluator probe loop, ThresholdFitter (FP-minimising, FN-capped). Config: SeatingDetectorConfigSchema in config.py + null-threshold seating_detector block in hardware.yaml. Script: scripts/record_seating_traces.py (dry-run default, --live). Tests: 57/57 pass; latency 0.007 ms vs 20 ms budget; synthetic FP=0.000/FN=0.000. ACs #2 and #3 left unchecked (require owner-supervised hardware session with real data).
<!-- SECTION:FINAL_SUMMARY:END -->
