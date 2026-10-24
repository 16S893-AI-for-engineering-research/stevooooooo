---
id: AA-33
title: Apply per-hop leveling correction to the DexArm print frame
status: Done
assignee:
  - '@agent'
created_date: '2026-10-20 21:13'
updated_date: '2026-10-24 18:47'
labels: []
milestone: m-5
dependencies:
  - AA-32
  - AA-19
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 33000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
A measured correction is useless until it changes where the print head actually goes. After every hop the part sits slightly differently, so the correction must be applied to the DexArm print frame before the next segment is printed, and it must be applied in a way that is auditable and bounded: a bad scan that produced a large correction must not be allowed to drive the nozzle into the part.

This closes the leveling loop and is the last piece needed before unattended multi-cycle operation.

See doc-1 sections 1, 3 and 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 The Z offset and tilt correction are applied as a transform on the DexArm print frame, and the applied transform is logged with the scan that produced it
- [x] #2 Corrections larger than a configured sanity bound are rejected and trigger a rescan, then a safe halt if the rescan also exceeds the bound
- [x] #3 Correction is applied after folding and before any print motion, and the ordering is enforced rather than left to the caller
- [x] #4 Verification rescans after applying the correction and confirms the residual Z offset and tilt are within tolerance, reporting the residual
- [x] #5 Corrections are not silently accumulated across hops; the policy for absolute versus incremental application is documented and implemented explicitly
- [x] #6 A per-hop record of measured correction, applied transform and verified residual is written to the session artifacts
- [x] #7 Tests cover application, the sanity-bound rejection and rescan path, ordering enforcement and residual verification
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Create inchworm/print_frame.py with:
   - PrintFrameConfig dataclass: sanity_bound_mm, residual_tolerance_mm, artifacts_dir (Optional)
   - PrintFrameState enum: AWAITING_FOLD, FOLDED_READY, APPLIED_VERIFIED, HALTED
   - FrameCorrectionHalt typed error for second-failure stop
   - PrintFrame class with state machine, apply_correction(LevelingResult, rescan_fn) -> transform, get_corrected_transform() -> np.ndarray
   - Absolute-replacement policy (AC#5): each hop correction replaces the previous one relative to nominal; documented
   - Ordering enforcement (AC#3): accept_fold_token() to transition AWAITING_FOLD -> FOLDED_READY; apply only in FOLDED_READY; print motion requests refused unless APPLIED_VERIFIED
   - Sanity bound check (AC#2): reject large corrections, call rescan_fn, halt on second failure
   - Verification rescan (AC#4): after applying correction, call rescan_fn() -> LevelingResult, check residual; fail if outside tolerance
   - Per-hop JSON-lines record written to artifacts dir (AC#6)
   - AA-7 telemetry via optional SessionLogger
2. Add PrintFrameConfig fields to HardwareConfig in config.py inside AA-33 markers (Optional, default None)
3. Add config/hardware.yaml entry for print_frame block (Optional, unverified placeholders)
4. Write tests/test_print_frame.py covering all ACs (application, sanity-bound rejection/rescan, ordering enforcement, residual verification)
5. Run pytest tests/test_print_frame.py and confirm all pass
6. Finalize task per finalization guide
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation: robot/src/inchworm/print_frame.py (new module). PrintFrame class with 4-state machine (AWAITING_FOLD→FOLDED_READY→APPLIED_VERIFIED→HALTED). Absolute-replacement policy: each hop correction replaces previous one relative to nominal frame (AC#5). Sanity bound check with rescan + double-violation halt (AC#2). Verification rescan after apply (AC#4). Per-hop JSON-lines record to artifacts dir (AC#6). PrintFrameConfigSchema added to config.py under AA-33 markers; hardware.yaml updated with print_frame block. 48 new tests in tests/test_print_frame.py all pass. Full test suite: 1113 passed. AC#4 uses synthetic LevelingResults per task instructions ('synthetic verification counts as the test').
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented inchworm/print_frame.py (AA-33). PrintFrame state machine enforces fold→apply→verify→reset ordering. 4x4 correction transform (Z+rx+ry) from LevelingResult applied as absolute replacement of previous correction relative to nominal frame (documented policy). Sanity bound check triggers rescan; double violation → FrameCorrectionHalt. Verification rescan confirms residual within tolerance or halts. Per-hop JSON-lines records written to session artifacts dir. PrintFrameConfigSchema added to HardwareConfig; hardware.yaml updated. 48 tests pass; full suite 1113 passed. All ACs checked with test output evidence. AC#4 verified with synthetic LevelingResults per task instructions.
<!-- SECTION:FINAL_SUMMARY:END -->
