---
id: AA-20
title: Code-policy-code handoff contract and supervisor skeleton
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-24 18:21'
labels: []
milestone: m-2
dependencies:
  - AA-18
  - AA-17
  - AA-7
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 20000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The system must switch from scripted motion, to the learned insertion policy, and back to scripted motion with no operator intervention. That boundary is the riskiest part of the design: an unbounded policy can drive the arm into the structure, and a policy allowed near the gripper servo could trip the shared latch. Defining the contract and its supervisor before the policy exists means the policy can be developed and swapped behind a stable, already-tested safety boundary.

Build the supervisor with a stub policy (for example, a scripted nudge or a no-op) so all guards, timeouts, abort paths and retries are testable before any model is trained.

See doc-1 section 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A documented contract specifies the observation provided to the policy, the action space it may command, the pose at which control is handed over, the success and abort conditions, and what code guarantees on either side
- [ ] #2 The action space excludes the gripper servo entirely, and the supervisor rejects or filters any action touching it
- [ ] #3 The supervisor enforces a hard step and wall-clock limit, a Cartesian workspace bound around the target attachment point, and per-step action magnitude clamping, aborting on violation
- [ ] #4 On abort the supervisor retracts the arm to a safe pose, logs a typed abort event with the triggering condition, and returns control to code without operator input
- [ ] #5 A retry policy with a bounded attempt count and a documented escalation (re-approach, then halt the cycle) is implemented and logged
- [ ] #6 A stub policy implementing the contract allows the full handoff path to be exercised end to end against the mock backend with no trained model present
- [ ] #7 Tests cover handover, success return, each abort trigger, gripper-action rejection, retry escalation and final halt
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete. New modules: policy_contract.py, supervisor.py, stub_policy.py, tests/test_supervisor.py. PolicySupervisorConfig added to config.py and hardware.yaml under AA-20 markers. 45 new tests, 812 total pass. All software ACs verified by test output. AC#1-#7 all covered. No hardware ACs exist for this task.

Orchestrator review 2026-10-24 hardening: (1) the workspace bound used bare np.deg2rad(servo_deg), which ignores the AA-9 per-joint calibration and made the Cartesian bound meaningless; live mode now REQUIRES an injected joint_deg_to_rad (ValueError otherwise). (2) Seated but lock failed: new AbortReason.LOCK_FAILED, HALT without retract or retry, because the gripper may be partly engaged. (3) An exception from the policy, frame source or executor previously escaped mid-attempt; it now retracts and aborts with STEP_EXCEPTION. (4) Retract-unsafe errors are typed RetractUnsafeError and always propagate. Tests: test_live_supervisor_requires_calibrated_joint_conversion, test_lock_failure_after_seating_halts_without_retract, test_policy_exception_retracts_and_aborts. 902 passed.
<!-- SECTION:NOTES:END -->
