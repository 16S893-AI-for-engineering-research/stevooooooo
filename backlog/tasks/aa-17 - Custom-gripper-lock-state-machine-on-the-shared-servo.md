---
id: AA-17
title: Custom gripper lock state machine on the shared servo
status: To Do
assignee: []
created_date: '2026-10-20 21:11'
labels: []
milestone: m-2
dependencies:
  - AA-8
  - AA-10
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 17000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner replaced the stock SO-101 gripper with a custom design in which the latch and the jaw are driven by the same servo ID. Lock and unlock are therefore position bands of one servo rather than separate actuators, which means an ordinary gripper close command could partially engage the latch, and a learned policy could trip the latch at the wrong moment. The owner has specified that locking must be commanded by code and never delegated to the model.

This task owns the single authoritative interface for that servo, including the measured position bands, the verification that the latch is actually holding, and the guarantee that the lock holds until the next deliberate regrip. Everything else in the system must go through it.

See doc-1 sections 3 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The measured servo position bands for unlocked, engaged-but-unlocked and locked states are determined empirically and stored in the hardware config with the measurement method noted
- [ ] #2 An explicit state machine exposes unlock, approach-ready, lock and hold transitions, rejects illegal transitions, and is the only code path permitted to command the gripper servo
- [ ] #3 Locking verifies success from servo position plus current or load, distinguishing a true latch from a jam or a miss, and reports a typed failure instead of assuming success
- [ ] #4 Once locked the state is actively held until an explicit unlock, including re-asserting position if drift beyond a configured threshold is detected, and every hold correction is logged
- [ ] #5 The interface is unavailable to or hard-blocked from the learned policy action path, verified by a test that the policy runner cannot command the gripper servo
- [ ] #6 Unlocking under load is handled safely (documented behaviour when the arm is still bearing the tube) and never commanded while the opposite arm is not holding
- [ ] #7 Tests cover band logic, illegal transitions, lock verification success and failure, drift re-assertion and the policy-blocked path
<!-- AC:END -->
