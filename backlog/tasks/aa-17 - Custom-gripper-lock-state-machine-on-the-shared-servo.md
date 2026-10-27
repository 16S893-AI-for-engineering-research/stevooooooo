---
id: AA-17
title: Custom gripper lock state machine on the shared servo
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:11'
updated_date: '2026-10-27 20:25'
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
- [x] #2 An explicit state machine exposes unlock, approach-ready, lock and hold transitions, rejects illegal transitions, and is the only code path permitted to command the gripper servo
- [x] #3 Locking verifies success from servo position plus current or load, distinguishing a true latch from a jam or a miss, and reports a typed failure instead of assuming success
- [x] #4 Once locked the state is actively held until an explicit unlock, including re-asserting position if drift beyond a configured threshold is detected, and every hold correction is logged
- [x] #5 The interface is unavailable to or hard-blocked from the learned policy action path, verified by a test that the policy runner cannot command the gripper servo
- [x] #6 Unlocking under load is handled safely (documented behaviour when the arm is still bearing the tube) and never commanded while the opposite arm is not holding
- [x] #7 Tests cover band logic, illegal transitions, lock verification success and failure, drift re-assertion and the policy-blocked path
<!-- AC:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Plan executed. New modules: gripper_lock.py (state machine), policy_actions.py (policy adapter), gripper_bands.py (script). Shared-file additions: config.py + hardware.yaml under AA-17 markers. 81 new tests in test_gripper_lock.py. 664 total tests passing. AC#1 unchecked (hardware session required). ACs #2-7 covered by software tests.

Orchestrator review 2026-10-24: hold_tick previously swallowed read failures forever while still reporting LOCKED. It now escalates to FAULT and raises GripperLockError after 3 consecutive failures (gripper_hold_lost event). Tests: test_hold_tick_escalates_to_fault_after_repeated_read_failures, test_hold_tick_read_failure_counter_resets_on_success. Checked: unlock refuses without a passing opposite-arm predicate. 767 passed.

Orchestrator re-verification 2026-10-24 (the worker's AC checks were lost in the worktree patch merge, so they were re-run here): `uv run pytest tests/test_gripper_lock.py` -> 83 passed. TestStateMachineTransitions 16 (AC#2 transitions and illegal ones), TestLockVerification 13 (AC#3 latch/jam/miss), TestActiveHold 7 + the orchestrator hold-loss tests (AC#4), TestPolicyActions 15 + test_no_token_import_outside_aa17 in test_servo_bus.py (AC#5 policy path blocked), TestUnlockSafety 6 (AC#6 opposite-arm gate and under-load warning). AC#7 is the union. AC#1 (empirical bands) is hardware-only and stays unchecked.

Owner 2026-10-25: the gripper is a lock-and-key. The key inserts at the fully-open position, so 'engaged but unlocked' is the SAME servo position as 'unlocked/open'. Left samples (calibration_data/gripper_samples_2026-10-25.jsonl): open 1531, engaged_unlocked 1550, closed_empty 2521 ticks. CONSEQUENCE / OWNER QUESTION: position alone cannot tell unlocked from engaged-but-unlocked. The AC#1 three-band model may need another signal (load/current, or a camera) to detect key insertion. Raised with the owner; AC not changed.

Owner-supervised band measurement 2026-10-25 (read-only, torque off; calibration_data/gripper_samples_2026-10-25.jsonl, 40 samples each, all steady):
LEFT: open 1531, engaged_unlocked (key in) 1550, locked 1802/1802/1801 (3 full cycles), closed_empty 2521.
RIGHT: open 1576, locked 1792/1800/1801, closed_empty 2494.
Owner: lock-and-key; the key is inserted at the open position; locked with key stops short of closed_empty; key detection method = (a) position.
DESIGN ISSUE FOUND (needs owner decision, AC not changed): GripperLockSM._goal_pos_locked() commands the MIDPOINT of the locked band. Without a key, the servo would also reach that goal, so position could not detect a missing key. For (a) to work, the lock goal must lie BEYOND the key stop (e.g. about 1950-2000 ticks), so: key in = stalls at about 1800 (in band); no key = passes it (MISS). That needs a torque/current limit for the stall. Also: the measured open and closed positions lie outside the EEPROM limits (L 1558-2487, R 1573-2479); a limit update (EEPROM write) is needed before powered gripper use.

Overtravel lock merged 2026-10-25 (orchestrator-reviewed). Lock commands lock_overtravel_goal_deg after applying lock_torque_limit_raw; MISS_NO_KEY_OVERTRAVEL verdict; band-based hold (slip toward closed => HoldSlipFault). Bands filled from the measured samples, still unverified; lock_torque_limit_raw null => live lock refused. REVIEW FINDING, to fix before the powered test: lock() reads back the position immediately after commanding, without waiting for motion or a stall to settle. With a ~600-tick overtravel move the readback would be the pre-move position => false MISS. It needs a bounded settle wait (until velocity is ~0 or a timeout) before verification.

2026-10-27 orchestrator: settle-wait fix for the review finding (lock() read back immediately after commanding the overtravel move -> false MISS). lock() now polls read_joint_state until two consecutive reads have |speed_raw| <= settle_speed_max_raw and a position change <= settle_position_tol_deg, bounded by lock_settle_timeout_s; timeout -> new LockVerdict.NOT_SETTLED -> FAULT (fail closed); read errors propagate and never leave LOCKED. New GripperLockConfig fields (defaults 2.0 s / 0.02 s / 5 raw / 0.3 deg, unverified). Tests: TestLockSettleWait (5): moving servo waited out and LATCHED, never-settles faults, settled no-key overtravel still detected, speed-0-but-jumping not trusted, read error never locks. tests/test_gripper_lock.py 114 passed. Owner (2026-10-27) offered to help with the torque-limit measurement: scripts/gripper_lock_probe.py (dry-run default; gripper commanded only via GripperLockSM; refuses if any arm joint has torque on; restores torque limit 1000 and torque off on exit; tests/test_gripper_lock_probe.py) and human task AA-41 proposed for the supervised run.
<!-- SECTION:NOTES:END -->
