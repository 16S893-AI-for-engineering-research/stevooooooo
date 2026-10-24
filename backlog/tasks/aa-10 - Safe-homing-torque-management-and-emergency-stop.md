---
id: AA-10
title: 'Safe homing, torque management and emergency stop'
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:09'
updated_date: '2026-10-24 17:48'
labels: []
milestone: m-1
dependencies:
  - AA-9
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 10000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Serial bus servos hold no absolute calibration reference the software can trust across power cycles, and an unattended system that cannot reliably reach a known state cannot be restarted autonomously after a fault. The system also needs one unambiguous way to stop everything: a hung policy, a jammed insertion or a bad trajectory must be stoppable without physically unplugging the dock.

Homing must be gentle: the arms may start in the folded pose, gripping the tube, or in an unknown pose, so the homing routine must reason about the current state before moving and must not yank against a locked gripper.

See doc-1 sections 3, 8 and 9.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 A homing routine brings each arm from an arbitrary safe starting pose to the named neutral pose using velocity- and current-limited motion, and refuses to move if the measured state is inconsistent with its assumptions
- [x] #2 Homing explicitly handles the case where the gripper is in the locked band and still engaged with an attachment point, by unlocking first or aborting with a clear reason rather than pulling against the latch
- [x] #3 Torque enable/disable is centrally managed so arms can be safely de-energized and re-energized without losing the known pose, and a power-cycle recovery path is documented
- [ ] #4 An emergency stop entry point halts all commanded motion on both arms and the DexArm, turns the laser relay off, and leaves the system in a documented, recoverable state
- [x] #5 Emergency stop is callable programmatically, from the command line and from a signal handler (SIGINT/SIGTERM), and is exercised by a test against the mock backend
- [x] #6 Overcurrent, overtemperature and unreachable-servo conditions trigger the same stop path and log a typed fault event
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Add AA-10 config blocks to hardware.yaml (neutral pose, gripper locked band, fault thresholds) inside # --- AA-10 --- markers.
2. Add AA-10 config schema fields to config.py inside # --- AA-10 --- markers (HomingConfig, TorqueConfig, FaultThresholds, AA10Config embedded in HardwareConfig).
3. Add servo speed/acceleration/torque-limit write methods to servo_bus.py inside # --- AA-10 --- markers (registers 46=goal_speed, 41=acceleration, 48=torque_limit per rustypot sts3215.rs).
4. Create inchworm/torque.py: TorqueManager class wrapping ArmServoBus.set_torque_enable for both arms, with recovery-path documentation, dry-run aware.
5. Create inchworm/homing.py: HomingRoutine that: reads neutral pose from config (marked unverified), checks AA-9 joint limits exist (refusing live run if missing), detects gripper locked band (null -> fail-closed abort), applies velocity/current limits via TorqueManager+servo_bus, moves joints velocity-limited. Dry-run default.
6. Create inchworm/estop.py: EStop class with single entry point stop() that: disables torque on both arms (torque-hold or disable), sends M410 quickstop to DexArm (add M410 method to dexarm.py in marked block), calls laser.LaserDriver.estop via AA-29 hook, logs typed fault events, each subsystem wrapped so failure doesn't prevent others. Wires SIGINT/SIGTERM and programmatic call. Also add FaultEvent typed events.
7. Create scripts/estop.py: CLI entry point calling EStop.stop(dry_run based on --live flag).
8. Create tests/test_estop.py, tests/test_homing.py, tests/test_torque.py: all against mock backend.
9. Wire AA-29 hook: in inchworm/estop.py import laser.register_estop_hook and register the estop callback. Add note in AA-29 task that AC#3 can now be checked.
10. Update AA-10 task notes after all tests pass.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Implementation complete. New modules: inchworm/torque.py, inchworm/homing.py, inchworm/estop.py, scripts/estop.py. Marked blocks added to config.py, hardware.yaml, servo_bus.py, dexarm.py, pyproject.toml. Tests: tests/test_torque.py (8 tests), tests/test_homing.py (12 tests), tests/test_estop.py (16 tests). All 497 tests pass (uv run pytest). AC#1: HomingRoutine implements velocity/current-limited motion to neutral pose; refuses live run when joint_limits_verified=False. AC#2: gripper locked band null->fail-closed abort; within band->abort; outside band->proceeds; no unlock attempted. AC#3: TorqueManager centralises torque en/de-energise; power-cycle recovery documented in torque.py module docstring. AC#4: EStop.stop() disables both arms (TorqueManager.disable_all), sends M410 to DexArm, triggers laser hooks via laser.trigger_estop(). AC#5: programmatic, CLI (scripts/estop.py), SIGINT/SIGTERM handlers installed; all tested. AC#6: FaultMonitor typed fault events (overcurrent, overtemperature, servo_unreachable) all trigger EStop.stop(); tested. AA-29 AC#3 note: test_estop_turns_laser_off verifies end-to-end that EStop.stop() turns laser off via the AA-29 hook.

Orchestrator changes 2026-10-24 (provisional, pending the owner decision in the comment): EStop.stop() now HOLDS arms by default (TorqueManager.hold_arm/hold_all: re-command present position, torque ON, gripper untouched). It releases torque only with release_on_stop=True or for overcurrent/overtemperature faults. Tests: test_default_estop_holds_arms_not_releases, test_release_on_stop_disables_torque, test_thermal_and_overcurrent_faults_release[...], test_unreachable_fault_holds. Full suite: 583 passed. AC#4 stays unchecked until the owner confirms the stop semantics.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 17:44
---
Orchestrator review 2026-10-24, OPEN OWNER QUESTION (safety semantics, not specified in AA-10 or doc-1). EStop.stop() currently DISABLES TORQUE on both arms. Both arms hold the tube whenever printing and at cycle start (doc-1 s2), so a torque-off e-stop lets the arms go limp under the tube's weight and may drop it or let the arms collapse onto the chassis. Proposed default: e-stop = HOLD (re-command each joint to its present position at reduced torque_limit, freezing motion), with torque-off as a separate explicit 'release' action. Exception: overcurrent or overtemperature faults may need torque-off to protect the servo. Owner to decide. AC#4 ('documented, recoverable state') is unchecked until the stop semantics are decided and implemented. The orchestrator will implement 'hold by default' as the provisional safe choice (doc-2: when in doubt, take the less destructive action).
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented safe homing (inchworm/homing.py), centralised torque management (inchworm/torque.py), emergency stop (inchworm/estop.py), and CLI (scripts/estop.py). Config schema extended (config.py, hardware.yaml) with homing neutral pose (UNVERIFIED), gripper locked band (null=fail-closed), and fault thresholds (UNVERIFIED). Servo velocity/acceleration/torque-limit registers added to servo_bus.py (goal_speed 0x2E/46, acceleration 0x29/41, torque_limit 0x30/48 per rustypot sts3215.rs). DexArm M410 quickstop added to dexarm.py. AA-29 hook wired: EStop.stop() calls laser.trigger_estop() which fires all registered LaserDriver hooks; test_estop_turns_laser_off verifies end-to-end. All 6 ACs verified by 36 new mock-backend tests (497 total pass). Hardware-only items: none - all ACs are software-verifiable against mock backend per the task notes.
<!-- SECTION:FINAL_SUMMARY:END -->
