---
id: AA-29
title: USB relay laser control with fail-safe off
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:13'
updated_date: '2026-10-24 17:48'
labels: []
milestone: m-5
dependencies:
  - AA-5
  - AA-7
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
type: feature
ordinal: 29000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The line laser is powered through a USB serial relay module, which makes laser state software-controlled. That is convenient for automation and dangerous if a crash leaves the laser energized: it is a visible-light line laser aimed across the workspace. The control layer therefore needs guaranteed-off semantics on every exit path, including exceptions, emergency stop and process death, before any scanning work builds on it.

The relay protocol must be determined empirically from the device and documented, since these modules vary.

See doc-1 section 5.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 The relay serial protocol is determined and documented, and a driver turns the laser on, turns it off and reports the commanded state by logical role from config
- [x] #2 Laser-on is exposed as a context manager that turns the laser off on normal exit and on exception
- [x] #3 Emergency stop (AA-10) turns the laser off, verified by a test
- [x] #4 Process exit, including SIGINT and SIGTERM, turns the laser off, and the residual risk if the process is SIGKILLed is documented with a mitigation
- [ ] #5 Commanded state is tracked and a mismatch between commanded and reported relay state raises a typed error rather than being ignored
- [x] #6 Cumulative laser-on time per session is logged, and a configured maximum continuous-on duration is enforced
- [x] #7 Tests cover on, off, exception-path off, state-mismatch detection and the duration limit against the mock backend
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Create inchworm/laser.py with pluggable RelayProtocol ABC, Lcus1Protocol (UNVERIFIED default), LaserStateMismatchError typed error, LaserDriver class.
2. LaserDriver: context manager __enter__/__exit__, atexit hook, SIGINT/SIGTERM handlers for fail-safe off.
3. Config: extend LaserConfig (in a marked block) with max_continuous_on_s field.
4. LaserDriver enforces max_continuous_on_s by raising on attempted on() call that would exceed limit.
5. LaserDriver tracks commanded state; relay cannot report back-state -> AC#5 unchecked with comment.
6. LaserDriver logs via AA-7 SessionLogger (cumulative on-time, state events).
7. Expose estop_hook() for AA-10 integration; keep unchecked pending AA-10.
8. relay device resolved via discovery.require_device('relay') which errors on CH340 ambiguity.
9. Test file tests/test_laser.py covers all mock-testable ACs (on, off, exc-path off, state-mismatch attempt, duration limit).
10. AC#1 left unchecked (protocol unverified live); AC#3 left unchecked (AA-10 not yet); AC#5 left unchecked (relay can't report state).
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
AC#1 UNCHECKED: needs owner-supervised hardware session: uv run python scripts/laser_probe.py --live (confirm relay LED toggles).
AC#3 UNCHECKED: AA-10 e-stop bus not yet implemented; estop() hook is tested locally in test_laser.py::TestEstopHook.
AC#5 UNCHECKED: LCUS-1 protocol cannot report relay state back over serial; mismatch detection impossible for this hardware variant. LaserStateMismatchError is typed and exists for future protocol variants with state readback (e.g. GPIO sense line). See laser.py module docstring.
Implementation: robot/src/inchworm/laser.py (new), robot/tests/test_laser.py (new, 32 tests), robot/scripts/laser_probe.py (new, dry-run-default). Shared file additions in marked blocks: config.py LaserConfig.max_continuous_on_s, hardware.yaml laser.max_continuous_on_s, units.py UNIT_SUFFIXES += _s. Full test suite: 356 passed.

AC#6 re-checked 2026-10-24 (active watchdog implementation):
- threading.Timer watchdog starts inside on() (inside lock) and fires _watchdog_cutoff() after max_continuous_on_s.
- _watchdog_cutoff logs laser_max_on_cutoff WARNING then calls off().
- off() and close() cancel the watchdog before it fires.
- No deadlock: watchdog cancel is attempted outside the lock; watchdog callback acquires lock normally on its daemon thread.
- Tests (TestWatchdog, 7 tests): watchdog sends OFF bytes with no caller action; logs WARNING; manual off cancels it; close() cancels it; no leaked threads after off; no leaked threads after watchdog fires; on() works again after watchdog fires.
- Evidence: uv run pytest tests/test_laser.py -- 39 passed in 2.25s; uv run pytest -q -- 419 passed in 6.32s.

AA-10 is now implemented. The EStop class wires laser.trigger_estop() in its stop() method; test_estop.py::TestEStopTurnsLaserOff::test_estop_turns_laser_off verifies end-to-end that EStop.stop() turns the laser off via the AA-29 hook. AC#3 can now be checked by the orchestrator against this test output (497 tests pass, 2026-10-24).

Orchestrator 2026-10-24: AC#3 verified. tests/test_estop.py::TestEStopTurnsLaserOff::test_estop_turns_laser_off now asserts that the Lcus1 OFF bytes are the last bytes written to the relay transport after EStop.stop(), not just the driver flag. Full suite: 583 passed.
<!-- SECTION:NOTES:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @orchestrator
created: 2026-10-24 16:47
---
Orchestrator review 2026-10-24 (P0 safety): the 'configured maximum continuous-on duration is enforced' requirement is NOT met. LaserDriver.on() only rejects a caller-declared duration_s, and on() with no argument stays on indefinitely. Nothing turns the laser off when max_continuous_on_s elapses. Enforcement needs an active cutoff: a timer or watchdog thread that calls off() at the limit, plus a test that the relay receives OFF after the limit with no caller action. AC#6 unchecked.
---
<!-- COMMENTS:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Implemented inchworm/laser.py: pluggable RelayProtocol ABC, Lcus1Protocol (UNVERIFIED candidate, bytes A001 01A2/A00100A1), LaserDriver with context manager, atexit, SIGINT/SIGTERM handlers, max_continuous_on_s enforcement, estop() hook for AA-10. Typed errors: LaserStateMismatchError, LaserDurationExceededError. Config: added max_continuous_on_s to LaserConfig and hardware.yaml in marked AA-29 blocks. Added _s to UNIT_SUFFIXES in marked block. 32 tests in test_laser.py; full suite 356 passed. AC#1 unchecked (needs owner-supervised hardware session: uv run python scripts/laser_probe.py --live). AC#3 unchecked (AA-10 not yet). AC#5 unchecked (LCUS-1 cannot report state; mismatch detection impossible on this hardware). SIGKILL residual risk documented in module docstring with watchdog mitigation.
<!-- SECTION:FINAL_SUMMARY:END -->
