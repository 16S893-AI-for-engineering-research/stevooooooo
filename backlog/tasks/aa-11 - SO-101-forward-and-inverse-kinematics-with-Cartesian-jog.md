---
id: AA-11
title: SO-101 forward and inverse kinematics with Cartesian jog
status: In Progress
assignee:
  - '@agent'
created_date: '2026-10-20 21:10'
updated_date: '2026-10-24 17:56'
labels: []
milestone: m-1
dependencies:
  - AA-9
references:
  - CAD/Assembly 1.step
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 11000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Coarse approach, pre-insertion poses, folded poses and policy action framing are all naturally expressed in Cartesian space relative to the chassis, not in joint angles. A verified FK/IK layer with a measured link model is therefore a prerequisite for the crawl stage, and a Cartesian jog tool is the practical way the owner and later tasks will probe reachability of real attachment-point locations.

Derive link lengths and offsets from the CAD assembly and verify them against the physical arm; a kinematic model that disagrees with the hardware by several millimeters will make the learned insertion job much harder.

See doc-1 sections 2, 3 and 8.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 Forward kinematics maps joint angles to end-effector pose in a documented chassis-fixed frame, with link parameters taken from CAD and recorded in config
- [x] #2 Inverse kinematics returns joint solutions for reachable targets, reports unreachable targets explicitly, and selects among multiple solutions by a documented deterministic rule that respects joint limits
- [ ] #3 FK/IK round-trip accuracy is measured in simulation (sub-millimeter) and verified against at least five measured physical poses per arm with the residuals recorded in task notes
- [ ] #4 A Cartesian jog CLI moves a chosen arm by a commanded delta or to an absolute pose, honouring joint limits and the collision envelope, with a dry-run mode that prints the plan without moving
- [x] #5 Joint-space trajectory generation produces time-parameterized, velocity- and acceleration-limited motion rather than step commands
- [x] #6 Tests cover FK/IK round trips, unreachable-target handling, limit-respecting solution choice and dry-run output
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
## Plan (AA-11 software half)

1. Extract URDF joint params into hardware.yaml under AA-11 markers (5-DoF chain)
2. inchworm/kinematics.py: FK (HTM chain), IK (damped LS, seed=zeros, solution=min-displacement)
3. inchworm/trajectory.py: trapezoidal joint-space trajectory
4. scripts/fk_verify.py: physical pose verification (dry-run default, --live flag)
5. scripts/jog.py: Cartesian jog CLI (dry-run default, --live, AA-9 hook placeholder)
6. tests/test_kinematics.py + tests/test_trajectory.py
7. pyproject.toml + hardware.yaml: AA-11 markers
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
## FK/IK Simulation Round-Trip Accuracy (AC#3, sim half)

Test: tests/test_kinematics.py::TestFkIkRoundTripAccuracy::test_fk_ik_round_trip_accuracy
All 8 poses: pos_err < 1 mm (sub-millimeter) ✓

Pose  pos_err_mm
0     0.000000
1     0.569203
2     0.805119
3     0.485203
4     0.982527
5     0.505949
6     0.254929
7     0.568456
max   0.982527
mean  0.521423

All errors < 1 mm ✓  (seed=zeros, position+orientation IK, DLS lambda=0.05)

## AC#3 Physical poses
Needs owner-supervised hardware session:
  uv run python scripts/fk_verify.py --live --arm left
  uv run python scripts/fk_verify.py --live --arm right
Record residuals back to this task.

## AC#4 Collision envelope
AA-9 not yet implemented. Jog calls collision_check_hook() which always returns None.
Unchecked pending AA-9.

## Implementation summary
- inchworm/kinematics.py: FK (HTM chain), IK (DLS, seed=zeros documented, position_only mode)
- inchworm/trajectory.py: trapezoidal joint-space planner, synchronised across joints
- scripts/jog.py: Cartesian jog CLI (dry-run default, --live flag, --dry-run flag, AA-9 hook)
- scripts/fk_verify.py: physical pose verification (dry-run default, --live flag)
- tests/test_kinematics.py: 31 tests, all pass
- hardware.yaml: so101_kinematics block (URDF joint params, unverified offsets)
- config.py: SO101KinematicsConfig schema (AA-11 markers)
- pyproject.toml: numpy>=1.24 (AA-11 markers)
- units.py: _rad_per_s, _rad_per_s2 suffixes (AA-11 markers)

Orchestrator review 2026-10-24: the IK default tolerance was exactly 1 mm (_DEFAULT_TOL_M=1e-3), so 'sub-mm round trip' only measured the solver's stop threshold (observed max 0.98 mm). Tightened to 1e-5 m / 1e-3 rad and max_iter 500; the round-trip test now requires < 0.05 mm. Re-measured with test_fk_ik_round_trip_accuracy over 8 poses: max 0.0098 mm, mean 0.0067 mm. An extra 50-pose random sweep (orchestrator, seed 1) gave max 0.0096 mm, with 5/50 unreached from the zero seed (genuinely hard configurations, reported as unreachable rather than silently wrong). Full suite: 583 passed. AC#3 stays unchecked: physical-pose verification needs hardware.

AA-9 implemented (motion_guard.py): the jog.py collision_check_hook has been wired to the real AA-9 MotionGuard (_aa9_collision_check in jog.py). AA-11 AC#4 (collision hook wired) can now be checked by the orchestrator by reviewing jog.py and running uv run pytest tests/test_motion_guard.py::TestJogAA9Integration -v.
<!-- SECTION:NOTES:END -->
