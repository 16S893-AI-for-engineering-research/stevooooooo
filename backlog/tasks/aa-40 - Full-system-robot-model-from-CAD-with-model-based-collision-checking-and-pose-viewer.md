---
id: AA-40
title: >-
  Full-system robot model from CAD with model-based collision checking and pose
  viewer
status: To Do
assignee: []
created_date: '2026-10-27 14:12'
updated_date: '2026-10-27 14:13'
labels: []
dependencies: []
priority: high
type: feature
ordinal: 40000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Build a full-system kinematic and geometric model of the as-built robot from CAD/Assembly 1.step. The owner confirmed on 2026-10-25 that the CAD matches the build and that the arms are fixed parts in Onshape. The model covers the chassis, DexArm, tube (with position as a parameter) and both SO-101 arms, using the vendored SO-101 URDF meshes placed at the CAD base placements. Parse the placements from the STEP text, since the OCP placement reader segfaults on this file. Use it for (1) collision checking between part shapes, replacing the 6-point vs keep-out-box check in motion_guard, which misses link bodies between points, and (2) an owner-facing pose viewer that is cheap to reuse for many poses (owner preference: cheapest and easiest long term). No ROS: decided with the owner 2026-10-25 because the open problems are geometric. The model is never the sole safety layer: servo EEPROM limits and soft limits stay independent.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 Each arm's base placement (position and orientation) in the chassis frame is extracted from the STEP file, cross-checked against the CAD bounding boxes and the 182.5 mm arm spacing, and recorded with its source
- [ ] #2 A full-system model (URDF or MJCF) contains the chassis, DexArm, tube (position as a parameter) and both arms with collision geometry, and loads in the chosen library in this repo without ROS
- [ ] #3 Collision checking uses part shapes for arm-arm, arm-chassis, arm-DexArm and arm-tube contact plus self-collision, with a configured margin, and is wired into motion_guard so it fails closed on any contact or a missing model
- [ ] #4 The owner-measured rest and folded poses of both arms are collision-free in the model, and a known colliding pose is detected, verified by tests
- [ ] #5 A pose viewer renders any logged or planned joint state of both arms, and a planned trajectory, for owner review, without per-pose manual setup
- [ ] #6 The owner compares at least two rendered poses (rest, fold) against the real robot and confirms they match, with the result recorded in task notes
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 Acceptance criteria are each verified by concrete evidence (test output, measurement, or a logged hardware run) and not by code inspection or intent
- [ ] #2 Acceptance criteria and task description were NOT altered to match the implementation; any scope or criterion change was approved by the owner first and is recorded in the task
- [ ] #3 Hardware facts used came from doc-1; any corrected measurement was written back to doc-1 via `backlog doc update doc-1` in the same task
- [ ] #4 Tests pass via `uv run pytest` from robot/, including mock-backend tests that run with no hardware attached
- [ ] #5 Safety invariants still hold and are covered by a test: single-camera USB exclusivity, always-one-arm-holding, gripper servo commanded only by the AA-17 state machine, laser off on every exit path
- [ ] #6 Implementation notes record measured values, deviations from plan, and anything deferred
<!-- DOD:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Tooling decision (orchestrator 2026-10-25): python-fcl + trimesh for collision, since FCL handles concave meshes and MuJoCo would reduce meshes to convex hulls, making the chassis and DexArm falsely large. meshcat for the viewer: a browser viewer driven from Python with a persistent server, so new poses need no setup; the Windows browser reaches WSL localhost. Offscreen PNG snapshots let the orchestrator self-check.
Stage 1 (worker): STEP text parser for placements; per-part meshes in the chassis frame (STEP tessellation of the non-arm parts; vendored SO-101 meshes for the arms); model assembly; meshcat viewer + PNG snapshot; render the rest and fold. Stage 2: FCL collision module, motion_guard integration, tests.
<!-- SECTION:PLAN:END -->
