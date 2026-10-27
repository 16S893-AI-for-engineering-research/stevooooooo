---
id: AA-40
title: >-
  Full-system robot model from CAD with model-based collision checking and pose
  viewer
status: To Do
assignee: []
created_date: '2026-10-27 14:12'
updated_date: '2026-10-27 17:40'
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Owner measurements 2026-10-25 (tape): (a) arm base to arm base edge-to-edge is correct (CAD 182.5 mm). (b) Bottom of the DexArm BASE to the closest edge of the LEFT SO-101 BASE = 71.108 mm. The rest of the SO-101 overhangs, so it gets closer than that. (c) Table surface to the FAR side of the tube is about 360 mm. The tube runs PARALLEL to the table. Environment: no wires in the way; the SO-101s can reach the table surface (the table must be a collision object); the DexArm cannot reach the table. Left pan: NO LONGER obstructed, so its limits must be re-measured.

2026-10-25: approach changed with the owner. Assemble the model from open-source models (SO-101 URDF; DexArm URDF) plus owner-exported STLs, instead of classifying STEP parts by name (that failed: the DexArm guess was off by about 100 mm and 'Part 1' appears 7 times). DexArm model vendored: robot/vendor/dexarm (ybisk/robot, MIT, 2021, 'WIP' community model; links upper arm and forearm about 150-160 mm, base about 128x145 mm, plausible but unverified). Owner will export: (1) one STL with all arm-base pieces and the mounts they sit on, (2) one STL with the camera mount + Wrist_Roll_Pitch. The WIP branch wip/aa-40-stage1 keeps the STEP placement parser (works, 6 s).

Fits 2026-10-25 (robot/tools/cad/fit_bases.py, fit_all.py; results in calibration_data/aa40_*_2026-10-25.json). Owner exports are in METRES.
- SO-101 base_link onto the owner bases.stl: L xyz [-114.748, 235.474, 16.464] mm, rpy [0.0002, 0.0014, -1.5712]; R [-114.677, 417.987, 16.445], rpy [-0.0005, 0.0001, -1.5695]. RMS 0.03 mm both ways; spacing 182.51 mm.
- Wrist_Roll_Pitch in wrist_camera/wrist_gripper: RMS 0.05-0.06 mm (same pose in both files, so the files share a frame). The 32.8 cm3 body is a different part (owner's lock).
- DexArm B_Fix (community model) onto the export: the export lies on the model to 0.08 mm RMS; model-to-export 2.5 mm RMS (725/3000 model points 3-6 mm INSIDE the export, the same convex hull, so internal detail differs, e.g. ribs). Pose xyz [-114.494, 99.391, 81.744] mm, rpy [-1.5708, -0.0045, 3.1415].
- 71.108 check: Y gap between the DexArm export base (maxY 99.39) and the left SO-101 base (minY 170.78) = 71.386 mm vs the owner's 71.108 (0.28 mm).
- CAD-pose solve: fitting left FK to the CAD wrist part gives the CAD arm drawn at [-2.9, -90.4, 90.2, -89.4] deg, residual 3.1 mm / 2.9 deg. That is a square, idealised fold, NOT the real fold (measured [0, -79.2, 96.7, -74.8]), so the CAD cannot validate joint zeros; the real check is the owner comparing renders against photos.

Owner 2026-10-25: in wrist_gripper.stl, Part 10 = lock body and Part 9 = locking key (turns when the gripper locks). Body volumes in that export: lock body probably 32.8 cm3 (47x47x35 mm) at Z about 248 mm; key probably 9.96 cm3 (44x45x28) at Z about 274. The STL carries no names, so the identification is by size/position and needs owner confirmation from the render.
<!-- SECTION:NOTES:END -->
