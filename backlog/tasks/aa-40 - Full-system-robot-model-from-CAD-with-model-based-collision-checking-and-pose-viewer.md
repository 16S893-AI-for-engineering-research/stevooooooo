---
id: AA-40
title: >-
  Full-system robot model from CAD with model-based collision checking and pose
  viewer
status: In Progress
assignee:
  - '@orchestrator'
created_date: '2026-10-27 14:12'
updated_date: '2026-10-28 01:48'
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
- [x] #1 Each arm's base placement (position and orientation) in the chassis frame is extracted from the STEP file, cross-checked against the CAD bounding boxes and the 182.5 mm arm spacing, and recorded with its source
- [x] #2 A full-system model (URDF or MJCF) contains the chassis, DexArm, tube (position as a parameter) and both arms with collision geometry, and loads in the chosen library in this repo without ROS
- [x] #3 Collision checking uses part shapes for arm-arm, arm-chassis, arm-DexArm and arm-tube contact plus self-collision, with a configured margin, and is wired into motion_guard so it fails closed on any contact or a missing model
- [x] #4 The owner-measured rest and folded poses of both arms are collision-free in the model, and a known colliding pose is detected, verified by tests
- [x] #5 A pose viewer renders any logged or planned joint state of both arms, and a planned trajectory, for owner review, without per-pose manual setup
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
Stage 2 (orchestrator, 2026-10-27, on the owner cell URDF CAD/robot_urdf_new): 1. system_model/urdf.py: small URDF loader + FK (fixed/revolute/continuous/mimic), fail closed. 2. system_model/collision.py: python-fcl checker over every link; checked pairs = moving SO-101 links vs everything; allowed = joint-adjacent, owner SRDF, static-vs-static; tube_1 placed by a run-time advance along its fitted axis; table not modelled (owner). 3. Joint mapping: pan/lift/elbow/wrist_flex 1:1 (verified vs vendored chain); wrist_roll offset -127 deg (owner confirmed). 4. Owner 'go with the real fold': pose-scoped allowance for the pairs the model reports at the measured fold. 5. MotionGuard: additive MeshCollisionGuard; live fails closed on missing model, unknown other-arm pose, unstated tube or DexArm state, or unverified model. 6. scripts/view_pose.py on the cell URDF (presets, logged poses, trajectories, contacts in red). 7. tests/test_cell_collision.py.
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

2026-10-27 orchestrator: stage 1 committed 75505dc (system_model package: fitted placements, scene assembly, view_pose.py PNG/meshcat, tests). Model deps (trimesh, scipy, networkx, matplotlib) added to the dev group so a bare uv run pytest runs them; previously a partial env made view_pose dry-run and test_wrist_frame_bookkeeping fail. Full suite 1551 passed. Self-check: wrist export roundtrip 0.000 mm, wrist FK offset 3.14 mm (the CAD idealised fold, see the notes above). meshcat itself is not installed yet (PNG path via matplotlib works; pyglet missing so the trimesh renderer falls back). Stage 2 (FCL collision + motion_guard wiring) next.

2026-10-27 stage 2, first worker: timed out after 30 min, exploring only (no files changed). Findings, the DexArm one verified by the orchestrator: (1) assets.load_dexarm_meshes applies URDF visual origins without the joint-origin chain, so moving DexArm links sit 130-300 mm off, inside the SO-101 region (stage-1 In_1 bbox Y[-66,115] mm vs raw assembly-frame STL Y[-193,-12]; the CAD report puts the DexArm at Y[-215,0]). The stage-1 renders are therefore wrong for the DexArm. (2) The tube cylinder is 81 mm long; CAD Part 13 is 185.7 mm along Y. (3) bases.stl duplicates both SO-101 bases (and the DexArm base). (4) The stored pan q_rad for the fold in poses_2026-10-25.jsonl predates the pan landmark zero; recomputed from ticks, the left fold pan is about +3.3 deg, not 35.9. (5) Possible real contacts at the measured fold (self pairs at the end stops; left upper_arm vs right wrist about 9 mm): to be quantified after the fixes, never allowed blindly. Re-delegated with these as fixed facts (90 min limit, checkpoint 10 min before).

2026-10-27: the second stage-2 worker was stopped by the owner after about 45 min of exploration with no deliverable written (worktree removed, nothing merged). Owner decision: the owner will provide a better, simplified gripper STL, and the orchestrator will do stage 2 itself, not via a sub-agent. Stage 2 waits for that STL. The findings noted above (DexArm FK, tube length, duplicate bases, stale pan q) still apply.

Owner 2026-10-27 (session 3): (1) the owner supplied a complete cell URDF at CAD/robot_urdf/ (three_arm_cell.urdf with relative mesh paths; both SO-101s with the new gripper and camera mounts, DexArm with the laser camera, base mount baseforreal, SRDF of always-in-contact pairs). It REPLACES the stage-1 STL-fit assembly as the model. Adding joint limits is allowed; nothing else in it is to be changed. The lock servo has no joint (the lock is rigid with the gripper); the DexArm attachment is an approximation of the print head and is accepted as is. (2) The TABLE is NOT a collision object: the arms never need to go near it (owner decision; this replaces the earlier note that the table must be a collision object). (3) The owner will supply the tube as a separate URDF. Its placement is not fixed: the arms move it, so the model places it from a run-time parameter, using the attachment points spaced at the doc-1 pitch of 183.676 mm. Diagnosis of the stalls: workers 1 and 2 explored without writing; the orchestrator's third attempt crashed on a tool call with no result (a session API error) while cross-checking the URDF against the stage-1 model, which was the wrong direction anyway.

2026-10-27 stage 2 DONE on the owner cell URDF (CAD/robot_urdf_new, with the tube). Owner answers: (1) go with the real fold; (2) wrist roll: our 0 = cell -127 deg (confirmed); tube = 3 sections, one attachment point each, each printed section adds one; the tube frame is not at an attachment point, but in the URDF placement the attachment points line up with the grippers.
Evidence (tests/test_cell_collision.py, 37 tests; full suite 1617 passed):
- AC1: base placements come from the owner URDF (world_to_left/right joints); spacing test 182.5 mm +-0.5 passes; owner joints match the vendored chain to <0.01 mm at 4 poses (sign +1, offset 0, pan..wrist_flex).
- AC2: URDF loads with our loader (no ROS): chassis (baseforreal), DexArm, tube_1 (advance parameter, axis fitted from the 3-section periodicity, residual 0.000 mm, 1.1 deg off +Y), both arms + grippers + camera mounts.
- AC3: python-fcl on all link meshes; arm-arm, arm-chassis, arm-DexArm, arm-tube, self tests pass; margin 5 mm (motion_guard.collision.mesh.margin_m, unverified default); wired into MotionGuard.check_pose/check_trajectory; missing URDF, missing mesh, NaN joints, unknown other-arm pose (live), unstated tube/DexArm state (live), unconfigured mesh layer (live) all fail closed (tests).
- AC4: measured fold of both arms is collision-free with the tube in place (0 and +1 pitch); known colliding poses detected; an interior-only collision in a pan sweep detected. NOTE: the measured fold is clean ONLY via the owner-approved pose-scoped fold allowance (11 pairs, radius 0.05 rad per joint around each arm's measured fold; hardware.yaml). Without it the model reports lower_arm-vs-shoulder (elbow folds ~6 deg past the model's contact at ~91 deg) and left upper/lower arm vs right wrist/gripper (~9-10 mm overlap). Outside the radius the pairs are checked (test).
- AC5: scripts/view_pose.py renders presets, a pose-log label (recomputed from ticks), --q-left/--q-right, and a JSONL trajectory, with contacts in red; test renders a 3-sample trajectory.
- Timing: ~45 ms per pose per arm; meshes load once per process (~11 s).
Deviation: the named folded_transit placeholder in hardware.yaml [0,-1.25,1.64,-1.31,1.36] is NOT inside the fold allowance (lift 7.6 deg from the left measured fold) and is reported in contact; the fold waypoints need re-deriving from the measured fold (follow-up). Dry-run checks skip the mesh layer (with a warning) when the caller gives no other-arm pose; live never does. Callers (fold, cycle, approach, jog) do not yet wire other-arm/tube/DexArm sources, so LIVE motion through them refuses until wired (fail closed; mesh layer also unverified until AC6).
Open: AC6 (owner compares rest + fold renders vs the real robot). Render with: cd robot && uv run python scripts/view_pose.py --presets --tube-pitches 0
<!-- SECTION:NOTES:END -->
