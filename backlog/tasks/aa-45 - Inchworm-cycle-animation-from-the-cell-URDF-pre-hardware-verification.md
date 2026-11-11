---
id: AA-45
title: Inchworm cycle animation from the cell URDF (pre-hardware verification)
status: In Progress
assignee:
  - '@agent'
created_date: '2026-11-10 20:28'
updated_date: '2026-11-11 17:59'
labels:
  - simulation
  - verification
milestone: m-2
dependencies: []
priority: high
type: feature
ordinal: 45000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The owner cannot trust the agents' text descriptions of physical motion; rendered images from the cell URDF are what confirm behaviour (devlog 2026-11-10). Before any live crawl, the owner wants to watch the proposed inchworm motion animated on the real cell model (CAD/robot_urdf_new). Today only the approach phase produces joint trajectories; the advance is bookkeeping only (FrameModel.complete_advance) and no code describes how the arms physically carry the tube. Owner-specified cycle (2026-11-10): start with both arms locked in the printing pose; both arms (locked) reposition the tube along its axis away from the DexArm until the next attachment point is reachable by the left arm; the left arm (nearest the DexArm) unlocks and regrips the attachment point one pitch closer to the DexArm; the right arm then unlocks and regrips the point the left arm just left; extra both-arm repositions are allowed between regrips if a target is out of reach; only after both regrips do both arms fold back to the printing pose (net tube advance one pitch, 183.676 mm, away from the DexArm). Insertion is assumed perfect (straight-line pre-insertion to seated), standing in for the learned policy. No printing or laser. The printing pose is the hardware.yaml placeholder until the owner re-provides it. Single side view: DexArm, left arm, right arm left to right, camera on the +x (cable) side; owner confirms the camera side from one test frame.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A planner produces a timeline (JSONL) of both arms' joint angles, gripper lock states and tube pose for the owner-specified cycle, using the existing approach planner, frame model, joint limits and mesh collision guard
- [ ] #2 During every both-arms-locked segment, the two grippers keep a constant relative pose along the tube (rigid-tube constraint), checked per frame within a stated tolerance
- [ ] #3 Every frame is checked for mesh collision, joint limits and the gripper invariants (never both unlocked with the tube attached, at most one unlocked); any violation fails the run with the frame and reason, and tests show a colliding frame and a both-unlocked frame each fail
- [ ] #4 Any extra reposition the planner inserts is logged with its reason; if an owner-specified regrip target does not exist on the modelled tube the run stops and reports instead of inventing one
- [ ] #5 An MP4 or GIF renders the cycle in one side view (DexArm, left, right from left to right, +x cable side) with the step name, per-gripper lock state and tube advance overlaid, and contacting parts drawn red; the placeholder printing pose is labelled as such
- [ ] #6 The owner has watched the animation and confirmed the motion matches the intended inchworm cycle
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

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
2026-11-10 (worker run 8dcd014c, paused by owner; partial code lost with its worktree, nothing merged).
FINDING (cell model, not hardware): the placeholder printing pose folded_transit q=[0,-1.25,1.64,-1.31,1.36] at tube advance 0 does NOT put the grippers on the attachment points: each tip ~25 mm from the tube and ~74 mm along Y from the attachment features (left tip y=0.069 vs feature y~0.143; right tip y=0.252 vs y~0.327); gripper z axis ~37 deg off the working-pose grip. The owner working pose (-90/90/-90) lines up: left on the middle point (y~0.143), right on the far point (y~0.327); the point nearest the DexArm (y~-0.040) is free = left arm regrip target.
Camera for the side view: at +X looking -X, DexArm at -Y on the left (test frame rendered, not yet shown to the owner).
DECISION (orchestrator, 2026-11-10): attachment frames = mesh features (working-pose grips, one pitch along tube_axis). Start/fold-back state = IK solution with left on the middle point and right on the far point, seeded from folded_transit, labelled 'printing pose (IK, seeded from placeholder)'. Do NOT write it to hardware.yaml. report.txt leads with the PLACEHOLDER finding and the stored vs IK joint values. If IK cannot reach, stop and report. The printing pose is a single input so the owner's re-measured pose drops in later.
NEXT: re-launch one worker with the original brief plus this decision; first deliverable = test_frame.png shown to the owner.

2026-11-11 worker run 890df617 (opus-4-8, 30 min cap) timed out while writing tests; orchestrator rescued the patch, fixed exists_on_tube (was checked at advance=+1 pitch, so the absent-target test could not fail; now checked in the tube frame), moved the pitch literal out of the docstring (test_constants), smoke-test the script with --help only (full plan ~1 min, exits 1 while violations exist). Commit 91409d1. Suite passes. Outputs (local, gitignored): robot/out/inchworm/{test_frame.png,timeline.jsonl,report.txt,cycle.gif}. RESULT: IK printing pose reachable (L [0,-1.573,1.571,-1.569,0], R [-0.062,-1.604,1.564,-1.531,0.062] rad, residual ~0.07 mm) but it is NOT collision-clean (lower_arm within ~3 mm of shoulder at 5 mm margin), and the both-locked carry breaks the rigid-tube constraint (drift 3.9 mm at 0.25 pitch to 84 mm at 1 pitch) and intersects lower_arm/shoulder: 181 violations over 53 frames. So the owner cycle is not achievable from this placeholder pose with fixed grips; the planner reports this and does not hide it. AC status: #1-#4 have code + tests (tests/test_inchworm_plan.py, 7 pass) but are not checked: they need owner review of the report. #5 GIF exists (14 frames, every 4th). #6 owner. NEXT: owner looks at test_frame.png (camera side) and cycle.gif; owner provides the real printing pose; a both-locked carry probably needs the arms to move along the tube axis, which the current IK tracking does not model well.

2026-11-11 OWNER REVIEW of cycle.gif + refined cycle (owner words, recorded here; description NOT edited): the tube is not limited to moving straight back; first move the tube AWAY from both robots (up in the frame), then translate it right (away from the DexArm). The tube only moves because the arms move it. Then left regrip, then right regrip, then move the tube down and the arms return to their start pose. Joint 1 should not be needed; the elbow must be used.
REWORK (orchestrator): plan_cycle now = start -> lift (both locked, chassis +Z) -> carry right (both locked, chassis +Y, default half a pitch) -> left regrip (retract down by approach.standoff_m, move under the near point, insert, relock) -> right regrip (same, onto the point the left just left) -> lower + return (both locked, ends exactly one pitch along the tube axis). IK = planar joints 2-4 solve, then weighted DLS (pan/roll weight 50). CellState gained tube_transform (rigid 4x4, checked) so the lifted tube is drawn and collision-checked where it is. Rigid-tube reference now follows the current grip pair (the old fixed reference falsely flagged the relock after the left regrip). Report adds JOINT USE. Lift default 170 mm, from a model sweep (lift 130-190 x carry 70-110 mm; fewest violations). Tested: a 1 deg tube yaw to remove the CAD axis tilt (worse, reverted).
RESULT (placeholder printing pose): 87 frames, 11 violations. Joint use max deg: left pan 2.5, lift 81, elbow 118, wrist_flex 78, roll 2.5; right pan 13.8, lift 83, elbow 119, wrist_flex 77, roll 13.8. Remaining: (a) start/end pose lower_arm within ~3 mm of shoulder (placeholder pose, 5 mm margin); (b) carry frames 18-20 right gripper misses by up to 3.6 mm (and rigid drift same) where the right gripper passes near its own pan axis: the CAD tube axis is ~1 deg off the arm plane so the far point is ~3 mm out of plane and needs pan; (c) right regrip frame 58 misses 2.9 mm. Nothing relaxed. Suite passes. NEXT: owner watches the new cycle.gif; the real printing pose and a CAD check of the ~1 deg tube-axis tilt / 1.2 mm grip spacing mismatch (grips 182.4 mm apart at the working pose vs pitch 183.676) should clear (b) and (c).

2026-11-11 owner on the reworked cycle.gif: 'this is really good', 'the animation looks great'. Owner decision: skip the ~1 deg CAD tube-tilt error for now (the right-gripper reach misses of up to 3.6 mm stay reported, not fixed). AC #6 NOT checked yet: awaiting the owner's explicit confirmation that the motion matches the intended cycle. Owner asked what the real printing pose means (explained; it is still the IK placeholder).
<!-- SECTION:NOTES:END -->
