# Tube Fixture Specification — AA-23

**Status: Owner deliverable.**
This document specifies what the fixture must reproduce.
Actual dimensions are NOT invented here; they are an owner measurement task (AC#4).

---

## Purpose

During teleop demonstration capture (AA-23) the tube (or a representative
attachment-point section) must be held stationary while one SO-101 arm is torque-off
(the leader).  A physical fixture supports the tube so that:

1. The tube cannot fall or shift when neither arm is holding it.
2. The **inserting arm's wrist camera sees the same scene** during demonstrations as
   it does during autonomous rollout (the real crawl pose).
3. The **follower arm's joint configuration at the pre-insertion pose** in the fixture
   matches the pre-insertion pose used during rollout (AA-18).

If conditions 2 and 3 are not met, the policy is trained outside the distribution it
encounters at rollout (distribution shift).

---

## What the fixture must reproduce

### Attachment-point pose (relative to the inserting arm)

The fixture must hold the tube attachment-point in the same position and orientation
relative to the inserting arm base frame that it occupies in the real crawl pose.
Specifically:

- **Position**: The attachment-point centre must be at the same `[x, y, z]` (mm) in
  the inserting arm's base frame as measured during the AA-16 calibration session.
  *These coordinates are an owner measurement; they are NOT listed here.*
- **Orientation**: The attachment-point bore axis must be aligned with the same
  direction as in the crawl pose, to within the AA-18 orientation tolerance
  (`pose_tolerance_rad` in hardware.yaml; current placeholder: 0.1 rad).

### Wrist-camera field of view

The follower wrist camera must see the attachment-point bore (and the surrounding
tube surface) at the same scale, position, and orientation in the frame as it does
during rollout.  This is verified by the AC#4 validation step:

> Compare wrist-camera frames captured in the fixture (at the AA-18 pre-insertion
> pose) against wrist-camera frames captured in the real crawl pose at the same
> pre-insertion pose definition.  Record the pixel-space difference.

Acceptable difference threshold: **TBD by owner after measurement.**

### Pre-insertion joint pose

The follower arm's joint configuration when it is at the pre-insertion pose in the
fixture must match the configuration it would be in during rollout.  This is
automatically satisfied if the attachment-point position and orientation conditions
above are met and the same `get_pre_insertion_pose()` function (AA-18) is used in
both cases.

---

## Physical interface constraints

The following constraints are known from doc-1 and must be reproduced by the fixture:

- **Attachment-point pitch**: 183.676 mm centre-to-centre along the tube axis (doc-1 s2).
- **Attachment-point type**: The fixture must hold at least one attachment-point section
  (or a representative mock with the same bore diameter and latch geometry) so that
  insertion demonstrations are representative.

> **Owner task**: measure the following and add them to this document:
> - Bore diameter (mm)
> - Latch geometry (depth, width, engagement angle)
> - Tube outer diameter (mm)
> - Distance from tube axis to arm base frame origin (mm)
> - Height of attachment-point centre above bench/mounting surface (mm)
> - Required fixture alignment tolerance (mm and deg)

---

## Validation procedure (AC#4)

1. Mount the fixture on the robot chassis or bench in the position derived from
   AA-16 calibration.
2. Move the follower arm to the AA-18 pre-insertion pose.
3. Capture 10 wrist-camera frames.
4. Move the robot to the real crawl pose (both arms holding the tube).
5. Capture 10 wrist-camera frames from the follower wrist camera.
6. Compare:
   - Mean pixel-space SSIM or L2 difference between fixture frames and crawl frames.
   - Follower joint configuration difference (degrees per joint).
7. Record the differences in the task comment for AA-23 AC#4.

Script:
```
cd robot && uv run python scripts/teleop.py --live --validate-fixture
```

This script is a hardware-only step that requires owner supervision.

---

## Safety notes

- The fixture must not obstruct the follower arm's motion envelope.
- The fixture must not be attached to the leader arm at any point.
- The leader arm (torque off) must be free to move by hand without contacting
  the fixture.
- All safety invariants in AGENTS.md remain in effect during fixture use.

---

*Last updated: 2026-10-24 (AA-23 software half; dimensions are owner deliverable)*
