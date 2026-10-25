# Raw hardware measurements (committed evidence)

- `joint_ranges_left_2026-10-24.json`: left SO-101, owner-supervised torque-off manual sweep,
  read-only `scripts/record_joint_ranges.py --live` (wrap-aware). Raw servo units.
  Servo EEPROM at the time: min/max position limit 0/4095 on all 6, homing offset raw
  0x55 (IDs 1-3) / 0 (IDs 4-6), mode 0. Shoulder_pan, elbow_flex and wrist_roll travel
  CROSSES the 4095->0 seam, so they must not be position-commanded until homing
  offsets are recalibrated.

## Left arm servo EEPROM calibration, 2026-10-24 (owner-approved and supervised)

LeRobot-style half-turn homing (lerobot motors_bus.set_half_turn_homings /
record_calibration): the owner held every joint mid-travel, and each servo got a
homing offset so that pose reads 2047 (STS3215: reported = actual - offset,
sign-magnitude bit 11). Then a read-only re-sweep and per-servo
Min/Max_Position_Limit = recorded range +/- 30-tick margin. Torque stayed OFF
throughout; every write was read back; EEPROM lock reopened only for each write and
re-closed.

- `eeprom_backup_left_2026-10-24.json`: PRE-change values (restore to undo).
- `joint_ranges_left_homed_2026-10-24.json`: post-homing sweep (no seam crossing).
- `eeprom_limits_left_2026-10-24.json`: limits written and verified.

Homing offsets written: id1 -1352, id2 384, id3 1095, id4 -420, id5 1092, id6 1573.

| joint | servo limits (ticks) | servo deg |
|---|---|---|
| shoulder_pan | 674..2527 | 59.2..222.1 |
| shoulder_lift | 1255..3000 | 110.3..263.7 |
| elbow_flex | 939..3091 | 82.5..271.7 |
| wrist_flex | 929..3187 | 81.7..280.1 |
| wrist_roll | 500..3290 | 43.9..289.2 |
| gripper | 1558..2487 | 136.9..218.6 |

**Owner note:** these are the INDEPENDENT per-joint maxima. The safe envelope
depends on all joints together (self, chassis, other arm, DexArm and tube
collisions). The servo limits are only an outer box; the AA-9 MotionGuard collision
model enforces combined-joint safety, and its keep-out geometry must be built and
validated before any motion near those limits.

## Right arm servo EEPROM calibration, 2026-10-24 (owner-approved and supervised)

Same procedure as the left arm. Homing offsets: id1 -312, id2 397, id3 -99, id4 269,
id5 -1063, id6 611 (each read 2046-2048 after writing). Backup:
`eeprom_backup_right_2026-10-24.json`. Two read-only sweeps (pass 1 missed the
shoulder pan); the merged union is `joint_ranges_right_homed_2026-10-24.json`.

The measured right shoulder_pan travel was 758-3477 (239 deg) against 168 deg on the left.
**Owner decision (b):** cap the right shoulder_pan to the left arm's limit span (1853
ticks) centred on its measured midpoint: 1191..3044. Left shoulder_pan was re-swept to
confirm (654-2565, 168 deg, repeatable), so its limit 674..2527 was kept.

| joint | servo limits (ticks) | servo deg |
|---|---|---|
| shoulder_pan | 1191..3044 | 104.7..267.5 |
| shoulder_lift | 1323..2935 | 116.3..258.0 |
| elbow_flex | 906..3046 | 79.6..267.7 |
| wrist_flex | 899..3167 | 79.0..278.3 |
| wrist_roll | 554..3434 | 48.7..301.8 |
| gripper | 1573..2479 | 138.3..217.9 |

## Joint direction test, left arm, 2026-10-24 (read-only)

Owner convention: view each joint along its rotation axis from the side where the
servo's cable exits AWAY from the viewer (from above for the base; from the robot's
left side for the pitch joints; the robot front faces the DexArm, which sits on the
structure's left). Results (`direction_test_left_2026-10-24.jsonl`): shoulder_pan CW
+255 ticks, shoulder_lift CW +343, elbow_flex CCW -329, wrist_flex no movement
recorded. Conclusion: **higher servo reading = clockwise** in that view, consistent
with all 12 servos having the same phase register (0x0C). Mapping to the URDF +axis
per joint is still pending (needs the URDF/CAD mounting orientation).

Joint zero offsets (servo deg) written to hardware.yaml servo_calibration = midpoint
of each EEPROM limit range (URDF so101_new_calib zero = joint mid-range). Signs are
ASSUMED +1 and the block remains `unverified: true`.
