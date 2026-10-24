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
