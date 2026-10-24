# Raw hardware measurements (committed evidence)

- `joint_ranges_left_2026-10-24.json`: left SO-101, owner-supervised torque-off manual sweep,
  read-only `scripts/record_joint_ranges.py --live` (wrap-aware). Raw servo units.
  Servo EEPROM at the time: min/max position limit 0/4095 on all 6, homing offset raw
  0x55 (IDs 1-3) / 0 (IDs 4-6), mode 0. Shoulder_pan, elbow_flex and wrist_roll travel
  CROSSES the 4095->0 seam, so they must not be position-commanded until homing
  offsets are recalibrated.
