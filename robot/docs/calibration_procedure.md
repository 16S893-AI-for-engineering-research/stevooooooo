# AA-16: Chassis–Arm–Tube Calibration Procedure

This document describes the repeatable procedure for calibrating all
physical-frame transforms on the inchworm robot.  It is the owner-facing
companion to `robot/src/inchworm/calibration.py` and the three calibration
scripts.

Calibration must be re-run after any mechanical change (arm repositioning,
chassis rebuild, tube-holder replacement).

---

## 0. Prerequisites

Before running any calibration:

1. **AA-8**: Servo IDs are assigned in `hardware.yaml` (`servo_joints`).
2. **AA-9**: Joint calibration has been run (`scripts/measure_limits.py --live`)
   and `motion_guard.servo_calibration.{arm}.unverified: false`.
3. **AA-11**: Forward kinematics (FK) is verified (TCP position error < 1 mm
   on a test pose) with `scripts/fk_verify.py --live`.
4. All arms are in a **safe, accessible position** and the chassis is on a
   stable, level surface.

---

## 1. Instruments

| Instrument | Specification | Purpose |
|---|---|---|
| Digital calipers | ≥ 150 mm reach, ≥ 0.1 mm resolution | Measuring chassis-frame reference positions |
| ChArUco board | 5×7 cells, 6×6 ArUco dictionary, **30 mm squares, 22.5 mm marker size** | Hand-eye and chassis feature reference |
| Rigid backing (A4) | Foam board or aluminium plate | Keeps fiducial flat |
| Reference datum block | Known-dimension gauge block or machinist square | Establishing chassis frame origin |

### Printing the ChArUco Board

Print the board at exactly **A4 size** (do not scale to fit).  Verify the
square size with calipers after printing: individual squares should measure
30.0 ± 0.5 mm.  Laminate or tape flat to rigid backing.

The board generator command (requires `opencv-contrib-python`):

```python
import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((5, 7), 0.030, 0.0225, aruco_dict)
img = board.generateImage((2480, 3508), marginSize=20)  # A4 at 300dpi
cv2.imwrite("charuco_board.png", img)
```

---

## 2. Chassis Frame Convention

The chassis frame origin is defined as:

- **Origin**: the centre of the front-left mounting bolt hole of the chassis
  base plate.
- **+X axis**: along the long axis of the tube (tube advance direction).
- **+Z axis**: pointing upward (away from the base plate surface).
- **+Y axis**: completing the right-handed frame (pointing to the right when
  looking from the origin along +X).

Measure all reference positions from this origin with calipers.

> **Note**: The tube translates along +X.  The chassis is fixed.

---

## 3. Arm-Base-to-Chassis Calibration (AC#1)

**Script**: `scripts/calibrate_arm_base.py --live --device arm_left` (or `arm_right`, `dexarm`)

**Method**: Kabsch/Umeyama rigid transform from point correspondences.

### 3.1 Reference Points

Select 6–8 **non-coplanar** features visible from the arm TCP.  Recommended:

- 4 chassis bolt holes (measure their centres with calipers relative to the
  chassis frame origin defined in §2).
- 2 corners of the fiducial board when placed on the chassis plate.

Record each point as `(x_mm, y_mm, z_mm)` in the chassis frame.

### 3.2 Procedure

1. Jog the arm TCP to touch each reference point in turn using
   `scripts/jog.py --arm arm_left --live`.
2. Run `scripts/calibrate_arm_base.py --live --device arm_left`.
   The script reads FK (TCP in arm base frame) and prompts for the
   chassis-frame position at each point.
3. Collect all correspondence pairs, then type `done`.
4. Review the residual.  Accept if RMS ≤ 2 mm; re-measure otherwise.
5. Paste the result into `hardware.yaml`:
   - `so101_kinematics.left_arm_base_to_chassis` (xyz + rpy form)
   - `calibration.left_arm_base` (full result dict)
   - Set `unverified: false`.

Repeat for `arm_right` and `dexarm`.

### 3.3 Expected Residual

RMS ≤ 2 mm is acceptable for coarse approach.  RMS ≤ 1 mm is preferred.
Higher residuals usually indicate a bad TCP touch (re-touch that point).

---

## 4. Hand-Eye Calibration (AC#2)

**Script**: `scripts/hand_eye.py --live --arm left` (or `right`)

**Method**: Park-Martin AX=XB (pure-NumPy; or `cv2.calibrateHandEye` when
available).

### 4.1 Setup

1. Mount the ChArUco board rigidly on the chassis (e.g. clamp upright facing
   the wrist camera's FOV).  It must not move during collection.
2. Ensure the wrist camera intrinsics are known.  If not, calibrate them
   first with a separate checkerboard intrinsic session:

   ```bash
   uv run python -c "
   import cv2, numpy as np, glob
   imgs = [cv2.imread(f) for f in glob.glob('calib_frames/*.png')]
   # ... standard cv2.calibrateCamera workflow
   "
   ```

   Paste the resulting `camera_matrix` and `dist_coeffs` values into
   `scripts/hand_eye.py` (the placeholder lines near `fx, fy, cx, cy`).

### 4.2 Procedure

1. Position the arm so the board is visible in the wrist camera.
2. Run `scripts/hand_eye.py --live --arm left`.
3. For each pose (minimum 3, recommend 8–12):
   a. Jog the arm to a new orientation (vary all joints, especially wrist).
   b. Press Enter in the script.  It captures one frame and detects the board.
4. When done, type `done`.
5. Review rotation/translation residuals.  Accept if rotation RMS ≤ 0.02 rad
   and translation RMS ≤ 3 mm.
6. Paste into `hardware.yaml` under `calibration.{arm}_wrist_camera`.

### 4.3 Degenerate Pose Warning

Avoid poses where all arm orientations have the same wrist angle.  The
Park-Martin solver needs diverse rotation axes across the motion pairs.

---

## 5. Tube-Axis Calibration (AC#3)

**Script**: `scripts/tube_axis.py --live`

**Method**: PCA line fit through measured attachment-point centres.

### 5.1 Procedure

1. Fix the tube in a stable position (clamp or place on the chassis fixture).
2. Identify at least 3 attachment points by their **lattice index** (count
   from your chosen index-0 end of the tube).
3. Run `scripts/tube_axis.py --live`.
4. For each attachment point:
   - Enter the lattice index when prompted.
   - Measure the centre of the attachment hole with calipers relative to the
     chassis frame origin (§2) and enter `x_mm,y_mm,z_mm`.
5. After collecting ≥ 3 points, type `done`.
6. Review:
   - **RMS deviation** should be < 0.5 mm (indicates a straight tube).
   - **Pitch estimate** should match `ATTACHMENT_PITCH_MM` (183.676 mm) ± 1 mm.
   - A large pitch error suggests a wrong lattice index or measurement error.
7. Paste result into `hardware.yaml` under `calibration.tube_axis`.

---

## 6. Verification (AC#5)

After all transforms are calibrated:

1. Drive each arm to a known joint configuration (e.g. near-neutral pose).
2. Read the FK-predicted TCP position (in base frame).
3. Compute the predicted chassis-frame position:
   `T_chassis_base @ [tcp_x, tcp_y, tcp_z, 1]`
4. Measure the actual TCP position with calipers (or probe to a chassis
   feature at the known configuration).
5. Report the per-axis error using
   `inchworm.calibration.predict_attachment_point(T_chassis_base, tcp_in_base, measured)`.
6. Record the error in the task notes.  Coarse approach requires the predicted
   position to be within the insertion policy's capture radius (see AA-20
   `workspace_radius_m`).

---

## 7. Stale Calibration Warning

When loading the hardware config, `inchworm.calibration.check_calibration_age`
emits a `CalibrationStalenessWarning` if:

- Any calibration block is `null` (not yet measured).
- The `ts_iso` timestamp is older than `calibration.max_calibration_age_days`
  days (default: 90 days).

The warning is also forwarded to the AA-7 session logger as a structured event.

To suppress the warning, re-run the affected calibration script with `--live`.

---

## 8. Config Update Checklist

After each calibration session:

- [ ] `so101_kinematics.left_arm_base_to_chassis.unverified: false`
- [ ] `so101_kinematics.right_arm_base_to_chassis.unverified: false`
- [ ] `calibration.left_arm_base` block populated (transform, method, ts_iso, residual_m)
- [ ] `calibration.right_arm_base` block populated
- [ ] `calibration.dexarm_base` block populated
- [ ] `calibration.left_wrist_camera` block populated
- [ ] `calibration.right_wrist_camera` block populated
- [ ] `calibration.tube_axis` block populated
- [ ] Run tests: `cd robot && uv run pytest tests/test_calibration.py -v`
- [ ] Run full suite: `cd robot && uv run pytest`
- [ ] Record calibration date and results in task AA-16 notes

---

## 9. Re-calibration Triggers

Re-calibrate any affected transform when:

- An arm is removed and re-attached to the chassis.
- The chassis is rebuilt or any mounting hardware is changed.
- The tube fixture or holder is modified.
- A wrist camera is removed or repositioned.
- The robot is transported (vibration may shift arm bases).

The stale-calibration warning (§7) will alert you when the configured
`max_calibration_age_days` expires.
