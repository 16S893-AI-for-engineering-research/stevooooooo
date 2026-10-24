# Scanner Camera and Laser-Plane Calibration Procedure

**Task:** AA-30 · **Module:** `inchworm/scanner_calibration.py`
**Script:** `scripts/calibrate_scanner.py` (dry-run default, `--live` for hardware)
**Owner supervision required** for all live steps.

---

## Overview

The scanner assembly consists of:
- **Camera:** Arducam OV9281 (1280 × 800, global shutter, USB)
- **Laser:** line laser switched via USB relay (AA-29 LaserDriver)
- **Assembly:** mounted on the DexArm end-effector; laser angled at ~40° from camera vertical, ~67 mm baseline, ~80 mm design standoff

Calibration is required before any scan-based leveling can be used, because:
- The design values (40°, 67 mm, 80 mm, 0.1 mm/px) are nominal and the as-built geometry will differ.
- An uncalibrated 0.1 mm/px assumption silently biases every leveling correction.

This procedure must be **repeated after any physical disturbance** of the scanner assembly (changing the camera, laser, mounting screws, or cable routing that could alter the relative geometry).

---

## Prerequisites

- `uv run pytest robot/tests/test_scanner_calibration.py -v` passes (software half, no hardware).
- Camera and laser wired and discoverable (`backlog task view AA-14`, `AA-29`).
- DexArm connected and AA-12 driver functional (`backlog task view AA-12`).
- Flat rigid chessboard target printed and mounted (see §Target below).
- Calibration area clear; laser safety: no one looking directly at the line laser.

---

## Calibration Target

**Primary target (recommended):** 9 × 6 inner chessboard (i.e. a 10 × 7 grid of squares), 25 mm per square.

Print `assets/chessboard_9x6_25mm.pdf` at 100% scale on flat white paper, laminate or mount on glass/MDF backing.  Verify the printed square size with a ruler; the physical size must match the `square_mm` parameter in the script (25 mm by default).

**Alternative target (partial occlusion tolerance):** ChArUco 7 × 5 board, 35 mm chessboard squares, 25 mm ArUco markers, dictionary `DICT_4X4_50`.  Print `assets/charuco_7x5_35mm.pdf`.  Use `--use-charuco` flag when running the script.

**Flatness requirement:** The target must be flat to < 0.5 mm across its working area.  Glass or anodised aluminium backing is preferred.

---

## Calibration Steps

All steps run in dry-run mode by default (no hardware).  Add `--live` for real hardware.

### Step 1 — Camera Intrinsics (AC#1)

**What:** Estimates the focal lengths (fx, fy), principal point (cx, cy), and Brown–Conrady distortion coefficients (k1, k2, p1, p2) using `cv2.calibrateCamera`.

**Procedure:**

1. Ensure the laser is **OFF** (script enforces this; LaserDriver controls the relay).
2. Mount the chessboard target so it fills most of the camera field at ~200–400 mm distance.
3. Run:
   ```bash
   uv run python scripts/calibrate_scanner.py --live --step intrinsics --n-frames 20
   ```
4. At each prompt, **rotate or translate** the board to a new pose (change angle by ≥ 15° or translate > 50 mm).  Aim for:
   - Coverage of all four quadrants of the image.
   - At least 5 frames with the board tilted > 30° from frontal.
   - No blurred frames (check the printed SNR in the log).
5. The script reports the per-image RMS reprojection error.  Typical good values: **< 0.5 px** overall RMS.  Reject frames with error > 1.5 px and re-run.

**Evidence for AC#1:** Paste the script output including `rms_reprojection_error_px`, `n_images_used`, `fx`, `fy`, and the `target_description` into the task comment.

---

### Step 2 — Laser Plane (AC#2)

**What:** Fits the laser plane in the camera frame using SVD over 3-D points obtained by intersecting laser-line pixels with the calibration target plane at several standoffs.

**Procedure:**

1. Complete Step 1 first (or use an existing intrinsics result).
2. Place the flat chessboard target at each standoff listed in `--standoffs` (default: 65, 70, 75, 80, 85, 90 mm from the camera front face).
3. Run:
   ```bash
   uv run python scripts/calibrate_scanner.py --live --step laser_plane \
       --standoffs 65 70 75 80 85 90
   ```
4. For each standoff:
   a. The script first captures a **background frame** (laser OFF) and detects the chessboard corners to estimate the target pose.
   b. The laser fires for **one frame** via `LaserDriver.on(duration_s=0.5)` then immediately turns off.
   c. The laser-line pixel positions are extracted with `extract_laser_line` and intersected with the estimated target plane.
5. The script reports `rms_residual_mm` and `max_residual_mm` over all pooled 3-D points.  Good values: RMS < 0.2 mm, max < 0.5 mm.

**Laser safety:** The laser switch is exclusively via `LaserDriver`.  The watchdog timer cuts off the relay after `max_continuous_on_s` (default 30 s) even if the process fails.

**Evidence for AC#2:** Paste the script output including `rms_residual_mm`, `max_residual_mm`, `n_points`, and the fitted `plane_normal`.

---

### Step 3 — Sensitivity at 80 mm Standoff (AC#3)

**What:** Computes the height-per-pixel sensitivity from the calibrated geometry (intrinsics + laser plane) and compares to the 0.1 mm/px design value.

This step runs automatically when both Steps 1 and 2 complete.  The script logs:
```
Sensitivity at 80 mm: <measured> mm/px  (design 0.1000 mm/px, deviation <N>%)
```

**Interpretation:** A deviation > ±10% means the as-built geometry differs significantly from the design.  Likely causes:
- Laser angle differs from 40° (re-measure with a protractor).
- Standoff differs from 80 mm (re-measure assembly).
- Camera focal length differs from design placeholder (Step 1 should correct this).

**Evidence for AC#3:** Paste the sensitivity log line, the calibrated angle implied by the plane normal, and any explanation of discrepancy > ±10%.

---

### Step 4 — Camera-to-DexArm-Tool Transform (AC#4)

**What:** Determines the 4 × 4 rigid transform from the camera frame to the DexArm tool frame using hand-eye calibration (`cv2.calibrateHandEye` when available, or the built-in Tsai–Lenz SVD fallback).

**Procedure:**

1. Complete Step 1 first.
2. Place the chessboard target in a fixed position in the workspace (do not move it during this step).
3. Run:
   ```bash
   uv run python scripts/calibrate_scanner.py --live --step hand_eye --n-frames 12
   ```
4. The DexArm moves to 12 pre-defined XYZ positions within the workspace.  At each position, the script:
   a. Reads the DexArm position via M114.
   b. Detects the chessboard and estimates its pose in the camera frame.
5. After all poses, `calibrate_hand_eye` solves AX = XB for the camera-to-gripper transform.
6. The script logs `T_tool_camera` (4 × 4 matrix).

**Evidence for AC#4:** Paste the `T_tool_camera` matrix and the method/n_poses from the log.

---

### Step 5 — Artifact Validation (AC#6)

**What:** Validates the calibrated height sensitivity against an artifact of known geometry (a gauge block or step target of known height).

**Procedure (manual, not yet automated):**

1. Complete Steps 1–4 and write the calibration result.
2. Load the calibration into AA-31 `triangulate_profile` (set `unverified=False` in the YAML).
3. Place a calibrated gauge block (e.g. 5 mm, 10 mm, or 25 mm) on the scan surface.
4. Capture a laser-line frame over the gauge block.
5. Triangulate the profile and measure the reconstructed height at the block position.
6. Compare to the nominal height.  Report the absolute error and the percentage of the nominal.

**Evidence for AC#6:** Report the gauge block nominal height, the reconstructed height, and the absolute error.

---

## Output

Results are written to `robot/config/scanner_calibration.yaml` with the following structure:

```yaml
calibration_date: "YYYY-MM-DD"
target_description: "chessboard 9x6 inner corners, 25mm squares"
notes: "..."
unverified: true   # set false after owner review
intrinsics:
  camera_matrix: [[...], [...], [...]]
  dist_coeffs: [k1, k2, p1, p2]
  rms_reprojection_error_px: 0.xxxx
  n_images_used: N
  ...
laser_plane:
  normal: [nx, ny, nz]
  d: float
  rms_residual_mm: float
  ...
sensitivity:
  sensitivity_mm_per_px: float
  design_sensitivity_mm_per_px: 0.1
  relative_deviation_pct: float
  ...
hand_eye:
  T_tool_camera: [[...], ...]
  method: "tsai"
  n_poses: N
  ...
```

After the owner reviews and validates all numbers, set `unverified: false`.  Loading with `live=True` will reject the calibration as long as `unverified: true`.

---

## Stale Calibration Warning

The loader (`load_calibration`) warns when the calibration is older than **90 days**
(`STALE_CALIBRATION_DAYS`).  In live mode it raises `UnverifiedCalibrationLoadError`.

Recalibrate after:
- Any physical disturbance of the scanner assembly.
- Replacing the camera, lens, laser, or mount screws.
- More than 90 days since the last calibration session.

---

## Typical Run (dry-run)

```bash
cd robot
uv run python scripts/calibrate_scanner.py
# Output: dry-run calibration written to config/scanner_calibration.yaml
```

## Typical Run (live, all steps)

```bash
cd robot
uv run python scripts/calibrate_scanner.py --live
# Operator prompts guide each capture; laser fires only during step 2 captures.
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `InsufficientDataError` on intrinsics | Too few corners detected | Use better lighting; print target at correct size; try ChArUco |
| RMS reprojection > 1 px | Motion blur or lighting glare | Use lower exposure; shield from ambient light |
| Laser-line plane RMS > 0.5 mm | Target not flat; wrong standoff | Use glass-backed target; re-measure standoffs |
| Sensitivity deviation > 20% | Assembly geometry differs from design | Measure actual laser angle with protractor; update design values in `hardware.yaml` |
| `UnverifiedCalibrationLoadError` in live mode | `unverified: true` in YAML | Review results, set `unverified: false` after validation |
| Stale calibration warning | Calibration older than 90 days | Re-run `calibrate_scanner.py --live` |

---

*Last updated by AA-30 implementation.*
