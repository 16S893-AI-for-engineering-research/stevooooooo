#!/usr/bin/env python3
"""AA-16: Hand-eye calibration for wrist cameras (AX = XB).

Procedure
---------
This script implements AC#2 of AA-16.  In **dry-run** mode (default) it prints
the procedure instructions; in **live** mode (``--live``) it collects arm
end-effector poses (via FK) and corresponding fiducial observations (via the
wrist camera) and runs the Park-Martin hand-eye solver.

The result is the camera pose in the end-effector frame: T_ee_cam.

Instruments needed
------------------
* A printed ChArUco 5×7 marker board (6×6 dictionary, 30 mm squares,
  22.5 mm marker size) fixed in a stable chassis-mounted position.
* Camera intrinsics pre-calibrated (the solve uses pose-from-board, not
  reprojection — intrinsics only needed for the board detection step).
* At least 3 distinct end-effector poses with clearly different orientations
  (avoid degenerate rotations; 8–12 poses are recommended).

Safety invariants
-----------------
* **Dry-run by default** — ``--live`` must be passed explicitly.
* Only one camera is opened at a time (USB 2.0 invariant).
* No motion is commanded; the operator jogs the arm and presses Enter.

Usage
-----
Dry-run (default — no hardware)::

    uv run python scripts/hand_eye.py

Live (REQUIRES HARDWARE + owner supervision)::

    uv run python scripts/hand_eye.py --live --arm left
    uv run python scripts/hand_eye.py --live --arm right
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("hand_eye")

_DRY_RUN_TEXT = """
================================================================================
  AA-16 Hand-Eye Calibration (AX = XB)  —  DRY-RUN (no hardware)
================================================================================

  Pass --live to perform the real calibration (REQUIRES HARDWARE + owner).

  Overview
  --------
  This procedure determines T_ee_cam: the pose of the wrist camera in the
  arm end-effector frame.  It uses the Park-Martin AX = XB formulation
  (or cv2.calibrateHandEye when available).

  Instruments needed (see docs/calibration_procedure.md)
  -------------------------------------------------------
  1. ChArUco board 5×7, 6×6 dictionary, 30 mm squares, 22.5 mm marker size.
     (Printable at A4 size; laminate or tape to a rigid backing.)
  2. Mount the board in a STABLE position visible from all collection poses
     (e.g. clamp to the chassis frame pointing upward).
  3. Camera intrinsics must be known (calibrate with calibrateCamera first).
     For this wrist camera, use the scanner camera calibration workflow or
     separate checkerboard intrinsic calibration.

  Procedure (live)
  ----------------
  1. Ensure the board is rigidly fixed (it must not move during collection).

  2. For each pose (minimum 3, recommend 8–12 distinct orientations):
     a. Jog the arm end-effector to a new orientation:
          uv run python scripts/jog.py --arm {arm} --live
     b. Run this script with --live; it will:
        - Read joint positions via FK to get T_base_ee (end-effector in base).
        - Capture one frame from the wrist camera.
        - Detect the ChArUco board and estimate T_cam_board (board in cam).
        - Record both transforms.
     c. Repeat from a new arm orientation.

  3. After collecting >= 3 poses, press 'd' or type 'done'.
     The script runs the Park-Martin solver and prints:
     - T_ee_cam (4×4) as xyz + rpy
     - Rotation residual RMS (rad) and translation residual RMS (m)

  4. Accept the result and paste into hardware.yaml:
       calibration.{arm}_wrist_camera:
         transform: [...]
         ...
     Set unverified: false only after confirming residuals are acceptable.

================================================================================
[DRY-RUN complete — no hardware commands issued]
================================================================================
"""


def _run_dry(arm: str) -> None:
    print(_DRY_RUN_TEXT.format(arm=arm))


def _run_live(arm: str) -> None:
    """Live hand-eye calibration using the wrist camera and FK."""
    import numpy as np
    import cv2  # type: ignore

    from inchworm.config import load_config, BackendName
    from inchworm.calibration import calibrate_hand_eye
    from inchworm.kinematics import build_chain
    from inchworm.backends import get_backend
    from inchworm.servo_bus import ArmServoBus
    from inchworm.camera_capture import CameraCapture

    cfg = load_config()
    if cfg.backend == BackendName.MOCK:
        logger.error("Config backend is 'mock'; live calibration requires 'hardware' backend.")
        sys.exit(1)

    chain = build_chain(cfg.so101_kinematics)
    arm_label = "left" if arm == "left" else "right"
    arm_cfg = cfg.arms.left if arm_label == "left" else cfg.arms.right
    servo_joints = arm_cfg.servo_joints.model_dump()
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    joint_id_map: dict[str, int] = {}
    for j in joint_names:
        sid = servo_joints.get(j)
        if sid is not None:
            joint_id_map[j] = sid
    gripper_id = servo_joints.get("gripper")
    joint_id_map["gripper"] = gripper_id if gripper_id is not None else 6

    backend = get_backend(cfg)
    transport = backend.open_serial(f"arm_{arm_label}")
    bus = ArmServoBus(
        transport=transport,
        role=f"arm_{arm_label}",
        joint_id_map=joint_id_map,
        dry_run=False,
    )

    # Camera config
    cam_cfg = cfg.cameras.left_arm if arm_label == "left" else cfg.cameras.right_arm
    cap = CameraCapture(cam_cfg.identity)

    # ChArUco board (5×7, 6×6 dictionary, 30mm squares, 22.5mm markers)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((5, 7), 0.030, 0.0225, aruco_dict)

    # Placeholder intrinsics — operator must supply calibrated values
    # These are used for board pose estimation only; replace with real values
    fx, fy, cx, cy = 800.0, 800.0, 320.0, 240.0
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)
    logger.warning(
        "Using PLACEHOLDER camera intrinsics. Replace fx,fy,cx,cy,dist with "
        "calibrated values before accepting the hand-eye result."
    )

    R_g2b_list: list[np.ndarray] = []
    t_g2b_list: list[np.ndarray] = []
    R_t2c_list: list[np.ndarray] = []
    t_t2c_list: list[np.ndarray] = []

    import inchworm.joint_calibration as jc
    servo_cal = None
    if cfg.motion_guard and cfg.motion_guard.servo_calibration:
        servo_cal = (
            cfg.motion_guard.servo_calibration.left_arm
            if arm_label == "left"
            else cfg.motion_guard.servo_calibration.right_arm
        )

    def read_ee_pose() -> tuple[np.ndarray, np.ndarray]:
        """Return (R_ee_in_base 3×3, t_ee_in_base 3) from FK."""
        positions_deg = {}
        for jname, jid in joint_id_map.items():
            if jname == "gripper":
                continue
            positions_deg[jname] = bus.read_position_deg(jid)
        if servo_cal and not servo_cal.unverified:
            q = jc.servo_deg_to_joint_rad(positions_deg, servo_cal)
        else:
            logger.warning("Servo calibration unverified; using raw degrees.")
            q = np.array(
                [(positions_deg.get(j, 180.0) - 180.0) * (3.14159 / 180.0)
                 for j in joint_names]
            )
        T = chain.fk(q)
        return T[:3, :3], T[:3, 3]

    print()
    print(f"=== Live hand-eye calibration: {arm} arm ===")
    print("Collect >= 3 arm poses with clearly different orientations.")
    print("Type 'done' when finished, 'abort' to quit without saving.")
    print()

    i = 0
    while True:
        cmd = input(f"\n--- Pose {i + 1} --- Jog arm to a new orientation and press Enter (or 'done'/'abort'): ").strip().lower()
        if cmd == "abort":
            transport.close()
            cap.close()
            sys.exit(0)
        if cmd == "done":
            if len(R_g2b_list) < 3:
                print(f"Need >= 3 poses; only have {len(R_g2b_list)}. Continue.")
                continue
            break

        # Read FK end-effector pose
        R_ee, t_ee = read_ee_pose()
        print(f"  T_base_ee translation: {t_ee * 1000} mm")

        # Capture frame and detect board
        frame = cap.capture_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is None or len(ids) < 4:
            print("  Board not detected (too few markers). Skipping this pose.")
            continue

        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )
        if ret < 4:
            print(f"  Only {ret} ChArUco corners; need >= 4. Skipping.")
            continue

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
        )
        if not ok:
            print("  Pose estimation failed. Skipping.")
            continue

        R_board2cam, _ = cv2.Rodrigues(rvec)
        t_board2cam = tvec.ravel()

        R_g2b_list.append(R_ee)
        t_g2b_list.append(t_ee)
        R_t2c_list.append(R_board2cam)
        t_t2c_list.append(t_board2cam)
        print(f"  Pose recorded. ({len(R_g2b_list)} total)")
        i += 1

    transport.close()
    cap.close()

    result = calibrate_hand_eye(
        R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
        notes=f"charuco_board, {arm} wrist camera, live",
    )

    import math, datetime
    T = result.transform
    R = T[:3, :3]
    xyz = T[:3, 3].tolist()
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    rpy = [roll, pitch, yaw]

    ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
    key = f"{arm}_wrist_camera"

    print()
    print("=" * 70)
    print("  HAND-EYE CALIBRATION COMPLETE")
    print("=" * 70)
    print(f"  Method: {result.method}")
    print(f"  Rotation residual RMS: {result.residual_rad:.6f} rad")
    print(f"  Translation residual RMS: {result.residual_m * 1000:.2f} mm  (n={result.n_samples})")
    print()
    print("  Paste into hardware.yaml under calibration:")
    print()
    print(f"  {key}:")
    print(f"    transform: {result.transform.tolist()}")
    print(f"    method: '{result.method}'")
    print(f"    residual_m: {result.residual_m:.6f}")
    print(f"    residual_rad: {result.residual_rad:.6f}")
    print(f"    ts_iso: '{ts}'")
    print(f"    n_samples: {result.n_samples}")
    print(f"    unverified: false  # verify residuals acceptable before setting false")
    print("=" * 70)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="hand_eye.py",
        description=(
            "AA-16: Hand-eye calibration for wrist cameras (AX = XB).\n"
            "DRY-RUN by default. Pass --live for hardware (owner supervision required)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm", choices=["left", "right"], default="left",
                   help="Which arm's wrist camera to calibrate (default: left).")
    p.add_argument("--live", action="store_true", default=False,
                   help="Live calibration (REQUIRES HARDWARE + owner supervision).")
    args = p.parse_args(argv)

    if not args.live:
        _run_dry(args.arm)
        return 0

    logger.warning(
        "LIVE MODE: hand_eye.py. Owner supervision required. Arm: '%s'", args.arm
    )
    _run_live(args.arm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
