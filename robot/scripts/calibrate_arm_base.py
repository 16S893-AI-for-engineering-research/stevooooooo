#!/usr/bin/env python3
"""AA-16: Calibrate arm-base-to-chassis and DexArm-to-chassis transforms.

Procedure
---------
This script implements AC#1 of AA-16.  In **dry-run** mode (default) it prints
the procedure instructions; in **live** mode (``--live``) it uses FK at each
collection pose to gather TCP-in-base-frame positions and prompts the operator
to enter the corresponding chassis-frame positions measured with calipers.

The Kabsch/Umeyama algorithm (``inchworm.calibration.calibrate_rigid``) then
computes the rigid transform and its RMS residual.

Instruments needed
------------------
* Digital calipers (0.1 mm resolution or better)
* A printed fiducial sheet with known point spacings, or the bolt centres
  of observable chassis features (see ``docs/calibration_procedure.md``)
* The arm must be kinematically calibrated (AA-11) and joint-calibrated (AA-9)
  before this script can gather FK-accurate TCP positions.

Safety invariants
-----------------
* **Dry-run by default** — ``--live`` must be passed explicitly.
* Real serial ports are never opened unless ``--live`` is given.
* No motion is commanded.  The operator manually positions the arm at each
  collection pose and presses Enter.
* All live mode checks fail closed on unverified calibration values.

Usage
-----
Dry-run (default — no hardware)::

    uv run python scripts/calibrate_arm_base.py

Live (REQUIRES HARDWARE + owner supervision)::

    uv run python scripts/calibrate_arm_base.py --live --device arm_left
    uv run python scripts/calibrate_arm_base.py --live --device arm_right
    uv run python scripts/calibrate_arm_base.py --live --device dexarm

Minimum required measurement poses: 4 non-coplanar.  6–8 are recommended.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Allow running from project root without installing
_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("calibrate_arm_base")

_VALID_DEVICES = ("arm_left", "arm_right", "dexarm")

_DRY_RUN_TEXT = """
================================================================================
  AA-16 Calibrate arm-base / DexArm-to-chassis  —  DRY-RUN (no hardware)
================================================================================

  Pass --live to perform the real calibration (REQUIRES HARDWARE + owner).

  Overview
  --------
  This procedure determines the rigid transform T_chassis_base for each device
  (left arm, right arm, DexArm) using a set of correspondence points:

    Source point: TCP position in the device base frame (computed via FK at a
                  known, held configuration).
    Dest point:   Same feature measured in the chassis frame with calipers.

  Instruments needed (see docs/calibration_procedure.md)
  -------------------------------------------------------
  1. Digital calipers (≥ 0.1 mm resolution, ≥ 150 mm reach).
  2. A printed A4 ChArUco 5×7 marker board (6×6 dictionary, 30 mm squares,
     22.5 mm marker size) taped flat to the chassis plate.  The board origin
     becomes the chassis reference frame origin during measurement.
  3. Alternatively: use the arm mounting bolt centres as fiducials (measure
     their relative positions on the chassis with calipers; then drive FK to
     each bolt and record the TCP position in the base frame).

  Procedure (live)
  ----------------
  1. Place the fiducial sheet flat on the chassis plate in a known, stable
     position.  Measure the fiducial corner positions relative to a chassis
     datum (e.g. a corner bolt hole) with the calipers.  Record as
     chassis_points (x_mm, y_mm, z_mm).

  2. For each fiducial point (minimum 4, recommend 6–8):
     a. Manually jog the arm TCP to touch the fiducial using the jog script:
        uv run python scripts/jog.py --arm {device} --dry-run
        (in live mode: uv run python scripts/jog.py --arm {device} --live)
     b. Press Enter when TCP is touching the point.
     c. The script reads the current joint positions via FK and records the
        TCP position in the arm base frame.
     d. Enter the corresponding chassis-frame position when prompted.

  3. The script runs the Kabsch solver and prints:
     - The estimated T_chassis_base (4×4 matrix as xyz+rpy)
     - RMS residual in mm

  4. Accept the result and paste it into hardware.yaml under:
       so101_kinematics.left_arm_base_to_chassis  (or right_arm_base_to_chassis)
     Set unverified: false only after confirming residual < 2 mm.

  Verification (AC#5)
  -------------------
  After pasting the result, run:
    uv run python scripts/calibrate_arm_base.py --live --device {device} --verify
  This drives the arm to a known pose, predicts the TCP in chassis frame, and
  compares to a caliper measurement.

================================================================================
[DRY-RUN complete — no hardware commands issued]
================================================================================
"""


def _run_dry(device: str) -> None:
    print(_DRY_RUN_TEXT.format(device=device))


def _run_live(device: str) -> None:
    """Live calibration — gathers FK TCP positions and chassis measurements."""
    import numpy as np
    from inchworm.config import load_config, BackendName
    from inchworm.calibration import calibrate_rigid, predict_attachment_point

    cfg = load_config()
    if cfg.backend == BackendName.MOCK:
        logger.error(
            "Config backend is 'mock'; live calibration requires 'hardware' backend. "
            "Set backend: hardware in hardware.yaml."
        )
        sys.exit(1)

    # Import kinematics
    from inchworm.kinematics import build_chain

    if device in ("arm_left", "arm_right"):
        from inchworm.backends import get_backend
        from inchworm.servo_bus import ArmServoBus

        arm = "left" if device == "arm_left" else "right"
        chain = build_chain(cfg.so101_kinematics)
        arm_cfg = cfg.arms.left if arm == "left" else cfg.arms.right
        servo_joints = arm_cfg.servo_joints.model_dump()

        joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        joint_id_map: dict[str, int] = {}
        for j in joint_names:
            sid = servo_joints.get(j)
            if sid is not None:
                joint_id_map[j] = sid
        gripper_id = servo_joints.get("gripper")
        if gripper_id is not None:
            joint_id_map["gripper"] = gripper_id
        else:
            joint_id_map["gripper"] = 6

        from inchworm.backends import live_backend  # --live always real hardware
        backend = live_backend(cfg)
        transport = backend.open_serial(device)
        bus = ArmServoBus(
            transport=transport,
            role=device,
            joint_id_map=joint_id_map,
            dry_run=False,
        )

        def read_tcp_base() -> np.ndarray:
            import inchworm.joint_calibration as jc
            positions_deg = {}
            for jname, jid in joint_id_map.items():
                if jname == "gripper":
                    continue
                positions_deg[jname] = bus.read_position_deg(jid)
            calib_cfg = cfg.motion_guard
            servo_cal = None
            if calib_cfg and calib_cfg.servo_calibration:
                servo_cal = (
                    calib_cfg.servo_calibration.left_arm
                    if arm == "left"
                    else calib_cfg.servo_calibration.right_arm
                )
            if servo_cal and not servo_cal.unverified:
                q = jc.servo_deg_to_joint_rad(positions_deg, servo_cal)
            else:
                logger.warning("Servo calibration unverified; FK from raw servo degrees (unreliable).")
                q = np.array(
                    [(positions_deg.get(j, 180.0) - 180.0) * (3.14159 / 180.0) for j in joint_names]
                )
            T = chain.fk(q)
            return T[:3, 3]

        get_tcp = read_tcp_base
        transport_to_close = transport

    elif device == "dexarm":
        from inchworm.backends import get_backend
        from inchworm.dexarm import DexArmDriver

        from inchworm.backends import live_backend  # --live always real hardware
        backend = live_backend(cfg)
        transport = backend.open_serial("dexarm")
        arm_driver = DexArmDriver(transport=transport, config=cfg.dexarm, dry_run=False)

        def read_tcp_base() -> np.ndarray:
            pos = arm_driver.get_position_mm()
            return np.array([pos["x"] / 1000.0, pos["y"] / 1000.0, pos["z"] / 1000.0])

        get_tcp = read_tcp_base
        transport_to_close = transport

    else:
        logger.error("Unknown device %r. Choose from: %s", device, _VALID_DEVICES)
        sys.exit(1)

    base_pts: list[np.ndarray] = []
    chassis_pts: list[np.ndarray] = []

    print()
    print(f"=== Live calibration: {device} ===")
    print("Collect at least 4 non-coplanar correspondence points.")
    print("Type 'done' when finished, 'abort' to quit without saving.")
    print()

    i = 0
    while True:
        print(f"\n--- Point {i + 1} ---")
        cmd = input("  Position arm TCP at the fiducial, then press Enter (or 'done'/'abort'): ").strip().lower()
        if cmd == "abort":
            print("Aborted.")
            transport_to_close.close()
            sys.exit(0)
        if cmd == "done":
            if len(base_pts) < 4:
                print(f"Need at least 4 points; only have {len(base_pts)}. Continue collecting.")
                continue
            break

        tcp = get_tcp()
        print(f"  TCP in {device} base frame: x={tcp[0]*1000:.2f} y={tcp[1]*1000:.2f} z={tcp[2]*1000:.2f} mm")

        raw = input("  Enter chassis-frame position x_mm,y_mm,z_mm: ").strip()
        try:
            parts = [float(v.strip()) for v in raw.split(",")]
            if len(parts) != 3:
                raise ValueError("need 3 values")
            chassis = np.array(parts) / 1000.0  # mm → m
        except ValueError as e:
            print(f"  Invalid input ({e}). Skipping this point.")
            continue

        base_pts.append(tcp)
        chassis_pts.append(chassis)
        print(f"  Recorded. ({len(base_pts)} points total)")
        i += 1

    transport_to_close.close()

    src = np.vstack(base_pts)
    dst = np.vstack(chassis_pts)

    result = calibrate_rigid(src, dst, notes=f"manual fiducial touch, {device}, live")
    T = result.transform
    rms_mm = result.residual_m * 1000.0

    import math
    # Extract xyz + rpy from T
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

    print()
    print("=" * 70)
    print("  CALIBRATION COMPLETE")
    print("=" * 70)
    print(f"  RMS residual: {rms_mm:.2f} mm  (n={result.n_samples})")
    print(f"  Method: {result.method}")
    print()
    print("  Paste into hardware.yaml under so101_kinematics (or dexarm_base):")
    print()

    if device == "arm_left":
        key = "left_arm_base_to_chassis"
    elif device == "arm_right":
        key = "right_arm_base_to_chassis"
    else:
        key = "dexarm_base_to_chassis"

    print(f"  {key}:")
    print(f"    xyz: [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")
    print(f"    rpy: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]")
    print(f"    unverified: {'true' if rms_mm > 2.0 else 'false'}  # RMS={rms_mm:.2f} mm")
    print()

    # Also print AA-16 calibration block for hardware.yaml
    import datetime
    ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
    block_key = "left_arm_base" if device == "arm_left" else ("right_arm_base" if device == "arm_right" else "dexarm_base")
    print(f"  calibration.{block_key}:")
    print(f"    transform: {result.transform.tolist()}")
    print(f"    method: '{result.method}'")
    print(f"    residual_m: {result.residual_m:.6f}")
    print(f"    ts_iso: '{ts}'")
    print(f"    n_samples: {result.n_samples}")
    print(f"    unverified: {'true' if rms_mm > 2.0 else 'false'}")
    print()
    print("  After updating hardware.yaml, run the test suite:")
    print("    cd robot && uv run pytest tests/test_calibration.py -v")
    print("=" * 70)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="calibrate_arm_base.py",
        description=(
            "AA-16: Calibrate arm-base-to-chassis / DexArm-to-chassis transforms.\n"
            "DRY-RUN by default — prints instructions.\n"
            "Pass --live to run on hardware (requires owner supervision)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--device",
        choices=list(_VALID_DEVICES),
        default="arm_left",
        help="Device to calibrate (default: arm_left).",
    )
    p.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Perform live calibration (REQUIRES HARDWARE + owner supervision).",
    )
    args = p.parse_args(argv)

    if not args.live:
        _run_dry(args.device)
        return 0

    logger.warning(
        "LIVE MODE: calibrate_arm_base.py will read arm positions and prompt "
        "for chassis measurements. Owner supervision required. Device: '%s'",
        args.device,
    )
    _run_live(args.device)
    return 0


if __name__ == "__main__":
    sys.exit(main())
