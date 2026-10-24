#!/usr/bin/env python3
"""Measure per-joint soft travel limits for the SO-101 arms.

This script supports the AA-9 joint-limits measurement workflow (AC#1).

Procedure
---------
1. **Dry-run (default)**: Prints the procedure instructions and the config
   template to update in ``hardware.yaml``.  No hardware is touched.

2. **Live mode** (``--live`` flag, requires hardware + owner supervision):
   For each joint in sequence:
     a. Disables torque on the target servo (allows manual movement).
     b. Prompts the operator to manually sweep the joint to its minimum
        travel and press Enter.  Records the position.
     c. Prompts the operator to sweep to maximum travel and press Enter.
        Records the position.
     d. Re-enables torque.
     e. Prints the measured limits in radians and the YAML snippet to paste
        into ``hardware.yaml``.

Safety invariants
-----------------
* **Dry-run by default** — ``--live`` must be passed explicitly.
* Real serial ports are never opened unless ``--live`` is given.
* Only the target joint's torque is disabled; all others remain enabled.
* The operator supervises every movement; the script never commands motion.
* After measurement, torque is re-enabled before moving to the next joint.

Usage
-----
Dry-run (default — no hardware)::

    uv run python scripts/measure_limits.py

Live (REQUIRES HARDWARE + owner supervision)::

    uv run python scripts/measure_limits.py --live --arm left
    uv run python scripts/measure_limits.py --live --arm right
    uv run python scripts/measure_limits.py --live --arm left --joints shoulder_pan,elbow_flex

Notes
-----
The recorded angles are in **degrees** (STS3215 range 0-360) and are
converted to **radians** for the config file.  The conversion used is:
    q_rad = (position_deg - 180.0) * (math.pi / 180.0)
which maps STS3215 range 0-360 to -π..+π (180 deg = 0 rad).
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path
from typing import Optional

# Allow running from project root without installing
_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s  %(name)s  %(message)s",
)
logger = logging.getLogger("measure_limits")

# Joint names in chain order (proximal → distal) — matches motion_guard.JOINT_NAMES
JOINT_NAMES_ORDERED = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def deg_to_rad_sts3215(position_deg: float) -> float:
    """Convert STS3215 degrees (0-360, 180=centre) to radians (-π..+π)."""
    return (position_deg - 180.0) * (math.pi / 180.0)


def _print_dry_run_instructions(arm: str, joints: list[str]) -> None:
    print()
    print("=" * 70)
    print("  AA-9 Measure Limits — DRY-RUN (no hardware touched)")
    print("=" * 70)
    print()
    print(f"  Target arm: {arm!r}")
    print(f"  Target joints: {', '.join(joints)}")
    print()
    print("  To measure real joint limits, run with --live and hardware attached:")
    print()
    print(f"    uv run python scripts/measure_limits.py --live --arm {arm}")
    print()
    print("  Procedure (live):")
    print("    1. Ensure the arm is in a safe position before starting.")
    print("    2. For each joint, the script disables torque on that servo.")
    print("    3. Manually sweep the joint to its MINIMUM travel (mechanical stop).")
    print("    4. Press Enter; the script records the current position.")
    print("    5. Manually sweep the joint to its MAXIMUM travel (mechanical stop).")
    print("    6. Press Enter; the script records and re-enables torque.")
    print("    7. After all joints, the script prints a YAML snippet to paste")
    print("       into hardware.yaml under motion_guard.<arm>_arm_limits.")
    print()
    print("  After updating hardware.yaml:")
    print("    - Set unverified: false")
    print("    - Set method: 'torque-off manual sweep, measure_limits.py --live'")
    print("    - Set measured_date: '<YYYY-MM-DD>'")
    print()
    print("  Config block to update (hardware.yaml):")
    print()
    print(f"  motion_guard:")
    print(f"    {arm}_arm_limits:")
    for j in JOINT_NAMES_ORDERED:
        print(f"      {j}_lower_rad: null  # UNVERIFIED — run --live")
        print(f"      {j}_upper_rad: null  # UNVERIFIED — run --live")
    print(f"      unverified: true  # set false after measurement")
    print(f"      method: null")
    print(f"      measured_date: null")
    print()
    print("[DRY-RUN complete — no hardware commands issued]")
    print("=" * 70)
    print()


def _read_position_deg(
    bus,  # ArmServoBus
    joint_name: str,
    joint_id_map: dict,
) -> float:
    """Read the current position of a joint (degrees)."""
    servo_id = joint_id_map.get(joint_name)
    if servo_id is None:
        raise RuntimeError(
            f"Joint '{joint_name}' has no servo ID in the joint map. "
            "Run scan_servos.py --live first."
        )
    return bus.read_position_deg(servo_id)


def _measure_live(arm: str, joints: list[str]) -> None:
    """Run the live measurement procedure for the given arm and joints."""
    from inchworm.config import load_config
    from inchworm.backends import get_backend, BackendName
    from inchworm.servo_bus import ArmServoBus

    cfg = load_config()

    if cfg.backend == BackendName.MOCK:
        logger.error(
            "Config backend is 'mock'; live measurement requires 'hardware' backend. "
            "Set backend: hardware in hardware.yaml or set INCHWORM_BACKEND=hardware."
        )
        sys.exit(1)

    arm_cfg = cfg.arms.left if arm == "left" else cfg.arms.right
    servo_joints = arm_cfg.servo_joints

    # Build joint_id_map — skip null IDs
    joint_id_map: dict[str, int] = {}
    servo_joints_dict = servo_joints.model_dump()
    for j in JOINT_NAMES_ORDERED:
        sid = servo_joints_dict.get(j)
        if sid is not None:
            joint_id_map[j] = sid

    if not joint_id_map:
        logger.error(
            "No servo IDs assigned for arm '%s'. "
            "Run scan_servos.py --live first to populate servo_joints in hardware.yaml.",
            arm,
        )
        sys.exit(1)

    # Add gripper to satisfy ArmServoBus constructor (required but not swept)
    gripper_id = servo_joints_dict.get("gripper")
    if gripper_id is not None:
        joint_id_map["gripper"] = gripper_id
    else:
        # Use a placeholder ID that we won't use (ArmServoBus requires gripper)
        logger.warning("No gripper servo ID; using placeholder 6 for ArmServoBus.")
        joint_id_map["gripper"] = 6

    backend = get_backend(cfg)
    transport = backend.open_serial(f"arm_{arm}")
    bus = ArmServoBus(
        transport=transport,
        role=f"arm_{arm}",
        joint_id_map=joint_id_map,
        dry_run=False,
    )

    results: dict[str, tuple[float, float]] = {}

    for joint_name in joints:
        if joint_name not in joint_id_map:
            logger.warning(
                "Joint '%s' has no servo ID on %s arm; skipping.", joint_name, arm
            )
            continue

        servo_id = joint_id_map[joint_name]
        print()
        print(f"  ── Joint: {joint_name} (servo ID {servo_id}) ──")
        print(f"  Disabling torque on servo {servo_id}...")
        bus.set_torque_enable(servo_id, False)
        print("  Torque OFF. The joint can now be moved freely.")
        print()

        input(
            f"  >>> Slowly move {joint_name!r} to its MINIMUM travel (mechanical stop).\n"
            "      Press Enter when at minimum... "
        )
        pos_min_deg = _read_position_deg(bus, joint_name, joint_id_map)
        pos_min_rad = deg_to_rad_sts3215(pos_min_deg)
        print(f"  Recorded MIN: {pos_min_deg:.2f} deg  →  {pos_min_rad:.4f} rad")
        print()

        input(
            f"  >>> Now move {joint_name!r} to its MAXIMUM travel (mechanical stop).\n"
            "      Press Enter when at maximum... "
        )
        pos_max_deg = _read_position_deg(bus, joint_name, joint_id_map)
        pos_max_rad = deg_to_rad_sts3215(pos_max_deg)
        print(f"  Recorded MAX: {pos_max_deg:.2f} deg  →  {pos_max_rad:.4f} rad")

        # Sanity check
        if pos_min_rad >= pos_max_rad:
            logger.warning(
                "MIN (%+.4f rad) >= MAX (%+.4f rad) for joint '%s'. "
                "Did you sweep in the wrong order?",
                pos_min_rad, pos_max_rad, joint_name,
            )

        print(f"  Re-enabling torque on servo {servo_id}...")
        bus.set_torque_enable(servo_id, True)
        print("  Torque ON.")

        results[joint_name] = (pos_min_rad, pos_max_rad)

    transport.close()

    # Print YAML snippet
    import datetime
    today = datetime.date.today().isoformat()
    print()
    print("=" * 70)
    print("  MEASUREMENT COMPLETE")
    print("=" * 70)
    print()
    print(f"  Paste this block into hardware.yaml under motion_guard.{arm}_arm_limits:")
    print()
    print(f"  {arm}_arm_limits:")
    for j in JOINT_NAMES_ORDERED:
        if j in results:
            lo, hi = results[j]
            print(f"    {j}_lower_rad: {lo:.6f}")
            print(f"    {j}_upper_rad: {hi:.6f}")
        else:
            print(f"    {j}_lower_rad: null  # not measured")
            print(f"    {j}_upper_rad: null  # not measured")
    print(f"    unverified: false  # set after confirming values are correct")
    print(f"    method: 'torque-off manual sweep, measure_limits.py --live'")
    print(f"    measured_date: '{today}'")
    print()
    print("  NOTE: After updating hardware.yaml, run the test suite:")
    print("    cd robot && uv run pytest tests/test_motion_guard.py -v")
    print("=" * 70)
    print()


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="measure_limits.py",
        description=(
            "AA-9: Measure per-joint soft travel limits for the SO-101 arms.\n"
            "DRY-RUN by default — prints instructions without touching hardware.\n"
            "Pass --live to perform the real measurement (REQUIRES HARDWARE + owner supervision)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--arm",
        choices=["left", "right"],
        default="left",
        help="Which arm to measure (default: left).",
    )
    p.add_argument(
        "--joints",
        metavar="j1,j2,...",
        default=None,
        help=(
            "Comma-separated list of joint names to measure "
            "(default: all 5 arm joints in order). "
            f"Valid names: {', '.join(JOINT_NAMES_ORDERED)}"
        ),
    )
    p.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Perform the real measurement on hardware. "
            "REQUIRES: hardware attached, owner supervision, "
            "servo IDs populated in hardware.yaml. "
            "Default: dry-run (print instructions only)."
        ),
    )
    return p


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    arm: str = args.arm

    if args.joints is not None:
        joints = [j.strip() for j in args.joints.split(",")]
        invalid = [j for j in joints if j not in JOINT_NAMES_ORDERED]
        if invalid:
            parser.error(
                f"Unknown joint name(s): {', '.join(invalid)}. "
                f"Valid: {', '.join(JOINT_NAMES_ORDERED)}"
            )
    else:
        joints = list(JOINT_NAMES_ORDERED)

    if not args.live:
        _print_dry_run_instructions(arm, joints)
        return 0

    # Live mode — HARDWARE ONLY
    logger.warning(
        "LIVE MODE: measure_limits.py will disable torque on servos and "
        "record positions. Owner supervision required. "
        "Arm: '%s', Joints: %s",
        arm, joints,
    )
    _measure_live(arm, joints)
    return 0


if __name__ == "__main__":
    sys.exit(main())
