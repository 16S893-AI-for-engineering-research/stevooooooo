#!/usr/bin/env python3
"""FK physical-pose verification script.

This script verifies FK accuracy against measured physical poses.
Dry-run default: prints the FK-predicted poses for a set of joint
configurations without moving the arm.  Pass ``--live`` to read actual
joint positions from the servo bus and measure the physical end-effector pose.

AC#3 second half ("verified against at least five measured physical poses
per arm with the residuals recorded in task notes") requires hardware and
owner supervision.

AC#3 second half status
~~~~~~~~~~~~~~~~~~~~~~~~
    **UNCHECKED** — needs owner-supervised hardware session::

        uv run python scripts/fk_verify.py --live --arm left
        uv run python scripts/fk_verify.py --live --arm right

    Record residuals in AA-11 task notes via::

        backlog task edit AA-11 --append-notes "..."

Usage
-----
Dry-run (prints FK-predicted poses, no hardware)::

    uv run python scripts/fk_verify.py
    uv run python scripts/fk_verify.py --arm right

Live (reads real joint positions and measures FK residuals — HARDWARE + supervision)::

    uv run python scripts/fk_verify.py --live --arm left
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Allow running from project root without installing
_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

from inchworm.config import load_config
from inchworm.kinematics import build_chain

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s  %(name)s  %(message)s",
)
logger = logging.getLogger("fk_verify")

# ---------------------------------------------------------------------------
# Reference test poses for dry-run / simulation verification
# ---------------------------------------------------------------------------

# Five representative configurations (rad) to demonstrate FK in dry-run.
# These are NOT measured physical poses; they are simulation reference configs.
# In live mode, the actual measured joint positions are read from the servo bus.
_DRY_RUN_CONFIGS = [
    ("pose_0_zeros",   np.array([0.0,    0.0,   0.0,    0.0,   0.0  ])),
    ("pose_1_small",   np.array([0.2,   -0.3,   0.4,    0.1,  -0.2  ])),
    ("pose_2_mid",     np.array([-0.5,   0.5,  -0.4,    0.3,   0.8  ])),
    ("pose_3_wrist",   np.array([0.0,    0.0,   0.0,    0.5,   1.0  ])),
    ("pose_4_pan",     np.array([1.0,   -0.2,   0.3,   -0.1,   0.2  ])),
]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="fk_verify.py",
        description=(
            "FK physical-pose verification script.\n"
            "Dry-run (default): prints FK-predicted TCP poses for reference configs.\n"
            "Live (--live): reads actual joint positions from servo bus and measures\n"
            "               the physical end-effector pose.\n\n"
            "AC#3 second half UNCHECKED — needs owner-supervised hardware session:\n"
            "    uv run python scripts/fk_verify.py --live --arm left\n"
            "    uv run python scripts/fk_verify.py --live --arm right"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--arm",
        choices=["left", "right"],
        default="left",
        help="Arm to verify.  Default: left.",
    )
    p.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Read actual joint positions from servo bus and report FK residuals.  "
            "REQUIRES REAL HARDWARE and owner supervision.  "
            "Default: dry-run."
        ),
    )
    return p


def main(argv: Optional[list[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    arm_name = args.arm

    if not args.live:
        # ---------------------------------------------------------------
        # Dry-run: compute FK for reference configurations and print results
        # ---------------------------------------------------------------
        print()
        print("=" * 70)
        print("  FK VERIFICATION — DRY-RUN (simulation only, no hardware)")
        print(f"  arm: {arm_name}")
        print("=" * 70)
        print(
            f"  {'Name':<20s}  {'q (rad)':<40s}  {'TCP x':>8}  {'TCP y':>8}  {'TCP z':>8}"
        )
        print("-" * 100)
        for name, q in _DRY_RUN_CONFIGS:
            T = chain.fk(q)
            pos = T[:3, 3]
            q_str = np.array2string(q, precision=3, separator=",")
            print(f"  {name:<20s}  {q_str:<40s}  {pos[0]:8.4f}  {pos[1]:8.4f}  {pos[2]:8.4f}")
        print("-" * 100)
        print()
        print("  NOTE: base-to-chassis transform is UNVERIFIED (AA-16 pending).")
        print("  NOTE: tool-frame offset is UNVERIFIED (AA-16 pending).")
        print()
        print("  AC#3 second half (physical poses) UNCHECKED.")
        print("  needs owner-supervised hardware session: "
              "uv run python scripts/fk_verify.py --live --arm left")
        print("  needs owner-supervised hardware session: "
              "uv run python scripts/fk_verify.py --live --arm right")
        print("=" * 70)
        return 0

    # -----------------------------------------------------------------------
    # Live mode: read actual joint positions, prompt owner to measure TCP pose
    # -----------------------------------------------------------------------
    logger.warning("LIVE mode — opening servo bus for arm '%s'.", arm_name)
    print()
    print("=" * 70)
    print(f"  FK VERIFICATION — LIVE MODE  arm: {arm_name}")
    print("=" * 70)

    try:
        from inchworm.backends import get_backend
        from inchworm.servo_bus import ServoBusDriver
        backend = get_backend(cfg)
        bus = ServoBusDriver(
            backend=backend,
            arm_cfg=cfg.arms.left if arm_name == "left" else cfg.arms.right,
            arm_role=arm_name,
        )
    except Exception as exc:
        logger.error("Failed to open servo bus: %s", exc)
        return 1

    results: list[dict] = []
    pose_index = 0

    print()
    print("  For each pose:")
    print("    1. Move the arm to the desired configuration.")
    print("    2. Press ENTER to read joint positions and compute FK.")
    print("    3. Measure the physical TCP position with a ruler.")
    print("    4. Enter the measured XYZ (mm) when prompted.")
    print("    5. Residuals are recorded automatically.")
    print()

    while True:
        pose_index += 1
        user_input = input(
            f"  Pose {pose_index}: press ENTER to read joints, or 'q' to quit: "
        ).strip()
        if user_input.lower() == "q":
            break

        try:
            q_measured = bus.read_joint_positions_rad()
        except Exception as exc:
            logger.error("Failed to read joint positions: %s", exc)
            continue

        T_fk = chain.fk(q_measured)
        tcp_fk_mm = T_fk[:3, 3] * 1000.0

        print(f"    q_measured (rad): {np.round(q_measured, 4)}")
        print(f"    FK-predicted TCP (mm):  x={tcp_fk_mm[0]:.2f}  "
              f"y={tcp_fk_mm[1]:.2f}  z={tcp_fk_mm[2]:.2f}")

        meas_str = input(
            "    Enter measured TCP in mm as 'x,y,z' (or ENTER to skip): "
        ).strip()
        if not meas_str:
            continue

        try:
            meas_vals = [float(v) for v in meas_str.split(",")]
            if len(meas_vals) != 3:
                raise ValueError
            tcp_meas_mm = np.array(meas_vals, dtype=np.float64)
        except ValueError:
            print("    Invalid input — skipping measurement.")
            continue

        residual_mm = np.linalg.norm(tcp_fk_mm - tcp_meas_mm)
        print(
            f"    Residual: {residual_mm:.2f} mm  "
            f"(FK {tcp_fk_mm.round(2)} vs measured {tcp_meas_mm.round(2)})"
        )
        results.append({
            "pose": pose_index,
            "q_rad": q_measured.tolist(),
            "fk_tcp_mm": tcp_fk_mm.tolist(),
            "measured_tcp_mm": tcp_meas_mm.tolist(),
            "residual_mm": residual_mm,
        })

    if results:
        print()
        print("  === Summary ===")
        print(f"  {'Pose':<6}  {'Residual (mm)':>14}")
        for r in results:
            print(f"  {r['pose']:<6}  {r['residual_mm']:14.2f}")
        mean_res = sum(r["residual_mm"] for r in results) / len(results)
        max_res  = max(r["residual_mm"] for r in results)
        print(f"  {'mean':<6}  {mean_res:14.2f}")
        print(f"  {'max':<6}  {max_res:14.2f}")
        print()
        print("  Record these residuals in AA-11 task notes via:")
        print("    backlog task edit AA-11 --append-notes '...'")
        print()
        print("  If max residual < 5 mm, AC#3 physical half may be checked.")
        print("  If max residual >= 5 mm, update the kinematic parameters.")
    else:
        print("  No measurements recorded.")

    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
