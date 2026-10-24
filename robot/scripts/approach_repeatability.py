#!/usr/bin/env python3
"""AA-18 approach repeatability measurement script.

Measures the repeatability of the achieved pre-insertion pose over N
successive approaches and reports the per-axis standard deviation.

Usage
-----
Dry-run (default — no hardware, prints the plan for each approach):

    cd robot && uv run python scripts/approach_repeatability.py

Live hardware session (requires --live flag and owner supervision):

    cd robot && uv run python scripts/approach_repeatability.py --live --n 10

AC#4 of AA-18 (repeatability over ≥10 approaches) is a HARDWARE acceptance
criterion.  It cannot be satisfied by this script in dry-run mode.

Safety
------
* Never runs live motion without --live.
* Dry-run mode prints the plan for each trial, does not move the arm.
* Live mode fails closed if approach config is unverified (see hardware.yaml).
* All motion goes through GuardedArm / MotionGuard (AA-9).
* Retreat pose is commanded on any tolerance failure.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Add project root to path so this can be run as a script
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

import numpy as np

from inchworm.config import load_config
from inchworm.frames import FrameModel
from inchworm.kinematics import build_chain
from inchworm.motion_guard import build_guarded_arm
from inchworm.approach import (
    plan_approach,
    execute_approach,
    get_pre_insertion_pose,
    ApproachUnreachableError,
    ApproachUnsafeError,
    ApproachToleranceError,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("approach_repeatability")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="AA-18 pre-insertion pose repeatability measurement."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Enable live hardware motion (REQUIRES owner supervision). "
             "Without --live, dry-run mode prints the plan without moving.",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=3,
        help="Number of approach trials (default: 3 for dry-run, ≥10 for AC#4).",
    )
    parser.add_argument(
        "--arm",
        choices=["left", "right"],
        default="left",
        help="Which arm to use (default: left).",
    )
    parser.add_argument(
        "--lattice-index",
        type=int,
        default=0,
        help="Attachment-point lattice index to approach (default: 0).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to hardware.yaml (default: robot/config/hardware.yaml).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    if not args.live:
        print(
            "AA-18 approach_repeatability.py: DRY-RUN mode.\n"
            "  Prints the planned trajectory for each trial without moving hardware.\n"
            "  Add --live for live hardware execution (owner supervision required).\n"
            "  AC#4 requires --live --n 10 (10 real approaches).\n"
        )

    cfg = load_config(args.config)
    approach_cfg = cfg.approach

    if approach_cfg is None:
        logger.error(
            "No 'approach' block in hardware.yaml. "
            "Add the AA-18 approach config block and retry."
        )
        sys.exit(1)

    if args.live and approach_cfg.unverified:
        logger.error(
            "approach.unverified is True — live mode is blocked (fail-closed). "
            "Run AA-16 calibration, update hardware.yaml, set unverified: false."
        )
        sys.exit(1)

    chain_left = build_chain(cfg.so101_kinematics)
    chain_right = build_chain(cfg.so101_kinematics)
    chain = chain_left if args.arm == "left" else chain_right

    frame_model = FrameModel.from_config(cfg, chain_left, chain_right)
    traj_limits = cfg.so101_kinematics.trajectory

    # --- Live mode: open hardware buses ---
    executor = None
    if args.live:
        from inchworm.backends import get_backend
        from inchworm.servo_bus import ArmServoBus

        backend = get_backend(cfg)
        arm_cfg = cfg.arms.left if args.arm == "left" else cfg.arms.right
        try:
            bus = ArmServoBus(arm_cfg, backend=backend)
        except Exception as exc:
            logger.error("Cannot open servo bus for %s arm: %s", args.arm, exc)
            sys.exit(1)

        executor = build_guarded_arm(
            bus=bus,
            cfg=cfg,
            arm_role=args.arm,
            live=True,
        )
        logger.info("Live hardware executor ready for %s arm.", args.arm)

    # --- Collect achieved poses ---
    achieved_positions: list[np.ndarray] = []
    achieved_orientations: list[np.ndarray] = []
    n_ok = 0
    n_failed = 0

    # Compute planned pre-insertion pose for reference
    T_target = get_pre_insertion_pose(
        args.lattice_index, args.arm, frame_model, approach_cfg, chain
    )
    print(
        f"Pre-insertion target position (arm base frame): "
        f"{np.round(T_target[:3, 3] * 1000, 2)} mm"
    )

    for trial in range(args.n):
        print(f"\n=== Trial {trial + 1}/{args.n} ===")

        # Read present pose
        if args.live and executor is not None:
            read_fn = getattr(executor, "read_positions_rad", None)
            if read_fn is not None:
                q_present = np.asarray(read_fn(), dtype=np.float64)
            else:
                logger.warning("Cannot read present pose; using zeros.")
                q_present = np.zeros(5, dtype=np.float64)
        else:
            q_present = np.zeros(5, dtype=np.float64)

        # Plan
        try:
            plan = plan_approach(
                lattice_index=args.lattice_index,
                arm=args.arm,
                q_present=q_present,
                frame_model=frame_model,
                chain=chain,
                guard=(executor.guard if executor is not None else
                       build_guarded_arm(
                           bus=_MockBus(),
                           cfg=cfg,
                           arm_role=args.arm,
                           live=False,
                       ).guard),
                approach_cfg=approach_cfg,
                traj_limits=traj_limits,
                live=args.live,
            )
        except ApproachUnreachableError as e:
            logger.error("Trial %d: unreachable: %s", trial + 1, e)
            n_failed += 1
            continue
        except ApproachUnsafeError as e:
            logger.error("Trial %d: unsafe: %s", trial + 1, e)
            n_failed += 1
            continue

        # Execute (dry-run or live)
        if not args.live:
            print(plan.summary)
            # In dry-run, record the planned (not achieved) position
            T_planned = chain.fk(plan.q_preinsert)
            achieved_positions.append(T_planned[:3, 3].copy())
            achieved_orientations.append(T_planned[:3, :3].copy())
            n_ok += 1
        else:
            try:
                execute_approach(
                    plan=plan,
                    executor=executor,
                    chain=chain,
                    approach_cfg=approach_cfg,
                    live=True,
                )
                # Read achieved pose
                read_fn = getattr(executor, "read_positions_rad", None)
                if read_fn is not None:
                    q_achieved = np.asarray(read_fn(), dtype=np.float64)
                else:
                    q_achieved = plan.q_preinsert.copy()
                T_achieved = chain.fk(q_achieved)
                achieved_positions.append(T_achieved[:3, 3].copy())
                achieved_orientations.append(T_achieved[:3, :3].copy())
                pos_err_mm = float(
                    np.linalg.norm(T_achieved[:3, 3] - T_target[:3, 3])
                ) * 1000
                print(f"  Achieved pos: {np.round(T_achieved[:3, 3] * 1000, 2)} mm "
                      f"  error={pos_err_mm:.2f} mm")
                n_ok += 1
            except ApproachToleranceError as e:
                logger.error("Trial %d: tolerance failure: %s", trial + 1, e)
                n_failed += 1
            except Exception as e:
                logger.error("Trial %d: unexpected error: %s", trial + 1, e)
                n_failed += 1

    # --- Report ---
    print(f"\n=== Repeatability Report: {n_ok}/{args.n} successful ===")

    if len(achieved_positions) >= 2:
        pos_arr = np.array(achieved_positions)  # shape (N, 3)
        std_xyz = np.std(pos_arr, axis=0) * 1000  # mm
        mean_xyz = np.mean(pos_arr, axis=0) * 1000  # mm
        print(f"  Position mean (mm):  x={mean_xyz[0]:.3f}  y={mean_xyz[1]:.3f}  z={mean_xyz[2]:.3f}")
        print(f"  Position std  (mm):  x={std_xyz[0]:.3f}  y={std_xyz[1]:.3f}  z={std_xyz[2]:.3f}")
        print(f"  Position max std: {float(np.max(std_xyz)):.3f} mm")
        print(f"  Position 3D range: "
              f"{float(np.linalg.norm(np.max(pos_arr, 0) - np.min(pos_arr, 0)))*1000:.3f} mm")
    elif len(achieved_positions) == 1:
        print("  Only 1 successful trial; no repeatability statistic available.")
    else:
        print("  No successful trials; no repeatability data.")

    if n_failed > 0:
        print(f"  WARNING: {n_failed} trial(s) failed.")

    if not args.live:
        print(
            "\n  NOTE: This is a DRY-RUN report (FK of planned joint config, "
            "not achieved hardware pose).\n"
            "  AC#4 requires: --live --n 10\n"
            "  Command: cd robot && uv run python scripts/approach_repeatability.py "
            "--live --n 10"
        )
    elif n_ok < 10:
        print(
            f"\n  NOTE: Only {n_ok} successful trials completed. "
            "AC#4 requires ≥ 10 live approaches."
        )
    else:
        print(
            f"\n  AC#4 evidence: {n_ok} live approaches completed. "
            "Record this output and paste into the task comment."
        )


class _MockBus:
    """Minimal bus mock for dry-run guard construction."""

    _joint_id_map: dict = {}

    def read_position_deg(self, servo_id: int) -> float:
        return 180.0

    def write_position_deg(self, joint_name: str, position_deg: float) -> None:
        pass


if __name__ == "__main__":
    main()
