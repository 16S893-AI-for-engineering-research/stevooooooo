#!/usr/bin/env python3
"""Cartesian jog CLI for the SO-101 arm.

Dry-run is the default.  Real motion requires ``--live`` flag and hardware.

Usage examples
--------------
Jog left arm +10 mm in X (dry-run, prints plan only)::

    uv run python scripts/jog.py --arm left --delta 0.01,0.0,0.0

Move right arm to absolute TCP position 0.10,0.00,0.20 (dry-run)::

    uv run python scripts/jog.py --arm right --abs 0.10,0.00,0.20

Live motion (REQUIRES HARDWARE, owner supervision)::

    uv run python scripts/jog.py --arm left --delta 0.01,0.0,0.0 --live

Notes
-----
*   Gripper servo is **never** commanded by this script.  The gripper is
    reserved for the AA-17 state machine via the AA-8 gripper_channel token.
*   Collision check: calls ``collision_check_hook(arm, q_target)`` which
    currently always returns ``None`` (no collision detected).  This hook
    is a placeholder for AA-9.  AC#4 remains unchecked until AA-9 is
    implemented.
*   AA-7 telemetry: jog events are logged via the AA-7 SessionLogger when
    ``--live`` is active.  Dry-run logs to stdout only.

Hardware-only motion: ``--live`` flag
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
When ``--live`` is passed the script:
  1. Loads hardware config and opens the servo bus backend.
  2. Reads current joint angles from the arm.
  3. Runs IK to find the target configuration.
  4. Plans a trapezoidal trajectory.
  5. Sends each waypoint to the servo bus.
Never commands the gripper servo.
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
from inchworm.kinematics import build_chain, IKResult
from inchworm.trajectory import plan_joint_trajectory, sample_trajectory

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s  %(name)s  %(message)s",
)
logger = logging.getLogger("jog")

# ---------------------------------------------------------------------------
# AA-9 collision check hook (placeholder)
# ---------------------------------------------------------------------------


def collision_check_hook(arm: str, q: np.ndarray) -> Optional[str]:
    """Check for collisions before commanding motion.

    This is a **placeholder** for the AA-9 collision envelope.  It always
    returns ``None`` (no collision) until AA-9 is implemented.

    Parameters
    ----------
    arm:
        ``"left"`` or ``"right"``.
    q:
        Proposed joint configuration (rad, 5-DoF).

    Returns
    -------
    str or None
        Collision description string if a collision is detected, else ``None``.

    .. note::
        AC#4 (collision envelope) is unchecked pending AA-9.
    """
    # --- AA-9 hook ---
    # Replace this body with the AA-9 collision check when available.
    return None
    # --- AA-9 hook end ---


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_xyz(s: str, name: str) -> np.ndarray:
    """Parse a comma-separated ``x,y,z`` string into a float array."""
    try:
        parts = [float(v.strip()) for v in s.split(",")]
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"Invalid {name} '{s}': expected three comma-separated floats"
        ) from e
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Invalid {name} '{s}': expected exactly three values (x,y,z)"
        )
    return np.array(parts, dtype=np.float64)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="jog.py",
        description=(
            "Cartesian jog CLI for the SO-101 arm.\n"
            "DRY-RUN by default — prints plan without moving hardware.\n"
            "Pass --live to send motion commands (requires hardware + owner supervision)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--arm",
        choices=["left", "right"],
        required=True,
        help="Which arm to jog.",
    )
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--delta",
        metavar="dx,dy,dz",
        help=(
            "Cartesian delta (metres) relative to current TCP position, "
            "e.g. '0.01,0.0,0.0'."
        ),
    )
    group.add_argument(
        "--abs",
        dest="abs_target",
        metavar="x,y,z",
        help=(
            "Absolute TCP target position (metres) in arm base frame, "
            "e.g. '0.10,0.00,0.20'."
        ),
    )
    p.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Send motion commands to hardware.  "
            "REQUIRES REAL HARDWARE and owner supervision.  "
            "Default: dry-run (print plan only)."
        ),
    )
    p.add_argument(
        "--dry-run",
        dest="dry_run",
        action="store_true",
        default=False,
        help=(
            "Print the jog plan without commanding any hardware (default behaviour "
            "when --live is not given).  Passing this flag explicitly is equivalent "
            "to omitting --live."
        ),
    )
    p.add_argument(
        "--q-seed",
        metavar="q0,q1,q2,q3,q4",
        default=None,
        help=(
            "IK seed joint angles (rad), comma-separated.  "
            "Default: zeros (URDF rest pose)."
        ),
    )
    p.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Trajectory sample interval (seconds).  Default 0.02 (50 Hz).",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    # --- Load config and build kinematic chain ---
    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    arm_name = args.arm

    # --- Determine current joint configuration ---
    # In dry-run mode we use q=zeros as the "current" configuration.
    # In live mode we would read from the servo bus.
    if args.live:
        # Hardware path: read current joint positions from servo bus.
        # HARDWARE ONLY — never executed in dry-run / mock mode.
        logger.warning(
            "LIVE mode requested — opening servo bus for arm '%s'.", arm_name
        )
        try:
            from inchworm.backends import get_backend
            from inchworm.servo_bus import ServoBusDriver
            backend = get_backend(cfg)
            bus = ServoBusDriver(
                backend=backend,
                arm_cfg=cfg.arms.left if arm_name == "left" else cfg.arms.right,
                arm_role=arm_name,
            )
            q_current = bus.read_joint_positions_rad()
        except Exception as exc:
            logger.error("Failed to read joint positions in live mode: %s", exc)
            return 1
    else:
        logger.info("[DRY-RUN] Using q_current = zeros (5) as current configuration.")
        q_current = np.zeros(5, dtype=np.float64)

    # --- Determine target TCP pose ---
    T_current = chain.fk(q_current)
    current_pos = T_current[:3, 3]

    if args.delta is not None:
        try:
            delta_xyz = parse_xyz(args.delta, "--delta")
        except argparse.ArgumentTypeError as e:
            parser.error(str(e))
        target_pos = current_pos + delta_xyz
    else:
        try:
            target_pos = parse_xyz(args.abs_target, "--abs")
        except argparse.ArgumentTypeError as e:
            parser.error(str(e))

    # Build target HTM (keep current orientation)
    T_target = T_current.copy()
    T_target[:3, 3] = target_pos

    # --- IK ---
    if args.q_seed is not None:
        try:
            seed_vals = [float(v) for v in args.q_seed.split(",")]
            if len(seed_vals) != 5:
                raise ValueError
            q_seed = np.array(seed_vals, dtype=np.float64)
        except ValueError:
            parser.error("--q-seed must be five comma-separated floats")
    else:
        q_seed = None

    ik_result: IKResult = chain.ik(T_target, q_seed=q_seed, position_only=True)

    if not ik_result.reachable:
        logger.error(
            "IK: target position %s is UNREACHABLE.  "
            "pos_err=%.3f mm  ori_err=%.4f deg",
            target_pos,
            ik_result.pos_err_m * 1000,
            np.degrees(ik_result.ori_err_rad),
        )
        return 2

    q_target = ik_result.q

    # --- Collision check (AA-9 hook) ---
    collision_msg = collision_check_hook(arm_name, q_target)
    if collision_msg is not None:
        logger.error("Collision check FAILED: %s", collision_msg)
        return 3

    # --- Plan trajectory ---
    traj = plan_joint_trajectory(
        q_start=q_current,
        q_end=q_target,
        limits=cfg.so101_kinematics.trajectory,
        joint_names=chain.joint_names(),
    )

    # --- Print plan (always, even in live mode) ---
    print()
    print("=" * 60)
    if args.live:
        print("  JOG PLAN  (LIVE MODE — motion will execute)")
    else:
        print("  DRY-RUN JOG PLAN  (no hardware commands)")
    print("=" * 60)
    print(f"  arm          : {arm_name}")
    print(f"  current_pos  : {current_pos}")
    print(f"  target_pos   : {target_pos}")
    print(f"  IK pos_err   : {ik_result.pos_err_m*1000:.4f} mm")
    print(f"  q_current    : {np.round(q_current, 4)}")
    print(f"  q_target     : {np.round(q_target, 4)}")
    print(f"  collision     : {collision_msg or 'none (AA-9 hook pending)'}")
    print()
    print(traj.summary())
    print()
    print("  NOTE: gripper joint is EXCLUDED — commanded only via AA-17/AA-8.")
    print("=" * 60)

    if not args.live:
        print("\n[DRY-RUN complete — no motion commanded]")
        return 0

    # --- Live motion execution ---
    # HARDWARE ONLY — never reaches this point in dry-run mode.
    logger.info("Executing trajectory on arm '%s' (T=%.3f s)...", arm_name, traj.T)
    try:
        from inchworm.backends import get_backend
        from inchworm.servo_bus import ServoBusDriver
        backend = get_backend(cfg)
        bus = ServoBusDriver(
            backend=backend,
            arm_cfg=cfg.arms.left if arm_name == "left" else cfg.arms.right,
            arm_role=arm_name,
        )
        for t, q_step in sample_trajectory(traj, dt=args.dt):
            # Never command the gripper joint (index not in 5-DOF chain)
            bus.write_joint_positions_rad(q_step)
        logger.info("Trajectory complete.")
    except Exception as exc:
        logger.error("Motion execution failed: %s", exc)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
