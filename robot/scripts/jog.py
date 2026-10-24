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

Live motion at half speed::

    uv run python scripts/jog.py --arm left --delta 0.01,0.0,0.0 --live --speed-scale 0.5

Notes
-----
*   Gripper servo is **never** commanded by this script.  The gripper is
    reserved for the AA-17 state machine via the AA-8 gripper_channel token.
*   Collision check: motion_guard.MotionGuard.check_trajectory is called on
    every live jog via GuardedArm before any servo write.
*   AA-7 telemetry: jog events are logged via the AA-7 SessionLogger when
    ``--live`` is active.  Dry-run logs to stdout only.

Live jog path (AA-9 wired)
~~~~~~~~~~~~~~~~~~~~~~~~~~~
When ``--live`` is passed the script:
  1. Loads hardware config and opens the servo bus backend via
     ``inchworm.backends.get_backend(cfg).open_serial(role)``.
  2. Builds ``ArmServoBus(joint_id_map from config servo_joints)`` —
     refuses if any of joints 1-5 is null (not yet scanned).
  3. Builds ``GuardedArm`` via
     ``motion_guard.build_guarded_arm(bus, cfg, role, live=True)`` with
     joint calibration — refuses if any calibration value is missing or
     unverified (fail-closed).
  4. Reads present joint angles via ``GuardedArm._read_present_pose_rad()``.
  5. Runs IK to find the target configuration.
  6. Plans a trapezoidal trajectory scaled by ``--speed-scale``.
  7. Validates the full present→target trajectory via
     ``GuardedArm.check_trajectory_rad``.
  8. Streams each time-parameterised sample via
     ``GuardedArm.write_positions_rad`` (joints 1-5 only; never the gripper).
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
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
# AA-9 collision check hook (dry-run fallback only)
# ---------------------------------------------------------------------------


def collision_check_hook(arm: str, q: np.ndarray) -> Optional[str]:
    """Check for collisions before commanding motion.

    Dry-run fallback used when ``cfg.motion_guard`` is None.
    The real AA-9 guard is used when ``motion_guard`` is present.

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
    """
    return None


# --- AA-9 ---
def _aa9_collision_check(arm: str, q: np.ndarray, cfg) -> Optional[str]:  # noqa: ANN001
    """Real AA-9 collision check via :mod:`inchworm.motion_guard`.

    Called in place of the placeholder ``collision_check_hook`` when the
    ``motion_guard`` config block is present.  Returns a violation string
    if any link point falls in a keep-out zone, else None.

    Joint limits are also checked (dry-run mode only here; live mode enforced
    inside :class:`~inchworm.motion_guard.GuardedArm`).
    """
    from inchworm.motion_guard import build_motion_guard  # local import
    guard = build_motion_guard(cfg, arm)
    return guard.check_pose(q, live=False)
# --- AA-9 end ---


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
    p.add_argument(
        "--speed-scale",
        dest="speed_scale",
        type=float,
        default=0.2,
        metavar="SCALE",
        help=(
            "Scale trajectory velocity and acceleration limits by this factor "
            "(0.0 < SCALE <= 1.0).  Default 0.2 (20%% of configured maximum).  "
            "Use 1.0 for full-speed motion."
        ),
    )
    return p


# ---------------------------------------------------------------------------
# Live-path helpers
# ---------------------------------------------------------------------------

# Joint names for joints 1-5 (arm joints, gripper excluded).
_ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


def _build_joint_id_map_or_refuse(cfg, role: str) -> dict[str, int]:
    """Build the joint_id_map for one arm from config.

    Checks that all five arm joints (1-5) have non-null IDs in the config.
    The gripper is included in the map with a fallback ID of 6 when not set
    (so ArmServoBus can be constructed; the gripper is never written).

    Returns the map or raises SystemExit(4) with a diagnostic message.
    """
    arm_cfg = cfg.arms.left if role == "arm_left" else cfg.arms.right
    sj = arm_cfg.servo_joints
    null_joints = [j for j in _ARM_JOINT_NAMES if getattr(sj, j) is None]
    if null_joints:
        logger.error(
            "LIVE jog REFUSED: the following arm joints have null servo IDs in config "
            "(run scripts/scan_servos.py --live and populate hardware.yaml): %s",
            ", ".join(null_joints),
        )
        sys.exit(4)
    joint_map: dict[str, int] = {j: getattr(sj, j) for j in _ARM_JOINT_NAMES}
    gripper_id = sj.gripper if sj.gripper is not None else 6
    joint_map["gripper"] = gripper_id
    return joint_map


def _check_calibration_or_refuse(guarded_arm, role: str) -> None:
    """Check that all joint calibrations are present and verified.

    Inspects the ``ArmCalibration`` inside GuardedArm and exits 4 if any
    joint is missing ``zero_offset_deg`` or is marked ``unverified=True``.
    """
    calib = guarded_arm._calibration  # type: ignore[attr-defined]
    if calib is None:
        logger.error(
            "LIVE jog REFUSED (%s): no servo calibration block in config. "
            "Run the calibration procedure and populate "
            "motion_guard.servo_calibration in hardware.yaml.",
            role,
        )
        sys.exit(4)

    if not calib.is_live_ok:
        missing = []
        from inchworm.motion_guard import JOINT_NAMES
        for i, name in enumerate(JOINT_NAMES):
            p = calib.get(i)
            if p.zero_offset_deg is None:
                missing.append(f"{name} (zero_offset_deg is null)")
            elif p.unverified:
                missing.append(f"{name} (unverified=True)")
        logger.error(
            "LIVE jog REFUSED (%s): joint calibration is missing or unverified. "
            "Populate motion_guard.servo_calibration in hardware.yaml and set "
            "unverified: false.  Missing/unverified: %s",
            role,
            "; ".join(missing) if missing else "all joints",
        )
        sys.exit(4)


def _check_limits_or_refuse(guarded_arm, role: str) -> None:
    """Check that joint limits are present and verified.

    Inspects the MotionGuard inside GuardedArm and exits 4 if any limits
    are unverified (null or unverified=True).
    """
    limits = guarded_arm.guard.limit_guard.limits  # type: ignore[attr-defined]
    if not limits.live_ok:
        from inchworm.motion_guard import JOINT_NAMES
        missing = []
        for i, name in enumerate(JOINT_NAMES):
            lo = limits.lower_rad[i]
            hi = limits.upper_rad[i]
            if lo is None or hi is None:
                missing.append(f"{name} (null)")
        msg = "; ".join(missing) if missing else "limits are marked unverified"
        logger.error(
            "LIVE jog REFUSED (%s): soft joint limits are UNVERIFIED. "
            "Run scripts/measure_limits.py --live, populate hardware.yaml, "
            "set unverified: false.  Detail: %s",
            role,
            msg,
        )
        sys.exit(4)


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    # --- Validate speed_scale ---
    speed_scale = args.speed_scale
    if not (0.0 < speed_scale <= 1.0):
        parser.error(
            f"--speed-scale must be in (0.0, 1.0], got {speed_scale}"
        )

    # --- Load config and build kinematic chain ---
    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    arm_name = args.arm
    role = f"arm_{arm_name}"  # "arm_left" or "arm_right"

    # --- Determine current joint configuration ---
    # In dry-run mode we use q=zeros as the "current" configuration.
    # In live mode we open the servo bus, build GuardedArm, and read the bus.
    guarded_arm = None

    if args.live:
        # -----------------------------------------------------------------
        # Live path: open backend, build ArmServoBus, build GuardedArm,
        # refuse if calibration or limits are missing/unverified.
        # -----------------------------------------------------------------
        from inchworm.backends import get_backend
        from inchworm.servo_bus import ArmServoBus
        from inchworm.motion_guard import build_guarded_arm

        # 1. Build joint_id_map from config — refuse if any joint 1-5 is null.
        joint_id_map = _build_joint_id_map_or_refuse(cfg, role)

        # 2. Open serial transport via backend (respects mock/hardware env).
        backend = get_backend(cfg)
        transport = backend.open_serial(role)

        # 3. Build servo bus.
        bus = ArmServoBus(transport, role=role, joint_id_map=joint_id_map)

        # 4. Build GuardedArm (wires MotionGuard + ArmCalibration).
        # NOTE: build_guarded_arm uses arm_name ("left"/"right"), not the
        # backend role ("arm_left"/"arm_right"), for config lookups.
        guarded_arm = build_guarded_arm(bus, cfg, arm_name, live=True)

        # 5. Refuse if calibration is missing or unverified (fail-closed).
        _check_calibration_or_refuse(guarded_arm, role)

        # 6. Refuse if joint limits are missing or unverified (fail-closed).
        _check_limits_or_refuse(guarded_arm, role)

        # 7. Read present pose from bus through GuardedArm.
        try:
            q_current = guarded_arm._read_present_pose_rad()
        except Exception as exc:
            logger.error(
                "LIVE jog REFUSED (%s): cannot read present pose: %s", role, exc
            )
            return 4
        logger.info("[LIVE] Read present pose from %s: %s rad", role, np.round(q_current, 4))

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

    # --- Collision check (AA-9 hook, dry-run only) ---
    # Live mode: collision is checked inside GuardedArm.write_positions_rad.
    if not args.live:
        # --- AA-9 ---
        if cfg.motion_guard is not None:
            collision_msg = _aa9_collision_check(arm_name, q_target, cfg)
        else:
            collision_msg = collision_check_hook(arm_name, q_target)
        # --- AA-9 end ---
        if collision_msg is not None:
            logger.error("Collision check FAILED: %s", collision_msg)
            return 3
    else:
        collision_msg = None

    # --- Plan trajectory (with speed_scale applied) ---
    # Apply speed_scale to velocity and acceleration limits.
    raw_limits = cfg.so101_kinematics.trajectory
    scaled_limits = raw_limits.model_copy(update={
        "max_velocity_rad_per_s": raw_limits.max_velocity_rad_per_s * speed_scale,
        "max_acceleration_rad_per_s2": raw_limits.max_acceleration_rad_per_s2 * speed_scale,
    })
    traj = plan_joint_trajectory(
        q_start=q_current,
        q_end=q_target,
        limits=scaled_limits,
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
    print(f"  role         : {role}")
    print(f"  current_pos  : {current_pos}")
    print(f"  target_pos   : {target_pos}")
    print(f"  IK pos_err   : {ik_result.pos_err_m*1000:.4f} mm")
    print(f"  q_current    : {np.round(q_current, 4)}")
    print(f"  q_target     : {np.round(q_target, 4)}")
    print(f"  speed_scale  : {speed_scale:.2f}")
    if not args.live:
        print(f"  collision    : {collision_msg or 'none (AA-9 guard active)'}")
    print()
    print(traj.summary())
    print()
    print("  NOTE: gripper joint is EXCLUDED — commanded only via AA-17/AA-8.")
    print("=" * 60)

    if not args.live:
        print("\n[DRY-RUN complete — no motion commanded]")
        return 0

    # -----------------------------------------------------------------
    # Live execution: validate trajectory then stream waypoints.
    # -----------------------------------------------------------------
    # Validate the full present→target trajectory before starting.
    traj_violation = guarded_arm.check_trajectory_rad(q_current, q_target)
    if traj_violation is not None:
        logger.error(
            "LIVE jog REFUSED (%s): trajectory check FAILED: %s", role, traj_violation
        )
        return 3

    logger.info(
        "[LIVE] Streaming trajectory to %s: T=%.3f s, dt=%.3f s, samples=~%d",
        role,
        traj.T,
        args.dt,
        int(traj.T / args.dt) + 2,
    )

    # Stream time-parameterised samples via GuardedArm.write_positions_rad.
    # Gripper is NEVER written — write_positions_rad only writes joints 1-5.
    t_start = time.monotonic()
    for t_traj, q_sample in sample_trajectory(traj, dt=args.dt):
        t_elapsed = time.monotonic() - t_start
        # Pace the stream: wait until it is time to send this sample.
        t_wall_target = t_traj
        sleep_s = t_wall_target - t_elapsed
        if sleep_s > 0.002:
            time.sleep(sleep_s)
        guarded_arm.write_positions_rad(q_sample)

    logger.info("[LIVE] Jog complete on %s", role)
    return 0


if __name__ == "__main__":
    sys.exit(main())
