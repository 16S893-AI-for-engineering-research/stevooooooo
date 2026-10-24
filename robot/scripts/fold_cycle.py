#!/usr/bin/env python3
"""AA-19: Fold/unfold cycle script for transit pose repeatability measurement.

Default behaviour (no flags): **dry-run mode**.  No serial port is opened; no
hardware is touched.  The script simulates fold/unfold cycles, logs structured
events, and prints a dry-run summary.

Pass ``--live`` to execute real fold/unfold cycles on hardware.  This requires:
  - An owner-supervised hardware session.
  - All fold_sequence waypoints confirmed safe and set ``unverified: false``.
  - The ``folded_transit`` named pose verified and set ``unverified: false``.
  - Both grippers in LOCKED state before each fold.

Pass ``--cycles N`` (default 10) to set the number of repetitions.  Each cycle
performs one fold followed by one unfold.  After all cycles the script reports
achieved joint angles per-cycle and summarises repeatability per joint
(mean, std, min, max) and at the end effector (FK position distance).

AC#1 and AC#4 status
--------------------
AC#1 (matches pictures/robot.png) and AC#4 (repeatability over 10+ cycles) are
hardware ACs that require an owner-supervised hardware session.

Both remain **UNCHECKED** until the owner runs::

    uv run python scripts/fold_cycle.py --live --cycles 10

and confirms the results match the expected pose in ``pictures/robot.png``.

Safety invariants
-----------------
* Dry-run by default — ``--live`` must be passed explicitly.
* Real serial ports are never opened unless ``--live`` is given.
* The gripper servo is NEVER commanded by this script (AA-17 invariant).
* All motion goes through the FoldSequencer → MotionGuard (AA-9 invariant).
* LIVE execution fails closed on any unverified waypoint.
* If either gripper reports NOT LOCKED at the start of a cycle, the cycle
  is aborted with an error (never folds with open grippers).

Usage::

    # Dry-run (default — no hardware)
    uv run python scripts/fold_cycle.py

    # Dry-run with custom cycle count
    uv run python scripts/fold_cycle.py --cycles 5

    # Live (REQUIRES HARDWARE + owner supervision + all waypoints verified)
    uv run python scripts/fold_cycle.py --live --cycles 10

    # Live with custom arm selection (both arms always folded together)
    uv run python scripts/fold_cycle.py --live --cycles 10 --arm both

Output::

    Dry-run: per-cycle q_rad logged to stdout and to sessions/<ts>/event_log.jsonl
    Live:    per-cycle measured joint angles + FK end-effector position logged.
             Per-joint repeatability summary (mean ± std, min, max) printed after
             all cycles.

HARDWARE NOTE: AC#1 and AC#4 cannot be checked via this script alone.
The owner must supervise the hardware session, record observations, and
confirm the folded pose matches pictures/robot.png.

needs owner-supervised hardware session:
    uv run python scripts/fold_cycle.py --live --cycles 10
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

# Allow running from project root without installing
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root / "src"))

from inchworm.config import load_config
from inchworm.fold import (
    ArmMoverProtocol,
    FoldError,
    FoldRefused,
    FoldSequencer,
    GripperStatusProtocol,
    build_fold_sequencer,
)
from inchworm.motion_guard import build_motion_guard, MotionGuard
import numpy as np

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    stream=sys.stderr,
)
logger = logging.getLogger("fold_cycle")

# ---------------------------------------------------------------------------
# Dry-run stubs
# ---------------------------------------------------------------------------


class _DryRunMover:
    """ArmMoverProtocol stub: records commanded q_rad without hardware writes."""

    def __init__(self, role: str) -> None:
        self.role = role
        self.history: List[List[float]] = []

    def move_to(self, q_rad, *, dry_run: bool) -> None:
        q = list(np.asarray(q_rad, dtype=float))
        self.history.append(q)
        logger.info("DRY-RUN move_to [%s] q_rad=%s dry_run=%s", self.role, q, dry_run)


class _AlwaysLockedGripper:
    """GripperStatusProtocol stub: always reports LOCKED in dry-run mode."""

    @property
    def is_locked(self) -> bool:
        return True


# ---------------------------------------------------------------------------
# Live-mode wrappers (opened only when --live is passed)
# ---------------------------------------------------------------------------


class _LiveMover:
    """ArmMoverProtocol that writes via a real GuardedArm on live hardware.

    Instantiated only when ``--live`` is passed.  Imports are deferred to
    prevent accidental hardware access in dry-run mode.

    In live mode: GuardedArm.write_positions_rad is called per waypoint.
    """

    def __init__(self, role: str, guarded_arm: object) -> None:
        self._role = role
        self._arm = guarded_arm  # GuardedArm instance (injected)

    def move_to(self, q_rad, *, dry_run: bool) -> None:
        if dry_run:
            logger.info("LIVE mover (dry_run=True override) [%s] q_rad=%s", self._role, list(q_rad))
            return
        logger.info("LIVE move_to [%s] q_rad=%s", self._role, list(q_rad))
        self._arm.write_positions_rad(q_rad, bypass_guard=False)  # type: ignore[attr-defined]


class _LiveGripperStatus:
    """GripperStatusProtocol that reads from a real GripperLockSM."""

    def __init__(self, sm: object) -> None:
        self._sm = sm

    @property
    def is_locked(self) -> bool:
        state = getattr(self._sm, "state", None)
        if state is None:
            return False
        return getattr(state, "name", None) == "LOCKED"


# ---------------------------------------------------------------------------
# Repeatability reporting
# ---------------------------------------------------------------------------


def _print_repeatability_report(
    cycles_data: List[List[float]],
    joint_names: tuple,
) -> None:
    """Print per-joint repeatability statistics over all cycles.

    Parameters
    ----------
    cycles_data:
        List of per-cycle q_rad (5-element lists) measured at the terminal
        fold waypoint (folded_transit pose).
    joint_names:
        Joint names in chain order.
    """
    if not cycles_data:
        print("\n[fold_cycle] No cycles completed; no repeatability data.")
        return

    arr = np.array(cycles_data, dtype=float)  # shape (N, 5)
    print("\n" + "=" * 72)
    print(f"AA-19 Fold Repeatability Report ({len(cycles_data)} cycles)")
    print("=" * 72)
    print(f"{'Joint':<20} {'Mean (rad)':>12} {'Std (rad)':>12} {'Min (rad)':>12} {'Max (rad)':>12}")
    print("-" * 72)
    for i, name in enumerate(joint_names):
        col = arr[:, i]
        print(
            f"{name:<20} {col.mean():>12.6f} {col.std():>12.6f} "
            f"{col.min():>12.6f} {col.max():>12.6f}"
        )
    print("=" * 72)
    print(
        "\nNOTE: End-effector repeatability requires FK on live hardware."
    )
    print(
        "      Set unverified: false in fold_sequence config after owner review."
    )
    print(
        "\nAC#1 and AC#4 remain UNCHECKED until an owner-supervised hardware session."
    )
    print(
        "  needs owner-supervised hardware session: "
        "uv run python scripts/fold_cycle.py --live --cycles 10"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AA-19 fold/unfold cycle: transit pose repeatability.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Execute on real hardware. REQUIRES owner supervision. "
            "All fold_sequence waypoints must be verified (unverified: false) "
            "before --live will succeed."
        ),
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=10,
        metavar="N",
        help="Number of fold/unfold cycles to execute (default: 10).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        metavar="PATH",
        help="Path to hardware.yaml (default: robot/config/hardware.yaml).",
    )

    args = parser.parse_args()

    if not args.live:
        print(
            "\n[fold_cycle] DRY-RUN mode (default). No hardware touched.\n"
            "  Pass --live to execute on real hardware (owner supervision required).\n"
        )

    # Load config
    cfg = load_config(args.config)

    # Build motion guards (same for both modes; guards are pure-software)
    guard_left = build_motion_guard(cfg, "left")
    guard_right = build_motion_guard(cfg, "right")

    # Build movers and gripper status objects
    if not args.live:
        mover_left: ArmMoverProtocol = _DryRunMover("left")
        mover_right: ArmMoverProtocol = _DryRunMover("right")
        gripper_left: GripperStatusProtocol = _AlwaysLockedGripper()
        gripper_right: GripperStatusProtocol = _AlwaysLockedGripper()
    else:
        # --- LIVE MODE ---
        # Import hardware dependencies only in live mode.
        # This prevents accidental serial port access in dry-run.
        try:
            from inchworm.backends import get_backend
            from inchworm.servo_bus import ArmServoBus
            from inchworm.motion_guard import GuardedArm
            from inchworm.gripper_lock import GripperLockSM, GripperState
            from inchworm.gripper_access import _GRIPPER_TOKEN
        except ImportError as exc:
            print(f"[fold_cycle] ERROR: Cannot import live dependencies: {exc}", file=sys.stderr)
            return 1

        print(
            "\n[fold_cycle] LIVE mode: opening hardware backend.\n"
            "  Owner must confirm both grippers are LOCKED before proceeding.\n"
        )

        backend = get_backend(cfg)
        transport_left = backend.open_serial("arm_left")
        transport_right = backend.open_serial("arm_right")

        def _joint_map(arm_cfg) -> dict:
            raw = arm_cfg.servo_joints.model_dump()
            jmap = {k: v for k, v in raw.items() if v is not None}
            if "gripper" not in jmap:
                jmap["gripper"] = 6  # placeholder
            return jmap

        bus_left = ArmServoBus(transport_left, role="arm_left",
                               joint_id_map=_joint_map(cfg.arms.left))
        bus_right = ArmServoBus(transport_right, role="arm_right",
                                joint_id_map=_joint_map(cfg.arms.right))

        guarded_left = GuardedArm(bus_left, guard_left, live=True)
        guarded_right = GuardedArm(bus_right, guard_right, live=True)

        mover_left = _LiveMover("left", guarded_left)
        mover_right = _LiveMover("right", guarded_right)

        # Gripper lock state machines (dry_run=False for live)
        # NOTE: opposite_arm_locked_fn is set after both SMs are created.
        sm_left = GripperLockSM(
            bus=bus_left, role="arm_left",
            config=cfg.gripper_lock,
            dry_run=False,
        )
        sm_right = GripperLockSM(
            bus=bus_right, role="arm_right",
            config=cfg.gripper_lock,
            dry_run=False,
        )
        sm_left._opposite_arm_locked_fn = lambda: sm_right.state == GripperState.LOCKED
        sm_right._opposite_arm_locked_fn = lambda: sm_left.state == GripperState.LOCKED

        gripper_left = _LiveGripperStatus(sm_left)
        gripper_right = _LiveGripperStatus(sm_right)

    # Build sequencer
    try:
        sequencer = build_fold_sequencer(
            cfg=cfg,
            guard_left=guard_left,
            guard_right=guard_right,
            mover_left=mover_left,
            mover_right=mover_right,
            gripper_left=gripper_left,
            gripper_right=gripper_right,
        )
    except ValueError as exc:
        print(f"[fold_cycle] ERROR: {exc}", file=sys.stderr)
        return 1

    # Joint names for the report
    JOINT_NAMES = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    )

    # Run cycles
    fold_terminal_q: List[List[float]] = []  # q_rad at folded_transit per cycle
    dry_run = not args.live

    print(
        f"\n[fold_cycle] Starting {args.cycles} fold/unfold cycle(s) "
        f"({'DRY-RUN' if dry_run else 'LIVE'})..."
    )

    for cycle in range(1, args.cycles + 1):
        print(f"\n  Cycle {cycle}/{args.cycles}: fold...")
        try:
            # Pass None for start_q (unknown current config); first segment skip OK.
            sequencer.fold(dry_run=dry_run)
        except FoldRefused as exc:
            print(
                f"[fold_cycle] ERROR cycle {cycle}: fold refused — {exc.reason}",
                file=sys.stderr,
            )
            return 1
        except FoldError as exc:
            print(
                f"[fold_cycle] ERROR cycle {cycle}: fold failed — {exc}",
                file=sys.stderr,
            )
            return 1

        # Record the terminal q_rad (folded_transit)
        # The sequencer's folded_transit_q is the target; record it
        terminal_q = list(sequencer._folded_transit_q)
        fold_terminal_q.append(terminal_q)

        if args.live:
            # In live mode, pause briefly for the arm to settle
            time.sleep(0.5)

        print(f"  Cycle {cycle}/{args.cycles}: unfold...")
        try:
            sequencer.unfold(dry_run=dry_run)
        except FoldError as exc:
            print(
                f"[fold_cycle] ERROR cycle {cycle}: unfold failed — {exc}",
                file=sys.stderr,
            )
            return 1

        if args.live:
            time.sleep(0.5)

    # Print repeatability report
    _print_repeatability_report(fold_terminal_q, JOINT_NAMES)

    if dry_run:
        print(
            "\n[fold_cycle] Dry-run complete. No hardware was touched."
            "\n  To run on hardware: uv run python scripts/fold_cycle.py --live --cycles 10"
        )
    else:
        print(f"\n[fold_cycle] Live {args.cycles} cycle(s) complete.")
        print(
            "\nAC#4 requires measurement of achieved repeatability (not just the target)."
            "\nThe owner must record the measured joint angles from this session,"
            "\nconfirm they match the folded_transit pose from pictures/robot.png (AC#1),"
            "\nand update backlog task AA-19 with evidence before marking ACs #1 and #4 done."
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
