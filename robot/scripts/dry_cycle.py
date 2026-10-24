#!/usr/bin/env python3
"""AA-21: Dry crawl cycle runner script.

Default behaviour (no flags): **dry-run mode**.  No serial port is opened;
no hardware is touched.  The script plans the full crawl-cycle sequence,
runs the scripted stub policy, logs structured events, and prints a timeline
summary and per-phase duration breakdown.

Pass ``--live`` to execute real motion on hardware.  This requires:
  - An owner-supervised hardware session.
  - AA-16 calibration complete and hardware.yaml updated.
  - AA-9 joint limits measured and set to ``unverified: false``.
  - Both grippers confirmed LOCKED before the first cycle.
  - ``advance_direction_owner_confirmed: true`` set in cycle_runner config.
  - Physical dry-run (no tube) only — never use ``--live`` with a tube attached.

Pass ``--cycles N`` (default 5) to set the number of consecutive cycles.

AC#7 status
-----------
AC#7 (measured cycle time) is **UNCHECKED** — hardware required.
Planned dry-run durations (from a typical run on a 2 GHz development machine):

  preflight           0.000 s  (instantaneous check)
  unlock_moving       0.050 s  (estimated servo travel)
  retract_moving      0.200 s  (estimated retract travel)
  approach_moving     ~1.421 s (IK + trapezoidal trajectory, lattice index 1)
  supervisor_handoff  0.300 s  (2 stub policy steps × 0.1 s)
  lock_moving         0.050 s  (estimated servo travel)
  advance_bookkeeping 0.000 s  (instantaneous state update)
  fold                0.500 s  (estimated, unverified)
  ─────────────────────────────────────────────────────────────────────
  Total estimated     ~2.521 s / cycle  (wall-clock will vary; unverified)

These are software estimates only.  The hardware baseline must be measured
with ``--live`` on the actual robot.

Safety invariants
-----------------
* Dry-run by default — ``--live`` must be passed explicitly.
* Real serial ports are never opened unless ``--live`` is given.
* The gripper servo is only commanded through AA-17 GripperLockSM.
* All arm motion goes through AA-9 GuardedArm / MotionGuard.
* Live execution fails closed on any unverified config or unchecked preflight.
* Tube must NOT be attached during the dry cycle (no insertion is attempted).

Usage::

    # Dry-run (default — no hardware, no tube)
    uv run python scripts/dry_cycle.py

    # Dry-run with 10 cycles
    uv run python scripts/dry_cycle.py --cycles 10

    # Live (REQUIRES HARDWARE + owner supervision; tube NOT attached)
    uv run python scripts/dry_cycle.py --live

    # Live with 5 cycles
    uv run python scripts/dry_cycle.py --live --cycles 5

needs owner-supervised hardware session (for AC#7):
    uv run python scripts/dry_cycle.py --live --cycles 5
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import List, Optional

# Allow running from project root without installing
_repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_repo_root / "src"))

from inchworm.config import load_config
from inchworm.cycle import (
    CycleFaultError,
    CyclePlan,
    PreflightError,
    SafetyInvariantViolation,
    build_cycle_runner,
)
from inchworm.gripper_lock import GripperState
from inchworm.summarizer import format_timeline

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
)
_LOG = logging.getLogger("dry_cycle")


def _print_cycle_plan(plan: CyclePlan) -> None:
    """Print a formatted summary of one cycle's plan."""
    print(f"\n  ── Cycle {plan.cycle_index} plan ──")
    for ph in plan.phases:
        dur = (
            f"{ph.planned_duration_s:.3f}s"
            if ph.planned_duration_s is not None
            else "  ?    "
        )
        status = f"[{ph.outcome.upper():8s}]"
        note = f"  # {ph.note}" if ph.note else ""
        print(f"    {status} {ph.phase:<22s} {dur:>8s}{note}")
    print(f"  Total planned: {plan.total_planned_duration_s:.3f} s")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AA-21 Dry crawl cycle runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Execute real motion on hardware (requires owner supervision; "
             "NO TUBE ATTACHED). Default: dry-run.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="Number of consecutive dry cycles to execute (default: 5).",
    )
    args = parser.parse_args()

    dry_run = not args.live
    n_cycles = args.cycles

    if n_cycles < 1:
        print("ERROR: --cycles must be >= 1", file=sys.stderr)
        return 1

    mode_label = "DRY-RUN" if dry_run else "LIVE (NO TUBE)"
    print(f"\nAA-21 Dry Crawl Cycle Runner  [{mode_label}]")
    print(f"  cycles: {n_cycles}")
    print(f"  mode:   {'dry-run (no servos commanded)' if dry_run else 'LIVE'}")
    if not dry_run:
        print(
            "\n  *** LIVE MODE ACTIVE ***\n"
            "  • Owner supervision required.\n"
            "  • No tube may be attached.\n"
            "  • Ensure both grippers are in a known state before proceeding.\n"
        )
        ans = input("  Type 'yes' to confirm live execution: ")
        if ans.strip().lower() != "yes":
            print("  Aborted.")
            return 0

    # Load config
    cfg = load_config()

    # Build the cycle runner
    print("\nBuilding cycle runner...")
    try:
        runner = build_cycle_runner(cfg, dry_run=dry_run)
    except ValueError as e:
        print(f"ERROR: cannot build cycle runner: {e}", file=sys.stderr)
        return 1

    # Set up initial gripper states for dry-run
    if dry_run:
        print("Initialising gripper SMs (dry-run: simulating LOCKED holding arm)...")
        runner._right_sm.reset()
        runner._right_sm.set_approach_ready()
        runner._right_sm.lock()
        runner._left_sm.reset()
        print(
            f"  left SM:  {runner._left_sm.state.name}\n"
            f"  right SM: {runner._right_sm.state.name}"
        )
    else:
        # Live: verify both gripper SMs report a known state
        left_state = runner._left_sm.state
        right_state = runner._right_sm.state
        print(f"  left SM:  {left_state.name}")
        print(f"  right SM: {right_state.name}")
        if left_state == GripperState.UNKNOWN or right_state == GripperState.UNKNOWN:
            print(
                "ERROR: one or both gripper SMs are in UNKNOWN state. "
                "Reset them before starting.",
                file=sys.stderr,
            )
            return 1

    # Print planned sequence for cycle 0 (planning only, no servo commands)
    print("\nPlanning cycle 0 (IK + trajectory estimate)...")
    plan_0 = runner.plan_cycle(0)
    _print_cycle_plan(plan_0)
    print(
        f"\n  Note: approach_plan available: {plan_0.approach_plan is not None}"
    )
    if plan_0.approach_plan is not None:
        print(f"  Approach safe: {plan_0.approach_plan.safe}")
        print(f"  Approach provisional: {plan_0.approach_plan.provisional}")

    # Run cycles
    print(f"\nRunning {n_cycles} consecutive {'dry ' if dry_run else 'LIVE '}cycle(s)...")
    t_start = time.monotonic()
    plans: List[CyclePlan] = []
    fault_count = 0

    for i in range(n_cycles):
        t_cycle = time.monotonic()
        print(f"\n  ── Cycle {i} ──  moving={runner.moving_arm}  holding={runner.holding_arm}")
        try:
            plan = runner.run_one_cycle(i)
            elapsed = time.monotonic() - t_cycle
            plans.append(plan)
            print(
                f"  OK  wall_time={elapsed:.3f}s  "
                f"planned={plan.total_planned_duration_s:.3f}s  "
                f"advance_count={runner.frame_model.tube_pose.advance_count}"
            )
        except CycleFaultError as e:
            elapsed = time.monotonic() - t_cycle
            fault_count += 1
            print(
                f"  FAULT  phase={e.phase}  safe={e.safe}  "
                f"wall_time={elapsed:.3f}s\n  {e}",
                file=sys.stderr,
            )
            # Re-raise if not safe
            if not e.safe:
                print("  UNSAFE FAULT — stopping.", file=sys.stderr)
                return 1
            # Continue to next cycle (bookkeeping may be partial)
            plans.append(CyclePlan(
                cycle_index=i,
                moving_arm=runner.moving_arm,
                holding_arm=runner.holding_arm,
                target_lattice_index=0,
            ))
        except SafetyInvariantViolation as e:
            print(f"\n  SAFETY VIOLATION: {e}", file=sys.stderr)
            return 1
        except PreflightError as e:
            print(f"\n  PREFLIGHT ERROR:\n" + "\n".join(f"    • {f}" for f in e.failures),
                  file=sys.stderr)
            return 1

    total_elapsed = time.monotonic() - t_start

    # Summary
    print(f"\n{'='*60}")
    print(f"AA-21 Dry Cycle Run Complete")
    print(f"  Total cycles: {n_cycles}  Faults: {fault_count}")
    print(f"  Wall-clock total: {total_elapsed:.3f} s  "
          f"(avg per cycle: {total_elapsed/n_cycles:.3f} s)")
    print(f"  Final advance_count: {runner.frame_model.tube_pose.advance_count}")

    # Serialize the final frame model state
    fm_dict = runner.frame_model.to_dict()
    print(f"\nFinal FrameModel state (serialized):")
    print(f"  advance_count:  {fm_dict['tube_pose']['advance_count']}")
    print(f"  held_indices:   {fm_dict['lattice_state']}")
    print(f"  tube_x_m:       {fm_dict['tube_pose']['T_chassis_tube'][0][3]:.4f}")
    print(f"  schema_version: {fm_dict['schema_version']}")

    # Per-phase duration breakdown across all successful cycles
    if plans:
        print(f"\nPer-phase planned durations (averaged across {len(plans)} cycles):")
        phase_totals: dict = {}
        phase_counts: dict = {}
        for p in plans:
            for ph in p.phases:
                if ph.planned_duration_s is not None:
                    phase_totals[ph.phase] = (
                        phase_totals.get(ph.phase, 0.0) + ph.planned_duration_s
                    )
                    phase_counts[ph.phase] = phase_counts.get(ph.phase, 0) + 1
        for phase, total in sorted(phase_totals.items()):
            count = phase_counts[phase]
            avg = total / count if count else 0.0
            print(f"  {phase:<22s}  avg {avg:.3f} s  (over {count} cycle(s))")

    if dry_run:
        print(
            "\nNote: all durations are software estimates (unverified).\n"
            "AC#7 requires a hardware session: "
            "uv run python scripts/dry_cycle.py --live --cycles 5"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
