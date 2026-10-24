#!/usr/bin/env python3
"""Scan the Feetech servo bus on one SO-101 arm and report discovered servos.

This script is the hardware companion for AA-8 AC2 and AC4.
It is **dry-run by default**: without ``--live`` it prints what it *would* do
and returns a simulated empty scan.  The ``--live`` flag opens the real serial
port and commands the bus.

Usage (dry-run, safe — no hardware commanded)::

    uv run python scripts/scan_servos.py --arm left
    uv run python scripts/scan_servos.py --arm right

Usage (live hardware — owner-supervised first run, doc-1 s8)::

    uv run python scripts/scan_servos.py --arm left --live
    uv run python scripts/scan_servos.py --arm right --live [--persist]

Options::

    --arm {left,right}   Which arm to scan (required)
    --live               Open the real serial port (default: dry-run)
    --persist            Write discovered IDs to hardware.yaml (requires --live)
    --id-range START END Servo ID range to probe (default 1 19)
    --timeout S          Per-servo probe timeout in seconds (default 0.05)

AC4 readback rate measurement::

    uv run python scripts/scan_servos.py --arm left --live --benchmark

This adds a 20-iteration joint-state read loop and prints the achieved Hz.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Dict

# Allow running from repo root or from robot/
_ROBOT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROBOT_ROOT / "src"))

from inchworm.config import load_config, DEFAULT_CONFIG_PATH
from inchworm.servo_bus import ArmServoBus, JOINT_NAMES, ServoError


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Scan Feetech servo bus (dry-run by default)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm", choices=["left", "right"], required=True,
                   help="Which arm to scan")
    p.add_argument("--live", action="store_true",
                   help="Open the real serial port (default: dry-run)")
    p.add_argument("--persist", action="store_true",
                   help="Write discovered IDs to hardware.yaml (requires --live)")
    p.add_argument("--id-range", nargs=2, type=int, default=[1, 19],
                   metavar=("START", "END"),
                   help="Servo ID range to probe, inclusive (default: 1 19)")
    p.add_argument("--timeout", type=float, default=0.05,
                   help="Per-servo probe timeout in seconds (default: 0.05)")
    p.add_argument("--benchmark", action="store_true",
                   help="Measure 20-iteration joint-state read rate (requires --live)")
    return p


def _dry_run_report(arm: str, id_range: range) -> None:
    print(f"[DRY-RUN] Would scan arm_{arm} servo IDs {id_range.start}–{id_range.stop - 1}")
    print("[DRY-RUN] No serial port opened; no hardware commanded.")
    print("[DRY-RUN] Pass --live to run against real hardware.")
    print()
    print("Joint names expected (from JOINT_NAMES):")
    for i, name in enumerate(JOINT_NAMES, 1):
        print(f"  joint {i}: {name}")
    print()
    print("After a live scan, update hardware.yaml servo_joints to map")
    print("joint names to the discovered IDs.")


def _live_scan(
    arm: str,
    id_range: range,
    timeout_s: float,
    persist: bool,
    benchmark: bool,
) -> None:
    """Perform the real bus scan."""
    arm_role = f"arm_{arm}"
    device_name = f"arm_{arm}"

    config = load_config()

    import os
    os.environ.setdefault("INCHWORM_BACKEND", "hardware")

    from inchworm.backends import create_backend, BackendError

    try:
        backend = create_backend(config)
    except BackendError as exc:
        print(f"ERROR: cannot create hardware backend: {exc}", file=sys.stderr)
        sys.exit(1)

    try:
        transport = backend.open_serial(device_name)
    except BackendError as exc:
        print(f"ERROR: cannot open serial for {device_name!r}: {exc}", file=sys.stderr)
        sys.exit(1)

    # Build a dummy joint map (scan ignores joint names)
    dummy_map = {name: i + 1 for i, name in enumerate(JOINT_NAMES)}

    bus = ArmServoBus(
        transport,
        role=arm_role,
        joint_id_map=dummy_map,
        timeout_s=timeout_s,
    )

    print(f"Scanning {arm_role} servo IDs {id_range.start}–{id_range.stop - 1} ...")
    found: List[Dict] = bus.scan_bus(id_range=id_range)

    if not found:
        print("No servos responded.")
    else:
        print(f"Found {len(found)} servo(s):")
        for entry in found:
            print(
                f"  ID={entry['servo_id']:3d}  model={entry['model_num']}  "
                f"position={entry['position_deg']:.1f}°"
            )

    if persist and found:
        from inchworm.servo_discovery import scan_and_persist
        # Re-run scan via scan_and_persist (re-scans & writes config)
        print(f"\nPersisting to {DEFAULT_CONFIG_PATH} ...")
        transport2 = backend.open_serial(device_name)
        scan_and_persist(transport2, arm_role=arm_role, id_range=id_range, timeout_s=timeout_s)
        print("Done. Review hardware.yaml and assign joint names to the servo_N entries.")

    if benchmark and found:
        first_id = found[0]["servo_id"]
        print(f"\nBenchmarking read_joint_state for servo ID {first_id} ...")
        ITERATIONS = 20
        start = time.monotonic()
        for _ in range(ITERATIONS):
            try:
                bus.read_joint_state(first_id)
            except ServoError as exc:
                print(f"  read error: {exc}", file=sys.stderr)
                break
        elapsed = time.monotonic() - start
        hz = ITERATIONS / elapsed if elapsed > 0 else float("inf")
        print(f"  {ITERATIONS} iterations in {elapsed:.3f}s → {hz:.1f} Hz")
        if hz >= 20.0:
            print(f"  ✓ AC4 satisfied: {hz:.1f} Hz ≥ 20 Hz")
        else:
            print(f"  ✗ AC4 NOT satisfied: {hz:.1f} Hz < 20 Hz")

    transport.close()


def main() -> None:
    args = _build_parser().parse_args()
    id_range = range(args.id_range[0], args.id_range[1] + 1)

    if not args.live:
        _dry_run_report(args.arm, id_range)
        return

    if args.persist and not args.live:
        print("ERROR: --persist requires --live", file=sys.stderr)
        sys.exit(1)

    _live_scan(
        arm=args.arm,
        id_range=id_range,
        timeout_s=args.timeout,
        persist=args.persist,
        benchmark=args.benchmark,
    )


if __name__ == "__main__":
    main()
