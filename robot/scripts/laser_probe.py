#!/usr/bin/env python3
"""Laser probe script – verifies the LCUS-1 relay protocol against live hardware.

This script is a DRY-RUN by default: it runs the mock backend and prints what
would be sent to the relay. Pass ``--live`` to open the real relay device.

Usage:
    # Dry run (safe, no hardware):
    uv run python scripts/laser_probe.py

    # Live run (requires relay device attached; owner supervision required):
    uv run python scripts/laser_probe.py --live

AC coverage:
    - AC#1: Relay serial protocol (LCUS-1) verified on real hardware with --live.
            (UNCHECKED until owner runs this with --live and confirms LED response)
    - AC#4: SIGINT/SIGTERM handlers shown by KeyboardInterrupt trace.

SIGKILL residual risk (documented per AC#4):
    If the process is SIGKILLed (kill -9), no Python handler runs and the
    relay stays in its last commanded state (laser remains on).
    Mitigation: deploy a hardware watchdog (e.g. mechanical timer outlet or
    a host-side watchdog script that detects process death and cuts USB power
    to the relay hub). The watchdog timeout sets the maximum unattended
    laser-on exposure window.
"""

from __future__ import annotations

import argparse
import sys
import time

from inchworm.backends import MockBackend, create_backend
from inchworm.config import load_config
from inchworm.laser import Lcus1Protocol, LaserDriver, open_laser

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument(
    "--live",
    action="store_true",
    default=False,
    help="Open the real relay device. OWNER SUPERVISION REQUIRED.",
)
parser.add_argument(
    "--on-seconds",
    type=float,
    default=1.0,
    metavar="S",
    help="How long to hold the relay on (default 1.0 s; mock ignores timing).",
)


def main() -> int:
    args = parser.parse_args()
    config = load_config()

    if args.live:
        print("WARNING: --live mode. Real relay will be energised. Owner supervision required.")
        print(f"  Protocol: {Lcus1Protocol.name} (UNVERIFIED)")
        print(f"  ON bytes:  {list(Lcus1Protocol._ON)}")
        print(f"  OFF bytes: {list(Lcus1Protocol._OFF)}")
        backend = create_backend(config)
    else:
        print("DRY-RUN mode (mock backend). No hardware touched.")
        backend = MockBackend(config)

    print(f"Opening laser driver (max_continuous_on_s={config.laser.max_continuous_on_s})...")
    with open_laser(config, backend) as drv:
        print(f"  Sending ON command (duration_s={args.on_seconds}) ...")
        drv.on(duration_s=args.on_seconds)
        print(f"  commanded_on={drv.commanded_on}")

        if args.live:
            print(f"  Holding ON for {args.on_seconds}s ...")
            time.sleep(args.on_seconds)
        else:
            print("  (mock: no sleep in dry-run)")

        print("  Sending OFF command ...")
        drv.off()
        print(f"  commanded_on={drv.commanded_on}")
        print(f"  cumulative_on_s={drv.cumulative_on_s:.3f}")

    if not args.live:
        transport = backend.serial.get("relay")
        if transport:
            print(f"\nMock transport wrote {len(transport.written)} packets:")
            for i, pkt in enumerate(transport.written):
                print(f"  [{i}] {list(pkt)}")
    else:
        print("\nLive run complete. Confirm relay LED toggled OFF.")

    print("\nTo check AC#1, owner must:")
    print("  1. Attach the relay module.")
    print("  2. Run: uv run python scripts/laser_probe.py --live")
    print("  3. Confirm the relay LED turns on, then off.")
    print("  4. Check AC#1 in AA-29 with test output as evidence.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
