"""CLI entry point for device discovery.

Usage::

    python -m inchworm.discover          # human-readable table
    python -m inchworm.discover --json   # machine-readable JSON

Or via the installed console script::

    inchworm-discover
    inchworm-discover --json
"""

from __future__ import annotations

import argparse
import sys

from inchworm.config import load_config
from inchworm.discovery import DeviceStatus, discover, format_human, format_json


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="inchworm-discover",
        description="Discover inchworm robot USB devices and report their status.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit machine-readable JSON instead of the human-readable table.",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to hardware.yaml (default: robot/config/hardware.yaml or $INCHWORM_CONFIG).",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run discovery and print results.  Returns an exit code."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    config = load_config(args.config)
    results = discover(config)

    if args.json:
        print(format_json(results))
    else:
        print(format_human(results))

    # Exit 1 if any device is not attached or absent (NO_UDEV_RULE is a soft warning).
    bad = [
        r for r in results
        if r.status in (DeviceStatus.ABSENT, DeviceStatus.NOT_ATTACHED)
    ]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
