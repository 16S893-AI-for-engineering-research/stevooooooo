#!/usr/bin/env python3
"""AA-23 AC#10: Dataset inspection tool for local teleop datasets.

Reports:
  - Episode count
  - Duration distribution (min, mean, max, percentiles)
  - Success rate
  - Per-joint start-offset coverage histogram with under-covered bins flagged

Usage::

    uv run python scripts/inspect_dataset.py datasets/teleop
    uv run python scripts/inspect_dataset.py --bins 8 robot/datasets/teleop
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

from inchworm.teleop_capture import (
    TELEOP_JOINTS,
    DatasetStats,
    load_dataset_stats,
)


def _ascii_histogram(values: list, bins: int, lo: float, hi: float, width: int = 30) -> str:
    """Return a compact ASCII bar chart string."""
    import math

    if not values or hi - lo < 1e-9:
        return "  (all values identical)"

    bin_width = (hi - lo) / bins
    counts = [0] * bins
    for v in values:
        idx = min(int((v - lo) / bin_width), bins - 1)
        counts[idx] = counts[idx] + 1

    max_count = max(counts) or 1
    lines = []
    for i, c in enumerate(counts):
        bar_lo = lo + i * bin_width
        bar_hi = lo + (i + 1) * bin_width
        bar_len = int(round(c / max_count * width))
        bar = "█" * bar_len
        flag = " ← UNDER-COVERED" if c == 0 else ""
        lines.append(f"  [{bar_lo:+6.2f}, {bar_hi:+6.2f}): {bar:<{width}} ({c}){flag}")
    return "\n".join(lines)


def print_report(stats: DatasetStats, bins: int) -> None:
    """Print a human-readable inspection report to stdout."""
    print("=" * 60)
    print(f"AA-23 Dataset Inspection: {stats.dataset_root}")
    print("=" * 60)
    print(f"Episode count   : {stats.episode_count}")

    total = stats.episode_count
    if total == 0:
        print("(no episodes)")
        return

    # Duration distribution
    durations = stats.durations_s
    import statistics
    print(f"\nDuration (s):")
    print(f"  min  = {min(durations):.2f}")
    print(f"  mean = {statistics.mean(durations):.2f}")
    print(f"  max  = {max(durations):.2f}")
    if len(durations) >= 2:
        print(f"  stdev = {statistics.stdev(durations):.2f}")
    if len(durations) >= 4:
        sorted_d = sorted(durations)
        n = len(sorted_d)
        q1 = sorted_d[n // 4]
        q3 = sorted_d[3 * n // 4]
        print(f"  p25={q1:.2f}  p75={q3:.2f}")

    # Success rate
    print(f"\nLabels:")
    print(f"  success  = {stats.success_count} ({100*stats.success_count/total:.1f}%)")
    print(f"  failure  = {stats.failure_count} ({100*stats.failure_count/total:.1f}%)")
    print(f"  aborted  = {stats.aborted_count} ({100*stats.aborted_count/total:.1f}%)")
    success_rate = stats.success_count / total if total else 0.0
    print(f"  success rate = {100*success_rate:.1f}%")

    # Per-joint offset coverage histogram
    print(f"\nStart-offset coverage ({bins} bins per joint):")
    for joint in TELEOP_JOINTS:
        values = [float(off.get(joint, 0.0)) for off in stats.start_offsets if off]
        flag = " *** UNDER-COVERED ***" if joint in stats.under_covered_joints else ""
        print(f"\n  {joint}{flag}")
        if values:
            lo, hi = min(values), max(values)
            print(_ascii_histogram(values, bins, lo, hi))
        else:
            print("  (no data)")

    if stats.under_covered_joints:
        print(f"\n⚠ Under-covered joints: {', '.join(stats.under_covered_joints)}")
        print("  Consider recording more episodes to improve offset coverage.")
    else:
        print(f"\n✓ All joints covered across {bins} bins.")
    print("=" * 60)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AA-23 AC#10: Inspect a local teleop dataset."
    )
    parser.add_argument("dataset_root", help="Path to the dataset root directory.")
    parser.add_argument(
        "--bins",
        type=int,
        default=5,
        help="Number of bins for per-joint offset histogram (default 5).",
    )
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    if not dataset_root.exists():
        print(f"ERROR: dataset root does not exist: {dataset_root}", file=sys.stderr)
        return 1

    try:
        stats = load_dataset_stats(dataset_root, offset_bins=args.bins)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print_report(stats, bins=args.bins)
    return 0


if __name__ == "__main__":
    sys.exit(main())
