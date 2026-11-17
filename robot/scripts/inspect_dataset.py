#!/usr/bin/env python3
"""AA-23 AC#10: Dataset inspection tool for local teleop datasets.

Reports:
  - Episode count
  - Duration distribution (min, mean, max, percentiles)
  - Success rate
  - Per-joint start-offset coverage histogram with under-covered bins flagged

LeRobot v3.0 datasets (``record_handoff.py`` output; needs pandas + PyAV, so run it from
``robot/lerobot_env``) report instead: episodes, frames, fps, features, per-episode length,
start pose (frame-0 ``observation.state``), success label, the wrist camera's new-image rate
per episode, and the 360-degree action-offset check (|action - state| per joint).

Usage::

    uv run python scripts/inspect_dataset.py datasets/teleop
    uv run python scripts/inspect_dataset.py --bins 8 robot/datasets/teleop
    cd robot/lerobot_env && uv run python ../scripts/inspect_dataset.py ../datasets/insert_left_<UTC>
    # post-hoc success label (written to meta/episode_labels.json next to LeRobot's meta):
    uv run python ../scripts/inspect_dataset.py ../datasets/insert_left_<UTC> --label 3=failure
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


# --- LeRobot v3.0 datasets -------------------------------------------------------------

#: |action - state| above this (deg) on any frame means the action was saved in another
#: servo turn than the state (the 360-degree offset bug fixed 2026-11-15 in record_handoff).
ACTION_OFFSET_LIMIT_DEG = 180.0
#: A frame is a new camera image when its mean |grey difference| to the previous frame
#: exceeds this (0..255 grey levels); repeats of the same image decode within codec noise.
NEW_IMAGE_THRESHOLD = 0.255
WRIST_VIDEO_KEY = "observation.images.wrist"
LABELS_FILE = "meta/episode_labels.json"
LABELS = ("success", "failure")


def is_lerobot_v3(root: Path) -> bool:
    info = root / "meta" / "info.json"
    return info.exists() and json.loads(info.read_text()).get("codebase_version") == "v3.0"


def read_labels(root: Path) -> dict:
    p = root / LABELS_FILE
    return {int(k): v for k, v in json.loads(p.read_text()).items()} if p.exists() else {}


def write_label(root: Path, episode: int, label: str) -> None:
    if label not in LABELS:
        raise ValueError(f"label must be one of {LABELS}, not {label!r}")
    info = json.loads((root / "meta" / "info.json").read_text())
    if not 0 <= episode < info["total_episodes"]:
        raise ValueError(f"episode {episode} not in 0..{info['total_episodes'] - 1}")
    labels = read_labels(root)
    labels[episode] = label
    (root / LABELS_FILE).write_text(json.dumps({str(k): v for k, v in sorted(labels.items())}, indent=1) + "\n")


def _episodes_table(root: Path):
    import pandas as pd
    files = sorted((root / "meta" / "episodes").glob("chunk-*/file-*.parquet"))
    return pd.concat([pd.read_parquet(f) for f in files]).sort_values("episode_index") if files else None


def _frames_table(root: Path):
    import pandas as pd
    files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
    return pd.concat([pd.read_parquet(f) for f in files]).sort_values("index") if files else None


def _video_new_image_flags(path: Path, threshold: float) -> "tuple[list, list]":
    """(timestamps, is_new) for every decoded frame of one video file."""
    import av
    import numpy as np
    ts, new, prev = [], [], None
    with av.open(str(path)) as c:
        stream = c.streams.video[0]
        for f in c.decode(stream):
            g = f.to_ndarray(format="gray").astype(np.int16)
            ts.append(float(f.pts * stream.time_base))
            new.append(prev is None or float(np.abs(g - prev).mean()) > threshold)
            prev = g
    return ts, new


def lerobot_v3_report(root: Path, threshold: float = NEW_IMAGE_THRESHOLD) -> dict:
    """Facts of a LeRobot v3.0 dataset (no LeRobot import; pandas + PyAV)."""
    import numpy as np
    info = json.loads((root / "meta" / "info.json").read_text())
    rep = {"root": str(root), "codebase_version": info["codebase_version"], "fps": info["fps"],
           "robot_type": info.get("robot_type"), "episodes": info["total_episodes"],
           "frames": info["total_frames"],
           "features": {k: {"dtype": v["dtype"], "shape": list(v["shape"]), "names": v.get("names")}
                        for k, v in info["features"].items()},
           "per_episode": []}
    eps, df = _episodes_table(root), _frames_table(root)
    if eps is None or df is None or not len(eps):
        return rep
    fps = float(info["fps"])
    names = info["features"]["observation.state"]["names"]
    labels = read_labels(root)
    videos: dict = {}
    for e in eps.to_dict("records"):
        i = int(e["episode_index"])
        rows = df[df["episode_index"] == i]
        s, a = np.stack(rows["observation.state"].values), np.stack(rows["action"].values)
        diff = np.abs(a - s)
        ep = {"episode": i, "length": int(e["length"]), "duration_s": round(int(e["length"]) / fps, 2),
              "label": labels.get(i),
              "start_state": {n: round(float(v), 2) for n, v in zip(names, s[0])},
              "action_state_p99_deg": {n: round(float(v), 2) for n, v in zip(names, np.percentile(diff, 99, axis=0))},
              "action_offset_joints": [n for n, v in zip(names, diff.max(axis=0)) if v > ACTION_OFFSET_LIMIT_DEG]}
        vk = f"videos/{WRIST_VIDEO_KEY}"
        if f"{vk}/chunk_index" in e:
            vpath = root / info["video_path"].format(video_key=WRIST_VIDEO_KEY,
                                                    chunk_index=int(e[f"{vk}/chunk_index"]),
                                                    file_index=int(e[f"{vk}/file_index"]))
            if vpath not in videos:
                videos[vpath] = _video_new_image_flags(vpath, threshold)
            ts, new = videos[vpath]
            t0, t1 = float(e[f"{vk}/from_timestamp"]), float(e[f"{vk}/to_timestamp"])
            sel = [n for t, n in zip(ts, new) if t0 - 0.5 / fps <= t < t1 - 0.5 / fps]
            ep["wrist_frames_decoded"] = len(sel)
            ep["wrist_new_images_per_s"] = round(sum(sel) / (len(sel) / fps), 2) if sel else None
        rep["per_episode"].append(ep)
    return rep


def print_lerobot_v3_report(rep: dict) -> None:
    print("=" * 60)
    print(f"LeRobot dataset: {rep['root']}")
    print("=" * 60)
    print(f"Format          : {rep['codebase_version']}  robot_type={rep['robot_type']}")
    print(f"Episode count   : {rep['episodes']}")
    print(f"Frames          : {rep['frames']}")
    print(f"fps             : {rep['fps']}")
    print("Features:")
    for k, v in rep["features"].items():
        print(f"  {k:28s} {v['dtype']:8s} {v['shape']}" + (f" {v['names']}" if k in ("action", "observation.state") else ""))
    if not rep["per_episode"]:
        print("(no episodes)")
        return
    print("\nPer episode:")
    for e in rep["per_episode"]:
        off = e["action_offset_joints"]
        print(f"  ep {e['episode']:3d}: {e['length']:5d} frames {e['duration_s']:7.2f} s  "
              f"label={e['label'] or '-'}  wrist new images/s={e.get('wrist_new_images_per_s')}  "
              f"360-offset={'YES ' + ','.join(off) if off else 'no'}")
        print(f"          start {list(e['start_state'].values())}  |action-state| p99 "
              f"{list(e['action_state_p99_deg'].values())}")
    bad = [e["episode"] for e in rep["per_episode"] if e["action_offset_joints"]]
    labelled = [e for e in rep["per_episode"] if e["label"]]
    print(f"\n360-degree action-offset check: " + (f"FAIL on episodes {bad}" if bad else "pass (all episodes)"))
    if labelled:
        n_ok = sum(e["label"] == "success" for e in labelled)
        print(f"Labels: {n_ok} success / {len(labelled)} labelled of {len(rep['per_episode'])}")
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
    parser.add_argument("--label", action="append", default=[], metavar="EP=LABEL",
                        help="LeRobot v3.0: set a post-hoc label (success|failure) for episode EP; repeatable")
    parser.add_argument("--json", action="store_true", help="LeRobot v3.0: print the report as JSON")
    parser.add_argument("--new-image-threshold", type=float, default=NEW_IMAGE_THRESHOLD,
                        help="mean |grey diff| (0..255) above which a frame is a new camera image")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    if not dataset_root.exists():
        print(f"ERROR: dataset root does not exist: {dataset_root}", file=sys.stderr)
        return 1

    if is_lerobot_v3(dataset_root):
        try:
            for item in args.label:
                ep, _, label = item.partition("=")
                write_label(dataset_root, int(ep), label)
        except ValueError as exc:
            print(f"ERROR: --label: {exc}", file=sys.stderr)
            return 1
        try:
            rep = lerobot_v3_report(dataset_root, args.new_image_threshold)
        except ImportError as exc:
            print(f"ERROR: {exc}; run from robot/lerobot_env (pandas + PyAV)", file=sys.stderr)
            return 1
        if args.json:
            print(json.dumps(rep, indent=1))
        else:
            print_lerobot_v3_report(rep)
        return 0
    if args.label:
        print("ERROR: --label needs a LeRobot v3.0 dataset", file=sys.stderr)
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
