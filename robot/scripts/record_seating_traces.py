#!/usr/bin/env python3
"""AA-24: Record and fit seating-detector traces.

Usage
-----
Record labelled traces (dry-run default):

    python scripts/record_seating_traces.py record --label seated --out dataset.jsonl
    python scripts/record_seating_traces.py record --label not_seated --out dataset.jsonl

Record in live mode (requires --live flag and owner supervision):

    python scripts/record_seating_traces.py record --label seated --out dataset.jsonl --live

Fit thresholds from a labelled dataset:

    python scripts/record_seating_traces.py fit dataset.jsonl

Safety
------
- Dry-run by default: no serial ports are opened.
- Live mode requires --live and opens the servo bus; the arm must be
  positioned manually at or away from the attachment point before each
  recording.
- No gripper commands are issued; this script only reads sensor state.
- AGENTS.md: one camera at a time; no camera is used here.

JSON-lines format
-----------------
Each line is a JSON object with keys:
  {
    "label": "seated" | "not_seated",
    "samples": [
      {
        "servo_id": int,
        "joint_name": str,
        "timestamp_s": float,
        "position_deg": float,
        "load_raw": int,
        "current_raw": int,
        "speed_raw": int
      },
      ...
    ]
  }
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List

# Ensure inchworm package is importable when run from the repo root.
_HERE = Path(__file__).resolve().parent
_ROBOT_SRC = _HERE.parent / "src"
if str(_ROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(_ROBOT_SRC))

from inchworm.seating import (
    JointStateSample,
    LabelledTrace,
    SeatingDetectorConfig,
    ThresholdFitter,
)


# ---------------------------------------------------------------------------
# Dry-run mock: generates synthetic samples for testing the record pipeline
# ---------------------------------------------------------------------------


def _generate_dry_run_samples(
    label: str,
    n_samples: int = 20,
    servo_id: int = 6,
) -> List[JointStateSample]:
    """Generate synthetic samples for dry-run mode.

    Seated: position near 150 deg, load_raw rises to ~300.
    Not-seated: position near 90 deg, load_raw stays low (~20).
    """
    import math
    import random

    samples = []
    t0 = time.monotonic()
    for i in range(n_samples):
        ts = t0 + i * 0.02  # 20 ms intervals
        if label == "seated":
            pos = 150.0 + random.gauss(0, 0.5)
            # Load ramps up then levels
            peak_load = int(300 + random.gauss(0, 10))
            ramp = int(peak_load * min(1.0, (i + 1) / (n_samples * 0.6)))
            load = ramp + int(random.gauss(0, 5))
        else:
            pos = 90.0 + random.gauss(0, 2.0)
            load = int(20 + random.gauss(0, 5))
        samples.append(
            JointStateSample(
                servo_id=servo_id,
                joint_name="gripper",
                timestamp_s=ts,
                position_deg=max(0.0, min(360.0, pos)),
                load_raw=load,
                current_raw=abs(load) // 2,
                speed_raw=0,
            )
        )
    return samples


# ---------------------------------------------------------------------------
# Live recording (requires --live)
# ---------------------------------------------------------------------------


def _record_live(
    label: str,
    n_samples: int,
    out_path: Path,
    arm_role: str,
) -> None:
    """Record live samples from the servo bus and append to dataset.

    Opens the mock backend if hardware is not available; the owner must
    set INCHWORM_BACKEND=hardware or configure the device to get real data.
    """
    from inchworm.config import load_config
    from inchworm.backends import get_backend
    from inchworm.servo_bus import ArmServoBus

    cfg = load_config()
    backend = get_backend(cfg)
    transport = backend.open_serial(arm_role)

    role_cfg = cfg.arms.left if arm_role == "arm_left" else cfg.arms.right
    joint_map = {
        k: v for k, v in role_cfg.servo_joints.model_dump().items() if v is not None
    }
    # Filter: only gripper ID needed (servo_id = joint_map.get("gripper"))
    gripper_id = joint_map.get("gripper")
    if gripper_id is None:
        print(
            "WARNING: gripper servo ID is null in hardware.yaml — reading all joints.",
            file=sys.stderr,
        )
        # Fall through and read all joints
    bus = ArmServoBus(transport, role=arm_role, joint_id_map=joint_map)

    print(
        f"Recording {n_samples} samples for label={label!r} on {arm_role}...",
        file=sys.stderr,
    )
    samples: List[JointStateSample] = []
    for i in range(n_samples):
        t0 = time.monotonic()
        all_states = bus.read_all_joint_states()
        target_state = all_states.get("gripper")
        if target_state is None and all_states:
            target_state = next(iter(all_states.values()))
        if target_state is None:
            print(f"  sample {i}: no joint state — skipping", file=sys.stderr)
            time.sleep(0.02)
            continue
        sample = JointStateSample(
            servo_id=target_state.servo_id,
            joint_name="gripper",
            timestamp_s=t0,
            position_deg=target_state.position_deg,
            load_raw=target_state.load_raw,
            current_raw=target_state.current_raw,
            speed_raw=target_state.speed_raw,
        )
        samples.append(sample)
        elapsed = time.monotonic() - t0
        sleep_s = max(0.0, 0.02 - elapsed)
        time.sleep(sleep_s)

    trace = LabelledTrace(label=label, samples=samples)
    _append_trace(out_path, trace)
    print(
        f"Appended {len(samples)} samples (label={label!r}) to {out_path}",
        file=sys.stderr,
    )


def _append_trace(path: Path, trace: LabelledTrace) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(trace.to_dict()) + "\n")


# ---------------------------------------------------------------------------
# Load dataset
# ---------------------------------------------------------------------------


def _load_dataset(path: Path) -> List[LabelledTrace]:
    traces = []
    with path.open("r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
                traces.append(LabelledTrace.from_dict(d))
            except (json.JSONDecodeError, KeyError, TypeError) as exc:
                print(f"WARNING: line {lineno} parse error: {exc}", file=sys.stderr)
    return traces


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _cmd_record(args: argparse.Namespace) -> None:
    out_path = Path(args.out)
    if args.live:
        print(
            "LIVE MODE: opening servo bus. Ensure the arm is positioned correctly.",
            file=sys.stderr,
        )
        _record_live(
            label=args.label,
            n_samples=args.n_samples,
            out_path=out_path,
            arm_role=args.arm,
        )
    else:
        print(
            "DRY-RUN MODE: generating synthetic samples (pass --live for real data).",
            file=sys.stderr,
        )
        samples = _generate_dry_run_samples(label=args.label, n_samples=args.n_samples)
        trace = LabelledTrace(label=args.label, samples=samples)
        _append_trace(out_path, trace)
        print(
            f"Appended {len(samples)} synthetic samples (label={args.label!r}) to {out_path}",
            file=sys.stderr,
        )


def _cmd_fit(args: argparse.Namespace) -> None:
    path = Path(args.dataset)
    traces = _load_dataset(path)
    seated = [t for t in traces if t.label == "seated"]
    not_seated = [t for t in traces if t.label == "not_seated"]
    print(f"Loaded {len(traces)} traces: {len(seated)} seated, {len(not_seated)} not_seated")

    fitter = ThresholdFitter(fn_cap=args.fn_cap, test_fraction=args.test_fraction)
    result = fitter.fit(traces)

    print("\n--- Fit Result ---")
    print(f"  best_load_threshold:  {result.best_load_threshold}")
    print(f"  false_positive_rate:  {result.fp_rate:.3f}  (serious failure)")
    print(f"  false_negative_rate:  {result.fn_rate:.3f}")
    print(f"  train: {result.n_train_seated} seated, {result.n_train_not_seated} not_seated")
    print(f"  test:  {result.n_test_seated} seated, {result.n_test_not_seated} not_seated")
    print(f"  thresholds_tried: {result.thresholds_tried}")

    if result.best_load_threshold is not None:
        print(
            f"\nSuggested hardware.yaml update:\n"
            f"  seating_detector:\n"
            f"    load_rise_threshold_raw: {result.best_load_threshold}"
        )
    else:
        print("\nInsufficient data to fit threshold — record more labelled traces.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="AA-24: Record and fit seating-detector traces."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # record sub-command
    rec = sub.add_parser("record", help="Record labelled traces")
    rec.add_argument(
        "--label",
        required=True,
        choices=["seated", "not_seated"],
        help="Label for the recorded traces",
    )
    rec.add_argument(
        "--out",
        default="seating_traces.jsonl",
        help="Output JSON-lines file (appends)",
    )
    rec.add_argument(
        "--n-samples",
        type=int,
        default=30,
        dest="n_samples",
        help="Number of samples per trace",
    )
    rec.add_argument(
        "--arm",
        default="arm_left",
        choices=["arm_left", "arm_right"],
        help="Arm role to read from (live mode only)",
    )
    rec.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Open the real servo bus. "
            "OWNER SUPERVISION REQUIRED. "
            "Default: dry-run (synthetic data)."
        ),
    )

    # fit sub-command
    fit = sub.add_parser("fit", help="Fit thresholds from labelled dataset")
    fit.add_argument("dataset", help="Path to JSON-lines dataset file")
    fit.add_argument(
        "--fn-cap",
        type=float,
        default=0.30,
        dest="fn_cap",
        help="Maximum acceptable false-negative rate (default 0.30)",
    )
    fit.add_argument(
        "--test-fraction",
        type=float,
        default=0.20,
        dest="test_fraction",
        help="Fraction of each class to hold out for testing (default 0.20)",
    )

    args = parser.parse_args()
    if args.command == "record":
        _cmd_record(args)
    elif args.command == "fit":
        _cmd_fit(args)


if __name__ == "__main__":
    main()
