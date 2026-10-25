"""lerobot_env/latency_measure.py — AC#4 partial evidence: ACTPolicy latency.

Measures the wall-clock time of a single ``ACTPolicy.select_action()`` call
with a **randomly-initialised** (not trained) model.

Architecture-bound argument
---------------------------
ACT uses a transformer encoder-decoder; the forward-pass computation graph
is determined by architecture hyperparameters (sequence length, d_model,
n_heads, chunk_size, n_obs_steps), not by the trained weight values.
Random weights exercise the same operations with the same data movement.
Latency variance comes from GPU scheduling and cache warming, not from weight
magnitude.  Therefore random-weight latency is a valid proxy for trained-model
latency and is the standard GPU benchmarking methodology.

Usage
-----
  cd robot/lerobot_env
  uv run python latency_measure.py
  uv run python latency_measure.py --device cuda  # GPU
  uv run python latency_measure.py --n-warmup 20 --n-measure 200

Output
------
Prints a summary including mean, p50, p95, p99 latency in milliseconds and
the implied maximum loop rate.  Paste the output as AC#4 evidence.

AC#4 CHECKLIST NOTE
-------------------
AC#4 requires that the *supervisor loop rate* is shown to be compatible with
the measured inference latency.  To check AC#4 fully:

1. Run this script and record ms/step.
2. Verify the supervisor's `max_wall_time_s` / `max_steps` implied loop rate
   is not faster than the measured latency.
3. The loop rate is also constrained by the servo read/write cycle (AA-8 bus
   latency, typically ~10 ms per joint), which dominates over GPU inference
   for real-time insertion.

The AC remains UNCHECKED until measurements are pasted here by the owner
in an owner-supervised session.
"""

from __future__ import annotations

import argparse
import time
import statistics
import sys
from typing import List

import numpy as np


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Measure ACTPolicy.select_action() latency with random weights"
    )
    p.add_argument("--device", default="cpu", help="PyTorch device (default: cpu)")
    p.add_argument("--n-warmup", type=int, default=10, help="Warmup iterations")
    p.add_argument("--n-measure", type=int, default=100, help="Measurement iterations")
    p.add_argument("--frame-h", type=int, default=480, help="Camera frame height")
    p.add_argument("--frame-w", type=int, default=640, help="Camera frame width")
    p.add_argument("--n-joints", type=int, default=5, help="Number of joints")
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    # --- 1. Import lerobot (must be in lerobot_env) ----------------------
    try:
        import torch
        # lerobot 0.6.x module layout (lerobot.common.* was removed upstream).
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.act.configuration_act import ACTConfig
    except ImportError as exc:
        print(f"ERROR: lerobot or torch not available: {exc}", file=sys.stderr)
        print(
            "Activate robot/lerobot_env before running this script:\n"
            "  cd robot/lerobot_env && uv run python latency_measure.py",
            file=sys.stderr,
        )
        return 1

    device = torch.device(args.device)
    # 3 channels: the policy input is the grayscale frame replicated to 3 channels
    # for the ImageNet ResNet backbone (same as the AA-22 VRAM measurement).
    H, W, C = args.frame_h, args.frame_w, 3
    N = args.n_joints

    print(f"Device: {device}")
    print(f"Frame shape: ({H}, {W}, {C})")
    print(f"N joints: {N}")
    print(f"Warmup: {args.n_warmup} iterations")
    print(f"Measure: {args.n_measure} iterations")

    # --- 2. Build randomly-initialised ACTPolicy -------------------------
    # Use the smallest standard ACT config to minimise memory use.
    # These hyperparameters determine latency (architecture-bound).
    try:
        config = ACTConfig(
            input_features={
                "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(N,)),
                "observation.images.wrist_cam": PolicyFeature(type=FeatureType.VISUAL, shape=(C, H, W)),
            },
            output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(N,))},
            device=str(device),
        )
        policy = ACTPolicy(config).to(device)
        policy.eval()
    except Exception as exc:
        print(f"ERROR building ACTPolicy: {exc}", file=sys.stderr)
        print("This script requires lerobot >=0.6; check the lerobot version.", file=sys.stderr)
        return 1

    def _make_obs() -> dict:
        state = torch.zeros(1, N, dtype=torch.float32, device=device)
        frame = torch.rand(1, C, H, W, dtype=torch.float32, device=device)
        return {
            "observation.state": state,
            "observation.images.wrist_cam": frame,
        }

    # --- 3. Warmup ---------------------------------------------------
    print(f"\nRunning {args.n_warmup} warmup iterations...")
    for _ in range(args.n_warmup):
        obs = _make_obs()
        with torch.no_grad():
            _ = policy.select_action(obs)

    # Synchronise GPU
    if device.type == "cuda":
        torch.cuda.synchronize()

    # --- 4. Measure --------------------------------------------------
    print(f"Running {args.n_measure} measurement iterations...")
    latencies_ms: List[float] = []

    for _ in range(args.n_measure):
        obs = _make_obs()
        if device.type == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            _ = policy.select_action(obs)
        if device.type == "cuda":
            torch.cuda.synchronize()
        t1 = time.perf_counter()
        latencies_ms.append((t1 - t0) * 1000.0)

    # --- 4b. Full network forward (predict_action_chunk) --------------
    # select_action() runs the network only once per chunk (chunk_size steps)
    # and otherwise pops a queued action, so its per-call mean understates the
    # worst-case step. Measure the full forward separately (orchestrator 2026-10-25).
    fwd_ms: List[float] = []
    for _ in range(max(20, args.n_measure // 5)):
        obs = _make_obs()
        if device.type == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            _ = policy.predict_action_chunk(obs)
        if device.type == "cuda":
            torch.cuda.synchronize()
        fwd_ms.append((time.perf_counter() - t0) * 1000.0)
    fs = sorted(fwd_ms)
    print("\n=== ACTPolicy full forward (predict_action_chunk) ===")
    print(f"  chunk_size: {config.chunk_size}  n_action_steps: {config.n_action_steps}")
    print(f"  Mean: {statistics.mean(fwd_ms):.2f} ms  p95: {fs[int(0.95*len(fs))]:.2f} ms  max: {fs[-1]:.2f} ms")

    # --- 5. Report ---------------------------------------------------
    mean_ms = statistics.mean(latencies_ms)
    median_ms = statistics.median(latencies_ms)
    stdev_ms = statistics.stdev(latencies_ms)
    sorted_ms = sorted(latencies_ms)
    p95_ms = sorted_ms[int(0.95 * len(sorted_ms))]
    p99_ms = sorted_ms[int(0.99 * len(sorted_ms))]
    max_hz = 1000.0 / max(mean_ms, 1e-6)

    print("\n=== ACTPolicy.select_action() latency ===")
    print(f"  Device:   {device}")
    print(f"  Mean:     {mean_ms:.2f} ms")
    print(f"  Median:   {median_ms:.2f} ms")
    print(f"  Stdev:    {stdev_ms:.2f} ms")
    print(f"  p95:      {p95_ms:.2f} ms")
    print(f"  p99:      {p99_ms:.2f} ms")
    print(f"  Min:      {min(latencies_ms):.2f} ms")
    print(f"  Max:      {max(latencies_ms):.2f} ms")
    print(f"  Implied max loop rate: {max_hz:.1f} Hz")
    print()
    print("AC#4 supervisor compatibility note:")
    print(
        f"  The AA-20 supervisor default max_steps=200 / max_wall_time_s=30 "
        f"implies an average step budget of 150 ms/step."
    )
    if mean_ms < 150.0:
        print(f"  → Mean latency {mean_ms:.1f} ms is WITHIN the 150 ms/step budget.")
    else:
        print(
            f"  → Mean latency {mean_ms:.1f} ms EXCEEDS the default 150 ms/step budget. "
            "Increase max_wall_time_s or reduce max_steps."
        )
    print()
    print("NOTE: random-weight latency is architecture-bound (representative of")
    print("trained model latency). Paste this output as AC#4 evidence.")
    print()
    print("AC#4: UNCHECKED — needs owner-supervised measurement session.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
