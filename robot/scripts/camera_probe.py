#!/usr/bin/env python3
"""Camera probe script — measures achieved FPS and per-frame latency (AA-14 AC#4).

This script is a DRY-RUN by default: it uses the mock MJPEG backend and
prints synthetic measurement results. Pass ``--live`` to measure real cameras.

Usage (dry run, safe, no hardware):
    uv run python scripts/camera_probe.py

Usage (live — requires cameras attached, owner supervision required):
    uv run python scripts/camera_probe.py --live
    uv run python scripts/camera_probe.py --live --role camera_scanner --frames 120

AC#4 note — UNCHECKED:
    needs owner-supervised hardware session:
        uv run python scripts/camera_probe.py --live --role camera_left_arm --frames 120
        uv run python scripts/camera_probe.py --live --role camera_right_arm --frames 120
        uv run python scripts/camera_probe.py --live --role camera_scanner --frames 120
    Record the output (achieved FPS, mean/p95/max latency) as task notes in AA-14.

Results are printed to stdout and also saved to robot/sessions/<ts>/artifacts/probe_<role>.txt.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


_VALID_ROLES = [
    "camera_left_arm",
    "camera_right_arm",
    "camera_scanner",
]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="camera_probe",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Use real hardware via AA-5 discovery + AA-6 arbiter. "
            "OWNER SUPERVISION REQUIRED. Default: mock (dry run)."
        ),
    )
    parser.add_argument(
        "--role",
        choices=_VALID_ROLES,
        default="camera_left_arm",
        help="Camera role to probe (default: camera_left_arm).",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=30,
        help="Number of frames to capture for the measurement (default: 30).",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Requested width (default: 640).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Requested height (default: 480).",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Requested FPS (default: 30).",
    )
    parser.add_argument(
        "--exposure",
        type=int,
        default=None,
        help="Manual exposure value (V4L2 units, 100 µs). None = leave default.",
    )
    parser.add_argument(
        "--no-auto-exposure",
        action="store_true",
        default=False,
        help="Disable auto-exposure.",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to hardware.yaml.",
    )
    parser.add_argument(
        "--session-root",
        metavar="PATH",
        default=None,
        help="Override session root directory.",
    )
    return parser


def _probe(args: argparse.Namespace) -> int:
    import statistics

    from inchworm.backends import MockBackend, MockMjpegBackend, create_backend
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraSettings, open_capture
    from inchworm.config import BackendName, load_config
    from inchworm.session import new_session

    config = load_config(args.config)

    if args.live:
        print(f"[camera_probe] LIVE mode: {args.role!r} via AA-5 discovery.")
        from inchworm.backends import MockMjpegBackend as _MjpegBase, V4L2CameraTransport
        class _LiveBackend(_MjpegBase):
            def _open_camera(self, device, identity):
                return V4L2CameraTransport.open(device, identity)
        backend = _LiveBackend(config)
    else:
        print(f"[camera_probe] DRY-RUN: mock MJPEG backend for {args.role!r}.")
        backend = MockMjpegBackend(config)

    session_root = Path(args.session_root) if args.session_root else None

    with new_session(config, session_root=session_root) as ctx:
        arbiter = CameraArbiter(backend)
        with arbiter.acquire(args.role) as transport:
            capture = open_capture(transport, role=args.role, logger=ctx.logger)

            settings = CameraSettings(
                width=args.width,
                height=args.height,
                fps=args.fps,
                exposure=args.exposure,
                auto_exposure=(False if args.no_auto_exposure else None),
            )
            applied = capture.apply_settings(settings)
            print(
                f"[camera_probe] Applied: {applied.width}×{applied.height} "
                f"@ {applied.fps} fps, exposure={applied.exposure}, "
                f"auto_exposure={applied.auto_exposure}"
            )

            # Warm-up: discard first few frames (pipeline fill).
            WARMUP = min(5, args.frames // 4)
            for _ in range(WARMUP):
                capture.grab()

            latencies_s: list[float] = []
            frame_ts: list[float] = []
            dropped_total = 0
            corrupt_total = 0

            for i in range(args.frames):
                t0 = time.monotonic()
                result = capture.grab()
                t1 = time.monotonic()
                latencies_s.append(t1 - t0)
                frame_ts.append(result.ts_monotonic)
                if result.dropped:
                    dropped_total += 1
                if result.corrupt:
                    corrupt_total += 1

        # Compute statistics.
        total_wall = frame_ts[-1] - frame_ts[0] if len(frame_ts) > 1 else 0.0
        achieved_fps = (args.frames - 1) / total_wall if total_wall > 0 else float("nan")
        mean_lat_ms = statistics.mean(latencies_s) * 1000
        p95_lat_ms = sorted(latencies_s)[int(len(latencies_s) * 0.95)] * 1000
        max_lat_ms = max(latencies_s) * 1000

        lines = [
            f"camera_probe results — role={args.role!r}",
            f"  backend        : {'hardware (LIVE)' if args.live else 'mock (dry-run)'}",
            f"  requested      : {args.width}×{args.height} @ {args.fps} fps",
            f"  applied        : {applied.width}×{applied.height} @ {applied.fps} fps",
            f"  frames         : {args.frames} (warmup={WARMUP}, measured={args.frames})",
            f"  achieved FPS   : {achieved_fps:.2f}",
            f"  latency mean   : {mean_lat_ms:.2f} ms",
            f"  latency p95    : {p95_lat_ms:.2f} ms",
            f"  latency max    : {max_lat_ms:.2f} ms",
            f"  dropped        : {dropped_total}",
            f"  corrupt        : {corrupt_total}",
        ]
        report = "\n".join(lines)
        print(report)

        # Save report artifact.
        report_path = ctx.artifact_dir / f"probe_{args.role}.txt"
        report_path.write_text(report + "\n", encoding="utf-8")
        print(f"[camera_probe] Report saved to {report_path}")

        ctx.logger.info(
            role=args.role,
            subsystem="camera_probe",
            event_type="probe_complete",
            payload={
                "live": args.live,
                "frames": args.frames,
                "achieved_fps": achieved_fps,
                "mean_lat_ms": mean_lat_ms,
                "p95_lat_ms": p95_lat_ms,
                "max_lat_ms": max_lat_ms,
                "dropped": dropped_total,
                "corrupt": corrupt_total,
            },
        )

    return 0


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    return _probe(args)


if __name__ == "__main__":
    sys.exit(main())
