"""CLI for camera snapshot and short recording — AA-14 AC#5.

Usage (dry-run, safe, no hardware)::

    uv run python -m inchworm.camera_cli snapshot --role camera_left_arm
    uv run python -m inchworm.camera_cli record --role camera_scanner --frames 30

Or via the installed entry point::

    inchworm-camera snapshot --role camera_left_arm
    inchworm-camera record   --role camera_scanner --frames 30

Output: ``robot/sessions/<timestamp>_<hex>/artifacts/<role>_<N>.png``

Pass ``--live`` to use real hardware. Without ``--live`` the mock MJPEG
backend is used and synthetic frames are saved to the session directory.

This script is a DRY-RUN by default: it touches no serial port or real camera
node. Pass ``--live`` for live hardware (owner supervision required; the camera
is opened through the AA-5 discovery and AA-6 arbiter).

AC#4 (measured FPS/latency): use ``scripts/camera_probe.py --live`` instead.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_VALID_ROLES = [
    "camera_left_arm",
    "camera_right_arm",
    "camera_scanner",
]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="inchworm-camera",
        description="Camera snapshot and recording CLI (AA-14).",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Use the real hardware backend and open the device via AA-5 discovery. "
            "OWNER SUPERVISION REQUIRED. Default: mock backend (dry run)."
        ),
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to hardware.yaml (default: auto-detected).",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    # ---- snapshot ----
    snap = sub.add_parser("snapshot", help="Capture one frame and save as PNG.")
    snap.add_argument(
        "--role",
        choices=_VALID_ROLES,
        required=True,
        help="Camera role to capture.",
    )
    snap.add_argument(
        "--width", type=int, default=None, help="Requested width in pixels."
    )
    snap.add_argument(
        "--height", type=int, default=None, help="Requested height in pixels."
    )
    snap.add_argument(
        "--exposure", type=int, default=None, help="Manual exposure value (100 µs units for V4L2)."
    )
    snap.add_argument(
        "--gain", type=int, default=None, help="Manual gain value."
    )
    snap.add_argument(
        "--no-auto-exposure",
        action="store_true",
        default=False,
        help="Disable auto-exposure (manual mode).",
    )
    snap.add_argument(
        "--session-root",
        metavar="PATH",
        default=None,
        help="Override session root directory (default: robot/sessions/).",
    )

    # ---- record ----
    rec = sub.add_parser("record", help="Capture N frames and save as PNGs.")
    rec.add_argument(
        "--role",
        choices=_VALID_ROLES,
        required=True,
        help="Camera role to capture.",
    )
    rec.add_argument(
        "--frames",
        type=int,
        default=10,
        help="Number of frames to capture (default: 10).",
    )
    rec.add_argument(
        "--width", type=int, default=None, help="Requested width in pixels."
    )
    rec.add_argument(
        "--height", type=int, default=None, help="Requested height in pixels."
    )
    rec.add_argument(
        "--exposure", type=int, default=None, help="Manual exposure value."
    )
    rec.add_argument(
        "--gain", type=int, default=None, help="Manual gain value."
    )
    rec.add_argument(
        "--no-auto-exposure",
        action="store_true",
        default=False,
        help="Disable auto-exposure (manual mode).",
    )
    rec.add_argument(
        "--session-root",
        metavar="PATH",
        default=None,
        help="Override session root directory (default: robot/sessions/).",
    )

    return parser


def _run_capture(
    args: argparse.Namespace,
    n_frames: int,
) -> int:
    """Core capture logic shared by snapshot and record."""
    import os

    from inchworm.backends import MockMjpegBackend, create_backend
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraSettings, open_capture
    from inchworm.config import load_config
    from inchworm.session import new_session

    config = load_config(args.config)

    if args.live:
        # Hardware backend — real camera opened via AA-5 discovery + V4L2.
        print(f"[camera_cli] LIVE mode: opening {args.role!r} via AA-5 discovery …")
        from inchworm.backends import V4L2CameraTransport
        from inchworm.config import load_config as _lc
        _live_config = load_config(args.config)
        # Build a minimal live arbiter wrapping a V4L2 transport.
        # The arbiter is still used for the global lock; the transport is V4L2.
        class _LiveBackend(MockMjpegBackend):
            def _open_camera(self, device, identity):
                return V4L2CameraTransport.open(device, identity)
        backend = _LiveBackend(_live_config)
    else:
        print(f"[camera_cli] DRY-RUN mode: using mock MJPEG backend for {args.role!r}.")
        from inchworm.backends import MockMjpegBackend
        backend = MockMjpegBackend(config)

    session_root = Path(args.session_root) if args.session_root else None

    with new_session(config, session_root=session_root) as ctx:
        print(f"[camera_cli] Session: {ctx.session_dir}")
        arbiter = CameraArbiter(backend)
        with arbiter.acquire(args.role) as transport:
            capture = open_capture(transport, role=args.role, logger=ctx.logger)

            settings = CameraSettings(
                width=args.width,
                height=args.height,
                exposure=args.exposure,
                gain=args.gain,
                auto_exposure=(False if args.no_auto_exposure else None),
            )
            applied = capture.apply_settings(settings)
            print(
                f"[camera_cli] Applied settings: "
                f"{applied.width}×{applied.height} @ {applied.fps} fps, "
                f"exposure={applied.exposure}, gain={applied.gain}, "
                f"auto_exposure={applied.auto_exposure}"
            )

            try:
                import cv2
                _have_cv2 = True
            except ImportError:
                _have_cv2 = False

            saved = []
            for i in range(n_frames):
                result = capture.grab()
                if result.corrupt:
                    print(f"[camera_cli] WARNING: frame {result.counter} is corrupt (decode failed).")
                    continue
                if result.dropped:
                    print(f"[camera_cli] WARNING: frames dropped before frame {result.counter}.")

                fname = ctx.artifact_dir / f"{args.role}_{result.counter:06d}.png"
                if _have_cv2 and result.data.size > 0:
                    import cv2
                    cv2.imwrite(str(fname), result.data)
                else:
                    # Fallback: write raw bytes as .bin
                    fname = fname.with_suffix(".bin")
                    fname.write_bytes(result.data.tobytes() if result.data.size > 0 else b"")
                saved.append(fname)
                print(
                    f"[camera_cli] Frame {result.counter:4d}: "
                    f"{result.data.shape if result.data.size > 0 else '(corrupt)'} "
                    f"ts={result.ts_monotonic:.6f} → {fname.name}"
                )

        ctx.logger.info(
            role=args.role,
            subsystem="camera_cli",
            event_type="capture_complete",
            payload={"frames_saved": len(saved), "session_dir": str(ctx.session_dir)},
        )
        print(f"[camera_cli] Saved {len(saved)} frame(s) to {ctx.artifact_dir}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "snapshot":
        return _run_capture(args, n_frames=1)
    elif args.command == "record":
        return _run_capture(args, n_frames=args.frames)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
