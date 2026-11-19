#!/usr/bin/env python3
"""Camera frame-rate diagnostic (AA-23, 2026-11-17): find where the wrist camera loses frames.

Owner measurement 2026-11-17: the OV9281 wrist camera runs at 60 fps in the Windows
Camera app. Our 30 fps recordings get only ~8-12 NEW images/s. This one script runs
every stage under the AA-6 camera arbiter (one camera at a time) and names the stage
that loses the frames.

Stages, for each setting variant (fps 30/60; auto exposure; manual exposure 10 ms / 5 ms):
  A  raw    cv2 grab() only: frames dequeued from V4L2, no decode (USB/usbip/WSL limit)
  B  decode cv2 read(): decoded frames/s and NEW (non-identical) frames/s
  C  prod   CameraCapture.grab_valid(): our production path (libjpeg-flagged frames skipped)
  D  loop   30 Hz loop like the LeRobot follower (repeats the last frame when one is skipped)

Usage (devices attached with scripts/attach_usb.py OUTSIDE any sandbox first):
  uv run python scripts/camera_rate_diag.py                 # dry-run, mock camera
  uv run python scripts/camera_rate_diag.py --live --arm left
Report: robot/out/camera_diag_<UTC>.json (+ printed text). No motion and no serial
port is opened.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: (label, fps, auto_exposure, exposure in 100 us units)
VARIANTS = [
    ("fps30_auto", 30, None, None),
    ("fps60_auto", 60, None, None),
    ("fps60_exp10ms", 60, False, 100),
    ("fps60_exp5ms", 60, False, 50),
]


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="Use the real camera (default: dry-run mock).")
    p.add_argument("--arm", choices=("left", "right"), default="left")
    p.add_argument("--seconds", type=float, default=None,
                   help="Seconds per stage (default 5 live, 0.3 dry-run).")
    p.add_argument("--transport", choices=("config", "usbip", "bridge"), default="config",
                   help="Camera path for --live: config (hardware.yaml camera_transport), usbip (V4L2 in WSL) "
                        "or bridge (Windows DirectShow bridge, AA-53).")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--out", type=Path, default=None, help="Report path (default robot/out/camera_diag_<UTC>.json).")
    return p


def _digest(frame) -> str:
    return hashlib.blake2b(memoryview(frame).tobytes() if hasattr(frame, "tobytes") else bytes(frame),
                           digest_size=8).hexdigest()


def _rate(n: int, dt: float) -> float:
    return round(n / dt, 2) if dt > 0 else 0.0


def stage_raw(cap, seconds: float) -> dict:
    """A: cv2 grab() only (dequeue from V4L2, no decode)."""
    n = fails = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if cap.grab():
            n += 1
        else:
            fails += 1
    return {"grab_per_s": _rate(n, time.monotonic() - t0), "grab_fail": fails}


def stage_decode(cap, seconds: float) -> dict:
    """B: cv2 read() - decoded frames and NEW (non-identical) frames per second."""
    n = new = fails = 0
    last = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ok, frame = cap.read()
        if not ok or frame is None:
            fails += 1
            continue
        n += 1
        d = _digest(frame)
        new += d != last
        last = d
    dt = time.monotonic() - t0
    return {"read_per_s": _rate(n, dt), "new_per_s": _rate(new, dt), "read_fail": fails}


def stage_prod(capture, seconds: float) -> dict:
    """C: production grab_valid() (libjpeg-flagged frames skipped)."""
    n = new = corrupt = 0
    last = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        r = capture.grab_valid()
        if r.corrupt or r.data is None or r.data.size == 0:
            corrupt += 1
            continue
        n += 1
        d = _digest(r.data)
        new += d != last
        last = d
    dt = time.monotonic() - t0
    return {"valid_per_s": _rate(n, dt), "new_per_s": _rate(new, dt), "all_corrupt_returns": corrupt}


def stage_loop(capture, seconds: float, hz: float = 30.0) -> dict:
    """D: a fixed-rate loop like the follower: a skipped frame repeats the last one."""
    period = 1.0 / hz
    ticks = new = repeats = overruns = 0
    last = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ts = time.monotonic()
        r = capture.grab()
        if r.corrupt or r.data is None or r.data.size == 0:
            repeats += 1
        else:
            d = _digest(r.data)
            new += d != last
            last = d
        ticks += 1
        rest = period - (time.monotonic() - ts)
        if rest > 0:
            time.sleep(rest)
        else:
            overruns += 1
    dt = time.monotonic() - t0
    return {"loop_hz": _rate(ticks, dt), "new_per_s": _rate(new, dt), "repeats": repeats, "overruns": overruns}


def v4l2_info(dev_node: str | None) -> dict:
    exe = shutil.which("v4l2-ctl")
    if exe is None or dev_node is None:
        return {"available": False, "note": "install v4l-utils (sudo apt install v4l-utils) for format/control info"}
    out = {}
    for key, args in (("formats", ["--list-formats-ext"]), ("controls", ["--list-ctrls"])):
        try:
            out[key] = subprocess.run([exe, "-d", dev_node, *args], capture_output=True, text=True,
                                      timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired) as exc:
            out[key] = f"error: {exc}"
    return {"available": True, **out}


def verdict(results: dict) -> str:
    """One paragraph naming the stage that loses frames."""
    v = results.get("variants", {})
    a30, a60 = v.get("fps30_auto", {}), v.get("fps60_auto", {})
    best_exp = max((v.get(k, {}) for k in ("fps60_exp10ms", "fps60_exp5ms")),
                   key=lambda r: r.get("raw", {}).get("grab_per_s", 0), default={})
    raw30 = a30.get("raw", {}).get("grab_per_s")
    raw60 = a60.get("raw", {}).get("grab_per_s")
    rawexp = best_exp.get("raw", {}).get("grab_per_s")
    new30 = a30.get("decode", {}).get("new_per_s")
    prod30 = a30.get("prod", {}).get("new_per_s")
    loop30 = a30.get("loop", {}).get("new_per_s")
    if raw30 is None and results.get("transport") == "bridge":
        ok = loop30 is not None and loop30 >= 25
        return (f"BRIDGE {'PASS' if ok else 'FAIL'} (AA-53 AC6 target >= 25 new/s in the 30 Hz loop): "
                f"grab_valid {prod30} new/s, 30 Hz loop {loop30} new/s (fps30_auto). Raw/decode stages are "
                "not available through the bridge.")
    if raw30 is None:
        return "Dry-run / no raw stage: no verdict (run with --live)."
    if raw30 < 20 and rawexp is not None and rawexp >= 1.5 * raw30:
        return (f"EXPOSURE: auto exposure limits the sensor (raw {raw30}/s at fps30-auto, {rawexp}/s with "
                "manual exposure). Fix: set auto_exposure off + a manual exposure (and gain) in the "
                "follower's CameraSettings, or add light.")
    if raw30 < 20 and (raw60 or 0) < 20:
        return (f"TRANSPORT: frames are already missing before decode (raw {raw30}/s at 30, {raw60}/s at 60). "
                "The usbip/WSL link (or the negotiated mode) is the limit. Check the 'applied' fps and "
                "the v4l2 formats; try a lower resolution, another USB port/hub, or a newer usbipd/WSL kernel.")
    if new30 is not None and new30 < 0.7 * raw30:
        return (f"DUPLICATE BUFFERS: V4L2 delivers {raw30}/s but only {new30} are new images; the driver/usbip "
                "is re-delivering identical buffers. Try fps 60 (and keep 1 in 2) or a different MJPG mode.")
    if prod30 is not None and new30 and prod30 < 0.7 * new30:
        return (f"OUR DECODE/REJECT: {new30} new/s decoded but only {prod30}/s pass grab_valid; the libjpeg "
                "flag rejects too many frames (see corrupt counts). Fix in backends.py reject policy (owner decision).")
    if loop30 is not None and prod30 and loop30 < 0.7 * min(prod30, 30):
        return (f"LOOP: the capture is fine ({prod30} new/s) but a 30 Hz loop sees only {loop30} new/s; "
                "the synchronous read in the control loop or buffering is the limit. Fix: a background "
                "capture thread with latest-frame semantics in the follower.")
    return (f"No single stage loses frames here (raw {raw30}/s, new {new30}/s, prod {prod30}/s, loop {loop30}/s). "
            "Compare with the recorder's new-images/s in inspect_dataset.py; the loss may be in the recorder "
            "under load (video encode).")


def run(args: argparse.Namespace) -> dict:
    from inchworm.backends import MockMjpegBackend
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import CameraSettings, open_capture
    from inchworm.config import load_config

    seconds = args.seconds if args.seconds is not None else (5.0 if args.live else 0.3)
    config = load_config()
    if args.live and args.transport != "config":
        from inchworm.config import CameraTransportName
        config = config.model_copy(update={"camera_transport": CameraTransportName(args.transport)})
    bridge = args.live and config.camera_transport.value == "bridge"
    if bridge:
        from inchworm.backends import live_backend
        backend = live_backend(config)          # HardwareBackend opens BridgeCameraTransport
    elif args.live:
        from inchworm.backends import V4L2CameraTransport

        class _LiveBackend(MockMjpegBackend):
            def _open_camera(self, device, identity):
                return V4L2CameraTransport.open(device, identity)
        backend = _LiveBackend(config)
    else:
        backend = MockMjpegBackend(config)

    device = f"camera_{args.arm}_arm"
    dev_node = None
    if args.live and not bridge:
        from inchworm.discovery import require_device
        dev_node = require_device(f"cam_{args.arm}")
    results: dict = {"utc": datetime.now(timezone.utc).isoformat(), "live": args.live, "device": device,
                     "dev_node": dev_node, "transport": config.camera_transport.value if args.live else "mock", "seconds_per_stage": seconds, "v4l2": v4l2_info(dev_node),
                     "variants": {}}

    with CameraArbiter(backend).acquire(device) as transport:
        capture = open_capture(transport, role=device)
        cap = getattr(transport, "_cap", None)       # cv2.VideoCapture on the live transport
        for label, fps, auto, exposure in VARIANTS:
            applied = capture.apply_settings(CameraSettings(width=args.width, height=args.height, fps=fps,
                                                            auto_exposure=auto, exposure=exposure))
            row: dict = {"requested": {"fps": fps, "auto_exposure": auto, "exposure": exposure},
                         "applied": {"width": applied.width, "height": applied.height, "fps": applied.fps,
                                     "exposure": applied.exposure, "auto_exposure": applied.auto_exposure}}
            for _ in range(5):                        # let the new mode settle
                capture.grab()
            if cap is not None:
                row["raw"] = stage_raw(cap, seconds)
                row["decode"] = stage_decode(cap, seconds)
            row["prod"] = stage_prod(capture, seconds)
            row["loop"] = stage_loop(capture, seconds)
            results["variants"][label] = row
            print(f"[{label}] applied {row['applied']}  " +
                  "  ".join(f"{k}={row[k]}" for k in ("raw", "decode", "prod", "loop") if k in row), flush=True)
        # leave the camera in its default mode for the next user
        capture.apply_settings(CameraSettings(width=args.width, height=args.height, fps=30, auto_exposure=True))
    results["verdict"] = verdict(results)
    return results


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    print(f"[camera_rate_diag] {'LIVE' if args.live else 'DRY-RUN (mock camera)'}: arm={args.arm}")
    try:
        results = run(args)
    except Exception as exc:  # noqa: BLE001 - report any device/arbiter problem cleanly
        print(f"[camera_rate_diag] could not run: {type(exc).__name__}: {exc}\n"
              "Attach the cameras first (outside any sandbox): uv run python scripts/attach_usb.py",
              file=sys.stderr)
        return 2
    out = args.out or ROOT / "out" / f"camera_diag_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=2, default=str) + "\n", encoding="utf-8")
    print(f"\nVERDICT: {results['verdict']}\nReport: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
