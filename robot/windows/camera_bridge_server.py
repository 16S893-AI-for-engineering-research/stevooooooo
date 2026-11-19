"""Windows-side camera bridge (AA-53): one DirectShow camera -> TCP -> WSL robot code.

Why: over usbip the arm cameras deliver ~5-8 new images/s in WSL; through
DirectShow on Windows they deliver 60/s. This program reads ONE camera with
OpenCV's DirectShow backend (never MSMF) and streams JPEG frames to the robot
side (``inchworm.camera_bridge.BridgeCameraTransport``).

Connection direction: this program only CONNECTS OUT to the robot's listener
in WSL; it never listens, so no inbound Windows firewall rule is needed. The
robot side decides which camera and settings it wants (request handshake).

One stream at a time: the program holds one outbound connection and at most
one open camera; the camera is released when the stream ends. A second copy
of this program on the same machine refuses to start (lock file). The robot
side additionally closes its listener once one server has connected.

Latest-frame semantics: a capture thread keeps only the newest frame; the
sender always sends the newest frame after the previous send finished, so a
slow link drops old frames (visible as seq gaps on the robot side) instead of
queueing them.

Requirements: Windows, Python 3.12, opencv-python (cv2) + numpy. Stdlib otherwise.

Usage (PowerShell on Windows; WSL address from ``hostname -I`` inside WSL)::

    py -3.12 camera_bridge_server.py --connect 172.26.80.62:50853

The program keeps retrying the connection (the robot listens only while a
camera is being opened), serves one stream, then waits for the next request.
``--once`` exits after the first stream. Ctrl+C stops it.

Wire protocol: see the module docstring of ``inchworm.camera_bridge``; the
constants below must match it (checked by robot/tests/test_camera_bridge.py).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import tempfile
import threading
import time

PROTOCOL_MAGIC = b"IWCB"
PROTOCOL_VERSION = 1
MSG_HEADER = struct.Struct("<4sBBI")
FRAME_HEADER = struct.Struct("<QqqHHB")
MSG_REQUEST = 1
MSG_REPLY = 2
MSG_SETTINGS = 3
MSG_SETTINGS_APPLIED = 4
MSG_FRAME = 5
ENCODING_JPEG = 1
JPEG_QUALITY = 90
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


def pack_message(msg_type: int, payload: bytes) -> bytes:
    return MSG_HEADER.pack(PROTOCOL_MAGIC, PROTOCOL_VERSION, msg_type, len(payload)) + payload


def recv_exact(sock: socket.socket, n: int) -> bytes:
    chunks = []
    while n:
        chunk = sock.recv(n)
        if not chunk:
            raise ConnectionError("robot closed the stream")
        chunks.append(chunk)
        n -= len(chunk)
    return b"".join(chunks)


def recv_message(sock: socket.socket) -> "tuple[int, bytes]":
    magic, version, msg_type, length = MSG_HEADER.unpack(recv_exact(sock, MSG_HEADER.size))
    if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION or length > MAX_PAYLOAD_BYTES:
        raise ConnectionError(f"bad message header magic={magic!r} version={version} length={length}")
    return msg_type, recv_exact(sock, length)


class Camera:
    """One DirectShow camera; all cv2 calls happen on the capture thread."""

    def __init__(self, dshow_index: int) -> None:
        import cv2

        self.cv2 = cv2
        self.cap = cv2.VideoCapture(dshow_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open DirectShow camera index {dshow_index}")

    def apply(self, s: dict) -> dict:
        cv2, cap = self.cv2, self.cap
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if s.get("width"):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(s["width"]))
        if s.get("height"):
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(s["height"]))
        if s.get("fps"):
            cap.set(cv2.CAP_PROP_FPS, float(s["fps"]))
        if s.get("auto_exposure") is not None:
            # DirectShow: CAP_PROP_AUTO_EXPOSURE 0.75 = auto, 0.25 = manual.
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if s["auto_exposure"] else 0.25)
        if s.get("exposure") is not None:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(s["exposure"]))
        if s.get("gain") is not None:
            cap.set(cv2.CAP_PROP_GAIN, float(s["gain"]))
        ae = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        return {
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or None,
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or None,
            "fps": cap.get(cv2.CAP_PROP_FPS) or None,
            "exposure": int(cap.get(cv2.CAP_PROP_EXPOSURE)),
            "gain": int(cap.get(cv2.CAP_PROP_GAIN)),
            "auto_exposure": None if ae < 0 else ae > 0.5,
        }

    def read_jpeg(self) -> "tuple[bytes, int, int] | None":
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        ok, buf = self.cv2.imencode(".jpg", frame, [self.cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return None
        return buf.tobytes(), frame.shape[1], frame.shape[0]

    def release(self) -> None:
        self.cap.release()


def serve_stream(sock: socket.socket, open_camera=Camera) -> int:
    """Serve one stream on a connected socket; return the number of frames sent."""
    msg_type, payload = recv_message(sock)
    if msg_type != MSG_REQUEST:
        sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": False, "error": "expected request"}).encode()))
        return 0
    request = json.loads(payload)
    try:
        camera = open_camera(int(request["camera"]["dshow_index"]))
        applied = camera.apply(request)
    except Exception as err:  # report any open failure to the robot, then end the stream
        sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": False, "error": str(err)}).encode()))
        return 0
    sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": True, "applied": applied}).encode()))
    print(f"[bridge] streaming {request.get('device')} applied={applied}", flush=True)

    cond = threading.Condition()
    state = {"latest": None, "stop": False, "settings": [], "applied": [], "sent": 0}

    def capture() -> None:
        seq = 0
        try:
            while True:
                with cond:
                    if state["stop"]:
                        return
                    pending, state["settings"] = state["settings"], []
                for s in pending:
                    result = camera.apply(s)
                    with cond:
                        state["applied"].append(result)
                        cond.notify_all()
                got = camera.read_jpeg()
                if got is None:
                    continue
                jpeg, width, height = got
                header = FRAME_HEADER.pack(seq, time.monotonic_ns(), time.time_ns(), width, height, ENCODING_JPEG)
                seq += 1
                with cond:
                    state["latest"] = header + jpeg  # replaces any unsent older frame
                    cond.notify_all()
        finally:
            camera.release()
            with cond:
                state["stop"] = True
                cond.notify_all()

    def control() -> None:
        try:
            while True:
                msg_type, payload = recv_message(sock)
                if msg_type == MSG_SETTINGS:
                    with cond:
                        state["settings"].append(json.loads(payload))
        except (OSError, ConnectionError, ValueError):
            pass
        with cond:
            state["stop"] = True
            cond.notify_all()

    capture_thread = threading.Thread(target=capture, daemon=True)
    control_thread = threading.Thread(target=control, daemon=True)
    capture_thread.start()
    control_thread.start()
    try:
        while True:
            with cond:
                while not state["stop"] and state["latest"] is None and not state["applied"]:
                    cond.wait(1.0)
                if state["stop"]:
                    break
                applied_msgs, state["applied"] = state["applied"], []
                frame, state["latest"] = state["latest"], None
            for a in applied_msgs:
                sock.sendall(pack_message(MSG_SETTINGS_APPLIED, json.dumps(a).encode()))
            if frame is not None:
                sock.sendall(pack_message(MSG_FRAME, frame))
                state["sent"] += 1
    except OSError:
        pass
    finally:
        with cond:
            state["stop"] = True
            cond.notify_all()
        capture_thread.join(timeout=5.0)
    return state["sent"]


def single_instance_lock():
    """Refuse a second copy of this program (it would be a second camera stream)."""
    import msvcrt

    fh = open(os.path.join(tempfile.gettempdir(), "inchworm_camera_bridge.lock"), "a+")
    try:
        msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        sys.exit("[bridge] another camera_bridge_server.py is already running; one stream at a time")
    return fh


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--connect", required=True, help="robot listener host:port (WSL address from hostname -I)")
    ap.add_argument("--retry-s", type=float, default=1.0, help="seconds between connection attempts")
    ap.add_argument("--once", action="store_true", help="exit after the first stream")
    args = ap.parse_args()
    host, port = args.connect.rsplit(":", 1)
    _lock = single_instance_lock()
    while True:
        try:
            sock = socket.create_connection((host, int(port)), timeout=2.0)
        except OSError:
            time.sleep(args.retry_s)
            continue
        sock.settimeout(None)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sent = serve_stream(sock)
        except (OSError, ConnectionError, ValueError) as err:
            print(f"[bridge] stream error: {err}", flush=True)
            sent = 0
        finally:
            sock.close()
        print(f"[bridge] stream ended, frames sent={sent}", flush=True)
        if args.once:
            return 0
        time.sleep(args.retry_s)


if __name__ == "__main__":
    sys.exit(main())
