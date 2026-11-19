r"""Windows-side camera bridge (AA-53): one DirectShow camera -> TCP -> WSL robot code.

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

Camera identity (AA-53 AC4): the two arm cameras are identical (32e6:9251, same
USB serial), so neither name nor serial nor DirectShow index identifies them.
The robot asks for a camera by its usbipd BusId (``{"busid": "9-4"}``), which
encodes the physical hub port. At stream open this program enumerates the
DirectShow video input devices in the same order OpenCV uses, takes each
device's DevicePath (``\\?\usb#vid_32e6&pid_9251&mi_00#<instance>#{guid}...``),
looks up the USB device node owning that interface with cfgmgr32 and turns its
LocationInformation ``Port_#0004.Hub_#0009`` into the BusId ``9-4`` (the same
hub/port numbers usbipd prints). ``{"dshow_index": N}`` remains an explicit
override for diagnostics.

Requirements: Windows, Python 3.12, opencv-python (cv2) + numpy, and pygrabber
(for its comtypes DirectShow enumeration; ``py -3.12 -m pip install --user
pygrabber``). Stdlib otherwise.

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
import re
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


_LOCATION_INFO = re.compile(r"^Port_#(\d+)\.Hub_#(\d+)$")
DEVICE_PATH_PREFIX = "\\\\?\\"


def busid_from_location_info(location_info: str) -> "str | None":
    """``Port_#0004.Hub_#0009`` -> ``9-4`` (usbipd BusId = hub-port)."""
    m = _LOCATION_INFO.match(location_info.strip())
    return f"{int(m.group(2))}-{int(m.group(1))}" if m else None


def instance_id_from_device_path(device_path: str) -> "str | None":
    r"""``\\?\usb#vid_x&pid_y&mi_00#7&2cfe5692&0&0000#{guid}\global`` -> PnP instance ID."""
    path = device_path
    if path.startswith(DEVICE_PATH_PREFIX):
        path = path[4:]
    parts = path.split("#")
    if len(parts) < 4:
        return None
    return "\\".join(parts[:3]).upper()


def resolve_dshow_index(selector: dict, devices: list) -> int:
    """Return the DirectShow index for *selector*.

    *devices* is the DirectShow enumeration: list of dicts with ``index``,
    ``name``, ``device_path`` and ``busid`` (None if unknown), in OpenCV order.
    ``busid`` must match exactly one device; ``vid_pid`` (``"32e6:9251"``), when
    given, must match that device's DevicePath, so a moved cable is refused
    instead of streaming the wrong camera.
    """
    if selector.get("dshow_index") is not None:
        return int(selector["dshow_index"])
    busid = selector.get("busid")
    if not busid:
        raise RuntimeError(f"camera selector needs busid or dshow_index: {selector!r}")
    matches = [d for d in devices if d.get("busid") == busid]
    if len(matches) != 1:
        seen = ", ".join(f"{d['index']}={d.get('busid')}({d['name']})" for d in devices)
        raise RuntimeError(f"{len(matches)} DirectShow cameras at BusId {busid}; enumerated: {seen}")
    device = matches[0]
    vid_pid = selector.get("vid_pid")
    if vid_pid:
        vid, pid = vid_pid.lower().split(":")
        if f"vid_{vid}&pid_{pid}" not in device["device_path"].lower():
            raise RuntimeError(
                f"camera at BusId {busid} is {device['device_path']!r}, expected VID:PID {vid_pid}"
            )
    return int(device["index"])


def _location_info_of_usb_device(instance_id: str) -> "str | None":
    """cfgmgr32: LocationInformation of the USB device owning interface *instance_id*."""
    import ctypes
    from ctypes import wintypes

    cfg = ctypes.WinDLL("cfgmgr32")
    devinst = wintypes.DWORD()
    if cfg.CM_Locate_DevNodeW(ctypes.byref(devinst), ctypes.c_wchar_p(instance_id), 0) != 0:
        return None
    if "&MI_" in instance_id:  # composite interface -> its parent is the USB port device
        parent = wintypes.DWORD()
        if cfg.CM_Get_Parent(ctypes.byref(parent), devinst, 0) != 0:
            return None
        devinst = parent
    CM_DRP_LOCATION_INFORMATION = 0x0E
    buf = ctypes.create_unicode_buffer(256)
    size = wintypes.ULONG(ctypes.sizeof(buf))
    reg_type = wintypes.ULONG()
    rc = cfg.CM_Get_DevNode_Registry_PropertyW(
        devinst, CM_DRP_LOCATION_INFORMATION, ctypes.byref(reg_type), buf, ctypes.byref(size), 0
    )
    return buf.value if rc == 0 else None


def list_dshow_video_devices() -> list:
    """DirectShow video input devices in OpenCV CAP_DSHOW index order, with BusId."""
    from comtypes import GUID
    from pygrabber.dshow_graph import IPropertyBag, SystemDeviceEnum
    from pygrabber.dshow_ids import DeviceCategories

    enum = SystemDeviceEnum().system_device_enum.CreateClassEnumerator(
        GUID(DeviceCategories.VideoInputDevice), dwFlags=0
    )
    devices = []
    while True:
        moniker, count = enum.Next(1)
        if count == 0:
            break
        bag = moniker.BindToStorage(0, 0, IPropertyBag._iid_).QueryInterface(IPropertyBag)
        name = bag.Read("FriendlyName", pErrorLog=None)
        try:
            device_path = bag.Read("DevicePath", pErrorLog=None)
        except Exception:  # virtual cameras have no DevicePath
            device_path = ""
        instance_id = instance_id_from_device_path(device_path) if device_path else None
        location = _location_info_of_usb_device(instance_id) if instance_id else None
        devices.append({
            "index": len(devices),
            "name": name,
            "device_path": device_path,
            "busid": busid_from_location_info(location) if location else None,
        })
    return devices


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


def serve_stream(sock: socket.socket, open_camera=Camera, list_video_devices=list_dshow_video_devices) -> int:
    """Serve one stream on a connected socket; return the number of frames sent."""
    msg_type, payload = recv_message(sock)
    if msg_type != MSG_REQUEST:
        sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": False, "error": "expected request"}).encode()))
        return 0
    request = json.loads(payload)
    try:
        selector = request["camera"]
        devices = [] if selector.get("dshow_index") is not None else list_video_devices()
        dshow_index = resolve_dshow_index(selector, devices)
        camera = open_camera(dshow_index)
        applied = camera.apply(request)
    except Exception as err:  # report any open failure to the robot, then end the stream
        sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": False, "error": str(err)}).encode()))
        return 0
    sock.sendall(pack_message(MSG_REPLY, json.dumps({"ok": True, "applied": applied}).encode()))
    print(f"[bridge] streaming {request.get('device')} selector={selector} dshow_index={dshow_index} "
          f"applied={applied}", flush=True)

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
    ap.add_argument("--connect", help="robot listener host:port (WSL address from hostname -I)")
    ap.add_argument("--retry-s", type=float, default=1.0, help="seconds between connection attempts")
    ap.add_argument("--once", action="store_true", help="exit after the first stream")
    ap.add_argument("--list", action="store_true",
                    help="print the DirectShow cameras with their BusId and exit (opens no camera)")
    args = ap.parse_args()
    if args.list:
        for d in list_dshow_video_devices():
            print(f"{d['index']}\t{d['busid']}\t{d['name']}\t{d['device_path']}")
        return 0
    if not args.connect:
        ap.error("--connect is required unless --list is given")
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
