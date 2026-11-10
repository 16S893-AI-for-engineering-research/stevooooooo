#!/usr/bin/env python3
"""AA-44: read-only scan of the leader arm (encoder-only SO-101 leader), by USB serial.

Sends only PING and READ packets (no writes, torque untouched). The port is found by the
board's USB serial number, never by /dev node. Reports per servo: ID, model, firmware,
error byte, position, EEPROM limits and homing offset, then measures the joint-read rate
(sync-free: six sequential position reads).

    uv run python scripts/scan_leader.py [--serial 5C93150485] [--seconds 3]
"""
from __future__ import annotations

import argparse
import glob
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
LEADER_SERIAL = "5C93150485"
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def find_port(serial_no: str) -> str:
    for node in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        out = subprocess.run(["udevadm", "info", "-q", "property", "-n", node],
                             capture_output=True, text=True).stdout
        if f"ID_SERIAL_SHORT={serial_no}" in out:
            return node
    raise SystemExit(f"REFUSED: no serial port with USB serial {serial_no} (attach it: usbipd attach --wsl --busid 11-2)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default=LEADER_SERIAL)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    a = ap.parse_args()
    import serial
    from inchworm.servo_bus import _ServoBus, ServoError

    node = find_port(a.serial)
    print(f"leader board {a.serial} at {node} @ {a.baud}")
    port = serial.Serial(node, a.baud, timeout=0.05, exclusive=True)

    class T:  # minimal SerialTransport shim
        is_open = True
        def write(self, b): port.write(b)
        def read(self, n, timeout_s=None):
            port.timeout = timeout_s if timeout_s is not None else 0.05
            return port.read(n)
        def flush_input(self): port.reset_input_buffer()
        def reset_input_buffer(self): port.reset_input_buffer()
        def close(self): port.close()
    try:
        from inchworm.backends import PySerialTransport
        bus = _ServoBus(PySerialTransport(node, port))
    except Exception:
        bus = _ServoBus(T())
    u16 = lambda r: r[0] | (r[1] << 8)
    from inchworm.servo_bus import ServoStatusError
    flagged = {}

    def rd(sid, addr, n):
        """read_bytes tolerating ONLY error byte 0x08 (OverEle), the fw 3.10 encoder-only signature."""
        try:
            return bus.read_bytes(sid, addr, n)
        except ServoStatusError as e:
            if e.error_byte != 0x08:
                raise
            flagged[sid] = e.error_byte
            return _raw(sid, addr, n)

    from inchworm.servo_bus import _INST_READ, _build_request, _checksum

    def _raw(sid, addr, n):
        raw = bus._transact(_build_request(sid, _INST_READ, bytes([addr, n])), sid, f"read {addr}")
        i = raw.find(b"\xff\xff")
        pkt = raw[i:]
        ln, err = pkt[3], pkt[4]
        payload, chk = pkt[5:4 + ln - 1], pkt[4 + ln - 1]
        if pkt[2] != sid or chk != _checksum(sid, ln, 0x00, bytes([err]) + payload) or len(payload) < n:
            raise ServoError(f"servo {sid}: bad reply {raw!r}")
        if err & ~0x08:
            raise ServoError(f"servo {sid}: status error 0x{err:02X}")
        return payload[:n]

    found = {}
    for sid in range(1, 13):
        try:
            found[sid] = rd(sid, 0, 40) + rd(sid, 40, 31)
        except ServoError as e:
            if sid <= 6:
                print(f"ID {sid} ({NAMES[sid]}): NO ANSWER / {e}")
    print(f"answering IDs: {sorted(found)}")
    for sid, r in found.items():
        print(f"ID {sid} {NAMES.get(sid,'?'):13s} model {u16(r[3:5])} fw {r[0]}.{r[1]} "
              f"pos {u16(r[56:58])} limits {u16(r[9:11])}..{u16(r[11:13])} offset {u16(r[31:33])} "
              f"torque_en {r[40]} temp {r[63]}C volt {r[62]/10:.1f}V current {u16(r[69:71])}")
    if flagged:
        print(f"note: error byte 0x08 (overcurrent flag) on IDs {sorted(flagged)}: "
              "expected for firmware-3.10 encoder-only servos; any OTHER error bit is refused.")
    ok = sorted(found) == [1, 2, 3, 4, 5, 6]
    if ok:
        t0, n, errs = time.monotonic(), 0, 0
        while time.monotonic() - t0 < a.seconds:
            for sid in range(1, 7):
                try:
                    rd(sid, 56, 2)
                except ServoError:
                    errs += 1
            n += 1
        dt = time.monotonic() - t0
        print(f"joint-set read rate: {n/dt:.1f} Hz over {dt:.1f}s ({n} sets, {errs} errors)")
    port.close()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
