"""Read-only SO-101 pose source for cell-model checks (AA40).

Reads present positions of joints 1-5 on both arms (never writes) and
converts them to URDF radians with the verified servo calibration. Used as the
``so101_pose_source`` of the DexArm cell guard, so DexArm moves are checked
against where the SO-101 arms really are.

Fail closed: refuses if a servo has torque ON (then the arm may be moving and
a single reading is not a safe snapshot), if the calibration is unverified, or
if any read fails. Status error bit 0x08 is tolerated ONLY on servos 4-6:
those are encoder-only leader servos on this build (doc-1, 2026-10-27) that
report a bogus overcurrent flag while their position reads are valid.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
_ENCODER_ONLY_IDS = (4, 5, 6)
_TOLERATED_BITS = 0x08
_REG_TORQUE_ENABLE = 0x28


class PoseReadError(RuntimeError):
    pass


def _read(raw, sid: int, addr: int, n: int) -> bytes:
    from inchworm.servo_bus import ServoStatusError, _INST_READ, _build_request, _checksum
    try:
        return raw.read_bytes(sid, addr, n)
    except ServoStatusError as exc:
        if sid not in _ENCODER_ONLY_IDS or exc.error_byte & ~_TOLERATED_BITS:
            raise
    rep = raw._transact(_build_request(sid, _INST_READ, bytes([addr, n])), sid, f"read {addr}")
    pkt = rep[rep.find(b"\xff\xff"):]
    ln, err = pkt[3], pkt[4]
    payload, chk = pkt[5:4 + ln - 1], pkt[4 + ln - 1]
    if pkt[2] != sid or chk != _checksum(sid, ln, 0x00, bytes([err]) + payload) or len(payload) < n:
        raise PoseReadError(f"servo {sid}: bad reply")
    if err & ~_TOLERATED_BITS:
        raise PoseReadError(f"servo {sid}: status error 0x{err:02X}")
    return payload[:n]


def read_arm_pose(cfg, backend, arm: str) -> List[float]:
    from inchworm.servo_bus import ArmServoBus, JOINT_NAMES
    cal = getattr(cfg.motion_guard.servo_calibration, f"{arm}_arm")
    if cal is None or cal.unverified:
        raise PoseReadError(f"{arm} servo_calibration missing or unverified")
    t = backend.open_serial(f"arm_{arm}")
    try:
        bus = ArmServoBus(t, role=f"arm_{arm}", joint_id_map={n: i + 1 for i, n in enumerate(JOINT_NAMES)})
        raw = bus._bus
        if any(_read(raw, s, _REG_TORQUE_ENABLE, 1)[0] for s in range(1, 6)):
            raise PoseReadError(f"arm_{arm}: a joint has torque ON; pose snapshot refused")
        q = []
        for i, j in enumerate(J5):
            b = _read(raw, i + 1, 56, 2)
            ticks = b[0] | (b[1] << 8)
            q.append(getattr(cal, f"{j}_sign") * math.radians(ticks * 360 / 4096 - getattr(cal, f"{j}_zero_offset_deg")))
        return q
    finally:
        t.close()


def live_so101_pose_source(cfg, backend) -> Callable[[], Dict[str, List[float]]]:
    """A pose source that re-reads both arms on every call."""
    return lambda: {arm: read_arm_pose(cfg, backend, arm) for arm in ("left", "right")}
