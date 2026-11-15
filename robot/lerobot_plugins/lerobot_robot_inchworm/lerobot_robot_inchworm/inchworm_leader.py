"""AA-44: LeRobot teleoperator ``inchworm_leader``: the dedicated encoder-only leader arm.

Read-only: only Present_Position (reg 56) of IDs 1-5 is read (error bit 0x08 tolerated,
any other bit raises). Nothing is ever written to the leader. The leader ticks are mapped
with the software fit ``calibration_data/leader_map_<follower>.json`` (sign/offset per joint)
into follower degrees (ticks x 360/4096, unwrapped; the follower picks the representative
nearest its last goal). Joint 6 is never read or mapped.
"""
from __future__ import annotations

from functools import cached_property
from pathlib import Path
from typing import Any, Callable, Optional

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_inchworm import InchwormLeaderConfig

ROBOT_ROOT = Path(__file__).resolve().parents[3]          # robot/


class InchwormLeader(Teleoperator):
    config_class = InchwormLeaderConfig
    name = "inchworm_leader"

    def __init__(self, config: InchwormLeaderConfig):
        super().__init__(config)
        from inchworm import teleop_live as tl
        self.config = config
        self._tl = tl
        self.mapping: Optional[dict] = None
        self._read: Optional[Callable[[int, int, int], bytes]] = None
        self._close: Optional[Callable[[], None]] = None

    @cached_property
    def action_features(self) -> dict:
        return {f"{j}.pos": float for j in self._tl.J5}

    @cached_property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._read is not None

    @property
    def is_calibrated(self) -> bool:
        return True            # the leader map is fitted by scripts/leader_mapping.py

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _open_leader(self) -> tuple:
        """(read_bytes_tolerating, close). Tests replace this."""
        from inchworm.backends import live_backend
        from inchworm.config import load_config
        from inchworm.servo_bus import _ServoBus
        t = live_backend(load_config(self.config.config_path or None)).open_serial("arm_leader")
        return _ServoBus(t).read_bytes_tolerating, t.close

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self} already connected")
        c = self.config
        path = Path(c.map_path) if c.map_path else ROBOT_ROOT / "calibration_data" / f"leader_map_{c.follower}.json"
        self.mapping = self._tl.load_leader_map(path, c.follower)     # TeleopRefused if unusable
        self._read, self._close = self._open_leader()

    def get_action(self) -> dict:
        if not self.is_connected:
            raise RuntimeError(f"{self} is not connected")
        tl = self._tl
        lt = [int.from_bytes(self._read(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]
        f = tl.map_unwrapped(self.mapping, lt)
        return {f"{j}.pos": t * tl.DEG_PER_TICK for j, t in zip(tl.J5, f)}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._close is not None:
            close, self._close = self._close, None
            self._read = None
            close()
