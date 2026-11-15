"""AA-44 / AA-23: LeRobot robot ``inchworm_follower``: one SO-101 follower of the inchworm cell.

It drives the arm through ``inchworm.teleop_live`` (the core that ran the supervised live
teleop), NOT through LeRobot's Feetech bus, so LeRobot never writes servo memory.

Action / observation: joints 1-5 as ``<joint>.pos`` in raw servo degrees (ticks x 360/4096,
any 4096 representative; the core picks the one nearest the last goal). The gripper is in
neither space: it is never read into the policy or written (AA-17 owns it).

Safety (same as ``scripts/teleop_live.py``):
* connect(): refuses without the owner's ``no_collision_check`` reason, without the
  ``allow_unverified_soft_limits`` approval while the soft limits are unverified, without a
  ``tube_support`` statement, or if ``follower_preflight`` fails (torque ON, drive registers 0,
  limits do not overlap). ``locked_arm``: the other arm's gripper must read torque ON inside
  its locked band and its joints 1-5 torque ON (read-only check). Then YES on the terminal.
  Nothing is energised in connect().
* First send_action(): start gate (every joint target within 10 deg of the follower's pose),
  then energise (goal := present), then follow. Gate refused = nothing energised, error raised.
* Limits are clamped inside the core (soft ∩ EEPROM ∩ hardware.yaml, 10 ticks inside).
* The stop file ``robot/out/STOP_TELEOP`` raises KeyboardInterrupt (same as Ctrl-C).
* disconnect(): HOLD in place (torque on), wait for the operator (Enter), then torque off on
  joints 1-5 only. Idempotent; runs on every LeRobot exit path.
* Camera: the arm's wrist camera through the AA-6 arbiter (global one-camera lock held for the
  whole connection); damaged frames are skipped (grab_valid), never raised.
"""
from __future__ import annotations

import json
import time
from functools import cached_property
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np
from lerobot.robots.robot import Robot

from .config_inchworm import InchwormFollowerConfig

ROBOT_ROOT = Path(__file__).resolve().parents[3]          # robot/
STOP_FILE = ROBOT_ROOT / "out" / "STOP_TELEOP"
TUBE_SUPPORT = ("no_tube", "locked_arm")
CAMERA_KEY = "wrist"


class InchwormRefused(RuntimeError):
    """A safety precondition refused the run (nothing was energised)."""


def camera_backend(cfg):
    """Real hardware backend whose cameras open through V4L2 (AA-5 discovery identity).

    HardwareBackend itself refuses cameras on purpose (AA-14: callers open V4L2 explicitly,
    as camera_cli/camera_probe --live do). V4L2CameraTransport.open() still refuses unless
    CameraArbiter.acquire() holds the global one-camera lock for that device.
    Found live 2026-11-15: the plugin used the plain HardwareBackend and connect() refused.
    """
    from inchworm.backends import HardwareBackend, V4L2CameraTransport, live_backend
    live_backend(cfg)                      # same refusal of INCHWORM_BACKEND=mock as live runs

    class _CameraBackend(HardwareBackend):
        def _open_camera(self, device, identity):
            return V4L2CameraTransport.open(device, identity)

    return _CameraBackend(cfg)


class FollowerHardware:
    """Real hardware handles. Tests replace ``InchwormFollower._open_hardware`` with simulated ones."""

    def __init__(self, cfg, follower: str):
        from inchworm.backends import live_backend
        from inchworm.servo_bus import ArmServoBus
        from inchworm import teleop_live as tl
        self._cfg = cfg
        self._be = live_backend(cfg)
        self.role = f"arm_{follower}"
        self._t = self._be.open_serial(self.role)
        self.bus = ArmServoBus(self._t, role=self.role,
                               joint_id_map={n: i + 1 for i, n in enumerate((*tl.J5, "gripper"))})
        self.read_bytes = self.bus._bus.read_bytes
        self.write_bytes = self.bus._bus.write_bytes
        self._cam = None

    def other_arm_read_bytes(self, other: str) -> tuple:
        """(read_bytes, close) for the other arm, read-only use only."""
        from inchworm.servo_bus import _ServoBus
        t = self._be.open_serial(f"arm_{other}")
        return _ServoBus(t).read_bytes, t.close

    def open_camera(self, follower: str, width: int, height: int, fps: int) -> Callable[[], Any]:
        from inchworm.camera_arbiter import CameraArbiter
        from inchworm.camera_capture import CameraSettings, open_capture
        from inchworm.lerobot_adapter import _ARM_ROLE_TO_BACKEND_CAMERA
        device = _ARM_ROLE_TO_BACKEND_CAMERA[f"arm_{follower}"]
        self._cam = CameraArbiter(camera_backend(self._cfg)).acquire(device)
        try:
            cap = open_capture(self._cam.__enter__(), role=device)
            cap.apply_settings(CameraSettings(width=width, height=height, fps=fps))
        except Exception:
            self.close_camera()
            raise
        return cap.grab_valid

    def close_camera(self) -> None:
        if self._cam is not None:
            cam, self._cam = self._cam, None
            cam.__exit__(None, None, None)

    def close(self) -> None:
        try:
            self.close_camera()
        finally:
            self._t.close()


class InchwormFollower(Robot):
    config_class = InchwormFollowerConfig
    name = "inchworm_follower"

    def __init__(self, config: InchwormFollowerConfig):
        super().__init__(config)
        from inchworm import teleop_live as tl
        self.config = config
        self._tl = tl
        self.joints = tl.J5
        self.hw: Optional[Any] = None
        self.core = None
        self._connected = False
        self._grab: Optional[Callable[[], Any]] = None
        self._last_frame: Optional[np.ndarray] = None
        self._log_fh = None

    def prompt(self, msg: str) -> str:                 # tests replace it
        return input(msg)

    @property
    def cameras(self) -> dict:
        """Camera names (lerobot-record sizes its image writer threads with len(robot.cameras))."""
        return {CAMERA_KEY: f"arm_{self.config.follower} wrist camera via the AA-6 arbiter"} if self.config.camera else {}

    # -- features -----------------------------------------------------------
    @property
    def _motors_ft(self) -> Dict[str, type]:
        return {f"{j}.pos": float for j in self.joints}

    @cached_property
    def observation_features(self) -> dict:
        ft: Dict[str, Any] = dict(self._motors_ft)
        if self.config.camera:
            ft[CAMERA_KEY] = (self.config.camera_height, self.config.camera_width, self.config.camera_channels)
        return ft

    @cached_property
    def action_features(self) -> dict:
        return dict(self._motors_ft)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True            # calibration = hardware.yaml + servo EEPROM, never written here

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass                   # no servo memory writes from this plugin

    # -- logging -------------------------------------------------------------
    def _log(self, rec: dict) -> None:
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": "lerobot_plugin", **rec}
        if self._log_fh is not None:
            self._log_fh.write(json.dumps(rec, default=str) + "\n")
            self._log_fh.flush()
        if rec.get("event") != "teleop_stats":
            print(json.dumps(rec, default=str), flush=True)

    def _open_log(self) -> None:
        d = Path(self.config.log_dir) if self.config.log_dir else ROBOT_ROOT / "calibration_data"
        d.mkdir(parents=True, exist_ok=True)
        self._log_fh = open(d / f"teleop_lerobot_{time.strftime('%Y-%m-%d')}.jsonl", "a")

    # -- hardware hook -------------------------------------------------------
    def _open_hardware(self, cfg) -> Any:
        return FollowerHardware(cfg, self.config.follower)

    def _load_cfg(self):
        from inchworm.config import load_config
        return load_config(self.config.config_path or None)

    # -- checks --------------------------------------------------------------
    def _static_refusals(self, cfg) -> list:
        c = self.config
        out = []
        if c.follower not in ("left", "right"):
            out.append(f"follower must be 'left' or 'right', not {c.follower!r}")
            return out
        if not c.no_collision_check:
            out.append("this path has no mesh collision check; set --robot.no_collision_check=\"<owner reason>\"")
        lim = getattr(cfg.motion_guard, f"{c.follower}_arm_limits")
        if getattr(lim, "unverified", True) and not c.allow_unverified_soft_limits:
            out.append("soft limits are unverified; set --robot.allow_unverified_soft_limits=\"<owner approval>\"")
        if c.tube_support not in TUBE_SUPPORT:
            out.append(f"set --robot.tube_support to one of {TUBE_SUPPORT} (got {c.tube_support!r})")
        if c.camera and c.camera_channels not in (1, 3):
            out.append("camera_channels must be 1 or 3")
        return out

    def _check_locked_arm(self, cfg) -> dict:
        """Read-only: the other arm holds the tube (gripper torque ON in its locked band, joints torque ON)."""
        tl = self._tl
        other = "right" if self.config.follower == "left" else "left"
        bands = getattr(cfg.gripper_lock, f"arm_{other}", None) if cfg.gripper_lock else None
        if bands is None or bands.locked_low_deg is None or bands.locked_high_deg is None:
            raise InchwormRefused(f"tube_support=locked_arm: no locked band configured for arm_{other}")
        read, close = self.hw.other_arm_read_bytes(other)
        try:
            torque = {s: read(s, tl.REG_TORQUE_ENABLE, 1)[0] for s in range(1, 7)}
            g_ticks = int.from_bytes(read(6, tl.REG_PRESENT_POSITION, 2), "little")
        finally:
            close()
        g_deg = g_ticks * tl.DEG_PER_TICK
        rep = {"other_arm": other, "gripper_ticks": g_ticks, "gripper_deg": round(g_deg, 2),
               "locked_band_deg": [bands.locked_low_deg, bands.locked_high_deg], "torque": torque}
        probs = []
        if not torque[6]:
            probs.append("gripper torque is OFF")
        if not bands.locked_low_deg <= g_deg <= bands.locked_high_deg:
            probs.append(f"gripper at {g_deg:.2f} deg is outside the locked band "
                         f"{bands.locked_low_deg}..{bands.locked_high_deg}")
        off = [s for s in range(1, 6) if not torque[s]]
        if off:
            probs.append(f"joints {off} torque OFF (the holding arm must be torque on)")
        if probs:
            raise InchwormRefused(f"tube_support=locked_arm, arm_{other}: " + "; ".join(probs))
        return rep

    # -- lifecycle -----------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self} already connected")
        tl, c = self._tl, self.config
        cfg = self._load_cfg()
        probs = self._static_refusals(cfg)
        if probs:
            raise InchwormRefused("REFUSED: " + "; ".join(probs))
        self._open_log()
        self.hw = self._open_hardware(cfg)
        try:
            limits = tl.follower_preflight(cfg, c.follower, self.hw.read_bytes)
            tube = {"tube_support": c.tube_support}
            if c.tube_support == "locked_arm":
                tube.update(self._check_locked_arm(cfg))
            self.core = tl.TeleopCore(leader_read=_no_leader, follower_bus=self.hw.bus, mapping={},
                                      limits=limits, speed_ticks_s=c.speed_ticks_s, log=self._log,
                                      smooth_alpha=c.smooth_alpha)
            self.core.period_s = 1.0 / c.control_hz
            if c.camera:
                self._grab = self.hw.open_camera(c.follower, c.camera_width, c.camera_height, c.camera_fps)
                self._frame()                                  # shape check before anything moves
            self._log({"event": "connect", "follower": c.follower, "speed": c.speed_ticks_s,
                       "smooth_alpha": c.smooth_alpha, "control_hz": c.control_hz, "camera": c.camera,
                       "follower_ticks": self.core.follower_ticks(),
                       "limits": {j: [round(x) for x in v] for j, v in limits.items()},
                       "soft_limits_override": c.allow_unverified_soft_limits,
                       "no_collision_check": c.no_collision_check, **tube})
            if c.confirm:
                print("\nCONFIRM before anything moves:\n"
                      + ("  - NO tube in the cell\n" if c.tube_support == "no_tube" else
                         "  - the other arm is LOCKED on the tube, torque on; it will not be commanded\n")
                      + f"  - hands and cables clear of the {c.follower} follower; you are at the power switch\n"
                      f"Torque goes on at the first action if the leader is within 10 deg of the follower "
                      f"(about {c.speed_ticks_s * 360 / 4096:.0f} deg/s cap). Ctrl-C = hold.")
                if self.prompt("Type YES to continue: ").strip() != "YES":
                    raise InchwormRefused("not confirmed; nothing energised")
            STOP_FILE.unlink(missing_ok=True)
        except BaseException as exc:
            self._log({"event": "refused", "reason": f"{type(exc).__name__}: {exc}"})
            self._close_all()
            if isinstance(exc, tl.TeleopRefused):
                raise InchwormRefused(f"REFUSED: {exc}") from exc
            raise
        self._connected = True

    def _frame(self) -> np.ndarray:
        c = self.config
        res = self._grab()
        data = getattr(res, "data", res)
        if getattr(res, "corrupt", False) or data is None or data.size == 0:
            if self._last_frame is None:                       # nothing valid yet: black frame
                return np.zeros((c.camera_height, c.camera_width, c.camera_channels), dtype=np.uint8)
            return self._last_frame                            # damaged frames never stop a run
        if data.shape[:2] != (c.camera_height, c.camera_width):
            raise RuntimeError(f"camera frame {data.shape} != configured {c.camera_height}x{c.camera_width}")
        if data.ndim == 2:
            data = data[:, :, None]
        if c.camera_channels == 3 and data.shape[2] == 1:
            data = np.repeat(data, 3, axis=2)
        self._last_frame = np.ascontiguousarray(data, dtype=np.uint8)
        return self._last_frame

    def get_observation(self) -> dict:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        tl = self._tl
        obs = {f"{j}.pos": t * tl.DEG_PER_TICK for j, t in zip(self.joints, self.core.follower_ticks())}
        if self.config.camera:
            obs[CAMERA_KEY] = self._frame()
        return obs

    def send_action(self, action: dict) -> dict:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        tl = self._tl
        extra = set(action) - set(self.action_features)
        if extra:
            raise ValueError(f"unexpected action keys {sorted(extra)} (the gripper is never commanded)")
        if STOP_FILE.exists():
            self._log({"event": "stop_file"})
            raise KeyboardInterrupt("stop file")
        raw = [float(action[f"{j}.pos"]) / tl.DEG_PER_TICK for j in self.joints]
        if not self.core.energised:
            probs = self.core.gate_problems(raw)
            if probs:
                self._log({"event": "start_gate_refused", "problems": probs})
                raise InchwormRefused("REFUSED (start gate, nothing energised):\n  " + "\n  ".join(probs))
            self.core.energise()
            self._log({"event": "energised"})
        out = self.core.command(raw)
        return {f"{j}.pos": g * tl.DEG_PER_TICK for j, g in zip(self.joints, out["goal"])}

    def disconnect(self) -> None:
        if self.hw is None and not self._connected:
            return
        tl = self._tl
        try:
            if self.core is not None and self.core.energised:
                from inchworm.torque import TorqueManager
                held = TorqueManager({self.hw.role: self.hw.bus}).hold_all()
                self._log({"event": "hold", "hold": {k: str(v) for k, v in held.items()},
                           "clamps": dict(self.core.clamps)})
                print(f"\nHolding the {self.config.follower} arm in place (torque on).")
                try:
                    self.prompt("Support the arm, then press Enter (or Ctrl-C) to release torque: ")
                except (KeyboardInterrupt, EOFError):
                    pass
        finally:
            try:   # joints 1-5 only: the gripper is never touched (AA-17)
                for s in range(1, 6):
                    if self.hw.read_bytes(s, tl.REG_TORQUE_ENABLE, 1)[0]:
                        self.hw.write_bytes(s, tl.REG_TORQUE_ENABLE, b"\x00")
                self._log({"event": "released"})
            except Exception as exc:  # noqa: BLE001
                print(f"WARNING: final torque-off sweep failed: {exc}")
                self._log({"event": "release_failed", "error": str(exc)})
            finally:
                if self.core is not None:
                    self.core.energised = False
                self._connected = False
                self._close_all()

    def _close_all(self) -> None:
        try:
            if self.hw is not None:
                self.hw.close()
        finally:
            self.hw = None
            if self._log_fh is not None:
                self._log_fh.close()
                self._log_fh = None


def _no_leader(*_a):
    raise RuntimeError("inchworm_follower has no leader bus: actions come from LeRobot")
