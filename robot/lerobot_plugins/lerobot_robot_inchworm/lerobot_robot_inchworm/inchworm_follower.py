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
* ``tube_support=handoff`` (AA-23 demo start, owner 2026-11-15; see ``inchworm.handoff``):
  the cycle replay stopped and left BOTH arms holding the tube. connect() refuses unless the
  handoff state matches the servos (both arms torque ON at the stop pose, the other gripper
  torque ON in its locked band, this gripper not locked); it never cuts torque. The mesh
  collision check is ON (tube where the cycle left it, other arm where it reads). The first
  actions CATCH UP slowly (catchup_deg_s, default 5 deg/s) from the stop pose to the leader
  (replaces the 10 deg start gate); a goal that touches anything is not sent (the arm holds,
  then catches up again). ``policy_no_mesh_check`` (AA-57, policy runners only, owner
  2026-11-24): goals after the catch-up are not mesh-checked (logged at connect).
  disconnect(): support the tube and both arms, Enter, joints 1-5 of
  BOTH arms torque OFF; the grippers are never touched (the other one stays LOCKED).
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
TUBE_SUPPORT = ("no_tube", "locked_arm", "handoff")
CAMERA_KEY = "wrist"


def follower_features(config) -> tuple:
    """(observation_features, action_features) of the follower for ``config``: joints 1-5 as
    ``<joint>.pos`` floats (raw servo degrees) and, with the camera, ``wrist`` (H, W, C).
    The AA-26 policy runner builds its LeRobot dataset features from this, so training data
    and deployment observations use one definition."""
    from inchworm import teleop_live as tl
    motors = {f"{j}.pos": float for j in tl.J5}
    obs: Dict[str, Any] = dict(motors)
    if config.camera:
        obs[CAMERA_KEY] = (config.camera_height, config.camera_width, config.camera_channels)
    return obs, dict(motors)


def format_frame(data: np.ndarray, height: int, width: int, channels: int) -> np.ndarray:
    """Camera frame -> (H, W, channels) uint8 as recorded: grey (H, W) or (H, W, 1) is copied
    into 3 channels when ``channels == 3``. Raises on a frame of the wrong size."""
    if data.shape[:2] != (height, width):
        raise RuntimeError(f"camera frame {data.shape} != configured {height}x{width}")
    if data.ndim == 2:
        data = data[:, :, None]
    if channels == 3 and data.shape[2] == 1:
        data = np.repeat(data, 3, axis=2)
    return np.ascontiguousarray(data, dtype=np.uint8)


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
    backend = live_backend(cfg)            # same refusal of INCHWORM_BACKEND=mock as live runs
    if getattr(getattr(cfg, "camera_transport", None), "value", "usbip") == "bridge":
        return backend                     # AA-53: HardwareBackend opens the Windows camera bridge

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

    def other_arm_bus(self, other: str) -> tuple:
        """(read_bytes, write_bytes, close) for the other arm: handoff end-of-session torque-off
        of joints 1-5 only (inchworm.handoff.joints_off never touches servo 6)."""
        from inchworm.servo_bus import _ServoBus
        t = self._be.open_serial(f"arm_{other}")
        b = _ServoBus(t)
        return b.read_bytes, b.write_bytes, t.close

    def other_arm_servo_bus(self, other: str) -> tuple:
        """(ArmServoBus, read_bytes, close) for the other arm: AA-23 return to its stop pose
        (joints 1-5 sync writes only; the bus itself rejects gripper writes)."""
        from inchworm.servo_bus import ArmServoBus
        from inchworm import teleop_live as tl
        t = self._be.open_serial(f"arm_{other}")
        b = ArmServoBus(t, role=f"arm_{other}", joint_id_map={n: i + 1 for i, n in enumerate((*tl.J5, "gripper"))})
        return b, b._bus.read_bytes, t.close

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
        self._handoff: Optional[dict] = None      # handoff mode: state, gate, catch-up flags
        self._cfg = None
        self.episode: Optional[int] = None         # record_handoff.py: tags log lines
        self.returns_done = 0

    def prompt(self, msg: str) -> str:                 # tests replace it
        return input(msg)

    @property
    def cameras(self) -> dict:
        """Camera names (lerobot-record sizes its image writer threads with len(robot.cameras))."""
        return {CAMERA_KEY: f"arm_{self.config.follower} wrist camera via the AA-6 arbiter"} if self.config.camera else {}

    # -- features -----------------------------------------------------------
    @cached_property
    def observation_features(self) -> dict:
        return follower_features(self.config)[0]

    @cached_property
    def action_features(self) -> dict:
        return follower_features(self.config)[1]

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
        if self.episode is not None:
            rec.setdefault("episode", self.episode)
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
        if c.tube_support == "handoff":
            if c.no_collision_check:
                out.append("tube_support=handoff runs the mesh collision check; drop --robot.no_collision_check")
            if not 0.5 <= float(c.catchup_deg_s) <= 10.0:
                out.append("catchup_deg_s must be 0.5..10")
        elif not c.no_collision_check:
            out.append("this path has no mesh collision check; set --robot.no_collision_check=\"<owner reason>\"")
        if c.policy_no_mesh_check and c.tube_support != "handoff":
            out.append("policy_no_mesh_check applies to tube_support=handoff only")
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
        cfg = self._cfg = self._load_cfg()
        probs = self._static_refusals(cfg)
        if probs:
            raise InchwormRefused("REFUSED: " + "; ".join(probs))
        self._open_log()
        self.hw = self._open_hardware(cfg)
        try:
            handoff = c.tube_support == "handoff"
            limits = tl.follower_preflight(cfg, c.follower, self.hw.read_bytes, require_torque_off=not handoff)
            tube = {"tube_support": c.tube_support}
            if c.tube_support in ("locked_arm", "handoff"):
                tube.update(self._check_locked_arm(cfg))
            if handoff:
                tube.update(self._check_handoff(cfg))
                now = [int.from_bytes(self.hw.read_bytes(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]
                out = [f"{j} at {t} outside {round(limits[j][0])}..{round(limits[j][1])}"
                       for j, t in zip(tl.J5, now) if not limits[j][0] <= t <= limits[j][1]]
                if out:   # the first clamped command would jump to the limit
                    raise InchwormRefused("tube_support=handoff: the stop pose is outside the teleop limits: "
                                          + "; ".join(out))
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
                       "no_collision_check": c.no_collision_check,
                       "policy_no_mesh_check": c.policy_no_mesh_check, **tube})
            if handoff and c.policy_no_mesh_check:
                self._log({"event": "mesh_check_skipped_policy", "reason": c.policy_no_mesh_check,
                           "scope": "policy goals after the start catch-up; catch-up, returns and "
                                    "planned moves stay mesh-checked"})
            if c.confirm and handoff:
                print("\nCONFIRM before anything moves (demo start, both arms are holding now):\n"
                      "  - the other arm is LOCKED on the tube, torque on; it will not be commanded\n"
                      f"  - the {c.follower} arm will move SLOWLY (~{c.catchup_deg_s:g} deg/s) from where it holds to "
                      "where the LEADER is, then follow it; every goal is checked against the cell model "
                      "(tube where the cycle left it)\n"
                      f"  - hands and cables clear of the {c.follower} follower; you are at the power switch\n"
                      "Ctrl-C = hold. At the end: support the TUBE and BOTH arms; joints of both arms go "
                      "torque OFF, the grippers stay as they are.")
                if self.prompt("Type YES to continue: ").strip() != "YES":
                    raise InchwormRefused("not confirmed; nothing changed (both arms still holding)")
            elif c.confirm:
                print("\nCONFIRM before anything moves:\n"
                      + ("  - NO tube in the cell\n" if c.tube_support == "no_tube" else
                         "  - the other arm is LOCKED on the tube, torque on; it will not be commanded\n")
                      + f"  - hands and cables clear of the {c.follower} follower; you are at the power switch\n"
                      f"Torque goes on at the first action if the leader is within 10 deg of the follower "
                      f"(about {c.speed_ticks_s * 360 / 4096:.0f} deg/s cap). Ctrl-C = hold.")
                if self.prompt("Type YES to continue: ").strip() != "YES":
                    raise InchwormRefused("not confirmed; nothing energised")
            STOP_FILE.unlink(missing_ok=True)
            if handoff:
                self.core.adopt_holding()          # no torque cut, no goal write
                self._log({"event": "handoff_adopted", "follower_ticks": self.core.follower_ticks()})
        except BaseException as exc:
            self._log({"event": "refused", "reason": f"{type(exc).__name__}: {exc}"})
            self._close_all()
            if isinstance(exc, tl.TeleopRefused):
                raise InchwormRefused(f"REFUSED: {exc}") from exc
            raise
        self._connected = True

    def _check_handoff(self, cfg) -> dict:
        """Read-only: the handoff state matches the servos (inchworm.handoff). Builds the mesh gate."""
        from inchworm import handoff as ho
        from inchworm.so101_pose_reader import ticks_to_q
        c, tl = self.config, self._tl
        other = "right" if c.follower == "left" else "left"
        st = ho.load_state(Path(c.handoff_state) if c.handoff_state else ho.STATE_PATH)
        probs = ho.state_problems(st, c.follower) + ho.arm_problems(c.follower, self.hw.read_bytes, st)
        read, close = self.hw.other_arm_read_bytes(other)
        try:
            probs += ho.arm_problems(other, read, st)
            other_ticks = [int.from_bytes(read(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]
        finally:
            close()
        bands = getattr(cfg.gripper_lock, f"arm_{c.follower}", None)
        g = int.from_bytes(self.hw.read_bytes(6, tl.REG_PRESENT_POSITION, 2), "little") * tl.DEG_PER_TICK
        if (bands is not None and bands.locked_low_deg is not None
                and bands.locked_low_deg <= g <= bands.locked_high_deg):
            probs.append(f"the {c.follower} gripper reads {g:.1f} deg, inside its LOCKED band (it holds the tube)")
        if probs:
            raise InchwormRefused("tube_support=handoff: " + "; ".join(probs))
        gate = self._make_gate(cfg, c.follower, ticks_to_q(cfg, other, other_ticks), np.asarray(st["tube_pose"]))
        present = [int.from_bytes(self.hw.read_bytes(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]
        p = gate.pose_problem(present)
        if p is not None:
            raise InchwormRefused(f"tube_support=handoff: the {c.follower} arm's present pose fails the cell check: {p}")
        self._handoff = {"state": st, "gate": gate, "catching_up": True, "held": 0, "other": other}
        return {"handoff_stop_frame": st["stop_frame"], "handoff_state_utc": st["utc"],
                "handoff_tube_in_grippers": st["tube_in_grippers"], "other_arm_ticks": other_ticks,
                "catchup_deg_s": c.catchup_deg_s}

    def _make_gate(self, cfg, follower, other_q, tube):          # tests replace it
        from inchworm.handoff import CollisionGate
        return CollisionGate(cfg, follower, other_q, tube)

    def _handoff_goal(self, raw: list) -> Optional[list]:
        """Handoff mode: rate-limited catch-up, then the leader target; every step mesh-checked,
        except with ``policy_no_mesh_check`` (AA-57) after the catch-up is done.
        Returns the follower-tick goal to command, or None = hold the last goal."""
        from inchworm import handoff as ho
        h, tl, c = self._handoff, self._tl, self.config
        target, _ = self.core.target_from_follower_ticks(raw, self.core.last_goal)
        goal = target
        if not h["catching_up"] and c.policy_no_mesh_check:
            return goal                          # AA-57: owner-approved, logged at connect
        if h["catching_up"]:
            step = c.catchup_deg_s / c.control_hz / tl.DEG_PER_TICK
            goal, done = ho.catch_up_target(self.core.last_goal, target, step)
            if done:
                h["catching_up"] = False
                self._log({"event": "catchup_done", "goal": [round(x) for x in goal],
                           "mesh_checks": h["gate"].checks,
                           "mesh_ms_mean": round(h["gate"].ms_total / max(1, h["gate"].checks), 2)})
        p = h["gate"].step_problem(self.core.last_goal, goal)
        if p is not None:
            if h["held"] == 0:
                self._log({"event": "collision_hold", "problem": p, "goal": [round(x) for x in goal]})
            h["held"] += 1
            h["catching_up"] = True              # when clear again: slow catch-up, never a jump
            return None
        if h["held"]:
            self._log({"event": "collision_clear", "held_steps": h["held"]})
            h["held"] = 0
        return goal

    @property
    def catching_up(self) -> bool:
        """Handoff mode: still in the slow catch-up to the leader (follower != leader target)."""
        return self._handoff is not None and bool(self._handoff["catching_up"])

    # -- AA-23 rapid-fire recording: automatic return between demos -------------
    def _make_return_model(self, cfg):                       # tests replace it
        from inchworm.handoff_return import CarriedTubeModel
        st, h = self._handoff["state"], self._handoff
        home, tube = dict(st["goal_ticks"]), np.asarray(st["tube_pose"])
        if h.get("hold_ticks") is not None:            # AA-54: the holding arm sits at a perturbed pose
            home[h["other"]], tube = list(h["hold_ticks"]), np.asarray(h["hold_tube"])
        return CarriedTubeModel(cfg, self._handoff["gate"].checker, home, tube, follower=self.config.follower)

    def _make_chain(self, cfg):                              # tests replace it
        from inchworm.kinematics import build_chain, with_soft_limits
        return with_soft_limits(build_chain(cfg.so101_kinematics),
                                getattr(cfg.motion_guard, f"{self.config.follower}_arm_limits"))

    def handoff_return(self, confirm: bool = True, interrupted=lambda: False, clock=time.monotonic,
                       sleep=time.sleep) -> dict:
        """Owner spec 2026-11-15: straight out of the key (50 mm, -z of the base frame), joint line
        to the stop pose, then the other arm (tube carried) to its stop pose. The whole path is
        mesh-checked before anything moves; ``confirm`` asks for a typed YES first. Grippers are
        never commanded. Raises ReturnRefused (nothing moved) / ReturnAborted (holding)."""
        from inchworm import handoff_return as hr
        if not self._connected or self._handoff is None:
            raise RuntimeError("handoff_return needs a connected tube_support=handoff robot")
        tl, cfg, h, f = self._tl, self._cfg, self._handoff, self.config.follower
        o = h["other"]
        bus, read, close = self.hw.other_arm_servo_bus(o)
        try:
            off = [s for s in range(1, 6) if not read(s, tl.REG_TORQUE_ENABLE, 1)[0]]
            if off:
                raise hr.ReturnRefused(f"the {o} arm joints {off} are torque OFF. Nothing moved")
            o_limits = tl.follower_preflight(cfg, o, read, require_torque_off=False)
            now = {f: self.core.follower_ticks(),
                   o: [int.from_bytes(read(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]}
            plan = hr.plan_return(cfg, self._make_return_model(cfg), self._make_chain(cfg),
                                  {f: self.core.limits, o: o_limits}, now)
            self._log({"event": "return_planned", "from": now, "legs": plan.summary, "mesh_checks": plan.checks})
            if confirm:
                print("\nRETURN (first of this session, owner supervises):\n  " + "\n  ".join(plan.summary)
                      + f"\n  grippers are not touched; the {o} gripper stays LOCKED. Ctrl-C = hold.")
                if self.prompt("Type YES to move: ").strip() != "YES":
                    raise hr.ReturnRefused("return not confirmed. Nothing moved")
            ocore = hr.holding_core(cfg, o, bus, read, log=self._log)
            legs = [hr.run_leg(self.core if leg.arm == f else ocore, leg, hz=self.config.control_hz,
                               log=self._log, interrupted=interrupted, clock=clock, sleep=sleep)
                    for leg in plan.legs]
        except hr.ReturnRefused as exc:
            self._log({"event": "return_refused", "reason": str(exc)})
            raise
        except BaseException as exc:
            self._log({"event": "return_aborted", "reason": f"{type(exc).__name__}: {exc}"})
            raise
        finally:
            close()
        h["catching_up"], h["held"] = True, 0     # next demo starts with the slow catch-up again
        self.returns_done += 1
        self._log({"event": "return_done", "legs": legs})
        return {"legs": legs, "plan": plan.summary}

    # -- AA-54 perturbed tube placement: the HOLDING arm moves the tube between demos -----
    def move_holding(self, target_ticks, tube_after, confirm: bool = True, interrupted=lambda: False,
                     clock=time.monotonic, sleep=time.sleep, deg_s: Optional[float] = None) -> dict:
        """Move the other (holding, gripper LOCKED) arm with the tube along a joint line to
        ``target_ticks``; the follower holds still (it must be at its start pose, out of the key).
        Every step is mesh-checked with the tube carried before anything moves; afterwards the
        teleop gate and the return use the new holding pose and ``tube_after``. Grippers are
        never commanded. Raises ReturnRefused (nothing moved) / ReturnAborted (holding)."""
        from inchworm import handoff_return as hr
        if not self._connected or self._handoff is None:
            raise RuntimeError("move_holding needs a connected tube_support=handoff robot")
        tl, cfg, h, f = self._tl, self._cfg, self._handoff, self.config.follower
        o = h["other"]
        bus, read, close = self.hw.other_arm_servo_bus(o)
        try:
            off = [s for s in range(1, 6) if not read(s, tl.REG_TORQUE_ENABLE, 1)[0]]
            if off:
                raise hr.ReturnRefused(f"the {o} arm joints {off} are torque OFF. Nothing moved")
            o_limits = tl.follower_preflight(cfg, o, read, require_torque_off=False)
            fnow = self.core.follower_ticks()
            onow = [int.from_bytes(read(s, tl.REG_PRESENT_POSITION, 2), "little") for s in range(1, 6)]
            model = self._make_return_model(cfg)
            expect = model.home[o]
            drift = max(abs(a - b) for a, b in zip(onow, expect)) * hr.DEG_PER_TICK
            if drift > hr.RIGHT_DRIFT_MAX_DEG:
                raise hr.ReturnRefused(f"the {o} arm is {drift:.1f} deg from where it should hold the tube "
                                       f"(max {hr.RIGHT_DRIFT_MAX_DEG:g}): the tube was probably moved. Nothing moved")
            target = [float(t) for t in target_ticks]
            out = [j for j, t in zip(tl.J5, target) if not o_limits[j][0] <= t <= o_limits[j][1]]
            if out:
                raise hr.ReturnRefused(f"holding target outside the teleop limits on {out}. Nothing moved")
            pts = hr._line(onow, target)
            for w in pts[1:]:
                p = model.problem(fnow, w)
                if p is not None:
                    raise hr.ReturnRefused(f"{o} (with the tube) to the perturbed pose: {p}. Nothing moved")
            d = hr._deg(onow, target)
            leg = hr.Leg(o, f"{o} (with the tube) to the next placement", pts,
                         max(0.5, d / (deg_s or hr.RETURN_DEG_S)))
            self._log({"event": "holding_move_planned", "from": onow, "to": [round(x) for x in target],
                       "deg": round(d, 2), "s": round(leg.duration_s, 1), "mesh_checks": len(pts) - 1})
            if confirm:
                print(f"\nMOVE THE TUBE (first of this session, owner supervises): the {o} arm (gripper LOCKED) "
                      f"moves the tube {d:.1f} deg max joint, {leg.duration_s:.1f} s; the {f} arm holds still. "
                      "Ctrl-C = hold.")
                if self.prompt("Type YES to move: ").strip() != "YES":
                    raise hr.ReturnRefused("tube move not confirmed. Nothing moved")
            ocore = hr.holding_core(cfg, o, bus, read, log=self._log)
            rep = hr.run_leg(ocore, leg, hz=self.config.control_hz, log=self._log, interrupted=interrupted,
                             clock=clock, sleep=sleep)
        except hr.ReturnRefused as exc:
            self._log({"event": "holding_move_refused", "reason": str(exc)})
            raise
        except BaseException as exc:
            self._log({"event": "holding_move_aborted", "reason": f"{type(exc).__name__}: {exc}"})
            raise
        finally:
            close()
        from inchworm.so101_pose_reader import ticks_to_q
        h["hold_ticks"] = [int(round(x)) for x in target]
        h["hold_tube"] = np.asarray(tube_after, dtype=float).tolist()
        gate = h["gate"]
        gate.other_q = np.asarray(ticks_to_q(cfg, o, h["hold_ticks"]), dtype=float)
        gate.tube = np.asarray(tube_after, dtype=float)
        h["catching_up"], h["held"] = True, 0
        self.holding_moves_done = getattr(self, "holding_moves_done", 0) + 1
        self._log({"event": "holding_move_done", "leg": rep, "hold_ticks": h["hold_ticks"]})
        return rep

    def _frame(self) -> np.ndarray:
        c = self.config
        res = self._grab()
        data = getattr(res, "data", res)
        if getattr(res, "corrupt", False) or data is None or data.size == 0:
            if self._last_frame is None:                       # nothing valid yet: black frame
                return np.zeros((c.camera_height, c.camera_width, c.camera_channels), dtype=np.uint8)
            return self._last_frame                            # damaged frames never stop a run
        self._last_frame = format_frame(data, c.camera_height, c.camera_width, c.camera_channels)
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
        if self._handoff is not None:
            goal = self._handoff_goal(raw)
            if goal is None:
                return {f"{j}.pos": g * tl.DEG_PER_TICK for j, g in zip(self.joints, self.core.last_goal)}
            out = self.core.command(goal)
            return {f"{j}.pos": g * tl.DEG_PER_TICK for j, g in zip(self.joints, out["goal"])}
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
                msg = ("Support the TUBE and BOTH arms, then press Enter (or Ctrl-C): joints 1-5 of both arms "
                       "go torque OFF, the grippers stay as they are: " if self._handoff is not None else
                       "Support the arm, then press Enter (or Ctrl-C) to release torque: ")
                try:
                    self.prompt(msg)
                except (KeyboardInterrupt, EOFError):
                    pass
        finally:
            try:   # joints 1-5 only: the gripper is never touched (AA-17)
                for s in range(1, 6):
                    if self.hw.read_bytes(s, tl.REG_TORQUE_ENABLE, 1)[0]:
                        self.hw.write_bytes(s, tl.REG_TORQUE_ENABLE, b"\x00")
                extra = {}
                if self._handoff is not None:   # owner 2026-11-15: joints of BOTH arms off, grippers untouched
                    from inchworm.handoff import joints_off
                    r, w, close = self.hw.other_arm_bus(self._handoff["other"])
                    try:
                        extra = {"other_arm": self._handoff["other"], "other_joints_off": joints_off(r, w)}
                    finally:
                        close()
                self._log({"event": "released", **extra})
            except Exception as exc:  # noqa: BLE001
                print(f"WARNING: final torque-off sweep failed: {exc}")
                self._log({"event": "release_failed", "error": str(exc)})
            finally:
                if self.core is not None:
                    self.core.energised = False
                self._handoff = None
                self._connected = False
                self._close_all()

    def detach_holding(self) -> dict:
        """AA-39 full test (owner 2026-11-20): end the session WITHOUT the torque-off: the
        follower re-commands its present pose (torque on) and its port closes; the other arm is
        not touched. The servos keep holding by themselves (STS3215, no host watchdog), so the
        next step (the resumed cycle) can open both ports. No prompt. Grippers never touched."""
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        from inchworm.torque import TorqueManager
        try:
            held = TorqueManager({self.hw.role: self.hw.bus}).hold_all()
            rec = {"event": "detached_holding", "hold": {k: str(v) for k, v in held.items()},
                   "follower_ticks": self.core.follower_ticks()}
            self._log(rec)
        finally:
            self._handoff = None
            self._connected = False
            self._close_all()
        if any(v != "None" for v in rec["hold"].values()):
            raise RuntimeError(f"hold before detaching failed: {rec['hold']}")
        return rec

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
