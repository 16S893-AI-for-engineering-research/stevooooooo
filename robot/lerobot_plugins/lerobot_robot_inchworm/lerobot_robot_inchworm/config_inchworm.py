"""AA-44: LeRobot config classes for the inchworm follower robot and the leader teleoperator."""
from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig


@RobotConfig.register_subclass("inchworm_follower")
@dataclass(kw_only=True)
class InchwormFollowerConfig(RobotConfig):
    """One SO-101 follower of the inchworm cell, driven through ``inchworm.teleop_live``.

    Actions and observations: joints 1-5 as ``<joint>.pos`` in raw servo degrees
    (ticks x 360/4096). The gripper is not in either space (AA-17 owns it).
    """

    follower: str = "left"
    #: hardware.yaml path; empty = the inchworm default.
    config_path: str = ""
    #: Owner approval to run with ``unverified: true`` soft limits (logged). Required while unverified.
    allow_unverified_soft_limits: str = ""
    #: Owner reason for running without the mesh collision check (logged). Required.
    no_collision_check: str = ""
    #: "no_tube" (operator states the cell has no tube), "locked_arm" (the other arm holds the
    #: tube: its gripper reads inside the locked band with torque on, checked read-only) or
    #: "handoff" (the cycle replay stopped and left BOTH arms holding; mesh check ON, see
    #: inchworm.handoff).
    tube_support: str = ""
    #: tube_support=handoff (AA-23 demo start, owner 2026-11-15): handoff state written by
    #: ``scripts/replay_cycle.py --stop-after-frame N``; empty = robot/out/handoff_state.json.
    handoff_state: str = ""
    #: handoff: catch-up speed from the stop pose to the leader (deg/s per joint; max 10).
    catchup_deg_s: float = 5.0
    #: handoff, policy runners only (AA-57, owner 2026-11-24): owner reason (logged) to skip the
    #: per-goal mesh check once the start catch-up is done. The catch-up stays mesh-checked;
    #: limits, stop file and the runner's jump guard / speed cap stay. Empty = every goal checked.
    policy_no_mesh_check: str = ""
    speed_ticks_s: int = 250
    smooth_alpha: float = 1.0
    #: Wrist camera through the AA-6 arbiter. False = joints only (e.g. a first teleoperate run).
    camera: bool = True
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    #: 3 = grey frame copied into 3 channels. LeRobot 0.6.1 treats (H, W, 1) as a depth map and
    #: lerobot-record rejects uint8 frames of that shape (measured 2026-11-15).
    camera_channels: int = 3
    #: Ask for YES on the terminal before torque goes on (keep True on hardware).
    confirm: bool = True
    #: Expected control rate (LeRobot --fps); sets the Goal_Speed step period of the core.
    control_hz: float = 30.0
    #: Log directory; empty = robot/calibration_data (teleop_lerobot_<date>.jsonl).
    log_dir: str = ""


@TeleoperatorConfig.register_subclass("inchworm_leader")
@dataclass(kw_only=True)
class InchwormLeaderConfig(TeleoperatorConfig):
    """The dedicated encoder-only leader arm, read-only, mapped with leader_map_<follower>.json."""

    follower: str = "left"
    config_path: str = ""
    #: Leader map path; empty = robot/calibration_data/leader_map_<follower>.json.
    map_path: str = ""
