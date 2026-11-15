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
    #: "no_tube" (operator states the cell has no tube) or "locked_arm" (the other arm holds the
    #: tube: its gripper reads inside the locked band with torque on, checked read-only).
    tube_support: str = ""
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
