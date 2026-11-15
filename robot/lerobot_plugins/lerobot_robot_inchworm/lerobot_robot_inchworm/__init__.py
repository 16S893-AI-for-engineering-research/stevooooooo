"""AA-44: LeRobot plugin for the inchworm cell (WORK IN PROGRESS, 2026-11-15).

Will register robot ``inchworm_follower`` and teleoperator ``inchworm_leader``. LeRobot imports this
package via ``register_third_party_plugins()`` (distribution name prefix ``lerobot_robot_``).
Not yet written: inchworm_follower.py, inchworm_leader.py (see doc-3 / AA-44 notes). Not installed
into lerobot_env yet.
"""
from .config_inchworm import InchwormFollowerConfig, InchwormLeaderConfig

__all__ = ["InchwormFollowerConfig", "InchwormLeaderConfig"]
