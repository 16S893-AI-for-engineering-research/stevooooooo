"""AA-44: LeRobot plugin for the inchworm cell.

Registers robot ``inchworm_follower`` and teleoperator ``inchworm_leader``. LeRobot imports this
package via ``register_third_party_plugins()`` (distribution name prefix ``lerobot_robot_``).
Installed editable into robot/lerobot_env. Neither class uses LeRobot's Feetech bus, so no
servo memory is written.
"""
from .config_inchworm import InchwormFollowerConfig, InchwormLeaderConfig
from .inchworm_follower import InchwormFollower, InchwormRefused
from .inchworm_leader import InchwormLeader

__all__ = ["InchwormFollowerConfig", "InchwormLeaderConfig", "InchwormFollower", "InchwormLeader",
           "InchwormRefused"]
