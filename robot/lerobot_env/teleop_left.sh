#!/usr/bin/env bash
# AA-44: supervised LeRobot teleop of the LEFT follower from the leader arm (camera off, no tube).
# Run from anywhere:  bash ~/893/robot/lerobot_env/teleop_left.sh
# Ctrl-C = hold in place; support the arm, then Enter releases torque.
set -euo pipefail
cd "$(dirname "$0")"
exec uv run lerobot-teleoperate \
  --robot.type=inchworm_follower \
  --robot.id=left \
  --robot.camera=false \
  --robot.tube_support=no_tube \
  --robot.no_collision_check="owner 2026-11-13: first teleop without mesh check" \
  --robot.allow_unverified_soft_limits="owner 2026-11-15: AA-44 first LeRobot teleop" \
  --teleop.type=inchworm_leader \
  --teleop.id=leader \
  --fps=30 \
  "$@"
