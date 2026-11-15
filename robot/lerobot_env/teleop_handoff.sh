#!/usr/bin/env bash
# AA-23/AA-44 demo start: take over the LEFT follower where replay_cycle.py --stop-after-frame 42
# left it (both arms holding, right gripper LOCKED). No torque cut; slow catch-up to the leader
# (~5 deg/s) with the mesh collision check ON; a predicted contact holds the arm.
# Run from anywhere:  bash ~/893/robot/lerobot_env/teleop_handoff.sh
# End: Ctrl-C, support the TUBE and BOTH arms, Enter -> joints 1-5 of both arms torque OFF,
# grippers untouched (the right one stays LOCKED).
set -euo pipefail
cd "$(dirname "$0")"
exec uv run lerobot-teleoperate \
  --robot.type=inchworm_follower \
  --robot.id=left \
  --robot.follower=left \
  --robot.camera=false \
  --robot.tube_support=handoff \
  --robot.allow_unverified_soft_limits="owner 2026-11-15 - AA-23 demo-start handoff" \
  --teleop.type=inchworm_leader \
  --teleop.id=leader \
  --fps=30 \
  "$@"
