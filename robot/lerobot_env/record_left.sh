#!/usr/bin/env bash
# AA-44 / AA-23: record LOCAL LeRobot episodes of the LEFT follower from the leader arm.
# Nothing is uploaded (--dataset.push_to_hub=false). Each run writes a NEW folder
#   robot/datasets/<NAME>_<UTC stamp>/   (gitignored)
#
#   bash ~/893/robot/lerobot_env/record_left.sh                       # 3 practice episodes, no tube
#   DATASET=insert_left EPISODES=10 bash ~/893/robot/lerobot_env/record_left.sh
#   bash ~/893/robot/lerobot_env/record_left.sh --dataset.episode_time_s=30   # extra flags pass through
#
# Keys while recording: n = end this episode early, r = re-record it, q = stop and save.
# Ctrl-C = stop; the arm HOLDS. At the release prompt, support the arm and press Ctrl-C
# (Enter may be swallowed by LeRobot's key listener) to switch torque off.
# Between episodes there is a RESET phase (default 10 s): the arm keeps following the leader.
set -euo pipefail
cd "$(dirname "$0")"
DATASET="${DATASET:-practice_left}"
EPISODES="${EPISODES:-3}"
EPISODE_S="${EPISODE_S:-20}"
RESET_S="${RESET_S:-10}"
TASK="${TASK:-practice teleop, no tube}"
TUBE="${TUBE:-no_tube}"
ROOT="$(cd .. && pwd)/datasets/${DATASET}_$(date -u +%Y%m%dT%H%M%SZ)"
echo "Recording ${EPISODES} x ${EPISODE_S}s (reset ${RESET_S}s) into ${ROOT}  tube_support=${TUBE}"
exec uv run lerobot-record \
  --robot.type=inchworm_follower \
  --robot.id=left \
  --robot.camera=true \
  --robot.tube_support="${TUBE}" \
  --robot.no_collision_check="owner 2026-11-13 - first teleop without mesh check" \
  --robot.allow_unverified_soft_limits="owner 2026-11-15 - AA-44 LeRobot recording" \
  --teleop.type=inchworm_leader \
  --teleop.id=leader \
  --dataset.repo_id="local/${DATASET}" \
  --dataset.root="${ROOT}" \
  --dataset.no_stamp=true \
  --dataset.single_task="${TASK}" \
  --dataset.fps=30 \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.episode_time_s="${EPISODE_S}" \
  --dataset.reset_time_s="${RESET_S}" \
  --dataset.push_to_hub=false \
  --play_sounds=false \
  "$@"
