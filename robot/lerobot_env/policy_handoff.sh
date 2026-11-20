#!/usr/bin/env bash
# AA-25/AA-26 first physical policy test from the demo-start handoff (owner 2026-11-19).
# First: scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live ...
#   bash ~/893/robot/lerobot_env/policy_handoff.sh --dry-run   # no hardware
#   bash ~/893/robot/lerobot_env/policy_handoff.sh             # live: g go, s stop, r return, q quit
set -euo pipefail
cd "$(dirname "$0")"
exec uv run python policy_handoff.py "$@"
