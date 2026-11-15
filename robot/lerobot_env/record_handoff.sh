#!/usr/bin/env bash
# AA-23 rapid-fire demo recording from the demo-start handoff (owner spec 2026-11-15).
# First: scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live ...
#   bash ~/893/robot/lerobot_env/record_handoff.sh --dry-run     # no hardware: plan + check the returns
#   bash ~/893/robot/lerobot_env/record_handoff.sh               # live: r record, s save, d discard, q quit
# Live wrist view while it runs: open http://localhost:8765 in a browser (--preview-port 0 = off).
# Dataset: robot/datasets/insert_left_<UTC>/ (local, gitignored). Wrist camera of the left arm.
set -euo pipefail
cd "$(dirname "$0")"
exec uv run python record_handoff.py "$@"
