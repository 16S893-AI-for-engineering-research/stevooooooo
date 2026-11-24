#!/usr/bin/env bash
# AA-54: right-arm insertion recording from train_right (frozen; owner plan 2026-11-21). See record_train.sh.
#   bash ~/893/robot/lerobot_env/record_train_right.sh [--dry-run] [--profile aa54|v6] [-- record_train args]
exec bash "$(dirname "$0")/record_train.sh" --arm right "$@"
