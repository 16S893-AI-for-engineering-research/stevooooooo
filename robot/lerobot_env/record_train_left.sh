#!/usr/bin/env bash
# AA-54: left-arm insertion recording from train_left (frozen; owner plan 2026-11-21). See record_train.sh.
#   bash ~/893/robot/lerobot_env/record_train_left.sh [--dry-run] [-- record_train args]
exec bash "$(dirname "$0")/record_train.sh" --arm left "$@"
