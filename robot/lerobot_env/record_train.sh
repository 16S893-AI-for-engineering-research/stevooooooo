#!/usr/bin/env bash
# AA-54 per-arm insertion recording from the FROZEN training positions (owner plan 2026-11-21).
#   1. scripts/replay_cycle.py --stop-after-frame <train_<arm> frame>: both arms to the training
#      position WITHOUT the tube (grippers empty), stop holding.
#   2. scripts/place_training_tube.py --arm <arm>: you put the tube in the HOLDING gripper, it locks.
#   3. record_train.py --arm <arm>: 50 episodes (10 nominal + 40 with the tube placement perturbed by
#      the holding arm, <= 2 deg / <= 20 mm), keys r/s/d/q as record_handoff.
#   bash ~/893/robot/lerobot_env/record_train.sh --arm left --dry-run   # no hardware at all
#   bash ~/893/robot/lerobot_env/record_train.sh --arm left             # live, owner supervising
# Extra arguments after -- go to record_train.py (e.g. -- --seed 7).
# Start state (live): every servo torque OFF, grippers open and EMPTY (bash ~/893/robot/reset_start.sh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT="$(dirname "$HERE")"
DRY=0 ARM="" EXTRA=()
APPROVAL="owner 2026-11-21 - AA-54 training recording"
while [ $# -gt 0 ]; do
  case "$1" in
    --arm) ARM="${2:-}"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --allow-unverified-soft-limits) APPROVAL="${2:-}"; shift 2 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "REFUSED: unknown argument $1 (usage: --arm left|right [--dry-run] [-- record_train args])"; exit 2 ;;
  esac
done
case "$ARM" in left|right) ;; *) echo "REFUSED: --arm left|right"; exit 2 ;; esac
FRAME=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['positions']['train_$ARM']['timeline_frame'])" \
        "$ROBOT/config/training_poses.json")
STATE="$ROBOT/out/handoff_state.json"
[ "$DRY" = 1 ] && STATE="$ROBOT/out/sim_handoff_state.json"
T0=$(date +%s)
cd "$ROBOT"
if [ "$DRY" = 0 ] && [ ! -f "calibration_data/leader_map_$ARM.json" ]; then
  echo "REFUSED: no calibration_data/leader_map_$ARM.json (fit it first: scripts/leader_mapping.py --follower $ARM)"; exit 2
fi

if [ "$DRY" = 0 ]; then
  echo "== leader check (read-only)"
  uv run python scripts/scan_leader.py --seconds 0.5 > /tmp/record_train_leader.txt 2>&1 || true
  grep "answering IDs" /tmp/record_train_leader.txt || true
  grep -q "answering IDs: \[1, 2, 3, 4, 5, 6\]" /tmp/record_train_leader.txt || {
    echo "REFUSED: the leader arm does not answer on all 6 servos: check its power and cable; nothing moved"; exit 2; }
fi

echo "== 1/3 replay_cycle: frames 0..$FRAME WITHOUT the tube, stop holding (train_$ARM)"
if [ "$DRY" = 1 ]; then
  uv run python scripts/replay_cycle.py --stop-after-frame "$FRAME"
else
  uv run python scripts/replay_cycle.py --stop-after-frame "$FRAME" --live --allow-unverified-soft-limits "$APPROVAL"
fi
if ! python3 - "$STATE" "$T0" "$FRAME" <<'PY'
import json, os, sys
path, t0, frame = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
if not os.path.exists(path) or os.path.getmtime(path) < t0:
    sys.exit(f"REFUSED: {path} was not written by this replay run (not confirmed or failed)")
st = json.load(open(path))
if st["stop_frame"] != frame or st["tube_in_grippers"]:
    sys.exit(f"REFUSED: {path}: stop frame {st['stop_frame']}, tube {st['tube_in_grippers']} (want {frame}, no tube)")
print(f"replay stop OK: frame {frame} ({st['utc']})")
PY
then exit 3; fi

echo "== 2/3 place_training_tube: the tube goes in the holding gripper, it locks"
ARGS=(--arm "$ARM" --allow-unverified-soft-limits "$APPROVAL")
[ "$DRY" = 1 ] || ARGS+=(--live)
uv run python scripts/place_training_tube.py "${ARGS[@]}" || exit 4

echo "== 3/3 record_train: episodes from train_$ARM"
cd "$HERE"
if [ "$DRY" = 1 ]; then
  exec uv run python record_train.py --arm "$ARM" --dry-run ${EXTRA[@]+"${EXTRA[@]}"}
fi
exec uv run python record_train.py --arm "$ARM" --state "$STATE" --allow-unverified-soft-limits "$APPROVAL" \
  ${EXTRA[@]+"${EXTRA[@]}"}
