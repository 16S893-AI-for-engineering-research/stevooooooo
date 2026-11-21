#!/usr/bin/env bash
# AA-55 two-policy crawl (owner plan 2026-11-21): frames 0..42 -> left policy -> left lock + look ->
# right unlock + out -> left to train_right -> right to train_right -> right policy (own checkpoint)
# -> right lock + look -> both home -> release prompt (inchworm.two_policy.PHASES).
#   1. scripts/replay_cycle.py (main robot env): frames 0..42 with the tube, stop holding.
#   2. Only if that wrote a fresh frame-42 state (left open, right LOCKED, tube in):
#      crawl_two_policy.py (lerobot_env) in the same terminal. Every phase keeps its YES prompts.
#   bash ~/893/robot/lerobot_env/crawl_two_policy.sh --dry-run   # no hardware at all
#   bash ~/893/robot/lerobot_env/crawl_two_policy.sh             # live, owner supervising
# Extra arguments after -- go to crawl_two_policy.py (e.g. -- --max-s 15, -- --left-checkpoint DIR --right-checkpoint DIR, -- --device cpu).
# Start state (live): every servo torque OFF, grippers open and empty (bash ~/893/robot/reset_start.sh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT="$(dirname "$HERE")"
DRY=0 EXTRA=()
APPROVAL="owner 2026-11-21 - AA-55 two-policy crawl"
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --allow-unverified-soft-limits) APPROVAL="${2:-}"; shift 2 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "REFUSED: unknown argument $1 (usage: [--dry-run] [-- crawl_two_policy args])"; exit 2 ;;
  esac
done
STATE="$ROBOT/out/handoff_state.json"
[ "$DRY" = 1 ] && STATE="$ROBOT/out/sim_handoff_state.json"
T0=$(date +%s)

echo "== 1/2 replay_cycle: frames 0..42 with the tube, stop holding (left arm free)"
cd "$ROBOT"
if [ "$DRY" = 1 ]; then
  uv run python scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers
else
  uv run python scripts/replay_cycle.py --stop-after-frame 42 --tube-in-grippers --live \
    --allow-unverified-soft-limits "$APPROVAL"
fi

# replay_cycle exits 0 also when the YES is not given: require a state file this run wrote.
if ! python3 - "$STATE" "$T0" <<'PY'
import json, os, sys
path, t0 = sys.argv[1], int(sys.argv[2])
if not os.path.exists(path) or os.path.getmtime(path) < t0:
    sys.exit(f"REFUSED: {path} was not written by this replay run (not confirmed or failed); "
             "the crawl does not start")
st = json.load(open(path))
if st["stop_frame"] != 42 or st["locked"]["left"] or not st["locked"]["right"] or not st["tube_in_grippers"]:
    sys.exit(f"REFUSED: {path}: stop frame {st['stop_frame']}, locked {st['locked']}, tube "
             f"{st['tube_in_grippers']} (want frame 42, left open, right LOCKED, tube in)")
print(f"handoff state OK: frame 42, right LOCKED, left open ({st['utc']})")
PY
then
  exit 3
fi

echo "== 2/2 crawl_two_policy: left policy, lock, right out, train_right, right policy, lock, home"
cd "$HERE"
ARGS=(--handoff-state "$STATE" --allow-unverified-soft-limits "$APPROVAL")
[ "$DRY" = 1 ] && ARGS+=(--dry-run)
exec uv run python crawl_two_policy.py "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
