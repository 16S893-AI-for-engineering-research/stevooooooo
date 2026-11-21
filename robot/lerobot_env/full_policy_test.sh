#!/usr/bin/env bash
# AA-39 FULL policy test (owner 2026-11-20): left policy -> left lock + owner look -> left to the
# demo-end pose -> cycle frames 49..67 (right unlock, right carry) -> right policy (left-trained,
# joint-offset map) -> right lock + owner look -> release prompt.
#   1. scripts/replay_cycle.py (main robot env): frames 0..42 with the tube, stop holding.
#   2. Only if that wrote a fresh frame-42 state (left open, right LOCKED, tube in):
#      full_policy_test.py (lerobot_env) in the same terminal. Every phase keeps its YES prompts.
#   bash ~/893/robot/lerobot_env/full_policy_test.sh --dry-run   # no hardware at all
#   bash ~/893/robot/lerobot_env/full_policy_test.sh             # live, owner supervising
# Extra arguments after -- go to full_policy_test.py (e.g. -- --max-s 15).
# Start state (live): every servo torque OFF, grippers open and empty (bash ~/893/robot/reset_start.sh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT="$(dirname "$HERE")"
DRY=0 EXTRA=()
APPROVAL="owner 2026-11-20 - AA-39 full policy test"
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --allow-unverified-soft-limits) APPROVAL="${2:-}"; shift 2 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "REFUSED: unknown argument $1 (usage: [--dry-run] [-- full_policy_test args])"; exit 2 ;;
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
             "the policy test does not start")
st = json.load(open(path))
if st["stop_frame"] != 42 or st["locked"]["left"] or not st["locked"]["right"] or not st["tube_in_grippers"]:
    sys.exit(f"REFUSED: {path}: stop frame {st['stop_frame']}, locked {st['locked']}, tube "
             f"{st['tube_in_grippers']} (want frame 42, left open, right LOCKED, tube in)")
print(f"handoff state OK: frame 42, right LOCKED, left open ({st['utc']})")
PY
then
  exit 3
fi

echo "== 2/2 full_policy_test: left policy, lock, cycle 49..67, right policy, lock"
cd "$HERE"
ARGS=(--handoff-state "$STATE" --allow-unverified-soft-limits "$APPROVAL")
[ "$DRY" = 1 ] && ARGS+=(--dry-run)
exec uv run python full_policy_test.py "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
