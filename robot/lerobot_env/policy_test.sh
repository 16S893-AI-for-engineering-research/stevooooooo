#!/usr/bin/env bash
# AA-39 (owner 2026-11-20): ONE launcher for a policy test from the demo-start handoff.
#   1. scripts/replay_cycle.py (main robot env) carries the tube to the arm's handoff and STOPS
#      holding: --arm left = frame 42 (right LOCKED), --arm right = frame 67 (left LOCKED).
#   2. Only if that succeeded AND wrote a fresh handoff state for this arm: policy_handoff.py
#      (lerobot_env) in the same terminal. --arm right runs the LEFT-trained policy through the
#      per-arm joint zero-offset map (inchworm.arm_transfer).
# Both scripts keep their own YES prompts. If the replay fails or is not confirmed, the policy
# step never starts.
#   bash ~/893/robot/lerobot_env/policy_test.sh --arm right --dry-run   # no hardware at all
#   bash ~/893/robot/lerobot_env/policy_test.sh --arm right             # live, owner supervising
# Extra arguments after -- go to policy_handoff.py (e.g. -- --max-s 15).
# Start state (live): every servo torque OFF, grippers open and empty, arms roughly folded
# (bash ~/893/robot/reset_start.sh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT="$(dirname "$HERE")"
ARM="" DRY=0 EXTRA=()
APPROVAL="owner 2026-11-20 - AA-39 policy test"
while [ $# -gt 0 ]; do
  case "$1" in
    --arm) ARM="${2:-}"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --allow-unverified-soft-limits) APPROVAL="${2:-}"; shift 2 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "REFUSED: unknown argument $1 (usage: --arm left|right [--dry-run] [-- policy args])"; exit 2 ;;
  esac
done
case "$ARM" in
  left) FRAME=42 ;;
  right)
    # Owner 2026-11-20 after the first live run: the replay to frame 67 does the LEFT insertion +
    # lock scripted (frames 43-49), without the policy, and that failed. Disabled until the full
    # left-policy -> lock -> continue -> right-policy test replaces it.
    echo "REFUSED: --arm right is disabled (the replay to frame 67 inserts the left arm WITHOUT the policy)"
    exit 2 ;;
  *) echo "REFUSED: --arm left|right is required"; exit 2 ;;
esac
STATE="$ROBOT/out/handoff_state.json"
[ "$DRY" = 1 ] && STATE="$ROBOT/out/sim_handoff_state.json"
T0=$(date +%s)

echo "== 1/2 replay_cycle: frames 0..$FRAME with the tube, stop holding ($ARM arm free)"
cd "$ROBOT"
if [ "$DRY" = 1 ]; then
  uv run python scripts/replay_cycle.py --stop-after-frame "$FRAME" --tube-in-grippers
else
  uv run python scripts/replay_cycle.py --stop-after-frame "$FRAME" --tube-in-grippers --live \
    --allow-unverified-soft-limits "$APPROVAL"
fi

# replay_cycle exits 0 also when the YES is not given (nothing moved): require a state file that
# this run wrote, for this arm's stop frame, with this arm open and the other LOCKED.
if ! python3 - "$STATE" "$T0" "$FRAME" "$ARM" <<'EOF'
import json, os, sys
path, t0, frame, arm = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
other = "left" if arm == "right" else "right"
if not os.path.exists(path) or os.path.getmtime(path) < t0:
    sys.exit(f"REFUSED: {path} was not written by this replay run (not confirmed or failed); "
             "the policy step does not start")
st = json.load(open(path))
if st["stop_frame"] != frame or st["locked"][arm] or not st["locked"][other] or not st["tube_in_grippers"]:
    sys.exit(f"REFUSED: {path}: stop frame {st['stop_frame']}, locked {st['locked']}, tube "
             f"{st['tube_in_grippers']} (want frame {frame}, {arm} open, {other} LOCKED, tube in)")
print(f"handoff state OK: frame {frame}, {other} LOCKED, {arm} open ({st['utc']})")
EOF
then
  exit 3
fi

echo "== 2/2 policy_handoff on the $ARM arm (left-trained checkpoint)"
cd "$HERE"
ARGS=(--arm "$ARM" --policy-arm left --handoff-state "$STATE" --allow-unverified-soft-limits "$APPROVAL")
[ "$DRY" = 1 ] && ARGS+=(--dry-run)
exec uv run python policy_handoff.py "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
