#!/usr/bin/env bash
# AA-23: get back to the demo-start state after a handoff / recording session.
# A handoff session ends with joints 1-5 off but BOTH grippers still torque ON (right LOCKED on the
# tube). This: (1) shows the state, (2) asks you to take the tube out / hold it, (3) gripper torque
# off on both arms, (4) opens any gripper that is not open (AA-17 state machine, empty gripper),
# (5) you fold both arms by hand, (6) read-only start check until READY.
# Never opens a gripper before you confirm the tube is out of the grippers or held by your hand.
#   bash ~/893/robot/reset_start.sh
set -euo pipefail
cd "$(dirname "$0")"
band() { uv run python scripts/hold_tube.py status --arm "$1" --live 2>/dev/null | tail -1 \
         | python3 -c "import sys,json; print(json.loads(sys.stdin.read())['gripper_band'])"; }
echo "== state now (read-only)"
uv run python scripts/start_check.py || true
echo
read -r -p "Is the tube OUT of both grippers, or held by your hand? Type YES: " ok
[ "$ok" = "YES" ] || { echo "stopped; nothing changed"; exit 1; }
for arm in left right; do
  echo "== $arm: joints 1-5 + gripper torque OFF (support the $arm arm)"
  uv run python scripts/hold_tube.py release --arm "$arm" --live --gripper-off
done
for arm in left right; do
  b=$(band "$arm")
  if [ "$b" != "open" ]; then
    echo "== $arm gripper is $b: opening it (slow, ~3 s, empty gripper)"
    uv run python scripts/gripper_ends.py --arm "$arm" --action open --no-tube --live
  fi
done
while true; do
  echo
  read -r -p "Fold both arms by hand into the start pose, then press Enter (q = stop): " k
  [ "$k" = "q" ] && exit 1
  if uv run python scripts/start_check.py; then exit 0; fi
done
