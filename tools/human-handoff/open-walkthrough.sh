#!/bin/bash
# Open the Pi Walkthrough guide for the human-handoff queue.
# Uses an interactive login shell so provider credentials from ~/.bashrc are present.
set -u
HOME_STATE="$HOME/.pi/agent/human-handoff"
REPO=/home/stevotimp/893
PI=/home/stevotimp/.local/share/pi-node/node-v22.23.2-linux-x64/bin/pi

if [ -z "${PARLEY_API_KEY:-}" ]; then
  echo "PARLEY_API_KEY is not set in this shell. Open a normal terminal (not --exec) and retry." >&2
  exit 1
fi

mkdir -p "$HOME_STATE/sessions"
cd "$REPO" || exit 1

# PI_HUMAN_GUIDE=1 is what makes the extension treat this process as the guide
# and enable /walk; without it /walk next reports "Use /human open in the main
# session." PI_HUMAN_STATE_HOME points at the shared queue/lock directory.
# PI_SESSION_* are cleared so this process does not inherit an orchestrator
# session identity when launched from inside another pi session.
unset PI_SESSION_ID PI_SESSION_FILE PI_PROVIDER PI_MODEL PI_REASONING_LEVEL
export PI_HUMAN_GUIDE=1
export PI_HUMAN_STATE_HOME="$HOME_STATE"

exec flock -n "$HOME_STATE/guide.lock" \
  "$PI" --no-extensions \
    -e "$HOME/.pi/agent/npm/node_modules/mit-parley-pi-extension" \
    -e "$REPO/.pi/extensions/human-handoff/index.ts" \
    --no-skills --no-prompt-templates --no-context-files \
    --tools read,human_queue,human_checkpoint \
    --append-system-prompt "$REPO/tools/human-handoff/walkthrough.md" \
    --session-dir "$HOME_STATE/sessions" \
    --name Walkthrough \
    --model MIT-parley/bedrock/claude-opus-5
