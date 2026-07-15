#!/bin/bash
# SessionStart hook for Claude Code on the web: make `pio test -e native` runnable
# even when the sandbox egress policy blocks the PlatformIO registry.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
    exit 0
fi

"$CLAUDE_PROJECT_DIR/scripts/agent_setup.sh"
