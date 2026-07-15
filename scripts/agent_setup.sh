#!/usr/bin/env bash
# Prepare a (possibly network-restricted) environment so `pio test -e native` works.
#
# Claude Code web sandboxes may block api.registry.platformio.org (egress policy),
# which breaks every `pio` download. PyPI and github.com are typically reachable,
# so this script rebuilds the native-test toolchain from those sources instead:
#   - platformio + SCons via pip
#   - platform-native from GitHub, with a tool-scons shim package
#   - ArduinoJson + Unity into .pio/libdeps/native from GitHub
#
# The esp32s3 platform ships registry-hosted binary toolchains and has no such
# fallback — firmware compilation is covered by GitHub Actions CI in that case.
#
# Idempotent; safe to re-run. Called by .claude/hooks/session-start.sh, or run
# manually: ./scripts/agent_setup.sh

set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
say() { echo "[agent_setup] $*"; }
set_env() { [ -n "${CLAUDE_ENV_FILE:-}" ] && echo "export $1=$2" >> "$CLAUDE_ENV_FILE"; true; }

# ---- 1. PlatformIO itself -----------------------------------------------------
if ! command -v pio >/dev/null 2>&1; then
    say "installing platformio via pip"
    pip3 install --quiet platformio || { say "ERROR: cannot install platformio"; exit 1; }
fi
pio settings set enable_telemetry no >/dev/null 2>&1 || true

# ---- 2. Probe the PlatformIO registry ------------------------------------------
if curl -fsS --max-time 8 "https://api.registry.platformio.org/v3/search?query=native" >/dev/null 2>&1; then
    say "registry reachable — standard pio flow will work, no fallbacks needed"
    set_env ARM_PIO_REGISTRY ok
    exit 0
fi
say "registry unreachable — installing native-test toolchain from GitHub/PyPI"
set_env ARM_PIO_REGISTRY blocked

# ---- 3. native platform (no binary packages — installable from source) ---------
if [ ! -d "$HOME/.platformio/platforms/native" ]; then
    say "installing platform-native from GitHub"
    pio pkg install -g -p "https://github.com/platformio/platform-native.git" >/dev/null 2>&1 \
        || { say "ERROR: platform-native install failed"; exit 1; }
fi

# ---- 4. tool-scons shim (pio-core pins ~4.40801.0 == SCons 4.8.1) ---------------
python3 -c "import SCons" 2>/dev/null || {
    say "installing SCons 4.8.1 via pip"
    pip3 install --quiet "scons==4.8.1" || { say "ERROR: cannot install scons"; exit 1; }
}
SCONS_PKG="$HOME/.platformio/packages/tool-scons"
mkdir -p "$SCONS_PKG"
printf '%s' '{"name": "tool-scons", "version": "4.40801.0", "description": "SCons shim (pip-backed)", "system": "*"}' > "$SCONS_PKG/package.json"
printf '%s' '{"type": "tool", "name": "tool-scons", "version": "4.40801.0", "spec": {"owner": "platformio", "id": null, "name": "tool-scons", "requirements": null, "uri": null}}' > "$SCONS_PKG/.piopm"
cat > "$SCONS_PKG/scons.py" <<'EOF'
#!/usr/bin/env python
# Shim: PlatformIO invokes <tool-scons>/scons.py; delegate to pip-installed SCons.
from SCons.Script.Main import main
if __name__ == "__main__":
    main()
EOF

# ---- 5. Libraries for the native env from GitHub --------------------------------
LIBS="$ROOT/.pio/libdeps/native"
mkdir -p "$LIBS"

if [ ! -d "$LIBS/ArduinoJson" ]; then
    say "cloning ArduinoJson v7.4.2"
    git clone --quiet --depth 1 --branch v7.4.2 \
        https://github.com/bblanchon/ArduinoJson.git "$LIBS/ArduinoJson" \
        || { say "ERROR: ArduinoJson clone failed"; exit 1; }
fi
printf '%s' '{"type": "library", "name": "ArduinoJson", "version": "7.4.2", "spec": {"owner": "bblanchon", "id": null, "name": "ArduinoJson", "requirements": null, "uri": null}}' > "$LIBS/ArduinoJson/.piopm"

if [ ! -d "$LIBS/Unity" ]; then
    say "cloning Unity v2.6.1"
    git clone --quiet --depth 1 --branch v2.6.1 \
        https://github.com/ThrowTheSwitch/Unity.git "$LIBS/Unity" \
        || { say "ERROR: Unity clone failed"; exit 1; }
fi
# Upstream library.json carries a '#' comment banner the registry normally strips.
python3 - "$LIBS/Unity/library.json" <<'EOF'
import json, sys
p = sys.argv[1]
s = open(p).read()
s = s[s.index("{"):]
d = json.loads(s)
d["version"] = "2.6.1"
json.dump(d, open(p, "w"), indent=2)
EOF
printf '%s' '{"type": "library", "name": "Unity", "version": "2.6.1", "spec": {"owner": "throwtheswitch", "id": null, "name": "Unity", "requirements": null, "uri": null}}' > "$LIBS/Unity/.piopm"

say "done — 'pio test -e native' is usable; esp32s3 builds are delegated to CI here"
