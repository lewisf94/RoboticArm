# T07 — HTTP server, LittleFS assets, WebSocket transport

**Milestone:** M2 · **Depends on:** T06 · **Touches:** `platformio.ini`, `src/`, `web/`, `data/`, `scripts/`

## Goal
The second protocol transport: browser connects to `ws://roboarm.local/ws`, static UI served from LittleFS.

## Spec
- Enable in `platformio.ini` (already stubbed in comments): `mathieucarbou/ESPAsyncWebServer@^3`, `mathieucarbou/AsyncTCP@^3`.
- `src/web_server.{h,cpp}`:
  - Serve LittleFS `/web` at `/` (default `index.html`, cache headers off for now).
  - `/ws`: text frames → same `Protocol::handle_line` → reply frame to sender. `hello` message on client connect. Broadcast `state` at 10 Hz to all clients (reuse one serialization). Reject `wifi_set` over WS (`err bad_args`).
  - **Disconnect policy (safety rule):** connections dropping does nothing to motion — targets hold. No client-count-based disable.
  - Cap clients at 4 (close oldest).
- `scripts/sync_web.py` — copies `web/` → `data/web/` (delete-then-copy, prints file list). `data/` stays out of hand-editing; add a `data/README.md` saying so.
- Placeholder `web/index.html`: static page showing "RoboArm — UI arrives in T08" + live raw state via a 10-line inline JS WS client (proves the pipe; T08 replaces it).
- Document in README quick start: `python scripts/sync_web.py && pio run -e esp32s3 -t uploadfs`.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Browser at `http://roboarm.local` shows the placeholder with live state ticking; `websocat ws://roboarm.local/ws` + `{"cmd":"get_state"}` answers; two browser tabs both stream; killing a tab mid-move: arm holds.

## Out of scope
The real UI (T08). HTTPS/auth (out of v1 entirely — LAN device).
