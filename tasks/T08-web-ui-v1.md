# T08 — Web UI v1: joint sliders, enable/e-stop, trim panel

**Milestone:** M2 · **Depends on:** T07 · **Touches:** `web/`, `data/`

## Goal
Drive the arm from a phone: sliders per joint, big e-stop, connection status, trims. **Hardware gate for M2.**

## Spec
Vanilla ES modules only (CLAUDE.md rule). Files: `web/index.html`, `web/style.css`, `web/js/{app.js,ws.js,ui.js}`.
- `ws.js` — connect/reconnect (1 s backoff) to `ws://<host>/ws`; queue-less send (drop while disconnected); `hello`/`state`/`ack`/`err` event callbacks; request-id counter matching replies to callers.
- On `hello` + `get_profile`: build one row per joint — name, slider (min/max from profile, step 0.5°), numeric readout of live `j[i]`, target marker. Slider `input` events → `set_joint` **throttled to 10 Hz max**; gripper joint renders as open/close percentage instead.
- Header: connection dot, profile name, `enable` toggle, **E-STOP button (large, red, always reachable, works while disabled)**, wifi RSSI.
- Trim panel (collapsible): per-joint ±10° trim slider → `set_trim` on release (not on drag).
- Errors (`err` frames) surface as toast; `out_of_range` also flashes the joint row.
- Layout: single column, thumb-sized controls, works at 360 px wide. No CSS framework.
- Run `scripts/sync_web.py` and commit the generated `data/` state.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] Static sanity: `python -m http.server` in `web/` renders (dead WS shows disconnected state cleanly).
- [ ] **(hardware)** Phone: slide joints (smooth, no runaway repeats), e-stop drops arm instantly, trim persists after reboot, second device stays in sync.

## Out of scope
Poses (T10), sequences (T12), cartesian pad (T16), 3D view (T17).
