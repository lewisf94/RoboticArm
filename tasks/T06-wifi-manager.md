# T06 — WiFi manager: STA + AP fallback + mDNS

**Milestone:** M2 · **Depends on:** T05 · **Touches:** `src/`, `docs/protocol.md`

## Goal
Device gets on the network without hardcoded credentials and is findable at `roboarm.local`.

## Spec
- New `src/wifi_manager.{h,cpp}` (plain functions/struct, no core logic):
  - Credentials in NVS (namespace `net`, keys `ssid`/`pass`).
  - Boot: if creds exist, try STA for 15 s → on failure or no creds, start AP `RoboArm-Setup` (password `roboarm123`, channel 1) and keep serving.
  - mDNS `roboarm` in both modes.
  - Non-blocking: motion/serial must keep running during connect attempts (state machine polled from `loop()`, no `delay`).
- Protocol: implement `wifi_set` (serial-only — the WS transport must reject it later; note this in code where the dispatcher is wired): stores creds, acks, reboots after 500 ms. Add a `wifi` object to the `state` message: `{"mode":"sta"|"ap"|"off","ip":"…","rssi":int}` — **update `docs/protocol.md` in this commit.**
- Log transitions on serial as comment lines prefixed `#` (not JSON) so scripts can skip them.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Fresh flash → AP appears; `wifi_set` over serial → reboots into STA; `ping roboarm.local` works; wrong password → falls back to AP after 15 s; servo control over serial unaffected throughout.

## Out of scope
HTTP/WS server (T07), captive portal page.
