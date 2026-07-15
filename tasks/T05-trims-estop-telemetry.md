# T05 — Trims in NVS, e-stop input, serial telemetry

**Milestone:** M1 · **Depends on:** T04 · **Touches:** `src/`, `docs/protocol.md` if fields change

## Goal
Bench-quality polish: calibration trims survive reboot, a physical e-stop works, and the device can stream state over serial.

## Spec
- **Trims:** implement the `persist_trim` hook with `Preferences` (NVS namespace `arm`, key `trim<j>`); load all trims into `JointModel`s at boot. `set_trim` applies live (output only — targets unchanged).
- **E-stop pin:** `pins.h` GPIO 10, input pull-up, wired NC to GND (open = triggered). Debounce ~20 ms in the main loop; on trigger behave exactly like the `estop` command (detach + disable). While the pin is open, `enable` returns `err disabled` with msg `"estop pin active"`.
- **Serial telemetry:** implement `stream` command (protocol.md): `{"cmd":"stream","on":true}` → emit the `state` line at 10 Hz until off. Off by default; never enabled on boot.
- **Status LED:** onboard RGB (GPIO 48, `neopixelWrite`): red = disabled, green = enabled, blue blink = e-stop latched.
- Fill `heap` in the state message via a firmware hook (`ESP.getFreeHeap()`).

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Set a trim, reboot, `get_profile`/observe output still trimmed. Pulling the e-stop wire kills outputs mid-move; `enable` refused until it's restored. `stream on` shows 10 Hz state lines with moving `j` values during a move.

## Out of scope
WiFi anything (T06+). Trim UI (T08).
