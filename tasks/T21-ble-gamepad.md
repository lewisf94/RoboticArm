# T21 — BLE gamepad via Bluepad32 (stretch)

**Milestone:** M6 · **Depends on:** T20 · **Touches:** `platformio.ini`, `src/`, `docs/hardware.md`

## Goal
Xbox-series (BLE) controller drives the arm like the sticks do. **This task has an explicit bail-out path — read it.**

## Spec
- Constraint recap (docs/hardware.md): ESP32-S3 = BLE only. Target BLE HID pads (Xbox Series X|S). DualShock 4 and other BT-Classic pads are out of scope permanently on this chip.
- Investigate integrating Bluepad32 under PlatformIO for the S3 (it typically wants its own board platform packages; check current upstream docs). Two acceptable outcomes:
  1. **Integrated:** gamepad axes/buttons feed the exact same jog layer T20 built (one input abstraction — refactor `joystick.{h,cpp}` into `input_source` if needed). Pairing flow + LED feedback documented in `docs/hardware.md`.
  2. **Documented deferral:** if it requires forking the platform, pinning a fragile fork, or conflicts with ESPAsyncWebServer/WiFi coexistence, **stop**: write up findings + the recommended future path in `docs/build-log.md`, mark this task "deferred (see log)" in `tasks/INDEX.md`, and leave the tree clean. That is a *successful* completion of T21.
- Either way: WiFi + WS UI must keep working alongside BLE (coexistence flags may need tuning — document what you set).

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles (on outcome 2: with zero changes).
- [ ] Outcome 1 **(hardware)**: pair Xbox pad, jog all axes, UI stays responsive with both connected.
- [ ] Outcome 2: build-log entry + INDEX note exist; no half-integrated code committed.

## Out of scope
BT-Classic pads, ESP32 (non-S3) variants.
