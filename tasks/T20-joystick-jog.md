# T20 — Analog joystick jog mode

**Milestone:** M6 · **Depends on:** T05 (T15 for cartesian mode) · **Touches:** `src/`, `docs/hardware.md`, `docs/protocol.md`

## Goal
Hands-on driving: two thumbstick modules jog the arm without any browser.

## Spec
- Wiring per `docs/hardware.md`: axes on ADC1 GPIO 1/2 (stick A) and 4/5 (stick B), buttons 6/7. Update the doc if reality forces changes (measure → then change both doc and `pins.h`).
- `src/joystick.{h,cpp}`: 100 Hz sampling, calibration (center capture at boot + NVS-stored min/max via long-press both buttons 3 s), deadzone 8 %, cubic expo.
- **Velocity control**: stick deflection → joint velocity (fraction of `vmax_dps`), applied as continuous retargeting through MotionController (never bypass core clamps). Mapping: A-x=base, A-y=shoulder, B-y=elbow, B-x=wrist-or-nothing; A-button toggles grip open/close; B-button cycles mode.
- Modes: `joint` (above) and, if T15 is done, `cart` (sticks = dx/dy/dz via the jog_cart path). Mode + input activity appear in the `state` message (`"input":"sticks:joint"`) — update `docs/protocol.md`.
- Sticks only act while `enabled`; centered sticks generate zero traffic/motion. WS/serial commands and sticks can interleave (last writer wins — it's a jog, not a fight).

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles (build must succeed with no sticks attached — floating ADC must not cause motion thanks to deadzone + boot-center calibration).
- [ ] **(hardware)** Smooth proportional jog on all mapped axes; no drift with sticks released; grip toggle works; calibration survives reboot.

## Out of scope
BLE gamepad (T21).
