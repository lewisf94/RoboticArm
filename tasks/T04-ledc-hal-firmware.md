# T04 — LEDC servo HAL + firmware shell: first motion over serial

**Milestone:** M1 · **Depends on:** T03 · **Touches:** `lib/arm_core/` (one interface), `lib/arm_hal/`, `src/`

## Goal
The scaffold `main.cpp` becomes a real firmware: 50 Hz motion tick driving servos through LEDC, USB-serial NDJSON wired to the protocol dispatcher. **Hardware gate for M1.**

## Spec
- `lib/arm_core/include/arm_core/joint_output.h` — `class IJointOutput { virtual bool attach(uint8_t channel) = 0; virtual void detach_all() = 0; virtual void write_us(uint8_t channel, uint16_t us) = 0; virtual ~IJointOutput() = default; };`
- `lib/arm_hal/` (new library, `library.json` like arm_core's): `LedcServoOutput : IJointOutput` — LEDC 50 Hz, 14-bit; channel→GPIO from a table passed in constructor; `write_us` converts µs→duty. ~60 lines; no external servo library.
- `src/pins.h` — GPIO table **exactly** matching `docs/hardware.md` (servo ch 0–7 → 15, 16, 17, 18, 21, 47, 39, 40; e-stop 10; RGB 48).
- Rewrite `src/main.cpp`:
  - Instantiate `kBench3Dof`, `MotionController`, `LedcServoOutput`, `Protocol` (hooks: enable→attach/detach outputs, estop→`detach_all()`; persist_trim stub returns true until T05).
  - **Boot disabled/detached**; print `hello` line (protocol.md).
  - Non-blocking serial line reader (fixed 512-byte buffer, discard oversize with `err bad_json`) → `Protocol::handle_line` → print reply.
  - 50 Hz tick (esp32 `Ticker` or `millis()` scheduler): `MotionController::tick(dt)`, then for each joint `output.write_us(ch, joint.output_us())` — **only while enabled**.

## Acceptance
- [ ] `pio test -e native` passes (unchanged tests); `pio run -e esp32s3` compiles.
- [ ] **(hardware)** With a servo on ch 0: `{"cmd":"enable","on":true}` then `{"cmd":"set_joint","j":0,"deg":45}` moves it smoothly; out-of-range command refuses; `{"cmd":"estop"}` goes limp instantly.

## Out of scope
NVS persistence, e-stop pin, telemetry streaming (all T05); WiFi (T06).
