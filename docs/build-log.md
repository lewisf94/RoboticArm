# Build log

Running diary — newest first. Agents append a short dated entry per completed task; the human adds hardware notes.

## 2026-07-16 — T01: Core config: JointConfig, ArmProfile, JointModel

- Added `config.h` (JointConfig/ArmGeometry/ArmProfile + `validate()`), `profiles/bench_3dof.h` (matches kinematics.md Profile A), and `JointModel` (clamp/target/trim/servo-us mapping) to `lib/arm_core`.
- `JointModel` binds to its `JointConfig` via pointer, not reference, so it stays copy-assignable for T02's `MotionController` to hold in a fixed array.
- `kBench3Dof` is plain aggregate-initialized (no designated initializers) to stay portable across the native g++ and xtensa-esp32s3 toolchains without relying on a pre-C++20 GNU extension.
- 8 new Unity tests in `test/test_joint/`, all passing alongside the existing 5 (`pio test -e native`: 13/13). `pio run -e esp32s3` still blocked in this sandbox (`ARM_PIO_REGISTRY=blocked`) — delegated to CI per CLAUDE.md.

## 2026-07-15 — Project planned, repo scaffolded (M0)

- Decisions locked with Lewis: **custom printed frame later, bench rig first** (2× MG996R + SG90 clamped to a board); **ESP32-S3 DevKitC-1**; self-contained **web UI + serial JSON API** with a ROS-ready portable core (same pattern as the Self-Balancing-Robot repo); full v1 feature set: IK, poses/sequences, three.js visualizer, physical controls.
- Owned motors: SG90s, MG996Rs, NEMA 17s. Steppers deferred to M8 (need TMC drivers, 12–24 V rail, homing) so v1 stays on a single 5–6 V rail.
- Architecture, hardware, kinematics and protocol docs written; 22-task implementation queue created for cheap-model agent sessions; PlatformIO scaffold (esp32s3 + native Unity tests) and GitHub Actions CI added.
- Known S3 gotchas baked into the plan: 8 LEDC channels max, ADC2 dead under WiFi, GPIO 35–37 unavailable on R8 boards, BLE-only (no BT Classic gamepads).
