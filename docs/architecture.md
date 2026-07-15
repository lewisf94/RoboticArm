# Architecture

## Goals

1. **First motion fast, no dead ends.** A bench rig (servos clamped to a board) is the first target; the custom printed arm arrives later. Nothing may hardcode a specific arm geometry.
2. **Everything testable without hardware.** The same pattern as the Self-Balancing-Robot repo: a portable, framework-free core that compiles on desktop for unit tests, wrapped by a thin firmware shell.
3. **Small, independent increments.** Implementation is sliced into `tasks/` sized for cheap-model agent sessions; each task leaves `pio test -e native` and `pio run -e esp32s3` green.
4. **Self-contained device.** The ESP32-S3 serves the UI and speaks the protocol; a PC is only needed for flashing and development. ROS 2 integration is an optional later wrapper, not a foundation.

## Layers

```
┌─────────────────────────────────────────────────────────────┐
│  web/            Web UI — vanilla JS, WS client, three.js   │
│                  (served from LittleFS; also runs in        │
│                   browser-only sim mode)                    │
├─────────────────────────────────────────────────────────────┤
│  src/            Firmware shell (Arduino framework)         │
│                  boot, WiFi mgr, HTTP+WS server, serial     │
│                  console, NVS/LittleFS persistence,         │
│                  50 Hz motion tick, pins.h                  │
├─────────────────────────────────────────────────────────────┤
│  lib/arm_hal/    Drivers implementing core interfaces       │
│                  LedcServoOutput (v1) · Pca9685Output,      │
│                  StepperOutput (later)                      │
├─────────────────────────────────────────────────────────────┤
│  lib/arm_core/   PORTABLE C++17 — no Arduino/RTOS includes  │
│                  config: JointConfig, ArmProfile, profiles/ │
│                  motion: JointModel, MotionController       │
│                  kinematics: FK/IK (parameterized geometry) │
│                  sequencer: PoseStore, Sequencer            │
│                  protocol: parse/dispatch/serialize (JSON)  │
│                  interfaces: IJointOutput, IClock, IStore   │
└─────────────────────────────────────────────────────────────┘
                 ▲ unit-tested on host via test/ (Unity)
```

Dependency direction is strictly downward-only in this diagram's terms: `src` may include `arm_hal` and `arm_core`; `arm_hal` may include `arm_core`; `arm_core` includes nothing platform-specific (ArduinoJson v7 is the single allowed dependency — it's a portable header library used by protocol/persistence code and native tests alike).

## Runtime model (firmware)

- **Motion tick — 50 Hz** (matches the 20 ms servo PWM frame). A FreeRTOS timer/task calls `MotionController::tick(dt)`, which advances every joint toward its target under per-joint velocity limits and easing, then writes microsecond pulses through `IJointOutput`. No allocation, no JSON, no blocking in this path.
- **Command path.** WebSocket callbacks (async) and the serial line reader both feed complete JSON lines into a small ring buffer; the main loop drains it, runs the shared protocol dispatcher, and applies results to core objects. Single consumer → no locking around core state.
- **Telemetry.** A 10 Hz task serializes the state message (see `docs/protocol.md`) and broadcasts to WS clients; serial gets state on request (or `stream on`).
- **Persistence.** NVS (`Preferences`): WiFi credentials, per-joint trim offsets, active profile name. LittleFS: `/web/*` UI assets, `/data/poses.json`, `/data/sequences.json`.

## Safety model

- Boot → outputs **detached** and `enabled=false`. First `enable` command attaches at the profile's home/current targets.
- `estop` (protocol command, UI button, and optional physical pin) detaches all PWM immediately; re-arming requires explicit `enable`.
- Every commanded target passes `JointModel::clamp()` (profile soft limits) — in core, on every tick, regardless of source.
- Velocity slew limits in `MotionController` bound speed even if a client streams jumpy targets.
- WS disconnect while jogging: **hold position** (a falling arm is worse than a frozen one; hobby servos can't be read back anyway).

## Key decisions (ADR-style)

| # | Decision | Why |
|---|---|---|
| 1 | ESP32-S3 target, Arduino framework on PlatformIO (`espressif32@^6`) | User's board; Arduino layer has the mature servo/web ecosystem; PlatformIO gives headless builds + native tests for CI/agents |
| 2 | Geometry lives in data (`ArmProfile`), selected at compile time; trims at runtime (NVS) | Custom frame isn't designed yet; bench rig and future arms are just different profiles. Compile-time selection keeps core allocation-free |
| 3 | Degrees + millimetres at every boundary; radians only inside kinematics | Hobby-servo world thinks in degrees; eliminates a whole class of unit bugs in UI/protocol/tasks |
| 4 | Own ~60-line LEDC servo driver instead of a servo library | 8 LEDC channels on the S3, 20 ms/14-bit ≈ 1.2 µs resolution is plenty; keeps HAL a clean implementation of `IJointOutput` |
| 5 | One JSON protocol over both WS and serial, dispatcher in core | Testable on host; serial becomes a free integration-test harness; ROS/PC bridges reuse it |
| 6 | ArduinoJson v7 allowed inside `arm_core` | It's platform-independent, so protocol + pose persistence stay in the testable core |
| 7 | Vanilla-JS web UI, three.js vendored, no npm | Keeps agent sessions and the flash pipeline trivial; UI complexity doesn't warrant a toolchain |
| 8 | IK: closed-form for yaw + planar 2-link (+ optional wrist-pitch keeping tool angle) | Covers EEZYbot-style through 4–5 DOF hobby arms incl. the planned custom frame; no numeric solver on-device (see `docs/kinematics.md`) |
| 9 | Elbow-up solution preferred by default | Matches desktop-arm builds; configurable per profile |
| 10 | Sync moves: common duration = slowest joint, per-joint cubic ease over it | All joints arrive together → predictable straight-ish motion without a trajectory planner |
| 11 | ROS 2 later via micro-ROS node **or** host bridge translating the serial protocol | Core stays framework-free either way; bridge is the cheap first step |
| 12 | Steppers (NEMA 17 + TMC2209) are a later `IJointOutput` implementation + homing story, not a v1 concern | Keeps v1 power/electronics simple (single 5–6 V rail) |

## Testing strategy

- **Host unit tests** (`pio test -e native`, Unity): JointModel clamping/trim, MotionController timing/slew, FK/IK against golden vectors from `docs/kinematics.md`, protocol parse/dispatch round-trips, sequencer state machine.
- **Compile gate**: `pio run -e esp32s3` in CI on every push.
- **Hardware checklists**: each milestone's task files carry **(hardware)** acceptance items the human runs on the bench rig; agents report them "not verified".
- **Sim mode** (M5): browser-only fake device lets UI work be validated without flashing.
