# CLAUDE.md — agent guide for this repo

ESP32-S3 robot arm controller. Portable C++17 core (`lib/arm_core`) + thin Arduino-framework firmware shell (`src/`) + no-build-step web UI (`web/`). Implementation is executed task-by-task from `tasks/`.

## Commands

```bash
pio test -e native            # core unit tests on host — MUST pass before every commit
pio run  -e esp32s3           # firmware compile — MUST succeed before every commit
pio run  -e esp32s3 -t upload      # hardware only — never run in CI/remote sessions
pio check -e native                # optional static analysis
```

ROS 2 packages (`ros2/`) build only where ROS 2 Jazzy exists: the CI `ros2` job (`ros:jazzy` container) or the human's machine (`cd ros2 && colcon build && colcon test`). **Never attempt colcon/apt-ROS in the agent sandbox** — verify ROS-side work via CI, the same way as the esp32s3 build.

There is no hardware attached in remote/CI sessions. Acceptance items marked **(hardware)** in task files are verified later by the human — list them as "not verified (no hardware)" in your final summary instead of skipping silently. `docs/bringup.md` consolidates the outstanding ones into a single ordered bench session; when you add a **(hardware)** item to a task, add it there too.

**Restricted sandboxes (Claude Code on the web):** the SessionStart hook runs `scripts/agent_setup.sh`, which makes `pio test -e native` work even when the sandbox blocks the PlatformIO registry (fallbacks via GitHub/PyPI). If `ARM_PIO_REGISTRY=blocked` is set in your environment, the esp32s3 toolchain cannot be installed there: don't burn time retrying `pio run -e esp32s3` — say "esp32s3 build delegated to CI" in your summary and let GitHub Actions verify it. Native tests are still mandatory locally.

## Task workflow (the only way work gets done here)

1. Open `tasks/INDEX.md`, take the **first unchecked task** (or the task the user names).
2. Read the task file fully, plus any docs it links. **Implement only what the task specs.** If the task file conflicts with reality (API drifted, file moved), follow reality, note the discrepancy in the commit message, and update the task file.
3. Run the acceptance commands from the task file.
4. Tick the task's checkbox in `tasks/INDEX.md` and update the milestone table in `README.md` if a milestone completed.
5. Add a dated entry to `docs/build-log.md` (2–4 lines: what, decisions, surprises).
6. Commit as `T##: <task title>` and push. One task per commit.

Never start a task whose dependencies (listed in its header) are unchecked, and never batch multiple tasks into one commit unless the user asks.

## Architecture rules (non-negotiable)

- `lib/arm_core` must compile with plain g++ — **no** `Arduino.h`, FreeRTOS, ESP-IDF, or driver includes. The only allowed external dependency is **ArduinoJson v7** (it is platform-independent). The `native` test env enforces this — keep it green.
- Hardware access goes through interfaces defined in `arm_core` (e.g. `IJointOutput`) and implemented in `lib/arm_hal`. Firmware (`src/`) wires them together; it contains no logic worth unit-testing.
- **Units at boundaries: degrees and millimetres.** Protocol, UI, config, logs — all degrees/mm. Radians exist only inside kinematics implementation files.
- All limit clamping/validation happens in `arm_core` (`JointModel`, IK reachability). Never trust the UI or protocol input; never bypass the core to write a servo directly.
- GPIO numbers live **only** in `src/pins.h`, matching the table in `docs/hardware.md`. Change both together or neither.
- Web UI is vanilla JS ES modules — **no npm, no bundler, no framework**. Third-party JS (three.js) is vendored as a single minified file in `web/vendor/`. `data/` is generated from `web/` (copy), never hand-edited.
- Protocol messages are defined in `docs/protocol.md`. Any new command = update that doc in the same commit. Both transports (WebSocket, serial) share one dispatcher.
- No new library dependencies unless the task file names them.
- C++: C++17, no exceptions/RTTI in `arm_core` (return `Result`-style structs), no dynamic allocation in the 50 Hz motion path. `snake_case` files/functions, `PascalCase` types, `UPPER_SNAKE` constants.
- `ros2/` is a **translator layer only**: URDF + a bridge speaking `docs/protocol.md` over serial. No motion/kinematics/safety logic there — that lives in `arm_core` on the device. Radians/metres exist only inside `ros2/` (conversion at the bridge/xacro boundary); the device protocol stays degrees/mm. Keep the bridge's protocol codec in a plain-Python module with no rclpy/serial imports so `colcon test` can unit-test it.

## Safety rules (this thing moves)

- Outputs start **disabled**; motion requires an explicit `enable` command after boot.
- `estop` detaches PWM outputs immediately and requires a fresh `enable`. Never remove or weaken this path.
- Soft joint limits from the active `ArmProfile` are enforced in core on every target, every tick.
- On WebSocket disconnect mid-jog: hold position (do not detach, do not drop).

## Where things are

| Concern | Location |
|---|---|
| Design decisions & layer diagram | `docs/architecture.md` |
| Pin map, wiring, power | `docs/hardware.md` |
| FK/IK math + golden test vectors | `docs/kinematics.md` |
| Command schema | `docs/protocol.md` |
| Task queue | `tasks/INDEX.md` (top-to-bottom order = execution order, not milestone numbers) |
| Arm geometry/joint config | `lib/arm_core/include/arm_core/profiles/` (created in T01) |
| ROS 2 packages (URDF, bridge, launch) | `ros2/` (created in T23–T25) |
