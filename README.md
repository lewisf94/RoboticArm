# RoboticArm

Firmware and control stack for a 3D-printed robotic arm, built around an **ESP32-S3**.
Companion project to [Self-Balancing-Robot](https://github.com/lewisf94/Self-Balancing-Robot), reusing the same core idea: **all the interesting logic lives in a portable, framework-free C++ library** that runs identically on the microcontroller and on a desktop (for unit tests and simulation), with a thin firmware shell around it.

## What it will do (v1)

- **Self-contained control** — the ESP32-S3 hosts a WiFi web UI (joint sliders, poses, sequences) plus a line-delimited JSON API over USB serial. No PC required to drive the arm.
- **Cartesian control** — closed-form inverse kinematics for a configurable arm geometry; drive the gripper in X/Y/Z, not just joint-by-joint.
- **Poses & sequences** — teach named poses, chain them into timed pick-and-place sequences, persisted on the device.
- **Live 3D visualizer** — a Three.js digital twin in the web UI, driven by the same kinematics; also runs in a browser-only sim mode with no hardware attached.
- **Physical controls** — analog joystick jog mode (BLE gamepad as a stretch goal).
- **ROS 2, SBR-style** — a `ros2/` layer (URDF + RViz model, serial bridge node, bringup launch) wraps the same protocol, so the arm is drivable from RViz/ROS 2 exactly like the Self-Balancing-Robot workflow. The device never depends on ROS; micro-ROS on-chip stays a backlog idea.

## Hardware at a glance

| Part | Choice | Notes |
|---|---|---|
| Controller | ESP32-S3-DevKitC-1 | WiFi + BLE, native USB, 8 PWM (LEDC) channels |
| Joints (v1) | MG996R standard servos + SG90 gripper | Already owned; bench rig first, custom printed frame later |
| Joints (v2 option) | NEMA 17 steppers + TMC2209 | HAL is designed to accept stepper joints later |
| Frame | Custom CAD, printed on a Bambu Lab A1 | Design guidance in [docs/hardware.md](docs/hardware.md) |
| Servo power | 5–6 V supply, ≥3 A per moving MG996R | Never from the devkit's 5 V pin |

Full BOM, wiring, pin map, power budgeting and frame-design guidance: **[docs/hardware.md](docs/hardware.md)**.

## Status

Planning is complete; implementation is task-driven (see below).

| Milestone | Scope | Status |
|---|---|---|
| M0 | Repo scaffold, CI, planning docs | ✅ done |
| M1 | Core foundations + first servo motion over serial | 🟡 code done — bench check pending |
| M8 | ROS 2: URDF + RViz + serial bridge + bringup | 🟡 code done — bench check pending |
| M2 | WiFi, WebSocket, web UI with joint sliders | 🟡 code done — bench check pending |
| M3 | Poses + sequence record/replay | ⬜ |
| M4 | FK/IK + cartesian control | ⬜ |
| M5 | Three.js 3D visualizer + browser sim mode | ⬜ |
| M6 | Physical controls (joysticks, BLE gamepad stretch) | ⬜ |
| M7 | Custom printed arm bring-up + calibration | ⬜ |
| M9+ | Stepper joints, micro-ROS on-device, MoveIt/Gazebo, vision | backlog |

Details: [docs/roadmap.md](docs/roadmap.md) · running diary: [docs/build-log.md](docs/build-log.md)

## Repo layout

```
lib/arm_core/    Portable C++17 core: config, motion, kinematics, protocol, sequencer
                 (no Arduino/FreeRTOS includes — compiles on desktop; ArduinoJson is
                  the single allowed dependency, it is platform-independent)
lib/arm_hal/     Hardware drivers implementing core interfaces (LEDC servo, later PCA9685/steppers)
src/             Firmware shell: boot, WiFi, web/WS server, serial console, 50 Hz motion tick
web/             Web UI sources (vanilla JS, no build step) → copied to data/ → LittleFS
data/            LittleFS image contents (generated from web/, not hand-edited)
test/            Native unit tests for arm_core (Unity)
ros2/            ROS 2 Jazzy packages (arm_description, arm_bridge, arm_bringup) —
                 a pure translator layer over the serial protocol (created in M8)
docs/            Architecture, hardware, kinematics, protocol, roadmap, build log
tasks/           Agent-executable task specs (T01…) — the implementation queue
```

## Quick start (dev)

```bash
pip install platformio
pio test -e native          # run core unit tests, no hardware needed
pio run  -e esp32s3         # compile firmware
pio run  -e esp32s3 -t upload && pio device monitor   # flash + watch (hardware)

python scripts/sync_web.py && pio run -e esp32s3 -t uploadfs   # (re)flash the web UI (hardware)
```

## Agent-driven development

The heavy design work is done up front (this repo's docs); implementation is broken into small, self-contained tasks in [`tasks/`](tasks/INDEX.md) sized so an inexpensive coding model can execute them one at a time. Workflow, rules and guardrails live in [`CLAUDE.md`](CLAUDE.md).

To run the next task in a Claude Code session:

> Read CLAUDE.md, then implement the next unchecked task from tasks/INDEX.md. Follow the task file exactly, run the acceptance commands, tick the task in INDEX.md, and commit.

## Docs

- **[docs/bringup.md](docs/bringup.md) — bench bring-up: every hardware check, in order. Start here when you have the rig on the desk.**
- [docs/architecture.md](docs/architecture.md) — layers, runtime model, design decisions
- [docs/hardware.md](docs/hardware.md) — BOM, wiring, pin map, power, frame design for the A1
- [docs/kinematics.md](docs/kinematics.md) — geometry parameterization, FK/IK math, test vectors
- [docs/protocol.md](docs/protocol.md) — JSON command protocol (WebSocket + serial)
- [docs/roadmap.md](docs/roadmap.md) — milestones and task map
- [docs/build-log.md](docs/build-log.md) — decision diary

## License

[Apache 2.0](LICENSE)
