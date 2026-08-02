# Task queue

Execution order for agent sessions. Rules live in [`CLAUDE.md`](../CLAUDE.md): take the **first unchecked task**, implement exactly its spec, run its acceptance commands, tick it here, add a build-log entry, commit as `T##: <title>`.

Tasks marked **(hardware)** in their acceptance lists have items only the human can verify — implement, verify what's verifiable, and report the rest as "not verified (no hardware)".

## M1 — Core foundations & first motion
- [x] [T01 — Core config: JointConfig, ArmProfile, JointModel](T01-core-config.md)
- [x] [T02 — MotionController: slew limits + synchronized eased moves](T02-motion-controller.md)
- [x] [T03 — Protocol dispatcher in core](T03-protocol-dispatcher.md)
- [x] [T04 — LEDC servo HAL + firmware shell: first motion over serial](T04-ledc-hal-firmware.md)
- [x] [T05 — Trims in NVS, e-stop input, serial telemetry](T05-trims-estop-telemetry.md)

## M8 — ROS 2 integration (pulled forward — runs right after M1)
SBR-style workflow: pose the model in RViz first (no hardware, no firmware needed), then a live serial bridge to the bench. The ESP32 stays fully self-contained — these packages only *translate* the existing serial protocol; no logic moves into ROS. Numbered M8 because it was promoted from the backlog after the queue was created; **INDEX order (this file, top to bottom) is the execution order**, not milestone numbers.
- [x] [T23 — ROS 2: arm_description URDF + RViz](T23-ros2-description.md)
- [x] [T24 — ROS 2: serial bridge node](T24-ros2-bridge.md)
- [x] [T25 — ROS 2: bringup + live RViz mirror](T25-ros2-bringup.md)

## M2 — WiFi + web UI v1
- [x] [T06 — WiFi manager: STA + AP fallback + mDNS](T06-wifi-manager.md)
- [ ] [T07 — HTTP server, LittleFS assets, WebSocket transport](T07-http-websocket.md)
- [ ] [T08 — Web UI v1: joint sliders, enable/e-stop, trim panel](T08-web-ui-v1.md)

## M3 — Poses & sequences
- [ ] [T09 — PoseStore in core + persistence interface](T09-pose-store.md)
- [ ] [T10 — Poses end-to-end: LittleFS persistence + UI panel](T10-poses-ui.md)
- [ ] [T11 — Sequencer state machine in core](T11-sequencer-core.md)
- [ ] [T12 — Sequences end-to-end: persistence + editor/player UI](T12-sequences-ui.md)

## M4 — Kinematics
- [ ] [T13 — Forward kinematics + pose telemetry](T13-forward-kinematics.md)
- [ ] [T14 — Inverse kinematics (closed form)](T14-inverse-kinematics.md)
- [ ] [T15 — Cartesian motion commands](T15-cartesian-commands.md)
- [ ] [T16 — Cartesian control UI](T16-cartesian-ui.md)

## M5 — 3D visualizer & sim mode
- [ ] [T17 — Three.js digital twin](T17-visualizer.md)
- [ ] [T18 — IK ghost preview + click-to-move](T18-ghost-preview.md)
- [ ] [T19 — Browser sim mode](T19-sim-mode.md)

## M6 — Physical controls
- [ ] [T20 — Analog joystick jog mode](T20-joystick-jog.md)
- [ ] [T21 — BLE gamepad via Bluepad32 (stretch)](T21-ble-gamepad.md)

## M7 — Custom arm bring-up
- [ ] [T22 — Custom arm profile + calibration procedure](T22-custom-arm-bringup.md)
