# Roadmap

Milestones are ordered; tasks within a milestone are ordered too (dependencies in each task file). The live queue is [`tasks/INDEX.md`](../tasks/INDEX.md).

## M0 — Foundation ✅
Repo scaffold: PlatformIO envs (esp32s3 + native tests), CI, planning docs, exemplar core module (`easing`, `units`) with passing native tests.

## M1 — Core foundations & first motion (T01–T05)
Portable core takes shape: `ArmProfile`/`JointModel`, `MotionController` (slew + eased sync moves), protocol dispatcher. LEDC servo HAL + firmware shell. **Hardware gate: a bench servo moves via a JSON command over USB serial.**

## M2 — WiFi + web UI v1 (T06–T08)
WiFi manager (STA + AP fallback), HTTP server from LittleFS, WebSocket with 10 Hz telemetry, web UI with joint sliders, enable/e-stop, trim panel. **Gate: drive joints from a phone browser.**

## M3 — Poses & sequences (T09–T12)
Pose store + persistence, synchronized eased moves, sequencer state machine, UI editor/player. **Gate: record and loop a pick-and-place demo.**

## M4 — Kinematics (T13–T16)
FK + closed-form IK for the parameterized geometry (golden vectors in `docs/kinematics.md`), cartesian protocol commands, cartesian pad in UI. **Gate: drive the gripper in straight X/Y/Z from the UI.**

## M5 — 3D visualizer & sim mode (T17–T19)
Three.js digital twin rendered from the live profile + telemetry; IK ghost preview; browser-only sim mode (develop/demo the entire UI with no hardware).

## M6 — Physical controls (T20–T21)
Analog thumbstick jog (joint + cartesian modes). Stretch: BLE gamepad via Bluepad32 (S3 is BLE-only — Xbox-series pads yes, DualShock 4 no).

## M7 — Custom arm bring-up (T22)
When the A1-printed frame exists: new `ArmProfile` (measured link lengths, limits), calibration/trim procedure, torque sanity check vs. `docs/hardware.md` budget.

## M8+ — Backlog (unscheduled)
- NEMA 17 stepper joint (`StepperOutput` + TMC2209 + homing) for a stiffer/bigger arm
- micro-ROS node or host-side ROS 2 bridge speaking the serial protocol
- Straight-line cartesian interpolation; workspace visualization
- Current sensing (INA219) telemetry + stall detection
- Camera + pick automation experiments
