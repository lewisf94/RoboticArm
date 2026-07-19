# T24 — ROS 2: serial bridge node

**Milestone:** M8 · **Depends on:** T05 (needs `stream`), T23 (names/units conventions) · **Touches:** `ros2/`

## Goal
The translator that makes the physical arm a ROS 2 citizen: speaks `docs/protocol.md` over USB serial on one side, standard topics/services on the other. **No logic** — limits, easing and safety all stay in `arm_core` on the device; this node only converts message formats and units.

## Spec
- Package `ros2/arm_bridge` (ament_python). Node `bridge`:
  - Params: `port` (default `/dev/ttyACM0`), `baud` (115200).
  - On connect: read `hello`, send `get_profile` (joint names/limits), then `{"cmd":"stream","on":true}`. Reconnect with backoff if the port drops; republish nothing stale while disconnected.
  - Publishes: `/joint_states` (`sensor_msgs/JointState`, **radians**, names from the profile) from each `state` line; `/arm/enabled` (`std_msgs/Bool`).
  - Subscribes: `/arm/target_joint_states` (`sensor_msgs/JointState`, radians; joints omitted from the message → `null` entries in `set_joints`, i.e. "leave alone").
  - Services: `/arm/enable` (`std_srvs/SetBool`), `/arm/estop` (`std_srvs/Trigger`), `/arm/home` (`std_srvs/Trigger`). `estop` is sent immediately, never queued behind pending target writes.
  - Degrees/mm ↔ radians/metres conversion happens **here and only here**.
- Protocol codec in a plain-Python module `arm_bridge/protocol_codec.py` with **no rclpy or serial imports** — parse `state`/`hello`/`ack`/`err` lines, build command lines, unit conversions. All serial I/O behind a small interface so tests never open a port.
- `pytest` unit tests for the codec (run by `colcon test` in the CI `ros2` job): state-line parse, `set_joints` with omitted joints, unit conversion round-trip, malformed-line resilience (garbage in → line skipped, not a crash).

## Acceptance
- [ ] CI `ros2` job green including `colcon test`.
- [ ] **(hardware)** With the bench on USB: `ros2 topic echo /joint_states` shows live angles at 10 Hz; publishing a target moves a servo smoothly; `/arm/estop` goes limp instantly; unplugging/replugging USB recovers without restarting the node.

## Out of scope
Actions/MoveIt interfaces, multi-arm namespacing, parameterizing away from the bench profile.
