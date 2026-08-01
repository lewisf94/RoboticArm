# arm_bridge

Plugs the physical arm into ROS 2: speaks the device's JSON protocol
([`docs/protocol.md`](../../docs/protocol.md)) over USB serial on one side,
standard ROS topics and services on the other.

**Translator only.** Joint limits, easing, synchronized moves and e-stop
latching all live in `arm_core` on the ESP32. Killing, restarting or
disconnecting this node cannot make the arm accept something it otherwise
wouldn't — that's the whole point of keeping the logic on the device.

## Interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| pub | `/joint_states` | `sensor_msgs/JointState` | radians, names from the device profile, ~10 Hz |
| pub | `/arm/enabled` | `std_msgs/Bool` | published on change |
| sub | `/arm/target_joint_states` | `sensor_msgs/JointState` | radians; joints you omit are left alone |
| srv | `/arm/enable` | `std_srvs/SetBool` | |
| srv | `/arm/estop` | `std_srvs/Trigger` | bypasses the target rate limiter |
| srv | `/arm/home` | `std_srvs/Trigger` | |

Parameters: `port` (default `/dev/ttyACM0`), `baud` (default `115200`).

## Run

```bash
ros2 run arm_bridge bridge --ros-args -p port:=/dev/ttyACM0
ros2 topic echo /joint_states
```

Move a single joint (radians, and note you only name the joint you care
about — the rest hold position):

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /arm/target_joint_states sensor_msgs/msg/JointState \
  "{name: ['shoulder'], position: [1.05]}"
ros2 service call /arm/estop std_srvs/srv/Trigger
```

For the full RViz-mirror experience use `arm_bringup` (T25) instead.

## Behaviour worth knowing

- **`/joint_states` reports commanded angles, not measured ones.** Hobby
  servos have no position feedback; the device reports where it has told the
  servo to be. Hand-moving a servo will not move the model.
- **Nothing is published while disconnected.** No last-known values are
  repeated — a model that looks live but is stale is worse than one that
  visibly stops.
- **Targets are coalesced, not dropped.** A publisher faster than 20 Hz would
  flood a 115200 link, so writes are throttled — but the newest target is
  always the one that gets sent, so the arm never stops short of the last
  commanded pose.
- **E-stop clears any target still waiting**, so a throttled command can't be
  pushed at an arm that was just stopped.
- **Reconnects on its own** with backoff (0.5s → 4s) after an unplug; no
  restart needed.

## Layout and testing

```
arm_bridge/protocol_codec.py    pure Python: parse/build protocol lines, unit conversion
arm_bridge/serial_transport.py  pyserial line framing behind a 3-method interface
arm_bridge/bridge.py            the rclpy node
test/test_protocol_codec.py     pytest, runs under `colcon test`
```

`protocol_codec.py` imports neither `rclpy` nor `serial` on purpose: it holds
the parts that are easy to get subtly wrong (unit conversion, the "omitted
joint" rule, malformed-input handling) and can therefore be tested anywhere
pytest runs, with no ROS install and no device attached.

```bash
cd ros2 && colcon test && colcon test-result --verbose
# or, without ROS at all:
cd ros2/arm_bridge && python3 -m pytest test/
```
