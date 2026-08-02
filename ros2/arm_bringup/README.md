# arm_bringup

One command to bring up the physical bench arm with a live RViz mirror, plus
a demo that drives it through ROS 2.

## Quickstart

1. Wire and power the bench rig ([`docs/hardware.md`](../../docs/hardware.md)).
   **If you have no e-stop switch fitted, jumper GPIO 10 to GND** — the
   firmware treats an open pin as a triggered e-stop and will refuse to
   enable.
2. Plug the ESP32 into USB and check which port it took (`ls /dev/ttyACM*`).
3. Launch:

```bash
ros2 launch arm_bringup bench.launch.py
ros2 launch arm_bringup bench.launch.py port:=/dev/ttyACM1   # different port
ros2 launch arm_bringup bench.launch.py rviz:=false          # headless
```

RViz opens showing the arm; it now mirrors the real device. Move a joint from
another terminal and the model follows:

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /arm/target_joint_states sensor_msgs/msg/JointState \
  "{name: ['shoulder'], position: [1.2]}"
```

4. Or run the demo — enables the arm, sweeps the shoulder 40°–80°, and
   **e-stops on Ctrl-C**:

```bash
ros2 run arm_bringup wave_demo
ros2 run arm_bringup wave_demo --ros-args -p amplitude_deg:=10.0 -p period_s:=4.0
```

## What's running

```
arm_bridge ──/joint_states──> robot_state_publisher ──TF──> RViz
     │                                    ↑
   USB serial                    arm_description URDF
     │
  ESP32 (all limits, easing, e-stop live here)
```

Note there's no `joint_state_publisher_gui` in this launch — the device is
the source of joint angles now, and running both would fight over
`/joint_states`. For the no-hardware slider version, use
`ros2 launch arm_description view.launch.py` instead; the two are meant to be
run one at a time.

## The model shows commanded angles, not measured ones

Hobby servos have no position feedback. The device reports where it has
*told* each servo to be, so:

- Hand-moving a servo will **not** move the model.
- If a servo stalls, is unpowered, or is fighting a mechanical limit, the
  model will happily show it in a position the real arm never reached.

Treat RViz as "what the arm was asked to do", not ground truth. (Closing that
gap needs feedback-capable joints — a stepper with an encoder or a servo with
a position tap; that's M9 backlog territory.)

## Safety notes

- `wave_demo` will not move anything unless `/arm/enable` succeeded — if the
  bridge isn't up, or the device refuses (e-stop pin open), it logs why and
  exits without publishing a single target.
- On Ctrl-C it calls `/arm/estop` before exiting. If it ever reports it
  couldn't, stop the arm by hand:
  `ros2 service call /arm/estop std_srvs/srv/Trigger`
- Demo parameters aren't range-checked here on purpose. The device enforces
  its own joint limits and rejects anything outside them, so a silly
  `amplitude_deg` gets you `out_of_range` errors in the bridge log and an arm
  that doesn't move — not an arm that hurts itself.

## Files

```
launch/bench.launch.py     bridge + robot_state_publisher + rviz2 (port/baud/rviz args)
arm_bringup/wave_demo.py   sine sweep demo with enable-first and e-stop-on-exit
```
