# T25 — ROS 2: bringup + live RViz mirror

**Milestone:** M8 · **Depends on:** T23, T24 · **Touches:** `ros2/`

## Goal
One command = the SBR experience: RViz shows a live digital twin of the physical bench arm, and a demo node proves closed-loop commanding through ROS 2.

## Spec
- Package `ros2/arm_bringup` (ament_python or ament_cmake, launch-only + one script):
  - `launch/bench.launch.py` — `arm_bridge` node + `robot_state_publisher` (loading `arm_description`'s xacro) + rviz2 with saved config. Bridge's `/joint_states` feeds `robot_state_publisher` so RViz mirrors what the device reports (which is the *commanded* angle — hobby servos have no position feedback; note this in the README so nobody expects the model to track a hand-moved servo).
  - `wave_demo` node/script: calls `/arm/enable`, then publishes a slow sine on the shoulder between two safe angles (e.g. 40°–80° equivalent in radians); on Ctrl-C calls `/arm/estop` before exiting.
    - ~~at 1–2 Hz update~~ → **10 Hz** (implemented, parameterized as `rate_hz`). Corrected during T25: the device restarts an eased `MotionController` move on every `set_joints`, so a 1–2 Hz target stream arrives as a visible step-and-hold, contradicting this task's own "sweeps the physical shoulder smoothly" acceptance item. 10 Hz keeps each step ~2° and sits under `arm_bridge`'s 20 Hz coalescing limit.
  - `README.md` quickstart: plug in bench → `ros2 launch arm_bringup bench.launch.py` → run demo.
- Port/params passthrough on the launch file (`port:=/dev/ttyACM1` must work).

## Acceptance
- [ ] CI `ros2` job green (all three packages build; codec tests pass).
- [ ] **(hardware)** Launch: RViz model tracks serial-commanded motion with no visible lag; `wave_demo` sweeps the physical shoulder smoothly and Ctrl-C e-stops; joint slider GUI from T23 still works standalone.

## Out of scope
Gazebo simulation, MoveIt 2 planning, micro-ROS on the ESP32 (all M9 backlog).
