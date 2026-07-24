# arm_description

URDF model of the RoboticArm bench rig (the `bench_3dof` profile — see
[`lib/arm_core/include/arm_core/profiles/bench_3dof.h`](../../lib/arm_core/include/arm_core/profiles/bench_3dof.h)
and [`docs/kinematics.md`](../../docs/kinematics.md), Profile A) plus an RViz
launch file. This is the "drive it in software first" entry point — same
idea as the [Self-Balancing-Robot](https://github.com/lewisf94/Self-Balancing-Robot)
repo's simulation-first workflow, minus Gazebo: pose the arm with sliders
before any hardware is involved.

Pure description package — no logic, no serial, no hardware dependency. The
live bridge to the physical arm arrives in `arm_bridge` (T24).

## Install

Requires ROS 2 Jazzy.

```bash
cd ros2
rosdep install --from-paths src --ignore-src -y   # if using a src/ layout; otherwise install manually:
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch arm_description view.launch.py
```

Two windows open: RViz showing the arm, and a small slider panel
(`joint_state_publisher_gui`) — one slider per joint (`base`, `shoulder`,
`grip`). Move them and watch the model respond in RViz.

## Conventions (must match `docs/kinematics.md`)

- Frame: X forward, Z up, origin at the base yaw axis on the mounting
  surface.
- `base` (yaw): positive = counter-clockwise viewed from above.
- `shoulder` (pitch): 0° = horizontal, positive = up.
- No `elbow` joint exists on this profile — the forearm is rigidly straight
  (fixed joint `elbow_fixed`, angle 0). A future profile with a real elbow
  servo (T22) only needs to flip that one joint to `revolute`.
- `grip`: purely decorative (doesn't affect the tool point), 0° = open, 60°
  = closed.
- Units: URDF is metres/radians throughout; the mm/degree device convention
  never appears here — the xacro file does the conversion once, at the top.

**Self-check:** with every joint at 0, the tool point (`tool_link` origin,
reachable via `ros2 run tf2_ros tf2_echo base_link tool_link`) must sit at
`x=0.260 y=0.000 z=0.060` metres — the "straight out" golden vector from
`docs/kinematics.md`. This is exactly `L1 + L2` (0.120 + 0.140 m) out from
the yaw axis, at `base_h` (0.060 m) above the table.

## Files

```
urdf/bench_arm.urdf.xacro   the model
launch/view.launch.py       robot_state_publisher + joint_state_publisher_gui + rviz2
rviz/arm.rviz                saved RViz layout (robot model + TF, orbit camera)
```
