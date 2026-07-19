# T23 — ROS 2: arm_description URDF + RViz

**Milestone:** M8 · **Depends on:** — (model only; no firmware, no hardware) · **Touches:** `ros2/`, `.github/workflows/ci.yml`

## Goal
The SBR-style "drive it in software first" entry point: a URDF model of the bench arm you can pose with sliders in RViz on any ROS 2 Jazzy machine. Also stands up the `ros2/` workspace and its CI job.

## Spec
- New ROS 2 workspace dir `ros2/` (colcon-style: each package a subdir). **Never build it in the agent sandbox** — no ROS there; CI and the human's Jazzy machine do (CLAUDE.md).
- Package `ros2/arm_description` (ament_cmake, installs `urdf/`, `launch/`, `rviz/`):
  - `urdf/bench_arm.urdf.xacro` — parameterized from Profile A / `bench_3dof` constants (`base_h=60`, `r_off=0`, `L1=120`, `L2=140` — source-comment the link to `lib/arm_core/include/arm_core/profiles/bench_3dof.h`). Simple primitives (cylinders/boxes) are fine.
  - Joints: revolute `base` (yaw about Z, ±90°), revolute `shoulder` (0–120°, 0° = horizontal, positive = up), forearm **rigidly straight** (the bench rig has no elbow servo), revolute `grip` jaw (0–60°, purely visual).
  - Frames per `docs/kinematics.md`: X forward, Z up, origin at the yaw axis on the table. **ROS units are metres/radians** — all conversion happens inside the xacro (`${radians(...)}`, `/1000`); device units never leak into ROS.
  - Self-check built into the model: with all joints at 0°, the tool point must sit at **x=0.260 m, z=0.060 m** (the "straight out" golden vector). Verify via RViz/tf before calling it done.
  - `launch/view.launch.py` — robot_state_publisher + joint_state_publisher_gui + rviz2 with a saved `rviz/arm.rviz` config.
  - `README.md` — install/run steps for ROS 2 Jazzy (mirror the SBR repo's tone).
- CI: add a `ros2` job to `.github/workflows/ci.yml`: `container: ros:jazzy-ros-base`, install `liburdfdom-tools` + `ros-jazzy-xacro`, then `cd ros2 && colcon build`, then `xacro` the model to a file and `check_urdf` it. Keep the existing `build` job untouched.

## Acceptance
- [ ] CI `ros2` job green (colcon build + check_urdf); existing native/esp32s3 job untouched and green.
- [ ] **(host with ROS 2)** `ros2 launch arm_description view.launch.py`: sliders move the model; conventions match kinematics.md (yaw CCW from above, shoulder + = up); zero pose puts the tool at (0.26, 0, 0.06) m.

## Out of scope
The serial bridge (T24), Gazebo/MoveIt (M9), modelling the future custom printed arm (that profile doesn't exist yet — T22).
