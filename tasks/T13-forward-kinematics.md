# T13 — Forward kinematics + pose telemetry

**Milestone:** M4 · **Depends on:** T03 · **Touches:** `lib/arm_core/`, `test/`, `docs/protocol.md` (pose field goes live)

## Goal
Joint angles → tool position, matching `docs/kinematics.md` exactly (read it fully first — it is the spec; this file only adds plumbing details).

## Spec
- `lib/arm_core/include/arm_core/kinematics.h` + `src/kinematics.cpp`:
  - `struct ToolPose { float x, y, z, pitch; }` (deg/mm).
  - `ToolPose fk(const ArmGeometry&, const float* q_deg, uint8_t n)` — implements the FK equations for yaw + shoulder + elbow (+ wrist pitch when `has_wrist_pitch`). Gripper joints are ignored (not part of the chain). Radians strictly internal to the .cpp.
  - Joint-index convention: chain joints are, in order, the non-gripper joints (`q0`=yaw, `q1`=shoulder, `q2`=elbow, `q3`=wrist if present). Add `uint8_t chain_joints(const ArmProfile&, uint8_t out_idx[4])` helper mapping chain position → profile joint index; it's needed again in T14/T15.
- Protocol/state: `pose` field now returns `fk()` of current angles (was `null`). Update `docs/protocol.md` example.

## Acceptance
- [ ] New suite `test/test_kinematics/`: every FK-checkable golden vector from `docs/kinematics.md` Profile A (straight-out, mid-workspace verification row) within 0.1 mm/0.05°; symmetry: yaw rotates x/y correctly at ±90°; a 4-joint wrist profile case (invent one, add it to the doc's table in this commit).
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
IK (T14), cartesian commands (T15).
