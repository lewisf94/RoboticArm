# T01 — Core config: JointConfig, ArmProfile, JointModel

**Milestone:** M1 · **Depends on:** — · **Touches:** `lib/arm_core/`, `test/`

## Goal
The data model everything else builds on: joint definitions, arm geometry profiles, and the per-joint logic object with limits/trim/servo mapping.

## Spec
Create `lib/arm_core/include/arm_core/config.h`:
- `struct JointConfig` — `const char* name`; limits `min_deg`, `max_deg`; `home_deg`; `vmax_dps` (max velocity, default 120); output `channel` (uint8); servo calibration: `us_min` (default 500), `us_max` (default 2500) mapped linearly to servo frame −90°…+90°; `dir` (+1/−1, default +1); `mount_offset_deg` (default 0); `is_gripper` (bool, default false).
- `struct ArmGeometry` — `base_h`, `r_off`, `L1`, `L2`, `Lw` (mm, floats), `has_wrist_pitch` (bool). Meanings per `docs/kinematics.md`.
- `struct ArmProfile` — `const char* name`; `uint8_t n_joints`; `JointConfig joints[kMaxJoints]` with `constexpr uint8_t kMaxJoints = 8`; `ArmGeometry geo`.
- `bool validate(const ArmProfile&)` — every joint: `min < max`, `home` within limits, `us_min < us_max`, channels unique.

Create `lib/arm_core/include/arm_core/profiles/bench_3dof.h`:
- `inline const ArmProfile kBench3Dof` — joints: `base` (−90…90, home 0, ch 0), `shoulder` (0…120, home 60, ch 1), `grip` (0…60, home 30, ch 2, `is_gripper`); geometry `base_h=60, r_off=0, L1=120, L2=140, Lw=0, has_wrist_pitch=false` (matches Profile A in `docs/kinematics.md` so kinematics tests reuse it).

Create `lib/arm_core/include/arm_core/joint_model.h` + `src/joint_model.cpp`:
- `class JointModel` — bound to one `JointConfig`. State: `current_deg`, `target_deg` (both init to `home_deg`), `trim_deg` (runtime, init 0).
- `float clamp(float deg) const` → into `[min_deg, max_deg]`.
- `bool set_target(float deg)` → clamps; returns false (and still applies the clamped value? **No** — returns false and applies nothing) when input was out of limits.
- `float output_deg() const` → servo-frame angle: `dir * (current_deg + trim_deg) + mount_offset_deg`.
- `uint16_t output_us() const` → linear map of `output_deg()` from −90…+90 to `us_min…us_max`, clamped to that range.

## Acceptance
- [ ] New suite `test/test_joint/test_main.cpp`: clamp behavior; `set_target` rejects out-of-range without mutating; trim affects `output_us` but not `target_deg`; `dir=-1` mirrors output; `us` mapping endpoints (−90→us_min, +90→us_max, 0→midpoint); `validate(kBench3Dof)` true; validate catches home-outside-limits and duplicate channels.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
Motion over time (T02), any Arduino code, persistence of trims (T05).
