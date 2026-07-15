# T02 — MotionController: slew limits + synchronized eased moves

**Milestone:** M1 · **Depends on:** T01 · **Touches:** `lib/arm_core/`, `test/`

## Goal
Time-domain motion: every tick, joints progress smoothly toward targets; multi-joint moves arrive together (docs/kinematics.md "Synchronized multi-joint moves").

## Spec
`lib/arm_core/include/arm_core/motion.h` + `src/motion.cpp`:
- `class MotionController` — constructed over an `ArmProfile`; owns `JointModel` instances internally; no heap allocation after construction.
- `void tick(uint32_t dt_ms)` — advances `current_deg` of each joint. During a trajectory move, position = `start + delta * ease(kCubicInOut, elapsed/T)`; always additionally clamp per-tick change to `vmax_dps * dt` (guard). Idle joints: slew `current` toward `target` at `vmax_dps` (covers retargets mid-move).
- `MoveResult move_to(const float* targets_deg, uint8_t n, uint32_t dur_ms = 0)` — `nullptr`-safe per-entry sentinel `NAN` = leave joint. Validates all targets via `JointModel::clamp` semantics (reject out-of-range with `MoveResult::out_of_range` + joint index, no partial application). `dur_ms==0` → computed `T = max_i(|Δ| / vmax_i)`. Starting a move replaces any in-flight move (restart from live `current`).
- `bool set_joint(uint8_t j, float deg, float vmax_override_dps = 0)` — single-joint convenience over `move_to`.
- Queries: `current(j)`, `target(j)`, `bool moving()`, `float progress()` (0–1, 1 when idle), `JointModel& joint(j)`.
- `struct MoveResult { enum { ok, out_of_range, bad_joint } code; uint8_t joint; }`.

## Acceptance
- [ ] New suite `test/test_motion/`: joints with different deltas/vmax finish the same tick (sync arrival); no overshoot of targets (assert every tick over a simulated move); per-tick guard: |Δcurrent| ≤ vmax·dt + ε for all ticks; `dur_ms=0` duration equals slowest joint's `|Δ|/vmax`; retarget mid-move restarts smoothly from current; out-of-range `move_to` mutates nothing; NAN entries leave joints untouched.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
Servo output (T04), IK (T14), sequencer (T11).
