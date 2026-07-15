# T03 — Protocol dispatcher in core

**Milestone:** M1 · **Depends on:** T02 · **Touches:** `lib/arm_core/`, `test/`

## Goal
Transport-agnostic implementation of `docs/protocol.md` (read it first — it is the contract): JSON line in → action on core objects → JSON reply out.

## Spec
`lib/arm_core/include/arm_core/protocol.h` + `src/protocol.cpp` (ArduinoJson v7 allowed here):
- `class Protocol` — constructed with references: `MotionController&`, `const ArmProfile&`, and `struct SystemHooks { bool* enabled; void (*on_enable)(bool, void*); void (*on_estop)(void*); bool (*persist_trim)(uint8_t j, float deg, void*); void* ctx; }` (function pointers so firmware/tests plug in without core knowing about NVS).
- `size_t handle_line(const char* line, char* out, size_t out_cap)` — parses, dispatches, writes exactly one reply JSON line (ack or err per protocol.md envelope, echoing `id` when present). Returns bytes written.
- Commands to implement now: `get_state`, `get_profile`, `enable`, `estop`, `set_joint`, `set_joints`, `jog`, `grip`, `home`, `set_trim`. Motion commands while `!*enabled` → `err disabled` (estop/enable/get_* always work). `grip` maps 0–100 % onto the `is_gripper` joint's range.
- `size_t state_json(char* out, size_t cap, uint32_t t_ms)` — the `state` message (fields per protocol.md; `pose:null` until T13, `seq:null` until T11, `heap` filled by firmware via hook or left 0 natively).
- Fixed buffers (`StaticJsonDocument`-equivalent v7 sizing), no heap in steady state; `out_cap` respected.

## Acceptance
- [ ] New suite `test/test_protocol/`: garbage input → `err bad_json`; unknown cmd → `err unknown_cmd`; `set_joint` happy path acks and moves the MotionController target; out-of-limits → `err out_of_range` and target unchanged; `id` echoed; motion cmd while disabled → `err disabled`; `estop` fires hook + clears enabled; `grip` 0/100 hit gripper joint min/max; `state_json` round-trips through ArduinoJson and contains `j`/`tgt` arrays of length `n_joints`.
- [ ] Any deviation you needed from `docs/protocol.md` is an edit to that doc in this commit.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
Actual transports (T04 serial, T07 WS), poses/sequences commands (T09/T11), IK commands (T15).
