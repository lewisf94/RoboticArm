# Build log

Running diary — newest first. Agents append a short dated entry per completed task; the human adds hardware notes.

## 2026-07-17 — T03: Protocol dispatcher in core

- Added `Protocol` (`lib/arm_core`): `handle_line()` (parse → dispatch → one reply) and `state_json()`, implementing `get_state`, `get_profile`, `enable`, `estop`, `set_joint`, `set_joints`, `jog`, `grip`, `home`, `set_trim` per `docs/protocol.md`. Firmware/NVS access goes through `SystemHooks` (plain function pointers + `void* ctx`) so the core stays framework-free.
- ArduinoJson v7 dropped `StaticJsonDocument`'s genuinely-fixed buffer (it's now `JsonDocument` + an `Allocator*`, defaulting to heap malloc/realloc/free). Wrote a small `BoundedAllocator` (malloc/realloc/free-backed, hard byte cap, 16-byte size-header per block) to get "fixed size, no unbounded heap" back, rather than hand-rolling a bump/arena allocator — delegating the actual pointer arithmetic to libc removes an entire class of buffer-management bugs for ~30 lines of code.
- Hit and fixed a real bug from this: ArduinoJson always allocates its **first** variant-pool chunk at `ARDUINOJSON_POOL_CAPACITY` slots (4096 bytes on a 64-bit host) regardless of how little a message uses, then shrinks it back down after parsing (`ARDUINOJSON_AUTO_SHRINK`). An initial 2048-byte cap refused that very first allocation, so *every* command silently produced an empty reply, including ones that should've said `bad_json`. Fixed by sizing the budget (6144 bytes) to clear that first chunk rather than the message content. Verified with a small standalone repro outside the Unity/SCons loop before touching the real file — much faster than iterating through the full test harness.
- Also verified from source (not assumed) that assigning a `const char*`/`char[]` into a `JsonDocument` always copies — only a literal `"..."` token gets zero-copy treatment. That's what makes it safe to build error messages in a handler-local `char msg[64]` and hand them to a *different* document (`out`) than the one holding the request; no dangling-pointer risk despite the two documents having separate `BoundedAllocator` instances.
- Deviations from the original protocol.md text, now folded into that doc in this commit: `get_state`'s reply is `type:"state"` directly (not wrapped in an `ack`); `get_profile`'s exact reply shape is now spelled out as JSONC; `set_joints` requires `deg` to be exactly `n_joints` long (extra/missing entries → `bad_args`, not forwarded to a lower-level error); `set_trim` requires `enable` (grouped with the other "motion commands" per the task's own framing, even though it doesn't move anything through `MotionController`).
- 18 new Unity tests in `test/test_protocol/` (reusing `kBench3Dof`, unlike T02's dedicated profile — no need for differing per-joint vmax here). `pio test -e native`: 42/42 passed, no warnings. `pio run -e esp32s3` still blocked in this sandbox — delegated to CI.

## 2026-07-16 — T02: MotionController: slew limits + synchronized eased moves

- Added `MotionController` (`lib/arm_core`): owns a fixed `JointModel[kMaxJoints]` array bound to the profile's joints, `tick()`, `move_to()` (two-pass validate-then-apply, NaN sentinel = leave joint, restarts from live `current` on retarget), `set_joint()`.
- Non-obvious finding worth flagging for future tasks: the existing cubic ease-in-out (from M0) has peak velocity = **3x** its average, so the "clamp per-tick change to vmax·dt" guard in the spec isn't a rare backstop — it binds on whichever joint sets the shared move duration on every synchronized move. `tick()` treats active (eased) and idle (direct-to-target) joints identically: compute a desired position, then clamp the step, which is what makes "no overshoot" hold unconditionally.
- `moving()` is position-based (any joint off-target beyond a 0.01° epsilon), not elapsed-time-based, so it stays true until joints have *physically* caught up even if a move's nominal duration has already elapsed under clamp-induced lag. `progress()` stays elapsed-time-based (simple 0-1 ETA indicator) — the two intentionally answer different questions.
- Proved (and used to design the "sync arrival" test) that joints with *equal* `|Δ|/vmax` ratios stay at an identical normalized position every tick regardless of clamping, so they're guaranteed to arrive on the same tick even with different deltas and different vmax.
- 11 new Unity tests in `test/test_motion/` (dedicated 3-joint profile with differing vmax, since bench_3dof's joints share vmax). `pio test -e native`: 24/24 passed, no warnings. `pio run -e esp32s3` still blocked in this sandbox — delegated to CI.

## 2026-07-16 — T01: Core config: JointConfig, ArmProfile, JointModel

- Added `config.h` (JointConfig/ArmGeometry/ArmProfile + `validate()`), `profiles/bench_3dof.h` (matches kinematics.md Profile A), and `JointModel` (clamp/target/trim/servo-us mapping) to `lib/arm_core`.
- `JointModel` binds to its `JointConfig` via pointer, not reference, so it stays copy-assignable for T02's `MotionController` to hold in a fixed array.
- `kBench3Dof` is plain aggregate-initialized (no designated initializers) to stay portable across the native g++ and xtensa-esp32s3 toolchains without relying on a pre-C++20 GNU extension.
- 8 new Unity tests in `test/test_joint/`, all passing alongside the existing 5 (`pio test -e native`: 13/13). `pio run -e esp32s3` still blocked in this sandbox (`ARM_PIO_REGISTRY=blocked`) — delegated to CI per CLAUDE.md.

## 2026-07-15 — Project planned, repo scaffolded (M0)

- Decisions locked with Lewis: **custom printed frame later, bench rig first** (2× MG996R + SG90 clamped to a board); **ESP32-S3 DevKitC-1**; self-contained **web UI + serial JSON API** with a ROS-ready portable core (same pattern as the Self-Balancing-Robot repo); full v1 feature set: IK, poses/sequences, three.js visualizer, physical controls.
- Owned motors: SG90s, MG996Rs, NEMA 17s. Steppers deferred to M8 (need TMC drivers, 12–24 V rail, homing) so v1 stays on a single 5–6 V rail.
- Architecture, hardware, kinematics and protocol docs written; 22-task implementation queue created for cheap-model agent sessions; PlatformIO scaffold (esp32s3 + native Unity tests) and GitHub Actions CI added.
- Known S3 gotchas baked into the plan: 8 LEDC channels max, ADC2 dead under WiFi, GPIO 35–37 unavailable on R8 boards, BLE-only (no BT Classic gamepads).
