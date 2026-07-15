# T09 — PoseStore in core + persistence interface

**Milestone:** M3 · **Depends on:** T03 · **Touches:** `lib/arm_core/`, `test/`

## Goal
Named poses live in core, serialize to JSON, and load back — persistence backend abstracted so tests run on host.

## Spec
- `lib/arm_core/include/arm_core/file_store.h` — `class IFileStore { virtual bool read(const char* path, char* buf, size_t cap, size_t& len) = 0; virtual bool write(const char* path, const char* buf, size_t len) = 0; virtual ~IFileStore() = default; };` plus an in-memory `MemFileStore` implementation (used by tests; keep it in core, it's trivial).
- `lib/arm_core/include/arm_core/pose_store.h` + `src/pose_store.cpp`:
  - `struct Pose { char name[17]; float deg[kMaxJoints]; }` — capacity 32 poses, fixed array.
  - `class PoseStore` — `save(name, const MotionController&)` (captures **targets**, overwrite-on-same-name), `find(name)`, `remove(name)`, iteration for `list_poses`; name rules: 1–16 chars, `[A-Za-z0-9_-]`, else reject.
  - `bool persist(IFileStore&)` / `bool load(IFileStore&, uint8_t n_joints)` — JSON file `/data/poses.json` `{ "poses": [{"name":…, "deg":[…]}] }` via ArduinoJson; `load` drops entries whose `deg` length mismatches and clamps angles through profile limits at apply-time (never trust stored files — CLAUDE.md rule).
- Protocol: implement `save_pose`, `goto_pose` (`dur?` passthrough to `move_to`), `list_poses`, `delete_pose` per `docs/protocol.md`; `goto_pose` on unknown name → `err not_found`; store-full → `err storage`.

## Acceptance
- [ ] New suite `test/test_poses/`: CRUD; overwrite; bad names rejected; persist→load round-trip via `MemFileStore` equals original; corrupted JSON → `load` returns false, store empty-but-usable; protocol cmds ack/err per doc; `goto_pose` sets MotionController targets.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
LittleFS implementation + UI (T10), sequences (T11).
