# T11 — Sequencer state machine in core

**Milestone:** M3 · **Depends on:** T09 · **Touches:** `lib/arm_core/`, `test/`

## Goal
Chained pose playback with dwell times and looping — pure core logic, tick-driven, fully unit-tested.

## Spec
- `lib/arm_core/include/arm_core/sequencer.h` + `src/sequencer.cpp`:
  - `struct SeqStep { char pose[17]; uint32_t dur_ms; uint32_t dwell_ms; }`; `struct Sequence { char name[17]; SeqStep steps[16]; uint8_t n_steps; bool loop; }`; capacity 8 sequences. Same name rules as poses.
  - `class Sequencer` — refs to `PoseStore` + `MotionController`. API: `start(name)`, `stop()` (halts *sequencing*; current move finishes — an in-flight move is short and predictable; `estop` remains the hard stop), `tick()` (drives: idle → moving (issue `move_to` with step dur) → wait `moving()==false` → dwelling → next/loop/done), `status()` → `{name, step, playing}` for the `state` message.
  - `start` on unknown sequence or sequence referencing a missing pose → typed error before any motion (validate all steps up front).
  - Persistence `/data/sequences.json` via `IFileStore`, same defensive-load rules as poses.
- Protocol: `save_seq`, `run_seq`, `stop_seq`, `list_seqs`, `delete_seq`; while playing, manual motion commands (`set_joint(s)`, `jog`, `grip`, `home`, `goto_pose`) → `err busy`; `estop`/`stop_seq` always accepted. Fill `seq` in `state`.

## Acceptance
- [ ] New suite `test/test_sequencer/`: simulated tick loop plays a 3-step sequence in order with correct dwell timing; loop wraps; `stop` mid-dwell and mid-move; missing pose → error, nothing moves; `busy` rejections; persistence round-trip; status fields correct at each phase.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
UI (T12).
