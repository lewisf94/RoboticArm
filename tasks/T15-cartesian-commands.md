# T15 — Cartesian motion commands

**Milestone:** M4 · **Depends on:** T14 · **Touches:** `lib/arm_core/protocol`, `test/`, `docs/protocol.md`

## Goal
`move_ik` and `jog_cart` work over both transports.

## Spec
In the protocol dispatcher (core):
- `move_ik {x,y,z,pitch?,dur?}` — `ik()` → on `ok`, synchronized `move_to` of the chain joints (gripper untouched), `dur` passthrough; on failure → `err unreachable` / `err out_of_range` with a helpful `msg` (include the numbers, like the protocol.md example).
- `jog_cart {dx,dy,dz}` — FK of current **targets** (not currents — jogging while moving must not spiral), add deltas, clamp each |d| ≤ 20 mm, run through the same path with a short duration (~150 ms) so held buttons feel continuous.
- Both refuse while disabled (`err disabled`) or sequence playing (`err busy`).

## Acceptance
- [ ] Extend `test/test_protocol/`: `move_ik` golden target lands expected joint targets; unreachable → err and **no target change**; `jog_cart` accumulates correctly across three calls; busy/disabled rejections.
- [ ] `docs/protocol.md` marked accordingly (commands move from "M4+" to implemented).
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Bench: `move_ik` to a golden pose visibly matches the expected joint angles; `jog_cart` nudges smoothly; unreachable targets refuse without motion.

## Out of scope
UI (T16), straight-line interpolation (backlog).
