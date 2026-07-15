# T18 — IK ghost preview + click-to-move

**Milestone:** M5 · **Depends on:** T16, T17 · **Touches:** `web/`, `data/`

## Goal
Point at the 3D scene, see where the arm *would* go, click to send it.

## Spec
- `web/js/kin.js` gains the IK port (same closed form as `docs/kinematics.md`, elbow-up, returns null on unreachable/out-of-limits — port profile limits from `get_profile`). Extend `kin_selftest()` with the IK golden vectors + a small round-trip grid.
- Raycast pointer onto a horizontal plane at a user-adjustable height (small "target Z" slider, default 40 mm):
  - Hover: semi-transparent **ghost arm** posed at the local IK solution; green tint when valid, red flat marker when unreachable. Throttle to animation frames; zero protocol traffic while hovering.
  - Click/tap (when valid + enabled): send `move_ik` to that point.
- Show the firmware's authority: if the device rejects (`err`), toast + ghost flashes red — local IK is a preview, the core's answer wins.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles; extended `kin_selftest()` passes.
- [ ] **(hardware)** Hovered ghost matches where the arm actually lands (within servo slop); unreachable areas show red; disabled state never sends moves.

## Out of scope
Obstacle awareness, trajectory preview lines.
