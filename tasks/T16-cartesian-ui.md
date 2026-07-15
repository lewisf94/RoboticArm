# T16 — Cartesian control UI

**Milestone:** M4 · **Depends on:** T08, T15 · **Touches:** `web/`, `data/`

## Goal
Drive the gripper in X/Y/Z from the browser. **Hardware gate for M4.**

## Spec
- New "Cartesian" tab/section in the UI:
  - **XY pad**: square canvas, top-down view; current tool position dot (from `state.pose`), drag/tap → `move_ik` at the pad's XY, keeping current Z (throttle 5 Hz; send final position on release). Draw the reachable annulus (`|L1−L2|`…`L1+L2` scaled by profile geometry from `get_profile`) so users see why targets refuse.
  - **Z column**: vertical slider next to the pad, same semantics.
  - Pitch number input, shown only when profile `has_wrist_pitch`.
  - Nudge buttons (±10 mm each axis) → `jog_cart`.
  - Numeric X/Y/Z/pitch readout of live `state.pose`.
- `err unreachable` → toast + brief red flash of the pad target marker (marker snaps back to actual pose).
- Keep joint sliders working (tabs or stacked sections — your call, stay mobile-friendly).
- `sync_web.py`; commit `data/`.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles; UI loads via `python -m http.server` without WS (controls disabled cleanly).
- [ ] **(hardware)** Drag the pad → smooth pursuit; unreachable region visibly refused; Z slider moves straight up/down within servo realism; readout matches a ruler check within ~5 mm mid-workspace.

## Out of scope
3D view (T17).
