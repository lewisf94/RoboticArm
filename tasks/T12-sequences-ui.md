# T12 — Sequences end-to-end: persistence + editor/player UI

**Milestone:** M3 · **Depends on:** T10, T11 · **Touches:** `src/`, `web/`, `data/`

## Goal
Build and run pick-and-place loops from the browser. **Hardware gate for M3.**

## Spec
- Firmware: wire sequencer into boot-load/persist and the 50 Hz-adjacent loop (`Sequencer::tick()` from `loop()`, not the motion ISR path).
- UI sequence panel:
  - Editor: pick pose (dropdown from pose list), duration + dwell inputs, add/remove/reorder steps (up/down buttons fine — no drag-drop dependency), loop checkbox, save-as-name.
  - Player: list sequences; ▶ / ■; live progress from `state.seq` (highlight current step); playing locks manual controls client-side too (server already rejects with `busy`).
- E-stop button must remain fully functional during playback (it bypasses `busy` by design — verify, don't assume).
- `sync_web.py`; commit `data/`.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Record poses → build 3-step looping sequence → runs smoothly for 2+ minutes; stop lands cleanly; e-stop mid-sequence is instant; reboot → sequence still there and runnable.

## Out of scope
Cartesian/IK anything (M4).
