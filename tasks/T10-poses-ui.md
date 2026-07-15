# T10 — Poses end-to-end: LittleFS persistence + UI panel

**Milestone:** M3 · **Depends on:** T08, T09 · **Touches:** `lib/arm_hal/` or `src/`, `web/`, `data/`

## Goal
Poses usable from the browser and surviving reboot.

## Spec
- `LittleFsStore : IFileStore` (in `lib/arm_hal`); wire into firmware: load poses at boot (after LittleFS mount), `persist` after every mutating pose command (they're rare — write-through is fine).
- Web UI pose panel:
  - "Save pose" → name prompt (validate same rules client-side) → `save_pose`.
  - List (from `list_poses`) with per-pose: **Go** (uses a global duration slider 0.5–5 s), overwrite, delete (confirm).
  - Home button → `home`.
  - Disable pose buttons while `!enabled`; refresh list on every ack of a mutating command.
- `sync_web.py` run; `data/` committed.

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] **(hardware)** Save three poses, reboot, all listed; Go moves synchronized + eased at chosen duration; delete works; pose buttons inert while disabled.

## Out of scope
Sequences (T11/T12).
