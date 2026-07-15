# T19 — Browser sim mode

**Milestone:** M5 · **Depends on:** T17 · **Touches:** `web/`, `data/`, `README.md`

## Goal
The entire UI runs with zero hardware: demos, UI development, and agent verification.

## Spec
- `web/js/sim.js` — a fake device behind the same interface `ws.js` exposes (message in → message out), implementing: `hello`, `get_profile` (serve the bench profile geometry — duplicate the constants here, sourced from a comment-linked copy of `profiles/bench_3dof.h`), `get_state` + 10 Hz state timer, `enable`/`estop`, `set_joint(s)`/`jog`/`grip`/`home` with a simple JS slew model (per-joint vmax, 50 Hz internal tick), pose/sequence commands backed by `localStorage`, `move_ik`/`jog_cart` via `kin.js`.
- Activation: `?sim=1` URL param **or** automatically offered via a "No device found — try sim mode?" banner after 3 failed WS connects. Visible "SIM" badge in the header while active.
- No behavioral forks in UI code: components must not know whether `ws.js` is real or sim (single transport interface).
- Document in README (works from `python -m http.server` or even `file://`).

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.
- [ ] No-hardware check (agent-runnable): serve `web/`, open `?sim=1` — sliders move the 3D twin smoothly, poses/sequences record and replay, e-stop freezes, `kin_selftest()` passes. Verify with a headless browser if available (Chromium is preinstalled in remote sessions); otherwise document manual steps in the summary.

## Out of scope
Physics, gravity, servo dynamics beyond slew.
