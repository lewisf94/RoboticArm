# T17 — Three.js digital twin

**Milestone:** M5 · **Depends on:** T08 · **Touches:** `web/`, `data/`

## Goal
A live 3D view of the arm in the web UI, built parametrically from the profile — no hand-modelled meshes.

## Spec
- Vendor three.js: single minified `web/vendor/three.min.js` (a recent r16x release build) **plus** a small vendored `OrbitControls` (module or inline port) in `web/vendor/`. Record exact version + source URLs in `web/vendor/README.md`. This is the only third-party JS allowed in the repo (CLAUDE.md).
- `web/js/kin.js` — tiny FK port (yaw + planar chain, mirrors `docs/kinematics.md` equations, degrees in). Include `kin_selftest()` asserting the doc's golden FK vectors, callable from the console and run automatically in sim/dev mode; log pass/fail.
- `web/js/viz.js` — scene: ground grid (mm scale), base cylinder, links as rounded boxes sized from `get_profile` geometry (`L1`, `L2`, `Lw`, `base_h`), simple gripper indicator that opens with the gripper joint. Pose updates from `state.j` through `kin.js` each frame (interpolate between 10 Hz states for smoothness). Orbit + zoom; soft default lighting; runs at 30 fps+ on a mid phone (pause rendering when tab hidden).
- Layout: visualizer panel above/beside controls, collapsible on small screens.
- `sync_web.py`; commit `data/` (yes, three.min.js goes into flash — with 8 MB+ boards this is fine; note final LittleFS usage in the task summary).

## Acceptance
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles; `kin_selftest()` passes (screenshot/console output in summary).
- [ ] **(hardware)** Twin tracks the bench servos in real time; matches physical geometry direction conventions (yaw + = CCW from above — verify against docs/kinematics.md, not vibes).

## Out of scope
Ghost preview/click-to-move (T18), sim mode (T19).
