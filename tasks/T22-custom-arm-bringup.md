# T22 — Custom arm profile + calibration procedure

**Milestone:** M7 · **Depends on:** T14 (+ printed arm existing) · **Touches:** `lib/arm_core/include/arm_core/profiles/`, `docs/`, `src/`

## Goal
The custom A1-printed arm becomes a first-class profile with a repeatable calibration story. **This task is a human+agent pairing** — the human supplies measurements; the agent turns them into config + docs.

## Spec
- Collect from the human (block on these, don't invent): joint list + which servo each axis uses, measured link lengths (`base_h`, `r_off`, `L1`, `L2`, `Lw`) in mm, per-joint safe mechanical range (deg, measured by hand-moving before powering), mount orientation (`dir`, `mount_offset_deg` best guesses), gripper open/close extremes.
- Create `profiles/<name>.h`; profile selection = one `#define ARM_PROFILE_...`/include swap in `src/main.cpp` (add the mechanism now if T01's version didn't; keep it a one-line change).
- Write `docs/calibration.md`: step-by-step first-power-on procedure — joints detached → home targets → attach one joint at a time at low vmax → set trims via UI until physical home matches → verify limits approach slowly → record trims table. Include the torque-budget worksheet from `docs/hardware.md` filled in with the real arm's numbers.
- Sanity gates in code: `validate()` must pass; FK of home pose must be inside the reachable annulus; add the new geometry as a second parameterized case in the kinematics round-trip test.

## Acceptance
- [ ] `pio test -e native` passes (incl. new profile in round-trip tests); `pio run -e esp32s3` compiles for the new profile.
- [ ] **(hardware)** Full calibration procedure executed on the real arm and `docs/calibration.md` corrected where reality disagreed; IK pad drives the physical gripper accurately enough to pick up a known object at a taped floor mark.

## Out of scope
New joint types (steppers — M8 backlog), gripper redesigns.
