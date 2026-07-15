# T14 — Inverse kinematics (closed form)

**Milestone:** M4 · **Depends on:** T13 · **Touches:** `lib/arm_core/`, `test/`

## Goal
Tool position → joint angles, closed form, elbow-up, with honest failure — per `docs/kinematics.md` (the derivation there is the spec).

## Spec
In `kinematics.h/.cpp`:
- `struct IkResult { enum class Code { ok, unreachable, out_of_limits } code; float q_deg[4]; uint8_t n; uint8_t bad_joint; };`
- `IkResult ik(const ArmProfile&, const ToolPose& target)` — steps 1–5 from the doc: yaw, wrist removal (when `has_wrist_pitch`, honor `target.pitch`; otherwise ignore pitch), 2-link solve (elbow-up: `q2 = −acos(…)`), wrist angle, then validate every chain joint against profile limits (`out_of_limits` + `bad_joint`, no clamping).
- Reject `r < 0` and `|cos q2| > 1` as `unreachable`. Use an epsilon (1e-4) so boundary poses (fully stretched) solve.
- No allocation; pure function of inputs.

## Acceptance
- [ ] Extend `test/test_kinematics/`: all IK golden vectors from the doc within 0.05°; unreachable rows return `unreachable`; a target valid geometrically but violating a tightened test profile's limits returns `out_of_limits` with the right `bad_joint`; **round-trip grid**: for q over a lattice (yaw −90…90 step 30, shoulder 10…110 step 20, elbow −140…−20 step 20, within limits) assert `ik(fk(q)) ≈ q` to 0.05°; fully-stretched boundary solves.
- [ ] `pio test -e native` passes; `pio run -e esp32s3` compiles.

## Out of scope
Wiring into protocol/motion (T15).
