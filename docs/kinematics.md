# Kinematics

Math reference for `arm_core` kinematics (implemented in T13/T14). Also the source of the golden test vectors — **unit tests must use the numbers in this file**.

## Conventions

- Right-handed frame, origin at the intersection of the base-yaw axis and the mounting surface. **Z up, X forward**; yaw is about Z, measured from +X, CCW positive (viewed from above).
- Units at every API boundary: **degrees, millimetres**. Radians are an implementation detail inside kinematics source files.
- Joint angles: `q0` yaw · `q1` shoulder (0° = horizontal, + up) · `q2` elbow (0° = forearm in line with upper arm, − folds down in elbow-up pose) · optional `q3` wrist pitch.

## Geometry parameterization (`ArmProfile`)

| Param | Meaning |
|---|---|
| `base_h` | height of shoulder axis above origin (mm) |
| `r_off` | horizontal offset of shoulder axis from yaw axis (mm, usually small/0) |
| `L1` | upper-arm length: shoulder axis → elbow axis |
| `L2` | forearm length: elbow axis → wrist axis (or tool point if no wrist) |
| `Lw` | wrist axis → tool point (0 if no wrist joint) |
| `has_wrist_pitch` | whether `q3` exists; if so IK holds a commanded tool pitch `φ` |

This covers classic 3–5 DOF desktop arms (EEZYbot-style parallel linkages included — their effective geometry reduces to the same 2-link planar chain with a fixed-level tool).

## Forward kinematics

Planar radius/height from the shoulder, then rotate by yaw:

```
r  = r_off + L1·cos q1 + L2·cos(q1+q2) + Lw·cos(q1+q2+q3)
z  = base_h + L1·sin q1 + L2·sin(q1+q2) + Lw·sin(q1+q2+q3)
x  = r·cos q0
y  = r·sin q0
tool pitch φ = q1 + q2 + q3
```

## Inverse kinematics (closed form)

Input: tool point `(x, y, z)` and, when `has_wrist_pitch`, desired tool pitch `φ` (default 0° = level).

1. **Yaw**: `q0 = atan2(y, x)`; planar radius `r = √(x²+y²) − r_off`. (Reject r < 0.)
2. **Remove the wrist link** (if present): wrist target
   `r' = r − Lw·cos φ`, `z' = (z − base_h) − Lw·sin φ`. Without wrist: `r' = r`, `z' = z − base_h`.
3. **2-link planar solve** for `d² = r'² + z'²`:
   ```
   cos q2 = (d² − L1² − L2²) / (2·L1·L2)          # reject |cos q2| > 1 → unreachable
   q2 = −acos(cos q2)                              # elbow-up (default; sign flips for elbow-down)
   q1 = atan2(z', r') − atan2(L2·sin q2, L1 + L2·cos q2)
   ```
4. **Wrist**: `q3 = φ − q1 − q2`.
5. **Validate** every qᵢ against profile joint limits → typed error, never a silent clamp.

Reachability precondition: `|L1 − L2| ≤ d ≤ L1 + L2` (equivalent to step 3's rejection).

Return type is a `Result` struct: `ok(q[])` or `err(unreachable | out_of_limits)` — callers (protocol layer) decide how to report; motion never receives an invalid target.

## Golden test vectors (elbow-up, degrees/mm)

Profile A: `base_h=60, r_off=0, L1=120, L2=140, Lw=0`, no wrist.

| Case | Input (x, y, z) | Expected |
|---|---|---|
| Straight out | (260, 0, 60) | q0=0, q1=0, q2=0 |
| Mid workspace | (180, 0, 60) | q0=0, **q1=+50.98, q2=−92.73** |
| Yaw only | (0, 180, 60) | q0=+90, q1=+50.98, q2=−92.73 |
| Too far | (261, 0, 60) | `unreachable` (d = 261 > L1+L2 = 260) |
| Too close | (10, 0, 60) | `unreachable` (d = 10 < \|L1−L2\| = 20) |

Verification of the mid-workspace row: FK(50.98°, −92.73°) → r = 120·cos 50.98 + 140·cos(−41.75) = 75.5 + 104.5 = 180.0 ✓, z−base_h = 120·sin 50.98 + 140·sin(−41.75) = 93.2 − 93.2 = 0 ✓.

Tests must assert |Δq| ≤ 0.05° against these, plus the **round-trip property** `IK(FK(q)) = q` over a grid of valid poses (this is the strongest test — use it generously).

## Synchronized multi-joint moves

For a target pose with per-joint deltas Δqᵢ and profile max velocities vᵢ:

```
T = max_i(|Δqᵢ| / vᵢ)                 # slowest joint sets the duration (or caller passes T)
qᵢ(t) = qᵢ₀ + Δqᵢ · ease(t/T)         # cubic ease-in-out (arm_core/easing.h)
```

All joints arrive together; the tick loop still applies per-joint slew clamps as a final guard. Straight-line cartesian interpolation (IK per tick along a line) is a possible later refinement (M8) — not v1.
