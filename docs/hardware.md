# Hardware

Recommendations and reference for the electronics and the printed frame. The **pin map here and `src/pins.h` must always match** — change both or neither.

## v1 bill of materials (bench rig)

| Qty | Part | Notes |
|---|---|---|
| 1 | ESP32-S3-DevKitC-1 (N8R2 or N16R8) | 8 MB+ flash recommended (web UI lives in LittleFS) |
| 2–3 | MG996R standard servo | owned — main joints |
| 1–2 | SG90 micro servo | owned — gripper / wrist |
| 1 | 5 V or 6 V PSU, **≥ 8 A** (or bench supply) | 6 V gives MG996R noticeably more torque; see power budget |
| 1 | Electrolytic capacitor 2200–4700 µF (≥10 V) | across the servo rail, close to the servos |
| 1 | Screw-terminal / distribution board or thick busbar wiring | servo power must not run through a breadboard |
| — | Plywood/2020 offcut, servo brackets or printed clamps | the bench rig |
| opt | PCA9685 16-ch PWM board | only needed for >8 servos or cleaner wiring |
| opt | 2× analog thumbstick modules (KY-023 style) | M6 physical controls |
| opt | INA219/INA226 current sensor | nice telemetry, not required |

**Rules that prevent the classic failures**
- **Never power servos from the devkit 5 V pin.** USB current limits + servo stall spikes = brownout resets and a possibly dead board.
- **Common ground** between servo PSU and ESP32 is mandatory.
- MG996R stall current is roughly **2.5–3 A each at 6 V**. Budget: `PSU amps ≥ 3 × (MG996R moving simultaneously) + 1`. Symptom of undersizing: ESP32 reboots the moment a servo loads up.
- 3.3 V logic pulses drive hobby servos on a 5–6 V rail fine in practice. If a specific servo is marginal (jitter), the PCA9685 doesn't fix logic level (it also outputs at its own VCC) — a real level shifter or that servo's replacement does.

## ESP32-S3 pin map (bench default)

Authoritative once `src/pins.h` exists (T04). Chosen to dodge every S3 landmine (see "avoid" list).

| Function | GPIO | Notes |
|---|---|---|
| Servo ch 0–3 | 15, 16, 17, 18 | LEDC, 50 Hz |
| Servo ch 4–7 | 21, 47, 39, 40 | 39/40 sacrifice JTAG (fine) |
| I²C SDA / SCL | 8 / 9 | PCA9685, current sensor, OLED |
| Joystick axes (ADC1) | 1, 2, 4, 5 | **ADC1 only** — ADC2 (GPIO 11–20) is unusable while WiFi is on |
| Joystick buttons | 6, 7 | input pull-up |
| E-stop input | 10 | NC switch to GND, input pull-up. **No switch wired? Jumper GPIO 10 → GND**, or the firmware treats the open pin as a triggered e-stop and refuses `enable` (fail-safe by design, T05) |
| Onboard RGB LED | 48 | status (WS2812; some clones wire it to 38) |

**Avoid / reserved on the S3:** 0, 3, 45, 46 (strapping) · 19/20 (USB D−/D+) · 43/44 (UART0) · 26–32 (flash) · **35–37 unavailable on Octal-PSRAM (R8) boards** · ADC2 pins for anything analog while WiFi is on.

The S3 has exactly **8 LEDC PWM channels** → 8 direct-driven servos max. A bigger arm or a PCA9685 (I²C, 16 ch) both fit the same `IJointOutput` interface.

## Power wiring (bench)

```
 6V PSU ──┬── + servo rail ──── all servo red wires
          │        │
          │   [2200–4700 µF]
          │        │
          └── ─ ───┴── servo browns ──┬── ESP32 GND (common!)
                                      │
 USB → ESP32-S3 (logic only)          └── PSU −
 GPIO 15/16/… → servo signal (orange/yellow)
```

## Future: NEMA 17 joints (phase 2, M9 backlog)

Your NEMA 17s become worthwhile for a bigger/stiffer arm or a geared base axis:
- TMC2209 driver modules (step/dir, UART config), 12–24 V supply, per-axis endstop or sensorless homing for the missing absolute position.
- Steppers hold torque continuously (hot) and are open-loop — homing is not optional.
- Plan: `StepperOutput : IJointOutput` in `lib/arm_hal` (FastAccelStepper underneath), plus a homing routine. The core never knows the difference.
Don't mix rails casually: servo 6 V and stepper 24 V stay separate supplies, grounds common.

## Physical controls note (S3 = BLE only)

The S3 has **no Bluetooth Classic**. BLE gamepads (e.g. Xbox Series X|S controller) work via Bluepad32; DualShock 4 / older pads are Classic-only and **won't**. Wired analog thumbsticks are the reliable default (M6); BLE gamepad is the stretch task.

## Custom frame design guidance (Bambu Lab A1)

You're designing the frame in CAD. Constraints that make MG996R-class arms succeed:

**Torque budget (do this before printing).** Worst case = arm horizontal:
`T = Σ mᵢ·g·dᵢ` over everything outboard of the joint (links, servos, payload).
Worked example — shoulder joint, 140 mm upper arm + 140 mm forearm:
- structure 250 g at ~90 mm → 0.25·9.81·0.09 ≈ 0.22 N·m
- forearm servo 55 g at 140 mm → 0.08 N·m
- payload 100 g at 280 mm → 0.27 N·m
- **total ≈ 0.57 N·m ≈ 5.8 kg·cm** vs MG996R stall ≈ 9–11 kg·cm at 6 V.

Keep worst-case static torque **≤ ~50–60 % of stall** or the servo runs hot, chatters, and dies early. Levers that matter: shorter links, payload ≤ ~100–150 g, parallel-link (palletizer) geometry so the shoulder servo sits low, springs/rubber-band counterbalance, gripper mass minimal (SG90).

**Mechanical rules**
- Joint axles ride in **bearings** (625ZZ/608ZZ are cheap); the servo output spline transmits torque only — never radial load.
- Use the metal horns that come with MG996R or printed horns over the spline with an M3 through-bolt; splines strip printed plastic quickly if loose.
- Servo pockets: MG996R body is nominally ≈ 40×20×43 mm with flange mount holes ≈ 49.5 mm apart, SG90 ≈ 23×12×29 mm — **measure yours with calipers**; clones vary by ±0.5 mm.
- Design in servo **trim range**: slotted mounts or adjustable horn positions beat re-printing.

**Printing on the A1 (256³ mm volume — most desktop arms fit whole)**
- PETG or PLA+; PETG for anything near warm servos.
- Load parts: 4+ perimeters, 40–60 % gyroid infill — walls matter more than infill.
- Orient so layer lines run **perpendicular to bending loads** (a link snapped along layers is the #1 printed-arm failure).
- M3 heat-set inserts for anything assembled more than once.

**Geometry the firmware supports out of the box** (see `docs/kinematics.md`): base yaw + shoulder + elbow (+ optional wrist pitch) with configurable link lengths — i.e. a classic 3–5 DOF desktop arm. Design within that parameterization and IK is free; exotic linkages mean new math.

## Bench rig (M1 hardware target)

Two MG996R + one SG90 screwed/clamped to a board, wired per above — enough to exercise every joint/motion/protocol feature and even fake a 2-link arm for IK testing before any frame is printed.
