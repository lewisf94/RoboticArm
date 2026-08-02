# Bench bring-up guide

Everything waiting on real hardware, in the order to do it. This consolidates
the **(hardware)** acceptance items from tasks T04, T05, T06, T07, T23, T24
and T25 — those task files stay the source of truth; this is the version you
can actually work through at a desk.

Stages 1–3 need only the ESP32 and one servo. Stage 4 additionally needs a
WiFi network (or just a phone to check the AP appears). Stage 5 needs Stage
4 done plus a browser and (optionally) `websocat`. Stage 6 needs no hardware
at all. Budget ~100 minutes for the lot, less if the wiring is already done.

---

## Read this before you power anything

**The first `enable` snaps every servo to its home position at full servo
speed.** There's no soft-start on that first attach — the joints are
`base 0°`, `shoulder 60°`, `grip 30°`, and a servo that's currently somewhere
else will slam there. Keep fingers, cables and anything fragile clear of the
horns the first time you enable, and prefer running the first test with the
horn *removed* or pointing somewhere harmless.

**Three wiring rules that prevent the classic failures:**

1. **Never power servos from the ESP32's 5V pin.** USB current limits plus
   servo stall spikes cause brownout resets and can kill the board. Servos get
   their own supply.
2. **Grounds must be common.** The servo supply's ground and the ESP32's
   ground must be connected, or the signal wire has no reference and the
   servos twitch or ignore you.
3. **Size the supply for stall, not idle.** An MG996R pulls ~2.5–3 A stalled
   at 6 V. Rule of thumb: `amps ≥ 3 × (MG996Rs that can move at once) + 1`.
   The symptom of undersizing is the ESP32 rebooting the moment a servo takes
   load.

**And the one that will otherwise waste your first twenty minutes:** the
firmware treats an open e-stop input as *triggered*, so with nothing wired to
GPIO 10 it refuses to enable and looks broken. **Fit a jumper wire from GPIO
10 to GND before you start**, or wire the real switch (Stage 3).

---

## What you need

- ESP32-S3-DevKitC-1 + USB cable
- 1× servo to start (any of them; an SG90 is the gentlest thing to test with)
- 5–6 V supply, ≥3 A for one MG996R (bench supply is ideal)
- 2200–4700 µF capacitor across the servo rail, near the servos
- Jumper wire (GPIO 10 → GND) or an e-stop switch
- Later stages: the other servos, a browser, optionally
  [websocat](https://github.com/vi/websocat), a machine with ROS 2 Jazzy

---

## Stage 1 — Wire it

Servo channel → GPIO (from `src/pins.h`, matching `docs/hardware.md`):

| Device joint | `j` index | Channel | GPIO | Suggested servo |
|---|---|---|---|---|
| `base` | 0 | 0 | **15** | MG996R |
| `shoulder` | 1 | 1 | **16** | MG996R |
| `grip` | 2 | 2 | **17** | SG90 |

```
 5-6V PSU ──┬── + rail ────────── servo red wires
            │      │
            │  [2200-4700 µF]
            │      │
            └── −  ┴─────── servo brown/black ──┬── ESP32 GND   ← common ground!
                                                │
 USB ──────────────► ESP32-S3 (logic power only)└── PSU −

 GPIO 15/16/17 ─────────────────► servo signal (orange/yellow)
 GPIO 10 ───── jumper ──────────► GND            ← or NC e-stop switch
```

For Stages 2–3 one servo on **GPIO 15** is enough.

---

## Stage 2 — Flash and first motion  *(task T04)*

```bash
pio run -e esp32s3 -t upload
pio device monitor          # 115200; Ctrl-C to exit
```

On boot you should see one line:

```json
{"type":"hello","fw":"0.1.0","proto":1,"profile":"bench_3dof","joints":3}
```

The onboard LED should be **red** — outputs are disabled, which is the
correct state at boot. Now type these into the monitor, one per line
(mind the earlier warning — the arm moves on the first one):

```json
{"cmd":"enable","on":true}
{"cmd":"set_joint","j":0,"deg":45}
{"cmd":"set_joint","j":0,"deg":-45}
{"cmd":"set_joint","j":0,"deg":300}
{"cmd":"estop"}
```

| Check | Expected |
|---|---|
| ☐ `enable` acked, LED turns green | `{"type":"ack","cmd":"enable"}` |
| ☐ `set_joint` 45 then −45 | servo sweeps **smoothly**, not instantly — that's the easing working |
| ☐ out-of-range refused | `{"type":"err",...,"code":"out_of_range","msg":"j0 limit is -90.00..90.00"}` and **no movement** |
| ☐ `estop` | servo goes **limp instantly** (you can turn the horn by hand), LED blinks |
| ☐ after e-stop, `set_joint` | refused with `"code":"disabled"` until you `enable` again |

If `enable` returns `{"code":"disabled","msg":"estop pin active"}` → the GPIO
10 jumper is missing. That's the fail-safe working, not a bug.

---

## Stage 3 — E-stop, trims, telemetry  *(task T05)*

**Telemetry.** Turn on the 10 Hz feed, start a move, watch the numbers change:

```json
{"cmd":"stream","on":true}
{"cmd":"enable","on":true}
{"cmd":"set_joint","j":0,"deg":60}
```

☐ `state` lines arrive ~10 per second, and `j` visibly steps toward 60 across
several lines rather than jumping in one. Turn it off with
`{"cmd":"stream","on":false}` — it's noisy.

**Physical e-stop.** Replace the jumper with a normally-closed switch (or just
pull the jumper mid-move):

```json
{"cmd":"enable","on":true}
{"cmd":"set_joint","j":0,"deg":-80}
```
…and pull the wire while it's moving.

| Check | Expected |
|---|---|
| ☐ pull wire mid-move | outputs die **instantly**, LED blinks blue |
| ☐ `enable` while open | refused: `"estop pin active"` |
| ☐ reconnect wire | LED back to red — still disabled, by design |
| ☐ `enable` after reconnect | works again |

**Trims** (the "my servo horn isn't quite straight" correction). Note trims
need the arm enabled first:

```json
{"cmd":"enable","on":true}
{"cmd":"set_trim","j":0,"deg":5}
```

☐ Servo shifts ~5° without the commanded angle changing. Now **reboot the
board**, `enable` again, and confirm the offset is still applied — that's the
trim surviving in flash.

---

## Stage 4 — WiFi  *(task T06)*

**Fresh flash → AP appears.** After Stage 2/3's flash, before ever sending
`wifi_set`, check your phone's WiFi list:

☐ Network **`RoboArm-Setup`** appears (password `roboarm123`). That's the
device with no stored credentials, serving its fallback AP — correct and
expected on a fresh flash.

**Join your real network.** Back in the serial monitor:

```json
{"cmd":"wifi_set","ssid":"YourNetworkName","pass":"YourPassword"}
```

| Check | Expected |
|---|---|
| ☐ acked | `{"type":"ack","cmd":"wifi_set"}` |
| ☐ device reboots ~500ms later | you'll see the boot `hello` line again |
| ☐ boots into STA | `# wifi: connecting STA to YourNetworkName` then `# wifi: STA connected, ip=...` in the monitor |
| ☐ `ping roboarm.local` from another machine on the same network | replies |
| ☐ servo control unaffected throughout | re-run a `set_joint` from Stage 2 — WiFi connecting must not stall motion or serial |

**Wrong password → falls back to AP after 15s.** Re-run `wifi_set` with a
deliberately wrong password, wait it out:

☐ `# wifi: STA connect timed out, falling back to AP` appears ~15s after
reboot, and `RoboArm-Setup` reappears in your phone's WiFi list.

```json
{"cmd":"get_state"}
```

☐ the `wifi` object's `mode` matches what you'd expect at each stage
(`"connecting"` right after reboot, then `"sta"` or `"ap"`), and `ip` is a
real address once connected.

To get back onto your real network, send `wifi_set` again with the correct
password.

---

## Stage 5 — Web UI placeholder over WiFi  *(task T07)*

Needs Stage 4 done (device joined to your WiFi, `roboarm.local` resolving).

**Browser.** Open `http://roboarm.local` on a machine on the same network:

☐ Page loads "RoboArm — UI arrives in T08", and within ~100ms the `<pre>`
below it fills in with live JSON (`t`, `en`, `j`, `tgt`, `heap`, `wifi`, ...)
ticking at 10 Hz. (This placeholder just proves the pipe — the real sliders
land in T08.)

**WebSocket round-trip.** From a machine with
[websocat](https://github.com/vi/websocat):

```bash
websocat ws://roboarm.local/ws
{"cmd":"get_state"}
```

| Check | Expected |
|---|---|
| ☐ connect | a `hello` line arrives immediately, then `state` lines at ~10 Hz with no `stream` command needed |
| ☐ send `get_state` | one extra `state` reply on top of the broadcast stream |
| ☐ send `{"cmd":"wifi_set","ssid":"x","pass":"y"}` | rejected: `{"type":"err",...,"code":"bad_args","msg":"serial-only command"}` — WS must never be able to trigger a reboot |
| ☐ send `{"cmd":"stream","on":true}` | rejected the same way — WS already streams unconditionally, and this flag is shared with the serial console |

**Multiple clients + disconnect safety.** Open the browser page in two tabs
at once, and get the arm moving (a `set_joint` over serial, or a slow `home`,
works well since it takes a couple of seconds):

| Check | Expected |
|---|---|
| ☐ both tabs update simultaneously | same numbers, same ~10 Hz rate |
| ☐ close one tab mid-move | the other tab keeps streaming and the arm keeps moving to its target — a disconnect must never pause, hold early, or cancel motion |
| ☐ open a 5th concurrent client (e.g. 2 browser tabs + 3 `websocat` sessions) | the oldest connection gets closed to enforce the 4-client cap; the 4 newest keep streaming uninterrupted |

---

## Stage 6 — ROS 2 model, no hardware  *(task T23)*

Do this bit anywhere, even with the arm unplugged and boxed.

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
cd ros2 && colcon build && source install/setup.bash
ros2 launch arm_description view.launch.py
```

| Check | Expected |
|---|---|
| ☐ RViz shows an arm | base pedestal, upper arm, forearm, gripper |
| ☐ sliders move it | one slider each for `base`, `shoulder`, `grip` |
| ☐ `shoulder` positive goes **up** | not down — this is the convention I had to derive by hand |
| ☐ `base` positive rotates **counter-clockwise** seen from above | |
| ☐ zero pose puts the tool tip at (0.260, 0.000, 0.060) m | verify below |

```bash
ros2 run tf2_ros tf2_echo base_link tool_link      # with all sliders at 0
```

---

## Stage 7 — Bridge the real arm into ROS 2  *(task T24)*

Arm plugged in, jumper/switch fitted. Find the port with `ls /dev/ttyACM*`.

```bash
ros2 run arm_bridge bridge --ros-args -p port:=/dev/ttyACM0
# second terminal:
ros2 topic echo /joint_states
```

| Check | Expected |
|---|---|
| ☐ live angles at ~10 Hz | `position` in **radians** (0.785 ≈ 45°), names `base/shoulder/grip` |
| ☐ enable + target moves a servo | commands below |
| ☐ `/arm/estop` → limp instantly | |
| ☐ unplug USB → node keeps running, no stale data | it stops publishing rather than repeating old values |
| ☐ replug → recovers on its own | no restart needed, within a few seconds |

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /arm/target_joint_states sensor_msgs/msg/JointState \
  "{name: ['shoulder'], position: [1.2]}"
ros2 service call /arm/estop std_srvs/srv/Trigger
```

---

## Stage 8 — The whole thing  *(task T25)*

```bash
ros2 launch arm_bringup bench.launch.py            # add port:=/dev/ttyACM1 if needed
```

| Check | Expected |
|---|---|
| ☐ RViz mirrors real motion with no visible lag | send targets as in Stage 5 |
| ☐ `ros2 run arm_bringup wave_demo` sweeps the shoulder smoothly | 40°–80°, no stepping |
| ☐ Ctrl-C on the demo e-stops the arm | goes limp, doesn't just stop publishing |
| ☐ T23's slider GUI still works standalone | run `view.launch.py` **on its own** — not at the same time as this launch, they'd fight over `/joint_states` |

Also worth a quick look here: the browser UI from Stage 5 and this ROS 2
bringup can run at the same time (they're independent WS/serial clients of
the same device) — the 4-client cap counts WS connections only, ROS 2's
serial bridge doesn't compete with it.

Remember: **RViz shows commanded angles, not measured ones.** Hobby servos
have no feedback. If a servo stalls or loses power the model will still show
it exactly where it was told to go, and hand-moving a servo won't move the
model.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `enable` → `"estop pin active"` | GPIO 10 not jumpered to GND / switch open |
| ESP32 reboots when the servo loads up | supply too small, or servos drawing from the 5V pin |
| Servo jitters or ignores commands | no common ground between servo supply and ESP32 |
| Servo buzzes at a hard stop | commanded past its mechanical range — add a trim, or tighten the profile limits |
| Nothing on the serial monitor | wrong port, or wrong baud (115200) |
| `set_trim` → `"disabled"` | trims need `enable` first |
| `wifi_set` acked but device never reconnects | check the credentials — a wrong password looks identical to "still connecting" for the first 15s |
| `RoboArm-Setup` never appears | give it a few seconds after boot; if it never shows even after 15s+, check the serial log for what `wifi_set` actually stored |
| `http://roboarm.local` times out | mDNS can be flaky on some routers/OSes — try the IP from `get_state`'s `wifi.ip`, or the `# wifi: STA connected, ip=...` line on serial, directly |
| Page loads but the `<pre>` stays stuck on "connecting…" | check the serial monitor for `# web: LittleFS mount failed` — usually means `python scripts/sync_web.py && pio run -e esp32s3 -t uploadfs` was never run |
| `colcon build` can't find packages | run it from `ros2/`, and `source install/setup.bash` after |
| `/joint_states` silent | bridge can't open the port — check `ls /dev/ttyACM*`, and that you're in the `dialout` group |

---

## Record what happened

Worth filling in as you go and committing — future-you will want to know
which of these actually got verified, and `docs/build-log.md` is the place for
anything surprising.

| Stage | Date | Result / notes |
|---|---|---|
| 2 — first motion (T04) | | |
| 3 — e-stop, trims, telemetry (T05) | | |
| 4 — WiFi (T06) | | |
| 5 — web UI placeholder (T07) | | |
| 6 — RViz model (T23) | | |
| 7 — bridge (T24) | | |
| 8 — bringup + demo (T25) | | |

When a stage passes, tick its **(hardware)** box in the matching
`tasks/T##-*.md` file too.
