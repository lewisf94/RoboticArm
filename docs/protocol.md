# Control protocol

One JSON message schema over two transports:
- **WebSocket** `ws://<device>/ws` — text frames, one JSON object per frame. Primary UI transport.
- **USB serial** 115200 — NDJSON (one JSON object per `\n`-terminated line). Debug/scripting/bridge transport.

Both feed the same dispatcher in `arm_core/protocol`. Units: **degrees, millimetres, milliseconds**. Protocol version: `proto: 1`.

## Envelope

Request: `{"cmd": "<name>", ...args}` — optional `"id"` (int) is echoed in the reply.
Reply: `{"type":"ack","cmd":"<name>","id":…, ...payload}` or `{"type":"err","cmd":"<name>","id":…,"code":"<code>","msg":"human text"}`.
Device-initiated: `{"type":"hello",…}` on connect, `{"type":"state",…}` telemetry (10 Hz on WS; on serial only after `stream`).

Error codes: `bad_json` · `unknown_cmd` · `bad_args` · `out_of_range` (joint limit) · `unreachable` (IK) · `disabled` (motion while not enabled) · `busy` (sequence running) · `not_found` · `storage` .

## Commands

| cmd | args | notes |
|---|---|---|
| `get_state` | — | one-shot reply, `type:"state"` (not `ack` — see below), `id` echoed if sent |
| `get_profile` | — | see shape below |
| `enable` | `on: bool` | `on:true` attaches outputs at current targets; `false` detaches (always allowed). While the physical e-stop input is open, `on:true` → `err disabled` / msg `estop pin active` (T05) |
| `estop` | — | immediate detach; requires `enable` to re-arm; always acked |
| `set_joint` | `j:int, deg:float, vmax?:float` | single joint target; requires `enable` |
| `set_joints` | `deg:[float…]` (null = leave) | multi-joint, synchronized arrival; `deg` length must equal joint count; requires `enable` |
| `jog` | `j:int, delta:float` | relative nudge of the *target*, clamped (not rejected) at the joint limit; requires `enable` |
| `move_ik` | `x,y,z:float, pitch?:float, dur?:int` | M4+; cartesian target |
| `jog_cart` | `dx,dy,dz:float` | M4+; relative cartesian nudge |
| `grip` | `pct:float` (0=open, 100=closed) | maps to gripper joint range; `pct` is clamped to 0..100; requires `enable` |
| `set_trim` | `j:int, deg:float` | persisted (NVS) offset added at output; applied live even if persistence fails (→ `err storage`); requires `enable` |
| `home` | `dur?:int` | synchronized move to profile home pose; requires `enable` |
| `save_pose` | `name:str` | stores current **targets** |
| `goto_pose` | `name:str, dur?:int` | |
| `list_poses` / `delete_pose` | — / `name` | |
| `save_seq` | `name:str, steps:[{pose:str, dur:int, dwell:int}…], loop:bool` | |
| `run_seq` / `stop_seq` / `list_seqs` / `delete_seq` | `name` / — / — / `name` | |
| `stream` | `on: bool` | serial-only: toggles 10 Hz `state` lines (implemented T05; allowed while disabled — it's read-only telemetry. The WS transport must reject it when it lands in T07) |
| `wifi_set` | `ssid:str, pass:str` (both required; `pass:""` = open network) | serial-only (**the WS transport must reject it — T07, not yet implemented**); persists to NVS, acks, then reboots ~500ms later into STA. `ssid` 1–32 chars, `pass` ≤64 chars, else `bad_args`. Not gated on `enable` — it's network config, not motion (implemented T06) |

## Telemetry / handshake payloads

```jsonc
// on connect (WS) or boot (serial)
{"type":"hello","fw":"0.1.0","proto":1,"profile":"bench_3dof","joints":3}

// 10 Hz, and the reply to get_state (which adds "cmd"/"id" like any other
// reply but keeps type:"state" rather than wrapping in an ack envelope -
// the state shape already says everything needed)
{"type":"state","t":123456,"en":true,
 "j":[45.0,10.2,-30.0],          // current commanded angles (deg, post-easing)
 "tgt":[45.0,20.0,-30.0],        // final targets
 "pose":{"x":180.1,"y":0.0,"z":62.3,"pitch":0.0},   // FK of j; null before M4
 "seq":{"name":"demo","step":2,"playing":true},      // null when idle
 "heap":123456,                                      // free heap bytes on-device; 0 in host tests without a heap hook
 "wifi":{"mode":"sta","ip":"192.168.1.42","rssi":-58}}  // null in host tests without a wifi hook (T06)
 // wifi.mode: "off" (briefly, before first connect attempt) | "connecting" (STA attempt in
 // progress, up to 15s) | "sta" | "ap" (RoboArm-Setup fallback). ip is "0.0.0.0" until an
 // interface actually has one; rssi is 0 outside "sta".

// reply to get_profile
{"type":"ack","cmd":"get_profile",
 "name":"bench_3dof",
 "joints":[{"name":"base","min":-90,"max":90,"home":0,"vmax":120,"gripper":false}, "..."],
 "geo":{"base_h":60,"r_off":0,"L1":120,"L2":140,"Lw":0,"wrist":false},
 "fw":"0.1.0","proto":1}
```

## Examples

```
→ {"cmd":"enable","on":true}
← {"type":"ack","cmd":"enable"}
→ {"cmd":"set_joint","j":0,"deg":45,"id":7}
← {"type":"ack","cmd":"set_joint","id":7}
→ {"cmd":"set_joint","j":0,"deg":300}
← {"type":"err","cmd":"set_joint","code":"out_of_range","msg":"j0 limit is ±90"}
→ {"cmd":"move_ik","x":500,"y":0,"z":50}
← {"type":"err","cmd":"move_ik","code":"unreachable","msg":"d=497 > L1+L2=260"}
```

## Rules

- Any command/field change lands in this doc **in the same commit** as the code.
- Unknown fields are ignored (forward compatibility); unknown `cmd` → `unknown_cmd`.
- The dispatcher is transport-agnostic and lives in `arm_core` (host-testable); transports only frame/unframe lines.
