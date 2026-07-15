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
| `get_state` | — | one-shot state reply |
| `get_profile` | — | joints[] (name, min, max, home, vmax), links (L1…), fw/proto versions |
| `enable` | `on: bool` | `on:true` attaches outputs at current targets; `false` detaches |
| `estop` | — | immediate detach; requires `enable` to re-arm; always acked |
| `set_joint` | `j:int, deg:float, vmax?:float` | single joint target |
| `set_joints` | `deg:[float…]` (null = leave) | multi-joint, synchronized arrival |
| `jog` | `j:int, delta:float` | relative nudge, clamped |
| `move_ik` | `x,y,z:float, pitch?:float, dur?:int` | M4+; cartesian target |
| `jog_cart` | `dx,dy,dz:float` | M4+; relative cartesian nudge |
| `grip` | `pct:float` (0=open, 100=closed) | maps to gripper joint range |
| `set_trim` | `j:int, deg:float` | persisted (NVS) offset added at output |
| `home` | `dur?:int` | synchronized move to profile home pose |
| `save_pose` | `name:str` | stores current **targets** |
| `goto_pose` | `name:str, dur?:int` | |
| `list_poses` / `delete_pose` | — / `name` | |
| `save_seq` | `name:str, steps:[{pose:str, dur:int, dwell:int}…], loop:bool` | |
| `run_seq` / `stop_seq` / `list_seqs` / `delete_seq` | `name` / — / — / `name` | |
| `stream` | `on: bool` | serial-only: toggles 10 Hz state lines |
| `wifi_set` | `ssid,pass` | serial-only (never over WS); reboots into STA |

## Telemetry / handshake payloads

```jsonc
// on connect (WS) or boot (serial)
{"type":"hello","fw":"0.1.0","proto":1,"profile":"bench_3dof","joints":3}

// 10 Hz
{"type":"state","t":123456,"en":true,
 "j":[45.0,10.2,-30.0],          // current commanded angles (deg, post-easing)
 "tgt":[45.0,20.0,-30.0],        // final targets
 "pose":{"x":180.1,"y":0.0,"z":62.3,"pitch":0.0},   // FK of j; null before M4
 "seq":{"name":"demo","step":2,"playing":true},      // null when idle
 "heap":123456}
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
