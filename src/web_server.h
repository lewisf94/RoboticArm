#pragma once

#include <cstdint>

#include "arm_core/config.h"
#include "arm_core/protocol.h"

// HTTP (LittleFS static files) + WebSocket transport. The second half of
// docs/protocol.md's "one schema, two transports" - the WS side of the same
// Protocol::handle_line() the serial transport already drives. Plain
// functions over module-local state (one radio, one server, nothing an
// instance would buy over a singleton), matching wifi_manager's shape.

namespace web_server {

// Call once from setup(), after `protocol`/`profile` exist. Mounts LittleFS,
// starts the HTTP server and the /ws WebSocket endpoint. Async under the
// hood - returns immediately, never blocks.
void begin(arm::Protocol& protocol, const arm::ArmProfile& profile);

// Call every loop() iteration: broadcasts `state` to every connected WS
// client at 10 Hz (docs/protocol.md - WS always gets telemetry, unlike
// serial's opt-in `stream`). A cheap no-op when no clients are connected.
void poll(uint32_t now_ms);

}  // namespace web_server
