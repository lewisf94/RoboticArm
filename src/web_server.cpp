#include "web_server.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include <ArduinoJson.h>

#include <cstring>

#include "arm_core/version.h"

using namespace ArduinoJson;

namespace web_server {

namespace {

constexpr uint16_t kHttpPort = 80;
constexpr size_t kMaxClients = 4;
constexpr uint32_t kBroadcastIntervalMs = 100;  // 10 Hz
constexpr size_t kLineBufSize = 512;
constexpr size_t kReplyBufSize = 512;

// Commands docs/protocol.md marks serial-only. The shared dispatcher
// (arm_core::Protocol) is transport-agnostic and has no notion of "which
// transport called me" - enforcing this split is explicitly this
// transport's job (see the comment on Protocol::cmd_wifi_set):
//   wifi_set: never over WS (docs/protocol.md) - it reboots the device into
//             whatever network it's just been told to join, including
//             possibly off of this one.
//   stream:   WS already gets state pushed at 10 Hz unconditionally
//             (poll() below). Protocol::stream_ is one flag shared by both
//             transports; letting a WS client toggle it would silently
//             start or stop a human's serial console telemetry.
constexpr const char* kSerialOnlyCommands[] = {"wifi_set", "stream"};

bool is_serial_only(const char* cmd) {
    for (const char* c : kSerialOnlyCommands) {
        if (!std::strcmp(cmd, c)) return true;
    }
    return false;
}

AsyncWebServer server(kHttpPort);
AsyncWebSocket ws("/ws");

arm::Protocol* g_protocol = nullptr;
const arm::ArmProfile* g_profile = nullptr;

// Connect order, oldest first, for the "cap at kMaxClients, close oldest"
// rule. Pointers only, never touched after a client's own WS_EVT_DISCONNECT
// has fired (removed from this list right there) - client objects are only
// ever assumed live up to and including their own disconnect event, which
// is the sole lifetime guarantee this code depends on.
AsyncWebSocketClient* g_ws_order[kMaxClients];
size_t g_ws_count = 0;

void ws_order_remove(AsyncWebSocketClient* client) {
    for (size_t i = 0; i < g_ws_count; ++i) {
        if (g_ws_order[i] == client) {
            for (size_t k = i; k + 1 < g_ws_count; ++k) g_ws_order[k] = g_ws_order[k + 1];
            --g_ws_count;
            return;
        }
    }
}

void ws_order_add_capping(AsyncWebSocketClient* client) {
    if (g_ws_count >= kMaxClients) {
        AsyncWebSocketClient* oldest = g_ws_order[0];
        ws_order_remove(oldest);
        oldest->close();
    }
    g_ws_order[g_ws_count++] = client;
}

void send_hello(AsyncWebSocketClient* client) {
    JsonDocument doc;
    doc["type"] = "hello";
    doc["fw"] = ARM_FW_VERSION;
    doc["proto"] = ARM_PROTO_VERSION;
    doc["profile"] = g_profile->name;
    doc["joints"] = g_profile->n_joints;
    char out[kReplyBufSize];
    const size_t n = serializeJson(doc, out, sizeof(out));
    client->text(out, n);
}

void send_rejected(AsyncWebSocketClient* client, const char* cmd, JsonVariantConst id) {
    JsonDocument reply;
    reply["type"] = "err";
    reply["cmd"] = cmd;
    if (!id.isNull()) reply["id"] = id;
    reply["code"] = "bad_args";
    reply["msg"] = "serial-only command";
    char out[kReplyBufSize];
    const size_t n = serializeJson(reply, out, sizeof(out));
    client->text(out, n);
}

void handle_text_message(AsyncWebSocketClient* client, const char* line) {
    // Peeked with the default (heap) allocator: this is firmware-only code
    // (not arm_core), off the 50 Hz motion path, and triggered at human/UI
    // pace - not the situation T03's BoundedAllocator was written for.
    JsonDocument peek;
    if (deserializeJson(peek, line) == DeserializationError::Ok) {
        const char* cmd = peek["cmd"] | "";
        if (is_serial_only(cmd)) {
            send_rejected(client, cmd, peek["id"]);
            return;
        }
    }
    char out[kReplyBufSize];
    const size_t n = g_protocol->handle_line(line, out, sizeof(out));
    client->text(out, n);
}

void on_ws_event(AsyncWebSocket* /*server*/, AsyncWebSocketClient* client, AwsEventType type, void* arg,
                  uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            ws_order_add_capping(client);
            send_hello(client);
            break;

        case WS_EVT_DISCONNECT:
            ws_order_remove(client);
            // Safety rule (docs/architecture.md): a client dropping does
            // NOTHING to motion. No enabled/estop/target change, ever.
            break;

        case WS_EVT_DATA: {
            auto* info = static_cast<AwsFrameInfo*>(arg);
            // Only a complete, single-frame text message is a protocol
            // line; fragmented or binary frames are silently ignored rather
            // than guessed at.
            if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) {
                break;
            }
            char line_buf[kLineBufSize];
            if (len >= sizeof(line_buf)) {
                char out[kReplyBufSize];
                const size_t n = g_protocol->handle_line("", out, sizeof(out));  // -> err bad_json
                client->text(out, n);
                break;
            }
            std::memcpy(line_buf, data, len);
            line_buf[len] = '\0';
            handle_text_message(client, line_buf);
            break;
        }

        default:
            break;
    }
}

}  // namespace

void begin(arm::Protocol& protocol, const arm::ArmProfile& profile) {
    g_protocol = &protocol;
    g_profile = &profile;

    if (!LittleFS.begin(/*formatOnFail=*/true)) {
        Serial.println("# web: LittleFS mount failed - static files will 404");
    }

    ws.onEvent(on_ws_event);
    server.addHandler(&ws);

    // Cache headers off for now (T08 will revisit once the UI is real and
    // worth caching): every reflash should show up immediately, not a
    // browser-cached copy of the previous placeholder.
    server.serveStatic("/", LittleFS, "/web/").setDefaultFile("index.html").setCacheControl("no-cache");

    server.begin();
}

void poll(uint32_t now_ms) {
    static uint32_t last = 0;
    if (ws.count() == 0 || now_ms - last < kBroadcastIntervalMs) return;
    last = now_ms;

    char out[kReplyBufSize];
    const size_t n = g_protocol->state_json(out, sizeof(out), now_ms);
    ws.textAll(out, n);
}

}  // namespace web_server
