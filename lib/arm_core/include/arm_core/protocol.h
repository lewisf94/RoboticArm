#pragma once

#include <cstddef>
#include <cstdint>

#include <ArduinoJson.h>

#include "arm_core/config.h"
#include "arm_core/motion.h"

// Transport-agnostic command protocol - docs/protocol.md is the contract,
// this class is its only implementation. One line of JSON in, one line of
// JSON out. Both the WebSocket (T07) and serial (T04) transports frame and
// unframe lines around this; neither has any protocol logic of its own.

namespace arm {

class Protocol {
public:
    // Function-pointer hooks so firmware (NVS, HAL attach/detach) and tests
    // can plug in without arm_core depending on any platform headers.
    // `enabled` is required (never null) - Protocol dereferences it directly.
    // `ctx` and the three callbacks may all be null: Protocol only ever
    // passes ctx through to a callback, never dereferences it itself, so a
    // caller with no per-callback state (e.g. firmware driving a single
    // global MotionController) can leave it null.
    // Field order note: new hooks are appended AFTER ctx so older 5-field
    // aggregate initializers keep compiling (missing trailing members
    // value-initialize to nullptr).
    struct SystemHooks {
        bool* enabled;                                           // shared with the firmware's tick loop
        void (*on_enable)(bool on, void* ctx);                    // attach/detach outputs
        void (*on_estop)(void* ctx);                              // immediate detach
        bool (*persist_trim)(uint8_t j, float deg, void* ctx);    // NVS write; false = storage error
        void* ctx;
        uint32_t (*free_heap)(void* ctx);                         // bytes; null -> state reports heap:0
        bool (*inhibit_enable)(void* ctx);                        // true while a physical e-stop forbids enable
    };

    // motion, profile and hooks.ctx must all outlive the Protocol.
    Protocol(MotionController& motion, const ArmProfile& profile, SystemHooks hooks);

    // Parses one line, dispatches it, writes exactly one reply JSON object
    // (no trailing newline - framing is the transport's job) into out.
    // Returns the number of bytes written (truncated to out_cap if the reply
    // doesn't fit).
    size_t handle_line(const char* line, char* out, size_t out_cap);

    // Writes the `state` telemetry message (docs/protocol.md) for time t_ms.
    size_t state_json(char* out, size_t cap, uint32_t t_ms);

    // True after {"cmd":"stream","on":true}: the serial transport should emit
    // state_json() at 10 Hz. Protocol only stores the flag - pacing and
    // writing are the transport's job. (WS must reject `stream`: T07.)
    bool stream() const { return stream_; }

private:
    void dispatch(const char* cmd, ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void fill_state_fields(ArduinoJson::JsonDocument& out, uint32_t t_ms);

    // Returns true if motion is enabled; otherwise writes an `err disabled`
    // reply and returns false. Call at the top of every handler that moves
    // something.
    bool require_enabled(ArduinoJson::JsonDocument& out);

    void cmd_get_state(ArduinoJson::JsonDocument& out);
    void cmd_get_profile(ArduinoJson::JsonDocument& out);
    void cmd_enable(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_estop(ArduinoJson::JsonDocument& out);
    void cmd_set_joint(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_set_joints(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_jog(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_grip(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_home(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_set_trim(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);
    void cmd_stream(ArduinoJson::JsonDocument& in, ArduinoJson::JsonDocument& out);

    MotionController& motion_;
    const ArmProfile& profile_;
    SystemHooks hooks_;
    bool stream_ = false;
};

}  // namespace arm
