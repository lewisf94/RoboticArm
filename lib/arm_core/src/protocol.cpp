#include "arm_core/protocol.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "arm_core/units.h"
#include "arm_core/version.h"

using namespace ArduinoJson;

namespace arm {

namespace {

// ArduinoJson v7 dropped StaticJsonDocument's genuinely-fixed buffer in favor
// of JsonDocument+Allocator; this gives back the "fixed size, no unbounded
// heap" behaviour the task asks for. It's a plain malloc/realloc/free
// allocator (correctness delegated to libc, not hand-rolled arena math) with
// a hard byte cap: requests beyond it fail, which ArduinoJson already
// surfaces as DeserializationError::NoMemory / a partially-built document.
// Every JsonDocument in this file is stack-local to one handle_line() or
// state_json() call, so nothing here ever grows across calls.
class BoundedAllocator : public Allocator {
public:
    explicit BoundedAllocator(size_t cap) : cap_(cap) {}

    void* allocate(size_t size) override {
        if (used_ + size > cap_) return nullptr;
        void* raw = std::malloc(size + kHeaderBytes);
        if (!raw) return nullptr;
        *static_cast<size_t*>(raw) = size;
        used_ += size;
        return static_cast<uint8_t*>(raw) + kHeaderBytes;
    }

    void deallocate(void* ptr) override {
        if (!ptr) return;
        void* raw = static_cast<uint8_t*>(ptr) - kHeaderBytes;
        used_ -= *static_cast<size_t*>(raw);
        std::free(raw);
    }

    void* reallocate(void* ptr, size_t new_size) override {
        if (!ptr) return allocate(new_size);
        void* old_raw = static_cast<uint8_t*>(ptr) - kHeaderBytes;
        const size_t old_size = *static_cast<size_t*>(old_raw);
        if (used_ - old_size + new_size > cap_) return nullptr;
        void* new_raw = std::realloc(old_raw, new_size + kHeaderBytes);
        if (!new_raw) return nullptr;
        *static_cast<size_t*>(new_raw) = new_size;
        used_ = used_ - old_size + new_size;
        return static_cast<uint8_t*>(new_raw) + kHeaderBytes;
    }

private:
    static constexpr size_t kHeaderBytes = 16;  // generous vs. any real alignment need
    size_t cap_;
    size_t used_ = 0;
};

// ArduinoJson v7 always allocates its first variant pool chunk at
// ARDUINOJSON_POOL_CAPACITY slots (4096 bytes on a 64-bit host, ~1-2KB on
// the S3) regardless of how little a message actually uses, and shrinks it
// back down after parsing (ARDUINOJSON_AUTO_SHRINK). The budget below must
// clear that first chunk or every document fails before it can hold a
// single field - it does not need to clear the full worst-case message size.
constexpr size_t kJsonPoolBytes = 6144;

void set_err(JsonDocument& out, const char* code, const char* msg) {
    out["type"] = "err";
    out["code"] = code;
    out["msg"] = msg;
}

}  // namespace

Protocol::Protocol(MotionController& motion, const ArmProfile& profile, SystemHooks hooks)
    : motion_(motion), profile_(profile), hooks_(hooks) {}

size_t Protocol::handle_line(const char* line, char* out, size_t out_cap) {
    if (!line) line = "";

    BoundedAllocator in_alloc(kJsonPoolBytes);
    JsonDocument in_doc(&in_alloc);
    const DeserializationError parse_err = deserializeJson(in_doc, line);

    BoundedAllocator out_alloc(kJsonPoolBytes);
    JsonDocument out_doc(&out_alloc);

    const char* cmd = (!parse_err && in_doc["cmd"].is<const char*>()) ? in_doc["cmd"].as<const char*>()
                                                                       : nullptr;
    if (!cmd) {
        out_doc["type"] = "err";
        out_doc["cmd"] = "";
        out_doc["code"] = "bad_json";
        out_doc["msg"] = parse_err ? parse_err.c_str() : "missing 'cmd'";
        return serializeJson(out_doc, out, out_cap);
    }

    out_doc["cmd"] = cmd;
    if (!in_doc["id"].isNull()) out_doc["id"] = in_doc["id"];

    dispatch(cmd, in_doc, out_doc);

    return serializeJson(out_doc, out, out_cap);
}

size_t Protocol::state_json(char* out, size_t cap, uint32_t t_ms) {
    BoundedAllocator alloc(kJsonPoolBytes);
    JsonDocument doc(&alloc);
    doc["type"] = "state";
    fill_state_fields(doc, t_ms);
    return serializeJson(doc, out, cap);
}

void Protocol::dispatch(const char* cmd, JsonDocument& in, JsonDocument& out) {
    if (!std::strcmp(cmd, "get_state")) {
        cmd_get_state(out);
    } else if (!std::strcmp(cmd, "get_profile")) {
        cmd_get_profile(out);
    } else if (!std::strcmp(cmd, "enable")) {
        cmd_enable(in, out);
    } else if (!std::strcmp(cmd, "estop")) {
        cmd_estop(out);
    } else if (!std::strcmp(cmd, "set_joint")) {
        cmd_set_joint(in, out);
    } else if (!std::strcmp(cmd, "set_joints")) {
        cmd_set_joints(in, out);
    } else if (!std::strcmp(cmd, "jog")) {
        cmd_jog(in, out);
    } else if (!std::strcmp(cmd, "grip")) {
        cmd_grip(in, out);
    } else if (!std::strcmp(cmd, "home")) {
        cmd_home(in, out);
    } else if (!std::strcmp(cmd, "set_trim")) {
        cmd_set_trim(in, out);
    } else {
        set_err(out, "unknown_cmd", "no such command");
    }
}

void Protocol::fill_state_fields(JsonDocument& out, uint32_t t_ms) {
    out["t"] = t_ms;
    out["en"] = *hooks_.enabled;

    JsonArray j = out["j"].to<JsonArray>();
    JsonArray tgt = out["tgt"].to<JsonArray>();
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        j.add(motion_.current(i));
        tgt.add(motion_.target(i));
    }

    out["pose"] = nullptr;  // FK arrives in T13
    out["seq"] = nullptr;   // sequencer arrives in T11
    out["heap"] = 0;        // firmware fills this in (or leaves it 0 natively)
}

bool Protocol::require_enabled(JsonDocument& out) {
    if (*hooks_.enabled) return true;
    set_err(out, "disabled", "motion is disabled; send enable first");
    return false;
}

void Protocol::cmd_get_state(JsonDocument& out) {
    out["type"] = "state";
    fill_state_fields(out, 0);
}

void Protocol::cmd_get_profile(JsonDocument& out) {
    out["type"] = "ack";
    out["name"] = profile_.name;

    JsonArray joints = out["joints"].to<JsonArray>();
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        const JointConfig& jc = profile_.joints[i];
        JsonObject o = joints.add<JsonObject>();
        o["name"] = jc.name;
        o["min"] = jc.min_deg;
        o["max"] = jc.max_deg;
        o["home"] = jc.home_deg;
        o["vmax"] = jc.vmax_dps;
        o["gripper"] = jc.is_gripper;
    }

    JsonObject geo = out["geo"].to<JsonObject>();
    geo["base_h"] = profile_.geo.base_h;
    geo["r_off"] = profile_.geo.r_off;
    geo["L1"] = profile_.geo.L1;
    geo["L2"] = profile_.geo.L2;
    geo["Lw"] = profile_.geo.Lw;
    geo["wrist"] = profile_.geo.has_wrist_pitch;

    out["fw"] = ARM_FW_VERSION;
    out["proto"] = ARM_PROTO_VERSION;
}

void Protocol::cmd_enable(JsonDocument& in, JsonDocument& out) {
    if (!in["on"].is<bool>()) {
        set_err(out, "bad_args", "enable needs a boolean 'on'");
        return;
    }
    const bool on = in["on"];
    *hooks_.enabled = on;
    if (hooks_.on_enable) hooks_.on_enable(on, hooks_.ctx);
    out["type"] = "ack";
}

void Protocol::cmd_estop(JsonDocument& out) {
    *hooks_.enabled = false;
    if (hooks_.on_estop) hooks_.on_estop(hooks_.ctx);
    out["type"] = "ack";
}

void Protocol::cmd_set_joint(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    if (!in["j"].is<int>() || !in["deg"].is<float>()) {
        set_err(out, "bad_args", "set_joint needs int 'j' and numeric 'deg'");
        return;
    }
    const int j = in["j"];
    if (j < 0 || j >= profile_.n_joints) {
        set_err(out, "bad_args", "no such joint");
        return;
    }
    const float deg = in["deg"];
    const float vmax = in["vmax"] | 0.0f;

    if (!motion_.set_joint(static_cast<uint8_t>(j), deg, vmax)) {
        char msg[64];
        const JointConfig& jc = profile_.joints[j];
        std::snprintf(msg, sizeof(msg), "j%d limit is %.2f..%.2f", j, jc.min_deg, jc.max_deg);
        set_err(out, "out_of_range", msg);
        return;
    }
    out["type"] = "ack";
}

void Protocol::cmd_set_joints(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    JsonVariantConst deg = in["deg"];
    if (!deg.is<JsonArrayConst>()) {
        set_err(out, "bad_args", "set_joints needs an array 'deg'");
        return;
    }
    JsonArrayConst arr = deg.as<JsonArrayConst>();
    if (arr.size() != profile_.n_joints) {
        set_err(out, "bad_args", "'deg' length must equal joint count");
        return;
    }

    float targets[kMaxJoints];
    uint8_t i = 0;
    for (JsonVariantConst v : arr) {
        targets[i] = v.isNull() ? NAN : v.as<float>();
        ++i;
    }

    const MoveResult r = motion_.move_to(targets, profile_.n_joints);
    if (r.code == MoveResult::out_of_range) {
        char msg[64];
        const JointConfig& jc = profile_.joints[r.joint];
        std::snprintf(msg, sizeof(msg), "j%u limit is %.2f..%.2f", r.joint, jc.min_deg, jc.max_deg);
        set_err(out, "out_of_range", msg);
        return;
    }
    if (r.code != MoveResult::ok) {
        set_err(out, "bad_args", "set_joints failed");
        return;
    }
    out["type"] = "ack";
}

void Protocol::cmd_jog(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    if (!in["j"].is<int>() || !in["delta"].is<float>()) {
        set_err(out, "bad_args", "jog needs int 'j' and numeric 'delta'");
        return;
    }
    const int j = in["j"];
    if (j < 0 || j >= profile_.n_joints) {
        set_err(out, "bad_args", "no such joint");
        return;
    }
    const float delta = in["delta"];

    JointModel& jm = motion_.joint(static_cast<uint8_t>(j));
    const float clamped = jm.clamp(jm.target_deg() + delta);
    motion_.set_joint(static_cast<uint8_t>(j), clamped);  // always in-range now
    out["type"] = "ack";
}

void Protocol::cmd_grip(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    if (!in["pct"].is<float>()) {
        set_err(out, "bad_args", "grip needs numeric 'pct'");
        return;
    }

    int gripper = -1;
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        if (profile_.joints[i].is_gripper) {
            gripper = i;
            break;
        }
    }
    if (gripper < 0) {
        set_err(out, "bad_args", "profile has no gripper joint");
        return;
    }

    const float pct = clampf(in["pct"].as<float>(), 0.0f, 100.0f);
    const JointConfig& jc = profile_.joints[gripper];
    const float deg = jc.min_deg + (pct / 100.0f) * (jc.max_deg - jc.min_deg);

    motion_.set_joint(static_cast<uint8_t>(gripper), deg);  // always in-range by construction
    out["type"] = "ack";
}

void Protocol::cmd_home(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    float targets[kMaxJoints];
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        targets[i] = profile_.joints[i].home_deg;
    }
    const int dur_arg = in["dur"] | 0;
    const uint32_t dur_ms = dur_arg > 0 ? static_cast<uint32_t>(dur_arg) : 0;

    motion_.move_to(targets, profile_.n_joints, dur_ms);  // home is always in-range (validate())
    out["type"] = "ack";
}

void Protocol::cmd_set_trim(JsonDocument& in, JsonDocument& out) {
    if (!require_enabled(out)) return;

    if (!in["j"].is<int>() || !in["deg"].is<float>()) {
        set_err(out, "bad_args", "set_trim needs int 'j' and numeric 'deg'");
        return;
    }
    const int j = in["j"];
    if (j < 0 || j >= profile_.n_joints) {
        set_err(out, "bad_args", "no such joint");
        return;
    }
    const float deg = in["deg"];

    motion_.joint(static_cast<uint8_t>(j)).set_trim_deg(deg);

    bool persisted = true;
    if (hooks_.persist_trim) persisted = hooks_.persist_trim(static_cast<uint8_t>(j), deg, hooks_.ctx);

    if (!persisted) {
        set_err(out, "storage", "trim applied but not saved");
        return;
    }
    out["type"] = "ack";
}

}  // namespace arm
