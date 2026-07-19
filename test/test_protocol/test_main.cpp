// Native unit tests for arm_core Protocol.
// Run: pio test -e native

#include <ArduinoJson.h>
#include <unity.h>

#include "arm_core/motion.h"
#include "arm_core/profiles/bench_3dof.h"
#include "arm_core/protocol.h"

void setUp() {}
void tearDown() {}

namespace {

struct TestHooks {
    bool enabled = false;
    int enable_calls = 0;
    bool last_enable_on = false;
    int estop_calls = 0;
    int persist_calls = 0;
    uint8_t last_trim_j = 0;
    float last_trim_deg = 0.0f;
    bool persist_should_fail = false;
    bool inhibit = false;  // simulates the physical e-stop pin being open
};

void on_enable_cb(bool on, void* ctx) {
    auto* h = static_cast<TestHooks*>(ctx);
    h->enable_calls++;
    h->last_enable_on = on;
}

void on_estop_cb(void* ctx) {
    static_cast<TestHooks*>(ctx)->estop_calls++;
}

bool persist_trim_cb(uint8_t j, float deg, void* ctx) {
    auto* h = static_cast<TestHooks*>(ctx);
    h->persist_calls++;
    h->last_trim_j = j;
    h->last_trim_deg = deg;
    return !h->persist_should_fail;
}

constexpr uint32_t kFakeHeapBytes = 42424;

uint32_t free_heap_cb(void*) { return kFakeHeapBytes; }

bool inhibit_enable_cb(void* ctx) { return static_cast<TestHooks*>(ctx)->inhibit; }

struct Fixture {
    arm::MotionController motion;
    TestHooks hooks;
    arm::Protocol proto;

    explicit Fixture(const arm::ArmProfile& profile)
        : motion(profile),
          proto(motion, profile,
                arm::Protocol::SystemHooks{&hooks.enabled, on_enable_cb, on_estop_cb, persist_trim_cb,
                                            &hooks, free_heap_cb, inhibit_enable_cb}) {}
};

// Calls handle_line and parses the reply with the library's own (default,
// heap-backed) allocator - fine here, this is test-only host code, not the
// bounded-allocator production path inside Protocol itself.
ArduinoJson::JsonDocument call(arm::Protocol& proto, const char* line) {
    char out[512] = {0};
    proto.handle_line(line, out, sizeof(out));
    ArduinoJson::JsonDocument doc;
    ArduinoJson::deserializeJson(doc, out);
    return doc;
}

}  // namespace

static void test_bad_json_returns_err() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, "not json {");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bad_json", doc["code"].as<const char*>());
}

static void test_missing_cmd_field_is_bad_json() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, R"({"foo":1})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bad_json", doc["code"].as<const char*>());
}

static void test_unknown_cmd_returns_err() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, R"({"cmd":"nonexistent_thing"})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("unknown_cmd", doc["code"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("nonexistent_thing", doc["cmd"].as<const char*>());
}

static void test_id_echoed() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, R"({"cmd":"get_profile","id":42})");
    TEST_ASSERT_EQUAL_INT(42, doc["id"].as<int>());
}

static void test_motion_cmd_while_disabled_is_rejected() {
    Fixture f(arm::kBench3Dof);
    TEST_ASSERT_FALSE(f.hooks.enabled);

    const auto doc = call(f.proto, R"({"cmd":"set_joint","j":0,"deg":10})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("disabled", doc["code"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, f.motion.target(0));  // base home is 0, unchanged
}

static void test_set_joint_happy_path_moves_target() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    const auto doc = call(f.proto, R"({"cmd":"set_joint","j":0,"deg":45,"id":7})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("set_joint", doc["cmd"].as<const char*>());
    TEST_ASSERT_EQUAL_INT(7, doc["id"].as<int>());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.0f, f.motion.target(0));
}

static void test_set_joint_out_of_range_leaves_target() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;
    const float before = f.motion.target(0);

    const auto doc = call(f.proto, R"({"cmd":"set_joint","j":0,"deg":500})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("out_of_range", doc["code"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before, f.motion.target(0));
}

static void test_set_joint_missing_field_is_bad_args() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;
    const auto doc = call(f.proto, R"({"cmd":"set_joint","j":0})");  // missing deg
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bad_args", doc["code"].as<const char*>());
}

static void test_estop_fires_hook_and_disables() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    const auto doc = call(f.proto, R"({"cmd":"estop"})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_INT(1, f.hooks.estop_calls);
    TEST_ASSERT_FALSE(f.hooks.enabled);
}

static void test_enable_sets_flag_and_calls_hook() {
    Fixture f(arm::kBench3Dof);

    const auto doc = call(f.proto, R"({"cmd":"enable","on":true})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_TRUE(f.hooks.enabled);
    TEST_ASSERT_EQUAL_INT(1, f.hooks.enable_calls);
    TEST_ASSERT_TRUE(f.hooks.last_enable_on);

    call(f.proto, R"({"cmd":"enable","on":false})");
    TEST_ASSERT_FALSE(f.hooks.enabled);
    TEST_ASSERT_EQUAL_INT(2, f.hooks.enable_calls);
}

static void test_grip_0_and_100_hit_gripper_limits() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    uint8_t grip_idx = 0;
    for (uint8_t i = 0; i < arm::kBench3Dof.n_joints; ++i) {
        if (arm::kBench3Dof.joints[i].is_gripper) grip_idx = i;
    }
    const arm::JointConfig& jc = arm::kBench3Dof.joints[grip_idx];

    call(f.proto, R"({"cmd":"grip","pct":0})");
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, jc.min_deg, f.motion.target(grip_idx));

    call(f.proto, R"({"cmd":"grip","pct":100})");
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, jc.max_deg, f.motion.target(grip_idx));
}

static void test_jog_clamps_at_limit() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    const auto doc = call(f.proto, R"({"cmd":"jog","j":0,"delta":500})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, f.motion.target(0));  // base max is 90; clamped not rejected
}

static void test_set_joints_null_entry_leaves_joint() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    const auto doc = call(f.proto, R"({"cmd":"set_joints","deg":[20,null,10]})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, f.motion.target(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f, f.motion.target(1));  // shoulder home=60, untouched
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, f.motion.target(2));
}

static void test_home_moves_to_home_pose() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;
    call(f.proto, R"({"cmd":"set_joint","j":1,"deg":100})");  // move shoulder away from home

    const auto doc = call(f.proto, R"({"cmd":"home"})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f, f.motion.target(1));  // shoulder home
}

static void test_set_trim_applies_live_and_persists() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;

    const auto doc = call(f.proto, R"({"cmd":"set_trim","j":1,"deg":3.5})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, f.motion.joint(1).trim_deg());
    TEST_ASSERT_EQUAL_INT(1, f.hooks.persist_calls);
    TEST_ASSERT_EQUAL_UINT8(1, f.hooks.last_trim_j);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, f.hooks.last_trim_deg);
}

static void test_set_trim_storage_failure_still_applies_live() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;
    f.hooks.persist_should_fail = true;

    const auto doc = call(f.proto, R"({"cmd":"set_trim","j":0,"deg":2.0})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("storage", doc["code"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, f.motion.joint(0).trim_deg());  // applied despite the error
}

static void test_get_profile_shape() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, R"({"cmd":"get_profile"})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bench_3dof", doc["name"].as<const char*>());

    const auto joints = doc["joints"].as<ArduinoJson::JsonArrayConst>();
    TEST_ASSERT_EQUAL_INT(3, joints.size());
    TEST_ASSERT_EQUAL_STRING("base", joints[0]["name"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -90.0f, joints[0]["min"].as<float>());
    TEST_ASSERT_TRUE(joints[2]["gripper"].as<bool>());
}

static void test_stream_toggles_flag_even_while_disabled() {
    Fixture f(arm::kBench3Dof);
    TEST_ASSERT_FALSE(f.hooks.enabled);  // stream is telemetry: no enable needed
    TEST_ASSERT_FALSE(f.proto.stream());

    auto doc = call(f.proto, R"({"cmd":"stream","on":true})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_TRUE(f.proto.stream());

    doc = call(f.proto, R"({"cmd":"stream","on":false})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FALSE(f.proto.stream());

    doc = call(f.proto, R"({"cmd":"stream"})");  // missing 'on'
    TEST_ASSERT_EQUAL_STRING("bad_args", doc["code"].as<const char*>());
}

static void test_heap_hook_value_appears_in_state() {
    Fixture f(arm::kBench3Dof);
    const auto doc = call(f.proto, R"({"cmd":"get_state"})");
    TEST_ASSERT_EQUAL_UINT32(kFakeHeapBytes, doc["heap"].as<uint32_t>());
}

static void test_inhibit_enable_blocks_enable_on() {
    Fixture f(arm::kBench3Dof);
    f.hooks.inhibit = true;

    const auto doc = call(f.proto, R"({"cmd":"enable","on":true})");
    TEST_ASSERT_EQUAL_STRING("err", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("disabled", doc["code"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("estop pin active", doc["msg"].as<const char*>());
    TEST_ASSERT_FALSE(f.hooks.enabled);
    TEST_ASSERT_EQUAL_INT(0, f.hooks.enable_calls);  // hook never fired
}

static void test_inhibit_still_allows_enable_off() {
    Fixture f(arm::kBench3Dof);
    f.hooks.enabled = true;
    f.hooks.inhibit = true;  // pin opens while running

    const auto doc = call(f.proto, R"({"cmd":"enable","on":false})");
    TEST_ASSERT_EQUAL_STRING("ack", doc["type"].as<const char*>());
    TEST_ASSERT_FALSE(f.hooks.enabled);
}

static void test_state_json_has_correct_array_lengths() {
    Fixture f(arm::kBench3Dof);
    char out[512];
    const size_t n = f.proto.state_json(out, sizeof(out), 12345);
    TEST_ASSERT_TRUE(n > 0);

    ArduinoJson::JsonDocument doc;
    ArduinoJson::deserializeJson(doc, out);
    TEST_ASSERT_EQUAL_STRING("state", doc["type"].as<const char*>());
    TEST_ASSERT_EQUAL_UINT32(12345, doc["t"].as<uint32_t>());
    TEST_ASSERT_EQUAL_INT(arm::kBench3Dof.n_joints, doc["j"].as<ArduinoJson::JsonArrayConst>().size());
    TEST_ASSERT_EQUAL_INT(arm::kBench3Dof.n_joints, doc["tgt"].as<ArduinoJson::JsonArrayConst>().size());
    TEST_ASSERT_TRUE(doc["pose"].isNull());
    TEST_ASSERT_TRUE(doc["seq"].isNull());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_bad_json_returns_err);
    RUN_TEST(test_missing_cmd_field_is_bad_json);
    RUN_TEST(test_unknown_cmd_returns_err);
    RUN_TEST(test_id_echoed);
    RUN_TEST(test_motion_cmd_while_disabled_is_rejected);
    RUN_TEST(test_set_joint_happy_path_moves_target);
    RUN_TEST(test_set_joint_out_of_range_leaves_target);
    RUN_TEST(test_set_joint_missing_field_is_bad_args);
    RUN_TEST(test_estop_fires_hook_and_disables);
    RUN_TEST(test_enable_sets_flag_and_calls_hook);
    RUN_TEST(test_grip_0_and_100_hit_gripper_limits);
    RUN_TEST(test_jog_clamps_at_limit);
    RUN_TEST(test_set_joints_null_entry_leaves_joint);
    RUN_TEST(test_home_moves_to_home_pose);
    RUN_TEST(test_set_trim_applies_live_and_persists);
    RUN_TEST(test_set_trim_storage_failure_still_applies_live);
    RUN_TEST(test_get_profile_shape);
    RUN_TEST(test_stream_toggles_flag_even_while_disabled);
    RUN_TEST(test_heap_hook_value_appears_in_state);
    RUN_TEST(test_inhibit_enable_blocks_enable_on);
    RUN_TEST(test_inhibit_still_allows_enable_off);
    RUN_TEST(test_state_json_has_correct_array_lengths);
    return UNITY_END();
}
