// Native unit tests for arm_core PoseStore + the pose protocol commands.
// Run: pio test -e native

#include <cstdio>
#include <cstring>

#include <ArduinoJson.h>
#include <unity.h>

#include "arm_core/file_store.h"
#include "arm_core/motion.h"
#include "arm_core/pose_store.h"
#include "arm_core/profiles/bench_3dof.h"
#include "arm_core/protocol.h"

void setUp() {}
void tearDown() {}

namespace {

// bench_3dof: base [-90,90], shoulder [0,120], grip [0,60] (gripper).

struct Fixture {
    arm::MotionController motion;
    bool enabled = false;
    arm::Protocol proto;

    explicit Fixture() : motion(arm::kBench3Dof), proto(motion, arm::kBench3Dof, arm::Protocol::SystemHooks{&enabled}) {}
};

ArduinoJson::JsonDocument call(arm::Protocol& proto, const char* line) {
    char out[512] = {0};
    proto.handle_line(line, out, sizeof(out));
    ArduinoJson::JsonDocument doc;
    ArduinoJson::deserializeJson(doc, out);
    return doc;
}

void set_targets(Fixture& f, float base, float shoulder, float grip) {
    f.enabled = true;
    char line[128];
    std::snprintf(line, sizeof(line), R"({"cmd":"set_joint","j":0,"deg":%f})", static_cast<double>(base));
    call(f.proto, line);
    std::snprintf(line, sizeof(line), R"({"cmd":"set_joint","j":1,"deg":%f})", static_cast<double>(shoulder));
    call(f.proto, line);
    std::snprintf(line, sizeof(line), R"({"cmd":"set_joint","j":2,"deg":%f})", static_cast<double>(grip));
    call(f.proto, line);
}

}  // namespace

// ---------------------------------------------------------------------
// PoseStore: CRUD, name validation, capacity
// ---------------------------------------------------------------------

static void test_save_creates_and_find_retrieves() {
    arm::MotionController motion(arm::kBench3Dof);
    motion.set_joint(0, 45.0f);
    motion.set_joint(1, 90.0f);
    motion.set_joint(2, 20.0f);

    arm::PoseStore store;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::ok), static_cast<int>(store.save("home", motion)));
    TEST_ASSERT_EQUAL_UINT8(1, store.count());

    const arm::Pose* p = store.find("home");
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_STRING("home", p->name);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, p->deg[0]);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, p->deg[1]);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, p->deg[2]);

    TEST_ASSERT_NULL(store.find("nope"));
}

static void test_save_overwrite_same_name_updates_in_place() {
    arm::MotionController motion(arm::kBench3Dof);
    arm::PoseStore store;

    motion.set_joint(0, 10.0f);
    store.save("p1", motion);
    TEST_ASSERT_EQUAL_UINT8(1, store.count());

    motion.set_joint(0, 20.0f);
    store.save("p1", motion);
    TEST_ASSERT_EQUAL_UINT8(1, store.count());  // still one pose, not two
    TEST_ASSERT_EQUAL_FLOAT(20.0f, store.find("p1")->deg[0]);
}

static void test_save_rejects_bad_names() {
    arm::MotionController motion(arm::kBench3Dof);
    arm::PoseStore store;

    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::bad_name), static_cast<int>(store.save("", motion)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::bad_name),
                           static_cast<int>(store.save("this-name-is-17ch", motion)));  // 17 chars, max is 16
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::bad_name), static_cast<int>(store.save("a b", motion)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::bad_name), static_cast<int>(store.save("a.b", motion)));
    TEST_ASSERT_EQUAL_UINT8(0, store.count());  // nothing mutated by any rejected save
}

static void test_save_rejects_when_full() {
    arm::MotionController motion(arm::kBench3Dof);
    arm::PoseStore store;

    char name[8];
    for (int i = 0; i < arm::kMaxPoses; ++i) {
        std::snprintf(name, sizeof(name), "p%d", i);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::ok), static_cast<int>(store.save(name, motion)));
    }
    TEST_ASSERT_EQUAL_UINT8(arm::kMaxPoses, store.count());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::full), static_cast<int>(store.save("one_too_many", motion)));
    TEST_ASSERT_EQUAL_UINT8(arm::kMaxPoses, store.count());

    // Overwriting an existing name must still work even while "full".
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::ok), static_cast<int>(store.save("p0", motion)));
}

static void test_remove_deletes_and_shifts() {
    arm::MotionController motion(arm::kBench3Dof);
    arm::PoseStore store;
    store.save("a", motion);
    store.save("b", motion);
    store.save("c", motion);

    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::ok), static_cast<int>(store.remove("b")));
    TEST_ASSERT_EQUAL_UINT8(2, store.count());
    TEST_ASSERT_NULL(store.find("b"));
    TEST_ASSERT_NOT_NULL(store.find("a"));
    TEST_ASSERT_NOT_NULL(store.find("c"));

    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::not_found), static_cast<int>(store.remove("nope")));
}

// ---------------------------------------------------------------------
// Persistence via MemFileStore
// ---------------------------------------------------------------------

static void test_persist_load_round_trip_via_mem_file_store() {
    arm::MotionController motion(arm::kBench3Dof);
    arm::PoseStore a;
    motion.set_joint(0, -45.0f);
    motion.set_joint(1, 10.0f);
    motion.set_joint(2, 60.0f);
    a.save("wave1", motion);
    motion.set_joint(0, 0.0f);
    motion.set_joint(1, 60.0f);
    motion.set_joint(2, 30.0f);
    a.save("home", motion);

    arm::MemFileStore mem;
    TEST_ASSERT_TRUE(a.persist(mem, 3));

    arm::PoseStore b;
    TEST_ASSERT_TRUE(b.load(mem, 3));
    TEST_ASSERT_EQUAL_UINT8(a.count(), b.count());

    for (uint8_t i = 0; i < a.count(); ++i) {
        const arm::Pose& pa = a.at(i);
        const arm::Pose* pb = b.find(pa.name);
        TEST_ASSERT_NOT_NULL(pb);
        for (int j = 0; j < 3; ++j) TEST_ASSERT_EQUAL_FLOAT(pa.deg[j], pb->deg[j]);
    }
}

static void test_load_corrupted_json_returns_false_and_empty_but_usable() {
    arm::MemFileStore mem;
    const char bad[] = "not json {";
    mem.write("/data/poses.json", bad, std::strlen(bad));

    arm::PoseStore store;
    TEST_ASSERT_FALSE(store.load(mem, 3));
    TEST_ASSERT_EQUAL_UINT8(0, store.count());

    // "empty-but-usable": the store must still work normally afterward.
    arm::MotionController motion(arm::kBench3Dof);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(arm::PoseResult::ok), static_cast<int>(store.save("fresh", motion)));
    TEST_ASSERT_EQUAL_UINT8(1, store.count());
}

static void test_load_missing_poses_array_returns_false() {
    arm::MemFileStore mem;
    const char json[] = R"({"foo":1})";
    mem.write("/data/poses.json", json, std::strlen(json));

    arm::PoseStore store;
    TEST_ASSERT_FALSE(store.load(mem, 3));
    TEST_ASSERT_EQUAL_UINT8(0, store.count());
}

static void test_load_drops_length_mismatched_entries() {
    arm::MemFileStore mem;
    // "short" has a 2-element deg array in a 3-joint profile - must be
    // dropped, not crash and not be kept truncated/padded.
    const char json[] = R"({"poses":[
        {"name":"good","deg":[1.0,2.0,3.0]},
        {"name":"short","deg":[1.0,2.0]}
    ]})";
    mem.write("/data/poses.json", json, std::strlen(json));

    arm::PoseStore store;
    TEST_ASSERT_TRUE(store.load(mem, 3));
    TEST_ASSERT_EQUAL_UINT8(1, store.count());
    TEST_ASSERT_NOT_NULL(store.find("good"));
    TEST_ASSERT_NULL(store.find("short"));
}

static void test_load_does_not_clamp_out_of_range_angles() {
    arm::MemFileStore mem;
    // Simulates a poses.json saved under a wider profile, or hand-edited -
    // 999 is nowhere near base's [-90,90]. load() must accept it as-is;
    // clamping is Protocol::cmd_goto_pose's job at apply-time, not load()'s.
    const char json[] = R"({"poses":[{"name":"stale","deg":[999.0,2.0,3.0]}]})";
    mem.write("/data/poses.json", json, std::strlen(json));

    arm::PoseStore store;
    TEST_ASSERT_TRUE(store.load(mem, 3));
    TEST_ASSERT_EQUAL_FLOAT(999.0f, store.find("stale")->deg[0]);
}

// ---------------------------------------------------------------------
// Protocol integration
// ---------------------------------------------------------------------

static void test_save_pose_then_goto_pose_sets_targets() {
    Fixture f;
    set_targets(f, 45.0f, 90.0f, 20.0f);

    auto save_reply = call(f.proto, R"({"cmd":"save_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("ack", save_reply["type"].as<const char*>());

    set_targets(f, -45.0f, 10.0f, 5.0f);  // move away from p1

    auto goto_reply = call(f.proto, R"({"cmd":"goto_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("ack", goto_reply["type"].as<const char*>());
    TEST_ASSERT_EQUAL_FLOAT(45.0f, f.motion.target(0));
    TEST_ASSERT_EQUAL_FLOAT(90.0f, f.motion.target(1));
    TEST_ASSERT_EQUAL_FLOAT(20.0f, f.motion.target(2));
}

static void test_save_pose_rejects_bad_name() {
    Fixture f;
    auto r1 = call(f.proto, R"({"cmd":"save_pose","name":""})");
    TEST_ASSERT_EQUAL_STRING("err", r1["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bad_args", r1["code"].as<const char*>());

    auto r2 = call(f.proto, R"({"cmd":"save_pose","name":"bad name!"})");
    TEST_ASSERT_EQUAL_STRING("err", r2["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("bad_args", r2["code"].as<const char*>());
}

static void test_save_pose_does_not_require_enabled() {
    Fixture f;
    f.enabled = false;
    auto r = call(f.proto, R"({"cmd":"save_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("ack", r["type"].as<const char*>());  // reads existing targets, doesn't move anything
}

static void test_save_pose_store_full_is_storage_error() {
    Fixture f;
    char line[64];
    for (int i = 0; i < arm::kMaxPoses; ++i) {
        std::snprintf(line, sizeof(line), R"({"cmd":"save_pose","name":"p%d"})", i);
        auto r = call(f.proto, line);
        TEST_ASSERT_EQUAL_STRING("ack", r["type"].as<const char*>());
    }
    auto full = call(f.proto, R"({"cmd":"save_pose","name":"one_too_many"})");
    TEST_ASSERT_EQUAL_STRING("err", full["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("storage", full["code"].as<const char*>());
}

static void test_goto_pose_unknown_name_is_not_found() {
    Fixture f;
    f.enabled = true;
    auto r = call(f.proto, R"({"cmd":"goto_pose","name":"nope"})");
    TEST_ASSERT_EQUAL_STRING("err", r["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("not_found", r["code"].as<const char*>());
}

static void test_goto_pose_requires_enabled() {
    Fixture f;
    set_targets(f, 45.0f, 90.0f, 20.0f);
    call(f.proto, R"({"cmd":"save_pose","name":"p1"})");

    f.enabled = false;
    auto disabled_reply = call(f.proto, R"({"cmd":"goto_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("err", disabled_reply["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("disabled", disabled_reply["code"].as<const char*>());

    f.enabled = true;
    auto enabled_reply = call(f.proto, R"({"cmd":"goto_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("ack", enabled_reply["type"].as<const char*>());
}

static void test_list_poses_returns_saved_names() {
    Fixture f;
    call(f.proto, R"({"cmd":"save_pose","name":"alpha"})");
    call(f.proto, R"({"cmd":"save_pose","name":"beta"})");

    auto r = call(f.proto, R"({"cmd":"list_poses"})");
    TEST_ASSERT_EQUAL_STRING("ack", r["type"].as<const char*>());
    auto poses = r["poses"].as<ArduinoJson::JsonArrayConst>();
    TEST_ASSERT_EQUAL_INT(2, poses.size());
    TEST_ASSERT_EQUAL_STRING("alpha", poses[0].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("beta", poses[1].as<const char*>());
}

static void test_delete_pose_removes_it() {
    Fixture f;
    f.enabled = true;
    call(f.proto, R"({"cmd":"save_pose","name":"p1"})");

    auto del = call(f.proto, R"({"cmd":"delete_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("ack", del["type"].as<const char*>());

    auto goto_reply = call(f.proto, R"({"cmd":"goto_pose","name":"p1"})");
    TEST_ASSERT_EQUAL_STRING("err", goto_reply["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("not_found", goto_reply["code"].as<const char*>());
}

static void test_delete_pose_unknown_name_is_not_found() {
    Fixture f;
    auto r = call(f.proto, R"({"cmd":"delete_pose","name":"nope"})");
    TEST_ASSERT_EQUAL_STRING("err", r["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("not_found", r["code"].as<const char*>());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_save_creates_and_find_retrieves);
    RUN_TEST(test_save_overwrite_same_name_updates_in_place);
    RUN_TEST(test_save_rejects_bad_names);
    RUN_TEST(test_save_rejects_when_full);
    RUN_TEST(test_remove_deletes_and_shifts);
    RUN_TEST(test_persist_load_round_trip_via_mem_file_store);
    RUN_TEST(test_load_corrupted_json_returns_false_and_empty_but_usable);
    RUN_TEST(test_load_missing_poses_array_returns_false);
    RUN_TEST(test_load_drops_length_mismatched_entries);
    RUN_TEST(test_load_does_not_clamp_out_of_range_angles);
    RUN_TEST(test_save_pose_then_goto_pose_sets_targets);
    RUN_TEST(test_save_pose_rejects_bad_name);
    RUN_TEST(test_save_pose_does_not_require_enabled);
    RUN_TEST(test_save_pose_store_full_is_storage_error);
    RUN_TEST(test_goto_pose_unknown_name_is_not_found);
    RUN_TEST(test_goto_pose_requires_enabled);
    RUN_TEST(test_list_poses_returns_saved_names);
    RUN_TEST(test_delete_pose_removes_it);
    RUN_TEST(test_delete_pose_unknown_name_is_not_found);
    return UNITY_END();
}
