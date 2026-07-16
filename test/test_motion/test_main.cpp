// Native unit tests for arm_core MotionController.
// Run: pio test -e native

#include <cmath>

#include <unity.h>

#include "arm_core/config.h"
#include "arm_core/motion.h"

void setUp() {}
void tearDown() {}

// Three joints with different limits/vmax (0/1/2 -> 90/45/30 deg/s), all
// homed at 0. Deliberately not kBench3Dof: the sync-arrival test needs
// per-joint vmax to differ.
static const arm::ArmProfile kTestProfile = {
    "test3",
    3,
    {
        arm::JointConfig{"j0", -180.0f, 180.0f, 0.0f, 90.0f, 0},
        arm::JointConfig{"j1", -180.0f, 180.0f, 0.0f, 45.0f, 1},
        arm::JointConfig{"j2", -180.0f, 180.0f, 0.0f, 30.0f, 2},
    },
    arm::ArmGeometry{},
};

static const uint32_t kDtMs = 20;  // 50 Hz, matches the firmware motion tick

static void test_moving_and_progress_lifecycle() {
    arm::MotionController mc(kTestProfile);
    TEST_ASSERT_FALSE(mc.moving());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, mc.progress());

    const float targets[3] = {90.0f, 45.0f, 30.0f};
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::ok, mc.move_to(targets, 3).code);
    TEST_ASSERT_TRUE(mc.moving());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, mc.progress());

    for (int i = 0; i < 100; ++i) mc.tick(kDtMs);  // 2000ms, generous vs. the 1000ms move

    TEST_ASSERT_FALSE(mc.moving());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, mc.progress());
}

// delta_i == vmax_i for every joint here (ratio 1.0s each, but different
// deltas AND different vmax), so all three are equally "the limiting joint" -
// they must lag identically under the per-tick guard and arrive together.
static void test_sync_arrival_different_deltas_and_vmax() {
    arm::MotionController mc(kTestProfile);
    const float targets[3] = {90.0f, 45.0f, 30.0f};
    const arm::MoveResult r = mc.move_to(targets, 3);
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::ok, r.code);
    TEST_ASSERT_EQUAL_UINT32(1000, mc.duration_ms());

    int arrive_tick[3] = {-1, -1, -1};
    for (int tick = 1; tick <= 80; ++tick) {
        mc.tick(kDtMs);
        for (int j = 0; j < 3; ++j) {
            if (arrive_tick[j] < 0 && std::fabs(mc.current(j) - targets[j]) < 0.01f) {
                arrive_tick[j] = tick;
            }
        }
    }

    TEST_ASSERT_TRUE(arrive_tick[0] > 0);
    TEST_ASSERT_TRUE(arrive_tick[1] > 0);
    TEST_ASSERT_TRUE(arrive_tick[2] > 0);
    TEST_ASSERT_EQUAL_INT(arrive_tick[0], arrive_tick[1]);
    TEST_ASSERT_EQUAL_INT(arrive_tick[0], arrive_tick[2]);
}

static void test_no_overshoot() {
    arm::MotionController mc(kTestProfile);
    const float targets[3] = {90.0f, -150.0f, 30.0f};  // mixed directions
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::ok, mc.move_to(targets, 3).code);

    for (int tick = 0; tick < 200; ++tick) {
        mc.tick(kDtMs);
        for (int j = 0; j < 3; ++j) {
            if (targets[j] >= 0.0f) {
                TEST_ASSERT_TRUE(mc.current(j) <= targets[j] + 0.01f);
            } else {
                TEST_ASSERT_TRUE(mc.current(j) >= targets[j] - 0.01f);
            }
        }
    }
}

static void test_per_tick_guard_never_exceeds_vmax() {
    arm::MotionController mc(kTestProfile);
    const float targets[3] = {90.0f, 45.0f, 30.0f};  // guaranteed to trigger clamping (see sync test)
    mc.move_to(targets, 3);

    const float dt_s = static_cast<float>(kDtMs) / 1000.0f;
    const float vmax[3] = {90.0f, 45.0f, 30.0f};

    for (int tick = 0; tick < 80; ++tick) {
        const float before[3] = {mc.current(0), mc.current(1), mc.current(2)};
        mc.tick(kDtMs);
        for (int j = 0; j < 3; ++j) {
            const float step = std::fabs(mc.current(j) - before[j]);
            TEST_ASSERT_TRUE(step <= vmax[j] * dt_s + 0.01f);
        }
    }
}

static void test_default_duration_equals_slowest_joint() {
    arm::MotionController mc(kTestProfile);
    // ratios: 180/90=2.0s, 45/45=1.0s, 15/30=0.5s -> slowest is j0 at 2.0s
    const float targets[3] = {180.0f, 45.0f, 15.0f};
    mc.move_to(targets, 3);
    TEST_ASSERT_EQUAL_UINT32(2000, mc.duration_ms());
}

static void test_retarget_mid_move_restarts_smoothly() {
    arm::MotionController mc(kTestProfile);
    float targets_a[3] = {90.0f, 45.0f, 30.0f};
    mc.move_to(targets_a, 3);  // T=1000ms

    for (int i = 0; i < 25; ++i) mc.tick(kDtMs);  // halfway through (500ms)

    const float mid_current0 = mc.current(0);
    TEST_ASSERT_TRUE(mid_current0 > 1.0f);   // it has moved
    TEST_ASSERT_TRUE(mid_current0 < 89.0f);  // but hasn't arrived

    TEST_ASSERT_TRUE(mc.set_joint(0, -60.0f));  // retarget joint 0 only

    // move_to() only sets up the new trajectory; current_deg doesn't jump
    // until the next tick().
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, mid_current0, mc.current(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -60.0f, mc.target(0));

    for (int i = 0; i < 80; ++i) {
        const float before = mc.current(0);
        mc.tick(kDtMs);
        const float step = std::fabs(mc.current(0) - before);
        TEST_ASSERT_TRUE(step <= 90.0f * 0.02f + 0.01f);  // still respects j0's vmax
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -60.0f, mc.current(0));  // eventually arrives
}

static void test_out_of_range_move_mutates_nothing() {
    arm::MotionController mc(kTestProfile);
    const float targets_baseline[3] = {10.0f, 10.0f, 10.0f};
    mc.move_to(targets_baseline, 3);
    for (int i = 0; i < 10; ++i) mc.tick(kDtMs);  // put it in a non-trivial mid-move state

    const float before_current[3] = {mc.current(0), mc.current(1), mc.current(2)};
    const float before_target[3] = {mc.target(0), mc.target(1), mc.target(2)};

    const float bad_targets[3] = {20.0f, 999.0f, 20.0f};  // j1 limit is +-180
    const arm::MoveResult r = mc.move_to(bad_targets, 3);

    TEST_ASSERT_EQUAL_INT(arm::MoveResult::out_of_range, r.code);
    TEST_ASSERT_EQUAL_UINT8(1, r.joint);

    for (int j = 0; j < 3; ++j) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, before_current[j], mc.current(j));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, before_target[j], mc.target(j));
    }
}

static void test_nan_entries_leave_joints_untouched() {
    arm::MotionController mc(kTestProfile);  // fresh: all at home=0, target=0

    const float targets[3] = {NAN, 45.0f, NAN};
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::ok, mc.move_to(targets, 3).code);

    for (int i = 0; i < 80; ++i) mc.tick(kDtMs);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, mc.current(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, mc.target(0));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 45.0f, mc.current(1));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, mc.current(2));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, mc.target(2));
}

static void test_move_to_rejects_n_mismatch() {
    arm::MotionController mc(kTestProfile);
    const float targets[2] = {10.0f, 10.0f};  // profile has 3 joints
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::bad_joint, mc.move_to(targets, 2).code);
}

static void test_move_to_null_is_noop() {
    arm::MotionController mc(kTestProfile);
    TEST_ASSERT_EQUAL_INT(arm::MoveResult::ok, mc.move_to(nullptr, 3).code);
    TEST_ASSERT_FALSE(mc.moving());
}

static void test_set_joint_rejects_bad_index() {
    arm::MotionController mc(kTestProfile);
    TEST_ASSERT_FALSE(mc.set_joint(5, 10.0f));  // only joints 0..2 exist
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_moving_and_progress_lifecycle);
    RUN_TEST(test_sync_arrival_different_deltas_and_vmax);
    RUN_TEST(test_no_overshoot);
    RUN_TEST(test_per_tick_guard_never_exceeds_vmax);
    RUN_TEST(test_default_duration_equals_slowest_joint);
    RUN_TEST(test_retarget_mid_move_restarts_smoothly);
    RUN_TEST(test_out_of_range_move_mutates_nothing);
    RUN_TEST(test_nan_entries_leave_joints_untouched);
    RUN_TEST(test_move_to_rejects_n_mismatch);
    RUN_TEST(test_move_to_null_is_noop);
    RUN_TEST(test_set_joint_rejects_bad_index);
    return UNITY_END();
}
