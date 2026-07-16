// Native unit tests for arm_core config + JointModel.
// Run: pio test -e native

#include <unity.h>

#include "arm_core/config.h"
#include "arm_core/joint_model.h"
#include "arm_core/profiles/bench_3dof.h"

void setUp() {}
void tearDown() {}

static const arm::JointConfig kTestJoint{"test", -90.0f, 90.0f, 0.0f, 120.0f, 0};

static void test_clamp_bounds() {
    arm::JointModel j(kTestJoint);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -90.0f, j.clamp(-200.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 90.0f, j.clamp(200.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 45.0f, j.clamp(45.0f));
}

static void test_set_target_rejects_out_of_range_without_mutating() {
    arm::JointModel j(kTestJoint);

    TEST_ASSERT_TRUE(j.set_target(30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 30.0f, j.target_deg());

    TEST_ASSERT_FALSE(j.set_target(500.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 30.0f, j.target_deg());  // unchanged, not clamped

    TEST_ASSERT_FALSE(j.set_target(-500.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 30.0f, j.target_deg());
}

static void test_trim_affects_output_not_target() {
    arm::JointModel j(kTestJoint);
    TEST_ASSERT_TRUE(j.set_target(20.0f));
    j.set_current_deg(20.0f);
    const uint16_t us_before = j.output_us();

    j.set_trim_deg(5.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, j.target_deg());  // trim doesn't touch target
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 25.0f, j.output_deg());
    TEST_ASSERT_TRUE(j.output_us() != us_before);
}

static void test_dir_negative_mirrors_output() {
    const arm::JointConfig cfg{"mirrored", -90.0f, 90.0f, 0.0f, 120.0f, 0, 500, 2500, -1, 0.0f, false};
    arm::JointModel j(cfg);
    j.set_current_deg(30.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -30.0f, j.output_deg());
}

static void test_output_us_endpoints_and_midpoint() {
    arm::JointModel j(kTestJoint);

    j.set_current_deg(-90.0f);
    TEST_ASSERT_EQUAL_UINT16(500, j.output_us());

    j.set_current_deg(90.0f);
    TEST_ASSERT_EQUAL_UINT16(2500, j.output_us());

    j.set_current_deg(0.0f);
    TEST_ASSERT_EQUAL_UINT16(1500, j.output_us());
}

static void test_validate_bench_3dof() {
    TEST_ASSERT_TRUE(arm::validate(arm::kBench3Dof));
}

static void test_validate_catches_home_outside_limits() {
    arm::ArmProfile p = arm::kBench3Dof;
    p.joints[0].home_deg = 500.0f;
    TEST_ASSERT_FALSE(arm::validate(p));
}

static void test_validate_catches_duplicate_channels() {
    arm::ArmProfile p = arm::kBench3Dof;
    p.joints[1].channel = p.joints[0].channel;
    TEST_ASSERT_FALSE(arm::validate(p));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_clamp_bounds);
    RUN_TEST(test_set_target_rejects_out_of_range_without_mutating);
    RUN_TEST(test_trim_affects_output_not_target);
    RUN_TEST(test_dir_negative_mirrors_output);
    RUN_TEST(test_output_us_endpoints_and_midpoint);
    RUN_TEST(test_validate_bench_3dof);
    RUN_TEST(test_validate_catches_home_outside_limits);
    RUN_TEST(test_validate_catches_duplicate_channels);
    return UNITY_END();
}
