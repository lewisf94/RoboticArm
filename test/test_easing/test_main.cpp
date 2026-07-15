// Native unit tests for arm_core easing + units.
// Pattern for all core test suites: one directory per suite under test/,
// plain main() with UNITY_BEGIN/RUN_TEST/UNITY_END. Run: pio test -e native

#include <unity.h>

#include "arm_core/easing.h"
#include "arm_core/units.h"

void setUp() {}
void tearDown() {}

static void test_ease_endpoints() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, arm::ease(arm::Easing::kCubicInOut, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, arm::ease(arm::Easing::kCubicInOut, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, arm::ease(arm::Easing::kLinear, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, arm::ease(arm::Easing::kLinear, 1.0f));
}

static void test_ease_midpoint() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, arm::ease(arm::Easing::kCubicInOut, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, arm::ease(arm::Easing::kLinear, 0.5f));
}

static void test_ease_clamps_outside_range() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, arm::ease(arm::Easing::kCubicInOut, -3.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, arm::ease(arm::Easing::kCubicInOut, 42.0f));
}

static void test_ease_monotonic() {
    float prev = -1.0f;
    for (int i = 0; i <= 100; ++i) {
        const float v = arm::ease(arm::Easing::kCubicInOut, i / 100.0f);
        TEST_ASSERT_TRUE(v >= prev);
        prev = v;
    }
}

static void test_units_roundtrip() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, arm::rad2deg(arm::deg2rad(90.0f)));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, arm::kPi, arm::deg2rad(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.0f, arm::clampf(7.0f, 0.0f, 5.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, arm::clampf(-1.0f, 0.0f, 5.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ease_endpoints);
    RUN_TEST(test_ease_midpoint);
    RUN_TEST(test_ease_clamps_outside_range);
    RUN_TEST(test_ease_monotonic);
    RUN_TEST(test_units_roundtrip);
    return UNITY_END();
}
