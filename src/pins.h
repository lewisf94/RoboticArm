#pragma once

#include <cstdint>

// GPIO pin map - MUST exactly match the table in docs/hardware.md. Change
// both together or neither (CLAUDE.md architecture rule).

namespace pins {

// arm_core channel i (JointConfig::channel) -> GPIO.
inline constexpr uint8_t kServoGpio[8] = {15, 16, 17, 18, 21, 47, 39, 40};

inline constexpr uint8_t kEstop = 10;   // NC switch to GND, input pull-up (wired in T05)
inline constexpr uint8_t kRgbLed = 48;  // status WS2812 (wired in T05)

}  // namespace pins
