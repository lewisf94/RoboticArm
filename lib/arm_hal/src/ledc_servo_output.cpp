#include "arm_hal/ledc_servo_output.h"

#include <Arduino.h>

namespace arm_hal {

namespace {
constexpr uint32_t kFreqHz = 50;
constexpr uint8_t kResolutionBits = 14;
constexpr uint32_t kDutyMax = (1u << kResolutionBits) - 1;
constexpr float kPeriodUs = 1000000.0f / static_cast<float>(kFreqHz);  // 20000
}  // namespace

LedcServoOutput::LedcServoOutput(const uint8_t* pins, uint8_t n) : pins_(pins), n_(n) {}

bool LedcServoOutput::attach(uint8_t channel) {
    if (channel >= n_) return false;
    ledcSetup(channel, kFreqHz, kResolutionBits);
    ledcAttachPin(pins_[channel], channel);
    return true;
}

void LedcServoOutput::detach_all() {
    for (uint8_t ch = 0; ch < n_; ++ch) {
        ledcDetachPin(pins_[ch]);
    }
}

void LedcServoOutput::write_us(uint8_t channel, uint16_t us) {
    if (channel >= n_) return;
    const uint32_t duty =
        static_cast<uint32_t>((static_cast<float>(us) / kPeriodUs) * static_cast<float>(kDutyMax) + 0.5f);
    ledcWrite(channel, duty);
}

}  // namespace arm_hal
