#pragma once

#include <cstdint>

#include "arm_core/joint_output.h"

// Direct LEDC-driven servo output: one physical GPIO per channel, 50 Hz /
// 14-bit PWM (docs/hardware.md). "Channel" here is arm_core's abstract
// joint-output channel (JointConfig::channel), not a GPIO number - the
// channel->GPIO table lives in src/pins.h and is passed in at construction,
// keeping this class profile-agnostic.

namespace arm_hal {

class LedcServoOutput : public arm::IJointOutput {
public:
    // pins[i] is the GPIO for arm_core channel i; n is the number of usable
    // LEDC channels (8 on the S3). pins must outlive this object.
    LedcServoOutput(const uint8_t* pins, uint8_t n);

    bool attach(uint8_t channel) override;
    void detach_all() override;
    void write_us(uint8_t channel, uint16_t us) override;

private:
    const uint8_t* pins_;
    uint8_t n_;
};

}  // namespace arm_hal
