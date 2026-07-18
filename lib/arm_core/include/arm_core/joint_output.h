#pragma once

#include <cstdint>

// Hardware output interface for one joint's physical actuator. Implemented
// in lib/arm_hal (e.g. LedcServoOutput for direct LEDC-driven servos, later
// a PCA9685 or stepper driver) - arm_core depends only on this abstraction,
// never a concrete driver. See docs/architecture.md.

namespace arm {

class IJointOutput {
public:
    virtual bool attach(uint8_t channel) = 0;
    virtual void detach_all() = 0;
    virtual void write_us(uint8_t channel, uint16_t us) = 0;
    virtual ~IJointOutput() = default;
};

}  // namespace arm
