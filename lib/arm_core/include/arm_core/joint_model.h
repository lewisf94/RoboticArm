#pragma once

#include <cstdint>

#include "arm_core/config.h"

// Per-joint runtime state bound to a JointConfig: current/target angle in the
// profile's logical frame (the frame limits/home/targets are expressed in),
// a runtime trim, and the mapping from that logical frame to a physical servo
// pulse width. Limit clamping happens here unconditionally — see
// docs/architecture.md safety model.

namespace arm {

class JointModel {
public:
    explicit JointModel(const JointConfig& config)
        : config_(&config), current_deg_(config.home_deg), target_deg_(config.home_deg) {}

    // Clamps deg into [min_deg, max_deg].
    float clamp(float deg) const;

    // Sets target_deg to deg if it's within [min_deg, max_deg]; otherwise
    // leaves target_deg unchanged and returns false.
    bool set_target(float deg);

    // Servo-frame angle: dir * (current_deg + trim_deg) + mount_offset_deg.
    float output_deg() const;

    // output_deg() linearly mapped from -90..+90 deg to us_min..us_max,
    // clamped to that range.
    uint16_t output_us() const;

    const JointConfig& config() const { return *config_; }

    float current_deg() const { return current_deg_; }
    void set_current_deg(float deg) { current_deg_ = deg; }

    float target_deg() const { return target_deg_; }

    float trim_deg() const { return trim_deg_; }
    void set_trim_deg(float deg) { trim_deg_ = deg; }

private:
    const JointConfig* config_;
    float current_deg_;
    float target_deg_;
    float trim_deg_ = 0.0f;
};

}  // namespace arm
