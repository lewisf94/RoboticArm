#pragma once

#include <cstdint>

// Joint + arm geometry configuration — the data model everything else in
// arm_core builds on. See docs/architecture.md ("Geometry lives in data")
// and docs/kinematics.md for what the ArmGeometry fields mean.

namespace arm {

inline constexpr uint8_t kMaxJoints = 8;

struct JointConfig {
    const char* name;
    float min_deg;
    float max_deg;
    float home_deg;
    float vmax_dps = 120.0f;
    uint8_t channel = 0;
    uint16_t us_min = 500;
    uint16_t us_max = 2500;
    int8_t dir = 1;
    float mount_offset_deg = 0.0f;
    bool is_gripper = false;
};

struct ArmGeometry {
    float base_h = 0.0f;
    float r_off = 0.0f;
    float L1 = 0.0f;
    float L2 = 0.0f;
    float Lw = 0.0f;
    bool has_wrist_pitch = false;
};

struct ArmProfile {
    const char* name;
    uint8_t n_joints;
    JointConfig joints[kMaxJoints];
    ArmGeometry geo;
};

// Every joint in [0, n_joints): min < max, home within [min,max], us_min <
// us_max, and channels unique across joints. Array slots at/beyond n_joints
// are ignored.
inline bool validate(const ArmProfile& profile) {
    if (profile.n_joints == 0 || profile.n_joints > kMaxJoints) return false;
    for (uint8_t i = 0; i < profile.n_joints; ++i) {
        const JointConfig& j = profile.joints[i];
        if (!(j.min_deg < j.max_deg)) return false;
        if (j.home_deg < j.min_deg || j.home_deg > j.max_deg) return false;
        if (!(j.us_min < j.us_max)) return false;
        for (uint8_t k = i + 1; k < profile.n_joints; ++k) {
            if (profile.joints[k].channel == j.channel) return false;
        }
    }
    return true;
}

}  // namespace arm
