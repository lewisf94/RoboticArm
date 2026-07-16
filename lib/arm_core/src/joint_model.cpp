#include "arm_core/joint_model.h"

#include "arm_core/units.h"

namespace arm {

float JointModel::clamp(float deg) const {
    return clampf(deg, config_->min_deg, config_->max_deg);
}

bool JointModel::set_target(float deg) {
    if (clamp(deg) != deg) return false;
    target_deg_ = deg;
    return true;
}

float JointModel::output_deg() const {
    return static_cast<float>(config_->dir) * (current_deg_ + trim_deg_) + config_->mount_offset_deg;
}

uint16_t JointModel::output_us() const {
    const float deg = clampf(output_deg(), -90.0f, 90.0f);
    const float t = (deg + 90.0f) / 180.0f;
    const float span = static_cast<float>(config_->us_max) - static_cast<float>(config_->us_min);
    const float us = static_cast<float>(config_->us_min) + t * span;
    return static_cast<uint16_t>(us + 0.5f);
}

}  // namespace arm
