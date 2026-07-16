#include "arm_core/motion.h"

#include <cmath>

#include "arm_core/easing.h"
#include "arm_core/units.h"

namespace arm {

namespace {
constexpr float kArriveEpsilonDeg = 0.01f;
}  // namespace

// joints_ is constructed directly from profile.joints[0..kMaxJoints-1] since
// JointModel has no default constructor (it must always be bound to a
// JointConfig). This list's length must track kMaxJoints (config.h).
MotionController::MotionController(const ArmProfile& profile)
    : profile_(profile),
      joints_{JointModel(profile.joints[0]), JointModel(profile.joints[1]),
              JointModel(profile.joints[2]), JointModel(profile.joints[3]),
              JointModel(profile.joints[4]), JointModel(profile.joints[5]),
              JointModel(profile.joints[6]), JointModel(profile.joints[7])} {}

void MotionController::tick(uint32_t dt_ms) {
    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
    const float t = (dur_ms_ == 0)
                         ? 1.0f
                         : static_cast<float>(elapsed_ms_) / static_cast<float>(dur_ms_);

    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        JointModel& jm = joints_[i];
        const float desired =
            traj_[i].active ? traj_[i].start_deg + traj_[i].delta_deg * ease(Easing::kCubicInOut, t)
                             : jm.target_deg();
        const float max_step = jm.config().vmax_dps * dt_s;
        const float step = clampf(desired - jm.current_deg(), -max_step, max_step);
        jm.set_current_deg(jm.current_deg() + step);
    }

    elapsed_ms_ += dt_ms;
}

MoveResult MotionController::move_to(const float* targets_deg, uint8_t n, uint32_t dur_ms) {
    if (targets_deg == nullptr) return MoveResult{MoveResult::ok, 0};
    if (n != profile_.n_joints) return MoveResult{MoveResult::bad_joint, profile_.n_joints};

    // Pass 1: validate everything before mutating anything.
    for (uint8_t i = 0; i < n; ++i) {
        if (std::isnan(targets_deg[i])) continue;
        if (joints_[i].clamp(targets_deg[i]) != targets_deg[i]) {
            return MoveResult{MoveResult::out_of_range, i};
        }
    }

    // Pass 2: apply. A joint not given a new target this call goes idle —
    // its target is unchanged and it slews toward that at vmax_dps (tick()).
    float max_ratio_s = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        if (std::isnan(targets_deg[i])) {
            traj_[i].active = false;
            continue;
        }
        const float start = joints_[i].current_deg();
        traj_[i].start_deg = start;
        traj_[i].delta_deg = targets_deg[i] - start;
        traj_[i].active = true;
        joints_[i].set_target(targets_deg[i]);  // known in-range from pass 1

        const float vmax = joints_[i].config().vmax_dps;
        if (vmax > 0.0f) {
            const float ratio_s = std::fabs(traj_[i].delta_deg) / vmax;
            if (ratio_s > max_ratio_s) max_ratio_s = ratio_s;
        }
    }

    dur_ms_ = (dur_ms != 0) ? dur_ms : static_cast<uint32_t>(max_ratio_s * 1000.0f + 0.5f);
    elapsed_ms_ = 0;
    return MoveResult{MoveResult::ok, 0};
}

bool MotionController::set_joint(uint8_t j, float deg, float vmax_override_dps) {
    if (j >= profile_.n_joints) return false;

    float targets[kMaxJoints];
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        targets[i] = (i == j) ? deg : NAN;
    }

    uint32_t dur_ms = 0;
    if (vmax_override_dps > 0.0f) {
        const float delta = std::fabs(deg - joints_[j].current_deg());
        dur_ms = static_cast<uint32_t>((delta / vmax_override_dps) * 1000.0f + 0.5f);
    }

    return move_to(targets, profile_.n_joints, dur_ms).code == MoveResult::ok;
}

bool MotionController::moving() const {
    for (uint8_t i = 0; i < profile_.n_joints; ++i) {
        if (std::fabs(joints_[i].current_deg() - joints_[i].target_deg()) > kArriveEpsilonDeg) {
            return true;
        }
    }
    return false;
}

float MotionController::progress() const {
    if (dur_ms_ == 0) return 1.0f;
    return clampf(static_cast<float>(elapsed_ms_) / static_cast<float>(dur_ms_), 0.0f, 1.0f);
}

}  // namespace arm
