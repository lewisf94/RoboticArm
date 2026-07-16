#pragma once

#include <cstdint>

#include "arm_core/config.h"
#include "arm_core/joint_model.h"

// Time-domain motion: advances every joint toward its target each tick under
// per-joint velocity limits, with synchronized multi-joint moves eased to
// arrive together. See docs/kinematics.md "Synchronized multi-joint moves"
// and docs/architecture.md safety model. The per-tick velocity guard applies
// unconditionally, even mid-ease: a cubic ease-in-out's peak velocity is 3x
// its average, so the guard binds on whichever joint sets the shared move
// duration — it's the mechanism that keeps that joint's actual speed at or
// under vmax_dps, not a rarely-hit backstop.

namespace arm {

struct MoveResult {
    enum { ok, out_of_range, bad_joint } code;
    uint8_t joint;
};

class MotionController {
public:
    // profile must outlive the MotionController.
    explicit MotionController(const ArmProfile& profile);

    // Advances current_deg of every joint by at most one tick's worth of
    // motion. Call at a steady rate (nominally 50 Hz).
    void tick(uint32_t dt_ms);

    // Starts a new synchronized move covering all n_joints() joints.
    // targets_deg[i] == NAN leaves joint i's target unchanged and out of the
    // synchronized trajectory (it becomes idle: still slews toward whatever
    // its target already is, just without easing). dur_ms == 0 computes the
    // duration from the slowest newly-targeted joint's own vmax_dps.
    // Rejects (and mutates nothing) if any target is outside that joint's
    // limits, if targets_deg is null, or if n doesn't match n_joints().
    MoveResult move_to(const float* targets_deg, uint8_t n, uint32_t dur_ms = 0);

    // Single-joint convenience: move_to() with every other joint left alone.
    // vmax_override_dps, when > 0, sets this move's duration instead of the
    // joint's configured vmax_dps.
    bool set_joint(uint8_t j, float deg, float vmax_override_dps = 0.0f);

    // Preconditions: j < n_joints().
    float current(uint8_t j) const { return joints_[j].current_deg(); }
    float target(uint8_t j) const { return joints_[j].target_deg(); }
    JointModel& joint(uint8_t j) { return joints_[j]; }
    const JointModel& joint(uint8_t j) const { return joints_[j]; }

    // True while any joint (active or idle) hasn't yet reached its target.
    bool moving() const;

    // Elapsed/duration of the current synchronized move, 0..1. Reaches 1 once
    // the nominal move duration has passed, independent of moving() — see
    // moving() for whether joints have physically caught up (they can lag
    // briefly under the per-tick guard, see class comment).
    float progress() const;

    uint32_t duration_ms() const { return dur_ms_; }

    uint8_t n_joints() const { return profile_.n_joints; }

private:
    struct Trajectory {
        float start_deg = 0.0f;
        float delta_deg = 0.0f;
        bool active = false;
    };

    const ArmProfile& profile_;
    JointModel joints_[kMaxJoints];
    Trajectory traj_[kMaxJoints];
    uint32_t dur_ms_ = 0;
    uint32_t elapsed_ms_ = 0;
};

}  // namespace arm
