#pragma once

#include "arm_core/config.h"

// Bench rig: 2x MG996R (base yaw, shoulder) + 1x SG90 gripper, clamped to a
// board (docs/hardware.md "Bench rig"). Geometry matches Profile A in
// docs/kinematics.md so the kinematics golden test vectors apply directly.

namespace arm {

inline constexpr ArmProfile kBench3Dof = {
    "bench_3dof",
    3,
    {
        JointConfig{"base", -90.0f, 90.0f, 0.0f, 120.0f, 0},
        JointConfig{"shoulder", 0.0f, 120.0f, 60.0f, 120.0f, 1},
        JointConfig{"grip", 0.0f, 60.0f, 30.0f, 120.0f, 2, 500, 2500, 1, 0.0f, true},
    },
    ArmGeometry{60.0f, 0.0f, 120.0f, 140.0f, 0.0f, false},
};

}  // namespace arm
