#pragma once

// Easing curves for synchronized joint moves (docs/kinematics.md, "Synchronized
// multi-joint moves"). Input t is normalized time; inputs outside [0,1] clamp.

namespace arm {

enum class Easing {
    kLinear,
    kCubicInOut,  // default for pose moves
};

// Returns eased progress in [0,1] for t in [0,1] (clamped outside).
float ease(Easing type, float t);

}  // namespace arm
