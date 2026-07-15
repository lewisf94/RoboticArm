#pragma once

// Unit conventions (see docs/architecture.md): degrees and millimetres at every
// API boundary; radians exist only inside kinematics implementation files.

namespace arm {

inline constexpr float kPi = 3.14159265358979323846f;

constexpr float deg2rad(float deg) { return deg * (kPi / 180.0f); }
constexpr float rad2deg(float rad) { return rad * (180.0f / kPi); }

constexpr float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace arm
