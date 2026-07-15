#include "arm_core/easing.h"

#include "arm_core/units.h"

namespace arm {

float ease(Easing type, float t) {
    t = clampf(t, 0.0f, 1.0f);
    switch (type) {
        case Easing::kLinear:
            return t;
        case Easing::kCubicInOut:
            if (t < 0.5f) return 4.0f * t * t * t;
            {
                const float u = -2.0f * t + 2.0f;
                return 1.0f - (u * u * u) / 2.0f;
            }
    }
    return t;
}

}  // namespace arm
