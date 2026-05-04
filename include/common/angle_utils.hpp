#pragma once

#include <cmath>

#include "common/math_utils.hpp"

namespace hybrid_astar::common {

inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

inline double shortestAngularDistance(double from, double to) {
    return normalizeAngle(to - from);
}

inline double absAngularDistance(double from, double to) {
    return std::abs(shortestAngularDistance(from, to));
}

}
