#pragma once

#include <cmath>

namespace hybrid_astar::common {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

constexpr double square(double value) {
    return value * value;
}

constexpr double clamp(double value, double min_value, double max_value) {
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

inline double distance2D(double x0, double y0, double x1, double y1) {
    return std::hypot(x1 - x0, y1 - y0);
}

inline bool nearlyEqual(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace hybrid_astar::common
