#pragma once

namespace hybrid_astar::common {

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    constexpr Pose2D() = default;

    constexpr Pose2D(double x_value, double y_value, double theta_value)
        : x{x_value}, y{y_value}, theta{theta_value} {}
};

}  // namespace hybrid_astar::common
