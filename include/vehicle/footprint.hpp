#pragma once

#include <array>

#include "common/pose.hpp"
#include "vehicle/vehicle_model.hpp"

namespace hybrid_astar::vehicle {

struct Point2D {
    double x{0.0};
    double y{0.0};
};

constexpr Point2D operator+(const Point2D& lhs, const Point2D& rhs) {
    return Point2D{lhs.x + rhs.x, lhs.y + rhs.y};
}

constexpr Point2D operator-(const Point2D& lhs, const Point2D& rhs) {
    return Point2D{lhs.x - rhs.x, lhs.y - rhs.y};
}

constexpr Point2D operator*(const Point2D& point, double scalar) {
    return Point2D{point.x * scalar, point.y * scalar};
}

constexpr Point2D operator*(double scalar, const Point2D& point) {
    return point * scalar;
}

class Footprint {
public:
    explicit Footprint(VehicleParams params);

    std::array<Point2D, 4> corners(const common::Pose2D& pose) const;

private:
    VehicleParams params_;
};

}  // namespace hybrid_astar::vehicle
