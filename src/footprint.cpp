#include "vehicle/footprint.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"

#include <cmath>

namespace hybrid_astar::vehicle {
    Footprint::Footprint(VehicleParams params) : params_(params) {}

    std::array<Point2D, 4> Footprint::corners(const common::Pose2D& pose) const {

        auto forward_vect = Point2D{std::cos(pose.theta), std::sin(pose.theta)};
        auto right_vect = Point2D{-std::sin(pose.theta), std::cos(pose.theta)};
        auto center = Point2D{pose.x, pose.y};

        double half_l = params_.length / 2;
        double half_w = params_.width / 2;


        auto front_left  = center + forward_vect * half_l + right_vect * half_w;
        auto front_right = center + forward_vect * half_l - right_vect * half_w;
        auto rear_right  = center - forward_vect * half_l - right_vect * half_w;
        auto rear_left   = center - forward_vect * half_l + right_vect * half_w;

        return {front_left, front_right, rear_right, rear_left};
    }

}  // namespace hybrid_astar::vehicle
