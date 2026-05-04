#include "vehicle/vehicle_model.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"

#include <cmath>

namespace hybrid_astar::vehicle {

VehicleModel::VehicleModel(VehicleParams params) : params_(params) {}

const VehicleParams& VehicleModel::params() const {
    return params_;
}

common::Pose2D VehicleModel::step(
    const common::Pose2D& pose,
    double steering_angle,
    double distance
) const {
    const double clamped_steering = common::clamp(
        steering_angle,
        -params_.max_steering_angle,
        params_.max_steering_angle
    );

    const double next_x = pose.x + distance * std::cos(pose.theta);
    const double next_y = pose.y + distance * std::sin(pose.theta);
    const double next_theta = common::normalizeAngle(
        pose.theta + distance / params_.wheelbase * std::tan(clamped_steering)
    );

    return common::Pose2D{next_x, next_y, next_theta};
}

}  // namespace hybrid_astar::vehicle
