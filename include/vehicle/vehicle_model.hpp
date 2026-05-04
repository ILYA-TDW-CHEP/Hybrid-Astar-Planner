#pragma once

#include <cmath>
#include "common/pose.hpp"

namespace hybrid_astar::vehicle {

struct VehicleParams {
    double length{4.5};
    double width{2.0};
    double wheelbase{2.7};
    double max_steering_angle{0.6};
};

class VehicleModel {
public:
    explicit VehicleModel(VehicleParams params);

    const VehicleParams& params() const;

    common::Pose2D step(
        const common::Pose2D& pose,
        double steering_angle,
        double distance
    ) const;

private:
    VehicleParams params_;
};

}  // namespace hybrid_astar::vehicle
