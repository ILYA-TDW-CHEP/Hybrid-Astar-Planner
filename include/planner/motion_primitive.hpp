#pragma once

#include <vector>

#include "common/pose.hpp"
#include "vehicle/vehicle_model.hpp"

namespace hybrid_astar::planner {

enum class MotionDirection {
    Forward,
    Reverse,
};

struct MotionPrimitive {
    double steering_angle{0.0};
    double distance{0.0};
    MotionDirection direction{MotionDirection::Forward};
    common::Pose2D end_pose{};
    double cost{0.0};
};

struct MotionPrimitiveConfig {
    double step_distance{1.0};
    int steering_angle_count{3};
    bool allow_reverse{true};
    double reverse_cost_multiplier{2.0};
    double steering_cost_weight{0.1};
};

class MotionPrimitiveGenerator {
public:
    MotionPrimitiveGenerator(
        const vehicle::VehicleModel& vehicle_model,
        MotionPrimitiveConfig config
    );

    const MotionPrimitiveConfig& config() const;

    std::vector<MotionPrimitive> generate(const common::Pose2D& pose) const;

private:
    const vehicle::VehicleModel& vehicle_model_;
    MotionPrimitiveConfig config_;
};

}  // namespace hybrid_astar::planner
