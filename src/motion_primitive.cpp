#include "planner/motion_primitive.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"


namespace hybrid_astar::planner {
    MotionPrimitiveGenerator::MotionPrimitiveGenerator(
        const vehicle::VehicleModel& vehicle_model,
        MotionPrimitiveConfig config
    ) : vehicle_model_(vehicle_model), config_(config) {}

    const MotionPrimitiveConfig& MotionPrimitiveGenerator::config() const {
        return config_;
    }

    std::vector<MotionPrimitive> MotionPrimitiveGenerator::generate(const common::Pose2D& pose) const {
        int steering_count = config_.steering_angle_count;

        if (steering_count <= 0) {
            return {};
        }
        if (steering_count % 2 == 0) {
            ++steering_count;
        }

        int primitive_count = steering_count * (config_.allow_reverse ? 2 : 1);

        std::vector<MotionPrimitive> result;
        result.reserve(primitive_count);

        double max_angle =vehicle_model_.params().max_steering_angle;
        double angle_step = steering_count == 1 ? 0.0 : (2.0 * max_angle) / (steering_count - 1);

        for (int i = 0; i < steering_count; ++i) {
            double steering_angle = -max_angle + i * angle_step;

            {
                double distance = config_.step_distance;
                auto end_pose = vehicle_model_.step(pose, steering_angle, distance);

                double cost = std::abs(distance);
                cost += config_.steering_cost_weight * std::abs(steering_angle);

                result.emplace_back(MotionPrimitive{
                    steering_angle,
                    distance,
                    MotionDirection::Forward,
                    end_pose,
                    cost
                });
            }
            if (config_.allow_reverse) {
                double distance = -config_.step_distance;
                auto end_pose = vehicle_model_.step(pose, steering_angle, distance);

                double cost = std::abs(distance);
                cost += config_.steering_cost_weight * std::abs(steering_angle);
                cost *= config_.reverse_cost_multiplier;

                result.emplace_back(MotionPrimitive{
                    steering_angle,
                    distance,
                    MotionDirection::Reverse,
                    end_pose,
                    cost
                });
            }
        }

        return result;
    }

} // namespace hybrid_astar::planner