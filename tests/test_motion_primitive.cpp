#include "planner/motion_primitive.hpp"

#include <algorithm>
#include <cstddef>

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::Pose2D;
using hybrid_astar::planner::MotionDirection;
using hybrid_astar::planner::MotionPrimitive;
using hybrid_astar::planner::MotionPrimitiveConfig;
using hybrid_astar::planner::MotionPrimitiveGenerator;
using hybrid_astar::vehicle::VehicleModel;
using hybrid_astar::vehicle::VehicleParams;

const MotionPrimitive* findPrimitive(
    const std::vector<MotionPrimitive>& primitives,
    MotionDirection direction,
    double steering_angle
) {
    const auto it = std::find_if(
        primitives.begin(),
        primitives.end(),
        [direction, steering_angle](const MotionPrimitive& primitive) {
            return primitive.direction == direction &&
                   std::abs(primitive.steering_angle - steering_angle) < 1e-12;
        }
    );
    return it == primitives.end() ? nullptr : &(*it);
}

TEST(MotionPrimitiveGeneratorTest, StoresConfig) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.5, 5, false, 3.0, 0.2};

    const MotionPrimitiveGenerator generator{vehicle_model, config};

    EXPECT_DOUBLE_EQ(generator.config().step_distance, 1.5);
    EXPECT_EQ(generator.config().steering_angle_count, 5);
    EXPECT_FALSE(generator.config().allow_reverse);
    EXPECT_DOUBLE_EQ(generator.config().reverse_cost_multiplier, 3.0);
    EXPECT_DOUBLE_EQ(generator.config().steering_cost_weight, 0.2);
}

TEST(MotionPrimitiveGeneratorTest, GeneratesForwardOnlyWhenReverseDisabled) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.0, 3, false, 2.0, 0.1};
    const MotionPrimitiveGenerator generator{vehicle_model, config};

    const auto primitives = generator.generate(Pose2D{0.0, 0.0, 0.0});

    EXPECT_EQ(primitives.size(), static_cast<std::size_t>(3));
    EXPECT_TRUE(std::all_of(primitives.begin(), primitives.end(), [](const MotionPrimitive& primitive) {
        return primitive.direction == MotionDirection::Forward && primitive.distance > 0.0;
    }));
}

TEST(MotionPrimitiveGeneratorTest, GeneratesForwardAndReverseWhenEnabled) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.0, 3, true, 2.0, 0.1};
    const MotionPrimitiveGenerator generator{vehicle_model, config};

    const auto primitives = generator.generate(Pose2D{0.0, 0.0, 0.0});

    const auto forward_count = std::count_if(primitives.begin(), primitives.end(), [](const MotionPrimitive& primitive) {
        return primitive.direction == MotionDirection::Forward;
    });
    const auto reverse_count = std::count_if(primitives.begin(), primitives.end(), [](const MotionPrimitive& primitive) {
        return primitive.direction == MotionDirection::Reverse;
    });

    EXPECT_EQ(primitives.size(), static_cast<std::size_t>(6));
    EXPECT_EQ(forward_count, 3);
    EXPECT_EQ(reverse_count, 3);
}

TEST(MotionPrimitiveGeneratorTest, IncludesSteeringExtremesAndZero) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.0, 3, false, 2.0, 0.1};
    const MotionPrimitiveGenerator generator{vehicle_model, config};

    const auto primitives = generator.generate(Pose2D{0.0, 0.0, 0.0});

    ASSERT_NE(findPrimitive(primitives, MotionDirection::Forward, -0.6), nullptr);
    ASSERT_NE(findPrimitive(primitives, MotionDirection::Forward, 0.0), nullptr);
    ASSERT_NE(findPrimitive(primitives, MotionDirection::Forward, 0.6), nullptr);
}

TEST(MotionPrimitiveGeneratorTest, UsesVehicleModelForEndPose) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.0, 3, false, 2.0, 0.1};
    const MotionPrimitiveGenerator generator{vehicle_model, config};

    const auto primitives = generator.generate(Pose2D{0.0, 0.0, 0.0});
    const auto* straight = findPrimitive(primitives, MotionDirection::Forward, 0.0);

    ASSERT_NE(straight, nullptr);
    EXPECT_NEAR(straight->end_pose.x, 1.0, 1e-12);
    EXPECT_NEAR(straight->end_pose.y, 0.0, 1e-12);
    EXPECT_NEAR(straight->end_pose.theta, 0.0, 1e-12);
}

TEST(MotionPrimitiveGeneratorTest, AssignsHigherCostToReversePrimitive) {
    const VehicleModel vehicle_model{VehicleParams{4.5, 2.0, 2.7, 0.6}};
    const MotionPrimitiveConfig config{1.0, 3, true, 2.5, 0.1};
    const MotionPrimitiveGenerator generator{vehicle_model, config};

    const auto primitives = generator.generate(Pose2D{0.0, 0.0, 0.0});
    const auto* forward = findPrimitive(primitives, MotionDirection::Forward, 0.0);
    const auto* reverse = findPrimitive(primitives, MotionDirection::Reverse, 0.0);

    ASSERT_NE(forward, nullptr);
    ASSERT_NE(reverse, nullptr);
    EXPECT_GT(forward->cost, 0.0);
    EXPECT_GT(reverse->cost, forward->cost);
}

}  // namespace
