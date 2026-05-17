#include "planner/hybrid_astar.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::Pose2D;
using hybrid_astar::map::OccupancyGrid;
using hybrid_astar::planner::CollisionChecker;
using hybrid_astar::planner::Heuristic;
using hybrid_astar::planner::HybridAStar;
using hybrid_astar::planner::HybridAStarConfig;
using hybrid_astar::planner::MotionPrimitiveConfig;
using hybrid_astar::planner::MotionPrimitiveGenerator;
using hybrid_astar::vehicle::Footprint;
using hybrid_astar::vehicle::VehicleModel;
using hybrid_astar::vehicle::VehicleParams;

TEST(HybridAStarTest, StoresConfig) {
    const OccupancyGrid grid{20, 20, 1.0};
    const VehicleParams vehicle_params{2.0, 1.0, 1.5, 0.6};
    const VehicleModel vehicle_model{vehicle_params};
    const Footprint footprint{vehicle_params};
    const CollisionChecker collision_checker{grid, footprint};
    const MotionPrimitiveGenerator motion_primitives{
        vehicle_model,
        MotionPrimitiveConfig{1.0, 3, false, 2.0, 0.1}
    };
    const Heuristic heuristic;
    const HybridAStarConfig config{123, 0.25, 0.1};

    const HybridAStar planner{grid, collision_checker, motion_primitives, heuristic, config};

    EXPECT_EQ(planner.config().max_iterations, 123);
    EXPECT_DOUBLE_EQ(planner.config().goal_position_tolerance, 0.25);
    EXPECT_DOUBLE_EQ(planner.config().goal_heading_tolerance, 0.1);
}

TEST(HybridAStarTest, ReturnsSuccessfulSinglePosePathWhenStartAlreadyAtGoal) {
    const OccupancyGrid grid{20, 20, 1.0};
    const VehicleParams vehicle_params{2.0, 1.0, 1.5, 0.6};
    const VehicleModel vehicle_model{vehicle_params};
    const Footprint footprint{vehicle_params};
    const CollisionChecker collision_checker{grid, footprint};
    const MotionPrimitiveGenerator motion_primitives{
        vehicle_model,
        MotionPrimitiveConfig{1.0, 3, false, 2.0, 0.1}
    };
    const Heuristic heuristic;
    const HybridAStar planner{grid, collision_checker, motion_primitives, heuristic};

    const auto result = planner.plan(Pose2D{5.0, 5.0, 0.0}, Pose2D{5.0, 5.0, 0.0});

    ASSERT_TRUE(result.stats.success);
    ASSERT_EQ(result.path.size(), 1U);
    EXPECT_DOUBLE_EQ(result.path.front().x, 5.0);
    EXPECT_DOUBLE_EQ(result.path.front().y, 5.0);
    EXPECT_DOUBLE_EQ(result.path.front().theta, 0.0);
}

TEST(HybridAStarTest, ReturnsFailureWhenStartIsInCollision) {
    OccupancyGrid grid{20, 20, 1.0};
    grid.setOccupied(5, 5, true);

    const VehicleParams vehicle_params{2.0, 1.0, 1.5, 0.6};
    const VehicleModel vehicle_model{vehicle_params};
    const Footprint footprint{vehicle_params};
    const CollisionChecker collision_checker{grid, footprint};
    const MotionPrimitiveGenerator motion_primitives{
        vehicle_model,
        MotionPrimitiveConfig{1.0, 3, false, 2.0, 0.1}
    };
    const Heuristic heuristic;
    const HybridAStar planner{grid, collision_checker, motion_primitives, heuristic};

    const auto result = planner.plan(Pose2D{5.0, 5.0, 0.0}, Pose2D{8.0, 5.0, 0.0});

    EXPECT_FALSE(result.stats.success);
    EXPECT_TRUE(result.path.empty());
}

TEST(HybridAStarTest, FindsStraightPathInEmptyGrid) {
    const OccupancyGrid grid{30, 10, 1.0};
    const VehicleParams vehicle_params{2.0, 1.0, 1.5, 0.6};
    const VehicleModel vehicle_model{vehicle_params};
    const Footprint footprint{vehicle_params};
    const CollisionChecker collision_checker{grid, footprint};
    const MotionPrimitiveGenerator motion_primitives{
        vehicle_model,
        MotionPrimitiveConfig{1.0, 3, false, 2.0, 0.1}
    };
    const Heuristic heuristic;
    const HybridAStar planner{
        grid,
        collision_checker,
        motion_primitives,
        heuristic,
        HybridAStarConfig{1000, 0.25, 0.1}
    };

    const auto result = planner.plan(Pose2D{5.0, 5.0, 0.0}, Pose2D{8.0, 5.0, 0.0});

    ASSERT_TRUE(result.stats.success);
    ASSERT_GE(result.path.size(), 2U);
    EXPECT_NEAR(result.path.front().x, 5.0, 1e-12);
    EXPECT_NEAR(result.path.front().y, 5.0, 1e-12);
    EXPECT_NEAR(result.path.back().x, 8.0, 0.25);
    EXPECT_NEAR(result.path.back().y, 5.0, 0.25);
    EXPECT_GT(result.stats.expanded_nodes, 0);
    EXPECT_GT(result.stats.generated_nodes, 0);
    EXPECT_GT(result.stats.path_length, 0.0);
}

}  // namespace
