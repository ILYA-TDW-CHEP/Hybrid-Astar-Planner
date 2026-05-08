#include "planner/collision_checker.hpp"

#include "common/math_utils.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::kPi;
using hybrid_astar::common::Pose2D;
using hybrid_astar::map::OccupancyGrid;
using hybrid_astar::planner::CollisionChecker;
using hybrid_astar::vehicle::Footprint;
using hybrid_astar::vehicle::VehicleParams;

CollisionChecker makeChecker(const OccupancyGrid& grid) {
    static const Footprint footprint{VehicleParams{2.0, 1.0, 1.5, 0.6}};
    return CollisionChecker{grid, footprint};
}

TEST(CollisionCheckerTest, ReturnsFalseInFreeSpace) {
    const OccupancyGrid grid{20, 20, 1.0};
    const auto checker = makeChecker(grid);

    EXPECT_FALSE(checker.isCollision(Pose2D{5.0, 5.0, 0.0}));
}

TEST(CollisionCheckerTest, ReturnsTrueWhenOccupiedCellIntersectsFootprint) {
    OccupancyGrid grid{20, 20, 1.0};
    grid.setOccupied(5, 5, true);
    const auto checker = makeChecker(grid);

    EXPECT_TRUE(checker.isCollision(Pose2D{5.0, 5.0, 0.0}));
}

TEST(CollisionCheckerTest, ReturnsFalseWhenOccupiedCellInsideAabbButOutsideFootprint) {
    OccupancyGrid grid{20, 20, 1.0};
    grid.setOccupied(8, 11, true);

    const Footprint footprint{VehicleParams{4.0, 1.0, 2.5, 0.6}};
    const CollisionChecker checker{grid, footprint};

    EXPECT_FALSE(checker.isCollision(Pose2D{10.0, 10.0, 0.25 * kPi}));
}

TEST(CollisionCheckerTest, ReturnsTrueWhenFootprintLeavesMap) {
    const OccupancyGrid grid{10, 10, 1.0};
    const auto checker = makeChecker(grid);

    EXPECT_TRUE(checker.isCollision(Pose2D{0.2, 0.2, 0.0}));
}

TEST(CollisionCheckerTest, ReturnsTrueForRotatedVehicleCollision) {
    OccupancyGrid grid{20, 20, 1.0};
    grid.setOccupied(11, 11, true);

    const Footprint footprint{VehicleParams{4.0, 1.0, 2.5, 0.6}};
    const CollisionChecker checker{grid, footprint};

    EXPECT_TRUE(checker.isCollision(Pose2D{10.0, 10.0, 0.25 * kPi}));
}

}  // namespace
