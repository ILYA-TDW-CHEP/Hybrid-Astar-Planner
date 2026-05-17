#include "planner/heuristic.hpp"

#include "common/math_utils.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::kPi;
using hybrid_astar::common::Pose2D;
using hybrid_astar::planner::Heuristic;
using hybrid_astar::planner::HeuristicConfig;

TEST(HeuristicTest, StoresConfig) {
    const Heuristic heuristic{HeuristicConfig{2.0, 0.5}};

    EXPECT_DOUBLE_EQ(heuristic.config().distance_weight, 2.0);
    EXPECT_DOUBLE_EQ(heuristic.config().angle_weight, 0.5);
}

TEST(HeuristicTest, ReturnsZeroForSamePose) {
    const Heuristic heuristic;

    EXPECT_DOUBLE_EQ(heuristic.estimate(Pose2D{1.0, 2.0, 0.3}, Pose2D{1.0, 2.0, 0.3}), 0.0);
}

TEST(HeuristicTest, UsesEuclideanDistance) {
    const Heuristic heuristic{HeuristicConfig{1.0, 0.0}};

    EXPECT_DOUBLE_EQ(heuristic.estimate(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, 0.0}), 5.0);
}

TEST(HeuristicTest, UsesShortestAngularDistance) {
    const Heuristic heuristic{HeuristicConfig{0.0, 2.0}};

    EXPECT_NEAR(heuristic.estimate(Pose2D{0.0, 0.0, 0.75 * kPi}, Pose2D{0.0, 0.0, -0.75 * kPi}), kPi, 1e-12);
}

TEST(HeuristicTest, CombinesDistanceAndAngleTerms) {
    const Heuristic heuristic{HeuristicConfig{2.0, 0.5}};

    EXPECT_NEAR(heuristic.estimate(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, kPi}), 10.0 + 0.5 * kPi, 1e-12);
}

}  // namespace
