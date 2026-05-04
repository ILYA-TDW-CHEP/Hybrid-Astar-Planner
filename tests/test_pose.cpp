#include "common/pose.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::Pose2D;

TEST(Pose2DTest, DefaultsToOrigin) {
    const Pose2D pose;

    EXPECT_DOUBLE_EQ(pose.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

TEST(Pose2DTest, StoresConstructorValues) {
    const Pose2D pose{1.5, -2.0, 0.75};

    EXPECT_DOUBLE_EQ(pose.x, 1.5);
    EXPECT_DOUBLE_EQ(pose.y, -2.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.75);
}

}  // namespace
