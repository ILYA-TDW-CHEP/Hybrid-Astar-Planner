#include "common/angle_utils.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::absAngularDistance;
using hybrid_astar::common::kPi;
using hybrid_astar::common::kTwoPi;
using hybrid_astar::common::normalizeAngle;
using hybrid_astar::common::shortestAngularDistance;

TEST(AngleUtilsTest, NormalizesZeroAndFullTurns) {
    EXPECT_DOUBLE_EQ(normalizeAngle(0.0), 0.0);
    EXPECT_DOUBLE_EQ(normalizeAngle(kTwoPi), 0.0);
    EXPECT_DOUBLE_EQ(normalizeAngle(-kTwoPi), 0.0);
}

TEST(AngleUtilsTest, NormalizesToHalfOpenMinusPiPiRange) {
    EXPECT_DOUBLE_EQ(normalizeAngle(kPi), -kPi);
    EXPECT_DOUBLE_EQ(normalizeAngle(-kPi), -kPi);
    EXPECT_DOUBLE_EQ(normalizeAngle(3.0 * kPi), -kPi);
    EXPECT_DOUBLE_EQ(normalizeAngle(-3.0 * kPi), -kPi);
}

TEST(AngleUtilsTest, ComputesShortestSignedDistance) {
    EXPECT_NEAR(shortestAngularDistance(0.75 * kPi, -0.75 * kPi), 0.5 * kPi, 1e-12);
    EXPECT_NEAR(shortestAngularDistance(-0.75 * kPi, 0.75 * kPi), -0.5 * kPi, 1e-12);
}

TEST(AngleUtilsTest, ComputesAbsoluteDistance) {
    EXPECT_NEAR(absAngularDistance(0.75 * kPi, -0.75 * kPi), 0.5 * kPi, 1e-12);
    EXPECT_NEAR(absAngularDistance(-0.75 * kPi, 0.75 * kPi), 0.5 * kPi, 1e-12);
}

}  // namespace
