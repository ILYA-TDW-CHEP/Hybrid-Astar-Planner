#include "common/math_utils.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::clamp;
using hybrid_astar::common::distance2D;
using hybrid_astar::common::nearlyEqual;
using hybrid_astar::common::square;

TEST(MathUtilsTest, SquaresValues) {
    EXPECT_DOUBLE_EQ(square(3.0), 9.0);
    EXPECT_DOUBLE_EQ(square(-4.0), 16.0);
}

TEST(MathUtilsTest, ClampsValuesToBounds) {
    EXPECT_DOUBLE_EQ(clamp(2.0, 0.0, 5.0), 2.0);
    EXPECT_DOUBLE_EQ(clamp(-1.0, 0.0, 5.0), 0.0);
    EXPECT_DOUBLE_EQ(clamp(6.0, 0.0, 5.0), 5.0);
}

TEST(MathUtilsTest, ComputesPlanarDistance) {
    EXPECT_DOUBLE_EQ(distance2D(0.0, 0.0, 3.0, 4.0), 5.0);
}

TEST(MathUtilsTest, ComparesWithTolerance) {
    EXPECT_TRUE(nearlyEqual(1.0, 1.0 + 1e-10));
    EXPECT_FALSE(nearlyEqual(1.0, 1.0 + 1e-6));
}

}  // namespace
