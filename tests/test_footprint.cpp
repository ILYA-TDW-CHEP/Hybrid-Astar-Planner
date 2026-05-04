#include "vehicle/footprint.hpp"

#include "common/math_utils.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::kPi;
using hybrid_astar::common::Pose2D;
using hybrid_astar::vehicle::Footprint;
using hybrid_astar::vehicle::Point2D;
using hybrid_astar::vehicle::VehicleParams;

void expectPointNear(const Point2D& point, double x, double y) {
    EXPECT_NEAR(point.x, x, 1e-12);
    EXPECT_NEAR(point.y, y, 1e-12);
}

TEST(Point2DTest, AddsPoints) {
    const Point2D result = Point2D{1.0, 2.0} + Point2D{3.0, -4.0};

    expectPointNear(result, 4.0, -2.0);
}

TEST(Point2DTest, SubtractsPoints) {
    const Point2D result = Point2D{1.0, 2.0} - Point2D{3.0, -4.0};

    expectPointNear(result, -2.0, 6.0);
}

TEST(Point2DTest, MultipliesByScalar) {
    expectPointNear(Point2D{2.0, -3.0} * 0.5, 1.0, -1.5);
    expectPointNear(0.5 * Point2D{2.0, -3.0}, 1.0, -1.5);
}

TEST(FootprintTest, CornersAtZeroHeading) {
    const Footprint footprint{VehicleParams{4.0, 2.0, 2.5, 0.6}};

    const auto corners = footprint.corners(Pose2D{0.0, 0.0, 0.0});

    expectPointNear(corners[0], 2.0, 1.0);
    expectPointNear(corners[1], 2.0, -1.0);
    expectPointNear(corners[2], -2.0, -1.0);
    expectPointNear(corners[3], -2.0, 1.0);
}

TEST(FootprintTest, CornersAtNinetyDegrees) {
    const Footprint footprint{VehicleParams{4.0, 2.0, 2.5, 0.6}};

    const auto corners = footprint.corners(Pose2D{0.0, 0.0, 0.5 * kPi});

    expectPointNear(corners[0], -1.0, 2.0);
    expectPointNear(corners[1], 1.0, 2.0);
    expectPointNear(corners[2], 1.0, -2.0);
    expectPointNear(corners[3], -1.0, -2.0);
}

TEST(FootprintTest, UsesPoseTranslation) {
    const Footprint footprint{VehicleParams{4.0, 2.0, 2.5, 0.6}};

    const auto corners = footprint.corners(Pose2D{10.0, -3.0, 0.0});

    expectPointNear(corners[0], 12.0, -2.0);
    expectPointNear(corners[1], 12.0, -4.0);
    expectPointNear(corners[2], 8.0, -4.0);
    expectPointNear(corners[3], 8.0, -2.0);
}

}  // namespace
