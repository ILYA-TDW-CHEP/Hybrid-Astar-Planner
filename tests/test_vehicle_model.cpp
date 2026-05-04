
#include "vehicle/vehicle_model.hpp"

#include "common/math_utils.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::kPi;
using hybrid_astar::common::Pose2D;
using hybrid_astar::vehicle::VehicleModel;
using hybrid_astar::vehicle::VehicleParams;

TEST(VehicleModelTest, StoresVehicleParams) {
    const VehicleParams params{4.8, 2.1, 2.9, 0.55};

    const VehicleModel model{params};

    EXPECT_DOUBLE_EQ(model.params().length, 4.8);
    EXPECT_DOUBLE_EQ(model.params().width, 2.1);
    EXPECT_DOUBLE_EQ(model.params().wheelbase, 2.9);
    EXPECT_DOUBLE_EQ(model.params().max_steering_angle, 0.55);
}

TEST(VehicleModelTest, MovesStraightForward) {
    const VehicleModel model{VehicleParams{}};
    const Pose2D next = model.step(Pose2D{1.0, 2.0, 0.0}, 0.0, 3.0);

    EXPECT_NEAR(next.x, 4.0, 1e-12);
    EXPECT_NEAR(next.y, 2.0, 1e-12);
    EXPECT_NEAR(next.theta, 0.0, 1e-12);
}

TEST(VehicleModelTest, MovesStraightBackward) {
    const VehicleModel model{VehicleParams{}};
    const Pose2D next = model.step(Pose2D{1.0, 2.0, 0.0}, 0.0, -3.0);

    EXPECT_NEAR(next.x, -2.0, 1e-12);
    EXPECT_NEAR(next.y, 2.0, 1e-12);
    EXPECT_NEAR(next.theta, 0.0, 1e-12);
}

TEST(VehicleModelTest, MovesAlongCurrentHeading) {
    const VehicleModel model{VehicleParams{}};
    const Pose2D next = model.step(Pose2D{1.0, 2.0, 0.5 * kPi}, 0.0, 3.0);

    EXPECT_NEAR(next.x, 1.0, 1e-12);
    EXPECT_NEAR(next.y, 5.0, 1e-12);
    EXPECT_NEAR(next.theta, 0.5 * kPi, 1e-12);
}

TEST(VehicleModelTest, ChangesHeadingWhenSteering) {
    const VehicleParams params{4.5, 2.0, 2.0, 0.6};
    const VehicleModel model{params};
    const Pose2D next = model.step(Pose2D{0.0, 0.0, 0.0}, 0.25, 2.0);

    EXPECT_NEAR(next.theta, std::tan(0.25), 1e-12);
}

TEST(VehicleModelTest, ClampsSteeringToMaxAngle) {
    const VehicleParams params{4.5, 2.0, 2.0, 0.3};
    const VehicleModel model{params};

    const Pose2D positive = model.step(Pose2D{0.0, 0.0, 0.0}, 1.0, 2.0);
    const Pose2D negative = model.step(Pose2D{0.0, 0.0, 0.0}, -1.0, 2.0);

    EXPECT_NEAR(positive.theta, std::tan(0.3), 1e-12);
    EXPECT_NEAR(negative.theta, std::tan(-0.3), 1e-12);
}

TEST(VehicleModelTest, NormalizesOutputHeading) {
    const VehicleParams params{4.5, 2.0, 1.0, 0.6};
    const VehicleModel model{params};
    const Pose2D next = model.step(Pose2D{0.0, 0.0, kPi - 0.1}, 0.6, 1.0);

    EXPECT_GE(next.theta, -kPi);
    EXPECT_LT(next.theta, kPi);
}

}  // namespace
