#include "map/occupancy_grid.hpp"

#include <gtest/gtest.h>

namespace {

using hybrid_astar::map::OccupancyGrid;

TEST(OccupancyGridTest, StoresDimensionsAndResolution) {
    const OccupancyGrid grid{10, 5, 0.25};

    EXPECT_EQ(grid.width(), 10);
    EXPECT_EQ(grid.height(), 5);
    EXPECT_DOUBLE_EQ(grid.resolution(), 0.25);
}

TEST(OccupancyGridTest, TreatsDefaultCellsAsFree) {
    const OccupancyGrid grid{4, 3, 0.5};

    EXPECT_FALSE(grid.isOccupied(0, 0));
    EXPECT_FALSE(grid.isOccupied(3, 2));
}

TEST(OccupancyGridTest, ChecksBounds) {
    const OccupancyGrid grid{4, 3, 0.5};

    EXPECT_TRUE(grid.isInside(0, 0));
    EXPECT_TRUE(grid.isInside(3, 2));

    EXPECT_FALSE(grid.isInside(-1, 0));
    EXPECT_FALSE(grid.isInside(0, -1));
    EXPECT_FALSE(grid.isInside(4, 0));
    EXPECT_FALSE(grid.isInside(0, 3));
}

TEST(OccupancyGridTest, CanSetAndClearOccupiedCells) {
    OccupancyGrid grid{4, 3, 0.5};

    grid.setOccupied(2, 1, true);
    EXPECT_TRUE(grid.isOccupied(2, 1));

    grid.setOccupied(2, 1, false);
    EXPECT_FALSE(grid.isOccupied(2, 1));
}

TEST(OccupancyGridTest, TreatsOutsideAsOccupied) {
    const OccupancyGrid grid{4, 3, 0.5};

    EXPECT_TRUE(grid.isOccupied(-1, 0));
    EXPECT_TRUE(grid.isOccupied(0, -1));
    EXPECT_TRUE(grid.isOccupied(4, 0));
    EXPECT_TRUE(grid.isOccupied(0, 3));
}

TEST(OccupancyGridTest, WorldToGridUsesResolution) {
    const OccupancyGrid grid{10, 10, 0.5};

    const auto index = grid.worldToGrid(1.25, 2.75);

    EXPECT_EQ(index.x, 2);
    EXPECT_EQ(index.y, 5);
}

TEST(OccupancyGridTest, WorldToGridFloorsCoordinates) {
    const OccupancyGrid grid{10, 10, 0.5};

    EXPECT_EQ(grid.worldToGrid(0.0, 0.0).x, 0);
    EXPECT_EQ(grid.worldToGrid(0.49, 0.49).x, 0);
    EXPECT_EQ(grid.worldToGrid(0.50, 0.50).x, 1);
    EXPECT_EQ(grid.worldToGrid(1.25, 1.25).x, 2);
}

TEST(OccupancyGridTest, GridToWorldReturnsCellCenter) {
    const OccupancyGrid grid{10, 10, 0.5};

    EXPECT_DOUBLE_EQ(grid.gridToWorldX(0), 0.25);
    EXPECT_DOUBLE_EQ(grid.gridToWorldY(0), 0.25);
    EXPECT_DOUBLE_EQ(grid.gridToWorldX(1), 0.75);
    EXPECT_DOUBLE_EQ(grid.gridToWorldY(2), 1.25);
}

TEST(OccupancyGridTest, WorldToGridCanReturnOutsideIndices) {
    const OccupancyGrid grid{10, 10, 0.5};

    const auto negative = grid.worldToGrid(-0.1, -0.1);
    const auto outside = grid.worldToGrid(5.0, 5.0);

    EXPECT_EQ(negative.x, -1);
    EXPECT_EQ(negative.y, -1);
    EXPECT_EQ(outside.x, 10);
    EXPECT_EQ(outside.y, 10);
    EXPECT_FALSE(grid.isInside(negative.x, negative.y));
    EXPECT_FALSE(grid.isInside(outside.x, outside.y));
}

}  // namespace
