#pragma once

#include "common/pose.hpp"
#include "map/occupancy_grid.hpp"
#include "vehicle/footprint.hpp"

namespace hybrid_astar::planner {

class CollisionChecker {
public:
    CollisionChecker(
        const map::OccupancyGrid& grid,
        const vehicle::Footprint& footprint
    );

    bool isCollision(const common::Pose2D& pose) const;

private:
    const map::OccupancyGrid& grid_;
    const vehicle::Footprint& footprint_;
};


}  // namespace hybrid_astar::planner
