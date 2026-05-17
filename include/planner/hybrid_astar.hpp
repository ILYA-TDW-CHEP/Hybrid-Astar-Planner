#pragma once

#include "common/pose.hpp"
#include "io/result_exporter.hpp"
#include "map/occupancy_grid.hpp"
#include "planner/collision_checker.hpp"
#include "planner/heuristic.hpp"
#include "planner/motion_primitive.hpp"

namespace hybrid_astar::planner {

struct HybridAStarConfig {
    int max_iterations{10000};
    double goal_position_tolerance{0.5};
    double goal_heading_tolerance{0.25};
};

class HybridAStar {
public:
    HybridAStar(
        const map::OccupancyGrid& grid,
        const CollisionChecker& collision_checker,
        const MotionPrimitiveGenerator& motion_primitives,
        const Heuristic& heuristic,
        HybridAStarConfig config = {}
    );

    const HybridAStarConfig& config() const;

    io::PlanningResult plan(
        const common::Pose2D& start,
        const common::Pose2D& goal
    ) const;

private:
    const map::OccupancyGrid& grid_;
    const CollisionChecker& collision_checker_;
    const MotionPrimitiveGenerator& motion_primitives_;
    const Heuristic& heuristic_;
    HybridAStarConfig config_;
};

}  // namespace hybrid_astar::planner
