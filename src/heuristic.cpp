#include "planner/heuristic.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"

namespace hybrid_astar::planner {

Heuristic::Heuristic(HeuristicConfig config) : config_(config) {}

const HeuristicConfig& Heuristic::config() const {
    return config_;
}

double Heuristic::estimate(const common::Pose2D& from, const common::Pose2D& goal) const {
    const double distance = common::distance2D(from.x, from.y, goal.x, goal.y);
    const double angle_distance = common::absAngularDistance(from.theta, goal.theta);

    return config_.distance_weight * distance + config_.angle_weight * angle_distance;
}

}  // namespace hybrid_astar::planner
