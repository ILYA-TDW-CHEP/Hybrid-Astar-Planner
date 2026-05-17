#pragma once

#include "common/pose.hpp"

namespace hybrid_astar::planner {

struct HeuristicConfig {
    double distance_weight{1.0};
    double angle_weight{0.1};
};

class Heuristic {
public:
    explicit Heuristic(HeuristicConfig config = {});

    const HeuristicConfig& config() const;

    double estimate(const common::Pose2D& from, const common::Pose2D& goal) const;

private:
    HeuristicConfig config_;
};

}  // namespace hybrid_astar::planner
