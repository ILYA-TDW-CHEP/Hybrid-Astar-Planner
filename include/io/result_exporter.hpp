#pragma once

#include <filesystem>
#include <vector>

#include "common/pose.hpp"
#include "map/occupancy_grid.hpp"

namespace hybrid_astar::io {

struct SearchStats {
    bool success{false};
    int expanded_nodes{0};
    int generated_nodes{0};
    double planning_time_ms{0.0};
    double path_length{0.0};
};

struct ExpandedNode {
    int grid_x{0};
    int grid_y{0};
    double world_x{0.0};
    double world_y{0.0};
    double g_cost{0.0};
};

struct PlanningResult {
    std::vector<common::Pose2D> path;
    std::vector<ExpandedNode> expanded;
    SearchStats stats{};
};

class ResultExporter {
public:
    static bool exportPathCsv(
        const std::filesystem::path& path,
        const std::vector<common::Pose2D>& poses
    );

    static bool exportResultJson(
        const std::filesystem::path& path,
        const PlanningResult& result
    );

    static bool exportMapJson(
        const std::filesystem::path& path,
        const map::OccupancyGrid& grid
    );
};

}  // namespace hybrid_astar::io
