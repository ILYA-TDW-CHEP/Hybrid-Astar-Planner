#include "io/result_exporter.hpp"

#include <fstream>
#include <iomanip>

namespace hybrid_astar::io {

bool ResultExporter::exportPathCsv(
    const std::filesystem::path& path,
    const std::vector<common::Pose2D>& poses
) {
    std::ofstream output{path};
    if (!output.is_open()) {
        return false;
    }

    output << std::setprecision(17);
    output << "x,y,theta\n";
    for (const auto& pose : poses) {
        output << pose.x << ',' << pose.y << ',' << pose.theta << '\n';
    }

    return output.good();
}

bool ResultExporter::exportResultJson(
    const std::filesystem::path& path,
    const PlanningResult& result
) {
    std::ofstream output{path};
    if (!output.is_open()) {
        return false;
    }

    output << std::setprecision(17);
    output << "{\n";
    output << "  \"stats\": {\n";
    output << "    \"success\": " << (result.stats.success ? "true" : "false") << ",\n";
    output << "    \"expanded_nodes\": " << result.stats.expanded_nodes << ",\n";
    output << "    \"generated_nodes\": " << result.stats.generated_nodes << ",\n";
    output << "    \"planning_time_ms\": " << result.stats.planning_time_ms << ",\n";
    output << "    \"path_length\": " << result.stats.path_length << "\n";
    output << "  },\n";
    output << "  \"expanded\": [\n";

    for (std::size_t i = 0; i < result.expanded.size(); ++i) {
        const auto& node = result.expanded[i];
        output << "    {\"grid_x\": " << node.grid_x
               << ", \"grid_y\": " << node.grid_y
               << ", \"world_x\": " << node.world_x
               << ", \"world_y\": " << node.world_y
               << ", \"g_cost\": " << node.g_cost << "}";
        if (i + 1 < result.expanded.size()) {
            output << ',';
        }
        output << '\n';
    }

    output << "  ],\n";
    output << "  \"path\": [\n";

    for (std::size_t i = 0; i < result.path.size(); ++i) {
        const auto& pose = result.path[i];
        output << "    {\"x\": " << pose.x
               << ", \"y\": " << pose.y
               << ", \"theta\": " << pose.theta << "}";
        if (i + 1 < result.path.size()) {
            output << ',';
        }
        output << '\n';
    }

    output << "  ]\n";
    output << "}\n";

    return output.good();
}

bool ResultExporter::exportMapJson(
    const std::filesystem::path& path,
    const map::OccupancyGrid& grid
) {
    std::ofstream output{path};
    if (!output.is_open()) {
        return false;
    }

    output << std::setprecision(17);
    output << "{\n";
    output << "  \"resolution\": " << grid.resolution() << ",\n";
    output << "  \"width\": " << grid.width() << ",\n";
    output << "  \"height\": " << grid.height() << ",\n";
    output << "  \"occupied\": [\n";

    bool first = true;
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            if (!grid.isOccupied(x, y)) {
                continue;
            }

            if (!first) {
                output << ",\n";
            }
            first = false;

            output << "    {\"x\": " << x
                   << ", \"y\": " << y
                   << ", \"world_x\": " << grid.gridToWorldX(x)
                   << ", \"world_y\": " << grid.gridToWorldY(y) << "}";
        }
    }

    output << '\n';
    output << "  ]\n";
    output << "}\n";

    return output.good();
}

}  // namespace hybrid_astar::io
