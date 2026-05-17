#include "common/pose.hpp"
#include "io/result_exporter.hpp"
#include "map/occupancy_grid.hpp"
#include "planner/collision_checker.hpp"
#include "planner/heuristic.hpp"
#include "planner/hybrid_astar.hpp"
#include "planner/motion_primitive.hpp"
#include "vehicle/footprint.hpp"
#include "vehicle/vehicle_model.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace {

void setDisk(hybrid_astar::map::OccupancyGrid& grid, int cx, int cy, int radius) {
    for (int y = cy - radius; y <= cy + radius; ++y) {
        for (int x = cx - radius; x <= cx + radius; ++x) {
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                grid.setOccupied(x, y, true);
            }
        }
    }
}

void addLine(
    hybrid_astar::map::OccupancyGrid& grid,
    int x0,
    int y0,
    int x1,
    int y1,
    int thickness = 1
) {
    const int dx = x1 - x0;
    const int dy = y1 - y0;
    const int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps == 0) {
        setDisk(grid, x0, y0, thickness);
        return;
    }

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const int x = static_cast<int>(std::round(x0 + t * dx));
        const int y = static_cast<int>(std::round(y0 + t * dy));
        setDisk(grid, x, y, thickness);
    }
}

void addRect(
    hybrid_astar::map::OccupancyGrid& grid,
    int x0,
    int y0,
    int x1,
    int y1
) {
    for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y) {
        for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
            grid.setOccupied(x, y, true);
        }
    }
}

void addSketchMapObstacles(hybrid_astar::map::OccupancyGrid& grid) {
    // Left-side sketch walls around the green start area.
    //addLine(grid, 0, 5, 6, 5, 1);
    addLine(grid, 50, 10, 64, 10, 1);
    addLine(grid, 6, 5, 6, 10, 1);
    addLine(grid, 6, 10, 0, 11, 1);

    addLine(grid, 5, 24, 13, 23, 1);
    addLine(grid, 13, 23, 20, 18, 1);
    addLine(grid, 20, 18, 34, 18, 1);
    addLine(grid, 34, 18, 32, 14, 1);
    addLine(grid, 32, 14, 27, 10, 1);
    addLine(grid, 27, 10, 27, 6, 1);
    addLine(grid, 27, 6, 37, 6, 1);
    addLine(grid, 37, 6, 38, 11, 1);

    // Main central barrier: this forces a long turn around the upper opening.
    addLine(grid, 48, 0, 48, 25, 1);
    addLine(grid, 49, 0, 49, 25, 1);
    addLine(grid, 36, 25, 46, 27, 1);
    addLine(grid, 34, 22, 36, 25, 1);
    addLine(grid, 34, 13, 34, 22, 1);
    addLine(grid, 34, 13, 31, 10, 1);
    addLine(grid, 31, 10, 27, 10, 1);

    // Right-hand blocks from the drawing.
    addRect(grid, 57, 23, 63, 36);
    addRect(grid, 57, 9, 60, 19);
    addLine(grid, 49, 24, 51, 24, 1);
    addLine(grid, 57, 24, 64, 24, 1);
    addLine(grid, 63, 4, 63, 24, 1);
    addLine(grid, 65, 4, 65, 24, 1);

    // Upper decorative wall fragments, kept away from the main corridor.
    addLine(grid, 22, 39, 28, 36, 1);
    addLine(grid, 28, 36, 36, 36, 1);
    addLine(grid, 45, 33, 50, 33, 1);
    addLine(grid, 45, 33, 45, 39, 1);
    addLine(grid, 50, 33, 50, 39, 1);
}

}  // namespace

int main() {
    using hybrid_astar::common::Pose2D;
    using hybrid_astar::io::ResultExporter;
    using hybrid_astar::map::OccupancyGrid;
    using hybrid_astar::planner::CollisionChecker;
    using hybrid_astar::planner::Heuristic;
    using hybrid_astar::planner::HybridAStar;
    using hybrid_astar::planner::HybridAStarConfig;
    using hybrid_astar::planner::MotionPrimitiveConfig;
    using hybrid_astar::planner::MotionPrimitiveGenerator;
    using hybrid_astar::vehicle::Footprint;
    using hybrid_astar::vehicle::VehicleModel;
    using hybrid_astar::vehicle::VehicleParams;

    OccupancyGrid grid{70, 40, 0.5};
    addSketchMapObstacles(grid);

    const VehicleParams vehicle_params{1.0, 0.6, 0.8, 0.6};
    const VehicleModel vehicle_model{vehicle_params};
    const Footprint footprint{vehicle_params};
    const CollisionChecker collision_checker{grid, footprint};
    const MotionPrimitiveGenerator motion_primitives{
        vehicle_model,
        MotionPrimitiveConfig{0.5, 7, true, 2.0, 0.1}
    };
    const Heuristic heuristic;
    const HybridAStar planner{
        grid,
        collision_checker,
        motion_primitives,
        heuristic,
        HybridAStarConfig{150000, 1.0, 3.14}
    };

    const Pose2D start{6.8, 6.8, -1.57079632679};
    const Pose2D goal{26.8, 1.9, 0.0};

    const auto started_at = std::chrono::steady_clock::now();
    auto result = planner.plan(start, goal);
    const auto finished_at = std::chrono::steady_clock::now();
    result.stats.planning_time_ms = std::chrono::duration<double, std::milli>(
        finished_at - started_at
    ).count();

    std::filesystem::create_directories("results");
    const bool csv_ok = ResultExporter::exportPathCsv("results/path.csv", result.path);
    const bool json_ok = ResultExporter::exportResultJson("results/result.json", result);
    const bool map_ok = ResultExporter::exportMapJson("results/map.json", grid);

    std::cout << "Hybrid A* demo finished\n";
    std::cout << "  success: " << (result.stats.success ? "true" : "false") << '\n';
    std::cout << "  path poses: " << result.path.size() << '\n';
    std::cout << "  expanded nodes: " << result.stats.expanded_nodes << '\n';
    std::cout << "  generated nodes: " << result.stats.generated_nodes << '\n';
    std::cout << "  path length: " << result.stats.path_length << '\n';
    std::cout << "  planning time ms: " << result.stats.planning_time_ms << '\n';
    std::cout << "  wrote results/path.csv: " << (csv_ok ? "yes" : "no") << '\n';
    std::cout << "  wrote results/result.json: " << (json_ok ? "yes" : "no") << '\n';
    std::cout << "  wrote results/map.json: " << (map_ok ? "yes" : "no") << '\n';

    return result.stats.success && csv_ok && json_ok && map_ok ? 0 : 1;
}
