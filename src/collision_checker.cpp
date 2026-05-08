#include "planner/collision_checker.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"

#include <cmath>
#include <cfloat>

namespace hybrid_astar::planner {

struct Aabb {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

Aabb computeAabb(const std::array<vehicle::Point2D, 4>& corners) {
    double min_x = DBL_MAX, min_y = DBL_MAX;
    double max_x = DBL_MIN, max_y = DBL_MIN;
    for (auto& corner : corners) {
        min_x = std::min(min_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_x = std::max(max_x, corner.x);
        max_y = std::max(max_y, corner.y);
    }
    return Aabb{min_x, max_x, min_y, max_y};
    
}

std::array<vehicle::Point2D, 4> cellCorners(
    const map::OccupancyGrid& grid,
    int grid_x,
    int grid_y
) {
    double resolution = grid.resolution();
    return {vehicle::Point2D{grid_x * resolution, (grid_y + 1) * resolution},
            vehicle::Point2D{(grid_x + 1) * resolution, (grid_y + 1) * resolution},
            vehicle::Point2D{(grid_x + 1) * resolution, grid_y * resolution},
            vehicle::Point2D{grid_x * resolution, grid_y * resolution}
    };
}

vehicle::Point2D edgeNormal(
    const vehicle::Point2D& a,
    const vehicle::Point2D& b
) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    return vehicle::Point2D{-dy, dx};
}

void projectPolygon(
    const std::array<vehicle::Point2D, 4>& polygon,
    const vehicle::Point2D& axis,
    double& min_proj,
    double& max_proj
) {
    min_proj = DBL_MAX;
    max_proj = -DBL_MAX;

    for (const auto& p : polygon) {
        double proj = p.x * axis.x + p.y * axis.y;

        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }
}

bool overlapOnAxis(
    const std::array<vehicle::Point2D, 4>& a,
    const std::array<vehicle::Point2D, 4>& b,
    const vehicle::Point2D& axis
) {
    double a_min, a_max;
    double b_min, b_max;

    projectPolygon(a, axis, a_min, a_max);
    projectPolygon(b, axis, b_min, b_max);

    return !(a_max < b_min || b_max < a_min);
}

bool rectanglesIntersectSat(
    const std::array<vehicle::Point2D, 4>& a,
    const std::array<vehicle::Point2D, 4>& b
) {
    for (int i = 0; i < 4; ++i) {
        vehicle::Point2D axis =
            edgeNormal(a[i], a[(i + 1) % 4]);

        if (!overlapOnAxis(a, b, axis)) {
            return false;
        }
    }

    for (int i = 0; i < 4; ++i) {
        vehicle::Point2D axis =
            edgeNormal(b[i], b[(i + 1) % 4]);

        if (!overlapOnAxis(a, b, axis)) {
            return false;
        }
    }

    return true;
}


CollisionChecker::CollisionChecker(
    const map::OccupancyGrid& grid,
    const vehicle::Footprint& footprint
) : grid_(grid), footprint_(footprint) {}


bool CollisionChecker::isCollision(const common::Pose2D& pose) const {
    auto vehicle_corners = footprint_.corners(pose);

    Aabb vehicle_aabb = computeAabb(vehicle_corners);

    double resolution = grid_.resolution();

    if (vehicle_aabb.min_x < 0 || vehicle_aabb.min_y < 0 ||
        vehicle_aabb.max_x > resolution * grid_.width() ||
        vehicle_aabb.max_y > resolution * grid_.height()) {
        return true;
    }

    int min_x = static_cast<int>(
        std::floor(vehicle_aabb.min_x / resolution)
    );

    int max_x = static_cast<int>(
        std::floor(vehicle_aabb.max_x / resolution)
    );

    int min_y = static_cast<int>(
        std::floor(vehicle_aabb.min_y / resolution)
    );

    int max_y = static_cast<int>(
        std::floor(vehicle_aabb.max_y / resolution)
    );

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {

            if (!grid_.isInside(x, y)) {
                continue;
            }

            if (!grid_.isOccupied(x, y)) {
                continue;
            }

            auto obstacle = cellCorners(grid_, x, y);

            if (rectanglesIntersectSat(
                    vehicle_corners,
                    obstacle
                )) {
                return true;
            }
        }
    }

    return false;
}

}  // namespace hybrid_astar::planner