#include "map/occupancy_grid.hpp"

namespace hybrid_astar::map {

    OccupancyGrid::OccupancyGrid(int width, int height, double resolution) : width_(width), height_(height), resolution_(resolution) {
        cells_ = std::vector<bool>(width_ * height_, false);
    }

    int OccupancyGrid::width() const {
        return width_;
    }

    int OccupancyGrid::height() const {
        return height_;
    }

    double OccupancyGrid::resolution() const {
        return resolution_;
    }

    bool OccupancyGrid::isInside(int x, int y) const {
        return (0 <= x && x < width_) && (0 <= y && y < height_);
    }

    bool OccupancyGrid::isOccupied(int x, int y) const {
        return !isInside(x, y) || cells_[x + y * width_];
    }

    void OccupancyGrid::setOccupied(int x, int y, bool occupied) {
        if (!isInside(x,y)) {
            return;
        }
        cells_[x + y * width_] = occupied;
    }

    GridIndex OccupancyGrid::worldToGrid(double world_x, double world_y) const {
        return GridIndex{
            static_cast<int>(std::floor(world_x / resolution_)),
            static_cast<int>(std::floor(world_y / resolution_))
        };
    }

    double OccupancyGrid::gridToWorldX(int grid_x) const {
        return (grid_x + 0.5) * resolution_;
    }

    double OccupancyGrid::gridToWorldY(int grid_y) const {
        return (grid_y + 0.5) * resolution_;
    }
}