#pragma once

#include <cmath>
#include <vector>

namespace hybrid_astar::map {

struct GridIndex {
    int x{0};
    int y{0};
};

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, double resolution);

    int width() const;
    int height() const;
    double resolution() const;

    bool isInside(int x, int y) const;
    bool isOccupied(int x, int y) const;
    void setOccupied(int x, int y, bool occupied);

    GridIndex worldToGrid(double world_x, double world_y) const;
    double gridToWorldX(int grid_x) const;
    double gridToWorldY(int grid_y) const;

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<bool> cells_;
};

}
