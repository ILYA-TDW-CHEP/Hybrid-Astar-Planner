#include <iostream>

#include "common/pose.hpp"

int main() {
    const hybrid_astar::common::Pose2D start;
    std::cout << "Hybrid A* planner bootstrap is ready. "
              << "Start pose: (" << start.x << ", " << start.y << ", " << start.theta << ")\n";
    return 0;
}
