#include "planner/hybrid_astar.hpp"

#include "common/angle_utils.hpp"
#include "common/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace hybrid_astar::planner {
namespace {

constexpr int kHeadingBins = 72;

struct StateKey {
    int x{0};
    int y{0};
    int heading{0};

    bool operator==(const StateKey& other) const {
        return x == other.x && y == other.y && heading == other.heading;
    }
};

struct StateKeyHash {
    std::size_t operator()(const StateKey& key) const {
        std::size_t seed = 0;
        seed ^= std::hash<int>{}(key.x) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<int>{}(key.heading) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct QueueEntry {
    StateKey key{};
    double f_cost{0.0};
    double g_cost{0.0};
};

struct QueueEntryCompare {
    bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
        return lhs.f_cost > rhs.f_cost;
    }
};

struct NodeRecord {
    common::Pose2D pose{};
    StateKey parent{};
    bool has_parent{false};
    double g_cost{std::numeric_limits<double>::infinity()};
};

int headingBin(double theta) {
    const double normalized = common::normalizeAngle(theta);
    const double shifted = normalized + common::kPi;
    int bin = static_cast<int>(std::floor(shifted / common::kTwoPi * kHeadingBins));
    if (bin >= kHeadingBins) {
        bin = kHeadingBins - 1;
    }
    return std::max(0, bin);
}

StateKey makeKey(const map::OccupancyGrid& grid, const common::Pose2D& pose) {
    const auto index = grid.worldToGrid(pose.x, pose.y);
    return StateKey{index.x, index.y, headingBin(pose.theta)};
}

bool reachedGoal(
    const common::Pose2D& pose,
    const common::Pose2D& goal,
    const HybridAStarConfig& config
) {
    return common::distance2D(pose.x, pose.y, goal.x, goal.y) <= config.goal_position_tolerance &&
           common::absAngularDistance(pose.theta, goal.theta) <= config.goal_heading_tolerance;
}

double computePathLength(const std::vector<common::Pose2D>& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += common::distance2D(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
    }
    return length;
}

std::vector<common::Pose2D> reconstructPath(
    const std::unordered_map<StateKey, NodeRecord, StateKeyHash>& records,
    StateKey current
) {
    std::vector<common::Pose2D> path;

    while (true) {
        const auto it = records.find(current);
        if (it == records.end()) {
            break;
        }

        path.push_back(it->second.pose);
        if (!it->second.has_parent) {
            break;
        }
        current = it->second.parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace

HybridAStar::HybridAStar(
    const map::OccupancyGrid& grid,
    const CollisionChecker& collision_checker,
    const MotionPrimitiveGenerator& motion_primitives,
    const Heuristic& heuristic,
    HybridAStarConfig config
) : grid_(grid),
    collision_checker_(collision_checker),
    motion_primitives_(motion_primitives),
    heuristic_(heuristic),
    config_(config) {}

const HybridAStarConfig& HybridAStar::config() const {
    return config_;
}

io::PlanningResult HybridAStar::plan(
    const common::Pose2D& start,
    const common::Pose2D& goal
) const {
    io::PlanningResult result;

    if (collision_checker_.isCollision(start) || collision_checker_.isCollision(goal)) {
        result.stats.success = false;
        return result;
    }

    if (reachedGoal(start, goal, config_)) {
        result.path = {start};
        result.stats.success = true;
        return result;
    }

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> open;
    std::unordered_map<StateKey, NodeRecord, StateKeyHash> records;

    const StateKey start_key = makeKey(grid_, start);
    records[start_key] = NodeRecord{start, StateKey{}, false, 0.0};
    open.push(QueueEntry{start_key, heuristic_.estimate(start, goal), 0.0});

    int iterations = 0;

    while (!open.empty() && iterations < config_.max_iterations) {
        const QueueEntry current_entry = open.top();
        open.pop();

        auto current_it = records.find(current_entry.key);
        if (current_it == records.end()) {
            continue;
        }

        if (current_entry.g_cost > current_it->second.g_cost) {
            continue;
        }

        ++iterations;
        result.stats.expanded_nodes = iterations;

        const common::Pose2D current_pose = current_it->second.pose;
        result.expanded.push_back(io::ExpandedNode{
            current_entry.key.x,
            current_entry.key.y,
            grid_.gridToWorldX(current_entry.key.x),
            grid_.gridToWorldY(current_entry.key.y),
            current_it->second.g_cost
        });

        if (reachedGoal(current_pose, goal, config_)) {
            result.path = reconstructPath(records, current_entry.key);
            result.stats.success = true;
            result.stats.path_length = computePathLength(result.path);
            return result;
        }

        for (const auto& primitive : motion_primitives_.generate(current_pose)) {
            const auto& next_pose = primitive.end_pose;
            if (collision_checker_.isCollision(next_pose)) {
                continue;
            }

            const StateKey next_key = makeKey(grid_, next_pose);
            if (!grid_.isInside(next_key.x, next_key.y)) {
                continue;
            }

            ++result.stats.generated_nodes;

            const double tentative_g = current_it->second.g_cost + primitive.cost;
            auto next_it = records.find(next_key);
            if (next_it != records.end() && tentative_g >= next_it->second.g_cost) {
                continue;
            }

            records[next_key] = NodeRecord{next_pose, current_entry.key, true, tentative_g};
            const double f_cost = tentative_g + heuristic_.estimate(next_pose, goal);
            open.push(QueueEntry{next_key, f_cost, tentative_g});
        }
    }

    result.stats.success = false;
    result.stats.expanded_nodes = iterations;
    return result;
}

}  // namespace hybrid_astar::planner
