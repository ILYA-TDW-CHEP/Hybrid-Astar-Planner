#include "io/result_exporter.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace {

using hybrid_astar::common::Pose2D;
using hybrid_astar::io::ExpandedNode;
using hybrid_astar::io::PlanningResult;
using hybrid_astar::io::ResultExporter;
using hybrid_astar::io::SearchStats;
using hybrid_astar::map::OccupancyGrid;

std::string readFile(const std::filesystem::path& path) {
    std::ifstream input{path};
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

TEST(ResultExporterTest, ExportsPathCsv) {
    const auto path = std::filesystem::temp_directory_path() / "hybrid_astar_path_test.csv";
    const std::vector<Pose2D> poses{
        Pose2D{1.0, 2.0, 0.1},
        Pose2D{3.0, 4.0, 0.2},
    };

    ASSERT_TRUE(ResultExporter::exportPathCsv(path, poses));

    const auto contents = readFile(path);
    EXPECT_NE(contents.find("x,y,theta\n"), std::string::npos);
    EXPECT_NE(contents.find("1,2,0.10000000000000001"), std::string::npos);
    EXPECT_NE(contents.find("3,4,0.20000000000000001"), std::string::npos);

    std::filesystem::remove(path);
}

TEST(ResultExporterTest, ExportsResultJson) {
    const auto path = std::filesystem::temp_directory_path() / "hybrid_astar_result_test.json";
    const PlanningResult result{
        .path = {
            Pose2D{1.0, 2.0, 0.1},
            Pose2D{3.0, 4.0, 0.2},
        },
        .expanded = {
            ExpandedNode{2, 4, 1.25, 2.25, 3.5},
        },
        .stats = SearchStats{
            .success = true,
            .expanded_nodes = 10,
            .generated_nodes = 25,
            .planning_time_ms = 3.5,
            .path_length = 7.25,
        },
    };

    ASSERT_TRUE(ResultExporter::exportResultJson(path, result));

    const auto contents = readFile(path);
    EXPECT_NE(contents.find("\"success\": true"), std::string::npos);
    EXPECT_NE(contents.find("\"expanded_nodes\": 10"), std::string::npos);
    EXPECT_NE(contents.find("\"generated_nodes\": 25"), std::string::npos);
    EXPECT_NE(contents.find("\"planning_time_ms\": 3.5"), std::string::npos);
    EXPECT_NE(contents.find("\"path_length\": 7.25"), std::string::npos);
    EXPECT_NE(contents.find("\"expanded\""), std::string::npos);
    EXPECT_NE(contents.find("\"grid_x\": 2"), std::string::npos);
    EXPECT_NE(contents.find("\"g_cost\": 3.5"), std::string::npos);
    EXPECT_NE(contents.find("\"path\""), std::string::npos);
    EXPECT_NE(contents.find("\"x\": 1"), std::string::npos);
    EXPECT_NE(contents.find("\"theta\": 0.10000000000000001"), std::string::npos);

    std::filesystem::remove(path);
}

TEST(ResultExporterTest, ExportsMapJson) {
    const auto path = std::filesystem::temp_directory_path() / "hybrid_astar_map_test.json";
    OccupancyGrid grid{4, 3, 0.5};
    grid.setOccupied(1, 2, true);

    ASSERT_TRUE(ResultExporter::exportMapJson(path, grid));

    const auto contents = readFile(path);
    EXPECT_NE(contents.find("\"resolution\": 0.5"), std::string::npos);
    EXPECT_NE(contents.find("\"width\": 4"), std::string::npos);
    EXPECT_NE(contents.find("\"height\": 3"), std::string::npos);
    EXPECT_NE(contents.find("\"x\": 1"), std::string::npos);
    EXPECT_NE(contents.find("\"y\": 2"), std::string::npos);
    EXPECT_NE(contents.find("\"world_x\": 0.75"), std::string::npos);
    EXPECT_NE(contents.find("\"world_y\": 1.25"), std::string::npos);

    std::filesystem::remove(path);
}

TEST(ResultExporterTest, ReturnsFalseForInvalidPath) {
    const auto path = std::filesystem::path{"/definitely/not/a/real/directory/result.csv"};

    EXPECT_FALSE(ResultExporter::exportPathCsv(path, {}));
    EXPECT_FALSE(ResultExporter::exportResultJson(path, PlanningResult{}));
    EXPECT_FALSE(ResultExporter::exportMapJson(path, OccupancyGrid{1, 1, 1.0}));
}

}  // namespace
