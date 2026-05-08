cmake_minimum_required(VERSION 3.20)

project(hybrid_astar_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hybrid_astar_core
    src/occupancy_grid.cpp
    src/footprint.cpp
    src/vehicle_model.cpp
    src/collision_checker.cpp
)
target_include_directories(hybrid_astar_core PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(hybrid_astar_app
    src/main.cpp
)
target_link_libraries(hybrid_astar_app PRIVATE hybrid_astar_core)

include(CTest)

if(BUILD_TESTING)
    include(FetchContent)

    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(hybrid_astar_tests
        tests/test_angle_utils.cpp
        tests/test_collision_checker.cpp
        tests/test_footprint.cpp
        tests/test_math_utils.cpp
        tests/test_occupancy_grid.cpp
        tests/test_pose.cpp
        tests/test_vehicle_model.cpp
    )
    target_link_libraries(hybrid_astar_tests PRIVATE
        hybrid_astar_core
        GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(hybrid_astar_tests)
endif()
