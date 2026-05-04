# Hybrid A* Planner

Portfolio project: a C++ Hybrid A* planner for autonomous vehicle path planning in a 2D occupancy grid, with Python visualization, tests, scenarios, and benchmarks.

The project is intentionally built in small engineering steps. This first stage only bootstraps the workspace: repository layout, CMake targets, a minimal C++ type, and one unit test.

## Roadmap

1. Bootstrap workspace, CMake, tests, and scenario placeholders.
2. Add geometry utilities, angle helpers, and occupancy grid basics.
3. Add vehicle parameters, kinematic bicycle model, and footprint geometry.
4. Add motion primitives and simple collision checking.
5. Add a first heuristic and path/result export format.
6. Implement baseline Hybrid A* search.
7. Add Python plotting and animation scripts.
8. Add benchmark scenarios and search statistics.
9. Explore smoothing and coarse-to-fine refinement.

## Repository Structure

```text
.
├── configs/        # Planner and benchmark configuration files
├── docs/           # Design notes and algorithm documentation
├── include/        # Public C++ headers
├── src/            # C++ implementation and application entrypoint
├── tests/          # Unit tests
├── scenarios/      # Input scenario JSON files
├── scripts/        # Python visualization and benchmark utilities
├── results/        # Generated planner outputs
└── third_party/    # Optional vendored dependencies or notes
```

## Build

The first CMake configure may download GoogleTest through `FetchContent`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
./build/hybrid_astar_app
```

## Current Status

- C++17 project scaffold is in place.
- `hybrid_astar_app` builds as a minimal executable.
- `hybrid_astar_tests` contains the first GoogleTest unit tests.
- Scenario JSON files are placeholders and are not loaded by C++ yet.
