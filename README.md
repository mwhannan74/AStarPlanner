# AStarPlanner

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

AStarPlanner is being developed as a header-only C++17 library for A* path
planning on two-dimensional occupancy grids.

## Current conversion checkpoint

The repository began as a copy of VisGraphPlanner. The active code currently
provides the reusable environment model while the grid and A* implementation
are being designed. It supports:

- Eigen world-coordinate points and polygon obstacles
- convex polygon normalization and validation
- an optional convex operation area
- clipping obstacles to the operation area
- separate original, clipped, and effective obstacle views
- optional environment visualization through MatPlotOpenCV

The former visibility-graph construction, query-vertex injection, adjacency
model, and Dijkstra search are disabled. This checkpoint does not calculate or
display a path.

## Assumptions and validation

- Obstacles and the optional operation area must be convex simple polygons.
- Clockwise polygons are normalized to counter-clockwise order.
- Repeated closing vertices, consecutive duplicates, and redundant collinear
  boundary vertices are removed.
- Non-finite, self-intersecting, degenerate, and concave polygons are rejected.
- Obstacle inputs with fewer than three vertices are ignored with a warning.
- Obstacles are clipped to the operation area when one is configured.
- Empty, point-only, and line-only clipping results are discarded.

Polygon holes and overlapping-obstacle validation are not supported.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- [Eigen](https://eigen.tuxfamily.org/) for the core environment model
- [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV) for visualization and demos
- OpenCV as required by MatPlotOpenCV

The core `a_star_planner.hpp` header does not depend on MatPlotOpenCV or OpenCV.

## Configure and build

Default local dependency paths are defined in `cmake/local_paths.cmake`:

- `ASTAR_PLANNER_EIGEN3_INCLUDE_DIR`
- `ASTAR_PLANNER_MPOCV_SOURCE_DIR`

Configure and build all targets on Windows with a multi-configuration generator:

```powershell
cmake -S . -B build -DMATPLOTOPENCV_BUILD_DEMO=OFF -DMATPLOTOPENCV_BUILD_DOCS=OFF
cmake --build build --config Release
```

To build only the core environment model and tests:

```powershell
cmake -S . -B build -DASTAR_PLANNER_ENABLE_VISUALIZATION=OFF -DASTAR_PLANNER_BUILD_DEMO=OFF
cmake --build build --config Release
```

## Tests

Run the environment-model tests directly:

```powershell
.\build\Release\a_star_planner_tests.exe
```

Or use CTest:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

## Demos

The two demos visualize the retained environment state. They intentionally do
not construct a grid or calculate a path yet.

```powershell
.\build\Release\a_star_planner_demo.exe
.\build\Release\operation_area_demo.exe
```

The first demo shows a randomized field of polygon obstacles. The second shows
the operation-area boundary, original obstacles, and clipped obstacle overlays.
Both show start and goal markers for continuity with the future planner display.
Pass an optional image filename as the first argument to save the rendered
figure before its window is displayed.

<p align="center">
  <img src="images/a_star_planner_demo.png"
       alt="AStarPlanner polygon environment demo"
       width="600">
</p>

<p align="center">
  <img src="images/a_star_planner_operation_area_demo.png"
       alt="AStarPlanner operation-area and obstacle-clipping demo"
       width="600">
</p>

## CMake targets

- `AStarPlanner::astar_planner` — header-only environment model
- `AStarPlanner::astar_planner_visualization` — optional MatPlotOpenCV plotting
- `a_star_planner_tests` — deterministic environment tests
- `a_star_planner_demo` — unconstrained-environment demo
- `operation_area_demo` — operation-area and clipping demo

## Basic usage

```cpp
#include "a_star_planner.hpp"

#include <vector>

int main()
{
    using namespace astar;

    std::vector<Polygon> obstacles{
        {
            Point2(0.0, 0.0),
            Point2(5.0, 0.0),
            Point2(5.0, 5.0),
            Point2(0.0, 5.0)
        }
    };

    AStarPlanner planner(obstacles);
    return planner.obstacles().empty() ? 1 : 0;
}
```

## License

AStarPlanner is licensed under the Apache License 2.0. See `LICENSE` for the
full license text.

Copyright 2025-2026 Michael Hannan.
