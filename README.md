# VisGraphPlanner

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

A C++ visibility-graph planner for static 2-D polygonal environments.

VisGraphPlanner constructs and uses a visibility graph for path planning around polygonal obstacles. The core planner is a header-only CMake target. Optional visualization support and the demonstration executables use [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV).

## How the Planner Works

VisGraphPlanner finds point-to-point routes through a known, static 2-D environment whose obstacles are represented as polygons. It is a geometric path planner rather than a motion controller: its output is a sequence of positions to follow, without vehicle dynamics, turning constraints, or automatic safety clearance. Applications planning for a robot with nonzero size should expand the obstacles by the required footprint and safety margin before building the graph.

An optional convex operation area can constrain planning to a keep-in zone. Start and goal points must be strictly inside this area. Obstacles are clipped to the operation area, and obstacles with no positive-area intersection are ignored.

Obstacle vertices created on, or already lying on, the operation-area boundary
are retained in the effective obstacle geometry for collision checks but are
excluded from the visibility graph. All graph vertices therefore remain
strictly inside the operation area.

The planner treats each eligible corner of the normalized, effective obstacles as a graph vertex. When an operation area is present, corners on its boundary are excluded. The planner connects pairs of vertices when the straight segment between them does not cross an obstacle, while rejecting diagonals that pass through the interior of the same polygon. Each edge is weighted by its Euclidean length. For a planning query, the start and goal are added to the graph and connected to the vertices they can see; Dijkstra's algorithm then returns the lowest-distance route available in the graph.

This approach is simple, deterministic, and effective for relatively small static maps, and it naturally produces direct paths that bend around obstacle corners. Its main tradeoff is scalability: the basic all-pairs visibility build is O(N³) in the number of obstacle vertices and can produce a dense graph. The implementation validates its polygon inputs, does not support polygon holes or changing obstacles, and uses floating-point geometric tests that may require care with nearly coincident geometry.

## Assumptions and Limitations

- Obstacles must be represented as convex, simple polygons. Clockwise input is normalized to counter-clockwise order.
- Obstacle inputs with fewer than three vertices are ignored with a warning. An operation area must contain at least three distinct, non-collinear vertices after normalization.
- Repeated closing vertices, consecutive duplicates, and redundant collinear boundary vertices are removed.
- Non-finite, self-intersecting, degenerate, and concave polygons are rejected.
- Polygon holes are not supported.
- Start and goal coordinates must be finite. Both points must be outside every obstacle and must not lie on an obstacle boundary.
- When an operation area is supplied, it must be convex and simple, and start and goal must be strictly inside it.
- Obstacles crossing an operation-area boundary are clipped to the portion inside the area.
- The obstacle visibility graph is constructed using a naive O(N³) algorithm, where N is the number of obstacle vertices.
- Each call to `injectQueryPts()` replaces the previous start and goal while reusing the obstacle-only graph.

## Run the Demo

From the repository root on Windows with a multi-configuration generator such as Visual Studio:

```powershell
.\build\Release\main_demo.exe
```

To run the deterministic operation-area and obstacle-clipping demo:

```powershell
.\build\Release\operation_area_demo.exe
```

The operation-area demo uses four alternating boundary-crossing barriers and
three interior obstacles to create a constrained route. It also includes an
entirely outside obstacle that is discarded and a diamond whose original
vertices are outside but whose edges cross an operation-area corner. Original
obstacles are drawn in blue, while their positive-area clipped results are
overlaid in magenta. The operation-area boundary is a thin dark-green outline.
Usable visibility-graph edges are drawn as thin gray lines with explicit vertex
markers, and the final path is drawn over them in orange with a marker at each
selected waypoint. The goal remains red.

**Example Terminal Output:**
```
buildBasic() took 1 ms
Query setup and injectQueryPts() took 0 ms
shortestPath() took 0 ms
Graph has 915 edges
Path found with 6 waypoints
```

<p align="center">
  <img
    src="images/visibility_graph_demo.png"
    alt="Visibility graph demo showing the planned path through polygonal obstacles"
    width="700">
</p>

<p align="center">
  <img
    src="images/visibility_graph_demo_op_area.png"
    alt="Visibility graph demo with an operation area"
    width="700">
</p>

Values vary between runs because the demo randomizes the obstacle positions and start and goal points.

With a single-configuration generator, the executable is normally `build/main_demo` (`build/main_demo.exe` on Windows). The exact location depends on the selected CMake generator and build configuration.

## Platform Status

This project has been built and tested only on Windows. The code and CMake configuration are intended to be portable to Linux, but Linux has not yet been tested.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- [Eigen](https://eigen.tuxfamily.org/) for the core planner
- [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV) for visualization and the demos
- OpenCV 4.5 or newer with the `core`, `highgui`, and `imgproc` components, as required by MatPlotOpenCV
- Doxygen if MatPlotOpenCV documentation is enabled; pass `-DMATPLOTOPENCV_BUILD_DOCS=OFF` if it is not needed

The visualization header includes `figure.h`. That file is provided by MatPlotOpenCV; it is not part of this repository. The core `visibility_graph.hpp` header does not depend on MatPlotOpenCV or OpenCV.

## Configure Dependencies

The default local paths are defined in [`cmake/local_paths.cmake`](cmake/local_paths.cmake):

- `VISGRAPH_EIGEN3_INCLUDE_DIR` points to the Eigen include directory.
- `VISGRAPH_MPOCV_SOURCE_DIR` points to the MatPlotOpenCV source directory containing its `CMakeLists.txt`.

The default MatPlotOpenCV path expects both repositories to have the same parent directory:

```text
parent-directory/
|-- MatPlotOpenCV/
`-- VisGraphPlanner/
```

Edit `cmake/local_paths.cmake` for your machine or override its cached values when configuring:

```bash
cmake -S . -B build -DVISGRAPH_EIGEN3_INCLUDE_DIR=/path/to/eigen -DVISGRAPH_MPOCV_SOURCE_DIR=/path/to/MatPlotOpenCV -DMATPLOTOPENCV_OPENCV_DIR=/path/to/opencv/cmake
```

`MATPLOTOPENCV_OPENCV_DIR` must identify the directory containing `OpenCVConfig.cmake`.

## Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build --config Release
```

To use only the core planner without the visualization dependencies or demo:

```bash
cmake -S . -B build -DVISGRAPH_ENABLE_VISUALIZATION=OFF -DVISGRAPH_BUILD_DEMO=OFF
cmake --build build --config Release
```

MatPlotOpenCV enables its own demo and documentation targets by default. Add `-DMATPLOTOPENCV_BUILD_DEMO=OFF -DMATPLOTOPENCV_BUILD_DOCS=OFF` to the configure command if you do not want to build them.

## Tests

Tests are built by default. From the repository root, run every test with:

```powershell
.\build\Release\visgraph_tests.exe
```

From the `build` directory, use:

```powershell
.\Release\visgraph_tests.exe
```

The executable runs every test in order and prints its elapsed time and `PASS` or `FAIL` status. Failed tests also print the reason.

CTest can also run the same executable:

```bash
ctest --test-dir build -C Release --output-on-failure
```

Use `-DBUILD_TESTING=OFF` when configuring to omit the test executable.

## Targets

The project provides the following CMake targets:

- `VisGraphPlanner::visgraph` — header-only core visibility-graph planner
- `VisGraphPlanner::visgraph_visualization` — optional visualization support through MatPlotOpenCV
- `main_demo` — demonstration executable, built when `VISGRAPH_BUILD_DEMO=ON`
- `operation_area_demo` — deterministic operation-area and clipping demo, built when `VISGRAPH_BUILD_DEMO=ON`

## Basic Usage

```cpp
#include "visibility_graph.hpp"

#include <iostream>
#include <vector>

int main()
{
    using namespace vg;

    std::vector<Polygon> obstacles{
        {
            Point2(0.0, 0.0),
            Point2(5.0, 0.0),
            Point2(5.0, 5.0),
            Point2(0.0, 5.0)
        }
    };

    VisibilityGraph graph(obstacles);
    graph.buildBasic();

    const Point2 start(-2.0, 2.0);
    const Point2 goal(7.0, 2.0);

    const auto [startId, goalId] = graph.injectQueryPts(start, goal);
    const auto path = graph.shortestPath(startId, goalId);

    std::cout << "Path contains " << path.size() << " points\n";
}
```

To constrain the same planner to a convex operation area, pass it before the
obstacle list:

```cpp
Polygon operationArea{
    Point2(-10.0, -10.0),
    Point2(10.0, -10.0),
    Point2(10.0, 10.0),
    Point2(-10.0, 10.0)
};

VisibilityGraph constrainedGraph(operationArea, obstacles);
```

## License

VisGraphPlanner is licensed under the Apache License 2.0.

See [`LICENSE`](LICENSE) for the full license text.

Copyright 2025-2026 Michael Hannan.
