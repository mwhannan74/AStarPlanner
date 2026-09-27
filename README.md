# AStarPlanner

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

AStarPlanner is a header-only C++17 library for path planning on two-dimensional
occupancy grids. It combines polygon-based map construction, safety inflation,
A*, Dijkstra, and weighted-A* search, automatic line-of-sight path
simplification, and detailed OpenCV-based visualization.

## Why this planner exists

There are already many open-source A* implementations, and this project is not
trying to replace them with a novel search algorithm. Its A* implementation is
fast and efficient enough for useful, real planning workloads, but the larger
goal is to provide a planner that is easy to understand, inspect, and debug.
It is not intended to claim the fastest implementation or the broadest set of
advanced planning features.

The project emphasizes:

- simple, readable planner and occupancy-grid code
- annotated demos that double as step-by-step tutorials
- live visualization of the search frontier and expanded cells
- clear visualization of the environment, occupancy grid, original path, and
  simplified path
- an integrated occupancy-grid workflow built from world-coordinate polygons
- an optional operation area for defining and clipping a permitted workspace
- practical diagnostics and profiling tools for understanding planner behavior

This makes the repository useful both as a working planner and as a transparent
reference for learning, experimenting, and diagnosing grid-based planning.

<p align="center">
  <img src="images/a_star_planner_demo.png"
       alt="AStarPlanner polygon environment and planned path"
       width="800">
</p>

## Key capabilities

- Convex polygon obstacles in world coordinates
- Arbitrary world-coordinate obstacle points rasterized into occupied cells
- Occupied-union fusion of aligned polygon and perception grids
- Optional convex keep-in operation area with obstacle clipping
- OpenCV-backed binary occupancy grids and planning subgrids
- Safety inflation in world units, including conservative grid-boundary handling
- Four- or eight-connected A*, Dijkstra, and weighted A*
- Configurable diagonal corner cutting and deterministic tie-breaking
- Original connected grid paths plus automatic collision-checked simplified paths
- Path cost, search-work, detailed expansion, and simplification diagnostics
- Live search animation and reusable occupancy-grid rendering
- Deterministic demos, benchmarks, and isolated profiling workloads

## Contents

- [Quick start](#quick-start)
- [Using the planner](#using-the-planner)
- [Environment and occupancy-grid model](#environment-and-occupancy-grid-model)
- [Search configuration](#search-configuration)
- [Visualization and debugging](#visualization-and-debugging)
- [Benchmarks and profiling](#benchmarks-and-profiling)
- [Build reference](#build-reference)
- [Important limitations and design boundaries](#important-limitations-and-design-boundaries)

## Quick start

### Requirements

- CMake 3.16 or newer
- A C++17 compiler
- [Eigen](https://eigen.tuxfamily.org/) for world-coordinate geometry
- OpenCV 4.5 or newer for occupancy storage and rasterization
- [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV) when building
  visualization support or the demos

Dependency paths are CMake cache variables with local defaults in
`cmake/local_paths.cmake`:

- `ASTAR_PLANNER_EIGEN3_INCLUDE_DIR`
- `ASTAR_PLANNER_MPOCV_SOURCE_DIR`
- `ASTAR_PLANNER_OPENCV_DIR`

Override them during configuration when your dependencies are installed
elsewhere.

### Configure and build

The following commands build all targets on Windows with a multi-configuration
generator:

```powershell
cmake -S . -B build -DMATPLOTOPENCV_BUILD_DEMO=OFF -DMATPLOTOPENCV_BUILD_DOCS=OFF
cmake --build build --config Release
```

These examples reflect the project's current Windows development setup. With a
single-configuration generator, select `-DCMAKE_BUILD_TYPE=Release` while
configuring and omit `--config Release` from build and CTest commands.

To build the planner and tests without MatPlotOpenCV or the visualization demos:

```powershell
cmake -S . -B build -DASTAR_PLANNER_ENABLE_VISUALIZATION=OFF -DASTAR_PLANNER_BUILD_DEMO=OFF
cmake --build build --config Release
```

### Run the demos

```powershell
.\build\Release\a_star_planner_demo.exe
.\build\Release\operation_area_demo.exe
```

Add `--debug` to animate the search state while planning:

```powershell
.\build\Release\a_star_planner_demo.exe --debug
.\build\Release\operation_area_demo.exe --debug
```

Each demo also accepts an optional output-image filename for the world-coordinate
figure:

```powershell
.\build\Release\a_star_planner_demo.exe planner_result.png
```

### Run the tests

```powershell
ctest --test-dir build -C Release --output-on-failure
```

The main planner test executable can also be run directly:

```powershell
.\build\Release\a_star_planner_tests.exe
```

## Using the planner

A normal request follows nine steps:

1. Define convex obstacle polygons and, optionally, a convex operation area.
2. Select the world-coordinate `planningBounds` and grid resolution.
3. Create `planningGeometry` covering those bounds.
4. Construct `PolygonEnvironment` to validate, normalize, and clip the geometry.
5. Rasterize the polygons directly into `planningGeometry`.
6. Inflate the resulting occupancy grid by the required safety radius.
7. Convert the world start and goal with the planning grid's `worldToCell()`.
8. Call `AStarGridPlanner::plan()` and check `GridPlanResult::succeeded()`.
9. Convert either returned path to world cell centers with `gridPathToWorld()`.

### Planning bounds and data inclusion

The application selects `planningBounds`; the library does not infer a planning
horizon from obstacles, terminals, a robot, or sensor range. Common choices are
a fixed world region, operation-area bounds, start and goal with padding, or an
application-defined local horizon.

```cpp
const WorldBounds planningBounds{
    minimumWorldPoint,
    maximumWorldPoint
};

const GridGeometry planningGeometry =
    GridGeometry::covering(planningBounds, resolution);
```

`covering()` preserves `planningBounds.minimum` as the grid origin and rounds
width and height up to whole cells. Consequently,
`planningGeometry.worldBounds().maximum` may extend by less than one cell beyond
the requested maximum. The resulting geometry is authoritative: data inside it
can be rasterized and searched, and data outside it cannot be used by the
planner.

The occupancy preprocessing pipeline is organized around that one shared
geometry. Polygon rasterization, point rasterization, and occupied-union fusion
are implemented:

```text
Application selects planningBounds and resolution
                        |
            Create planningGeometry
                        |
       +----------------+----------------+
       |                                 |
Rasterize polygons                Rasterize points
       |                                 |
  polygonGrid                     perceptionGrid
       +----------------+----------------+
                        |
                    fusedGrid
                        |
              Inflate once for safety
                        |
                  planningGrid
                        |
                     Planner
```

`GridGeometry::alignedCovering()` is an advanced alternative that expands both
ends of requested bounds onto a stable cell lattice. It is useful when changing
requests must retain the same world-aligned cell boundaries, but it is not
required for the normal planning-boundary workflow.

The following example selects explicit planning bounds, builds an operation
area inside them, inflates the polygon occupancy, and plans across the resulting
grid:

```cpp
#include "a_star_grid_planner.hpp"
#include "a_star_planner.hpp"
#include "occupancy_grid.hpp"

#include <iostream>
#include <vector>

int main()
{
    using namespace astar;

    const Polygon operationArea{
        Point2(0.0, 0.0), Point2(10.0, 0.0),
        Point2(10.0, 10.0), Point2(0.0, 10.0)
    };
    const std::vector<Polygon> obstacles{
        {
            Point2(4.0, 2.0), Point2(6.0, 2.0),
            Point2(6.0, 8.0), Point2(4.0, 8.0)
        }
    };

    const PolygonEnvironment environment(operationArea, obstacles);
    const WorldBounds planningBounds{
        Point2(0.0, 0.0),
        Point2(10.0, 10.0)
    };
    const GridGeometry planningGeometry =
        GridGeometry::covering(planningBounds, 0.5);
    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        planningGeometry, environment);
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        polygonGrid, 0.5);

    const auto start = planningGrid.geometry().worldToCell(Point2(1.0, 1.0));
    const auto goal = planningGrid.geometry().worldToCell(Point2(9.0, 9.0));
    if (!start || !goal)
        return 1;

    AStarOptions options; // A* and eight-connected movement by default.
    const AStarGridPlanner planner;
    const GridPlanResult result = planner.plan(
        planningGrid, *start, *goal, options);
    if (!result.succeeded())
    {
        std::cerr << gridPlanStatusName(result.status) << '\n';
        return 1;
    }

    const std::vector<Point2> worldPath =
        gridPathToWorld(planningGrid, result.path);
    const std::vector<Point2> simplifiedWorldPath =
        gridPathToWorld(planningGrid, result.simplifiedPath);

    std::cout << "Original path: " << worldPath.size() << " cells\n"
              << "Simplified path: " << simplifiedWorldPath.size()
              << " waypoints\n"
              << "Path cost: " << result.diagnostics.pathCost << '\n';
}
```

The annotated source files in `demo/` show the complete workflow, including
timing and visualization.

### Planning results

Every successful `GridPlanResult` retains two path representations:

- `path` is the original graph-search result. Consecutive cells are connected
  grid neighbors.
- `simplifiedPath` contains line-of-sight waypoints derived automatically after
  search. Consecutive waypoints may be many cells apart.

Simplified segments are collision-checked against the same occupancy grid passed
to `plan()`. When that grid is safety-inflated, the simplified path is checked
against the inflated obstacles as well. Exact cell-corner crossings follow
`AStarOptions::preventDiagonalCornerCutting`.

`GridPlanDiagnostics::pathCost` describes the original graph-search path.
`pathSimplificationMilliseconds` reports the post-processing time. The public
`simplifyGridPath()` function provides the same operation for other ordered grid
paths.

Failed plans return both paths empty and one of these statuses:

- `StartOutsideGrid` or `GoalOutsideGrid`
- `StartOccupied` or `GoalOccupied`
- `NoPath`

Use `gridPlanStatusName()` for a readable status description.

### Using AStarPlanner from another CMake project

When vendoring the source with `add_subdirectory()`, disable the repository's
standalone executables unless the parent project needs them. Link the header-only
core target to inherit its include paths and required dependencies:

```cmake
set(ASTAR_PLANNER_BUILD_DEMO OFF CACHE BOOL "" FORCE)
set(ASTAR_PLANNER_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(ASTAR_PLANNER_ENABLE_VISUALIZATION OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/AStarPlanner EXCLUDE_FROM_ALL)

target_link_libraries(my_application
    PRIVATE
        AStarPlanner::astar_planner
)
```

Keep `ASTAR_PLANNER_ENABLE_VISUALIZATION` enabled when using the optional
renderers, then link the specific visualization target the application needs:

```cmake
target_link_libraries(my_application
    PRIVATE
        AStarPlanner::astar_occupancy_grid_visualization
)
```

`AStarPlanner::astar_planner_visualization` adds the MatPlotOpenCV environment
plotter and transitively includes occupancy-grid visualization support.

## Environment and occupancy-grid model

### Polygon environment

`PolygonEnvironment` validates world-coordinate geometry before rasterization.
Obstacles and the optional operation area must be convex simple polygons.

- Clockwise polygons are normalized to counter-clockwise order.
- Repeated closing points, consecutive duplicates, and redundant collinear
  vertices are removed.
- Non-finite, self-intersecting, degenerate, and concave polygons are rejected.
- Inputs with fewer than three vertices are ignored.
- Obstacles are clipped to the operation area when one is present.
- Empty, point-only, and line-only clipping results are discarded.

The environment exposes normalized input obstacles, effective clipped obstacles,
and changed clipped overlays for visualization.

### Coordinates and cells

- World coordinates are Cartesian `Point2` values.
- `GridCell{column, row}` is also Cartesian. Cell `(0, 0)` is at the lower left,
  and rows increase with world `y`.
- OpenCV image rows increase downward. `GridGeometry` performs that conversion;
  callers should not invert rows themselves.
- A grid's world origin is the lower-left outer corner of cell `(0, 0)`.
- Grid world bounds are half-open; points on the maximum boundary are outside.
- `gridPathToWorld()` returns cell centers, so its terminal points may differ
  from the exact world-coordinate query positions.

### Rasterization

The preferred entry point accepts the caller-selected geometry and complete
validated environment:

```cpp
const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
    planningGeometry, environment);
```

This selects the correct initialization and painting behavior automatically.
Without an operation area, rasterization starts with a free grid and paints
obstacles occupied. With an operation area, it starts occupied, paints the
operation area free, and then paints effective obstacles occupied.

World-coordinate obstacle points use the same caller-selected geometry:

```cpp
const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
    planningGeometry, obstaclePoints);
```

Each finite point inside the geometry marks its containing cell occupied.
Duplicate points are harmless; nonfinite points and points outside the geometry
are ignored. Every call creates a new grid, so point rasterization does not
accumulate perception history.

Rasterization follows OpenCV `fillConvexPoly` pixel coverage. It does not mark
every cell touched by the continuous polygon geometry. Applications requiring a
safety margin should plan on an independently inflated grid.

### Occupancy fusion

Polygon and perception grids created from the same `planningGeometry` can be
combined with occupied-wins semantics:

```cpp
const OccupancyGrid fusedGrid = OccupancyGridFusion::occupiedUnion(
    polygonGrid, perceptionGrid);
```

A fused cell is occupied when either input cell is occupied. Inputs must have
identical dimensions, origin, resolution, and master-cell offset. Fusion does
not modify either input and returns independently owned storage. Apply safety
inflation once after all occupancy sources have been fused.

### Safety inflation

`OccupancyGridInflator::inflate()` accepts a nonnegative radius in world units,
rounds it up to a whole-cell radius, and applies a circular dilation kernel. It
returns independent storage and leaves the source grid unchanged.

Space beyond every edge of the supplied grid is treated as occupied. Inflation
therefore creates clearance around obstacles and a band along the grid boundary.
This is appropriate when the edge represents unknown space, the operation-area
boundary, or another hard planning limit.

### Existing maps and optional subgrids

The default workflow selects the required planning bounds first and rasterizes
directly into that geometry. This avoids allocating, rasterizing, fusing, and
inflating a larger grid than the current request needs.

`OccupancyGrid::subgrid()` remains useful when an application already owns a
larger persistent map and wants to search only part of it. A subgrid limits the
search domain; it is not merely a display crop. A route may exist in the source
grid but require cells outside the selected region. In that case the subgrid
plan correctly returns `NoPath`.

The order of inflation and cropping changes the meaning of an ROI. Inflate the
ROI directly when its edges are hard planning boundaries:

```cpp
const OccupancyGrid planningRegion = masterGrid.subgrid(planningBounds);
const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
    planningRegion, safetyRadius);
```

Inflate the master first when the ROI is only a search restriction inside a
larger known map:

```cpp
const OccupancyGrid inflatedMaster = OccupancyGridInflator::inflate(
    masterGrid, safetyRadius);
const OccupancyGrid planningGrid = inflatedMaster.subgrid(planningBounds);
```

Subgrids retain their cumulative master-cell offset and may use a shared OpenCV
view or an independent copy. `OccupancyGrid` is read-only through its grid API.
`imageView()` provides a zero-copy view that must be treated as read-only;
`cloneImage()` returns writable independent storage.

## Search configuration

`AStarGridPlanner` uses eight-connected A* by default. Orthogonal moves cost
`1`, diagonal moves cost `sqrt(2)`, and diagonal corner cutting is prevented.
Four-connected movement is available through `AStarOptions`.

### Search modes

All modes use the same graph search and priority equation:

```text
priority(n) = pathCostFromStart(n) + effectiveWeight * baseHeuristic(n, goal)
```

| Algorithm | Option | Effective weight | Behavior |
|---|---|---:|---|
| Dijkstra | `GridSearchAlgorithm::Dijkstra` | `0` | Optimal, with no goal-directed heuristic. |
| A* | `GridSearchAlgorithm::AStar` | `1` | Default, using the connectivity-matched admissible heuristic. |
| Weighted A* | `GridSearchAlgorithm::WeightedAStar` | `heuristicWeight` | May reduce work but can return a non-optimal path. |

```cpp
AStarOptions options;
options.algorithm = GridSearchAlgorithm::Dijkstra;

options.algorithm = GridSearchAlgorithm::WeightedAStar;
options.heuristicWeight = 2.0; // Must be finite and at least 1.
```

The heuristic weight changes only open-set priority. It does not change movement
costs, collision checking, reconstruction, or the reported path cost.

### Tie-breaking

Tie-breaking orders nodes whose search priorities are exactly equal. It can
change search effort and which equal-cost path is selected, but it does not
change normal A* optimality.

| Policy | Behavior |
|---|---|
| `StraightLineThenLargerG` | Prefers the direct start-to-goal line, then greater `g`. This is the default. |
| `LargerGThenStraightLine` | Prefers greater `g`, then straight-line deviation. |
| `LargerGOnly` | Prefers greater `g` without calculating a geometric key. |

```cpp
options.tieBreakPolicy = AStarTieBreakPolicy::LargerGOnly;
```

There is no universally fastest tie-break policy. Benchmark representative maps
before changing an application's default.

### Diagnostics

Every result includes lightweight diagnostics:

- `pathCost`
- `expandedNodes`
- `generatedNodes`
- `peakOpenSetSize`
- `pathSimplificationMilliseconds`

Set `AStarOptions::collectDetailedDiagnostics` when investigating search
behavior. Detailed diagnostics add a per-cell expansion-count image and counters
for unique expansions, repeated expansions, stale entries, and post-expansion
cost improvements. Collection is disabled by default because it adds storage and
work during search.

Grid costs use floating point because diagonal moves cost `sqrt(2)`. Relaxation
uses a small scale-aware tolerance so differently ordered sums of equivalent
steps do not cause spurious decrease-key operations.

## Visualization and debugging

Visualization is a primary feature of this project rather than an afterthought.
The two demos show the polygon environment, rasterized occupancy grid, start and
goal, original connected path, and simplified line-of-sight path.

- `a_star_planner_demo` demonstrates a complete master grid without an operation
  area. A fixed random seed keeps its obstacle field repeatable.
- `operation_area_demo` demonstrates a keep-in area, obstacle clipping, a master
  map, and a caller-selected planning ROI.

The operation-area example produces the following type of environment:

<p align="center">
  <img src="images/a_star_planner_operation_area_demo.png"
       alt="AStarPlanner operation area and obstacle clipping demo"
       width="800">
</p>

### Live search debugging

Pass a `GridSearchDebugCallback` to `plan()` to receive a synchronous view of the
reusable search-state image after accepted expansions and at completion:

```cpp
GridSearchDebugCallback debugCallback =
    [&](const cv::Mat1b& state)
    {
        showGridSearchState(
            planningGrid, state, start, goal, {}, "Search Debug");
        cv::waitKey(10);
    };

const GridPlanResult result = planner.plan(
    planningGrid, start, goal, options, debugCallback);
```

| Color | Meaning |
|---|---|
| White | Unseen traversable cell |
| Yellow | Open/frontier cell |
| Orange | Cell currently being expanded |
| Light blue | Expanded cell |
| Blue | Final reconstructed path |
| Black | Occupied cell |
| Green / red | Start / goal markers |

The callback blocks planning until it returns. Its matrix is valid only during
the callback and is modified afterward. Clone it only when a frame must be
retained or transferred to another thread. Callbacks may skip frames to reduce
rendering work.

### Representative demo output

Timings vary by machine and build configuration. The path-focused output keeps
the final result ahead of lower-level diagnostics:

```text
A* planning completed in 0.215 ms
  Path cost: 109.397
  Original path: 93 cells
  Simplified path: 9 waypoints
  Simplification time: 0.011 ms

Search diagnostics
  Tie-break: straight-line then larger g
  Expanded: 1197
  Generated: 2272
  Peak open set: 148

Grid preparation
  Effective obstacles: 25
  Master grid: 69 x 75 cells at 1.000 world units/cell
  Planning grid: full map, 1.000 world units of safety inflation
  Rasterization: 0.036 ms
  Inflation: 0.048 ms
```

## Benchmarks and profiling

Build and run the deterministic non-visual benchmark in Release mode:

```powershell
cmake --build build --config Release --target a_star_planner_benchmark
.\build\Release\a_star_planner_benchmark.exe
```

It measures environment preparation, rasterization, ROI creation, inflation,
terminal conversion, planning, path simplification, path conversion, and request
totals. It also records search-work counters, raw path cells, simplified
waypoints, and path cost in a timestamped CSV.

Useful options include:

- `--quick` for a `100 x 100` smoke run
- `--stress` to add a `2000 x 2000` case
- `--csv <file>` to choose the report path
- `--tie-break <policy>` to compare A* tie-breaking policies

### Isolated CPU profiling

`a_star_planner_profile` repeatedly runs A* on a deterministic `1000 x 1000`
alternating-barrier grid. Grid construction and one warm-up occur before the
timed workload. The output includes total and mean planning and simplification
time.

```powershell
.\build\Release\a_star_planner_profile.exe --iterations 50
```

### Isolated search visualization

`a_star_search_debug` compares categorical search state with a fixed-scale
expansion-count heat map. It supports A*, Dijkstra, and weighted A*:

```powershell
.\build\Release\a_star_search_debug.exe --algorithm astar
.\build\Release\a_star_search_debug.exe --algorithm dijkstra
.\build\Release\a_star_search_debug.exe --algorithm weighted --weight 1.5
```

Use `--no-animation` to show only the completed search, `--no-display` to save
without opening a window, `--output <file>` to select the PNG path, or
`--frame-stride <count>` to control animation sampling.

## Build reference

### CMake options

| Option | Default | Purpose |
|---|---:|---|
| `BUILD_TESTING` | `ON` | Builds the test executables and registers CTest tests. |
| `ASTAR_PLANNER_BUILD_DEMO` | `ON` | Builds the two tutorial demos. |
| `ASTAR_PLANNER_ENABLE_VISUALIZATION` | `ON` | Enables MatPlotOpenCV and OpenCV visualization support. |
| `ASTAR_PLANNER_BUILD_BENCHMARKS` | `ON` | Builds benchmark and profiling executables. |

The demos require visualization. Disable the demo, visualization, and benchmark
options when building only the core planner and tests.

### CMake targets

- `AStarPlanner::astar_planner` — header-only environment, grid, rasterizer, and
  planner API
- `AStarPlanner::astar_occupancy_grid_visualization` — OpenCV grid rendering
- `AStarPlanner::astar_planner_visualization` — polygon and planner plotting
- `a_star_planner_tests` — deterministic core tests
- `occupancy_grid_visualization_tests` — rendering tests
- `a_star_planner_demo` — full-grid tutorial
- `operation_area_demo` — operation-area and ROI tutorial
- `a_star_planner_benchmark` — repeatable performance suite and CSV output
- `a_star_planner_profile` — isolated repeated CPU workload
- `a_star_search_debug` — visual comparison of search modes

### Public headers

- `a_star_planner.hpp` — polygon environment model
- `occupancy_grid.hpp` — grid geometry, occupancy storage, polygon and point
  rasterization, occupied-union fusion, and inflation
- `a_star_grid_planner.hpp` — search, planning results, and path simplification
- `occupancy_grid_visualization.hpp` — OpenCV grid and search-state rendering
- `a_star_planner_visualization.hpp` — MatPlotOpenCV environment plotting

The environment-only header does not include OpenCV. Occupancy-grid and search
features do.

## Important limitations and design boundaries

- Obstacles and operation areas must be convex simple polygons.
- Polygon holes are not supported.
- Overlapping obstacles are rasterized correctly, but overlap relationships are
  not separately validated or reported.
- Occupancy is binary; graded traversal or clearance costs are not implemented.
- Rasterization uses OpenCV pixel coverage rather than exact continuous cell
  intersection.
- Search terminals and returned grid paths represent cell centers.
- Simplification creates straight collision-checked segments, not curves or
  vehicle-dynamics-constrained trajectories.
- A planning ROI is a hard search limit. There is no automatic expanding-ROI
  retry policy.
- The planner assumes a static occupancy grid during each request.
- Occupancy fusion currently requires already aligned grids; it does not
  resample, reproject, or reconcile different resolutions.

## Project background

The repository began as a copy of VisGraphPlanner. The former visibility-graph
implementation has been removed; the active project is an occupancy-grid
planner. `AStarPlanner` remains as a deprecated compatibility alias for the
environment model. New code should use `PolygonEnvironment` and
`AStarGridPlanner` directly.

## License

AStarPlanner is licensed under the Apache License 2.0. See `LICENSE` for the
full license text.

Copyright 2025-2026 Michael Hannan.
