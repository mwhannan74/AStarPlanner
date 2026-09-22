# AStarPlanner

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

AStarPlanner is being developed as a header-only C++17 library for A* path
planning on two-dimensional occupancy grids.

## Current development checkpoint

The repository began as a copy of VisGraphPlanner. The active code provides a
reusable environment model, occupancy grid, and working A* planner. It supports:

- Eigen world-coordinate points and polygon obstacles
- convex polygon normalization and validation
- an optional convex operation area
- clipping obstacles to the operation area
- separate normalized-input, clipped, and effective obstacle views
- an OpenCV-backed binary occupancy grid
- world, Cartesian-grid, and OpenCV-image coordinate conversion
- explicit cell-sized, world-bounds-sized, and polygon-sized grids
- optional alignment to a stable world-coordinate lattice
- OpenCV-accelerated operation-area and obstacle rasterization
- OpenCV-accelerated binary safety inflation in world units
- master maps with shared or independently copied planning subgrids
- eight-connected A* search with optional four-connected movement
- Dijkstra search through the same planning interface
- weighted A* with a configurable heuristic weight
- unit orthogonal and `sqrt(2)` diagonal costs with matching heuristics
- optional diagonal corner cutting, disabled by default
- path cost and search-work diagnostics for every planning request
- conversion of ROI-local grid paths to world-coordinate cell centers
- explicit planning outcomes for invalid terminals and unreachable goals
- optional environment visualization through MatPlotOpenCV
- direct OpenCV occupancy-grid visualization
- efficient OpenCV polyline overlays for ordered grid paths

The former visibility-graph implementation has been removed. The active planner
uses grid-based A* search with safe eight-connected movement by default.

`PolygonEnvironment` owns validated polygon and operation-area geometry;
`AStarGridPlanner` performs search on an `OccupancyGrid`. The former
`AStarPlanner` type name remains as a deprecated compatibility alias. New code
should use `normalizedObstacles()` for normalized inputs and
`effectiveObstacles()` for geometry after operation-area clipping.

## Assumptions and validation

- Obstacles and the optional operation area must be convex simple polygons.
- Clockwise polygons are normalized to counter-clockwise order.
- Repeated closing vertices, consecutive duplicates, and redundant collinear
  boundary vertices are removed.
- Non-finite, self-intersecting, degenerate, and concave polygons are rejected.
- Obstacle inputs with fewer than three vertices are ignored without producing
  library output.
- Obstacles are clipped to the operation area when one is configured.
- Empty, point-only, and line-only clipping results are discarded.
- Direct `PolygonRasterizer` inputs are independently normalized and validated;
  they must also describe convex simple polygons with nonzero area.
- Rasterization follows OpenCV `fillConvexPoly` pixel coverage. It is not a
  conservative rule that marks every cell touched by polygon geometry.

Polygon holes and overlapping-obstacle validation are not supported.

## Coordinates, rasterization, and search

- World coordinates are Cartesian `Point2` values.
- `GridCell{column, row}` is also Cartesian: cell `(0, 0)` is at the lower-left
  of a grid and rows increase with world `y`.
- OpenCV image rows increase downward. `GridGeometry` owns this vertical-axis
  conversion; callers should not invert rows themselves.
- A grid's world origin is the lower-left outer corner of cell `(0, 0)`, not its
  center. Each cell is `resolution` world units wide and high.
- Grid world bounds are half-open. Points on the maximum `x` or `y` boundary
  are outside the grid.
- `gridPathToWorld()` converts path cells to cell centers. The resulting first
  and last points therefore represent the centers of the selected start and
  goal cells, not necessarily the exact world inputs.

With an operation area, rasterization starts with every cell occupied, paints
the operation area free, and then paints effective obstacles occupied. Without
an operation area, rasterization starts free and paints obstacles occupied.
OpenCV filled-polygon pixel coverage determines which cells change state; the
current rasterizer does not conservatively mark every cell geometrically touched
by a polygon. `OccupancyGridInflator` can produce an independent binary grid
with a requested world-space safety radius around occupied cells and grid
boundaries. Graded clearance costs are not yet provided.

`AStarGridPlanner` uses A* with eight-connected movement by default. Every mode
uses the same priority equation `g + weight * h`: Dijkstra selects weight `0`,
A* selects weight `1`, and `GridSearchAlgorithm::WeightedAStar` uses
`AStarOptions::heuristicWeight`. Weighted-A* weights must be finite and at least
`1`; weight `1` is exactly equivalent to A*. Larger weights can reduce node
expansion but may return a non-optimal path. The reported path cost remains the
actual sum of movement costs, not the weighted queue priority.

Orthogonal moves cost `1`, diagonal moves cost `sqrt(2)`, and diagonal corner
cutting is prevented unless explicitly enabled. Four-connected movement is
available through `AStarOptions`.
Equal-cost candidates are resolved deterministically with a straight-line-
deviation tie-breaker. `gridSearchAlgorithmName()` provides a readable name for
the selected mode.

Every `GridPlanResult` includes `GridPlanDiagnostics`. Successful searches
report the final movement cost; unsuccessful searches use infinite path cost.
`expandedNodes` counts non-stale open-set entries processed, including the goal.
`generatedNodes` counts entries inserted into the open set, including the start
and any improved replacement entries. `peakOpenSetSize` records the largest
number of queued entries. Invalid terminals return zero work counts because no
search is started.

Successful paths include both terminal cells. Failed plans return an empty path
and one of these statuses:

- `StartOutsideGrid` or `GoalOutsideGrid`
- `StartOccupied` or `GoalOccupied`
- `NoPath`

`gridPlanStatusName()` provides a readable description of any status.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- [Eigen](https://eigen.tuxfamily.org/) for world-coordinate geometry
- OpenCV 4.5 or newer for occupancy storage, polygon rasterization, and grid rendering
- [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV) when visualization or demos are enabled

The environment-only `a_star_planner.hpp` header does not include OpenCV.
The occupancy-grid API is provided by `occupancy_grid.hpp`.
Generic grid rendering is provided separately by
`occupancy_grid_visualization.hpp`; planner/environment plotting remains in
`a_star_planner_visualization.hpp`.

## Configure and build

Default local dependency paths are defined in `cmake/local_paths.cmake`:

- `ASTAR_PLANNER_EIGEN3_INCLUDE_DIR`
- `ASTAR_PLANNER_MPOCV_SOURCE_DIR`
- `ASTAR_PLANNER_OPENCV_DIR`

Configure and build all targets on Windows with a multi-configuration generator:

```powershell
cmake -S . -B build -DMATPLOTOPENCV_BUILD_DEMO=OFF -DMATPLOTOPENCV_BUILD_DOCS=OFF
cmake --build build --config Release
```

To build the core library and tests without visualization or demos:

```powershell
cmake -S . -B build -DASTAR_PLANNER_ENABLE_VISUALIZATION=OFF -DASTAR_PLANNER_BUILD_DEMO=OFF
cmake --build build --config Release
```

## Tests

Run the environment, occupancy-grid, rasterization, and planner tests directly:

```powershell
.\build\Release\a_star_planner_tests.exe
```

Or use CTest:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

## Tutorial demos

The annotated demos are executable tutorials for the complete planning workflow:

- [`a_star_planner_demo.cpp`](demo/a_star_planner_demo.cpp) starts with obstacle
  polygons, explicitly sizes a master grid, runs weighted A* on the full grid,
  and renders the result. Its pseudo-random obstacle field uses a fixed seed so
  runs are repeatable.
- [`operation_area_demo.cpp`](demo/operation_area_demo.cpp) adds an operation
  area, obstacle clipping, master-map construction, and a caller-selected
  planning ROI.

Both examples label the major steps in code and visualize the polygon world,
occupancy grid, terminals, and resulting A* path.

```powershell
.\build\Release\a_star_planner_demo.exe
.\build\Release\operation_area_demo.exe
```

The first demo builds an explicit world-aligned rectangular grid around a
nonuniform field of polygon obstacles. The second builds a master grid directly
from the operation area's bounding box, paints that area free, and then paints
the effective obstacles occupied. The first demo runs weighted A* on its
complete master grid. The second runs Dijkstra on a caller-selected ROI that deliberately retains
the space needed to route around its obstacle walls. Both demos report
grid-rasterization, safety-inflation, and planning time and display the path in
the occupancy-grid view and the world-coordinate MatPlotOpenCV figure. Pass an optional image
filename as the first argument to save the world-coordinate figure before its
windows are displayed.

The reported grid time begins immediately before the rasterizer call and
excludes `PolygonEnvironment` construction, including obstacle clipping. The
operation-area rasterizer overload also selects its master-grid geometry.
Safety-inflation time covers only `OccupancyGridInflator::inflate()`. The
reported planning time covers only `AStarGridPlanner::plan()`; coordinate
conversion and rendering are excluded. These are single-run diagnostics, not
formal benchmarks.

A planning ROI restricts the search domain; it is not only a storage or display
crop. A path that exists in the master grid may require cells outside the ROI,
so `NoPath` means no path exists inside the supplied planning grid. Use the full
master grid when no safe application-specific planning window is known. An
automatic expanding-ROI retry policy is not currently provided.

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

- `AStarPlanner::astar_planner` — header-only environment, occupancy grid, rasterizer, and A* planner
- `AStarPlanner::astar_occupancy_grid_visualization` — reusable OpenCV grid rendering
- `AStarPlanner::astar_planner_visualization` — optional planner and grid visualization support
- `a_star_planner_tests` — deterministic environment, grid, rasterization, and planner tests
- `a_star_planner_demo` — unconstrained-environment demo
- `operation_area_demo` — operation-area and clipping demo

## Using the planner: basic workflow

A normal planning request has nine explicit steps:

1. Define convex obstacle polygons and, optionally, a convex operation area in
   world coordinates.
2. Construct `PolygonEnvironment` to validate, normalize, and clip that geometry.
3. Choose the grid resolution and master-map extent.
4. Use `PolygonRasterizer` to create the OpenCV-backed occupancy grid.
5. Plan on the full master grid or select a deliberately sized planning ROI.
6. Use `OccupancyGridInflator` to create a safety-inflated planning grid.
7. Convert world start and goal points with the selected grid's `worldToCell()`.
8. Call `AStarGridPlanner::plan()` and check `GridPlanResult::succeeded()`.
9. Convert the returned cells to world-space cell centers with
   `gridPathToWorld()` when world-coordinate output is required.

The following minimal example uses an operation area and plans on its complete
master grid:

```cpp
#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "occupancy_grid.hpp"

#include <iostream>
#include <vector>

int main()
{
    using namespace astar;

    std::vector<Polygon> obstacles{
        {
            Point2(4.0, 2.0),
            Point2(6.0, 2.0),
            Point2(6.0, 8.0),
            Point2(4.0, 8.0)
        }
    };

    const Polygon operationArea{
        Point2(0.0, 0.0), Point2(10.0, 0.0),
        Point2(10.0, 10.0), Point2(0.0, 10.0)
    };
    PolygonEnvironment environment(operationArea, obstacles);

    const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
        environment.operationArea(), environment.effectiveObstacles(), 0.5);
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        masterGrid, 0.5);

    const Point2 worldStart(1.0, 1.0);
    const Point2 worldGoal(9.0, 9.0);
    const auto start = planningGrid.geometry().worldToCell(worldStart);
    const auto goal = planningGrid.geometry().worldToCell(worldGoal);
    if (!start || !goal)
        return 1;

    AStarOptions options; // A* and eight-connected movement by default.
    // Dijkstra: options.algorithm = GridSearchAlgorithm::Dijkstra;
    // Weighted A*:
    // options.algorithm = GridSearchAlgorithm::WeightedAStar;
    // options.heuristicWeight = 2.0;
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
    std::cout << "Path contains " << worldPath.size() << " cell centers at cost "
              << result.diagnostics.pathCost << '\n';
    std::cout << "Expanded " << result.diagnostics.expandedNodes << " nodes\n";
    return 0;
}
```

For fully annotated and visualized versions of this workflow, see the
[full-grid tutorial](demo/a_star_planner_demo.cpp) and the
[operation-area/ROI tutorial](demo/operation_area_demo.cpp).

### Grid and ROI choices

`GridGeometry(origin, resolution, width, height)` provides explicit cell-sized
maps. `GridGeometry::covering(...)` derives rectangular cell dimensions from
world bounds or polygon bounds. `GridGeometry::alignedCovering(...)` additionally
keeps cell boundaries on a stable world lattice. A planning subgrid uses the
master resolution and records its cumulative master-cell offset; it can either
share an OpenCV ROI or own an independent copy. `SharedView` is the default: it
keeps reference-counted source pixels alive, aliases their storage, and may be
non-contiguous. `IndependentCopy` owns a deep copy of the selected pixels.

`OccupancyGrid` is read-only through its grid API. `imageView()` provides a
zero-copy OpenCV view whose shared storage must be treated as read-only;
`cloneImage()` returns an independent image suitable for modification. The
deprecated `image()` name remains temporarily available for source compatibility.

`OccupancyGridInflator::inflate()` accepts a nonnegative safety radius in world
units, rounds it up to a whole-cell radius, and applies a circular OpenCV
dilation kernel. It returns independent storage with the same geometry and
master-cell offset. The source grid is unchanged, and space beyond every edge
of the inflated grid is treated as occupied.

### Inflation boundaries and planning ROIs

The edge of the grid passed to `OccupancyGridInflator` is treated as the limit
of known or permitted space. Inflation therefore creates an occupied band along
that edge whose thickness grows with the requested safety radius. This is the
safe behavior for a complete map, an operation-area boundary, or any map whose
outside space is unknown.

The order of inflation and cropping matters. Inflate a planning ROI directly
when the ROI itself is intended to be a hard boundary:

```cpp
const OccupancyGrid planningRegion = masterGrid.subgrid(planningBounds);
const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
    planningRegion, safetyRadius);
```

This adds clearance around obstacles and every ROI edge. It can occupy a nearby
start or goal, remove a route along the edge, or eliminate all free space in a
narrow ROI.

When an ROI is only a search or performance restriction inside a larger known
map, inflate the master map first and crop afterward:

```cpp
const OccupancyGrid inflatedMaster = OccupancyGridInflator::inflate(
    masterGrid, safetyRadius);
const OccupancyGrid planningGrid = inflatedMaster.subgrid(planningBounds);
```

This preserves clearance from actual obstacles and the master-map boundary
without introducing an artificial inflated obstacle around the ROI. The basic
demo inflates its complete master map. The operation-area demo intentionally
uses the first form to demonstrate a caller-selected hard planning boundary.

## License

AStarPlanner is licensed under the Apache License 2.0. See `LICENSE` for the
full license text.

Copyright 2025-2026 Michael Hannan.
