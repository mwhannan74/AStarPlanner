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
- optional synchronous search-state callbacks for live OpenCV debugging
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

`AStarGridPlanner` uses A* with eight-connected movement by default. Orthogonal
moves cost `1`, diagonal moves cost `sqrt(2)`, and diagonal corner cutting is
prevented unless explicitly enabled. Four-connected movement is available
through `AStarOptions`.

## Search algorithm modes

The planner supports Dijkstra, A*, and weighted A* through one shared graph
search. It does not contain three separate planner implementations. Before the
search begins, `GridSearchAlgorithm` is converted to an effective heuristic
weight. Every mode then uses the same priority equation:

```text
priority(n) = pathCostFromStart(n) + effectiveWeight * baseHeuristic(n, goal)
```

| Algorithm | Option | Effective weight | Behavior |
|---|---|---:|---|
| Dijkstra | `GridSearchAlgorithm::Dijkstra` | `0` | Optimal, but normally explores the broadest region because it has no goal-directed heuristic. |
| A* | `GridSearchAlgorithm::AStar` | `1` | Default mode. Optimal with the planner's current admissible Manhattan or octile heuristic. |
| Weighted A* | `GridSearchAlgorithm::WeightedAStar` | `heuristicWeight` | Weight `1` is exactly A*. Larger weights can reduce expansions but may return a non-optimal path. |

The weighted-A* heuristic weight must be finite and at least `1`. It affects only
open-set priority; it does not change orthogonal or diagonal movement costs.
Consequently, `GridPlanDiagnostics::pathCost` always reports the actual sum of
movement costs rather than the weighted priority.

Select a mode through `AStarOptions`:

```cpp
AStarOptions options; // A* by default.

options.algorithm = GridSearchAlgorithm::Dijkstra;

options.algorithm = GridSearchAlgorithm::WeightedAStar;
options.heuristicWeight = 2.0;
```

`heuristicWeight` is used only in weighted-A* mode. Dijkstra always selects an
effective weight of `0`, and normal A* always selects `1`. The selected mode does
not alter neighbor generation, collision checking, movement costs, parent
tracking, path reconstruction, or diagnostics. Equal-cost candidates use a
deterministic straight-line-deviation tie-breaker.
`gridSearchAlgorithmName()` provides a readable name for the selected mode.

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

### Live search debugging

`AStarGridPlanner::plan()` accepts an optional `GridSearchDebugCallback`. An
empty callback is normal planning: no debug-state image is allocated or updated.
Supplying a callback allocates one reusable `cv::Mat1b` with the same dimensions
and image orientation as the planning grid. Each pixel contains a
`GridSearchCellState` value: `Unseen`, `Open`, `Current`, `Expanded`, `Path`, or
`Occupied`.

The default debug rendering uses this color key:

| Color | Search meaning |
|---|---|
| White | Unseen traversable cell |
| Yellow | Open/frontier cell discovered but not yet expanded |
| Orange | Cell currently being expanded |
| Light blue | Expanded cell whose neighbors have been examined |
| Blue | Final reconstructed path |
| Black | Occupied cell |
| Green marker | Start cell |
| Red marker | Goal cell |

These are visualization colors rather than occupancy values.
`GridSearchStateRenderOptions` allows every color to be changed without changing
the planner's `GridSearchCellState` values.

The callback runs synchronously after each accepted expansion and once more for
the final path or exhausted search. It therefore blocks planning until it
returns, which allows a debug callback to control animation speed with
`cv::waitKey()` without storing image snapshots:

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

The matrix is a read-only view of storage reused by the planner. It is valid
only during the callback and is modified after the callback returns. Immediate
display requires no copy; clone it inside the callback only when retaining a
frame or transferring it to another thread. `renderGridSearchState()` and
`showGridSearchState()` map the state values to configurable BGR colors. A
callback may skip display work on selected invocations to reduce the number of
rendered frames.

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

## Performance benchmark

`a_star_planner_benchmark` measures the complete non-visual planning pipeline
without changing the planner implementation. It separately times environment
validation and clipping, OpenCV rasterization, full-map ROI creation, one-cell
safety inflation, terminal conversion, graph search, path conversion, and the
corresponding preparation and per-request totals. Search results also include
expanded nodes, generated nodes, peak open-set size, path size, and path cost.

The default deterministic suite runs open, alternating-barrier, and unreachable
scenarios at `100 x 100`, `250 x 250`, `500 x 500`, and `1000 x 1000` cells.
At the benchmark's 25-meter resolution these represent square maps from 2.5 km
through 25 km per side; a `1000 x 1000` grid contains one million cells. A*
Dijkstra, and weighted A* with weight 1.5 are run on every case. Each measurement
has one untimed warm-up, followed by a size-dependent number of repetitions.

Run a Release build for meaningful results:

```powershell
cmake --build build --config Release --target a_star_planner_benchmark
.\build\Release\a_star_planner_benchmark.exe
```

The executable prints a compact terminal summary and writes a timestamped CSV
file in the working directory. Use `--csv <file>` to select its location,
`--quick` for a short `100 x 100` smoke run, or `--stress` to add a
`2000 x 2000` four-million-cell case. The CSV files are ignored by Git by
default. Benchmark timings should be compared only between similar Release
builds on the same machine under similar system load.

### Isolated search visualization

`a_star_search_debug` recreates the benchmark's `500 x 500`
alternating-barrier scenario for one selected algorithm. It first performs an
uninstrumented search for a meaningful baseline time, then repeats the search
with the debug callback enabled. Sampled search frames are animated in an
OpenCV window, and the final state is saved as a PNG with its color key and
search diagnostics embedded beside the map.

```powershell
.\build\Release\a_star_search_debug.exe --algorithm astar
.\build\Release\a_star_search_debug.exe --algorithm dijkstra
.\build\Release\a_star_search_debug.exe --algorithm weighted --weight 1.5
```

The default files are `astar_search_debug_astar.png`,
`astar_search_debug_dijkstra.png`, and
`astar_search_debug_weighted_astar.png`. Use `--output <file>` to change the
name, `--frame-stride <count>` to control animation sampling,
`--no-animation` to display only the completed search, or `--no-display` to
save the image without opening a window. Debug callback and rendering time are
deliberately excluded from the reported baseline planning time.

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

# Animate the search state while planning.
.\build\Release\a_star_planner_demo.exe --debug
.\build\Release\operation_area_demo.exe --debug
```

The first demo builds an explicit world-aligned rectangular grid around a
nonuniform field of polygon obstacles. The second builds a master grid directly
from the operation area's bounding box, paints that area free, and then paints
the effective obstacles occupied. The first demo runs weighted A* on its
complete master grid. The second runs Dijkstra on a caller-selected ROI that
deliberately retains the space needed to route around its obstacle walls. Both
demos report
grid-rasterization, safety-inflation, and planning time and display the path in
the occupancy-grid view and the world-coordinate MatPlotOpenCV figure. Pass an optional image
filename as the first argument to save the world-coordinate figure before its
windows are displayed.

The reported grid time begins immediately before the rasterizer call and
excludes `PolygonEnvironment` construction, including obstacle clipping. The
operation-area rasterizer overload also selects its master-grid geometry.
Safety-inflation time covers only `OccupancyGridInflator::inflate()`. The
reported planning time covers only `AStarGridPlanner::plan()`; coordinate
conversion and final rendering are excluded. In `--debug` mode, the synchronous
callback's rendering and frame delays are included in planning time. These are
single-run diagnostics, not formal benchmarks.

A planning ROI restricts the search domain; it is not only a storage or display
crop. A path that exists in the master grid may require cells outside the ROI,
so `NoPath` means no path exists inside the supplied planning grid. Use the full
master grid when no safe application-specific planning window is known. An
automatic expanding-ROI retry policy is not currently provided.

```bash
> .\build\Release\a_star_planner_demo.exe
Environment has 25 effective obstacles
Master occupancy grid: 69 x 75 cells at 1 world units per cell
Planning grid (full master map): 69 x 75 cells with 1 world units of safety inflation
Grid rasterization: 0.036 ms
Safety inflation: 0.051 ms
Weighted A* (weight 2.000) planning: 0.062 ms
Expanded nodes: 162
Generated nodes: 431
Peak open-set size: 255
Path cost: 112.912
A* path: 99 cells
```

<p align="center">
  <img src="images/a_star_planner_demo.png"
       alt="AStarPlanner polygon environment demo"
       width="800">
</p>

```bash
> .\build\Release\operation_area_demo.exe
Input obstacles: 9
Effective obstacles after clipping: 8
Clipped obstacle overlays: 5
Master occupancy grid: 100 x 60 cells at 1 world units per cell
Planning ROI (caller-selected): 94 x 60 cells, master offset (3, 0) with 1 world units of safety inflation
Grid rasterization: 0.031 ms
Safety inflation: 0.051 ms
Dijkstra planning: 0.195 ms
Expanded nodes: 3082
Generated nodes: 3186
Peak open-set size: 60
Path cost: 181.397
A* path: 165 cells
```
<p align="center">
  <img src="images/a_star_planner_operation_area_demo.png"
       alt="AStarPlanner operation-area and obstacle-clipping demo"
       width="800">
</p>

## CMake targets

- `AStarPlanner::astar_planner` — header-only environment, occupancy grid, rasterizer, and A* planner
- `AStarPlanner::astar_occupancy_grid_visualization` — reusable OpenCV grid rendering
- `AStarPlanner::astar_planner_visualization` — optional planner and grid visualization support
- `a_star_planner_tests` — deterministic environment, grid, rasterization, and planner tests
- `a_star_planner_benchmark` — deterministic terminal and CSV performance benchmark
- `a_star_search_debug` — isolated visual comparison of benchmark search modes
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
