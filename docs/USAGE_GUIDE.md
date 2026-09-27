# Using AStarPlanner

This guide explains how to build occupancy grids, plan paths, configure search,
and use the debugging and performance tools. For a class-by-class inventory,
see the [API Guide](API_GUIDE.md). For installation and a fast first run, start
with the [README](../README.md).

## Core workflow

A planning request is organized around one caller-selected `GridGeometry`:

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

The normal sequence is:

1. Define polygon obstacles and, optionally, an operation area.
2. Select the world-coordinate planning bounds and grid resolution.
3. Create the shared planning geometry.
4. Validate polygon geometry with `PolygonEnvironment`.
5. Rasterize polygons and current perception points independently.
6. Fuse the occupancy sources.
7. Inflate the fused occupancy once by the required safety radius.
8. Convert world start and goal positions to grid cells.
9. Call `AStarGridPlanner::plan()` and inspect the result.

Polygon-only and perception-only applications can omit the unused branch and
inflate their one occupancy grid directly.

## Complete mixed-input example

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
    const std::vector<Point2> obstaclePoints{
        Point2(3.25, 7.25),
        Point2(3.75, 7.25),
        Point2(3.75, 7.75)
    };

    const WorldBounds planningBounds{
        Point2(0.0, 0.0), Point2(10.0, 10.0)
    };
    const GridGeometry planningGeometry =
        GridGeometry::covering(planningBounds, 0.5);
    const PolygonEnvironment environment(operationArea, obstacles);

    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        planningGeometry, environment);
    const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
        planningGeometry, obstaclePoints);
    const OccupancyGrid fusedGrid = OccupancyGridFusion::occupiedUnion(
        polygonGrid, perceptionGrid);
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        fusedGrid, 0.5);

    const auto start = planningGrid.geometry().worldToCell(Point2(1.0, 1.0));
    const auto goal = planningGrid.geometry().worldToCell(Point2(9.0, 9.0));
    if (!start || !goal)
        return 1;

    const AStarGridPlanner planner;
    const GridPlanResult result = planner.plan(
        planningGrid, *start, *goal);
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

The annotated programs in [`demo/`](../demo) show complete workflows with
timing and visualization. In particular, [`robot_planning_demo.cpp`](../demo/robot_planning_demo.cpp)
uses `forwardSideBehind()` to build a compact grid around a robot position and
then combines an operation area, scattered polygon obstacles, and perception
blobs before running the normal inflation and planning pipeline. Its fixed
tutorial blobs can be replaced with a fresh layout using `--random`, or a
repeatable generated layout using `--seed N`. Generated layouts include blobs
across the larger local world plus several near the nominal start-to-goal route,
so perception changes the planning problem rather than only the background.
They also select a goal after the fused grid is inflated. Goal candidates must
be free, forward of the robot, sufficiently distant, and clear of the grid
boundary. The demo does not preflight reachability; A* reports a normal failure
if the selected free goal is disconnected from the start.

<p align="center">
  <img src="../images/robot_local_planning.png"
       alt="Robot-local planning with polygon and perception occupancy"
       width="600">
</p>

In this generated example, the green marker is the robot and start position,
while the red marker is the selected free goal. The black occupancy combines
the keep-in operation-area boundary, rectangular polygon keep-out zones,
perception blobs, and their safety inflation. The connected A* path is blue and
the collision-checked simplified path is orange. The larger forward region and
randomized goal allow perception changes to produce meaningfully different
plans between seeds.

## Planning bounds and geometry

The application selects the planning boundary. The library does not infer a
horizon from obstacles, terminals, a robot position, or sensor range. Common
choices include:

- a fixed world region;
- the operation-area bounds;
- start and goal with application-selected padding;
- a robot-local planning horizon prepared by the application.

```cpp
const WorldBounds planningBounds{
    minimumWorldPoint,
    maximumWorldPoint
};

const GridGeometry planningGeometry =
    GridGeometry::covering(planningBounds, resolution);
```

`covering()` uses the requested minimum as the grid origin and rounds width and
height up to complete cells. Its resulting `worldBounds()` are authoritative
and may extend less than one cell beyond the requested maximum. Data outside
those bounds is not part of the planning request.

`GridGeometry::alignedCovering()` is an advanced alternative. It expands both
ends of the requested bounds onto a stable lattice defined by a resolution and
anchor. Use it when separate requests must retain identical world-aligned cell
boundaries.

### Local robot-style planning bounds

`LocalPlanningRegion` is a geometry convenience helper for planning around a
robot or any other moving reference position. It does not change the behavior
of `GridGeometry`, `OccupancyGrid`, or the planner.

Create a forward-looking region with one symmetric side distance:

```cpp
const LocalPlanningRegion region =
    LocalPlanningRegion::forwardSideBehind(
        20.0,  // forward
        10.0,  // positive and negative side distance
        5.0);  // behind

const WorldBounds planningBounds =
    region.boundsAround(robotPosition);

const GridGeometry planningGeometry =
    GridGeometry::covering(planningBounds, resolution);
```

Create a symmetric square by specifying its half extent:

```cpp
const LocalPlanningRegion region =
    LocalPlanningRegion::centeredSquare(15.0);
```

This produces a square extending 15 world units in every direction, for a total
width and height of 30 world units.

The helper is deliberately axis-aligned in the caller's coordinate frame:

```text
forward = positive x
behind  = negative x
side    = positive and negative y
```

For a robot-aligned local frame, positive x naturally represents robot forward.
For a world/map frame, the caller is responsible for selecting an appropriate
frame or using the orientation-independent centered square. The helper does not
rotate points or polygons and does not create an axis-aligned bounding box for a
rotated robot footprint.

The helper returns `WorldBounds` rather than choosing grid alignment. The caller
can pass those bounds to either `GridGeometry::covering()` or
`GridGeometry::alignedCovering()`.

Forward and behind distances may individually be zero, but not both. Side
distance and square half extent must be positive. When inflation is enabled,
placing the reference position directly against a planning boundary may make
its cell occupied because space beyond the grid is treated as occupied.

Avoid constructing a grid larger than the planning request merely to crop it
later. Allocation, rasterization, fusion, inflation, and search all scale with
the number of cells.

## Polygon environments

`PolygonEnvironment` accepts convex, simple world-coordinate polygons. It:

- normalizes clockwise polygons to counter-clockwise order;
- removes repeated closing points, consecutive duplicates, and redundant
  collinear vertices;
- rejects non-finite, self-intersecting, degenerate, and concave polygons;
- ignores inputs with fewer than three vertices;
- clips obstacles to an optional convex operation area;
- discards empty, point-only, and line-only clipping results.

Construct an unconstrained environment with:

```cpp
const PolygonEnvironment environment(obstacles);
```

Construct a keep-in environment with:

```cpp
const PolygonEnvironment environment(operationArea, obstacles);
```

The environment exposes normalized input obstacles, effective clipped
obstacles, and changed clipping overlays for inspection and visualization.

## Coordinates and cells

- `Point2` uses Cartesian world coordinates.
- `GridCell{column, row}` is also Cartesian.
- Cell `(0, 0)` is at the lower left; rows increase with world `y`.
- OpenCV image rows increase downward. `GridGeometry` performs that conversion.
- A grid's world origin is the lower-left outer corner of cell `(0, 0)`.
- Grid world bounds are half-open. A point on the maximum boundary is outside.
- `gridPathToWorld()` returns cell centers, not the exact query coordinates.

Convert terminals with the geometry belonging to the grid that will actually
be searched:

```cpp
const auto start = planningGrid.geometry().worldToCell(worldStart);
const auto goal = planningGrid.geometry().worldToCell(worldGoal);
```

## Polygon rasterization

The preferred interface accepts caller-selected geometry and a complete
validated environment:

```cpp
const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
    planningGeometry, environment);
```

Without an operation area, rasterization begins with a free grid and paints
obstacles occupied. With an operation area, it begins occupied, paints the
operation area free, and then paints effective obstacles occupied.

Rasterization follows OpenCV `fillConvexPoly` pixel coverage. It is not an exact
continuous any-cell-intersection test. Apply safety inflation when the planner
requires clearance around obstacles.

## Perception-point rasterization

Supply current world-coordinate detections using the same geometry:

```cpp
const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
    planningGeometry, obstaclePoints);
```

Each finite point inside the geometry marks its containing cell occupied.
Duplicate points are harmless. Non-finite and out-of-bounds points are ignored.

Every call creates a fresh grid. `PointRasterizer` does not perform temporal
accumulation, confidence tracking, decay, ray clearing, sensor fusion, or robot
frame transforms. Those are application or mapping-system responsibilities.

## Occupancy fusion

Combine aligned sources using occupied-wins semantics:

```cpp
const OccupancyGrid fusedGrid = OccupancyGridFusion::occupiedUnion(
    polygonGrid, perceptionGrid);
```

A cell is occupied when either input cell is occupied. Inputs must have the
same dimensions, origin, resolution, and master-cell offset. Fusion does not
resample or reproject data. It leaves both inputs unchanged and returns
independent storage.

## Safety inflation

Inflate only after all current occupancy sources have been fused:

```cpp
const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
    fusedGrid, safetyRadius);
```

The radius is expressed in world units and is conservatively rounded up to a
whole-cell radius. Inflation returns independent storage and leaves the input
unchanged.

Space beyond each supplied grid edge is treated as occupied. Inflation
therefore creates a clearance band along the planning boundary as well as
around occupied cells.

## Planning results

Every successful `GridPlanResult` contains:

- `path`: the original connected graph-search path;
- `simplifiedPath`: collision-checked line-of-sight waypoints;
- `diagnostics`: path cost, search-work counts, and simplification time.

The planner simplifies successful paths automatically. Simplified segments are
checked against the same grid supplied to `plan()`, including its safety
inflation. `simplifyGridPath()` exposes the same operation for another ordered
grid path.

Failed requests return empty paths and one of:

- `StartOutsideGrid` or `GoalOutsideGrid`;
- `StartOccupied` or `GoalOccupied`;
- `NoPath`.

Use `result.succeeded()` and `gridPlanStatusName(result.status)` for normal
result handling.

## Search configuration

The default is eight-connected A*. Orthogonal moves cost `1`, diagonal moves
cost `sqrt(2)`, and diagonal corner cutting is prevented.

### Search algorithms

| Algorithm | Option | Effective heuristic weight | Behavior |
|---|---|---:|---|
| Dijkstra | `GridSearchAlgorithm::Dijkstra` | `0` | Optimal and not goal-directed |
| A* | `GridSearchAlgorithm::AStar` | `1` | Default optimal search |
| Weighted A* | `GridSearchAlgorithm::WeightedAStar` | Caller-selected | May trade optimality for less work |

```cpp
AStarOptions options;
options.algorithm = GridSearchAlgorithm::WeightedAStar;
options.heuristicWeight = 2.0;
options.connectivity = GridConnectivity::EightConnected;
options.preventDiagonalCornerCutting = true;
```

The common priority equation is:

```text
priority(n) = pathCostFromStart(n) + effectiveWeight * baseHeuristic(n, goal)
```

The weight affects open-set priority, not movement cost or the reported path
cost.

### Tie-breaking

Tie-breaking orders candidates whose search priorities are equal. It can alter
search effort and the selected equal-cost path, but not normal A* optimality.

| Policy | Behavior |
|---|---|
| `StraightLineThenLargerG` | Prefer the direct start-to-goal line, then greater `g`; default |
| `LargerGThenStraightLine` | Prefer greater `g`, then straight-line deviation |
| `LargerGOnly` | Prefer greater `g` without a geometric secondary key |

There is no universally fastest policy. Benchmark representative maps before
changing an application's default.

### Diagnostics

Every result reports:

- `pathCost`;
- `expandedNodes`;
- `generatedNodes`;
- `peakOpenSetSize`;
- `pathSimplificationMilliseconds`.

Set `AStarOptions::collectDetailedDiagnostics` to add a per-cell expansion-count
image and counters for unique expansions, repeated expansions, stale entries,
and post-expansion cost improvements. Detailed collection adds work and storage
and is disabled by default.

## Visualization and debugging

`renderOccupancyGrid()` produces a BGR image with optional paths and markers.
`showOccupancyGrid()` displays it directly. `visualize()` renders polygon
geometry, original and simplified world paths, and terminals.

For live search state, supply a synchronous callback:

```cpp
GridSearchDebugCallback debugCallback =
    [&](const cv::Mat1b& state)
    {
        showGridSearchState(
            planningGrid, state, *start, *goal, {}, "Search Debug");
        cv::waitKey(10);
    };

const GridPlanResult result = planner.plan(
    planningGrid, *start, *goal, options, debugCallback);
```

The callback's image is valid only during the callback and is modified after it
returns. Clone it when a frame must be retained or transferred to another
thread. The callback blocks planning, so visualization may dominate measured
runtime.

The default search colors are:

| Color | Meaning |
|---|---|
| White | Unseen traversable cell |
| Yellow | Open/frontier cell |
| Orange | Cell currently being expanded |
| Light blue | Expanded cell |
| Blue | Final reconstructed path |
| Black | Occupied cell |
| Green / red | Start / goal markers |

## Advanced: existing maps and subgrids

Subgrids are for applications that already own a larger persistent occupancy
map. Do not introduce a larger map solely to crop it immediately.

```cpp
const OccupancyGrid planningRegion = masterGrid.subgrid(planningBounds);
```

A subgrid limits the search domain. A route may exist in the source map but
require cells outside the selected region; planning in the subgrid then
correctly returns `NoPath`.

Inflate a subgrid directly when its edges are hard planning boundaries:

```cpp
const OccupancyGrid planningRegion = masterGrid.subgrid(planningBounds);
const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
    planningRegion, safetyRadius);
```

Inflate the larger map first when the subgrid is only a search restriction
inside otherwise usable known space:

```cpp
const OccupancyGrid inflatedMaster = OccupancyGridInflator::inflate(
    masterGrid, safetyRadius);
const OccupancyGrid planningGrid = inflatedMaster.subgrid(planningBounds);
```

Subgrids retain cumulative master-cell offsets and can use a shared OpenCV view
or independent copied storage. `imageView()` is zero-copy and read-only by
contract; `cloneImage()` returns writable independent pixels.

## Integrating with CMake

When vendoring the project, disable standalone targets that the parent project
does not need:

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

Keep visualization enabled and link one of these when required:

```cmake
AStarPlanner::astar_occupancy_grid_visualization
AStarPlanner::astar_planner_visualization
```

The second target includes the MatPlotOpenCV environment plotter and
transitively includes occupancy-grid visualization.

### Build configuration reference

| CMake option | Default | Purpose |
|---|---:|---|
| `BUILD_TESTING` | `ON` | Build and register the test executables |
| `ASTAR_PLANNER_BUILD_DEMO` | `ON` | Build the four tutorial demos |
| `ASTAR_PLANNER_ENABLE_VISUALIZATION` | `ON` | Enable MatPlotOpenCV and OpenCV visualization support |
| `ASTAR_PLANNER_BUILD_BENCHMARKS` | `ON` | Build benchmark, profiling, and search-debug tools |

The standalone executable targets are:

| Target | Purpose |
|---|---|
| `a_star_planner_demo` | Full-grid polygon tutorial |
| `operation_area_demo` | Operation-area and obstacle-clipping tutorial |
| `perception_fusion_demo` | Polygon and perception-fusion tutorial |
| `robot_planning_demo` | Robot-local bounds and mixed-occupancy tutorial |
| `a_star_planner_tests` | Deterministic core tests |
| `occupancy_grid_visualization_tests` | Rendering tests |
| `a_star_planner_benchmark` | Repeatable performance suite and CSV output |
| `a_star_planner_profile` | Isolated repeated CPU workload |
| `a_star_search_debug` | Visual comparison of search modes |

## Benchmarking and profiling

Run the deterministic benchmark in Release mode:

```powershell
cmake --build build --config Release --target a_star_planner_benchmark
.\build\Release\a_star_planner_benchmark.exe
```

It measures environment validation, polygon and point rasterization, occupancy
fusion, inflation, terminal conversion, search, simplification, path
conversion, and request totals. Results are also written to CSV.

The `mixed_occupancy` scenario uses both polygons and perception points. Its
points overlap the polygon barriers so preprocessing overhead can be compared
without changing the search problem. Polygon-only scenarios report zero for
the skipped point and fusion stages.

Useful options are:

- `--quick` for a `100 x 100` smoke run;
- `--stress` to add a `2000 x 2000` case;
- `--csv <file>` to choose the report path;
- `--tie-break <policy>` to compare tie-breaking policies.

For CSV compatibility, `rasterization_median_ms` represents polygon
rasterization. New input-count, point-rasterization, and fusion fields are
appended after the original columns.

The isolated CPU workload excludes grid construction from its timed loop:

```powershell
.\build\Release\a_star_planner_profile.exe --iterations 50
```

The search-visualization tool compares categorical state and expansion counts:

```powershell
.\build\Release\a_star_search_debug.exe --algorithm astar
.\build\Release\a_star_search_debug.exe --algorithm dijkstra
.\build\Release\a_star_search_debug.exe --algorithm weighted --weight 1.5
```

It supports `--no-animation`, `--no-display`, `--output <file>`, and
`--frame-stride <count>`.

## Design boundaries

- Obstacles and operation areas must be convex simple polygons.
- Polygon holes are not supported.
- Occupancy is binary; graded traversal and confidence costs are not supported.
- Point rasterization does not accumulate perception history.
- Fusion requires already aligned grids and does not resample or reproject.
- Search terminals and grid paths represent cell centers.
- Simplification produces straight collision-checked segments, not curves or
  vehicle-dynamics-constrained trajectories.
- The planner assumes a static occupancy grid during each request.

See the [API Guide](API_GUIDE.md) for exact entry points and the public headers
for authoritative behavior and exception documentation.
