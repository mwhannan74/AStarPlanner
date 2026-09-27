# AStarPlanner API Guide

This guide is a compact inventory of the public C++ interfaces. The public
headers remain authoritative for exact behavior, validation, and exceptions.
For task-oriented explanations and complete workflows, see the
[Usage Guide](USAGE_GUIDE.md).

## Public headers

| Header | Purpose |
|---|---|
| [`a_star_planner.hpp`](../include/a_star_planner.hpp) | World geometry and validated polygon environments |
| [`occupancy_grid.hpp`](../include/occupancy_grid.hpp) | Grid geometry, occupancy storage, rasterization, fusion, inflation, and path conversion |
| [`a_star_grid_planner.hpp`](../include/a_star_grid_planner.hpp) | Search configuration, planning, results, diagnostics, and path simplification |
| [`occupancy_grid_visualization.hpp`](../include/occupancy_grid_visualization.hpp) | OpenCV occupancy and search-state rendering |
| [`a_star_planner_visualization.hpp`](../include/a_star_planner_visualization.hpp) | MatPlotOpenCV polygon-environment plotting |

## Common pipeline interfaces

| Task | Interface |
|---|---|
| Validate polygons and an optional operation area | `PolygonEnvironment` |
| Define bounds, resolution, and coordinate transforms | `GridGeometry` |
| Rasterize polygon occupancy | `PolygonRasterizer` |
| Rasterize world-coordinate detections | `PointRasterizer` |
| Combine aligned occupancy sources | `OccupancyGridFusion` |
| Add safety clearance | `OccupancyGridInflator` |
| Plan and simplify a path | `AStarGridPlanner` |
| Inspect occupancy | `OccupancyGrid` |
| Inspect the planning outcome | `GridPlanResult` |

## Geometry types

```cpp
using Point2 = Eigen::Vector2d;
using Polygon = std::vector<Point2>;

struct GridCell
{
    int column;
    int row;
};

struct WorldBounds
{
    Point2 minimum;
    Point2 maximum;
};

struct GridRegion
{
    GridCell lowerLeft;
    int width;
    int height;
};
```

`Point2` and `GridCell` are Cartesian. Grid row zero is the bottom row.

## `LocalPlanningRegion`

`LocalPlanningRegion` creates axis-aligned `WorldBounds` around a reference
position. Positive x is forward, negative x is behind, and the side distance
extends equally in positive and negative y.

Factories:

```cpp
static LocalPlanningRegion forwardSideBehind(
    double forwardDistance,
    double sideDistance,
    double behindDistance);

static LocalPlanningRegion centeredSquare(
    double halfExtent);
```

Bounds and accessors:

```cpp
WorldBounds boundsAround(const Point2& referencePosition) const;

double forwardDistance() const;
double sideDistance() const;
double behindDistance() const;
```

The helper returns bounds only. Callers retain the choice between
`GridGeometry::covering()` and `GridGeometry::alignedCovering()`. It does not
apply pose transforms or rotate the grid.

## `PolygonEnvironment`

Constructors:

```cpp
explicit PolygonEnvironment(const std::vector<Polygon>& obstacles);

PolygonEnvironment(
    const Polygon& operationArea,
    const std::vector<Polygon>& obstacles);
```

Inspection:

```cpp
bool hasOperationArea() const;
const Polygon& operationArea() const;
const std::vector<Polygon>& normalizedObstacles() const;
const std::vector<Polygon>& clippedObstacles() const;
const std::vector<Polygon>& effectiveObstacles() const;
```

The class validates and normalizes polygon inputs. When an operation area is
present, `effectiveObstacles()` contains the positive-area clipped geometry used
for rasterization.

Deprecated compatibility members are retained:

```cpp
originalObstacles(); // Use normalizedObstacles().
obstacles();         // Use effectiveObstacles().
```

`AStarPlanner` is also a deprecated alias for `PolygonEnvironment`.

## `GridGeometry`

Explicit construction:

```cpp
GridGeometry(
    Point2 worldOrigin,
    double resolution,
    int width,
    int height);
```

Geometry factories:

```cpp
static GridGeometry covering(
    const WorldBounds& bounds,
    double resolution);

static GridGeometry covering(
    const Polygon& polygon,
    double resolution);

static GridGeometry covering(
    const std::vector<Polygon>& polygons,
    double resolution);

static GridGeometry alignedCovering(
    const WorldBounds& bounds,
    double resolution,
    const Point2& anchor = Point2::Zero());
```

`alignedCovering()` also has polygon and polygon-vector overloads.

Bounds helpers:

```cpp
static WorldBounds boundingBox(const Polygon& polygon);
static WorldBounds boundingBox(const std::vector<Polygon>& polygons);

const Point2& worldOrigin() const;
Point2 worldMaximum() const;
WorldBounds worldBounds() const;
double resolution() const;
int width() const;
int height() const;
```

Containment and transforms:

```cpp
bool contains(const WorldBounds& bounds) const;
bool contains(const GridCell& cell) const;
bool containsWorld(const Point2& point) const;

std::optional<GridCell> worldToCell(const Point2& point) const;
Point2 cellCenterToWorld(const GridCell& cell) const;
WorldBounds cellBounds(const GridCell& cell) const;

cv::Point cellToImage(const GridCell& cell) const;
std::optional<GridCell> imageToCell(const cv::Point& pixel) const;
cv::Point2d worldToImage(const Point2& point) const;
std::vector<cv::Point2d> worldToImage(const Polygon& polygon) const;
Point2 imageToWorld(const cv::Point2d& point) const;
```

## `OccupancyGrid`

Cell state:

```cpp
enum class CellState : std::uint8_t
{
    Free = 0,
    Occupied = 255
};
```

Uniform construction:

```cpp
OccupancyGrid(GridGeometry geometry, CellState initialState);
```

Inspection:

```cpp
const GridGeometry& geometry() const;
int width() const;
int height() const;
bool contains(const GridCell& cell) const;
CellState at(const GridCell& cell) const;
bool isTraversable(const GridCell& cell) const;
```

OpenCV interoperability:

```cpp
const cv::Mat1b& imageView() const;
cv::Mat1b cloneImage() const;
```

`imageView()` is zero-copy and read-only by contract. `cloneImage()` returns
independently writable pixels. There is currently no public constructor that
adopts an arbitrary external `cv::Mat`; normal occupancy input enters through
the polygon and point rasterizers.

## Rasterizers

Preferred polygon interface:

```cpp
static OccupancyGrid PolygonRasterizer::rasterize(
    const GridGeometry& geometry,
    const PolygonEnvironment& environment);
```

Lower-level polygon overloads:

```cpp
static OccupancyGrid PolygonRasterizer::rasterize(
    const GridGeometry& geometry,
    const std::vector<Polygon>& obstacles);

static OccupancyGrid PolygonRasterizer::rasterize(
    const GridGeometry& geometry,
    const Polygon& operationArea,
    const std::vector<Polygon>& obstacles);

static OccupancyGrid PolygonRasterizer::rasterize(
    const Polygon& operationArea,
    const std::vector<Polygon>& obstacles,
    double resolution,
    const Point2& alignmentAnchor = Point2::Zero());
```

Point interface:

```cpp
static OccupancyGrid PointRasterizer::rasterize(
    const GridGeometry& geometry,
    const std::vector<Point2>& obstaclePoints);
```

Each rasterizer returns a new binary grid.

## Occupancy fusion and inflation

Occupied-union fusion:

```cpp
static OccupancyGrid OccupancyGridFusion::occupiedUnion(
    const OccupancyGrid& first,
    const OccupancyGrid& second);
```

Inputs must describe the same cell lattice and master-grid region.

Safety inflation:

```cpp
static OccupancyGrid OccupancyGridInflator::inflate(
    const OccupancyGrid& source,
    double safetyRadius);
```

The radius is in world units. Both operations return independent storage and do
not modify their inputs.

## Existing-map and subgrid interfaces

```cpp
enum class SubgridStorage
{
    SharedView,
    IndependentCopy
};
```

Selection:

```cpp
GridRegion fullRegion() const;
GridRegion regionCovering(const WorldBounds& bounds) const;

OccupancyGrid subgrid(
    const GridRegion& region,
    SubgridStorage storage = SubgridStorage::SharedView) const;

OccupancyGrid subgrid(
    const WorldBounds& bounds,
    SubgridStorage storage = SubgridStorage::SharedView) const;
```

Coordinate mapping:

```cpp
const GridCell& masterCellOffset() const;
GridCell localToMaster(const GridCell& localCell) const;
std::optional<GridCell> masterToLocal(const GridCell& masterCell) const;
```

Subgrids are an advanced interface for applications that already own a larger
map; they are not required by the normal rasterization pipeline.

## Search configuration

```cpp
enum class GridConnectivity
{
    FourConnected,
    EightConnected
};

enum class GridSearchAlgorithm
{
    AStar,
    Dijkstra,
    WeightedAStar
};

enum class AStarTieBreakPolicy
{
    StraightLineThenLargerG,
    LargerGThenStraightLine,
    LargerGOnly
};
```

Options:

```cpp
struct AStarOptions
{
    GridSearchAlgorithm algorithm = GridSearchAlgorithm::AStar;
    double heuristicWeight = 1.0;
    GridConnectivity connectivity = GridConnectivity::EightConnected;
    bool preventDiagonalCornerCutting = true;
    bool collectDetailedDiagnostics = false;
    AStarTieBreakPolicy tieBreakPolicy =
        AStarTieBreakPolicy::StraightLineThenLargerG;
};
```

Readable names:

```cpp
gridSearchAlgorithmName(algorithm);
aStarTieBreakPolicyName(policy);
```

## `AStarGridPlanner`

Planning entry point:

```cpp
GridPlanResult plan(
    const OccupancyGrid& grid,
    const GridCell& start,
    const GridCell& goal,
    const AStarOptions& options = {},
    const GridSearchDebugCallback& debugCallback = {}) const;
```

The result includes both terminal cells when planning succeeds.

## Planning results

Status:

```cpp
enum class GridPlanStatus
{
    Success,
    StartOutsideGrid,
    GoalOutsideGrid,
    StartOccupied,
    GoalOccupied,
    NoPath
};
```

Use `gridPlanStatusName(status)` for a readable description.

Result:

```cpp
struct GridPlanResult
{
    GridPlanStatus status;
    std::vector<GridCell> path;
    std::vector<GridCell> simplifiedPath;
    GridPlanDiagnostics diagnostics;
    std::optional<GridSearchDetailedDiagnostics> detailedDiagnostics;

    bool succeeded() const;
};
```

Lightweight diagnostics:

```cpp
struct GridPlanDiagnostics
{
    double pathCost;
    std::size_t expandedNodes;
    std::size_t generatedNodes;
    std::size_t peakOpenSetSize;
    double pathSimplificationMilliseconds;
};
```

`GridSearchDetailedDiagnostics` adds an OpenCV expansion-count image and
detailed repeated-expansion and cost-improvement counters when requested.

## Path helpers

```cpp
std::vector<GridCell> simplifyGridPath(
    const OccupancyGrid& grid,
    const std::vector<GridCell>& path,
    bool preventDiagonalCornerCutting = true);

std::vector<Point2> gridPathToWorld(
    const OccupancyGrid& grid,
    const std::vector<GridCell>& path);
```

`plan()` already fills `simplifiedPath`; call `simplifyGridPath()` directly only
for another ordered path.

## Search debug callback

```cpp
using GridSearchDebugCallback =
    std::function<void(const cv::Mat1b&)>;
```

The matrix contains `GridSearchCellState` values:

```cpp
Unseen
Open
Current
Expanded
Path
Occupied
```

The callback is synchronous. Its matrix is reusable planner storage and is only
valid for the duration of the callback.

## Occupancy visualization

Configuration types:

```cpp
OccupancyGridMarker
OccupancyGridPath
OccupancyGridRenderOptions
GridSearchStateRenderOptions
```

Rendering functions:

```cpp
cv::Mat3b renderOccupancyGrid(
    const OccupancyGrid& grid,
    const OccupancyGridRenderOptions& options = {});

void showOccupancyGrid(
    const OccupancyGrid& grid,
    const OccupancyGridRenderOptions& options = {},
    const std::string& windowName = "Occupancy Grid");

cv::Mat3b renderGridSearchState(
    const OccupancyGrid& grid,
    const cv::Mat1b& state,
    const GridCell& start,
    const GridCell& goal,
    const GridSearchStateRenderOptions& options = {});

void showGridSearchState(
    const OccupancyGrid& grid,
    const cv::Mat1b& state,
    const GridCell& start,
    const GridCell& goal,
    const GridSearchStateRenderOptions& options = {},
    const std::string& windowName = "Grid Search Debug");
```

Colors use OpenCV BGR channel order.

## Polygon-environment visualization

```cpp
void visualize(
    const PolygonEnvironment& environment,
    const Point2& start,
    const Point2& goal,
    int pixelSize = 1200,
    const std::string& outputFile = {},
    const std::vector<Point2>& path = {},
    const std::vector<Point2>& simplifiedPath = {});
```

This renderer depends on MatPlotOpenCV.

## CMake targets

Core API:

```cmake
AStarPlanner::astar_planner
```

Optional rendering APIs:

```cmake
AStarPlanner::astar_occupancy_grid_visualization
AStarPlanner::astar_planner_visualization
```

The core target is header-only and propagates the include directories and core
OpenCV/Eigen dependencies required by the public headers.
