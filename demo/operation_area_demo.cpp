// operation_area_demo.cpp - Operation-area and obstacle-clipping tutorial.
//
// This example adds a convex keep-in area, obstacles that require clipping,
// and a caller-selected planning boundary. The planning steps otherwise match
// the basic full-grid demo.

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "a_star_planner_visualization.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    using namespace astar;
    using Clock = std::chrono::steady_clock;

    bool debugVisualizationEnabled = false;
    std::string outputFile;
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const std::string argument = argv[argumentIndex];
        if (argument == "--debug")
            debugVisualizationEnabled = true;
        else if (outputFile.empty())
            outputFile = argument;
        else
        {
            std::cerr << "Usage: operation_area_demo [output-image] [--debug]\n";
            return 1;
        }
    }

    // 1. Define the keep-in operation area and obstacle polygons in world
    // coordinates. Several obstacles intentionally cross or miss the boundary.
    const Polygon operationArea{
        Point2(0.0, 0.0),
        Point2(100.0, 0.0),
        Point2(100.0, 60.0),
        Point2(0.0, 60.0)
    };

    const std::vector<Polygon> obstacles{
        {
            Point2(18.0, -8.0), Point2(28.0, -8.0),
            Point2(28.0, 36.0), Point2(18.0, 36.0)
        },
        {
            Point2(38.0, 24.0), Point2(48.0, 24.0),
            Point2(48.0, 68.0), Point2(38.0, 68.0)
        },
        {
            Point2(58.0, -6.0), Point2(68.0, -6.0),
            Point2(68.0, 38.0), Point2(58.0, 38.0)
        },
        {
            Point2(78.0, 22.0), Point2(88.0, 22.0),
            Point2(88.0, 70.0), Point2(78.0, 70.0)
        },
        {
            Point2(7.0, 22.0), Point2(14.0, 22.0),
            Point2(14.0, 32.0), Point2(7.0, 32.0)
        },
        {
            Point2(33.0, 8.0), Point2(37.0, 14.0),
            Point2(33.0, 20.0), Point2(29.0, 14.0)
        },
        {
            Point2(72.0, 42.0), Point2(76.0, 47.0),
            Point2(72.0, 52.0), Point2(69.0, 47.0)
        },
        {
            Point2(106.0, 5.0), Point2(114.0, 5.0),
            Point2(114.0, 15.0), Point2(106.0, 15.0)
        },
        {
            Point2(90.0, 65.0), Point2(105.0, 50.0),
            Point2(120.0, 65.0), Point2(105.0, 80.0)
        }
    };

    const Point2 start(5.0, 8.0);
    const Point2 goal(95.0, 52.0);

    // 2. Validate the polygons and clip obstacles to the operation area.
    const auto environmentStartTime = Clock::now();
    const PolygonEnvironment environment(operationArea, obstacles);
    const auto environmentElapsed = Clock::now() - environmentStartTime;

    // 3. Select the planning boundary before rasterization. The operation area
    // extends beyond it, but no larger intermediate grid needs to be allocated.
    constexpr double gridResolution = 1.0;
    constexpr double safetyRadius = 1.0;
    const WorldBounds planningBounds{
        Point2(3.0, 0.0),
        Point2(97.0, 60.0)
    };
    const GridGeometry planningGeometry = GridGeometry::covering(
        planningBounds, gridResolution);

    // 4. Rasterize directly into the planning geometry. Rasterization starts
    // occupied, paints the operation area free, then paints effective obstacles.
    const auto gridStartTime = Clock::now();
    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        planningGeometry, environment);
    const auto gridElapsed = Clock::now() - gridStartTime;

    // 5. Inflate occupied cells and the planning boundary. The uninflated
    // polygon grid remains available for visualization or another safety radius.
    const auto inflationStartTime = Clock::now();
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        polygonGrid, safetyRadius);
    const auto inflationElapsed = Clock::now() - inflationStartTime;

    // 6. Convert world terminals using the planning geometry.
    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning grid\n";
        return 1;
    }

    // 7. Plan with normal A* inside the selected boundary. Dijkstra and weighted
    // A* can be selected through the same options.
    const AStarGridPlanner gridPlanner;
    AStarOptions options;
    //options.algorithm = GridSearchAlgorithm::Dijkstra;
    options.algorithm = GridSearchAlgorithm::AStar;
    //options.tieBreakPolicy = AStarTieBreakPolicy::LargerGOnly;

    GridSearchDebugCallback debugCallback;
    if (debugVisualizationEnabled)
    {
        std::cout
            << "Debug search animation enabled\n"
            << "  white: unseen free cell\n"
            << "  yellow: open/frontier cell\n"
            << "  orange: cell currently being expanded\n"
            << "  light blue: expanded cell\n"
            << "  blue: final path\n"
            << "  black: occupied cell\n"
            << "  green/red markers: start/goal\n";
        GridSearchStateRenderOptions debugView;
        debugView.pixelsPerCell = 8;
        debugCallback = [&, debugView, frame = std::size_t{ 0 }](
            const cv::Mat1b& state) mutable
        {
            constexpr std::size_t expansionsPerFrame = 10;
            constexpr int frameDelayMilliseconds = 10;
            if (++frame % expansionsPerFrame != 0)
                return;
            showGridSearchState(
                planningGrid,
                state,
                *startCell,
                *goalCell,
                debugView,
                "AStarPlanner Search Debug");
            cv::waitKey(frameDelayMilliseconds);
        };
    }
    const auto planningStartTime = Clock::now();
    const GridPlanResult plan = gridPlanner.plan(
        planningGrid, *startCell, *goalCell, options, debugCallback);
    const auto planningElapsed = Clock::now() - planningStartTime;
    if (!plan.succeeded())
    {
        std::cerr << "A* grid planner failed: "
                  << gridPlanStatusName(plan.status) << '\n';
        return 1;
    }

    std::cout << '\n' << gridSearchAlgorithmName(options.algorithm);
    if (options.algorithm == GridSearchAlgorithm::WeightedAStar)
        std::cout << " (weight " << options.heuristicWeight << ')';
    std::cout << " planning completed in " << std::fixed << std::setprecision(3)
              << std::chrono::duration<double, std::milli>(planningElapsed).count()
              << " ms"
              << (debugVisualizationEnabled ? " including debug display\n" : "\n")
              << "  Path cost: " << plan.diagnostics.pathCost << '\n'
              << "  Original path: " << plan.path.size() << " cells\n"
              << "  Simplified path: " << plan.simplifiedPath.size()
              << " waypoints\n"
              << "  Simplification time: "
              << plan.diagnostics.pathSimplificationMilliseconds << " ms\n\n"
              << "Search diagnostics\n"
              << "  Tie-break: "
              << aStarTieBreakPolicyName(options.tieBreakPolicy) << '\n'
              << "  Expanded: " << plan.diagnostics.expandedNodes << '\n'
              << "  Generated: " << plan.diagnostics.generatedNodes << '\n'
              << "  Peak open set: " << plan.diagnostics.peakOpenSetSize << "\n\n"
              << "Occupancy preprocessing\n"
              << "  Input obstacles: " << obstacles.size() << '\n'
              << "  Effective obstacles after clipping: "
              << environment.effectiveObstacles().size() << '\n'
              << "  Clipped obstacle overlays: "
              << environment.clippedObstacles().size() << '\n'
              << "  Polygon grid: " << polygonGrid.width() << " x "
              << polygonGrid.height() << " cells at "
              << polygonGrid.geometry().resolution() << " world units/cell\n"
              << "  Planning grid: " << safetyRadius
              << " world units of safety inflation\n"
              << "  Environment validation: "
              << std::chrono::duration<double, std::milli>(
                     environmentElapsed).count()
              << " ms\n"
              << "  Polygon rasterization: "
              << std::chrono::duration<double, std::milli>(gridElapsed).count()
              << " ms\n"
              << "  Safety inflation: "
              << std::chrono::duration<double, std::milli>(inflationElapsed).count()
              << " ms\n"
              << "  Total preprocessing: "
              << std::chrono::duration<double, std::milli>(
                     environmentElapsed + gridElapsed + inflationElapsed).count()
              << " ms\n";
    // 8. Convert both paths to world cell centers and render them.
    const std::vector<Point2> worldPath = gridPathToWorld(planningGrid, plan.path);
    const std::vector<Point2> simplifiedWorldPath =
        gridPathToWorld(planningGrid, plan.simplifiedPath);
    visualize(
        environment,
        start,
        goal,
        1200,
        outputFile,
        worldPath,
        simplifiedWorldPath);

    OccupancyGridRenderOptions gridView;
    gridView.pixelsPerCell = 8;
    gridView.paths.push_back({ plan.path, cv::Scalar(255, 120, 0), 2 });
    gridView.paths.push_back({
        plan.simplifiedPath, cv::Scalar(0, 140, 255), 3 });
    gridView.markers.push_back({ *startCell, cv::Scalar(0, 180, 0), 3 });
    gridView.markers.push_back({ *goalCell, cv::Scalar(0, 0, 255), 3 });

    showOccupancyGrid(planningGrid, gridView, "AStarPlanner Inflated Planning Grid");
    cv::waitKey(0);
    return 0;
}
