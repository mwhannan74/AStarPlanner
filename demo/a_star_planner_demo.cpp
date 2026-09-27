// a_star_planner_demo.cpp - Basic full-grid planning tutorial.
//
// This example has no operation area. It shows how to validate world-coordinate
// obstacles, choose an explicit planning extent, rasterize and inflate it,
// plan on the complete grid, and convert the resulting path to world coordinates.

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "a_star_planner_visualization.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace astar;

int main(int argc, char* argv[])
{
    using Clock = std::chrono::steady_clock;

    bool debugVisualizationEnabled = false;
    bool randomSeedRequested = false;
    bool explicitSeedProvided = false;
    std::uint32_t environmentSeed = 7;
    std::string outputFile;
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const std::string argument = argv[argumentIndex];
        if (argument == "--debug")
            debugVisualizationEnabled = true;
        else if (argument == "--random")
            randomSeedRequested = true;
        else if (argument == "--seed")
        {
            if (++argumentIndex >= argc)
            {
                std::cerr << "--seed requires an unsigned integer\n";
                return 1;
            }

            const std::string seedArgument = argv[argumentIndex];
            const auto [end, error] = std::from_chars(
                seedArgument.data(),
                seedArgument.data() + seedArgument.size(),
                environmentSeed);
            if (error != std::errc{} || end != seedArgument.data() + seedArgument.size())
            {
                std::cerr << "Invalid seed: " << seedArgument << '\n';
                return 1;
            }
            explicitSeedProvided = true;
        }
        else if (outputFile.empty())
            outputFile = argument;
        else
        {
            std::cerr
                << "Usage: a_star_planner_demo [output-image] [--debug] "
                   "[--random | --seed N]\n";
            return 1;
        }
    }
    if (randomSeedRequested && explicitSeedProvided)
    {
        std::cerr << "--random and --seed cannot be used together\n";
        return 1;
    }
    if (randomSeedRequested)
        environmentSeed = std::random_device{}();

    // 1. Define obstacle polygons in world coordinates. The default fixed seed
    // keeps the tutorial repeatable. Use --random for a fresh obstacle field or
    // --seed N to reproduce a particular field.
    const int rows = 5;
    const int columns = 5;
    const double obstacleSize = 5.0;
    const double gap = 5.0;
    const double noise = gap * 0.5;

    std::mt19937 generator(environmentSeed);
    std::uniform_real_distribution<double> noiseDistribution(-noise, noise);

    std::vector<Polygon> obstacles;
    for (int row = 0; row < rows; ++row)
    {
        for (int column = 0; column < columns; ++column)
        {
            const double x = column * (obstacleSize + gap) + noiseDistribution(generator);
            const double y = row * (obstacleSize + gap) + noiseDistribution(generator);
            obstacles.push_back({
                Point2(x, y),
                Point2(x + obstacleSize, y),
                Point2(x + obstacleSize, y + obstacleSize),
                Point2(x, y + obstacleSize)
            });
        }
    }

    // 2. Validate and normalize the polygon environment.
    const auto environmentStartTime = Clock::now();
    const PolygonEnvironment environment(obstacles);
    const auto environmentElapsed = Clock::now() - environmentStartTime;
    const WorldBounds obstacleBounds = GridGeometry::boundingBox(
        environment.effectiveObstacles());

    // 3. Select world-coordinate terminals and a grid extent that contains the
    // obstacles and terminals. Resolution is world units per grid cell.
    std::uniform_real_distribution<double> terminalOffset(0.0, 2.0 * gap);
    const Point2 start(
        obstacleBounds.minimum.x() - gap - terminalOffset(generator),
        obstacleBounds.minimum.y() - gap - terminalOffset(generator));
    const Point2 goal(
        obstacleBounds.maximum.x() + gap + terminalOffset(generator),
        obstacleBounds.maximum.y() + gap + terminalOffset(generator));

    constexpr double gridResolution = 1.0;
    constexpr double safetyRadius = 1.0;
    const double mapPadding = safetyRadius + gridResolution;
    const WorldBounds mapBounds{
        Point2(
            std::min(obstacleBounds.minimum.x(), start.x()) - mapPadding,
            std::min(obstacleBounds.minimum.y(), start.y()) - mapPadding),
        Point2(
            std::max(obstacleBounds.maximum.x(), goal.x()) + mapPadding,
            std::max(obstacleBounds.maximum.y(), goal.y()) + mapPadding)
    };
    const GridGeometry gridGeometry = GridGeometry::alignedCovering(
        mapBounds, gridResolution);

    // 4. Rasterize effective obstacles into a free polygon grid. With no
    // operation area, only pixels covered by obstacles become occupied.
    const auto gridStartTime = Clock::now();
    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        gridGeometry, environment);
    const auto gridElapsed = Clock::now() - gridStartTime;

    // 5. Inflate obstacles and the map boundary into a separate planning grid.
    // This example treats the planning-grid edge as the limit of known space,
    // so enforcing clearance from it is intentional. The raw polygon grid remains
    // available for other clearance choices.
    const auto inflationStartTime = Clock::now();
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        polygonGrid, safetyRadius);
    const auto inflationElapsed = Clock::now() - inflationStartTime;

    // 6. Convert world terminals to cells in the inflated planning grid.
    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning grid\n";
        return 1;
    }

    // 7. Run A*. Dijkstra and weighted A* use the same planner and can be selected
    // below. Tie-breaking only orders equal-priority candidates; larger-g-first
    // policies can help some difficult maps but should be benchmarked for the
    // intended workload.
    const AStarGridPlanner gridPlanner;
    AStarOptions options;
    //options.algorithm = GridSearchAlgorithm::Dijkstra;
    options.algorithm = GridSearchAlgorithm::AStar;
    //options.algorithm = GridSearchAlgorithm::WeightedAStar;
    options.heuristicWeight = 2.0; // only used by WeightedAStar
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
            constexpr std::size_t expansionsPerFrame = 1;
            constexpr int frameDelayMilliseconds = 15;
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

    std::cout << "\nEnvironment seed: " << environmentSeed << '\n'
              << gridSearchAlgorithmName(options.algorithm);
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
              << "  Effective obstacles: "
              << environment.effectiveObstacles().size() << '\n'
              << "  Polygon grid: " << polygonGrid.width() << " x "
              << polygonGrid.height() << " cells at "
              << polygonGrid.geometry().resolution() << " world units/cell\n"
              << "  Planning grid: full map, " << safetyRadius
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
    // 8. Convert cell centers back to world coordinates and render both views.
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
