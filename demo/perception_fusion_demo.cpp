// perception_fusion_demo.cpp - Polygon and point-occupancy fusion tutorial.
//
// This example creates one caller-selected planning geometry, independently
// rasterizes polygon keep-out zones and perception point blobs, fuses their
// occupancy, inflates once, plans, and visualizes every preprocessing stage.

#include "a_star_grid_planner.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <opencv2/imgcodecs.hpp>

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using namespace astar;

    void appendEllipsePoints(
        std::vector<Point2>& points,
        const Point2& center,
        double xRadius,
        double yRadius)
    {
        const int xExtent = static_cast<int>(std::ceil(xRadius));
        const int yExtent = static_cast<int>(std::ceil(yRadius));
        for (int yOffset = -yExtent; yOffset <= yExtent; ++yOffset)
        {
            for (int xOffset = -xExtent; xOffset <= xExtent; ++xOffset)
            {
                const double normalizedX =
                    static_cast<double>(xOffset) / xRadius;
                const double normalizedY =
                    static_cast<double>(yOffset) / yRadius;
                if (normalizedX * normalizedX + normalizedY * normalizedY <= 1.0)
                {
                    points.emplace_back(
                        center.x() + xOffset,
                        center.y() + yOffset);
                }
            }
        }
    }

    std::vector<Point2> fixedPerceptionPoints()
    {
        std::vector<Point2> points;
        appendEllipsePoints(points, Point2(30.5, 37.5), 4.5, 3.0);
        appendEllipsePoints(points, Point2(49.5, 11.5), 4.0, 3.5);
        appendEllipsePoints(points, Point2(68.5, 37.5), 3.5, 4.5);
        appendEllipsePoints(points, Point2(72.5, 35.5), 2.5, 2.0);
        points.emplace_back(100.0, 100.0); // Ignored: outside the grid.
        return points;
    }

    std::vector<Point2> randomPerceptionPoints(std::uint32_t seed)
    {
        std::mt19937 generator(seed);
        std::uniform_real_distribution<double> yCenter(7.5, 42.5);
        std::uniform_real_distribution<double> xRadius(2.5, 4.5);
        std::uniform_real_distribution<double> yRadius(2.0, 4.5);

        // Place one blob in each open band between polygon walls. Positions and
        // sizes vary, but no individual blob can span a complete passage.
        const std::vector<std::pair<double, double>> xBands{
            { 9.5, 14.5 },
            { 27.5, 33.5 },
            { 45.5, 52.5 },
            { 64.5, 72.5 }
        };

        std::vector<Point2> points;
        for (const auto& [minimumX, maximumX] : xBands)
        {
            std::uniform_real_distribution<double> xCenter(minimumX, maximumX);
            appendEllipsePoints(
                points,
                Point2(xCenter(generator), yCenter(generator)),
                xRadius(generator),
                yRadius(generator));
        }
        return points;
    }

    cv::Mat3b renderTitledGrid(
        const OccupancyGrid& grid,
        const OccupancyGridRenderOptions& options,
        const std::string& title)
    {
        constexpr int titleHeight = 34;
        const cv::Mat3b gridImage = renderOccupancyGrid(grid, options);
        cv::Mat3b panel;
        cv::copyMakeBorder(
            gridImage,
            panel,
            titleHeight,
            0,
            0,
            0,
            cv::BORDER_CONSTANT,
            cv::Scalar(245, 245, 245));
        cv::putText(
            panel,
            title,
            cv::Point(10, 23),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(30, 30, 30),
            1,
            cv::LINE_AA);
        return panel;
    }
}

int main(int argc, char* argv[])
{
    using namespace astar;
    using Clock = std::chrono::steady_clock;

    bool debugVisualizationEnabled = false;
    bool randomSeedRequested = false;
    bool explicitSeedProvided = false;
    std::uint32_t perceptionSeed = 0;
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
                perceptionSeed);
            if (error != std::errc{} ||
                end != seedArgument.data() + seedArgument.size())
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
                << "Usage: perception_fusion_demo [output-image] [--debug] "
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
        perceptionSeed = std::random_device{}();

    // 1. The application selects the complete planning boundary and resolution.
    // Both occupancy sources must use this exact geometry so they describe the
    // same cell lattice and can be fused without resampling.
    const WorldBounds planningBounds{
        Point2(0.0, 0.0), Point2(80.0, 50.0)
    };
    constexpr double gridResolution = 1.0;
    constexpr double safetyRadius = 1.0;
    const GridGeometry planningGeometry = GridGeometry::covering(
        planningBounds, gridResolution);

    // 2. Polygon obstacles represent persistent keep-out zones or prior map
    // knowledge. These alternating walls leave a navigable route through the map.
    const std::vector<Polygon> obstacles{
        {
            Point2(18.0, 0.0), Point2(25.0, 0.0),
            Point2(25.0, 29.0), Point2(18.0, 29.0)
        },
        {
            Point2(36.0, 20.0), Point2(43.0, 20.0),
            Point2(43.0, 50.0), Point2(36.0, 50.0)
        },
        {
            Point2(55.0, 0.0), Point2(62.0, 0.0),
            Point2(62.0, 29.0), Point2(55.0, 29.0)
        }
    };
    const auto environmentStartTime = Clock::now();
    const PolygonEnvironment environment(obstacles);
    const auto environmentElapsed = Clock::now() - environmentStartTime;

    // 3. Perception supplies world-coordinate obstacle samples. The default
    // layout is fixed for the tutorial; --random and --seed N generate new blob
    // positions and sizes. PointRasterizer accepts any vector of world points.
    const bool randomizedPerception = randomSeedRequested || explicitSeedProvided;
    const std::vector<Point2> perceptionPoints = randomizedPerception
        ? randomPerceptionPoints(perceptionSeed)
        : fixedPerceptionPoints();

    // 4. Rasterize both sources independently, fuse occupied cells, then apply
    // the safety margin once to the combined result.
    const auto polygonStartTime = Clock::now();
    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        planningGeometry, environment);
    const auto polygonElapsed = Clock::now() - polygonStartTime;

    const auto pointStartTime = Clock::now();
    const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
        planningGeometry, perceptionPoints);
    const auto pointElapsed = Clock::now() - pointStartTime;

    const auto fusionStartTime = Clock::now();
    const OccupancyGrid fusedGrid = OccupancyGridFusion::occupiedUnion(
        polygonGrid, perceptionGrid);
    const auto fusionElapsed = Clock::now() - fusionStartTime;

    const auto inflationStartTime = Clock::now();
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        fusedGrid, safetyRadius);
    const auto inflationElapsed = Clock::now() - inflationStartTime;

    // 5. Convert world terminals using that same geometry and plan normally.
    const Point2 start(4.5, 6.5);
    const Point2 goal(75.5, 43.5);
    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning grid\n";
        return 1;
    }

    GridSearchDebugCallback debugCallback;
    if (debugVisualizationEnabled)
    {
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
                "Perception Fusion Search Debug");
            cv::waitKey(frameDelayMilliseconds);
        };
    }

    const AStarGridPlanner planner;
    const auto planningStartTime = Clock::now();
    const GridPlanResult plan = planner.plan(
        planningGrid, *startCell, *goalCell, {}, debugCallback);
    const auto planningElapsed = Clock::now() - planningStartTime;
    if (!plan.succeeded())
    {
        std::cerr << "A* grid planner failed: "
                  << gridPlanStatusName(plan.status) << '\n';
        return 1;
    }

    std::cout << "\nPerception layout: ";
    if (randomizedPerception)
        std::cout << "generated with seed " << perceptionSeed << '\n';
    else
        std::cout << "fixed tutorial blobs\n";

    std::cout << "A* planning completed in " << std::fixed
              << std::setprecision(3)
              << std::chrono::duration<double, std::milli>(
                     planningElapsed).count()
              << " ms"
              << (debugVisualizationEnabled ? " including debug display\n" : "\n")
              << "  Path cost: " << plan.diagnostics.pathCost << '\n'
              << "  Original path: " << plan.path.size() << " cells\n"
              << "  Simplified path: " << plan.simplifiedPath.size()
              << " waypoints\n"
              << "  Simplification time: "
              << plan.diagnostics.pathSimplificationMilliseconds << " ms\n\n"
              << "Occupancy preprocessing\n"
              << "  Planning grid: " << planningGrid.width() << " x "
              << planningGrid.height() << " cells at "
              << planningGrid.geometry().resolution() << " world units/cell\n"
              << "  Polygon obstacles: " << obstacles.size() << '\n'
              << "  Perception samples: " << perceptionPoints.size() << '\n'
              << "  Environment validation: "
              << std::chrono::duration<double, std::milli>(
                     environmentElapsed).count() << " ms\n"
              << "  Polygon rasterization: "
              << std::chrono::duration<double, std::milli>(
                     polygonElapsed).count() << " ms\n"
              << "  Point rasterization: "
              << std::chrono::duration<double, std::milli>(
                     pointElapsed).count() << " ms\n"
              << "  Occupancy fusion: "
              << std::chrono::duration<double, std::milli>(
                     fusionElapsed).count() << " ms\n"
              << "  Safety inflation: "
              << std::chrono::duration<double, std::milli>(
                     inflationElapsed).count() << " ms\n"
              << "  Total preprocessing: "
              << std::chrono::duration<double, std::milli>(
                     environmentElapsed + polygonElapsed + pointElapsed +
                     fusionElapsed + inflationElapsed).count() << " ms\n";

    // 6. Render a four-panel view of the pipeline. The bottom-right panel shows
    // the inflated grid with the original path, simplified path, and terminals.
    OccupancyGridRenderOptions sourceView;
    sourceView.pixelsPerCell = 8;

    OccupancyGridRenderOptions planningView = sourceView;
    planningView.paths.push_back({
        plan.path, cv::Scalar(255, 120, 0), 2 });
    planningView.paths.push_back({
        plan.simplifiedPath, cv::Scalar(0, 140, 255), 3 });
    planningView.markers.push_back({
        *startCell, cv::Scalar(0, 180, 0), 3 });
    planningView.markers.push_back({
        *goalCell, cv::Scalar(0, 0, 255), 3 });

    const cv::Mat3b polygonPanel = renderTitledGrid(
        polygonGrid, sourceView, "1. Polygon occupancy");
    const cv::Mat3b perceptionPanel = renderTitledGrid(
        perceptionGrid, sourceView, "2. Perception point occupancy");
    const cv::Mat3b fusedPanel = renderTitledGrid(
        fusedGrid, sourceView, "3. Occupied-union fusion");
    const cv::Mat3b planningPanel = renderTitledGrid(
        planningGrid, planningView, "4. Inflated grid and planned paths");

    cv::Mat3b topRow;
    cv::Mat3b bottomRow;
    cv::Mat3b pipelineImage;
    cv::hconcat(polygonPanel, perceptionPanel, topRow);
    cv::hconcat(fusedPanel, planningPanel, bottomRow);
    cv::vconcat(topRow, bottomRow, pipelineImage);

    if (!outputFile.empty() && !cv::imwrite(outputFile, pipelineImage))
    {
        std::cerr << "Could not write visualization image: "
                  << outputFile << '\n';
        return 1;
    }
    cv::imshow("Polygon and Perception Occupancy Pipeline", pipelineImage);
    cv::waitKey(0);
    return 0;
}
