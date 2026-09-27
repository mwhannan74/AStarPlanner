// robot_planning_demo.cpp - Robot-local planning-bound tutorial.
//
// This example uses a robot position and forward/side/behind distances to
// create a compact planning grid. The grid and planner remain unaware of the
// robot-specific meaning of those bounds.

#include "a_star_grid_planner.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
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
                if (normalizedX * normalizedX +
                    normalizedY * normalizedY <= 1.0)
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
        appendEllipsePoints(points, Point2(-17.0, -12.0), 3.0, 4.0);
        appendEllipsePoints(points, Point2(-14.0, 34.0), 4.0, 3.0);
        appendEllipsePoints(points, Point2(-2.0, -25.0), 3.5, 3.5);
        appendEllipsePoints(points, Point2(5.0, 18.0), 4.0, 3.0);
        appendEllipsePoints(points, Point2(10.0, 2.0), 4.5, 4.0);
        appendEllipsePoints(points, Point2(17.0, -17.0), 4.0, 3.5);
        appendEllipsePoints(points, Point2(21.0, 20.0), 3.5, 4.5);
        appendEllipsePoints(points, Point2(26.0, 5.0), 4.0, 4.5);
        appendEllipsePoints(points, Point2(34.0, -7.0), 3.5, 3.0);
        appendEllipsePoints(points, Point2(38.0, 16.0), 4.5, 3.0);
        appendEllipsePoints(points, Point2(40.0, -35.0), 3.0, 4.0);
        appendEllipsePoints(points, Point2(42.0, 36.0), 3.0, 3.5);
        return points;
    }

    std::vector<Point2> randomPerceptionPoints(std::uint32_t seed)
    {
        std::mt19937 generator(seed);
        std::uniform_real_distribution<double> worldX(-18.0, 44.0);
        std::uniform_real_distribution<double> worldY(-39.0, 39.0);
        std::uniform_real_distribution<double> worldRadius(2.5, 5.5);
        std::uniform_real_distribution<double> pathFraction(0.15, 0.85);
        std::uniform_real_distribution<double> pathOffset(-7.0, 7.0);
        std::uniform_real_distribution<double> pathRadius(3.0, 5.5);

        const Point2 start(0.5, 0.5);
        const Point2 goal(45.5, 8.5);
        const Point2 startToGoal = goal - start;
        const Point2 pathNormal(
            -startToGoal.y() / startToGoal.norm(),
            startToGoal.x() / startToGoal.norm());

        std::vector<Point2> points;

        // Background blobs can appear anywhere in the usable local world.
        // Keep them clear of the terminals so a random sample cannot make the
        // request invalid before planning begins.
        constexpr int backgroundBlobCount = 14;
        for (int blobIndex = 0; blobIndex < backgroundBlobCount;)
        {
            const Point2 center(worldX(generator), worldY(generator));
            const double xRadius = worldRadius(generator);
            const double yRadius = worldRadius(generator);
            const double terminalClearance =
                std::max(xRadius, yRadius) + 4.0;
            if ((center - start).norm() < terminalClearance ||
                (center - goal).norm() < terminalClearance)
            {
                continue;
            }

            appendEllipsePoints(
                points,
                center,
                xRadius,
                yRadius);
            ++blobIndex;
        }

        // These blobs are independently positioned near the nominal direct
        // route. Their changing locations make perception affect the planned
        // path instead of serving only as background decoration.
        constexpr int pathBlobCount = 5;
        for (int blobIndex = 0; blobIndex < pathBlobCount; ++blobIndex)
        {
            const double fraction = pathFraction(generator);
            const Point2 center = start + fraction * startToGoal +
                pathOffset(generator) * pathNormal;
            const double xRadius = pathRadius(generator);
            const double yRadius = pathRadius(generator);
            appendEllipsePoints(
                points,
                center,
                xRadius,
                yRadius);
        }
        return points;
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
                << "Usage: robot_planning_demo [output-image] [--debug] "
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

    // 1. The application defines what "local" means for this robot. Positive
    // x is forward, negative x is behind, and y is the symmetric side extent.
    const Point2 robotPosition(0.5, 0.5);
    const LocalPlanningRegion localRegion =
        LocalPlanningRegion::forwardSideBehind(
            50.0,  // forward distance
            50.0,  // distance to either side
            25.0); // behind distance: 50% of forward
    const WorldBounds planningBounds =
        localRegion.boundsAround(robotPosition);

    constexpr double gridResolution = 1.0;
    constexpr double safetyRadius = 1.0;
    const GridGeometry planningGeometry = GridGeometry::covering(
        planningBounds, gridResolution);

    // 2. The operation area is a convex keep-in boundary. Rectangular polygons
    // represent prior keep-out information spread across the local world.
    const Polygon operationArea{
        Point2(-22.0, -38.0), Point2(-12.0, -48.0),
        Point2(32.0, -48.0), Point2(49.0, -31.0),
        Point2(49.0, 32.0), Point2(33.0, 48.0),
        Point2(-12.0, 48.0), Point2(-22.0, 38.0)
    };

    const std::vector<Polygon> obstacles{
        {
            Point2(-19.0, -32.0), Point2(-11.0, -32.0),
            Point2(-11.0, -23.0), Point2(-19.0, -23.0)
        },
        {
            Point2(-18.0, 16.0), Point2(-9.0, 16.0),
            Point2(-9.0, 25.0), Point2(-18.0, 25.0)
        },
        {
            Point2(3.0, -39.0), Point2(13.0, -39.0),
            Point2(13.0, -31.0), Point2(3.0, -31.0)
        },
        {
            Point2(8.0, 29.0), Point2(18.0, 29.0),
            Point2(18.0, 37.0), Point2(8.0, 37.0)
        },
        {
            Point2(21.0, -7.0), Point2(28.0, -7.0),
            Point2(28.0, -1.0), Point2(21.0, -1.0)
        },
        {
            Point2(28.0, -29.0), Point2(37.0, -29.0),
            Point2(37.0, -19.0), Point2(28.0, -19.0)
        },
        {
            Point2(32.0, 22.0), Point2(42.0, 22.0),
            Point2(42.0, 31.0), Point2(32.0, 31.0)
        }
    };

    const auto environmentStartTime = Clock::now();
    const PolygonEnvironment environment(operationArea, obstacles);
    const auto environmentElapsed = Clock::now() - environmentStartTime;

    // 3. Perception-style obstacle points form irregular blobs throughout the
    // same world frame. Both occupancy sources use the exact planning geometry.
    const bool randomizedPerception =
        randomSeedRequested || explicitSeedProvided;
    const std::vector<Point2> detectedPoints = randomizedPerception
        ? randomPerceptionPoints(perceptionSeed)
        : fixedPerceptionPoints();

    const auto polygonStartTime = Clock::now();
    const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
        planningGeometry, environment);
    const auto polygonElapsed = Clock::now() - polygonStartTime;

    const auto pointStartTime = Clock::now();
    const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
        planningGeometry, detectedPoints);
    const auto pointElapsed = Clock::now() - pointStartTime;

    // 4. Fuse the polygon and perception occupancy, then inflate the combined
    // result once for the required safety radius.
    const auto fusionStartTime = Clock::now();
    const OccupancyGrid fusedGrid = OccupancyGridFusion::occupiedUnion(
        polygonGrid, perceptionGrid);
    const auto fusionElapsed = Clock::now() - fusionStartTime;

    const auto inflationStartTime = Clock::now();
    const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
        fusedGrid, safetyRadius);
    const auto inflationElapsed = Clock::now() - inflationStartTime;

    // 5. In this example the robot position is also the start. The goal is in
    // front of the robot and inside the selected planning horizon.
    const Point2 goal(45.5, 8.5);
    const auto startCell = planningGrid.geometry().worldToCell(robotPosition);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Robot position or goal is outside the planning grid\n";
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
            constexpr std::size_t expansionsPerFrame = 5;
            constexpr int frameDelayMilliseconds = 15;
            if (++frame % expansionsPerFrame != 0)
                return;
            showGridSearchState(
                planningGrid,
                state,
                *startCell,
                *goalCell,
                debugView,
                "Robot-local Search Debug");
            cv::waitKey(frameDelayMilliseconds);
        };
    }

    // 6. The normal grid planner consumes the resulting occupancy grid; it
    // does not need to know that the bounds were selected around a robot.
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

    std::cout << "\nRobot-local planning region\n"
              << std::fixed << std::setprecision(3)
              << "  Robot/start: (" << robotPosition.x() << ", "
              << robotPosition.y() << ")\n"
              << "  Forward / side / behind: "
              << localRegion.forwardDistance() << " / "
              << localRegion.sideDistance() << " / "
              << localRegion.behindDistance() << " world units\n"
              << "  Bounds: [(" << planningBounds.minimum.x() << ", "
              << planningBounds.minimum.y() << "), ("
              << planningBounds.maximum.x() << ", "
              << planningBounds.maximum.y() << ")]\n"
              << "  Planning grid: " << planningGrid.width() << " x "
              << planningGrid.height() << " cells at "
              << gridResolution << " world units/cell\n\n"
              << "A* planning\n"
              << "  Planning time: "
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
              << "  Polygon obstacles: "
              << environment.effectiveObstacles().size() << '\n'
              << "  Perception samples: " << detectedPoints.size() << '\n'
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

    // 7. The green marker is the robot/start, red is the goal, blue is the
    // original cell path, and orange is the collision-checked simplified path.
    OccupancyGridRenderOptions view;
    view.pixelsPerCell = 8;
    view.paths.push_back({ plan.path, cv::Scalar(255, 120, 0), 2 });
    view.paths.push_back({
        plan.simplifiedPath, cv::Scalar(0, 140, 255), 3 });
    view.markers.push_back({ *startCell, cv::Scalar(0, 180, 0), 5 });
    view.markers.push_back({ *goalCell, cv::Scalar(0, 0, 255), 5 });

    const cv::Mat3b image = renderOccupancyGrid(planningGrid, view);
    if (!outputFile.empty() && !cv::imwrite(outputFile, image))
    {
        std::cerr << "Could not write visualization image: "
                  << outputFile << '\n';
        return 1;
    }

    cv::imshow("Robot-local Planning", image);
    cv::waitKey(0);
    return 0;
}
