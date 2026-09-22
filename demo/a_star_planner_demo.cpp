// a_star_planner_demo.cpp - Basic full-grid planning tutorial.
//
// This example has no operation area. It shows how to validate world-coordinate
// obstacles, choose an explicit master-grid extent, rasterize it, plan on the
// complete grid, and convert the resulting cell path back to world coordinates.

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "a_star_planner_visualization.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace astar;

int main(int argc, char* argv[])
{
    using Clock = std::chrono::steady_clock;

    // 1. Define obstacle polygons in world coordinates. A fixed random seed
    // keeps the tutorial repeatable while producing a nonuniform obstacle field.
    const int rows = 5;
    const int columns = 5;
    const double obstacleSize = 5.0;
    const double gap = 5.0;
    const double noise = gap * 0.5;

    std::mt19937 generator(7);
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
    const PolygonEnvironment environment(obstacles);
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
    const double mapPadding = gridResolution;
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

    // 4. Rasterize effective obstacles into a free master grid. With no
    // operation area, only pixels covered by obstacles become occupied.
    const auto gridStartTime = Clock::now();
    const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
        gridGeometry, environment.effectiveObstacles());
    const auto gridElapsed = Clock::now() - gridStartTime;

    // 5. Choose the planning domain. This basic example uses the complete
    // master map so a valid detour cannot be removed by an unsafe ROI crop.
    const OccupancyGrid& planningGrid = masterGrid;

    std::cout << "Environment has " << environment.effectiveObstacles().size()
              << " effective obstacles\n";
    std::cout << "Master occupancy grid: "
              << masterGrid.width() << " x " << masterGrid.height()
              << " cells at " << masterGrid.geometry().resolution()
              << " world units per cell\n";
    std::cout << "Planning grid (full master map): "
              << planningGrid.width() << " x " << planningGrid.height()
              << " cells\n";
    std::cout << std::fixed << std::setprecision(3)
              << "Grid rasterization: "
              << std::chrono::duration<double, std::milli>(gridElapsed).count()
              << " ms\n";
    const std::string outputFile = argc > 1 ? argv[1] : "";

    // 6. Convert world terminals to cells in the selected planning grid.
    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning grid\n";
        return 1;
    }

    // 7. Run A*. Default options use eight-connected movement and prevent
    // diagonal corner cutting.
    const AStarGridPlanner gridPlanner;
    const auto planningStartTime = Clock::now();
    const GridPlanResult plan = gridPlanner.plan(
        planningGrid, *startCell, *goalCell);
    const auto planningElapsed = Clock::now() - planningStartTime;
    std::cout << "A* planning: "
              << std::chrono::duration<double, std::milli>(planningElapsed).count()
              << " ms\n";
    if (!plan.succeeded())
    {
        std::cerr << "A* grid planner failed: "
                  << gridPlanStatusName(plan.status) << '\n';
        return 1;
    }
    // 8. Convert cell centers back to world coordinates and render both views.
    const std::vector<Point2> worldPath = gridPathToWorld(planningGrid, plan.path);
    visualize(environment, start, goal, 1200, outputFile, worldPath);

    OccupancyGridRenderOptions gridView;
    gridView.pixelsPerCell = 8;
    gridView.paths.push_back({ plan.path, cv::Scalar(255, 120, 0), 2 });
    gridView.markers.push_back({ *startCell, cv::Scalar(0, 180, 0), 3 });
    gridView.markers.push_back({ *goalCell, cv::Scalar(0, 0, 255), 3 });

    std::cout << "A* path: "
              << plan.path.size() << " cells\n";
    showOccupancyGrid(planningGrid, gridView, "AStarPlanner Master Grid");

    cv::waitKey(0);
    return 0;
}
