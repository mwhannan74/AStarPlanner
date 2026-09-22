// a_star_planner_demo.cpp - Environment demo with 25 non-overlapping obstacles.
// Build with the a_star_planner_demo CMake target; see README.md for instructions.

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "a_star_planner_visualization.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace astar;

int main(int argc, char* argv[])
{
    using Clock = std::chrono::steady_clock;

    const int rows = 5;
    const int columns = 5;
    const double obstacleSize = 5.0;
    const double gap = 5.0;
    const double noise = gap * 0.5;

    std::mt19937 generator(std::random_device{}());
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

    AStarPlanner planner(obstacles);

    double minimumX = std::numeric_limits<double>::infinity();
    double minimumY = std::numeric_limits<double>::infinity();
    double maximumX = -std::numeric_limits<double>::infinity();
    double maximumY = -std::numeric_limits<double>::infinity();
    for (const auto& obstacle : obstacles)
    {
        for (const auto& point : obstacle)
        {
            minimumX = std::min(minimumX, point.x());
            minimumY = std::min(minimumY, point.y());
            maximumX = std::max(maximumX, point.x());
            maximumY = std::max(maximumY, point.y());
        }
    }

    std::uniform_real_distribution<double> terminalOffset(0.0, 2.0 * gap);
    const Point2 start(
        minimumX - gap - terminalOffset(generator),
        minimumY - gap - terminalOffset(generator));
    const Point2 goal(
        maximumX + gap + terminalOffset(generator),
        maximumY + gap + terminalOffset(generator));

    constexpr double gridResolution = 1.0;
    const double mapPadding = gridResolution;
    const WorldBounds mapBounds{
        Point2(
            std::min(minimumX, start.x()) - mapPadding,
            std::min(minimumY, start.y()) - mapPadding),
        Point2(
            std::max(maximumX, goal.x()) + mapPadding,
            std::max(maximumY, goal.y()) + mapPadding)
    };
    const GridGeometry gridGeometry = GridGeometry::alignedCovering(
        mapBounds, gridResolution);
    const auto gridStartTime = Clock::now();
    const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
        gridGeometry, planner.obstacles());
    const auto gridElapsed = Clock::now() - gridStartTime;
    const double planningPadding = 0.5 * gridResolution;
    const OccupancyGrid planningGrid = masterGrid.subgrid(
        WorldBounds{
            Point2(
                std::min(start.x(), goal.x()) - planningPadding,
                std::min(start.y(), goal.y()) - planningPadding),
            Point2(
                std::max(start.x(), goal.x()) + planningPadding,
                std::max(start.y(), goal.y()) + planningPadding)
        });

    std::cout << "Environment has " << planner.obstacles().size()
              << " effective obstacles\n";
    std::cout << "Master occupancy grid: "
              << masterGrid.width() << " x " << masterGrid.height()
              << " cells at " << masterGrid.geometry().resolution()
              << " world units per cell\n";
    std::cout << "Planning ROI: "
              << planningGrid.width() << " x " << planningGrid.height()
              << " cells, master offset ("
              << planningGrid.masterCellOffset().column << ", "
              << planningGrid.masterCellOffset().row << ")\n";
    std::cout << std::fixed << std::setprecision(3)
              << "Grid rasterization: "
              << std::chrono::duration<double, std::milli>(gridElapsed).count()
              << " ms\n";
    const std::string outputFile = argc > 1 ? argv[1] : "";

    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning ROI\n";
        return 1;
    }

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
    const std::vector<Point2> worldPath = gridPathToWorld(planningGrid, plan.path);
    visualize(planner, start, goal, 1200, outputFile, worldPath);

    OccupancyGridRenderOptions gridView;
    gridView.pixelsPerCell = 8;
    for (const GridCell& cell : plan.path)
        gridView.markers.push_back({ cell, cv::Scalar(255, 120, 0), 2 });
    gridView.markers.push_back({ *startCell, cv::Scalar(0, 180, 0), 3 });
    gridView.markers.push_back({ *goalCell, cv::Scalar(0, 0, 255), 3 });

    std::cout << "A* path: "
              << plan.path.size() << " cells\n";
    showOccupancyGrid(planningGrid, gridView, "AStarPlanner Planning ROI");

    cv::waitKey(0);
    return 0;
}
