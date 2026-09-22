// operation_area_demo.cpp - Deterministic operation-area and clipping demo.

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
    PolygonEnvironment environment(operationArea, obstacles);
    constexpr double gridResolution = 1.0;
    const auto gridStartTime = Clock::now();
    const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
        environment.operationArea(),
        environment.effectiveObstacles(),
        gridResolution);
    const auto gridElapsed = Clock::now() - gridStartTime;
    // This caller-selected ROI retains the operation area's full vertical span,
    // including the available routes around the alternating obstacle walls.
    const WorldBounds planningBounds{
        Point2(4.0, 0.0),
        Point2(96.0, 60.0)
    };
    const OccupancyGrid planningGrid = masterGrid.subgrid(planningBounds);

    std::cout << "Input obstacles: " << obstacles.size() << '\n';
    std::cout << "Effective obstacles after clipping: "
              << environment.effectiveObstacles().size() << '\n';
    std::cout << "Clipped obstacle overlays: "
              << environment.clippedObstacles().size() << '\n';
    std::cout << "Master occupancy grid: "
              << masterGrid.width() << " x " << masterGrid.height()
              << " cells at " << masterGrid.geometry().resolution()
              << " world units per cell\n";
    std::cout << "Planning ROI (caller-selected): "
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
    visualize(environment, start, goal, 1200, outputFile, worldPath);

    OccupancyGridRenderOptions gridView;
    gridView.pixelsPerCell = 8;
    gridView.paths.push_back({ plan.path, cv::Scalar(255, 120, 0), 2 });
    gridView.markers.push_back({ *startCell, cv::Scalar(0, 180, 0), 3 });
    gridView.markers.push_back({ *goalCell, cv::Scalar(0, 0, 255), 3 });

    std::cout << "A* path: "
              << plan.path.size() << " cells\n";
    showOccupancyGrid(planningGrid, gridView, "AStarPlanner Planning ROI");
    cv::waitKey(0);
    return 0;
}
