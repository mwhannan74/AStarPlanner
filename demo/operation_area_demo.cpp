// operation_area_demo.cpp - Operation-area and planning-ROI tutorial.
//
// This example adds a convex keep-in area, obstacles that require clipping,
// and a caller-selected planning ROI. The planning steps otherwise match the
// basic full-grid demo.

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
    const PolygonEnvironment environment(operationArea, obstacles);

    // 3. Build an aligned master grid around the operation area. Rasterization
    // starts occupied, paints the operation area free, then paints obstacles.
    constexpr double gridResolution = 1.0;
    const auto gridStartTime = Clock::now();
    const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
        environment.operationArea(),
        environment.effectiveObstacles(),
        gridResolution);
    const auto gridElapsed = Clock::now() - gridStartTime;
    // 4. Select a planning ROI. This caller-selected window retains the full
    // vertical span and therefore the routes around the alternating walls.
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

    // 5. Convert world terminals using the ROI geometry, producing ROI-local
    // cells. masterCellOffset() relates those cells to the master grid.
    const auto startCell = planningGrid.geometry().worldToCell(start);
    const auto goalCell = planningGrid.geometry().worldToCell(goal);
    if (!startCell || !goalCell)
    {
        std::cerr << "Start or goal is outside the planning ROI\n";
        return 1;
    }

    // 6. Plan inside the selected ROI. NoPath would only mean that no route
    // exists inside this ROI, not necessarily inside the complete master grid.
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
    // 7. Convert the ROI-local cell path to world cell centers and render it.
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
