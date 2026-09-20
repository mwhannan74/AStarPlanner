// operation_area_demo.cpp - Deterministic operation-area and clipping demo.

#include "a_star_planner.hpp"
#include "a_star_planner_visualization.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    using namespace astar;

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
    AStarPlanner planner(operationArea, obstacles);

    std::cout << "Input obstacles: " << obstacles.size() << '\n';
    std::cout << "Effective obstacles after clipping: "
              << planner.obstacles().size() << '\n';
    std::cout << "Clipped obstacle overlays: "
              << planner.clippedObstacles().size() << '\n';

    const std::string outputFile = argc > 1 ? argv[1] : "";
    visualize(planner, start, goal, 1200, outputFile);
    cv::waitKey(0);
    return 0;
}
