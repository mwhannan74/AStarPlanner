// a_star_planner_demo.cpp - Environment demo with 25 non-overlapping obstacles.
// Build with the a_star_planner_demo CMake target; see README.md for instructions.

#include "a_star_planner.hpp"
#include "a_star_planner_visualization.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace astar;

int main(int argc, char* argv[])
{
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

    std::cout << "Environment has " << planner.obstacles().size()
              << " effective obstacles\n";
    const std::string outputFile = argc > 1 ? argv[1] : "";
    visualize(planner, start, goal, 1200, outputFile);

    cv::waitKey(0);
    return 0;
}
