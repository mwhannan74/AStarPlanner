// main_demo.cpp - Enhanced demo of the VisibilityGraph class with 25 non-overlapping obstacles.
// Build with the main_demo CMake target; see README.md for instructions.
// ---------------------------------------------------------------------------

#include "visibility_graph.hpp"
#include "visibility_graph_visualization.hpp"

#include <algorithm>
#include <iostream>
#include <chrono>
#include <limits>
#include <vector>
#include <random>

using namespace vg;

int main()
{
    // 1. Create non-overlapping square obstacles in a rows-by-columns grid.
    const int rows = 5, cols = 5;
    const double size = 5.0;
    const double gap = 5.0;

    const double noise_frac = 0.5;             // Fraction of gap used as max noise
    const double noise = gap * noise_frac;

    // Set up random number generator for obstacle noise and, later, start/goal offsets
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> noise_dist(-noise, noise);

    std::vector<Polygon> obstacles;
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            // Base grid position
            double x0_base = c * (size + gap);
            double y0_base = r * (size + gap);
            // Add noise jitter
            double x0 = x0_base + noise_dist(gen);
            double y0 = y0_base + noise_dist(gen);
            obstacles.push_back(Polygon{
                { x0,          y0 },
                { x0 + size,   y0 },
                { x0 + size, y0 + size },
                { x0,        y0 + size }
                });
        }
    }

    // 2. Construct and build the visibility graph
    VisibilityGraph vg(obstacles);
    const auto t0 = std::chrono::high_resolution_clock::now();
    vg.buildBasic();
    const auto t1 = std::chrono::high_resolution_clock::now();

    // 3. Define randomized start and goal positions guaranteed to be outside
    // the obstacle field.
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& obstacle : obstacles)
    {
        for (const auto& point : obstacle)
        {
            min_x = std::min(min_x, point.x());
            min_y = std::min(min_y, point.y());
            max_x = std::max(max_x, point.x());
            max_y = std::max(max_y, point.y());
        }
    }

    std::uniform_real_distribution<double> terminal_offset(0.0, 2.0 * gap);
    Point2 S(min_x - gap - terminal_offset(gen),
             min_y - gap - terminal_offset(gen));
    Point2 G(max_x + gap + terminal_offset(gen),
             max_y + gap + terminal_offset(gen));

    // 4. Inject S/G and solve for shortest path
    auto [sid, gid] = vg.injectQueryPts(S, G);
    const auto t2 = std::chrono::high_resolution_clock::now();
    auto path = vg.shortestPath(sid, gid);
    const auto t3 = std::chrono::high_resolution_clock::now();

    // 5. Timing output
    auto build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    auto query_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    auto solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
    std::cout << "buildBasic() took " << build_ms << " ms\n";
    std::cout << "Query setup and injectQueryPts() took " << query_ms << " ms\n";
    std::cout << "shortestPath() took " << solve_ms << " ms\n";
    
    std::cout << "Graph has " << vg.numEdges() << " edges\n";

    // 6. Visualise the obstacles, graph, and solution path
    vg::visualize(vg, S, G, path);

    // 7. Console summary
    if (path.empty())
        std::cout << "No path found\n";
    else
        std::cout << "Path found with " << path.size() << " waypoints\n";

    // 8. Wait for key press to close visualization
    cv::waitKey(0);
    return 0;
}
