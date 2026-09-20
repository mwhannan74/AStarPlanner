// operation_area_demo.cpp - Deterministic operation-area and clipping demo.

#include "visibility_graph.hpp"
#include "visibility_graph_visualization.hpp"

#include <iostream>
#include <vector>

int main()
{
    using namespace vg;

    const Polygon operationArea{
        Point2(0.0, 0.0),
        Point2(100.0, 0.0),
        Point2(100.0, 60.0),
        Point2(0.0, 60.0)
    };

    const std::vector<Polygon> obstacles{
        // Four alternating boundary-crossing barriers force the route through
        // upper and lower corridors. Their boundary intersection vertices are
        // used for collision geometry but excluded from the visibility graph.
        {
            Point2(18.0, -8.0),
            Point2(28.0, -8.0),
            Point2(28.0, 36.0),
            Point2(18.0, 36.0)
        },
        {
            Point2(38.0, 24.0),
            Point2(48.0, 24.0),
            Point2(48.0, 68.0),
            Point2(38.0, 68.0)
        },
        {
            Point2(58.0, -6.0),
            Point2(68.0, -6.0),
            Point2(68.0, 38.0),
            Point2(58.0, 38.0)
        },
        {
            Point2(78.0, 22.0),
            Point2(88.0, 22.0),
            Point2(88.0, 70.0),
            Point2(78.0, 70.0)
        },
        // Interior obstacles add alternate visibility choices in the gaps.
        {
            Point2(7.0, 22.0),
            Point2(14.0, 22.0),
            Point2(14.0, 32.0),
            Point2(7.0, 32.0)
        },
        {
            Point2(33.0, 8.0),
            Point2(37.0, 14.0),
            Point2(33.0, 20.0),
            Point2(29.0, 14.0)
        },
        {
            Point2(72.0, 42.0),
            Point2(76.0, 47.0),
            Point2(72.0, 52.0),
            Point2(69.0, 47.0)
        },
        // Entirely outside and therefore omitted from the effective graph.
        {
            Point2(106.0, 5.0),
            Point2(114.0, 5.0),
            Point2(114.0, 15.0),
            Point2(106.0, 15.0)
        },
        // Every original vertex is outside, but the diamond crosses the
        // operation area's upper-right corner and leaves a clipped triangle.
        {
            Point2(90.0, 65.0),
            Point2(105.0, 50.0),
            Point2(120.0, 65.0),
            Point2(105.0, 80.0)
        }
    };

    const Point2 start(5.0, 8.0);
    const Point2 goal(95.0, 52.0);

    VisibilityGraph graph(operationArea, obstacles);
    graph.buildBasic();
    const auto [startId, goalId] = graph.injectQueryPts(start, goal);
    const auto path = graph.shortestPath(startId, goalId);

    std::cout << "Input obstacles: " << obstacles.size() << '\n';
    std::cout << "Effective obstacles after clipping: "
              << graph.obstacles().size() << '\n';
    std::cout << "Graph edges: " << graph.numEdges() << '\n';

    if (path.empty())
        std::cout << "No path found\n";
    else
        std::cout << "Path found with " << path.size() << " waypoints\n";

    visualize(graph, start, goal, path);
    cv::waitKey(0);
    return 0;
}
