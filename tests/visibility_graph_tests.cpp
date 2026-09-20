/*
 * VisibilityGraph tests
 *
 * This is a small, dependency-free test program. It runs every test in the
 * order listed in TEST_CASES and prints PASS or FAIL with the elapsed time.
 *
 * From the repository root:
 *
 *   .\build\Release\visgraph_tests.exe
 *
 * Tests:
 *   - direct path without obstacles
 *   - shortest path around a square
 *   - rejection of diagonals through an obstacle
 *   - rejection of crossings through small-scale obstacles
 *   - convex polygon validation and normalization
 *   - rejection of concave, self-intersecting, and non-finite polygons
 *   - scale-aware behavior for large translated coordinates
 *   - translation-stable area validation for small polygons
 *   - rejection of non-finite query coordinates
 *   - operation-area validation and query containment
 *   - operation-area obstacle filtering
 *   - clipping obstacles at an operation-area boundary
 *   - exclusion of operation-boundary points from graph vertices
 *   - preservation of original and clipped obstacle views
 *   - rejection of a query inside an obstacle
 *   - ignoring polygons with fewer than three vertices
 *   - identical start and goal
 *   - query on an obstacle edge
 *   - query on an obstacle vertex
 *   - repeated query insertion
 *   - rebuilding after query insertion
 *   - querying before buildBasic()
 *   - shortest-path vertex index validation
 *
 * Each test is an ordinary function. Add a new function, use require() to
 * check its result, then add its name and function to TEST_CASES. main() runs
 * the full list and returns a nonzero exit code if any test fails.
 */

#include "visibility_graph.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using vg::Point2;
    using vg::Polygon;
    using vg::VisibilityGraph;

    constexpr double TEST_EPS = 1e-9;

    // Minimal test helpers. A failed requirement throws so the runner can
    // report the failure and continue with the remaining cases.
    class TestFailure : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(bool condition, const std::string& message)
    {
        if (!condition)
            throw TestFailure(message);
    }

    bool pointsNear(const Point2& lhs, const Point2& rhs)
    {
        return (lhs - rhs).norm() <= TEST_EPS;
    }

    double pathLength(const std::vector<Point2>& path)
    {
        double length = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i)
            length += (path[i] - path[i - 1]).norm();
        return length;
    }

    double signedAreaTwice(const Polygon& polygon)
    {
        const Point2& origin = polygon.front();
        double area = 0.0;
        for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
        {
            const Point2 first = polygon[i] - origin;
            const Point2 second = polygon[i + 1] - origin;
            area += first.x() * second.y() - first.y() * second.x();
        }
        return area;
    }

    bool hasEdge(const VisibilityGraph& graph, std::size_t from, std::size_t to)
    {
        for (const auto& edge : graph.adjacency().at(from))
            if (edge.to == to)
                return true;
        return false;
    }

    template <typename Exception, typename Function>
    void requireThrows(Function&& function, const std::string& message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const Exception&)
        {
            return;
        }
        catch (const std::exception& error)
        {
            throw TestFailure(message + "; caught different exception: " + error.what());
        }

        throw TestFailure(message + "; no exception was thrown");
    }

    Polygon makeSquare()
    {
        return {
            Point2(0.0, 0.0),
            Point2(4.0, 0.0),
            Point2(4.0, 4.0),
            Point2(0.0, 4.0)
        };
    }

    Polygon makeOperationArea()
    {
        return {
            Point2(0.0, 0.0),
            Point2(10.0, 0.0),
            Point2(10.0, 10.0),
            Point2(0.0, 10.0)
        };
    }

    // Working behavior and straightforward cases are listed first.

    void directPathInEmptyEnvironment()
    {
        VisibilityGraph graph({});
        graph.buildBasic();

        const Point2 start(-1.0, 2.0);
        const Point2 goal(6.0, 2.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(path.size() == 2, "unobstructed path should contain only start and goal");
        require(pointsNear(path.front(), start), "unobstructed path should start at S");
        require(pointsNear(path.back(), goal), "unobstructed path should end at G");
        require(std::abs(pathLength(path) - 7.0) <= TEST_EPS,
            "unobstructed path should have Euclidean length");
    }

    void shortestPathRoutesAroundSquare()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        const Point2 start(-1.0, 2.0);
        const Point2 goal(5.0, 2.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        const double expectedLength = 4.0 + 2.0 * std::sqrt(5.0);
        require(path.size() == 4, "path around a square should use two obstacle corners");
        require(pointsNear(path.front(), start), "square route should start at S");
        require(pointsNear(path.back(), goal), "square route should end at G");
        require(std::abs(pathLength(path) - expectedLength) <= TEST_EPS,
            "square route should have the expected shortest length");
    }

    void obstacleBoundaryEdgesExcludeInteriorChord()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        require(hasEdge(graph, 0, 1), "adjacent polygon vertices should be connected");
        require(hasEdge(graph, 0, 3), "closing polygon edge should be connected");
        require(!hasEdge(graph, 0, 2), "diagonal through polygon interior must be rejected");
        require(!hasEdge(graph, 1, 3), "opposite polygon diagonal must be rejected");
    }

    void smallScaleObstacleBlocksDirectPath()
    {
        const Polygon smallSquare{
            Point2(0.0, 0.0),
            Point2(1e-4, 0.0),
            Point2(1e-4, 1e-4),
            Point2(0.0, 1e-4)
        };
        VisibilityGraph graph({ smallSquare });
        graph.buildBasic();

        const Point2 start(-1e-4, 5e-5);
        const Point2 goal(2e-4, 5e-5);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(!hasEdge(graph, startId, goalId),
            "a direct edge crossing a small-scale obstacle must be rejected");
        require(path.size() > 2,
            "a path crossing a small-scale obstacle should route around it");
        require(pathLength(path) > (goal - start).norm(),
            "the routed path should be longer than the blocked direct segment");
    }

    void convexCounterClockwiseObstacleIsAccepted()
    {
        const Polygon pentagon{
            Point2(0.0, 0.0),
            Point2(3.0, 0.0),
            Point2(4.0, 2.0),
            Point2(2.0, 4.0),
            Point2(0.0, 2.0)
        };

        VisibilityGraph graph({ pentagon });
        graph.buildBasic();

        require(graph.obstacles().size() == 1,
            "a convex counter-clockwise obstacle should be accepted");
        require(graph.vertices().size() == pentagon.size(),
            "an accepted convex obstacle should retain all of its vertices");
    }

    void convexObstacleRemovesCollinearBoundaryVertex()
    {
        const Polygon rectangleWithExtraVertex{
            Point2(0.0, 0.0),
            Point2(2.0, 0.0),
            Point2(4.0, 0.0),
            Point2(4.0, 3.0),
            Point2(0.0, 3.0)
        };

        VisibilityGraph graph({ rectangleWithExtraVertex });

        require(graph.obstacles().size() == 1,
            "a convex obstacle with a collinear boundary vertex should be accepted");
        require(graph.obstacles().front().size() == 4,
            "a redundant collinear boundary vertex should be removed");
    }

    void concaveObstacleIsRejected()
    {
        const Polygon concave{
            Point2(0.0, 0.0),
            Point2(4.0, 0.0),
            Point2(2.0, 1.0),
            Point2(4.0, 4.0),
            Point2(0.0, 4.0)
        };

        requireThrows<std::invalid_argument>(
            [&concave] { VisibilityGraph graph({ concave }); },
            "a concave obstacle should be rejected");
    }

    void clockwiseObstacleIsNormalized()
    {
        const Polygon clockwiseSquare{
            Point2(0.0, 0.0),
            Point2(0.0, 4.0),
            Point2(4.0, 4.0),
            Point2(4.0, 0.0)
        };

        VisibilityGraph graph({ clockwiseSquare });
        graph.buildBasic();

        require(graph.obstacles().size() == 1,
            "a clockwise convex obstacle should be accepted after normalization");
        require(hasEdge(graph, 0, 1),
            "a normalized clockwise obstacle should retain its boundary edges");
    }

    void duplicateAndClosingVerticesAreNormalized()
    {
        const Polygon redundantSquare{
            Point2(0.0, 0.0),
            Point2(4.0, 0.0),
            Point2(4.0, 0.0),
            Point2(4.0, 4.0),
            Point2(0.0, 4.0),
            Point2(0.0, 0.0)
        };

        VisibilityGraph graph({ redundantSquare });

        require(graph.obstacles().front().size() == 4,
            "consecutive duplicates and a repeated closing vertex should be removed");
    }

    void invalidPolygonGeometryIsRejected()
    {
        const Polygon selfIntersecting{
            Point2(0.0, 0.0),
            Point2(4.0, 4.0),
            Point2(0.0, 4.0),
            Point2(4.0, 0.0)
        };
        const Polygon nonFinite{
            Point2(0.0, 0.0),
            Point2(std::numeric_limits<double>::infinity(), 0.0),
            Point2(0.0, 4.0)
        };
        const Polygon undersizedNonFinite{
            Point2(0.0, 0.0),
            Point2(std::numeric_limits<double>::quiet_NaN(), 1.0)
        };

        requireThrows<std::invalid_argument>(
            [&selfIntersecting] { VisibilityGraph graph({ selfIntersecting }); },
            "a self-intersecting obstacle should be rejected");
        requireThrows<std::invalid_argument>(
            [&nonFinite] { VisibilityGraph graph({ nonFinite }); },
            "an obstacle with non-finite coordinates should be rejected");
        requireThrows<std::invalid_argument>(
            [&undersizedNonFinite] { VisibilityGraph graph({ undersizedNonFinite }); },
            "non-finite obstacle coordinates should not be hidden by undersized-polygon filtering");
        requireThrows<std::invalid_argument>(
            [&nonFinite] { VisibilityGraph graph(nonFinite, {}); },
            "an operation area with non-finite coordinates should be rejected");
    }

    void largeTranslatedCoordinatesRemainUsable()
    {
        constexpr double offset = 1e9;
        const Polygon obstacle{
            Point2(offset, offset),
            Point2(offset + 100.0, offset),
            Point2(offset + 100.0, offset + 100.0),
            Point2(offset, offset + 100.0)
        };
        VisibilityGraph graph({ obstacle });
        graph.buildBasic();

        const auto [startId, goalId] = graph.injectQueryPts(
            Point2(offset - 10.0, offset + 50.0),
            Point2(offset + 110.0, offset + 50.0));
        const auto path = graph.shortestPath(startId, goalId);

        require(path.size() == 4,
            "scale-aware predicates should route around a large translated obstacle");
    }

    void smallPolygonAreaIsStableAfterTranslation()
    {
        constexpr double offset = 1e7;
        constexpr double side = 0.1;
        const Polygon obstacle{
            Point2(offset, offset),
            Point2(offset + side, offset),
            Point2(offset + side, offset + side),
            Point2(offset, offset + side)
        };
        VisibilityGraph graph({ obstacle });
        graph.buildBasic();

        const Point2 start(offset - side, offset + side / 2.0);
        const Point2 goal(offset + 2.0 * side, offset + side / 2.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(graph.obstacles().size() == 1,
            "translation should not cause a valid small polygon to be rejected");
        require(graph.obstacles().front().size() == 4,
            "translation should preserve the small polygon's four corners");
        require(path.size() == 4,
            "the translated small obstacle should still block the direct path");
    }

    void nonFiniteQueriesAreRejectedWithoutChangingTheGraph()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        VisibilityGraph graph({});
        graph.buildBasic();
        const auto validIds = graph.injectQueryPts(Point2(1.0, 1.0), Point2(2.0, 2.0));
        const std::size_t validVertexCount = graph.vertices().size();

        requireThrows<std::invalid_argument>(
            [&graph, nan] { graph.injectQueryPts(Point2(nan, 1.0), Point2(2.0, 2.0)); },
            "a non-finite start should be rejected without an operation area");
        requireThrows<std::invalid_argument>(
            [&graph, infinity] { graph.injectQueryPts(Point2(1.0, 1.0), Point2(infinity, 2.0)); },
            "a non-finite goal should be rejected without an operation area");
        require(graph.vertices().size() == validVertexCount,
            "a rejected query should preserve the existing graph");
        require(graph.shortestPath(validIds.first, validIds.second).size() == 2,
            "a rejected query should preserve the existing valid path");

        VisibilityGraph operationAreaGraph(makeOperationArea(), {});
        operationAreaGraph.buildBasic();
        requireThrows<std::invalid_argument>(
            [&operationAreaGraph, nan] {
                operationAreaGraph.injectQueryPts(Point2(1.0, 1.0), Point2(2.0, nan));
            },
            "a non-finite query should be rejected before operation-area classification");
    }

    void operationAreaSupportsContainedQueries()
    {
        const Polygon operationArea = makeOperationArea();
        VisibilityGraph graph(operationArea, {});
        graph.buildBasic();

        const Point2 start(1.0, 2.0);
        const Point2 goal(9.0, 8.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(graph.hasOperationArea(),
            "the operation-area constructor should record the operation area");
        require(graph.operationArea().size() == operationArea.size(),
            "the stored operation area should retain its vertices");
        require(path.size() == 2,
            "queries inside an empty operation area should have a direct path");
    }

    void graphWithoutOperationAreaPreservesExistingBehavior()
    {
        VisibilityGraph graph({});

        require(!graph.hasOperationArea(),
            "the original constructor should not create an operation area");
        requireThrows<std::logic_error>(
            [&graph] { static_cast<void>(graph.operationArea()); },
            "requesting a missing operation area should be rejected");
    }

    void operationAreaValidationAndWindingNormalization()
    {
        const Polygon undersized{
            Point2(0.0, 0.0),
            Point2(10.0, 0.0)
        };
        const Polygon concave{
            Point2(0.0, 0.0),
            Point2(10.0, 0.0),
            Point2(5.0, 2.0),
            Point2(10.0, 10.0),
            Point2(0.0, 10.0)
        };
        const Polygon clockwise{
            Point2(0.0, 0.0),
            Point2(0.0, 10.0),
            Point2(10.0, 10.0),
            Point2(10.0, 0.0)
        };

        requireThrows<std::invalid_argument>(
            [&undersized] { VisibilityGraph graph(undersized, {}); },
            "an undersized operation area should be rejected");
        requireThrows<std::invalid_argument>(
            [&concave] { VisibilityGraph graph(concave, {}); },
            "a concave operation area should be rejected");
        VisibilityGraph clockwiseGraph(clockwise, {});
        require(clockwiseGraph.operationArea().size() == 4,
            "a clockwise operation area should be accepted after normalization");
        require(signedAreaTwice(clockwiseGraph.operationArea()) > 0.0,
            "a clockwise operation area should be normalized to counter-clockwise winding");
    }

    void operationAreaRejectsQueriesNotStrictlyInside()
    {
        VisibilityGraph graph(makeOperationArea(), {});
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(-1.0, 5.0), Point2(5.0, 5.0)); },
            "a start outside the operation area should be rejected");
        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(5.0, 5.0), Point2(11.0, 5.0)); },
            "a goal outside the operation area should be rejected");
        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(0.0, 5.0), Point2(5.0, 5.0)); },
            "a start on the operation-area boundary should be rejected");
        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(5.0, 5.0), Point2(10.0, 5.0)); },
            "a goal on the operation-area boundary should be rejected");
    }

    void operationAreaRetainsContainedObstacle()
    {
        const Polygon obstacle{
            Point2(3.0, 3.0),
            Point2(7.0, 3.0),
            Point2(7.0, 7.0),
            Point2(3.0, 7.0)
        };
        VisibilityGraph graph(makeOperationArea(), { obstacle });
        graph.buildBasic();

        const auto [startId, goalId] = graph.injectQueryPts(
            Point2(1.0, 5.0), Point2(9.0, 5.0));
        const auto path = graph.shortestPath(startId, goalId);

        require(graph.obstacles().size() == 1,
            "an obstacle contained by the operation area should be retained");
        require(path.size() > 2,
            "a contained obstacle should still block a direct route");
    }

    void operationAreaRetainsObstacleOnBoundary()
    {
        const Polygon boundaryObstacle{
            Point2(0.0, 3.0),
            Point2(3.0, 3.0),
            Point2(3.0, 7.0),
            Point2(0.0, 7.0)
        };
        VisibilityGraph graph(makeOperationArea(), { boundaryObstacle });

        require(graph.obstacles().size() == 1,
            "an obstacle contained by and touching the operation-area boundary should be retained");
        require(graph.vertices().size() == 2,
            "vertices on the operation-area boundary must not enter the graph");
        for (const auto& vertex : graph.vertices())
        {
            require(vertex.pos.x() > TEST_EPS && vertex.pos.x() < 10.0 - TEST_EPS &&
                    vertex.pos.y() > TEST_EPS && vertex.pos.y() < 10.0 - TEST_EPS,
                "every retained obstacle graph vertex must be strictly inside the operation area");
        }
    }

    void operationAreaDiscardsOutsideObstacle()
    {
        const Polygon outsideObstacle{
            Point2(12.0, 2.0),
            Point2(14.0, 2.0),
            Point2(14.0, 4.0),
            Point2(12.0, 4.0)
        };
        VisibilityGraph graph(makeOperationArea(), { outsideObstacle });

        require(graph.obstacles().empty(),
            "an obstacle wholly outside the operation area should be discarded");
        require(graph.vertices().empty(),
            "a discarded outside obstacle should not contribute graph vertices");
    }

    void operationAreaClipsCrossingObstacleAndRoutesAroundIt()
    {
        const Polygon crossingObstacle{
            Point2(6.0, 3.0),
            Point2(12.0, 3.0),
            Point2(12.0, 7.0),
            Point2(6.0, 7.0)
        };
        VisibilityGraph graph(makeOperationArea(), { crossingObstacle });
        graph.buildBasic();

        const auto& clipped = graph.obstacles();
        require(clipped.size() == 1,
            "an obstacle crossing the operation-area boundary should be retained after clipping");
        require(clipped.front().size() == 4,
            "a clipped rectangle should contain four vertices");
        bool hasOperationBoundaryVertex = false;
        for (const auto& point : clipped.front())
        {
            require(point.x() <= 10.0 + TEST_EPS,
                "clipped obstacle vertices must remain inside the operation area");
            hasOperationBoundaryVertex = hasOperationBoundaryVertex ||
                std::abs(point.x() - 10.0) <= TEST_EPS;
        }
        require(hasOperationBoundaryVertex,
            "clipping should create vertices on the operation-area boundary");
        require(graph.vertices().size() == 2,
            "clipped operation-boundary intersections must not become graph vertices");
        for (const auto& vertex : graph.vertices())
        {
            require(vertex.pos.x() > TEST_EPS && vertex.pos.x() < 10.0 - TEST_EPS &&
                    vertex.pos.y() > TEST_EPS && vertex.pos.y() < 10.0 - TEST_EPS,
                "visibility-graph vertices must be strictly inside the operation area");
        }

        const auto [startId, goalId] = graph.injectQueryPts(
            Point2(2.0, 2.0), Point2(9.0, 9.0));
        const auto path = graph.shortestPath(startId, goalId);
        require(path.size() > 2,
            "a route should bend around the clipped obstacle");
        for (const auto& point : path)
        {
            require(point.x() >= -TEST_EPS && point.x() <= 10.0 + TEST_EPS &&
                    point.y() >= -TEST_EPS && point.y() <= 10.0 + TEST_EPS,
                "a route around a clipped obstacle must remain in the operation area");
        }
    }

    void operationAreaClipsCrossingObstacleWithAllVerticesOutside()
    {
        const Polygon crossingObstacle{
            Point2(-1.0, 4.0),
            Point2(11.0, 4.0),
            Point2(11.0, 6.0),
            Point2(-1.0, 6.0)
        };

        VisibilityGraph graph(makeOperationArea(), { crossingObstacle });

        require(graph.obstacles().size() == 1,
            "a crossing obstacle must not be discarded merely because all vertices are outside");
        require(graph.obstacles().front().size() == 4,
            "a strip crossing the operation area should clip to a rectangle");
        for (const auto& point : graph.obstacles().front())
        {
            require(point.x() >= -TEST_EPS && point.x() <= 10.0 + TEST_EPS,
                "the crossing strip should be clipped to the operation-area width");
        }
        require(graph.vertices().empty(),
            "a clipped obstacle with only operation-boundary vertices must add no graph vertices");

        graph.buildBasic();
        const auto [startId, goalId] = graph.injectQueryPts(
            Point2(5.0, 2.0), Point2(5.0, 8.0));
        require(graph.shortestPath(startId, goalId).empty(),
            "a boundary-to-boundary obstacle must not be bypassed through clipped boundary vertices");
    }

    void operationAreaClipsObstacleContainingEntireArea()
    {
        const Polygon containingObstacle{
            Point2(-2.0, -2.0),
            Point2(12.0, -2.0),
            Point2(12.0, 12.0),
            Point2(-2.0, 12.0)
        };

        VisibilityGraph graph(makeOperationArea(), { containingObstacle });
        graph.buildBasic();

        require(graph.obstacles().size() == 1,
            "an obstacle containing the operation area should clip to that area");
        require(graph.obstacles().front().size() == makeOperationArea().size(),
            "the clipped containing obstacle should match the operation-area shape");
        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(2.0, 2.0), Point2(8.0, 8.0)); },
            "a clipped obstacle covering the operation area should reject every query");
    }

    void operationAreaClipsObstacleCrossingCorner()
    {
        const Polygon cornerObstacle{
            Point2(8.0, 8.0),
            Point2(12.0, 8.0),
            Point2(12.0, 12.0),
            Point2(8.0, 12.0)
        };
        VisibilityGraph graph(makeOperationArea(), { cornerObstacle });

        require(graph.obstacles().size() == 1,
            "an obstacle crossing an operation-area corner should be retained");
        require(graph.obstacles().front().size() == 4,
            "a rectangle crossing a corner should clip to a rectangle");
        for (const auto& point : graph.obstacles().front())
        {
            require(point.x() >= 8.0 - TEST_EPS && point.x() <= 10.0 + TEST_EPS &&
                    point.y() >= 8.0 - TEST_EPS && point.y() <= 10.0 + TEST_EPS,
                "corner-clipped vertices should lie in the overlapping square");
        }
        require(graph.vertices().size() == 1,
            "corner clipping should retain only the obstacle vertex strictly inside the operation area");
        require(pointsNear(graph.vertices().front().pos, Point2(8.0, 8.0)),
            "the retained corner-obstacle graph vertex should be the interior input vertex");
    }

    void operationAreaDiscardsZeroAreaObstacleContacts()
    {
        const Polygon pointContact{
            Point2(10.0, 10.0),
            Point2(12.0, 10.0),
            Point2(12.0, 12.0),
            Point2(10.0, 12.0)
        };
        const Polygon edgeContact{
            Point2(2.0, 10.0),
            Point2(8.0, 10.0),
            Point2(8.0, 12.0),
            Point2(2.0, 12.0)
        };

        VisibilityGraph pointGraph(makeOperationArea(), { pointContact });
        VisibilityGraph edgeGraph(makeOperationArea(), { edgeContact });

        require(pointGraph.obstacles().empty(),
            "an obstacle touching only one operation-area point should be discarded");
        require(edgeGraph.obstacles().empty(),
            "an obstacle sharing only an operation-area edge should be discarded");
    }

    void operationAreaPreservesOriginalAndClippedObstacleViews()
    {
        const Polygon contained{
            Point2(2.0, 2.0),
            Point2(4.0, 2.0),
            Point2(4.0, 4.0),
            Point2(2.0, 4.0)
        };
        const Polygon crossing{
            Point2(8.0, 5.0),
            Point2(12.0, 5.0),
            Point2(12.0, 7.0),
            Point2(8.0, 7.0)
        };
        const Polygon outside{
            Point2(12.0, 1.0),
            Point2(14.0, 1.0),
            Point2(14.0, 3.0),
            Point2(12.0, 3.0)
        };

        VisibilityGraph graph(makeOperationArea(),
            { contained, crossing, outside });

        require(graph.originalObstacles().size() == 3,
            "the original obstacle view should preserve all validated input obstacles");
        require(graph.obstacles().size() == 2,
            "the effective obstacle view should omit fully outside obstacles");
        require(graph.clippedObstacles().size() == 1,
            "the clipped obstacle view should contain only changed positive-area geometry");
        require(graph.clippedObstacles().front().size() == 4,
            "the tracked clipped obstacle should contain its effective vertices");
    }

    void queryInsideObstacleIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(2.0, 2.0), Point2(6.0, 2.0)); },
            "query point inside an obstacle should be rejected");
    }

    void undersizedPolygonIsIgnored()
    {
        const Polygon line{ Point2(0.0, 0.0), Point2(1.0, 0.0) };
        std::ostringstream warning;
        auto* originalErrorBuffer = std::cerr.rdbuf(warning.rdbuf());
        VisibilityGraph graph({ line });
        std::cerr.rdbuf(originalErrorBuffer);
        graph.buildBasic();

        require(graph.obstacles().empty(), "polygon with fewer than three vertices should be ignored");
        require(graph.vertices().empty(), "ignored polygon should not contribute graph vertices");
        require(!warning.str().empty(), "ignored polygon should produce a warning");
    }

    // Less common lifecycle and boundary cases.

    void identicalStartAndGoalReturnsZeroLengthPath()
    {
        VisibilityGraph graph({});
        graph.buildBasic();

        const Point2 query(3.0, -2.0);
        const auto [startId, goalId] = graph.injectQueryPts(query, query);
        const auto path = graph.shortestPath(startId, goalId);

        require(path.size() == 1, "identical start and goal should return one point");
        require(pointsNear(path.front(), query), "zero-length path should contain the query point");
        require(pathLength(path) <= TEST_EPS, "identical start and goal should have zero path length");
    }

    void queryOnEdgeIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(2.0, 0.0), Point2(6.0, 2.0)); },
            "query point on an obstacle edge should be rejected");
    }

    void queryOnVertexIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(0.0, 0.0), Point2(6.0, 2.0)); },
            "query point on an obstacle vertex should be rejected");
    }

    void repeatedQueriesDoNotAccumulateVertices()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        graph.injectQueryPts(Point2(-1.0, 1.0), Point2(5.0, 1.0));
        graph.injectQueryPts(Point2(-1.0, 3.0), Point2(5.0, 3.0));

        require(graph.vertices().size() == makeSquare().size() + 2,
            "a new query should replace the previous start and goal vertices");
    }

    void rebuildRestoresObstacleOnlyGraph()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();
        graph.injectQueryPts(Point2(-1.0, 2.0), Point2(5.0, 2.0));

        graph.buildBasic();

        require(graph.vertices().size() == makeSquare().size(),
            "rebuilding should remove previously injected query vertices");
        require(graph.adjacency().size() == makeSquare().size(),
            "rebuilt adjacency should contain obstacle vertices only");
    }

    void queryBeforeBuildIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });

        requireThrows<std::logic_error>(
            [&graph] {
                graph.injectQueryPts(Point2(-1.0, 2.0), Point2(5.0, 2.0));
            },
            "injectQueryPts should reject a query before buildBasic is called");
    }

    void shortestPathRejectsInvalidVertexIndices()
    {
        VisibilityGraph graph({});
        graph.buildBasic();
        const auto [startId, goalId] = graph.injectQueryPts(
            Point2(0.0, 0.0), Point2(1.0, 1.0));
        const std::size_t invalidId = graph.vertices().size();

        requireThrows<std::out_of_range>(
            [&graph, invalidId, goalId] {
                static_cast<void>(graph.shortestPath(invalidId, goalId));
            },
            "shortestPath should reject an invalid source index");
        requireThrows<std::out_of_range>(
            [&graph, startId, invalidId] {
                static_cast<void>(graph.shortestPath(startId, invalidId));
            },
            "shortestPath should reject an invalid goal index");
    }

    struct TestCase
    {
        const char* name;
        void (*run)();
    };

    // To add a test, write a function above and add one entry here.
    const std::vector<TestCase> TEST_CASES{
        { "Direct path in empty environment", directPathInEmptyEnvironment },
        { "Shortest path routes around square", shortestPathRoutesAroundSquare },
        { "Obstacle boundary excludes interior chord", obstacleBoundaryEdgesExcludeInteriorChord },
        { "Small-scale obstacle blocks direct path", smallScaleObstacleBlocksDirectPath },
        { "Convex counter-clockwise obstacle is accepted", convexCounterClockwiseObstacleIsAccepted },
        { "Convex obstacle removes collinear boundary vertex", convexObstacleRemovesCollinearBoundaryVertex },
        { "Concave obstacle is rejected", concaveObstacleIsRejected },
        { "Clockwise obstacle is normalized", clockwiseObstacleIsNormalized },
        { "Duplicate and closing vertices are normalized", duplicateAndClosingVerticesAreNormalized },
        { "Invalid polygon geometry is rejected", invalidPolygonGeometryIsRejected },
        { "Large translated coordinates remain usable", largeTranslatedCoordinatesRemainUsable },
        { "Small polygon area is stable after translation", smallPolygonAreaIsStableAfterTranslation },
        { "Non-finite queries are rejected", nonFiniteQueriesAreRejectedWithoutChangingTheGraph },
        { "Operation area supports contained queries", operationAreaSupportsContainedQueries },
        { "Original constructor has no operation area", graphWithoutOperationAreaPreservesExistingBehavior },
        { "Operation area validation and winding normalization", operationAreaValidationAndWindingNormalization },
        { "Operation area rejects non-interior queries", operationAreaRejectsQueriesNotStrictlyInside },
        { "Operation area retains contained obstacle", operationAreaRetainsContainedObstacle },
        { "Operation area retains boundary obstacle", operationAreaRetainsObstacleOnBoundary },
        { "Operation area discards outside obstacle", operationAreaDiscardsOutsideObstacle },
        { "Operation area clips crossing obstacle", operationAreaClipsCrossingObstacleAndRoutesAroundIt },
        { "Operation area clips crossing with outside vertices", operationAreaClipsCrossingObstacleWithAllVerticesOutside },
        { "Operation area clips containing obstacle", operationAreaClipsObstacleContainingEntireArea },
        { "Operation area clips corner-crossing obstacle", operationAreaClipsObstacleCrossingCorner },
        { "Operation area discards zero-area contacts", operationAreaDiscardsZeroAreaObstacleContacts },
        { "Operation area preserves obstacle views", operationAreaPreservesOriginalAndClippedObstacleViews },
        { "Query inside obstacle is rejected", queryInsideObstacleIsRejected },
        { "Undersized polygon is ignored", undersizedPolygonIsIgnored },
        { "Identical start and goal", identicalStartAndGoalReturnsZeroLengthPath },
        { "Query on edge is rejected", queryOnEdgeIsRejected },
        { "Query on vertex is rejected", queryOnVertexIsRejected },
        { "Repeated queries replace old queries", repeatedQueriesDoNotAccumulateVertices },
        { "Rebuild restores obstacle-only graph", rebuildRestoresObstacleOnlyGraph },
        { "Query before build is rejected", queryBeforeBuildIsRejected },
        { "Shortest path rejects invalid indices", shortestPathRejectsInvalidVertexIndices }
    };
}

int main()
{
    using Clock = std::chrono::steady_clock;

    std::size_t failed = 0;
    const auto suiteStart = Clock::now();

    std::cout << "Running " << TEST_CASES.size() << " tests\n\n";

    for (std::size_t index = 0; index < TEST_CASES.size(); ++index)
    {
        const auto& test = TEST_CASES[index];
        const auto testStart = Clock::now();
        std::cout << '[' << index + 1 << '/' << TEST_CASES.size() << "] "
                  << test.name << " ... " << std::flush;

        try
        {
            test.run();
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "PASS (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n";
        }
        catch (const std::exception& error)
        {
            ++failed;
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "FAIL (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n"
                      << "       " << error.what() << '\n';
        }
        catch (...)
        {
            ++failed;
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "FAIL (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n"
                      << "       Unknown exception\n";
        }
    }

    const auto passed = TEST_CASES.size() - failed;
    const auto elapsed = std::chrono::duration<double, std::milli>(
        Clock::now() - suiteStart).count();

    std::cout << "\nResult: " << passed << " passed, " << failed
              << " failed, " << TEST_CASES.size() << " total ("
              << std::fixed << std::setprecision(2) << elapsed << " ms)\n";
    return failed == 0 ? 0 : 1;
}
