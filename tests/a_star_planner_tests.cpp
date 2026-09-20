/*
 * Environment-model tests for the AStarPlanner conversion checkpoint.
 * Grid construction and A* path planning are intentionally not active yet.
 */

#include "a_star_planner.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using astar::AStarPlanner;
    using astar::Point2;
    using astar::Polygon;

    constexpr double TEST_EPSILON = 1e-9;

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
        return (lhs - rhs).norm() <= TEST_EPSILON;
    }

    double signedAreaTwice(const Polygon& polygon)
    {
        const Point2& origin = polygon.front();
        double area = 0.0;
        for (std::size_t index = 1; index + 1 < polygon.size(); ++index)
        {
            const Point2 first = polygon[index] - origin;
            const Point2 second = polygon[index + 1] - origin;
            area += first.x() * second.y() - first.y() * second.x();
        }
        return area;
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

    Polygon makeOperationArea()
    {
        return {
            Point2(0.0, 0.0),
            Point2(10.0, 0.0),
            Point2(10.0, 10.0),
            Point2(0.0, 10.0)
        };
    }

    void emptyEnvironmentIsAccepted()
    {
        const AStarPlanner planner({});
        require(!planner.hasOperationArea(), "unconstrained environment should have no operation area");
        require(planner.originalObstacles().empty(), "empty input should retain no obstacles");
        require(planner.obstacles().empty(), "empty input should produce no effective obstacles");
        requireThrows<std::logic_error>(
            [&planner] { static_cast<void>(planner.operationArea()); },
            "operationArea should reject access when none is configured");
    }

    void clockwiseObstacleIsNormalized()
    {
        const Polygon clockwiseSquare{
            Point2(0.0, 0.0), Point2(0.0, 4.0),
            Point2(4.0, 4.0), Point2(4.0, 0.0)
        };

        const AStarPlanner planner({ clockwiseSquare });
        require(planner.obstacles().size() == 1, "valid obstacle should be retained");
        require(signedAreaTwice(planner.obstacles().front()) > 0.0,
            "obstacle should be normalized to counter-clockwise winding");
    }

    void duplicateAndCollinearVerticesAreRemoved()
    {
        const Polygon redundantRectangle{
            Point2(0.0, 0.0), Point2(2.0, 0.0), Point2(4.0, 0.0),
            Point2(4.0, 3.0), Point2(4.0, 3.0), Point2(0.0, 3.0),
            Point2(0.0, 0.0)
        };

        const AStarPlanner planner({ redundantRectangle });
        require(planner.obstacles().front().size() == 4,
            "normalization should remove duplicate and collinear boundary vertices");
    }

    void invalidObstacleGeometryIsRejected()
    {
        const Polygon concave{
            Point2(0.0, 0.0), Point2(4.0, 0.0), Point2(2.0, 1.0),
            Point2(4.0, 4.0), Point2(0.0, 4.0)
        };
        const Polygon selfIntersecting{
            Point2(0.0, 0.0), Point2(4.0, 4.0),
            Point2(0.0, 4.0), Point2(4.0, 0.0)
        };
        const Polygon nonFinite{
            Point2(0.0, 0.0),
            Point2(std::numeric_limits<double>::infinity(), 0.0),
            Point2(0.0, 1.0)
        };

        requireThrows<std::invalid_argument>(
            [&concave] { AStarPlanner planner({ concave }); },
            "concave obstacle should be rejected");
        requireThrows<std::invalid_argument>(
            [&selfIntersecting] { AStarPlanner planner({ selfIntersecting }); },
            "self-intersecting obstacle should be rejected");
        requireThrows<std::invalid_argument>(
            [&nonFinite] { AStarPlanner planner({ nonFinite }); },
            "non-finite obstacle should be rejected");
    }

    void undersizedObstacleIsIgnored()
    {
        const Polygon line{ Point2(0.0, 0.0), Point2(1.0, 1.0) };
        const AStarPlanner planner({ line });
        require(planner.originalObstacles().empty(), "undersized obstacle should not be retained");
        require(planner.obstacles().empty(), "undersized obstacle should not become effective geometry");
    }

    void operationAreaIsNormalized()
    {
        const Polygon clockwiseArea{
            Point2(0.0, 0.0), Point2(0.0, 10.0),
            Point2(10.0, 10.0), Point2(10.0, 0.0)
        };

        const AStarPlanner planner(clockwiseArea, {});
        require(planner.hasOperationArea(), "operation area should be recorded");
        require(signedAreaTwice(planner.operationArea()) > 0.0,
            "operation area should be normalized to counter-clockwise winding");
    }

    void invalidOperationAreaIsRejected()
    {
        const Polygon concaveArea{
            Point2(0.0, 0.0), Point2(10.0, 0.0), Point2(5.0, 2.0),
            Point2(10.0, 10.0), Point2(0.0, 10.0)
        };

        requireThrows<std::invalid_argument>(
            [&concaveArea] { AStarPlanner planner(concaveArea, {}); },
            "concave operation area should be rejected");
    }

    void containedObstacleIsRetainedUnchanged()
    {
        const Polygon obstacle{
            Point2(2.0, 2.0), Point2(4.0, 2.0),
            Point2(4.0, 4.0), Point2(2.0, 4.0)
        };

        const AStarPlanner planner(makeOperationArea(), { obstacle });
        require(planner.originalObstacles().size() == 1, "original obstacle should be retained");
        require(planner.obstacles().size() == 1, "contained obstacle should remain effective");
        require(planner.clippedObstacles().empty(), "unchanged obstacle should not be a clipped overlay");
    }

    void outsideObstacleIsDiscardedFromEffectiveGeometry()
    {
        const Polygon obstacle{
            Point2(12.0, 2.0), Point2(14.0, 2.0),
            Point2(14.0, 4.0), Point2(12.0, 4.0)
        };

        const AStarPlanner planner(makeOperationArea(), { obstacle });
        require(planner.originalObstacles().size() == 1, "outside obstacle should remain in original view");
        require(planner.obstacles().empty(), "outside obstacle should be absent from effective geometry");
        require(planner.clippedObstacles().empty(), "empty intersection should not create a clipped overlay");
    }

    void crossingObstacleIsClipped()
    {
        const Polygon obstacle{
            Point2(8.0, 2.0), Point2(12.0, 2.0),
            Point2(12.0, 8.0), Point2(8.0, 8.0)
        };

        const AStarPlanner planner(makeOperationArea(), { obstacle });
        require(planner.obstacles().size() == 1, "positive-area intersection should remain effective");
        require(planner.clippedObstacles().size() == 1, "changed geometry should be exposed as clipped");
        for (const auto& point : planner.obstacles().front())
        {
            require(point.x() >= -TEST_EPSILON && point.x() <= 10.0 + TEST_EPSILON &&
                    point.y() >= -TEST_EPSILON && point.y() <= 10.0 + TEST_EPSILON,
                "clipped obstacle point should remain inside operation-area bounds");
        }
    }

    void zeroAreaBoundaryContactIsDiscarded()
    {
        const Polygon obstacle{
            Point2(10.0, 2.0), Point2(12.0, 2.0),
            Point2(12.0, 4.0), Point2(10.0, 4.0)
        };

        const AStarPlanner planner(makeOperationArea(), { obstacle });
        require(planner.obstacles().empty(), "line-only boundary contact should not be effective geometry");
    }

    void containingObstacleClipsToOperationArea()
    {
        const Polygon obstacle{
            Point2(-5.0, -5.0), Point2(15.0, -5.0),
            Point2(15.0, 15.0), Point2(-5.0, 15.0)
        };

        const AStarPlanner planner(makeOperationArea(), { obstacle });
        require(planner.obstacles().size() == 1, "containing obstacle should have an effective intersection");
        require(planner.clippedObstacles().size() == 1, "containing obstacle should be reported as clipped");
        require(planner.obstacles().front().size() == 4, "clipped result should match rectangular operation area");
    }

    void translatedSmallPolygonRetainsArea()
    {
        const double offset = 1e9;
        const Polygon obstacle{
            Point2(offset, offset), Point2(offset + 0.01, offset),
            Point2(offset + 0.01, offset + 0.01), Point2(offset, offset + 0.01)
        };

        const AStarPlanner planner({ obstacle });
        require(planner.obstacles().size() == 1,
            "small polygon at large translated coordinates should remain valid");
        require(pointsNear(planner.obstacles().front().front(), obstacle.front()),
            "normalization should preserve translated polygon coordinates");
    }

    struct TestCase
    {
        const char* name;
        void (*run)();
    };

    const std::vector<TestCase> TEST_CASES{
        { "Empty environment is accepted", emptyEnvironmentIsAccepted },
        { "Clockwise obstacle is normalized", clockwiseObstacleIsNormalized },
        { "Duplicate and collinear vertices are removed", duplicateAndCollinearVerticesAreRemoved },
        { "Invalid obstacle geometry is rejected", invalidObstacleGeometryIsRejected },
        { "Undersized obstacle is ignored", undersizedObstacleIsIgnored },
        { "Operation area is normalized", operationAreaIsNormalized },
        { "Invalid operation area is rejected", invalidOperationAreaIsRejected },
        { "Contained obstacle is retained", containedObstacleIsRetainedUnchanged },
        { "Outside obstacle is discarded", outsideObstacleIsDiscardedFromEffectiveGeometry },
        { "Crossing obstacle is clipped", crossingObstacleIsClipped },
        { "Zero-area boundary contact is discarded", zeroAreaBoundaryContactIsDiscarded },
        { "Containing obstacle clips to operation area", containingObstacleClipsToOperationArea },
        { "Translated small polygon retains area", translatedSmallPolygonRetainsArea }
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
                      << elapsed << " ms)\n       " << error.what() << '\n';
        }
        catch (...)
        {
            ++failed;
            std::cout << "FAIL\n       Unknown exception\n";
        }
    }

    const auto elapsed = std::chrono::duration<double, std::milli>(
        Clock::now() - suiteStart).count();
    std::cout << "\nResult: " << TEST_CASES.size() - failed << " passed, "
              << failed << " failed, " << TEST_CASES.size() << " total ("
              << std::fixed << std::setprecision(2) << elapsed << " ms)\n";
    return failed == 0 ? 0 : 1;
}
