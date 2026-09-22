/*
 * a_star_planner.hpp – Environment model for the AStarPlanner library.
 *
 * ────
 * Provides validated polygon and operation-area geometry for grid-based
 * planning. Occupancy-grid construction and A* search are provided separately.
 * ────
 */
#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace astar
{
    class PolygonRasterizer;

    // Public world-coordinate geometry types.
    using Point2 = Eigen::Vector2d;
    using Polygon = std::vector<Point2>;   // Ordered vertices; validated by AStarPlanner.
    struct Segment2 { Point2 a, b; };

    // Base relative tolerance used by scale-aware geometric predicates.
    inline constexpr double EPS = 1e-12;

    /**
     * @class AStarPlanner
     * @brief Validated polygon environment for grid-based A* planning.
     */
    class AStarPlanner
    {
    public:
        /**
         * @brief Construct from a list of convex, simple polygons.
         *
         * Finite obstacle polygons with <3 vertices are ignored.
         * Valid polygons are normalized before their effective geometry is
         * retained. Polygons are converted to counter-clockwise
         * order. A repeated closing point, consecutive duplicates, and
         * redundant collinear boundary points are removed. Non-finite,
         * self-intersecting, degenerate, and concave polygons are rejected.
         * Overlapping obstacles are not validated.
         *
         * @param obstacles List of polygons representing obstacles.
         * @throws std::invalid_argument if an obstacle contains non-finite
         * coordinates or, after the undersized-input filter, is
         * self-intersecting, degenerate, or concave.
         */
        explicit AStarPlanner(const std::vector<Polygon>& obstacles)
        {
            initializeObstacles(obstacles);
        }

        /**
         * @brief Construct an environment constrained to a convex operation area.
         *
         * Start and goal queries must be strictly inside @p operationArea.
         * Obstacles are clipped to the operation area. Obstacles wholly
         * outside it, or touching it with zero intersection area, are ignored.
         * Overlapping obstacles are not validated.
         *
         * @param operationArea Convex, simple keep-in area.
         * @param obstacles List of convex obstacle polygons.
         * @throws std::invalid_argument if the operation area is non-finite,
         * non-simple, degenerate, or concave, or if an obstacle fails the same
         * validation after the undersized-input filter.
         */
        AStarPlanner(const Polygon& operationArea,
            const std::vector<Polygon>& obstacles)
            : _hasOperationArea(true)
        {
            _operationArea = normalizePolygon(operationArea, "operation area");

            initializeObstacles(obstacles);
        }

        // Read-only getters
        bool hasOperationArea() const { return _hasOperationArea; }

        const Polygon& operationArea() const
        {
            if (!_hasOperationArea)
                throw std::logic_error("operationArea: planner has no operation area");
            return _operationArea;
        }

        // Normalized input obstacles before operation-area clipping, including
        // geometry outside the operation area.
        const std::vector<Polygon>& originalObstacles() const { return _originalObstacles; }

        // Positive-area effective obstacles whose geometry changed during clipping.
        const std::vector<Polygon>& clippedObstacles() const { return _clippedObstacles; }

        // Effective obstacles available to occupancy-grid builders.
        const std::vector<Polygon>& obstacles() const { return _obstacles; }

    private:
        friend class PolygonRasterizer;

        static bool isFinite(const Point2& point)
        {
            return std::isfinite(point.x()) && std::isfinite(point.y());
        }

        /**
         * @brief Validates and filters input obstacles.
         */
        void initializeObstacles(const std::vector<Polygon>& obstacles)
        {
            for (const auto& poly : obstacles)
            {
                for (const auto& point : poly)
                {
                    if (!isFinite(point))
                    {
                        throw std::invalid_argument(
                            "AStarPlanner: obstacle contains a non-finite coordinate");
                    }
                }
                if (poly.size() < 3)
                    continue;
                Polygon normalizedObstacle = normalizePolygon(poly, "obstacle");

                _originalObstacles.push_back(normalizedObstacle);

                Polygon effectiveObstacle = _hasOperationArea
                    ? clipConvexPolygon(normalizedObstacle, _operationArea)
                    : normalizedObstacle;
                if (effectiveObstacle.size() < 3 ||
                    std::abs(signedAreaTwice(effectiveObstacle)) <=
                        polygonAreaTolerance(effectiveObstacle))
                {
                    continue;
                }

                if (_hasOperationArea &&
                    !haveSameOrderedVertices(normalizedObstacle, effectiveObstacle))
                {
                    _clippedObstacles.push_back(effectiveObstacle);
                }

                _obstacles.push_back(std::move(effectiveObstacle));
            }
        }

        static double pointTolerance(const Point2& lhs, const Point2& rhs)
        {
            const double scale = std::max({
                1.0,
                std::abs(lhs.x()), std::abs(lhs.y()),
                std::abs(rhs.x()), std::abs(rhs.y())
            });
            return EPS * scale;
        }

        static bool pointsNear(const Point2& lhs, const Point2& rhs)
        {
            const double tolerance = pointTolerance(lhs, rhs);
            return (lhs - rhs).squaredNorm() <= tolerance * tolerance;
        }

        static double orientationTolerance(const Point2& a,
            const Point2& b,
            const Point2& c)
        {
            const double scale = std::max(
                1.0,
                (b - a).norm() * (c - a).norm());
            return EPS * scale;
        }

        static int orientationSign(const Point2& a,
            const Point2& b,
            const Point2& c)
        {
            const double orientation = orient2D(a, b, c);
            const double tolerance = orientationTolerance(a, b, c);
            if (orientation > tolerance) return 1;
            if (orientation < -tolerance) return -1;
            return 0;
        }

        static double polygonAreaTolerance(const Polygon& poly)
        {
            double minX = poly.front().x();
            double maxX = minX;
            double minY = poly.front().y();
            double maxY = minY;

            for (const auto& point : poly)
            {
                minX = std::min(minX, point.x());
                maxX = std::max(maxX, point.x());
                minY = std::min(minY, point.y());
                maxY = std::max(maxY, point.y());
            }

            return EPS * std::max(1.0, (maxX - minX) * (maxY - minY));
        }

        static bool isSimplePolygon(const Polygon& poly)
        {
            const std::size_t n = poly.size();
            for (std::size_t i = 0; i < n; ++i)
            {
                const std::size_t iNext = (i + 1) % n;
                const Segment2 first{ poly[i], poly[iNext] };

                for (std::size_t j = i + 1; j < n; ++j)
                {
                    const std::size_t jNext = (j + 1) % n;
                    if (i == j || iNext == j || jNext == i)
                        continue;

                    if (segmentsIntersect(first, { poly[j], poly[jNext] }))
                        return false;
                }
            }
            return true;
        }

        static Polygon normalizePolygon(
            const Polygon& input,
            const char* polygonRole,
            const char* componentName = "AStarPlanner")
        {
            for (const auto& point : input)
            {
                if (!std::isfinite(point.x()) || !std::isfinite(point.y()))
                {
                    throw std::invalid_argument(
                        std::string(componentName) + ": " + polygonRole +
                        " contains a non-finite coordinate");
                }
            }

            Polygon normalized;
            normalized.reserve(input.size());
            for (const auto& point : input)
            {
                if (normalized.empty() || !pointsNear(normalized.back(), point))
                    normalized.push_back(point);
            }
            if (normalized.size() > 1 &&
                pointsNear(normalized.front(), normalized.back()))
            {
                normalized.pop_back();
            }

            bool removedPoint = true;
            while (removedPoint && normalized.size() >= 3)
            {
                removedPoint = false;
                for (std::size_t i = 0; i < normalized.size(); ++i)
                {
                    const std::size_t previous =
                        (i + normalized.size() - 1) % normalized.size();
                    const std::size_t next = (i + 1) % normalized.size();
                    if (orientationSign(
                            normalized[previous], normalized[i], normalized[next]) == 0 &&
                        onSegment(normalized[previous], normalized[next], normalized[i]))
                    {
                        normalized.erase(normalized.begin() +
                            static_cast<std::ptrdiff_t>(i));
                        removedPoint = true;
                        break;
                    }
                }
            }

            if (normalized.size() < 3)
            {
                throw std::invalid_argument(
                    std::string(componentName) + ": " + polygonRole +
                    " has fewer than three distinct non-collinear vertices");
            }
            if (!isSimplePolygon(normalized))
            {
                throw std::invalid_argument(
                    std::string(componentName) + ": " + polygonRole +
                    " must be simple and non-self-intersecting");
            }

            const double area = signedAreaTwice(normalized);
            if (std::abs(area) <= polygonAreaTolerance(normalized))
            {
                throw std::invalid_argument(
                    std::string(componentName) + ": " + polygonRole +
                    " must have nonzero area");
            }
            if (area < 0.0)
                std::reverse(normalized.begin(), normalized.end());

            if (!isConvexCounterClockwise(normalized))
            {
                throw std::invalid_argument(
                    std::string(componentName) + ": " + polygonRole +
                    " must be convex");
            }
            return normalized;
        }

        static double signedAreaTwice(const Polygon& poly)
        {
            // Translate to a nearby origin before multiplying coordinates.
            // Polygon area is translation invariant, and this avoids the loss
            // of precision caused by subtracting large global-coordinate
            // products for a comparatively small polygon.
            const Point2& origin = poly.front();
            double area = 0.0;
            for (std::size_t i = 1; i + 1 < poly.size(); ++i)
            {
                area += orient2D(origin, poly[i], poly[i + 1]);
            }
            return area;
        }

        static bool haveSameOrderedVertices(const Polygon& lhs, const Polygon& rhs)
        {
            if (lhs.size() != rhs.size())
                return false;

            for (std::size_t i = 0; i < lhs.size(); ++i)
            {
                if (!pointsNear(lhs[i], rhs[i]))
                    return false;
            }
            return true;
        }

        static void appendUniquePoint(Polygon& poly, const Point2& point)
        {
            if (poly.empty() || !pointsNear(poly.back(), point))
            {
                poly.push_back(point);
            }
        }

        static Polygon removeDuplicateClosingPoint(Polygon poly)
        {
            if (poly.size() > 1 && pointsNear(poly.front(), poly.back()))
            {
                poly.pop_back();
            }
            return poly;
        }

        static Point2 intersectWithBoundary(const Point2& start,
            const Point2& end,
            const Point2& boundaryStart,
            const Point2& boundaryEnd)
        {
            const double startSide = orient2D(
                boundaryStart, boundaryEnd, start);
            const double endSide = orient2D(
                boundaryStart, boundaryEnd, end);
            const double t = startSide / (startSide - endSide);
            return start + std::clamp(t, 0.0, 1.0) * (end - start);
        }

        /**
         * @brief Intersects one convex CCW polygon with another.
         *
         * Uses Sutherland-Hodgman clipping. Points on the operation-area
         * boundary are retained. Empty, point-only, and line-only results are
         * filtered by initializeObstacles().
         */
        static Polygon clipConvexPolygon(const Polygon& subject,
            const Polygon& clippingArea)
        {
            Polygon output = subject;

            for (std::size_t i = 0; i < clippingArea.size(); ++i)
            {
                if (output.empty())
                    break;

                const Point2& boundaryStart = clippingArea[i];
                const Point2& boundaryEnd =
                    clippingArea[(i + 1) % clippingArea.size()];
                Polygon input = std::move(output);
                output.clear();
                output.reserve(input.size() + 1);

                Point2 start = input.back();
                bool startInside = orientationSign(
                    boundaryStart, boundaryEnd, start) >= 0;

                for (const auto& end : input)
                {
                    const bool endInside = orientationSign(
                        boundaryStart, boundaryEnd, end) >= 0;

                    if (endInside)
                    {
                        if (!startInside)
                        {
                            appendUniquePoint(output, intersectWithBoundary(
                                start, end, boundaryStart, boundaryEnd));
                        }
                        appendUniquePoint(output, end);
                    }
                    else if (startInside)
                    {
                        appendUniquePoint(output, intersectWithBoundary(
                            start, end, boundaryStart, boundaryEnd));
                    }

                    start = end;
                    startInside = endInside;
                }

                output = removeDuplicateClosingPoint(std::move(output));
            }

            return output;
        }

        /**
         * @brief Checks that an ordered polygon is convex and counter-clockwise.
         *
         * Collinear consecutive vertices are allowed, but the polygon must
         * contain at least one counter-clockwise turn and no clockwise turns.
         * The polygon is normalized and checked for simplicity before this
         * function is called.
         */
        static bool isConvexCounterClockwise(const Polygon& poly)
        {
            bool hasCounterClockwiseTurn = false;
            const std::size_t n = poly.size();

            for (std::size_t i = 0; i < n; ++i)
            {
                const int turn = orientationSign(
                    poly[i],
                    poly[(i + 1) % n],
                    poly[(i + 2) % n]);

                if (turn < 0)
                    return false;
                if (turn > 0)
                    hasCounterClockwiseTurn = true;
            }

            return hasCounterClockwiseTurn;
        }

        // Data
        Polygon                           _operationArea;
        bool                              _hasOperationArea = false;
        std::vector<Polygon>              _originalObstacles;
        std::vector<Polygon>              _clippedObstacles;
        std::vector<Polygon>              _obstacles; // Effective obstacle geometry.
        /**
         * @brief Computes twice the signed area of the triangle (a, b, c).
         *
         * Used for orientation testing in 2D geometry. The result indicates
         * the relative orientation of the three points:
         *   - Positive: counter-clockwise (CCW)
         *   - Negative: clockwise (CW)
         *   - Zero: collinear
         *
         * @param a First point.
         * @param b Second point.
         * @param c Third point.
         * @return A scalar value proportional to the signed area of the triangle.
         */
        static double orient2D(const Point2& a, const Point2& b, const Point2& c)
        {
            return (b.x() - a.x()) * (c.y() - a.y()) -
                (b.y() - a.y()) * (c.x() - a.x());
        }

        /**
         * @brief Checks whether a point lies on a closed line segment.
         *
         * Assumes the point is collinear with the segment. Uses dot product
         * to determine if the point lies between endpoints a and b.
         *
         * @param a One endpoint of the segment.
         * @param b The other endpoint of the segment.
         * @param p The point to test.
         * @return true if point p lies on the segment [a, b], false otherwise.
         */
        static bool onSegment(const Point2& a, const Point2& b, const Point2& p)
        {
            const double tolerance = EPS * std::max(1.0, (b - a).squaredNorm());
            return (p - a).dot(b - p) >= -tolerance;
        }

        /**
         * @brief Tests whether two 2D line segments intersect or touch.
         *
         * Handles general and collinear nonzero-length segments using the
         * shared scale-aware tolerance. Zero-length segments return false.
         * Applies orientation tests to detect intersection, including endpoints
         * classified as lying on the other segment.
         *
         * @param s1 First segment.
         * @param s2 Second segment.
         * @return true if the segments intersect (or touch at endpoints),
         *         false otherwise.
         *
         * @note Complexity: O(1)
         */        
        static bool segmentsIntersect(const Segment2& s1, const Segment2& s2)
        {
            if (pointsNear(s1.a, s1.b)) return false;
            if (pointsNear(s2.a, s2.b)) return false;

            // Reject disjoint axis-aligned bounding boxes before orientation tests.
            const double min1x = std::min(s1.a.x(), s1.b.x()), max1x = std::max(s1.a.x(), s1.b.x());
            const double min2x = std::min(s2.a.x(), s2.b.x()), max2x = std::max(s2.a.x(), s2.b.x());
            if (max1x < min2x || max2x < min1x) return false;
            const double min1y = std::min(s1.a.y(), s1.b.y()), max1y = std::max(s1.a.y(), s1.b.y());
            const double min2y = std::min(s2.a.y(), s2.b.y()), max2y = std::max(s2.a.y(), s2.b.y());
            if (max1y < min2y || max2y < min1y) return false;

            const int o1 = orientationSign(s1.a, s1.b, s2.a);
            const int o2 = orientationSign(s1.a, s1.b, s2.b);
            const int o3 = orientationSign(s2.a, s2.b, s1.a);
            const int o4 = orientationSign(s2.a, s2.b, s1.b);

            if (o1 * o2 < 0 && o3 * o4 < 0) return true;
            if (o1 == 0 && onSegment(s1.a, s1.b, s2.a)) return true;
            if (o2 == 0 && onSegment(s1.a, s1.b, s2.b)) return true;
            if (o3 == 0 && onSegment(s2.a, s2.b, s1.a)) return true;
            if (o4 == 0 && onSegment(s2.a, s2.b, s1.b)) return true;

            return false;
        }

    };
} // namespace astar

