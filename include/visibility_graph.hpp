/*
 * visibility_graph.hpp – Header-only 2-D visibility graph.
 *
 * ────
 * Core features
 *   • Naïve O(N³) obstacle-only build, query-time insertion of S & G.
 *   • Segment-intersection tests plus “same-polygon chord” rejection.
 *   • Optional convex operation area with convex obstacle clipping.
 *   • Convex polygon normalization and validation.
 *   • Polygon-size validation (skips <3-vertex obstacles with a warning).
 *
 * Example (see main_demo.cpp):
 *   vg::VisibilityGraph vg(obstacles);
 *   vg.buildBasic();
 *   auto ids  = vg.injectQueryPts(S,G);
 *   auto path = vg.shortestPath(ids.first, ids.second);
 * ────
 */
#pragma once

#include <algorithm>
#include <cassert>
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vg
{
    // Geometry and graph component types.
    using Point2 = Eigen::Vector2d;
    using Polygon = std::vector<Point2>;   // Ordered vertices; validated by VisibilityGraph.
    struct Segment2 { Point2 a, b; };
    struct Edge { std::size_t to; double cost; };
    struct Vertex { Point2 pos; int poly_id; bool is_query; };

    // Base relative tolerance used by scale-aware geometric predicates.
    inline constexpr double EPS = 1e-12;

    /**
     * Position of a point relative to a simple CCW polygon.
     * Inside: strictly inside the polygon area
     * OnEdge: collinear with, and lying on, an edge or vertex
     * Outside: strictly outside
     */
    enum class PointLocation { Outside, OnEdge, Inside };

    /**
     * @class VisibilityGraph
     * @brief Obstacle visibility graph supporting query-time terminal insertion.
     *
     * Intended lifecycle: construct, call buildBasic(), inject a start/goal
     * pair, then call shortestPath(). Each call to injectQueryPts() replaces
     * the previous query pair while preserving the obstacle-only graph.
     *
     * @note The class provides no internal synchronization. Concurrent
     * read-only access is safe only while no thread is calling buildBasic() or
     * injectQueryPts().
     */
    class VisibilityGraph
    {
    public:
        /**
         * @brief Construct from a list of convex, simple polygons.
         *
         * Finite obstacle polygons with <3 vertices are skipped with a warning.
         * Valid polygons are normalized before their effective vertices are
         * added to the graph. Polygons are converted to counter-clockwise
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
        explicit VisibilityGraph(const std::vector<Polygon>& obstacles)
        {
            initializeObstacles(obstacles);
        }

        /**
         * @brief Construct a graph constrained to a convex operation area.
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
        VisibilityGraph(const Polygon& operationArea,
            const std::vector<Polygon>& obstacles)
            : _hasOperationArea(true)
        {
            _operationArea = normalizePolygon(operationArea, "operation area");

            initializeObstacles(obstacles);
        }

        /**
         * @brief Build the obstacle-only visibility graph (O(N³)).
         *
         * Recomputes _adjacency from scratch using naive all-pairs
         * visibility checks. Previously injected query vertices are removed.
         */
        void buildBasic()
        {
            _vertices.resize(_obstacleVertexCount);
            const std::size_t n = _obstacleVertexCount;
            _adjacency.assign(n, {});
            for (auto& nbrs : _adjacency)
                nbrs.reserve(6);

            // All-pairs visibility test
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = i + 1; j < n; ++j)
                    if (visible(i, j))
                        addEdge(i, j);

            _obstacleAdjacency = _adjacency;
            _isBuilt = true;
        }


        /**
         * @brief Inject start & goal terminals and connect them.
         *
         * @param S Start position.
         * @param G Goal position.
         * @return Pair of vertex indices (S,G) within the graph.
         *
         * @pre buildBasic() has been called for the obstacle graph.
         * @throws std::logic_error if buildBasic() has not been called.
         * @throws std::invalid_argument if S or G contains a non-finite
         * coordinate.
         * @throws std::runtime_error if S or G is inside an obstacle or on its
         * boundary, or is not strictly inside the operation area when one is
         * configured.
         *
         * Any previous query vertices are removed before the new query is
         * inserted. If S and G are coincident within the scale-aware point
         * tolerance, one query vertex is inserted and its index is returned
         * for both endpoints.
         *
         * Complexity O(N²) for each inserted point.
         */
        std::pair<std::size_t, std::size_t>
        injectQueryPts(const Point2& S, const Point2& G)
        {
            if (!_isBuilt)
                throw std::logic_error("injectQueryPts: buildBasic() must be called first");

            if (!isFinite(S))
                throw std::invalid_argument(
                    "injectQueryPts: Start contains a non-finite coordinate");
            if (!isFinite(G))
                throw std::invalid_argument(
                    "injectQueryPts: Goal contains a non-finite coordinate");

            if (_hasOperationArea)
            {
                if (pointInPolygon(S, _operationArea) != PointLocation::Inside)
                    throw std::runtime_error(
                        "injectQueryPts: Start must be strictly inside the operation area");
                if (pointInPolygon(G, _operationArea) != PointLocation::Inside)
                    throw std::runtime_error(
                        "injectQueryPts: Goal must be strictly inside the operation area");
            }

            // Reject points inside or on an obstacle boundary.
            for (const auto& poly : _obstacles)
            {
                if (pointInPolygon(S, poly) != PointLocation::Outside)
                    throw std::runtime_error("injectQueryPts: Start inside or on obstacle");
                if (pointInPolygon(G, poly) != PointLocation::Outside)
                    throw std::runtime_error("injectQueryPts: Goal inside or on obstacle");
            }

            restoreObstacleGraph();

            const std::size_t sid = addVertex(S, /*query=*/true, -1);
            connectQueryVertex(sid);

            if (pointsNear(S, G))
                return { sid, sid };

            const std::size_t gid = addVertex(G, /*query=*/true, -1);
            connectQueryVertex(gid);
            return { sid, gid };
        }

        /**
         * @brief Dijkstra shortest path in visibility graph.
         *
         * @param s Source vertex index.
         * @param g Target vertex index.
         * @return Sequence of points from @p s to @p g (inclusive); empty if
         *         no path exists.
         *
         * @pre The relevant graph edges have been created by buildBasic() and,
         * for query vertices, injectQueryPts().
         * @throws std::out_of_range if s or g is not a valid vertex index.
         *
         * Complexity O(E log V) with binary heap.
         */
        [[nodiscard]]
        std::vector<Point2> shortestPath(std::size_t s,
            std::size_t g) const
        {
            if (s >= _vertices.size() || g >= _vertices.size())
                throw std::out_of_range(
                    "shortestPath: source and goal must be valid vertex indices");

            const double INF = std::numeric_limits<double>::infinity();
            const std::size_t N = _vertices.size();

            std::vector<double>       dist(N, INF);
            std::vector<std::size_t>  prev(N, N);

            using Q = std::pair<double, std::size_t>;
            std::priority_queue<Q, std::vector<Q>, std::greater<>> pq;

            dist[s] = 0.0;
            pq.emplace(0.0, s);

            while (!pq.empty())
            {
                const auto [d, u] = pq.top();
                pq.pop();
                if (d > dist[u]) continue;   // stale
                if (u == g) break;           // reached goal

                for (const auto& e : _adjacency[u])
                {
                    const double alt = d + e.cost;
                    if (alt < dist[e.to])
                    {
                        dist[e.to] = alt;
                        prev[e.to] = u;
                        pq.emplace(alt, e.to);
                    }
                }
            }

            if (dist[g] == INF) return {};   // no path

            std::vector<Point2> path;
            for (auto v = g; v != N; v = prev[v])
                path.push_back(_vertices[v].pos);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Read-only getters
        bool hasOperationArea() const { return _hasOperationArea; }

        const Polygon& operationArea() const
        {
            if (!_hasOperationArea)
                throw std::logic_error("operationArea: graph has no operation area");
            return _operationArea;
        }

        // Normalized input obstacles before operation-area clipping, including
        // geometry outside the operation area.
        const std::vector<Polygon>& originalObstacles() const { return _originalObstacles; }

        // Positive-area effective obstacles whose geometry changed during clipping.
        const std::vector<Polygon>& clippedObstacles() const { return _clippedObstacles; }

        // Effective obstacles used to construct the visibility graph.
        const std::vector<Polygon>& obstacles() const { return _obstacles; }
        const std::vector<Vertex>& vertices()  const { return _vertices; }
        const std::vector<std::vector<Edge>>& adjacency() const { return _adjacency; }
        
        std::size_t numEdges() const
        {
            std::size_t half = 0;
            for (auto& nbrs : _adjacency) half += nbrs.size();
            return half / 2;
        }

    private:

        static bool isFinite(const Point2& point)
        {
            return std::isfinite(point.x()) && std::isfinite(point.y());
        }

        /**
         * @brief Validates, filters, and flattens input obstacles.
         */
        void initializeObstacles(const std::vector<Polygon>& obstacles)
        {
            std::size_t reserveN = 0;

            for (const auto& poly : obstacles)
            {
                for (const auto& point : poly)
                {
                    if (!isFinite(point))
                    {
                        throw std::invalid_argument(
                            "VisibilityGraph: obstacle contains a non-finite coordinate");
                    }
                }
                if (poly.size() < 3)
                {
                    std::cerr << "[VG] Warning: polygon with " << poly.size()
                              << " vertex/vertices ignored (need >=3).\n";
                    continue;
                }
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

                reserveN += effectiveObstacle.size();
                _obstacles.push_back(std::move(effectiveObstacle));
            }

            _vertices.reserve(reserveN);
            _adjacency.reserve(reserveN);

            for (std::size_t pid = 0; pid < _obstacles.size(); ++pid)
            {
                for (const auto& pt : _obstacles[pid])
                {
                    if (_hasOperationArea &&
                        pointInPolygon(pt, _operationArea) == PointLocation::OnEdge)
                    {
                        continue;
                    }
                    addVertex(pt, /*query=*/false, static_cast<int>(pid));
                }
            }

            _obstacleVertexCount = _vertices.size();
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

        static Polygon normalizePolygon(const Polygon& input,
            const char* polygonRole)
        {
            for (const auto& point : input)
            {
                if (!std::isfinite(point.x()) || !std::isfinite(point.y()))
                {
                    throw std::invalid_argument(
                        std::string("VisibilityGraph: ") + polygonRole +
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
                    std::string("VisibilityGraph: ") + polygonRole +
                    " has fewer than three distinct non-collinear vertices");
            }
            if (!isSimplePolygon(normalized))
            {
                throw std::invalid_argument(
                    std::string("VisibilityGraph: ") + polygonRole +
                    " must be simple and non-self-intersecting");
            }

            const double area = signedAreaTwice(normalized);
            if (std::abs(area) <= polygonAreaTolerance(normalized))
            {
                throw std::invalid_argument(
                    std::string("VisibilityGraph: ") + polygonRole +
                    " must have nonzero area");
            }
            if (area < 0.0)
                std::reverse(normalized.begin(), normalized.end());

            if (!isConvexCounterClockwise(normalized))
            {
                throw std::invalid_argument(
                    std::string("VisibilityGraph: ") + polygonRole +
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
        std::vector<Vertex>               _vertices;  // Obstacle vertices plus active queries.
        std::vector<std::vector<Edge>>    _adjacency; // Neighbors for each active vertex.
        std::vector<std::vector<Edge>>    _obstacleAdjacency; // Cached obstacle-only graph.
        std::size_t                       _obstacleVertexCount = 0;
        bool                              _isBuilt = false;

        /**
         * @brief Removes query vertices and restores cached obstacle-only edges.
         */
        void restoreObstacleGraph()
        {
            _vertices.resize(_obstacleVertexCount);
            _adjacency = _obstacleAdjacency;
        }

        /**
         * @brief Adds a new vertex to the graph.
         *
         * This method appends a vertex to the internal vertex list and initializes
         * an empty adjacency list entry for it.
         *
         * @param p      The 2D point position of the vertex.
         * @param query  True if this vertex is a query point (e.g. start/goal), false if part of an obstacle.
         * @param pid    Polygon ID this vertex belongs to; -1 for query vertices.
         * @return The index of the newly added vertex.
         */
        std::size_t addVertex(const Point2& p, bool query, int pid)
        {
            _vertices.push_back({ p, pid, query });
            _adjacency.push_back({});
            return _vertices.size() - 1;
        }

        /**
         * @brief Adds an undirected edge between two existing vertices.
         *
         * The edge weight is the Euclidean distance between the two vertices.
         * Adds the edge in both directions (i → j and j → i).
         *
         * @param i Index of the first vertex.
         * @param j Index of the second vertex.
         */
        void addEdge(std::size_t i, std::size_t j)
        {
            assert(i < _vertices.size() && j < _vertices.size());

            const double w = (_vertices[i].pos - _vertices[j].pos).norm();
            _adjacency[i].push_back({ j, w });
            _adjacency[j].push_back({ i, w });
        }

        /**
         * @brief Determines if two vertices are mutually visible.
         *
         * Checks whether the line segment between two vertices intersects any obstacle edge.
         * Obstacle edges incident to either candidate endpoint are skipped so
         * a path may meet or follow an obstacle at one of its vertices.
         * Also rejects chords lying entirely within a polygon.
         *
         * @param i Index of the first vertex.
         * @param j Index of the second vertex.
         * @return True if the segment i–j is visible (not blocked by any obstacle), false otherwise.
         */
        bool visible(std::size_t i, std::size_t j) const
        {
            assert(i < _vertices.size() && j < _vertices.size());

            const Segment2 seg{ _vertices[i].pos, _vertices[j].pos };
            if (pointsNear(seg.a, seg.b)) return false;

            // Skip obstacle edges incident to a candidate endpoint using the
            // scale-aware point tolerance rather than exact equality.
            for (const auto& poly : _obstacles)
            {
                const std::size_t m = poly.size();
                for (std::size_t k = 0; k < m; ++k)
                {
                    const std::size_t k2 = (k + 1) % m;

                    if (pointsNear(poly[k], seg.a) ||
                        pointsNear(poly[k], seg.b) ||
                        pointsNear(poly[k2], seg.a) ||
                        pointsNear(poly[k2], seg.b))
                        continue;

                    if (segmentsIntersect(seg, { poly[k], poly[k2] }))
                        return false;
                }
            }

            // Same-polygon chord passes through interior?
            const int pidA = _vertices[i].poly_id;
            const int pidB = _vertices[j].poly_id;
            if (pidA != -1 && pidA == pidB)
            {
                const Point2 mid = 0.5 * (seg.a + seg.b);
                if (pointInPolygon(mid, _obstacles[pidA]) == PointLocation::Inside)
                    return false;
            }
            return true;
        }

        /**
         * @brief Connects a newly appended query vertex to visible earlier vertices.
         *
         * S is connected to obstacle vertices. G is then connected to obstacle
         * vertices and S, which permits a direct S-G edge when unobstructed.
         *
         * @param q Index of the query vertex to connect.
         */
        void connectQueryVertex(std::size_t q)
        {
            for (std::size_t i = 0; i < q; ++i)
                if (visible(i, q))
                    addEdge(i, q);
        }


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

        /**
         * @brief Classifies a point with respect to a polygon (even-odd rule).
         *
         * Implements the crossing-number test. If the point lies on an edge
         * within the scale-aware tolerance, it is reported as
         * PointLocation::OnEdge; otherwise the usual inside/outside result is
         * returned.
         *
         * @param p    Query point.
         * @param poly Simple, non-self-intersecting polygon in CCW order.
         * @return     PointLocation enum value.
         *
         * Complexity O(n) where n = poly.size().
         */
        static PointLocation pointInPolygon(const Point2& p,
            const Polygon& poly)
        {
            bool inside = false;
            const std::size_t n = poly.size();

            for (std::size_t i = 0, j = n - 1; i < n; j = i++)
            {
                const Point2& a = poly[j];
                const Point2& b = poly[i];

                // Boundary test (collinear and within segment)
                if (orientationSign(a, b, p) == 0 && onSegment(a, b, p))
                    return PointLocation::OnEdge;

                // Ray-casting toggle
                const bool hit = ((a.y() > p.y()) != (b.y() > p.y())) &&
                                 (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x());
                if (hit) inside = !inside;
            }
            return inside ? PointLocation::Inside : PointLocation::Outside;
        }
    };
} // namespace vg

