/*
 * Environment-model, occupancy-grid, and A* search tests for AStarPlanner.
 */

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "occupancy_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using astar::AStarGridPlanner;
    using astar::AStarOptions;
    using astar::AStarTieBreakPolicy;
    using astar::CellState;
    using astar::GridCell;
    using astar::GridGeometry;
    using astar::GridConnectivity;
    using astar::GridPlanResult;
    using astar::GridPlanStatus;
    using astar::GridRegion;
    using astar::GridSearchAlgorithm;
    using astar::GridSearchCellState;
    using astar::OccupancyGrid;
    using astar::OccupancyGridFusion;
    using astar::OccupancyGridInflator;
    using astar::PointRasterizer;
    using astar::Point2;
    using astar::Polygon;
    using astar::PolygonEnvironment;
    using astar::PolygonRasterizer;
    using astar::SubgridStorage;
    using astar::WorldBounds;
    using astar::aStarTieBreakPolicyName;
    using astar::gridPathToWorld;
    using astar::simplifyGridPath;

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

    Polygon obstacleInsideCell(const GridCell& cell)
    {
        constexpr double inset = 0.25;
        const double minimumX = static_cast<double>(cell.column) + inset;
        const double minimumY = static_cast<double>(cell.row) + inset;
        const double maximumX = static_cast<double>(cell.column + 1) - inset;
        const double maximumY = static_cast<double>(cell.row + 1) - inset;
        return {
            Point2(minimumX, minimumY), Point2(maximumX, minimumY),
            Point2(maximumX, maximumY), Point2(minimumX, maximumY)
        };
    }

    OccupancyGrid gridWithOccupiedCells(
        int width,
        int height,
        const std::vector<GridCell>& occupiedCells)
    {
        std::vector<Polygon> obstacles;
        obstacles.reserve(occupiedCells.size());
        for (const GridCell& cell : occupiedCells)
            obstacles.push_back(obstacleInsideCell(cell));
        return PolygonRasterizer::rasterize(
            GridGeometry(Point2(0.0, 0.0), 1.0, width, height),
            obstacles);
    }

    void requireValidFourConnectedPath(
        const OccupancyGrid& grid,
        const std::vector<GridCell>& path,
        const GridCell& start,
        const GridCell& goal)
    {
        require(!path.empty(), "successful path should not be empty");
        require(path.front() == start && path.back() == goal,
            "path should include the requested start and goal cells");
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            require(grid.isTraversable(path[index]),
                "every returned path cell should be traversable");
            if (index == 0)
                continue;
            const int stepDistance =
                std::abs(path[index].column - path[index - 1].column) +
                std::abs(path[index].row - path[index - 1].row);
            require(stepDistance == 1,
                "consecutive path cells should be four-connected neighbors");
        }
    }

    void requireValidEightConnectedPath(
        const OccupancyGrid& grid,
        const std::vector<GridCell>& path,
        const GridCell& start,
        const GridCell& goal,
        bool preventDiagonalCornerCutting = true)
    {
        require(!path.empty(), "successful path should not be empty");
        require(path.front() == start && path.back() == goal,
            "path should include the requested start and goal cells");
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            require(grid.isTraversable(path[index]),
                "every returned path cell should be traversable");
            if (index == 0)
                continue;

            const GridCell& previous = path[index - 1];
            const int columnStep = std::abs(path[index].column - previous.column);
            const int rowStep = std::abs(path[index].row - previous.row);
            require(std::max(columnStep, rowStep) == 1,
                "consecutive path cells should be eight-connected neighbors");
            if (preventDiagonalCornerCutting &&
                columnStep == 1 && rowStep == 1)
            {
                require(
                    grid.isTraversable({ path[index].column, previous.row }) &&
                    grid.isTraversable({ previous.column, path[index].row }),
                    "diagonal path steps should not cut occupied corners");
            }
        }
    }

    double pathMovementCost(const std::vector<GridCell>& path)
    {
        double cost = 0.0;
        for (std::size_t index = 1; index < path.size(); ++index)
        {
            const GridCell& previous = path[index - 1];
            const GridCell& current = path[index];
            const bool diagonal =
                previous.column != current.column &&
                previous.row != current.row;
            cost += diagonal ? std::sqrt(2.0) : 1.0;
        }
        return cost;
    }

    bool costsNear(double lhs, double rhs)
    {
        const double scale = std::max({ 1.0, std::abs(lhs), std::abs(rhs) });
        return std::abs(lhs - rhs) <= 1e-10 * scale;
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
        const PolygonEnvironment planner({});
        require(!planner.hasOperationArea(), "unconstrained environment should have no operation area");
        require(planner.normalizedObstacles().empty(), "empty input should retain no obstacles");
        require(planner.effectiveObstacles().empty(), "empty input should produce no effective obstacles");
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

        const PolygonEnvironment planner({ clockwiseSquare });
        require(planner.effectiveObstacles().size() == 1, "valid obstacle should be retained");
        require(signedAreaTwice(planner.effectiveObstacles().front()) > 0.0,
            "obstacle should be normalized to counter-clockwise winding");
    }

    void duplicateAndCollinearVerticesAreRemoved()
    {
        const Polygon redundantRectangle{
            Point2(0.0, 0.0), Point2(2.0, 0.0), Point2(4.0, 0.0),
            Point2(4.0, 3.0), Point2(4.0, 3.0), Point2(0.0, 3.0),
            Point2(0.0, 0.0)
        };

        const PolygonEnvironment planner({ redundantRectangle });
        require(planner.effectiveObstacles().front().size() == 4,
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
            [&concave] { PolygonEnvironment planner({ concave }); },
            "concave obstacle should be rejected");
        requireThrows<std::invalid_argument>(
            [&selfIntersecting] { PolygonEnvironment planner({ selfIntersecting }); },
            "self-intersecting obstacle should be rejected");
        requireThrows<std::invalid_argument>(
            [&nonFinite] { PolygonEnvironment planner({ nonFinite }); },
            "non-finite obstacle should be rejected");
    }

    void undersizedObstacleIsIgnored()
    {
        const Polygon line{ Point2(0.0, 0.0), Point2(1.0, 1.0) };
        std::ostringstream capturedErrors;
        std::streambuf* previousErrors = std::cerr.rdbuf(capturedErrors.rdbuf());
        const PolygonEnvironment planner({ line });
        std::cerr.rdbuf(previousErrors);

        require(planner.normalizedObstacles().empty(), "undersized obstacle should not be retained");
        require(planner.effectiveObstacles().empty(), "undersized obstacle should not become effective geometry");
        require(capturedErrors.str().empty(),
            "ignoring an undersized obstacle should not write to standard error");
    }

    void operationAreaIsNormalized()
    {
        const Polygon clockwiseArea{
            Point2(0.0, 0.0), Point2(0.0, 10.0),
            Point2(10.0, 10.0), Point2(10.0, 0.0)
        };

        const PolygonEnvironment planner(clockwiseArea, {});
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
            [&concaveArea] { PolygonEnvironment planner(concaveArea, {}); },
            "concave operation area should be rejected");
    }

    void containedObstacleIsRetainedUnchanged()
    {
        const Polygon obstacle{
            Point2(2.0, 2.0), Point2(4.0, 2.0),
            Point2(4.0, 4.0), Point2(2.0, 4.0)
        };

        const PolygonEnvironment planner(makeOperationArea(), { obstacle });
        require(planner.normalizedObstacles().size() == 1, "normalized obstacle should be retained");
        require(planner.effectiveObstacles().size() == 1, "contained obstacle should remain effective");
        require(planner.clippedObstacles().empty(), "unchanged obstacle should not be a clipped overlay");
    }

    void outsideObstacleIsDiscardedFromEffectiveGeometry()
    {
        const Polygon obstacle{
            Point2(12.0, 2.0), Point2(14.0, 2.0),
            Point2(14.0, 4.0), Point2(12.0, 4.0)
        };

        const PolygonEnvironment planner(makeOperationArea(), { obstacle });
        require(planner.normalizedObstacles().size() == 1, "outside obstacle should remain in normalized view");
        require(planner.effectiveObstacles().empty(), "outside obstacle should be absent from effective geometry");
        require(planner.clippedObstacles().empty(), "empty intersection should not create a clipped overlay");
    }

    void crossingObstacleIsClipped()
    {
        const Polygon obstacle{
            Point2(8.0, 2.0), Point2(12.0, 2.0),
            Point2(12.0, 8.0), Point2(8.0, 8.0)
        };

        const PolygonEnvironment planner(makeOperationArea(), { obstacle });
        require(planner.effectiveObstacles().size() == 1, "positive-area intersection should remain effective");
        require(planner.clippedObstacles().size() == 1, "changed geometry should be exposed as clipped");
        for (const auto& point : planner.effectiveObstacles().front())
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

        const PolygonEnvironment planner(makeOperationArea(), { obstacle });
        require(planner.effectiveObstacles().empty(), "line-only boundary contact should not be effective geometry");
    }

    void containingObstacleClipsToOperationArea()
    {
        const Polygon obstacle{
            Point2(-5.0, -5.0), Point2(15.0, -5.0),
            Point2(15.0, 15.0), Point2(-5.0, 15.0)
        };

        const PolygonEnvironment planner(makeOperationArea(), { obstacle });
        require(planner.effectiveObstacles().size() == 1, "containing obstacle should have an effective intersection");
        require(planner.clippedObstacles().size() == 1, "containing obstacle should be reported as clipped");
        require(planner.effectiveObstacles().front().size() == 4, "clipped result should match rectangular operation area");
    }

    void translatedSmallPolygonRetainsArea()
    {
        const double offset = 1e9;
        const Polygon obstacle{
            Point2(offset, offset), Point2(offset + 0.01, offset),
            Point2(offset + 0.01, offset + 0.01), Point2(offset, offset + 0.01)
        };

        const PolygonEnvironment planner({ obstacle });
        require(planner.effectiveObstacles().size() == 1,
            "small polygon at large translated coordinates should remain valid");
        require(pointsNear(planner.effectiveObstacles().front().front(), obstacle.front()),
            "normalization should preserve translated polygon coordinates");
    }

    void gridGeometryConvertsBetweenCoordinateFrames()
    {
        const GridGeometry geometry(Point2(-2.0, 3.0), 0.5, 4, 3);

        const auto lowerLeft = geometry.worldToCell(Point2(-1.9, 3.1));
        require(lowerLeft && *lowerLeft == GridCell{ 0, 0 },
            "lower-left world point should map to the first Cartesian cell");
        require(geometry.cellToImage(*lowerLeft) == cv::Point(0, 2),
            "lower-left Cartesian cell should map to the bottom OpenCV row");
        require(geometry.imageToCell(cv::Point(0, 2)) == lowerLeft,
            "image-to-cell conversion should invert cell-to-image conversion");
        require(pointsNear(geometry.cellCenterToWorld({ 0, 0 }), Point2(-1.75, 3.25)),
            "cell center should include the translated world origin");

        const cv::Point2d imageCenter = geometry.worldToImage(Point2(-1.75, 3.25));
        require(std::abs(imageCenter.x) <= TEST_EPSILON &&
                std::abs(imageCenter.y - 2.0) <= TEST_EPSILON,
            "world cell center should map to an integer OpenCV pixel center");
        require(pointsNear(geometry.imageToWorld(imageCenter), Point2(-1.75, 3.25)),
            "continuous world/image conversion should round trip");
    }

    void gridGeometryUsesHalfOpenWorldBounds()
    {
        const GridGeometry geometry(Point2(10.0, -4.0), 1.0, 3, 2);

        require(geometry.worldToCell(Point2(10.0, -4.0)) == GridCell{ 0, 0 },
            "minimum world boundary should be included");
        require(!geometry.worldToCell(Point2(13.0, -3.0)),
            "maximum x boundary should be excluded");
        require(!geometry.worldToCell(Point2(11.0, -2.0)),
            "maximum y boundary should be excluded");
        require(!geometry.worldToCell(Point2(9.999, -3.0)),
            "point below the minimum x boundary should be excluded");
        requireThrows<std::out_of_range>(
            [&geometry] { static_cast<void>(geometry.cellCenterToWorld({ 3, 0 })); },
            "cell-to-world conversion should reject an out-of-bounds cell");
    }

    void coveringGeometryRoundsExtentUpToWholeCells()
    {
        const WorldBounds requestedBounds{
            Point2(-1.0, 2.0), Point2(1.1, 3.01)
        };
        const GridGeometry geometry = GridGeometry::covering(
            requestedBounds, 0.5);

        require(geometry.width() == 5 && geometry.height() == 3,
            "covering geometry should round each extent upward");
        require(geometry.contains(requestedBounds),
            "covering geometry should contain all requested world bounds");
        require(pointsNear(geometry.worldMaximum(), Point2(1.5, 3.5)),
            "covering geometry should expose its snapped world maximum");
    }

    void coveringGeometryDefinesAuthoritativePlanningBoundary()
    {
        const WorldBounds requestedBounds{
            Point2(0.2, 0.4), Point2(2.3, 1.6)
        };
        const GridGeometry geometry = GridGeometry::covering(
            requestedBounds, 1.0);

        require(pointsNear(geometry.worldOrigin(), requestedBounds.minimum) &&
                pointsNear(geometry.worldMaximum(), Point2(3.2, 2.4)),
            "covering geometry should expose the whole-cell planning extent");
        require(geometry.worldToCell(Point2(2.8, 2.0)) == GridCell{ 2, 1 },
            "whole-cell coverage beyond the requested maximum should remain inside the grid");
        require(!geometry.worldToCell(geometry.worldMaximum()),
            "the authoritative maximum grid boundary should remain half-open");

        const OccupancyGrid grid(geometry, CellState::Free);
        const AStarGridPlanner planner;
        const GridPlanResult valid = planner.plan(grid, { 0, 0 }, { 2, 1 });
        require(valid.succeeded(),
            "the planner should use every cell in the resulting grid geometry");

        const GridPlanResult invalidGoal = planner.plan(grid, { 0, 0 }, { 3, 1 });
        require(invalidGoal.status == GridPlanStatus::GoalOutsideGrid &&
                invalidGoal.path.empty() && invalidGoal.simplifiedPath.empty(),
            "the planner should reject terminals beyond the resulting grid geometry");
    }

    void invalidGridGeometryIsRejected()
    {
        requireThrows<std::invalid_argument>(
            [] { GridGeometry geometry(Point2(0.0, 0.0), 0.0, 2, 2); },
            "zero resolution should be rejected");
        requireThrows<std::invalid_argument>(
            [] { GridGeometry geometry(Point2(0.0, 0.0), 1.0, 0, 2); },
            "zero width should be rejected");
        requireThrows<std::invalid_argument>(
            []
            {
                GridGeometry::covering(
                    WorldBounds{ Point2(1.0, 0.0), Point2(0.0, 1.0) }, 1.0);
            },
            "reversed world bounds should be rejected");
    }

    void alignedCoveringUsesStableWorldLattice()
    {
        const WorldBounds bounds{ Point2(-0.2, 0.2), Point2(2.2, 2.2) };
        const GridGeometry geometry = GridGeometry::alignedCovering(bounds, 1.0);

        require(pointsNear(geometry.worldOrigin(), Point2(-1.0, 0.0)),
            "aligned covering should snap its origin down to the anchor lattice");
        require(geometry.width() == 4 && geometry.height() == 3,
            "aligned covering should extend upward to contain the requested bounds");
        require(pointsNear(geometry.worldMaximum(), Point2(3.0, 3.0)),
            "aligned covering should end on the same world lattice");

        const GridGeometry shifted = GridGeometry::alignedCovering(
            bounds, 1.0, Point2(0.5, 0.5));
        require(pointsNear(shifted.worldOrigin(), Point2(-0.5, -0.5)),
            "custom alignment anchor should shift the grid lattice");
    }

    void polygonBoundsCanSizeRectangularMaps()
    {
        const Polygon polygon{
            Point2(-2.0, 3.0), Point2(5.1, 3.0),
            Point2(5.1, 4.2), Point2(-2.0, 4.2)
        };
        const GridGeometry geometry = GridGeometry::covering(polygon, 0.5);

        require(geometry.width() == 15 && geometry.height() == 3,
            "polygon bounds should create a rectangular grid without forcing a square");
        require(pointsNear(geometry.worldOrigin(), Point2(-2.0, 3.0)),
            "polygon covering should preserve the bounding-box minimum as origin");
    }

    void unconstrainedRasterizationPaintsCoveredCells()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const Polygon obstacle{
            Point2(0.75, 0.75), Point2(1.25, 0.75),
            Point2(1.25, 1.25), Point2(0.75, 1.25)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(geometry, { obstacle });
        require(grid.at({ 0, 0 }) == CellState::Occupied,
            "OpenCV polygon coverage should occupy the lower-left cell");
        require(grid.at({ 1, 0 }) == CellState::Occupied,
            "OpenCV polygon coverage should occupy the lower-right cell");
        require(grid.at({ 0, 1 }) == CellState::Occupied,
            "OpenCV polygon coverage should occupy the upper-left cell");
        require(grid.at({ 1, 1 }) == CellState::Occupied,
            "OpenCV polygon coverage should occupy the upper-right cell");
        require(grid.isTraversable({ 2, 2 }),
            "cell outside the obstacle should remain traversable");
        require(!grid.isTraversable({ -1, 0 }),
            "out-of-bounds cell should never be traversable");
    }

    void occupancyGridSupportsUniformInitialization()
    {
        const GridGeometry geometry(Point2(-2.0, 4.0), 0.5, 3, 2);
        const OccupancyGrid freeGrid(geometry, CellState::Free);
        const OccupancyGrid occupiedGrid(geometry, CellState::Occupied);

        for (int row = 0; row < geometry.height(); ++row)
        {
            for (int column = 0; column < geometry.width(); ++column)
            {
                const GridCell cell{ column, row };
                require(freeGrid.at(cell) == CellState::Free,
                    "free initialization should set every grid cell to free");
                require(occupiedGrid.at(cell) == CellState::Occupied,
                    "occupied initialization should set every grid cell to occupied");
            }
        }

        require(cv::countNonZero(freeGrid.imageView()) == 0,
            "free initialization should fill the OpenCV image with zero");
        require(cv::countNonZero(occupiedGrid.imageView()) ==
                geometry.width() * geometry.height(),
            "occupied initialization should fill every OpenCV image pixel");

        requireThrows<std::invalid_argument>(
            [&geometry]
            {
                OccupancyGrid invalid(
                    geometry, static_cast<CellState>(1));
            },
            "occupancy grid should reject a non-binary initial cell state");
    }

    void inflationUsesConservativeCircularRadius()
    {
        const OccupancyGrid source = gridWithOccupiedCells(9, 9, { { 4, 4 } });
        const OccupancyGrid inflated = OccupancyGridInflator::inflate(source, 1.01);

        require(inflated.at({ 4, 4 }) == CellState::Occupied,
            "inflation should retain the source obstacle");
        require(inflated.at({ 6, 4 }) == CellState::Occupied &&
                inflated.at({ 5, 5 }) == CellState::Occupied,
            "world radius should round up to a two-cell circular kernel");
        require(inflated.isTraversable({ 6, 5 }),
            "circular inflation should exclude offsets outside its radius");
        require(source.isTraversable({ 6, 4 }) && source.isTraversable({ 5, 5 }),
            "inflation should not modify the source grid");
    }

    void inflationTreatsGridBoundaryAsOccupied()
    {
        const OccupancyGrid source(
            GridGeometry(Point2(0.0, 0.0), 1.0, 7, 6),
            CellState::Free);
        const OccupancyGrid inflated = OccupancyGridInflator::inflate(source, 1.0);

        for (int column = 0; column < inflated.width(); ++column)
        {
            require(!inflated.isTraversable({ column, 0 }) &&
                    !inflated.isTraversable({ column, inflated.height() - 1 }),
                "inflation should treat horizontal map boundaries as occupied");
        }
        for (int row = 0; row < inflated.height(); ++row)
        {
            require(!inflated.isTraversable({ 0, row }) &&
                    !inflated.isTraversable({ inflated.width() - 1, row }),
                "inflation should treat vertical map boundaries as occupied");
        }
        require(inflated.isTraversable({ 1, 1 }) &&
                inflated.isTraversable({ 5, 4 }),
            "one-cell boundary inflation should leave the remaining interior free");
    }

    void zeroInflationPreservesGridMetadataAndValidatesRadius()
    {
        const OccupancyGrid master = gridWithOccupiedCells(7, 7, { { 3, 3 } });
        const OccupancyGrid source = master.subgrid({ { 1, 2 }, 5, 4 });
        const OccupancyGrid inflated = OccupancyGridInflator::inflate(source, 0.0);

        require(inflated.masterCellOffset() == source.masterCellOffset(),
            "inflation should preserve the cumulative master-cell offset");
        require(pointsNear(
                inflated.geometry().worldOrigin(), source.geometry().worldOrigin()) &&
                inflated.geometry().resolution() == source.geometry().resolution() &&
                inflated.width() == source.width() && inflated.height() == source.height(),
            "inflation should preserve grid geometry");
        require(inflated.imageView().data != source.imageView().data,
            "inflation should return independently owned pixels even at zero radius");

        cv::Mat1b unequalPixels;
        cv::compare(
            inflated.imageView(), source.imageView(), unequalPixels, cv::CMP_NE);
        require(cv::countNonZero(unequalPixels) == 0,
            "zero-radius inflation should preserve every occupancy value");

        requireThrows<std::invalid_argument>(
            [&source]
            {
                static_cast<void>(OccupancyGridInflator::inflate(source, -0.1));
            },
            "inflation should reject a negative safety radius");
        requireThrows<std::invalid_argument>(
            [&source]
            {
                static_cast<void>(OccupancyGridInflator::inflate(
                    source, std::numeric_limits<double>::quiet_NaN()));
            },
            "inflation should reject a non-finite safety radius");
        requireThrows<std::invalid_argument>(
            [&source]
            {
                static_cast<void>(OccupancyGridInflator::inflate(
                    source, std::numeric_limits<double>::max()));
            },
            "inflation should reject an unrepresentable safety radius");
    }

    void operationAreaPaintsCoveredCellsFree()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 3);
        const Polygon operationArea{
            Point2(0.25, 0.25), Point2(2.75, 0.25),
            Point2(2.75, 2.75), Point2(0.25, 2.75)
        };

        const OccupancyGrid grid =
            PolygonRasterizer::rasterize(geometry, operationArea, {});
        require(grid.isTraversable({ 1, 1 }),
            "operation-area interior should be painted free");
        require(grid.isTraversable({ 0, 0 }) && grid.isTraversable({ 2, 2 }),
            "OpenCV polygon coverage should paint covered boundary cells free");
    }

    void obstaclePaintingUsesOpenCvCoverage()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 2);
        const Polygon obstacle{
            Point2(1.0, 0.2), Point2(1.2, 0.2),
            Point2(1.2, 0.8), Point2(1.0, 0.8)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(geometry, { obstacle });
        require(grid.isTraversable({ 0, 0 }),
            "touching a cell boundary should not conservatively occupy both cells");
        require(grid.at({ 1, 0 }) == CellState::Occupied,
            "OpenCV polygon coverage should paint the obstacle cell occupied");
        require(grid.isTraversable({ 2, 0 }),
            "unrelated cell should remain free");
    }

    void rasterizerNormalizesValidPolygons()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 3);
        const Polygon clockwiseWithRedundantVertices{
            Point2(0.25, 0.25), Point2(0.25, 1.75),
            Point2(1.75, 1.75), Point2(1.75, 1.0),
            Point2(1.75, 0.25), Point2(0.25, 0.25)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(
            geometry, { clockwiseWithRedundantVertices });

        require(grid.at({ 0, 0 }) == CellState::Occupied &&
                grid.at({ 1, 1 }) == CellState::Occupied,
            "rasterizer should normalize valid polygon winding and vertices");
        require(grid.isTraversable({ 2, 2 }),
            "normalizing an obstacle should not paint unrelated cells");
    }

    void rasterizerRejectsInvalidPolygons()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const Polygon concave{
            Point2(0.0, 0.0), Point2(3.0, 0.0), Point2(1.5, 1.0),
            Point2(3.0, 3.0), Point2(0.0, 3.0)
        };
        const Polygon selfIntersecting{
            Point2(0.0, 0.0), Point2(2.0, 2.0),
            Point2(0.0, 2.0), Point2(2.0, 0.0)
        };
        const Polygon degenerate{
            Point2(0.0, 0.0), Point2(1.0, 0.0), Point2(2.0, 0.0)
        };
        const Polygon nonFinite{
            Point2(0.0, 0.0),
            Point2(std::numeric_limits<double>::infinity(), 0.0),
            Point2(0.0, 1.0)
        };

        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(PolygonRasterizer::rasterize(geometry, { concave })); },
            "rasterizer should reject a concave obstacle");
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(PolygonRasterizer::rasterize(
                    geometry, selfIntersecting, {}));
            },
            "rasterizer should reject a self-intersecting operation area");
        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(PolygonRasterizer::rasterize(geometry, { degenerate })); },
            "rasterizer should reject a degenerate obstacle");
        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(PolygonRasterizer::rasterize(geometry, { nonFinite })); },
            "rasterizer should reject a non-finite obstacle");
    }

    void obstacleOutsideGridLeavesMapFree()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 2, 2);
        const Polygon obstacle{
            Point2(1e12, 1e12), Point2(1e12 + 10000.0, 1e12),
            Point2(1e12 + 10000.0, 1e12 + 10000.0), Point2(1e12, 1e12 + 10000.0)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(geometry, { obstacle });
        require(grid.isTraversable({ 0, 0 }) && grid.isTraversable({ 1, 1 }),
            "obstacle outside the grid should not affect occupancy");
    }

    void obstacleOverridesOperationAreaFreeSpace()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 3);
        const Polygon operationArea{
            Point2(0.0, 0.0), Point2(3.0, 0.0),
            Point2(3.0, 3.0), Point2(0.0, 3.0)
        };
        const Polygon obstacle{
            Point2(1.1, 1.1), Point2(1.9, 1.1),
            Point2(1.9, 1.9), Point2(1.1, 1.9)
        };

        const OccupancyGrid grid =
            PolygonRasterizer::rasterize(geometry, operationArea, { obstacle });
        require(grid.at({ 1, 1 }) == CellState::Occupied,
            "obstacle should override operation-area free space");
        require(grid.isTraversable({ 0, 0 }),
            "operation-area interior cells should remain free");
        require(grid.imageView().rows == 3 && grid.imageView().cols == 3,
            "OpenCV image dimensions should match grid dimensions");
    }

    void pointRasterizationPaintsContainedWorldPoints()
    {
        const GridGeometry geometry(Point2(-2.0, -1.0), 0.5, 6, 4);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();
        const std::vector<Point2> obstaclePoints{
            Point2(-2.0, -1.0),
            Point2(-2.0, -1.0), // Duplicate in the lower-left cell.
            Point2(-0.9, 0.1),
            Point2(0.99, 0.99),
            Point2(-2.01, 0.0),
            Point2(1.0, 0.0),
            Point2(0.0, 1.0),
            Point2(nan, 0.0),
            Point2(0.0, infinity)
        };

        const OccupancyGrid grid = PointRasterizer::rasterize(
            geometry, obstaclePoints);

        require(grid.at({ 0, 0 }) == CellState::Occupied &&
                grid.at({ 2, 2 }) == CellState::Occupied &&
                grid.at({ 5, 3 }) == CellState::Occupied,
            "contained world points should occupy their corresponding Cartesian cells");
        require(cv::countNonZero(grid.imageView()) == 3,
            "duplicates and invalid or out-of-bounds points should not add occupied cells");

        const OccupancyGrid emptyGrid = PointRasterizer::rasterize(geometry, {});
        require(cv::countNonZero(emptyGrid.imageView()) == 0,
            "an empty point collection should produce a completely free grid");
    }

    void occupancyFusionUsesOccupiedUnion()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const OccupancyGrid firstMaster = PointRasterizer::rasterize(
            geometry,
            { Point2(1.5, 1.5), Point2(2.5, 1.5) });
        const OccupancyGrid secondMaster = PointRasterizer::rasterize(
            geometry,
            { Point2(2.5, 1.5), Point2(1.5, 2.5) });
        const GridRegion region{ { 1, 1 }, 2, 2 };
        const OccupancyGrid first = firstMaster.subgrid(
            region, SubgridStorage::IndependentCopy);
        const OccupancyGrid second = secondMaster.subgrid(
            region, SubgridStorage::IndependentCopy);

        const OccupancyGrid fused = OccupancyGridFusion::occupiedUnion(
            first, second);

        require(fused.at({ 0, 0 }) == CellState::Occupied &&
                fused.at({ 1, 0 }) == CellState::Occupied &&
                fused.at({ 0, 1 }) == CellState::Occupied &&
                fused.at({ 1, 1 }) == CellState::Free,
            "occupied union should mark a cell occupied when either input is occupied");
        require(fused.masterCellOffset() == GridCell{ 1, 1 } &&
                pointsNear(fused.geometry().worldOrigin(), Point2(1.0, 1.0)),
            "occupied union should preserve grid geometry and master offset");
        require(fused.imageView().data != first.imageView().data &&
                fused.imageView().data != second.imageView().data,
            "occupied union should return independently owned storage");
        require(cv::countNonZero(first.imageView()) == 2 &&
                cv::countNonZero(second.imageView()) == 2,
            "occupied union should not modify either input grid");
    }

    void occupancyFusionRejectsIncompatibleGrids()
    {
        const OccupancyGrid reference(
            GridGeometry(Point2(0.0, 0.0), 1.0, 2, 2),
            CellState::Free);

        const OccupancyGrid differentDimensions(
            GridGeometry(Point2(0.0, 0.0), 1.0, 3, 2),
            CellState::Free);
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(OccupancyGridFusion::occupiedUnion(
                    reference, differentDimensions));
            },
            "occupied union should reject different grid dimensions");

        const OccupancyGrid differentOrigin(
            GridGeometry(Point2(0.5, 0.0), 1.0, 2, 2),
            CellState::Free);
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(OccupancyGridFusion::occupiedUnion(
                    reference, differentOrigin));
            },
            "occupied union should reject different grid origins");

        const OccupancyGrid differentResolution(
            GridGeometry(Point2(0.0, 0.0), 0.5, 2, 2),
            CellState::Free);
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(OccupancyGridFusion::occupiedUnion(
                    reference, differentResolution));
            },
            "occupied union should reject different grid resolutions");

        const OccupancyGrid master(
            GridGeometry(Point2(-1.0, -1.0), 1.0, 4, 4),
            CellState::Free);
        const OccupancyGrid offsetGrid = master.subgrid(
            { { 1, 1 }, 2, 2 }, SubgridStorage::IndependentCopy);
        require(pointsNear(
                offsetGrid.geometry().worldOrigin(), reference.geometry().worldOrigin()),
            "offset mismatch fixture should retain matching local geometry");
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(OccupancyGridFusion::occupiedUnion(
                    reference, offsetGrid));
            },
            "occupied union should reject different master-cell offsets");
    }

    void rasterizationProducesBinaryOpenCvImage()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 0.5, 10, 10);
        const Polygon operationArea{
            Point2(0.5, 2.5), Point2(2.5, 0.5),
            Point2(4.5, 2.5), Point2(2.5, 4.5)
        };
        const Polygon obstacle{
            Point2(1.75, 1.75), Point2(3.25, 1.75),
            Point2(3.25, 3.25), Point2(1.75, 3.25)
        };
        const OccupancyGrid grid = PolygonRasterizer::rasterize(
            geometry, operationArea, { obstacle });

        cv::Mat1b nonBinaryPixels;
        cv::inRange(grid.imageView(), cv::Scalar(1), cv::Scalar(254), nonBinaryPixels);
        require(cv::countNonZero(nonBinaryPixels) == 0,
            "OpenCV polygon painting should retain binary occupancy values");
    }

    void operationAreaCanBuildAlignedMasterGrid()
    {
        const Polygon operationArea{
            Point2(0.2, 0.2), Point2(3.7, 0.2),
            Point2(3.7, 2.6), Point2(0.2, 2.6)
        };
        const OccupancyGrid master = PolygonRasterizer::rasterize(
            operationArea, {}, 1.0);

        require(pointsNear(master.geometry().worldOrigin(), Point2(0.0, 0.0)),
            "master grid should align the operation area to the world lattice");
        require(master.width() == 4 && master.height() == 3,
            "master grid should cover the complete operation-area bounding box");
        require(master.isTraversable({ 0, 0 }),
            "OpenCV coverage should free operation-area boundary cells");
        require(master.isTraversable({ 1, 1 }),
            "operation-area interior cells should be free");
    }

    void validatedEnvironmentSelectsRasterizationMode()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const Polygon containedObstacle{
            Point2(1.1, 1.1), Point2(1.9, 1.1),
            Point2(1.9, 1.9), Point2(1.1, 1.9)
        };
        const PolygonEnvironment unconstrainedEnvironment({ containedObstacle });
        const OccupancyGrid unconstrainedGrid = PolygonRasterizer::rasterize(
            geometry, unconstrainedEnvironment);

        require(unconstrainedGrid.isTraversable({ 0, 0 }) &&
                unconstrainedGrid.at({ 1, 1 }) == CellState::Occupied,
            "an environment without an operation area should start free and paint obstacles occupied");

        const Polygon operationArea{
            Point2(0.0, 0.0), Point2(4.0, 0.0),
            Point2(4.0, 4.0), Point2(0.0, 4.0)
        };
        const Polygon crossingObstacle{
            Point2(3.0, 1.0), Point2(5.0, 1.0),
            Point2(5.0, 3.0), Point2(3.0, 3.0)
        };
        const PolygonEnvironment constrainedEnvironment(
            operationArea, { crossingObstacle });

        const OccupancyGrid constrainedGrid = PolygonRasterizer::rasterize(
            geometry, constrainedEnvironment);

        require(pointsNear(
                    constrainedGrid.geometry().worldOrigin(),
                    geometry.worldOrigin()) &&
                pointsNear(
                    constrainedGrid.geometry().worldMaximum(),
                    geometry.worldMaximum()),
            "environment rasterization should preserve caller-selected geometry");
        require(constrainedGrid.isTraversable({ 0, 0 }),
            "free operation-area cells should remain traversable");
        require(constrainedGrid.at({ 3, 1 }) == CellState::Occupied,
            "clipped effective obstacles should be present in the constrained grid");
    }

    void subgridPreservesWorldAndMasterCoordinates()
    {
        const GridGeometry geometry(Point2(-2.0, 3.0), 0.5, 6, 5);
        const OccupancyGrid master(geometry, CellState::Free);
        const GridRegion region{ { 1, 1 }, 3, 2 };
        const OccupancyGrid window = master.subgrid(region);

        require(window.width() == 3 && window.height() == 2,
            "subgrid dimensions should match the selected master region");
        require(pointsNear(window.geometry().worldOrigin(), Point2(-1.5, 3.5)),
            "subgrid origin should match its lower-left master cell");
        require(window.masterCellOffset() == GridCell{ 1, 1 },
            "subgrid should retain its master-cell offset");
        require(window.localToMaster({ 2, 1 }) == GridCell{ 3, 2 },
            "local cells should convert to master cells");
        require(window.masterToLocal({ 3, 2 }) == GridCell{ 2, 1 },
            "master cells should convert back to local cells");
        require(!window.masterToLocal({ 0, 0 }),
            "master cell outside the window should not have a local cell");
        require(pointsNear(
                window.geometry().cellCenterToWorld({ 0, 0 }),
                master.geometry().cellCenterToWorld({ 1, 1 })),
            "local and master cells should represent the same world position");

        const int masterImageRow = master.height() - (region.lowerLeft.row + region.height);
        require(window.imageView().data ==
                master.imageView().ptr(masterImageRow) + region.lowerLeft.column,
            "default subgrid should share the selected OpenCV ROI storage");
        require(!window.imageView().isContinuous(),
            "multi-row shared ROI narrower than its source should be non-contiguous");
    }

    void copiedSubgridOwnsIndependentPixels()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const OccupancyGrid master(geometry, CellState::Occupied);
        const GridRegion region{ { 1, 1 }, 2, 2 };
        const OccupancyGrid copy = master.subgrid(
            region, SubgridStorage::IndependentCopy);

        require(copy.imageView().data != master.imageView().data,
            "copied subgrid should own independent OpenCV storage");
        require(cv::countNonZero(copy.imageView()) == 4,
            "copied subgrid should preserve occupancy values");

        cv::Mat1b writableClone = master.cloneImage();
        writableClone.setTo(static_cast<std::uint8_t>(CellState::Free));
        require(cv::countNonZero(master.imageView()) == 16,
            "modifying cloneImage output should not affect the source grid");

        requireThrows<std::invalid_argument>(
            [&master, &region]
            {
                static_cast<void>(master.subgrid(
                    region, static_cast<SubgridStorage>(99)));
            },
            "subgrid should reject an unsupported storage policy");
    }

    void sharedSubgridRetainsStorageLifetime()
    {
        const OccupancyGrid window = []
        {
            const OccupancyGrid master = gridWithOccupiedCells(
                4, 4, { { 1, 1 }, { 2, 2 } });
            return master.subgrid({ { 1, 1 }, 2, 2 });
        }();

        require(window.at({ 0, 0 }) == CellState::Occupied &&
                window.at({ 1, 1 }) == CellState::Occupied,
            "shared subgrid should retain referenced pixels after source destruction");
        require(window.isTraversable({ 1, 0 }) && window.isTraversable({ 0, 1 }),
            "shared subgrid should retain free pixels after source destruction");
    }

    void worldBoundsProduceOutwardRoundedSubgrid()
    {
        const OccupancyGrid master(
            GridGeometry(Point2(0.0, 0.0), 1.0, 6, 5),
            CellState::Free);
        const WorldBounds requested{ Point2(1.2, 0.4), Point2(4.0, 3.1) };

        const GridRegion region = master.regionCovering(requested);
        require(region == GridRegion{ { 1, 0 }, 3, 4 },
            "world bounds should round outward to complete master cells");

        const OccupancyGrid window = master.subgrid(requested);
        require(pointsNear(window.geometry().worldOrigin(), Point2(1.0, 0.0)),
            "world-bounds subgrid should use the rounded lower-left cell origin");
        require(window.width() == 3 && window.height() == 4,
            "world-bounds subgrid should use the rounded cell dimensions");

        requireThrows<std::out_of_range>(
            [&master]
            {
                static_cast<void>(master.subgrid(
                    WorldBounds{ Point2(-0.1, 0.0), Point2(2.0, 2.0) }));
            },
            "subgrid bounds outside the master should be rejected");
    }

    void worldBoundsContainmentToleratesRoundoff()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.1, 0.2), 0.1, 3, 4),
            CellState::Free);
        const Point2 maximum = grid.geometry().worldMaximum();
        const WorldBounds roundedBounds{
            grid.geometry().worldOrigin(),
            Point2(
                std::nextafter(maximum.x(), std::numeric_limits<double>::infinity()),
                std::nextafter(maximum.y(), std::numeric_limits<double>::infinity()))
        };

        require(grid.geometry().contains(roundedBounds),
            "grid bounds should tolerate one representable step of roundoff");
        require(grid.regionCovering(roundedBounds) == grid.fullRegion(),
            "roundoff at the maximum boundary should still cover the full grid");

        const WorldBounds outsideBounds{
            grid.geometry().worldOrigin(),
            Point2(maximum.x() + 1e-6, maximum.y())
        };
        require(!grid.geometry().contains(outsideBounds),
            "containment tolerance should not accept materially outside bounds");
    }

    void nestedSubgridsRetainMasterOffset()
    {
        const OccupancyGrid master(
            GridGeometry(Point2(0.0, 0.0), 1.0, 8, 8),
            CellState::Free);
        const OccupancyGrid first = master.subgrid({ { 2, 1 }, 5, 6 });
        const OccupancyGrid nested = first.subgrid({ { 1, 2 }, 2, 3 });

        require(nested.masterCellOffset() == GridCell{ 3, 3 },
            "nested subgrid offsets should remain relative to the original master");
        require(nested.localToMaster({ 1, 2 }) == GridCell{ 4, 5 },
            "nested local-to-master conversion should include every window offset");
    }

    void aStarFindsShortestPathAcrossEmptyGrid()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 6, 4),
            CellState::Free);
        const AStarGridPlanner planner;
        AStarOptions options;
        options.connectivity = GridConnectivity::FourConnected;

        const auto result = planner.plan(grid, { 0, 0 }, { 5, 2 }, options);
        require(result.succeeded(),
            "A* should find a path between free cells in an empty grid");
        requireValidFourConnectedPath(grid, result.path, { 0, 0 }, { 5, 2 });
        require(result.path.size() == 8,
            "empty-grid path should have Manhattan distance plus one cells");
        require(result.simplifiedPath ==
                std::vector<GridCell>{ { 0, 0 }, { 5, 2 } },
            "an unobstructed path should simplify to its terminal cells");
        require(std::abs(result.diagnostics.pathCost - 7.0) <= TEST_EPSILON,
            "diagnostics should report the successful path's movement cost");
        require(result.diagnostics.expandedNodes > 0 &&
                result.diagnostics.generatedNodes >= result.diagnostics.expandedNodes &&
                result.diagnostics.peakOpenSetSize > 0 &&
                result.diagnostics.peakOpenSetSize <= result.diagnostics.generatedNodes,
            "successful search diagnostics should report consistent work counts");
    }

    void aStarHandlesCoincidentTerminals()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(-2.0, 3.0), 0.5, 3, 3),
            CellState::Free);
        const AStarGridPlanner planner;

        const auto result = planner.plan(grid, { 1, 2 }, { 1, 2 });
        require(result.status == GridPlanStatus::Success,
            "coincident terminals inside the grid should succeed");
        require(result.path == std::vector<GridCell>{ { 1, 2 } },
            "coincident terminals should return a one-cell path");
        require(result.simplifiedPath == result.path,
            "a one-cell path should remain unchanged after simplification");
        require(result.diagnostics.pathCost == 0.0 &&
                result.diagnostics.expandedNodes == 1 &&
                result.diagnostics.generatedNodes == 1 &&
                result.diagnostics.peakOpenSetSize == 1,
            "coincident terminals should report the single processed start node");
    }

    void aStarValidatesTerminals()
    {
        const OccupancyGrid occupiedGrid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 4, 4),
            CellState::Occupied);
        const AStarGridPlanner planner;

        const auto occupiedStart = planner.plan(occupiedGrid, { 0, 0 }, { 3, 3 });
        require(occupiedStart.status == GridPlanStatus::StartOccupied &&
                occupiedStart.path.empty(),
            "planner should reject an occupied start cell");

        const OccupancyGrid occupiedGoal = gridWithOccupiedCells(4, 4, { { 3, 3 } });
        const auto goalResult = planner.plan(occupiedGoal, { 0, 0 }, { 3, 3 });
        require(goalResult.status == GridPlanStatus::GoalOccupied &&
                goalResult.path.empty(),
            "planner should reject an occupied goal cell");

        const auto invalidStart = planner.plan(occupiedGrid, { -1, 0 }, { 3, 3 });
        require(invalidStart.status == GridPlanStatus::StartOutsideGrid &&
                invalidStart.path.empty(),
            "planner should reject a start cell outside the planning grid");

        const auto invalidGoal = planner.plan(occupiedGrid, { 0, 0 }, { 4, 3 });
        require(invalidGoal.status == GridPlanStatus::GoalOutsideGrid &&
                invalidGoal.path.empty(),
            "planner should reject a goal cell outside the planning grid");
    }

    void gridPlanResultHasSafeDefaultState()
    {
        const GridPlanResult result;
        require(result.status == GridPlanStatus::NoPath && result.path.empty() &&
                result.simplifiedPath.empty() &&
                std::isinf(result.diagnostics.pathCost) &&
                result.diagnostics.expandedNodes == 0 &&
                result.diagnostics.generatedNodes == 0 &&
                result.diagnostics.peakOpenSetSize == 0,
            "default planning result should be a deterministic unsuccessful result");
    }

    void gridPathSimplificationUsesConservativeLineOfSight()
    {
        const OccupancyGrid freeGrid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 6, 5),
            CellState::Free);
        const std::vector<GridCell> stairStepPath{
            { 0, 0 }, { 1, 0 }, { 1, 1 }, { 2, 1 },
            { 3, 1 }, { 3, 2 }, { 4, 2 }, { 5, 3 }
        };
        require(simplifyGridPath(freeGrid, stairStepPath) ==
                std::vector<GridCell>{ { 0, 0 }, { 5, 3 } },
            "an unobstructed stair-step path should simplify to one segment");

        const OccupancyGrid obstacleGrid = gridWithOccupiedCells(
            5, 4, { { 2, 1 } });
        const std::vector<GridCell> obstacleDetour{
            { 0, 1 }, { 0, 2 }, { 1, 2 }, { 2, 2 },
            { 3, 2 }, { 4, 2 }, { 4, 1 }
        };
        const std::vector<GridCell> simplifiedDetour =
            simplifyGridPath(obstacleGrid, obstacleDetour);
        require(simplifiedDetour.front() == obstacleDetour.front() &&
                simplifiedDetour.back() == obstacleDetour.back() &&
                simplifiedDetour.size() > 2 &&
                simplifiedDetour.size() < obstacleDetour.size(),
            "simplification should retain only the waypoints needed to avoid an obstacle");
        for (std::size_t index = 1; index < simplifiedDetour.size(); ++index)
        {
            const std::vector<GridCell> segment{
                simplifiedDetour[index - 1], simplifiedDetour[index]
            };
            require(simplifyGridPath(obstacleGrid, segment) == segment,
                "every simplified segment should remain collision-free");
        }

        const OccupancyGrid cornerGrid = gridWithOccupiedCells(
            3, 3, { { 1, 0 } });
        const std::vector<GridCell> cornerDetour{
            { 0, 0 }, { 0, 1 }, { 1, 1 }
        };
        require(simplifyGridPath(cornerGrid, cornerDetour) == cornerDetour,
            "simplification should not pass diagonally beside an occupied corner");
        require(simplifyGridPath(cornerGrid, cornerDetour, false) ==
                std::vector<GridCell>{ { 0, 0 }, { 1, 1 } },
            "simplification should honor an explicit corner-cutting policy");

        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(simplifyGridPath(
                    cornerGrid, std::vector<GridCell>{ { 0, 0 }, { 1, 1 } }));
            },
            "simplification should reject an input segment that crosses an occupied corner");
    }

    void gridPlannerRejectsUnsupportedOptions()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 2, 2),
            CellState::Free);
        const AStarGridPlanner planner;
        AStarOptions options;
        options.connectivity = static_cast<GridConnectivity>(99);

        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(planner.plan(grid, { 0, 0 }, { 1, 1 }, options)); },
            "planner should reject an unsupported connectivity value");

        options = AStarOptions{};
        options.algorithm = static_cast<GridSearchAlgorithm>(99);
        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(planner.plan(grid, { 0, 0 }, { 1, 1 }, options)); },
            "planner should reject an unsupported search algorithm");

        options = AStarOptions{};
        options.tieBreakPolicy = static_cast<AStarTieBreakPolicy>(99);
        requireThrows<std::invalid_argument>(
            [&] { static_cast<void>(planner.plan(grid, { 0, 0 }, { 1, 1 }, options)); },
            "planner should reject an unsupported tie-break policy");

        options.algorithm = GridSearchAlgorithm::WeightedAStar;
        for (const double invalidWeight : {
                 -1.0,
                 0.0,
                 0.99,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::quiet_NaN() })
        {
            options.heuristicWeight = invalidWeight;
            requireThrows<std::invalid_argument>(
                [&] { static_cast<void>(planner.plan(grid, { 0, 0 }, { 1, 1 }, options)); },
                "weighted A* should reject a non-finite or sub-unit heuristic weight");
        }
    }

    void dijkstraMatchesAStarOptimalCost()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 25, 25),
            CellState::Free);
        const AStarGridPlanner planner;

        AStarOptions aStarOptions;
        aStarOptions.algorithm = GridSearchAlgorithm::AStar;
        const auto aStarResult = planner.plan(
            grid, { 0, 0 }, { 24, 24 }, aStarOptions);

        AStarOptions dijkstraOptions;
        dijkstraOptions.algorithm = GridSearchAlgorithm::Dijkstra;
        const auto dijkstraResult = planner.plan(
            grid, { 0, 0 }, { 24, 24 }, dijkstraOptions);

        require(aStarResult.succeeded() && dijkstraResult.succeeded(),
            "A* and Dijkstra should both find a path across an empty grid");
        requireValidEightConnectedPath(
            grid, dijkstraResult.path, { 0, 0 }, { 24, 24 });
        require(std::abs(
                aStarResult.diagnostics.pathCost -
                dijkstraResult.diagnostics.pathCost) <= TEST_EPSILON,
            "Dijkstra and A* should report the same optimal path cost");
        require(dijkstraResult.diagnostics.expandedNodes >
                aStarResult.diagnostics.expandedNodes,
            "Dijkstra should expand more nodes than A* on an open diagonal search");
        require(std::string(astar::gridSearchAlgorithmName(
                    GridSearchAlgorithm::Dijkstra)) == "Dijkstra",
            "search algorithm name should identify Dijkstra mode");
    }

    void randomizedAStarMatchesDijkstra()
    {
        struct SearchConfiguration
        {
            GridConnectivity connectivity;
            bool preventDiagonalCornerCutting;
            const char* name;
        };

        const std::vector<SearchConfiguration> configurations{
            { GridConnectivity::FourConnected, true, "four-connected" },
            { GridConnectivity::EightConnected, true,
                "eight-connected protected" },
            { GridConnectivity::EightConnected, false,
                "eight-connected permissive" }
        };
        constexpr std::array<AStarTieBreakPolicy, 3> tieBreakPolicies{
            AStarTieBreakPolicy::StraightLineThenLargerG,
            AStarTieBreakPolicy::LargerGThenStraightLine,
            AStarTieBreakPolicy::LargerGOnly
        };

        constexpr int randomMapCount = 200;
        std::mt19937 generator(0xA57A2026u);
        std::uniform_int_distribution<int> dimensionDistribution(8, 30);
        std::size_t successfulComparisons = 0;
        std::size_t noPathComparisons = 0;

        for (int mapIndex = 0; mapIndex < randomMapCount; ++mapIndex)
        {
            const int width = dimensionDistribution(generator);
            const int height = dimensionDistribution(generator);
            std::uniform_int_distribution<int> columnDistribution(0, width - 1);
            std::uniform_int_distribution<int> rowDistribution(0, height - 1);
            const GridCell start{
                columnDistribution(generator), rowDistribution(generator)
            };
            GridCell goal{
                columnDistribution(generator), rowDistribution(generator)
            };
            while (goal == start)
            {
                goal = {
                    columnDistribution(generator), rowDistribution(generator)
                };
            }

            constexpr double obstacleDensities[]{ 0.05, 0.15, 0.25, 0.35 };
            const double obstacleDensity = obstacleDensities[
                mapIndex % static_cast<int>(std::size(obstacleDensities))];
            std::bernoulli_distribution occupiedDistribution(obstacleDensity);
            std::vector<GridCell> occupiedCells;
            for (int row = 0; row < height; ++row)
            {
                for (int column = 0; column < width; ++column)
                {
                    const GridCell cell{ column, row };
                    if (cell != start && cell != goal &&
                        occupiedDistribution(generator))
                    {
                        occupiedCells.push_back(cell);
                    }
                }
            }
            const OccupancyGrid grid = gridWithOccupiedCells(
                width, height, occupiedCells);
            const AStarGridPlanner planner;

            for (const SearchConfiguration& configuration : configurations)
            {
                AStarOptions dijkstraOptions;
                dijkstraOptions.algorithm = GridSearchAlgorithm::Dijkstra;
                dijkstraOptions.connectivity = configuration.connectivity;
                dijkstraOptions.preventDiagonalCornerCutting =
                    configuration.preventDiagonalCornerCutting;
                const GridPlanResult dijkstraResult = planner.plan(
                    grid, start, goal, dijkstraOptions);

                for (AStarTieBreakPolicy tieBreakPolicy : tieBreakPolicies)
                {
                    AStarOptions aStarOptions = dijkstraOptions;
                    aStarOptions.algorithm = GridSearchAlgorithm::AStar;
                    aStarOptions.tieBreakPolicy = tieBreakPolicy;
                    const GridPlanResult aStarResult = planner.plan(
                        grid, start, goal, aStarOptions);

                    std::ostringstream context;
                    context << "random map " << mapIndex << " ("
                            << width << 'x' << height << ", "
                            << configuration.name << ", "
                            << aStarTieBreakPolicyName(tieBreakPolicy) << ")";
                    require(
                        aStarResult.succeeded() == dijkstraResult.succeeded(),
                        "A* and Dijkstra should agree on reachability for " +
                            context.str());

                    if (!aStarResult.succeeded())
                    {
                        ++noPathComparisons;
                        require(
                            aStarResult.status == GridPlanStatus::NoPath &&
                            dijkstraResult.status == GridPlanStatus::NoPath &&
                            aStarResult.path.empty() &&
                            dijkstraResult.path.empty() &&
                            std::isinf(aStarResult.diagnostics.pathCost) &&
                            std::isinf(dijkstraResult.diagnostics.pathCost),
                            "unreachable searches should return consistent failures for " +
                                context.str());
                        continue;
                    }

                    ++successfulComparisons;
                    if (configuration.connectivity ==
                        GridConnectivity::FourConnected)
                    {
                        requireValidFourConnectedPath(
                            grid, aStarResult.path, start, goal);
                        requireValidFourConnectedPath(
                            grid, dijkstraResult.path, start, goal);
                    }
                    else
                    {
                        requireValidEightConnectedPath(
                            grid,
                            aStarResult.path,
                            start,
                            goal,
                            configuration.preventDiagonalCornerCutting);
                        requireValidEightConnectedPath(
                            grid,
                            dijkstraResult.path,
                            start,
                            goal,
                            configuration.preventDiagonalCornerCutting);
                    }

                    const double measuredAStarCost =
                        pathMovementCost(aStarResult.path);
                    const double measuredDijkstraCost =
                        pathMovementCost(dijkstraResult.path);
                    require(
                        costsNear(
                            aStarResult.diagnostics.pathCost,
                            measuredAStarCost) &&
                        costsNear(
                            dijkstraResult.diagnostics.pathCost,
                            measuredDijkstraCost),
                        "reported costs should match reconstructed paths for " +
                            context.str());
                    require(
                        costsNear(
                            aStarResult.diagnostics.pathCost,
                            dijkstraResult.diagnostics.pathCost),
                        "A* and Dijkstra should return the same optimal cost for " +
                            context.str());
                }
            }
        }

        require(successfulComparisons > 0 && noPathComparisons > 0,
            "randomized comparison should cover reachable and unreachable requests");
        require(
            successfulComparisons + noPathComparisons ==
                static_cast<std::size_t>(randomMapCount) * configurations.size() *
                    tieBreakPolicies.size(),
            "randomized comparison should execute every configured request");
    }

    void weightedAStarUsesSharedWeightedHeuristic()
    {
        std::vector<GridCell> wall;
        for (int row = 0; row < 18; ++row)
            wall.push_back({ 20, row });
        const OccupancyGrid grid = gridWithOccupiedCells(40, 20, wall);
        const AStarGridPlanner planner;
        const GridCell start{ 2, 10 };
        const GridCell goal{ 37, 10 };

        AStarOptions aStarOptions;
        const auto aStarResult = planner.plan(grid, start, goal, aStarOptions);

        AStarOptions unitWeightOptions;
        unitWeightOptions.algorithm = GridSearchAlgorithm::WeightedAStar;
        unitWeightOptions.heuristicWeight = 1.0;
        const auto unitWeightResult = planner.plan(
            grid, start, goal, unitWeightOptions);

        require(aStarResult.succeeded() && unitWeightResult.succeeded(),
            "A* and unit-weight A* should both find the wall detour");
        require(aStarResult.path == unitWeightResult.path &&
                aStarResult.diagnostics.pathCost ==
                    unitWeightResult.diagnostics.pathCost &&
                aStarResult.diagnostics.expandedNodes ==
                    unitWeightResult.diagnostics.expandedNodes,
            "weighted A* with weight one should behave exactly like A*");

        AStarOptions weightedOptions;
        weightedOptions.algorithm = GridSearchAlgorithm::WeightedAStar;
        weightedOptions.heuristicWeight = 2.0;
        const auto weightedResult = planner.plan(
            grid, start, goal, weightedOptions);

        require(weightedResult.succeeded(),
            "weighted A* should find the wall detour");
        requireValidEightConnectedPath(grid, weightedResult.path, start, goal);
        require(weightedResult.diagnostics.pathCost + TEST_EPSILON >=
                aStarResult.diagnostics.pathCost,
            "weighted A* should not report a path cheaper than optimal A*");
        require(weightedResult.diagnostics.expandedNodes <
                aStarResult.diagnostics.expandedNodes,
            "weighted A* should expand fewer nodes on the wall-detour map");

        double measuredPathCost = 0.0;
        for (std::size_t index = 1; index < weightedResult.path.size(); ++index)
        {
            const GridCell& previous = weightedResult.path[index - 1];
            const GridCell& current = weightedResult.path[index];
            const bool diagonal = previous.column != current.column &&
                previous.row != current.row;
            measuredPathCost += diagonal ? std::sqrt(2.0) : 1.0;
        }
        require(std::abs(
                weightedResult.diagnostics.pathCost - measuredPathCost) <=
                TEST_EPSILON,
            "weighted A* should report movement cost rather than weighted priority");
        require(std::string(astar::gridSearchAlgorithmName(
                    GridSearchAlgorithm::WeightedAStar)) == "Weighted A*",
            "search algorithm name should identify weighted A* mode");
    }

    void searchDebugCallbackPublishesReusableStateGrid()
    {
        const OccupancyGrid grid = gridWithOccupiedCells(5, 3, { { 2, 0 } });
        const AStarGridPlanner planner;
        const GridCell start{ 0, 1 };
        const GridCell goal{ 4, 1 };
        const GridPlanResult normalResult = planner.plan(grid, start, goal);

        std::size_t callbackCount = 0;
        bool observedCurrent = false;
        bool observedOpen = false;
        cv::Mat1b finalFrame;
        const astar::GridSearchDebugCallback callback =
            [&](const cv::Mat1b& state)
            {
                ++callbackCount;
                require(state.rows == grid.height() && state.cols == grid.width(),
                    "debug-state image dimensions should match the planning grid");

                cv::Mat1b currentCells;
                cv::compare(
                    state,
                    static_cast<std::uint8_t>(GridSearchCellState::Current),
                    currentCells,
                    cv::CMP_EQ);
                observedCurrent = observedCurrent ||
                    cv::countNonZero(currentCells) > 0;

                cv::Mat1b openCells;
                cv::compare(
                    state,
                    static_cast<std::uint8_t>(GridSearchCellState::Open),
                    openCells,
                    cv::CMP_EQ);
                observedOpen = observedOpen || cv::countNonZero(openCells) > 0;
                finalFrame = state.clone();
            };

        const GridPlanResult debugResult = planner.plan(
            grid, start, goal, AStarOptions{}, callback);

        require(debugResult.status == normalResult.status &&
                debugResult.path == normalResult.path &&
                debugResult.diagnostics.pathCost ==
                    normalResult.diagnostics.pathCost,
            "debug callbacks should not change the planning result");
        require(callbackCount == debugResult.diagnostics.expandedNodes + 1,
            "successful debug planning should publish every expansion and final path");
        require(observedCurrent && observedOpen,
            "debug-state callbacks should expose current and open cells");
        require(finalFrame(
                    grid.geometry().cellToImage({ 2, 0 })) ==
                static_cast<std::uint8_t>(GridSearchCellState::Occupied),
            "debug-state grid should preserve occupied cells");
        for (const GridCell& pathCell : debugResult.path)
        {
            const cv::Point pixel = grid.geometry().cellToImage(pathCell);
            require(finalFrame(pixel.y, pixel.x) ==
                    static_cast<std::uint8_t>(GridSearchCellState::Path),
                "final debug-state frame should mark the returned path");
        }

        callbackCount = 0;
        const auto invalidResult = planner.plan(
            grid, { -1, 0 }, goal, AStarOptions{}, callback);
        require(invalidResult.status == GridPlanStatus::StartOutsideGrid &&
                callbackCount == 0,
            "terminal validation failures should not allocate or publish debug state");
    }

    void detailedSearchDiagnosticsTrackRepeatedExpansions()
    {
        std::vector<GridCell> barriers;
        constexpr int gridSize = 50;
        constexpr int barrierCount = 6;
        for (int barrier = 0; barrier < barrierCount; ++barrier)
        {
            const int column =
                (barrier + 1) * gridSize / (barrierCount + 1);
            const bool gapAtTop = barrier % 2 == 0;
            const int firstRow = gapAtTop ? 0 : gridSize / 4;
            const int pastLastRow = gapAtTop
                ? 3 * gridSize / 4
                : gridSize;
            for (int row = firstRow; row < pastLastRow; ++row)
                barriers.push_back({ column, row });
        }

        const OccupancyGrid grid = gridWithOccupiedCells(
            gridSize, gridSize, barriers);
        const GridCell start{ 2, 2 };
        const GridCell goal{ gridSize - 3, gridSize - 3 };
        const AStarGridPlanner planner;

        AStarOptions detailedAStarOptions;
        detailedAStarOptions.collectDetailedDiagnostics = true;
        const GridPlanResult detailedAStarResult = planner.plan(
            grid, start, goal, detailedAStarOptions);
        require(detailedAStarResult.succeeded() &&
                detailedAStarResult.detailedDiagnostics.has_value(),
            "A* should provide requested detailed diagnostics");
        require(
            detailedAStarResult.detailedDiagnostics->repeatedExpansions == 0 &&
            detailedAStarResult.detailedDiagnostics->
                postExpansionCostImprovements == 0 &&
            detailedAStarResult.detailedDiagnostics->
                maximumExpansionsPerCell == 1,
            "A* should ignore roundoff-only cost changes on the barrier map");

        AStarOptions normalOptions;
        normalOptions.algorithm = GridSearchAlgorithm::WeightedAStar;
        normalOptions.heuristicWeight = 1.5;
        const GridPlanResult normalResult = planner.plan(
            grid, start, goal, normalOptions);
        require(normalResult.succeeded() && !normalResult.detailedDiagnostics,
            "normal planning should not allocate detailed diagnostics");

        AStarOptions detailedOptions = normalOptions;
        detailedOptions.collectDetailedDiagnostics = true;
        const GridPlanResult detailedResult = planner.plan(
            grid, start, goal, detailedOptions);
        require(detailedResult.succeeded() &&
                detailedResult.path == normalResult.path &&
                detailedResult.diagnostics.pathCost ==
                    normalResult.diagnostics.pathCost,
            "detailed diagnostics should not change the planning result");
        require(detailedResult.detailedDiagnostics.has_value(),
            "requested detailed diagnostics should be present");

        const auto& detailed = *detailedResult.detailedDiagnostics;
        require(detailed.staleOpenSetEntries == 0,
            "the indexed open set should not produce stale queue entries");
        require(detailed.expansionCounts.rows == grid.height() &&
                detailed.expansionCounts.cols == grid.width(),
            "expansion-count image dimensions should match the planning grid");

        const std::size_t matrixExpansionCount = static_cast<std::size_t>(
            cv::sum(detailed.expansionCounts)[0]);
        const std::size_t matrixUniqueCount = static_cast<std::size_t>(
            cv::countNonZero(detailed.expansionCounts));
        double matrixMaximum = 0.0;
        cv::minMaxLoc(
            detailed.expansionCounts, nullptr, &matrixMaximum);
        require(matrixExpansionCount ==
                    detailedResult.diagnostics.expandedNodes &&
                matrixUniqueCount == detailed.uniqueExpandedCells &&
                detailed.uniqueExpandedCells + detailed.repeatedExpansions ==
                    detailedResult.diagnostics.expandedNodes &&
                static_cast<std::size_t>(matrixMaximum) ==
                    detailed.maximumExpansionsPerCell,
            "detailed counters should agree with the expansion-count image");
        require(detailed.repeatedExpansions > 0 &&
                detailed.maximumExpansionsPerCell > 1,
            "weighted alternating-barrier search should expose repeated expansions");
        require(detailed.postExpansionCostImprovements > 0 &&
                std::isfinite(detailed.minimumPostExpansionCostImprovement) &&
                detailed.minimumPostExpansionCostImprovement > 0.0 &&
                detailed.maximumPostExpansionCostImprovement >=
                    detailed.minimumPostExpansionCostImprovement &&
                detailed.maximumRelativePostExpansionCostImprovement > 0.0,
            "detailed diagnostics should measure improvements to expanded cells");
    }

    void aStarRoutesAroundObstacle()
    {
        const OccupancyGrid grid = gridWithOccupiedCells(
            5, 5, { { 2, 0 }, { 2, 1 }, { 2, 2 }, { 2, 3 } });
        const AStarGridPlanner planner;
        AStarOptions options;
        options.connectivity = GridConnectivity::FourConnected;

        const auto result = planner.plan(grid, { 0, 2 }, { 4, 2 }, options);
        require(result.succeeded(), "A* should route through the wall opening");
        requireValidFourConnectedPath(grid, result.path, { 0, 2 }, { 4, 2 });
        require(result.path.size() == 9,
            "detour should be the shortest route through the wall opening");
    }

    void aStarReconstructsCompleteParentChain()
    {
        constexpr int corridorLength = 64;
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, corridorLength, 1),
            CellState::Free);
        const AStarGridPlanner planner;

        const auto result = planner.plan(
            grid, { 0, 0 }, { corridorLength - 1, 0 });

        require(result.succeeded(),
            "A* should reconstruct a path through a long corridor");
        require(result.path.size() == static_cast<std::size_t>(corridorLength),
            "reconstructed path should contain every cell in the parent chain");
        for (int column = 0; column < corridorLength; ++column)
        {
            require(result.path[static_cast<std::size_t>(column)] ==
                    GridCell{ column, 0 },
                "reconstructed parent chain should remain ordered from start to goal");
        }
    }

    void aStarUsesEightConnectedDiagonalPathByDefault()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 11, 11),
            CellState::Free);
        const AStarGridPlanner planner;

        const auto result = planner.plan(grid, { 0, 0 }, { 10, 10 });
        require(result.succeeded(), "A* should cross an empty square grid");
        requireValidEightConnectedPath(grid, result.path, { 0, 0 }, { 10, 10 });
        require(result.path.size() == 11,
            "eight-connected planning should use the direct diagonal path");
        for (const GridCell& cell : result.path)
        {
            require(cell.column == cell.row,
                "empty-grid diagonal path should stay on the direct line");
        }
    }

    void aStarControlsDiagonalCornerCutting()
    {
        const OccupancyGrid grid = gridWithOccupiedCells(
            2, 2, { { 1, 0 }, { 0, 1 } });
        const AStarGridPlanner planner;

        const auto protectedResult = planner.plan(grid, { 0, 0 }, { 1, 1 });
        require(protectedResult.status == GridPlanStatus::NoPath,
            "default eight-connected planning should not cut occupied corners");

        AStarOptions options;
        options.preventDiagonalCornerCutting = false;
        const auto permissiveResult = planner.plan(
            grid, { 0, 0 }, { 1, 1 }, options);
        require(permissiveResult.succeeded() &&
                permissiveResult.path ==
                    std::vector<GridCell>{ { 0, 0 }, { 1, 1 } },
            "corner cutting should be available as an explicit option");
    }

    void aStarReportsWhenNoPathExists()
    {
        const OccupancyGrid grid = gridWithOccupiedCells(
            5, 5, { { 2, 0 }, { 2, 1 }, { 2, 2 }, { 2, 3 }, { 2, 4 } });
        const AStarGridPlanner planner;

        const auto result = planner.plan(grid, { 0, 2 }, { 4, 2 });
        require(result.status == GridPlanStatus::NoPath && result.path.empty(),
            "A* should report no path when occupied cells divide the grid");
        require(std::isinf(result.diagnostics.pathCost) &&
                result.diagnostics.expandedNodes > 0 &&
                result.diagnostics.generatedNodes >= result.diagnostics.expandedNodes &&
                result.diagnostics.peakOpenSetSize > 0,
            "failed searches should retain work counts but have no finite path cost");
    }

    void aStarPlanningPipelineUsesRoiLocalCoordinates()
    {
        const Polygon operationArea{
            Point2(0.0, 0.0), Point2(10.0, 0.0),
            Point2(10.0, 10.0), Point2(0.0, 10.0)
        };
        const Polygon obstacle{
            Point2(4.0, 4.0), Point2(5.0, 4.0),
            Point2(5.0, 5.0), Point2(4.0, 5.0)
        };
        const PolygonEnvironment environment(operationArea, { obstacle });
        const OccupancyGrid master = PolygonRasterizer::rasterize(
            environment.operationArea(), environment.effectiveObstacles(), 1.0);
        const OccupancyGrid planningGrid = master.subgrid(
            WorldBounds{ Point2(2.0, 1.0), Point2(8.0, 9.0) });

        const Point2 worldStart(2.5, 1.5);
        const Point2 worldGoal(7.5, 8.5);
        const auto start = planningGrid.geometry().worldToCell(worldStart);
        const auto goal = planningGrid.geometry().worldToCell(worldGoal);
        require(start == GridCell{ 0, 0 } && goal == GridCell{ 5, 7 },
            "world terminals should convert to planning-ROI-local cells");

        const AStarGridPlanner planner;
        const auto result = planner.plan(planningGrid, *start, *goal);
        require(result.succeeded() && result.path.front() == *start &&
                result.path.back() == *goal,
            "planner result should retain the ROI-local start and goal cells");
        requireValidEightConnectedPath(planningGrid, result.path, *start, *goal);
        require(planningGrid.localToMaster(result.path.front()) == GridCell{ 2, 1 } &&
                planningGrid.localToMaster(result.path.back()) == GridCell{ 7, 8 },
            "ROI-local result cells should map back to the correct master cells");
        require(pointsNear(
                planningGrid.geometry().cellCenterToWorld(result.path.front()),
                worldStart) &&
                pointsNear(
                    planningGrid.geometry().cellCenterToWorld(result.path.back()),
                    worldGoal),
            "planner result endpoints should map back to their world positions");
    }

    void planningRoiCanExcludeAValidDetour()
    {
        const OccupancyGrid master = gridWithOccupiedCells(
            7, 5, { { 3, 1 }, { 3, 2 }, { 3, 3 } });
        const OccupancyGrid minimalTerminalRoi = master.subgrid(
            WorldBounds{ Point2(1.0, 2.0), Point2(6.0, 3.0) });
        const AStarGridPlanner planner;
        AStarOptions options;
        options.connectivity = GridConnectivity::FourConnected;

        const auto croppedResult = planner.plan(
            minimalTerminalRoi, { 0, 0 }, { 4, 0 }, options);
        require(croppedResult.status == GridPlanStatus::NoPath,
            "minimal terminal ROI should exclude the route around the wall");

        const auto masterResult = planner.plan(
            master, { 1, 2 }, { 5, 2 }, options);
        require(masterResult.succeeded(),
            "full master grid should retain the valid detour around the wall");
        requireValidFourConnectedPath(
            master, masterResult.path, { 1, 2 }, { 5, 2 });
    }

    void gridPathConvertsRoiCellsToWorldCenters()
    {
        const OccupancyGrid master(
            GridGeometry(Point2(10.0, -4.0), 0.5, 8, 6),
            CellState::Free);
        const OccupancyGrid planningGrid = master.subgrid(
            GridRegion{ { 2, 1 }, 4, 3 });
        const std::vector<GridCell> gridPath{
            { 0, 0 }, { 1, 1 }, { 3, 2 }
        };

        const std::vector<Point2> worldPath = gridPathToWorld(
            planningGrid, gridPath);
        require(worldPath.size() == gridPath.size(),
            "world path should preserve the number and order of grid cells");
        require(pointsNear(worldPath[0], Point2(11.25, -3.25)) &&
                pointsNear(worldPath[1], Point2(11.75, -2.75)) &&
                pointsNear(worldPath[2], Point2(12.75, -2.25)),
            "ROI-local path cells should convert to their world-space centers");
        require(gridPathToWorld(planningGrid, {}).empty(),
            "an empty grid path should produce an empty world path");
        requireThrows<std::out_of_range>(
            [&planningGrid]
            {
                gridPathToWorld(planningGrid, { { 4, 0 } });
            },
            "path conversion should reject cells outside the planning grid");
    }

    void operationAreaDemoEnvironmentFindsPath()
    {
        const Polygon operationArea{
            Point2(0.0, 0.0), Point2(100.0, 0.0),
            Point2(100.0, 60.0), Point2(0.0, 60.0)
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
        const PolygonEnvironment environment(operationArea, obstacles);
        const OccupancyGrid master = PolygonRasterizer::rasterize(
            environment.operationArea(), environment.effectiveObstacles(), 1.0);
        const OccupancyGrid planningRegion = master.subgrid(
            WorldBounds{ Point2(3.0, 0.0), Point2(97.0, 60.0) });
        const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
            planningRegion, 1.0);
        const auto start = planningGrid.geometry().worldToCell(Point2(5.0, 8.0));
        const auto goal = planningGrid.geometry().worldToCell(Point2(95.0, 52.0));
        require(start.has_value() && goal.has_value(),
            "operation-area demo terminals should be inside its planning ROI");

        const AStarGridPlanner planner;
        AStarOptions options;
        options.algorithm = GridSearchAlgorithm::Dijkstra;
        const auto result = planner.plan(planningGrid, *start, *goal, options);
        require(result.succeeded(),
            std::string("operation-area demo should find a path: ") +
                astar::gridPlanStatusName(result.status));
        requireValidEightConnectedPath(planningGrid, result.path, *start, *goal);
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
        { "Translated small polygon retains area", translatedSmallPolygonRetainsArea },
        { "Grid geometry converts coordinate frames", gridGeometryConvertsBetweenCoordinateFrames },
        { "Grid geometry uses half-open bounds", gridGeometryUsesHalfOpenWorldBounds },
        { "Covering geometry rounds extents up", coveringGeometryRoundsExtentUpToWholeCells },
        { "Covering geometry defines planning boundary", coveringGeometryDefinesAuthoritativePlanningBoundary },
        { "Invalid grid geometry is rejected", invalidGridGeometryIsRejected },
        { "Aligned covering uses stable world lattice", alignedCoveringUsesStableWorldLattice },
        { "Polygon bounds size rectangular maps", polygonBoundsCanSizeRectangularMaps },
        { "Occupancy grid supports uniform initialization", occupancyGridSupportsUniformInitialization },
        { "Inflation uses conservative circular radius", inflationUsesConservativeCircularRadius },
        { "Inflation treats grid boundary as occupied", inflationTreatsGridBoundaryAsOccupied },
        { "Zero inflation preserves metadata and validates radius", zeroInflationPreservesGridMetadataAndValidatesRadius },
        { "Rasterization paints covered cells", unconstrainedRasterizationPaintsCoveredCells },
        { "Operation area paints covered cells free", operationAreaPaintsCoveredCellsFree },
        { "Obstacle painting uses OpenCV coverage", obstaclePaintingUsesOpenCvCoverage },
        { "Rasterizer normalizes valid polygons", rasterizerNormalizesValidPolygons },
        { "Rasterizer rejects invalid polygons", rasterizerRejectsInvalidPolygons },
        { "Obstacle outside grid leaves map free", obstacleOutsideGridLeavesMapFree },
        { "Obstacle overrides operation area", obstacleOverridesOperationAreaFreeSpace },
        { "Point rasterization paints contained world points", pointRasterizationPaintsContainedWorldPoints },
        { "Occupancy fusion uses occupied union", occupancyFusionUsesOccupiedUnion },
        { "Occupancy fusion rejects incompatible grids", occupancyFusionRejectsIncompatibleGrids },
        { "Rasterization produces binary OpenCV image", rasterizationProducesBinaryOpenCvImage },
        { "Operation area builds aligned master grid", operationAreaCanBuildAlignedMasterGrid },
        { "Validated environment selects rasterization mode", validatedEnvironmentSelectsRasterizationMode },
        { "Subgrid preserves coordinates", subgridPreservesWorldAndMasterCoordinates },
        { "Copied subgrid owns independent pixels", copiedSubgridOwnsIndependentPixels },
        { "Shared subgrid retains storage lifetime", sharedSubgridRetainsStorageLifetime },
        { "World bounds produce rounded subgrid", worldBoundsProduceOutwardRoundedSubgrid },
        { "World bounds containment tolerates roundoff", worldBoundsContainmentToleratesRoundoff },
        { "Nested subgrids retain master offset", nestedSubgridsRetainMasterOffset },
        { "A* finds shortest path across empty grid", aStarFindsShortestPathAcrossEmptyGrid },
        { "A* handles coincident terminals", aStarHandlesCoincidentTerminals },
        { "A* validates terminals", aStarValidatesTerminals },
        { "Grid plan result has safe default state", gridPlanResultHasSafeDefaultState },
        { "Grid path simplification uses conservative line of sight", gridPathSimplificationUsesConservativeLineOfSight },
        { "Grid planner rejects unsupported options", gridPlannerRejectsUnsupportedOptions },
        { "Dijkstra matches A* optimal cost", dijkstraMatchesAStarOptimalCost },
        { "Randomized A* matches Dijkstra", randomizedAStarMatchesDijkstra },
        { "Weighted A* uses shared weighted heuristic", weightedAStarUsesSharedWeightedHeuristic },
        { "Search debug callback publishes state grid", searchDebugCallbackPublishesReusableStateGrid },
        { "Detailed diagnostics track repeated expansions", detailedSearchDiagnosticsTrackRepeatedExpansions },
        { "A* routes around obstacle", aStarRoutesAroundObstacle },
        { "A* reconstructs complete parent chain", aStarReconstructsCompleteParentChain },
        { "A* uses eight-connected diagonal path by default", aStarUsesEightConnectedDiagonalPathByDefault },
        { "A* controls diagonal corner cutting", aStarControlsDiagonalCornerCutting },
        { "A* reports when no path exists", aStarReportsWhenNoPathExists },
        { "A* planning pipeline uses ROI coordinates", aStarPlanningPipelineUsesRoiLocalCoordinates },
        { "Planning ROI can exclude valid detour", planningRoiCanExcludeAValidDetour },
        { "Grid path converts ROI cells to world centers", gridPathConvertsRoiCellsToWorldCenters },
        { "Operation-area demo environment finds path", operationAreaDemoEnvironmentFindsPath }
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
