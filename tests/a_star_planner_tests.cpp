/*
 * Environment-model, occupancy-grid, and A* search tests for AStarPlanner.
 */

#include "a_star_planner.hpp"
#include "a_star_grid_planner.hpp"
#include "grid_line.hpp"
#include "occupancy_grid.hpp"

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
    using astar::AStarGridPlanner;
    using astar::AStarOptions;
    using astar::CellState;
    using astar::GridCell;
    using astar::GridGeometry;
    using astar::GridConnectivity;
    using astar::GridPlanStatus;
    using astar::GridRegion;
    using astar::OccupancyGrid;
    using astar::Point2;
    using astar::Polygon;
    using astar::PolygonRasterizer;
    using astar::SubgridStorage;
    using astar::WorldBounds;
    using astar::gridPathToWorld;
    using astar::rasterizeGridLine;

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

            const GridCell& previous = path[index - 1];
            const int columnStep = std::abs(path[index].column - previous.column);
            const int rowStep = std::abs(path[index].row - previous.row);
            require(std::max(columnStep, rowStep) == 1,
                "consecutive path cells should be eight-connected neighbors");
            if (columnStep == 1 && rowStep == 1)
            {
                require(
                    grid.isTraversable({ path[index].column, previous.row }) &&
                    grid.isTraversable({ previous.column, path[index].row }),
                    "diagonal path steps should not cut occupied corners");
            }
        }
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
        const GridGeometry geometry = GridGeometry::covering(
            WorldBounds{ Point2(-1.0, 2.0), Point2(1.1, 3.01) }, 0.5);

        require(geometry.width() == 5 && geometry.height() == 3,
            "covering geometry should round each extent upward");
        require(pointsNear(geometry.worldMaximum(), Point2(1.5, 3.5)),
            "covering geometry should expose its snapped world maximum");
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

        require(cv::countNonZero(freeGrid.image()) == 0,
            "free initialization should fill the OpenCV image with zero");
        require(cv::countNonZero(occupiedGrid.image()) ==
                geometry.width() * geometry.height(),
            "occupied initialization should fill every OpenCV image pixel");
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
        require(grid.image().rows == 3 && grid.image().cols == 3,
            "OpenCV image dimensions should match grid dimensions");
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
        cv::inRange(grid.image(), cv::Scalar(1), cv::Scalar(254), nonBinaryPixels);
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

    void validatedEnvironmentFeedsMasterRasterization()
    {
        const Polygon operationArea{
            Point2(0.0, 0.0), Point2(4.0, 0.0),
            Point2(4.0, 4.0), Point2(0.0, 4.0)
        };
        const Polygon crossingObstacle{
            Point2(3.0, 1.0), Point2(5.0, 1.0),
            Point2(5.0, 3.0), Point2(3.0, 3.0)
        };
        const AStarPlanner planner(operationArea, { crossingObstacle });

        const OccupancyGrid master = PolygonRasterizer::rasterize(
            planner.operationArea(), planner.obstacles(), 1.0);

        require(master.width() == 4 && master.height() == 4,
            "validated operation area should determine master dimensions");
        require(master.isTraversable({ 0, 0 }),
            "free operation-area cells should remain traversable");
        require(master.at({ 3, 1 }) == CellState::Occupied,
            "clipped effective obstacle should be present in the master grid");
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
        require(window.image().data ==
                master.image().ptr(masterImageRow) + region.lowerLeft.column,
            "default subgrid should share the selected OpenCV ROI storage");
    }

    void copiedSubgridOwnsIndependentPixels()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const OccupancyGrid master(geometry, CellState::Occupied);
        const GridRegion region{ { 1, 1 }, 2, 2 };
        const OccupancyGrid copy = master.subgrid(
            region, SubgridStorage::IndependentCopy);

        require(copy.image().data != master.image().data,
            "copied subgrid should own independent OpenCV storage");
        require(cv::countNonZero(copy.image()) == 4,
            "copied subgrid should preserve occupancy values");
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

    void gridLineRasterizesAxisAlignedSegments()
    {
        const std::vector<GridCell> horizontal{
            { 1, 2 }, { 2, 2 }, { 3, 2 }, { 4, 2 }
        };
        const std::vector<GridCell> verticalReverse{
            { 3, 3 }, { 3, 2 }, { 3, 1 }, { 3, 0 }
        };

        require(rasterizeGridLine({ 1, 2 }, { 4, 2 }) == horizontal,
            "horizontal line should include every cell and both endpoints");
        require(rasterizeGridLine({ 3, 3 }, { 3, 0 }) == verticalReverse,
            "reverse vertical line should preserve start-to-goal ordering");
    }

    void gridLineRasterizesDiagonalAndCoincidentSegments()
    {
        const std::vector<GridCell> diagonal{
            { -1, 1 }, { 0, 2 }, { 1, 3 }, { 2, 4 }
        };

        require(rasterizeGridLine({ -1, 1 }, { 2, 4 }) == diagonal,
            "diagonal line should advance one column and row per cell");
        require(rasterizeGridLine({ 5, -2 }, { 5, -2 }) ==
                std::vector<GridCell>{ { 5, -2 } },
            "coincident endpoints should produce exactly one cell");
    }

    void gridLineRasterizesShallowAndSteepSegments()
    {
        const std::vector<GridCell> shallow{
            { 0, 0 }, { 1, 0 }, { 2, 1 }, { 3, 1 }, { 4, 2 }, { 5, 2 }
        };
        const std::vector<GridCell> steep{
            { 0, 0 }, { 0, 1 }, { 1, 2 }, { 1, 3 }, { 2, 4 }, { 2, 5 }
        };

        require(rasterizeGridLine({ 0, 0 }, { 5, 2 }) == shallow,
            "shallow line should select the expected discrete cells");
        require(rasterizeGridLine({ 0, 0 }, { 2, 5 }) == steep,
            "steep line should select the expected discrete cells");
    }

    void gridLineIsReversible()
    {
        const std::vector<GridCell> forward = rasterizeGridLine({ -3, 4 }, { 4, 1 });
        std::vector<GridCell> reverse = rasterizeGridLine({ 4, 1 }, { -3, 4 });
        std::reverse(reverse.begin(), reverse.end());

        require(forward == reverse,
            "reversing endpoints should reverse the rasterized line");
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
        const AStarPlanner environment(operationArea, { obstacle });
        const OccupancyGrid master = PolygonRasterizer::rasterize(
            environment.operationArea(), environment.obstacles(), 1.0);
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
        const AStarPlanner environment(operationArea, obstacles);
        const OccupancyGrid master = PolygonRasterizer::rasterize(
            environment.operationArea(), environment.obstacles(), 1.0);
        const OccupancyGrid planningGrid = master.subgrid(
            WorldBounds{ Point2(4.0, 0.0), Point2(96.0, 60.0) });
        const auto start = planningGrid.geometry().worldToCell(Point2(5.0, 8.0));
        const auto goal = planningGrid.geometry().worldToCell(Point2(95.0, 52.0));
        require(start.has_value() && goal.has_value(),
            "operation-area demo terminals should be inside its planning ROI");

        const AStarGridPlanner planner;
        const auto result = planner.plan(planningGrid, *start, *goal);
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
        { "Invalid grid geometry is rejected", invalidGridGeometryIsRejected },
        { "Aligned covering uses stable world lattice", alignedCoveringUsesStableWorldLattice },
        { "Polygon bounds size rectangular maps", polygonBoundsCanSizeRectangularMaps },
        { "Occupancy grid supports uniform initialization", occupancyGridSupportsUniformInitialization },
        { "Rasterization paints covered cells", unconstrainedRasterizationPaintsCoveredCells },
        { "Operation area paints covered cells free", operationAreaPaintsCoveredCellsFree },
        { "Obstacle painting uses OpenCV coverage", obstaclePaintingUsesOpenCvCoverage },
        { "Obstacle outside grid leaves map free", obstacleOutsideGridLeavesMapFree },
        { "Obstacle overrides operation area", obstacleOverridesOperationAreaFreeSpace },
        { "Rasterization produces binary OpenCV image", rasterizationProducesBinaryOpenCvImage },
        { "Operation area builds aligned master grid", operationAreaCanBuildAlignedMasterGrid },
        { "Validated environment feeds master rasterization", validatedEnvironmentFeedsMasterRasterization },
        { "Subgrid preserves coordinates", subgridPreservesWorldAndMasterCoordinates },
        { "Copied subgrid owns independent pixels", copiedSubgridOwnsIndependentPixels },
        { "World bounds produce rounded subgrid", worldBoundsProduceOutwardRoundedSubgrid },
        { "Nested subgrids retain master offset", nestedSubgridsRetainMasterOffset },
        { "Grid line rasterizes axis-aligned segments", gridLineRasterizesAxisAlignedSegments },
        { "Grid line rasterizes diagonal segments", gridLineRasterizesDiagonalAndCoincidentSegments },
        { "Grid line rasterizes shallow and steep segments", gridLineRasterizesShallowAndSteepSegments },
        { "Grid line is reversible", gridLineIsReversible },
        { "A* finds shortest path across empty grid", aStarFindsShortestPathAcrossEmptyGrid },
        { "A* handles coincident terminals", aStarHandlesCoincidentTerminals },
        { "A* validates terminals", aStarValidatesTerminals },
        { "A* routes around obstacle", aStarRoutesAroundObstacle },
        { "A* uses eight-connected diagonal path by default", aStarUsesEightConnectedDiagonalPathByDefault },
        { "A* controls diagonal corner cutting", aStarControlsDiagonalCornerCutting },
        { "A* reports when no path exists", aStarReportsWhenNoPathExists },
        { "A* planning pipeline uses ROI coordinates", aStarPlanningPipelineUsesRoiLocalCoordinates },
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
