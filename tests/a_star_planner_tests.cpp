/*
 * Environment-model and occupancy-grid tests for AStarPlanner.
 * A* path planning is intentionally not active yet.
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
    using astar::CellState;
    using astar::GridCell;
    using astar::GridGeometry;
    using astar::GridPlanStatus;
    using astar::GridRegion;
    using astar::OccupancyGrid;
    using astar::Point2;
    using astar::Polygon;
    using astar::PolygonRasterizer;
    using astar::SubgridStorage;
    using astar::WorldBounds;
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

    void unconstrainedRasterizationMarksIntersectingCells()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 4, 4);
        const Polygon obstacle{
            Point2(0.75, 0.75), Point2(1.25, 0.75),
            Point2(1.25, 1.25), Point2(0.75, 1.25)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(geometry, { obstacle });
        require(grid.at({ 0, 0 }) == CellState::Occupied,
            "obstacle crossing a cell corner should occupy the cell");
        require(grid.at({ 1, 0 }) == CellState::Occupied,
            "obstacle should occupy every intersected cell");
        require(grid.at({ 0, 1 }) == CellState::Occupied,
            "obstacle should occupy every intersected row");
        require(grid.at({ 1, 1 }) == CellState::Occupied,
            "obstacle should occupy all four intersected cells");
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

    void operationAreaRequiresWholeCellContainment()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 3);
        const Polygon operationArea{
            Point2(0.25, 0.25), Point2(2.75, 0.25),
            Point2(2.75, 2.75), Point2(0.25, 2.75)
        };

        const OccupancyGrid grid =
            PolygonRasterizer::rasterize(geometry, operationArea, {});
        require(grid.isTraversable({ 1, 1 }),
            "cell wholly inside the operation area should be traversable");
        require(grid.at({ 0, 0 }) == CellState::Occupied,
            "cell crossing the operation-area boundary should remain occupied");
        require(grid.at({ 2, 2 }) == CellState::Occupied,
            "boundary rule should apply at every side of the operation area");
    }

    void obstacleBoundaryContactIsConservativelyOccupied()
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 3, 2);
        const Polygon obstacle{
            Point2(1.0, 0.2), Point2(1.2, 0.2),
            Point2(1.2, 0.8), Point2(1.0, 0.8)
        };

        const OccupancyGrid grid = PolygonRasterizer::rasterize(geometry, { obstacle });
        require(grid.at({ 0, 0 }) == CellState::Occupied,
            "cell touched by an obstacle boundary should be occupied");
        require(grid.at({ 1, 0 }) == CellState::Occupied,
            "cell containing obstacle area should be occupied");
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
            "whole cells inside the operation area should remain free");
        require(grid.image().rows == 3 && grid.image().cols == 3,
            "OpenCV image dimensions should match grid dimensions");
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
        require(master.at({ 0, 0 }) == CellState::Occupied,
            "partial boundary cells should remain occupied in the master grid");
        require(master.isTraversable({ 1, 1 }),
            "whole cells inside the operation area should be free");
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

    void bootstrapPlannerReturnsRasterizedLine()
    {
        const OccupancyGrid grid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 6, 4),
            CellState::Free);
        const AStarGridPlanner planner;

        const auto result = planner.plan(grid, { 0, 0 }, { 5, 2 });
        require(result.succeeded(),
            "bootstrap planner should succeed for terminals inside the grid");
        require(result.path == rasterizeGridLine({ 0, 0 }, { 5, 2 }),
            "bootstrap planner should return the reusable rasterized line");
    }

    void bootstrapPlannerHandlesCoincidentTerminals()
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

    void bootstrapPlannerValidatesBoundsButIgnoresOccupancy()
    {
        const OccupancyGrid occupiedGrid(
            GridGeometry(Point2(0.0, 0.0), 1.0, 4, 4),
            CellState::Occupied);
        const AStarGridPlanner planner;

        const auto occupiedResult = planner.plan(occupiedGrid, { 0, 0 }, { 3, 3 });
        require(occupiedResult.succeeded() && !occupiedResult.path.empty(),
            "bootstrap planner should intentionally ignore occupancy in this stage");

        const auto invalidStart = planner.plan(occupiedGrid, { -1, 0 }, { 3, 3 });
        require(invalidStart.status == GridPlanStatus::StartOutsideGrid &&
                invalidStart.path.empty(),
            "planner should reject a start cell outside the planning grid");

        const auto invalidGoal = planner.plan(occupiedGrid, { 0, 0 }, { 4, 3 });
        require(invalidGoal.status == GridPlanStatus::GoalOutsideGrid &&
                invalidGoal.path.empty(),
            "planner should reject a goal cell outside the planning grid");
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
        { "Rasterization marks intersecting cells", unconstrainedRasterizationMarksIntersectingCells },
        { "Operation area requires whole-cell containment", operationAreaRequiresWholeCellContainment },
        { "Obstacle boundary contact is occupied", obstacleBoundaryContactIsConservativelyOccupied },
        { "Obstacle outside grid leaves map free", obstacleOutsideGridLeavesMapFree },
        { "Obstacle overrides operation area", obstacleOverridesOperationAreaFreeSpace },
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
        { "Bootstrap planner returns rasterized line", bootstrapPlannerReturnsRasterizedLine },
        { "Bootstrap planner handles coincident terminals", bootstrapPlannerHandlesCoincidentTerminals },
        { "Bootstrap planner validates grid bounds", bootstrapPlannerValidatesBoundsButIgnoresOccupancy }
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
