/*
 * occupancy_grid.hpp - OpenCV-backed occupancy-grid geometry and rasterization.
 */
#pragma once

#include "a_star_planner.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace astar
{
    struct GridCell
    {
        int column;
        int row; // Cartesian row: zero is the bottom row.

        bool operator==(const GridCell& other) const noexcept
        {
            return column == other.column && row == other.row;
        }

        bool operator!=(const GridCell& other) const noexcept
        {
            return !(*this == other);
        }
    };

    struct WorldBounds
    {
        Point2 minimum;
        Point2 maximum;
    };

    struct GridRegion
    {
        GridCell lowerLeft;
        int width;
        int height;

        bool operator==(const GridRegion& other) const noexcept
        {
            return lowerLeft == other.lowerLeft &&
                   width == other.width && height == other.height;
        }
    };

    enum class SubgridStorage
    {
        SharedView,
        IndependentCopy
    };

    /**
     * Fixed transform between world, Cartesian-grid, and OpenCV-image coordinates.
     *
     * The world origin is the lower-left outer corner of cell (0, 0). Grid rows
     * increase with world y; OpenCV image rows increase downward.
     */
    class GridGeometry
    {
    public:
        GridGeometry(Point2 worldOrigin, double resolution, int width, int height)
            : _worldOrigin(std::move(worldOrigin)),
              _resolution(resolution),
              _width(width),
              _height(height)
        {
            if (!isFinite(_worldOrigin))
                throw std::invalid_argument("GridGeometry: world origin must be finite");
            if (!std::isfinite(_resolution) || _resolution <= 0.0)
                throw std::invalid_argument("GridGeometry: resolution must be finite and positive");
            if (_width <= 0 || _height <= 0)
                throw std::invalid_argument("GridGeometry: width and height must be positive");
            if (!isFinite(worldMaximum()))
                throw std::invalid_argument("GridGeometry: world extent is not finite");
        }

        static GridGeometry covering(const WorldBounds& bounds, double resolution)
        {
            validateBoundsAndResolution(bounds, resolution, "GridGeometry::covering");

            const Point2 extent = bounds.maximum - bounds.minimum;
            return GridGeometry(
                bounds.minimum,
                resolution,
                cellCount(extent.x(), resolution),
                cellCount(extent.y(), resolution));
        }

        static GridGeometry covering(const Polygon& polygon, double resolution)
        {
            return covering(boundingBox(polygon), resolution);
        }

        static GridGeometry covering(
            const std::vector<Polygon>& polygons,
            double resolution)
        {
            return covering(boundingBox(polygons), resolution);
        }

        /**
         * Covers the bounds while keeping every cell boundary on the lattice
         * defined by @p anchor and @p resolution.
         */
        static GridGeometry alignedCovering(
            const WorldBounds& bounds,
            double resolution,
            const Point2& anchor = Point2::Zero())
        {
            validateBoundsAndResolution(bounds, resolution, "GridGeometry::alignedCovering");
            if (!isFinite(anchor))
                throw std::invalid_argument("GridGeometry::alignedCovering: anchor must be finite");

            const double firstColumn =
                std::floor((bounds.minimum.x() - anchor.x()) / resolution);
            const double pastLastColumn =
                std::ceil((bounds.maximum.x() - anchor.x()) / resolution);
            const double firstRow =
                std::floor((bounds.minimum.y() - anchor.y()) / resolution);
            const double pastLastRow =
                std::ceil((bounds.maximum.y() - anchor.y()) / resolution);

            const Point2 origin(
                anchor.x() + firstColumn * resolution,
                anchor.y() + firstRow * resolution);
            return GridGeometry(
                origin,
                resolution,
                indexSpan(firstColumn, pastLastColumn),
                indexSpan(firstRow, pastLastRow));
        }

        static GridGeometry alignedCovering(
            const Polygon& polygon,
            double resolution,
            const Point2& anchor = Point2::Zero())
        {
            return alignedCovering(boundingBox(polygon), resolution, anchor);
        }

        static GridGeometry alignedCovering(
            const std::vector<Polygon>& polygons,
            double resolution,
            const Point2& anchor = Point2::Zero())
        {
            return alignedCovering(boundingBox(polygons), resolution, anchor);
        }

        static WorldBounds boundingBox(const Polygon& polygon)
        {
            if (polygon.empty())
                throw std::invalid_argument("GridGeometry::boundingBox: polygon is empty");

            WorldBounds bounds{ polygon.front(), polygon.front() };
            for (const Point2& point : polygon)
                extendBounds(bounds, point, "GridGeometry::boundingBox");
            return bounds;
        }

        static WorldBounds boundingBox(const std::vector<Polygon>& polygons)
        {
            std::optional<WorldBounds> bounds;
            for (const Polygon& polygon : polygons)
            {
                for (const Point2& point : polygon)
                {
                    if (!bounds)
                    {
                        if (!isFinite(point))
                        {
                            throw std::invalid_argument(
                                "GridGeometry::boundingBox: polygon coordinate must be finite");
                        }
                        bounds = WorldBounds{ point, point };
                    }
                    else
                    {
                        extendBounds(*bounds, point, "GridGeometry::boundingBox");
                    }
                }
            }

            if (!bounds)
                throw std::invalid_argument("GridGeometry::boundingBox: no polygon vertices");
            return *bounds;
        }

        const Point2& worldOrigin() const noexcept { return _worldOrigin; }
        double resolution() const noexcept { return _resolution; }
        int width() const noexcept { return _width; }
        int height() const noexcept { return _height; }

        Point2 worldMaximum() const noexcept
        {
            return Point2(
                _worldOrigin.x() + static_cast<double>(_width) * _resolution,
                _worldOrigin.y() + static_cast<double>(_height) * _resolution);
        }

        WorldBounds worldBounds() const noexcept
        {
            return { _worldOrigin, worldMaximum() };
        }

        bool contains(const WorldBounds& bounds) const noexcept
        {
            if (!isFinite(bounds.minimum) || !isFinite(bounds.maximum) ||
                bounds.minimum.x() >= bounds.maximum.x() ||
                bounds.minimum.y() >= bounds.maximum.y())
            {
                return false;
            }

            const Point2 maximum = worldMaximum();
            return bounds.minimum.x() >= _worldOrigin.x() &&
                   bounds.minimum.y() >= _worldOrigin.y() &&
                   bounds.maximum.x() <= maximum.x() &&
                   bounds.maximum.y() <= maximum.y();
        }

        bool contains(const GridCell& cell) const noexcept
        {
            return cell.column >= 0 && cell.column < _width &&
                   cell.row >= 0 && cell.row < _height;
        }

        bool containsWorld(const Point2& point) const noexcept
        {
            if (!isFinite(point))
                return false;

            const Point2 maximum = worldMaximum();
            return point.x() >= _worldOrigin.x() && point.x() < maximum.x() &&
                   point.y() >= _worldOrigin.y() && point.y() < maximum.y();
        }

        std::optional<GridCell> worldToCell(const Point2& point) const noexcept
        {
            if (!containsWorld(point))
                return std::nullopt;

            const Point2 local = (point - _worldOrigin) / _resolution;
            const GridCell cell{
                static_cast<int>(std::floor(local.x())),
                static_cast<int>(std::floor(local.y()))
            };
            return contains(cell) ? std::optional<GridCell>(cell) : std::nullopt;
        }

        Point2 cellCenterToWorld(const GridCell& cell) const
        {
            requireContained(cell, "GridGeometry::cellCenterToWorld");
            return Point2(
                _worldOrigin.x() + (static_cast<double>(cell.column) + 0.5) * _resolution,
                _worldOrigin.y() + (static_cast<double>(cell.row) + 0.5) * _resolution);
        }

        WorldBounds cellBounds(const GridCell& cell) const
        {
            requireContained(cell, "GridGeometry::cellBounds");
            const Point2 minimum(
                _worldOrigin.x() + static_cast<double>(cell.column) * _resolution,
                _worldOrigin.y() + static_cast<double>(cell.row) * _resolution);
            return { minimum, minimum + Point2(_resolution, _resolution) };
        }

        cv::Point cellToImage(const GridCell& cell) const
        {
            requireContained(cell, "GridGeometry::cellToImage");
            return { cell.column, _height - 1 - cell.row };
        }

        std::optional<GridCell> imageToCell(const cv::Point& pixel) const noexcept
        {
            if (pixel.x < 0 || pixel.x >= _width || pixel.y < 0 || pixel.y >= _height)
                return std::nullopt;
            return GridCell{ pixel.x, _height - 1 - pixel.y };
        }

        cv::Point2d worldToImage(const Point2& point) const
        {
            if (!isFinite(point))
                throw std::invalid_argument("GridGeometry::worldToImage: point must be finite");

            const Point2 local = (point - _worldOrigin) / _resolution;
            return {
                local.x() - 0.5,
                static_cast<double>(_height) - local.y() - 0.5
            };
        }

        std::vector<cv::Point2d> worldToImage(const Polygon& polygon) const
        {
            std::vector<cv::Point2d> imagePolygon;
            imagePolygon.reserve(polygon.size());
            for (const Point2& point : polygon)
                imagePolygon.push_back(worldToImage(point));
            return imagePolygon;
        }

        Point2 imageToWorld(const cv::Point2d& point) const
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
                throw std::invalid_argument("GridGeometry::imageToWorld: point must be finite");

            return Point2(
                _worldOrigin.x() + (point.x + 0.5) * _resolution,
                _worldOrigin.y() +
                    (static_cast<double>(_height) - point.y - 0.5) * _resolution);
        }

    private:
        static bool isFinite(const Point2& point) noexcept
        {
            return std::isfinite(point.x()) && std::isfinite(point.y());
        }

        static int cellCount(double extent, double resolution)
        {
            const double count = std::ceil(extent / resolution);
            if (!std::isfinite(count) || count <= 0.0 ||
                count > static_cast<double>(std::numeric_limits<int>::max()))
            {
                throw std::invalid_argument(
                    "GridGeometry::covering: requested grid dimension is too large");
            }
            return static_cast<int>(count);
        }

        static int indexSpan(double first, double pastLast)
        {
            const double span = pastLast - first;
            if (!std::isfinite(first) || !std::isfinite(pastLast) ||
                !std::isfinite(span) || span <= 0.0 ||
                span > static_cast<double>(std::numeric_limits<int>::max()))
            {
                throw std::invalid_argument(
                    "GridGeometry::alignedCovering: requested grid dimension is too large");
            }
            return static_cast<int>(span);
        }

        static void validateBoundsAndResolution(
            const WorldBounds& bounds,
            double resolution,
            const char* functionName)
        {
            if (!isFinite(bounds.minimum) || !isFinite(bounds.maximum))
                throw std::invalid_argument(std::string(functionName) + ": bounds must be finite");
            if (!std::isfinite(resolution) || resolution <= 0.0)
            {
                throw std::invalid_argument(
                    std::string(functionName) + ": resolution must be finite and positive");
            }
            if (bounds.maximum.x() <= bounds.minimum.x() ||
                bounds.maximum.y() <= bounds.minimum.y())
            {
                throw std::invalid_argument(
                    std::string(functionName) +
                    ": maximum must be greater than minimum");
            }
        }

        static void extendBounds(
            WorldBounds& bounds,
            const Point2& point,
            const char* functionName)
        {
            if (!isFinite(point))
            {
                throw std::invalid_argument(
                    std::string(functionName) + ": polygon coordinate must be finite");
            }
            bounds.minimum.x() = std::min(bounds.minimum.x(), point.x());
            bounds.minimum.y() = std::min(bounds.minimum.y(), point.y());
            bounds.maximum.x() = std::max(bounds.maximum.x(), point.x());
            bounds.maximum.y() = std::max(bounds.maximum.y(), point.y());
        }

        void requireContained(const GridCell& cell, const char* functionName) const
        {
            if (!contains(cell))
                throw std::out_of_range(std::string(functionName) + ": cell is outside the grid");
        }

        Point2 _worldOrigin;
        double _resolution;
        int _width;
        int _height;
    };

    enum class CellState : std::uint8_t
    {
        Free = 0,
        Occupied = 255
    };

    class PolygonRasterizer;

    /** Immutable, binary occupancy map backed by a single-channel OpenCV image. */
    class OccupancyGrid
    {
    public:
        /** Constructs a grid with every cell set to the requested state. */
        OccupancyGrid(GridGeometry geometry, CellState initialState)
            : _geometry(std::move(geometry)),
              _occupancy(
                  _geometry.height(),
                  _geometry.width(),
                  static_cast<std::uint8_t>(initialState)),
              _masterCellOffset{ 0, 0 }
        {
        }

        const GridGeometry& geometry() const noexcept { return _geometry; }
        int width() const noexcept { return _geometry.width(); }
        int height() const noexcept { return _geometry.height(); }

        bool contains(const GridCell& cell) const noexcept
        {
            return _geometry.contains(cell);
        }

        CellState at(const GridCell& cell) const
        {
            const cv::Point pixel = _geometry.cellToImage(cell);
            return static_cast<CellState>(_occupancy(pixel.y, pixel.x));
        }

        bool isTraversable(const GridCell& cell) const noexcept
        {
            if (!contains(cell))
                return false;
            const cv::Point pixel{ cell.column, height() - 1 - cell.row };
            return _occupancy(pixel.y, pixel.x) ==
                static_cast<std::uint8_t>(CellState::Free);
        }

        const cv::Mat1b& image() const noexcept { return _occupancy; }
        cv::Mat1b cloneImage() const { return _occupancy.clone(); }

        /** Offset of local cell (0, 0) in the source master grid. */
        const GridCell& masterCellOffset() const noexcept
        {
            return _masterCellOffset;
        }

        GridCell localToMaster(const GridCell& localCell) const
        {
            if (!contains(localCell))
            {
                throw std::out_of_range(
                    "OccupancyGrid::localToMaster: cell is outside the grid");
            }
            return {
                _masterCellOffset.column + localCell.column,
                _masterCellOffset.row + localCell.row
            };
        }

        std::optional<GridCell> masterToLocal(const GridCell& masterCell) const noexcept
        {
            const GridCell local{
                masterCell.column - _masterCellOffset.column,
                masterCell.row - _masterCellOffset.row
            };
            return contains(local) ? std::optional<GridCell>(local) : std::nullopt;
        }

        GridRegion fullRegion() const noexcept
        {
            return { { 0, 0 }, width(), height() };
        }

        /** Returns the smallest cell-aligned region containing the world bounds. */
        GridRegion regionCovering(const WorldBounds& bounds) const
        {
            if (!_geometry.contains(bounds))
            {
                throw std::out_of_range(
                    "OccupancyGrid::regionCovering: bounds are outside the grid");
            }

            const Point2 localMinimum =
                (bounds.minimum - _geometry.worldOrigin()) / _geometry.resolution();
            const Point2 localMaximum =
                (bounds.maximum - _geometry.worldOrigin()) / _geometry.resolution();

            const int firstColumn = clampedFloor(localMinimum.x(), width());
            const int firstRow = clampedFloor(localMinimum.y(), height());
            const int pastLastColumn = clampedCeil(localMaximum.x(), width());
            const int pastLastRow = clampedCeil(localMaximum.y(), height());
            return {
                { firstColumn, firstRow },
                pastLastColumn - firstColumn,
                pastLastRow - firstRow
            };
        }

        /**
         * Creates a cell-aligned planning grid. Shared views retain the master's
         * reference-counted OpenCV storage; independent copies own their pixels.
         */
        OccupancyGrid subgrid(
            const GridRegion& region,
            SubgridStorage storage = SubgridStorage::SharedView) const
        {
            validateRegion(region);
            const int imageRow = height() - (region.lowerLeft.row + region.height);
            const cv::Rect imageRegion(
                region.lowerLeft.column,
                imageRow,
                region.width,
                region.height);

            cv::Mat1b pixels = _occupancy(imageRegion);
            if (storage == SubgridStorage::IndependentCopy)
                pixels = pixels.clone();

            const Point2 origin = _geometry.cellBounds(region.lowerLeft).minimum;
            return OccupancyGrid(
                GridGeometry(origin, _geometry.resolution(), region.width, region.height),
                std::move(pixels),
                GridCell{
                    _masterCellOffset.column + region.lowerLeft.column,
                    _masterCellOffset.row + region.lowerLeft.row
                });
        }

        OccupancyGrid subgrid(
            const WorldBounds& bounds,
            SubgridStorage storage = SubgridStorage::SharedView) const
        {
            return subgrid(regionCovering(bounds), storage);
        }

    private:
        friend class PolygonRasterizer;

        OccupancyGrid(
            GridGeometry geometry,
            cv::Mat1b occupancy,
            GridCell masterCellOffset)
            : _geometry(std::move(geometry)),
              _occupancy(std::move(occupancy)),
              _masterCellOffset(masterCellOffset)
        {
            if (_occupancy.rows != _geometry.height() ||
                _occupancy.cols != _geometry.width())
            {
                throw std::invalid_argument(
                    "OccupancyGrid: image dimensions must match grid geometry");
            }
        }

        static int clampedFloor(double value, int dimension) noexcept
        {
            return std::clamp(
                static_cast<int>(std::floor(snapNearInteger(value))), 0, dimension);
        }

        static int clampedCeil(double value, int dimension) noexcept
        {
            return std::clamp(
                static_cast<int>(std::ceil(snapNearInteger(value))), 0, dimension);
        }

        static double snapNearInteger(double value) noexcept
        {
            const double nearest = std::round(value);
            const double tolerance =
                1e-12 * std::max(1.0, std::abs(value));
            return std::abs(value - nearest) <= tolerance ? nearest : value;
        }

        void validateRegion(const GridRegion& region) const
        {
            if (region.width <= 0 || region.height <= 0)
            {
                throw std::invalid_argument(
                    "OccupancyGrid::subgrid: width and height must be positive");
            }
            if (region.lowerLeft.column < 0 || region.lowerLeft.row < 0 ||
                region.lowerLeft.column >= width() || region.lowerLeft.row >= height() ||
                region.width > width() - region.lowerLeft.column ||
                region.height > height() - region.lowerLeft.row)
            {
                throw std::out_of_range(
                    "OccupancyGrid::subgrid: region is outside the grid");
            }
        }

        void set(const GridCell& cell, CellState state)
        {
            const cv::Point pixel = _geometry.cellToImage(cell);
            _occupancy(pixel.y, pixel.x) = static_cast<std::uint8_t>(state);
        }

        GridGeometry _geometry;
        cv::Mat1b _occupancy;
        GridCell _masterCellOffset;
    };

    /** Converts validated world-coordinate polygons into conservative grid cells. */
    class PolygonRasterizer
    {
    public:
        static OccupancyGrid rasterize(
            const GridGeometry& geometry,
            const std::vector<Polygon>& obstacles)
        {
            OccupancyGrid grid(geometry, CellState::Free);
            for (const Polygon& obstacle : obstacles)
                markIntersectingCells(grid, obstacle, CellState::Occupied);
            return grid;
        }

        static OccupancyGrid rasterize(
            const GridGeometry& geometry,
            const Polygon& operationArea,
            const std::vector<Polygon>& obstacles)
        {
            const std::vector<cv::Point2d> imageArea =
                checkedImagePolygon(geometry, operationArea, "operation area");

            OccupancyGrid grid(geometry, CellState::Occupied);
            for (int row = 0; row < geometry.height(); ++row)
            {
                for (int column = 0; column < geometry.width(); ++column)
                {
                    const cv::Point pixel = geometry.cellToImage({ column, row });
                    if (cellIsContainedByPolygon(pixel.x, pixel.y, imageArea))
                        grid.set({ column, row }, CellState::Free);
                }
            }

            for (const Polygon& obstacle : obstacles)
                markIntersectingCells(grid, obstacle, CellState::Occupied);
            return grid;
        }

        /**
         * Builds a world-aligned master map covering the complete operation area.
         */
        static OccupancyGrid rasterize(
            const Polygon& operationArea,
            const std::vector<Polygon>& obstacles,
            double resolution,
            const Point2& alignmentAnchor = Point2::Zero())
        {
            const GridGeometry geometry = GridGeometry::alignedCovering(
                operationArea, resolution, alignmentAnchor);
            return rasterize(geometry, operationArea, obstacles);
        }

    private:
        static std::vector<cv::Point2d> checkedImagePolygon(
            const GridGeometry& geometry,
            const Polygon& polygon,
            const char* description)
        {
            if (polygon.size() < 3)
            {
                throw std::invalid_argument(
                    std::string("PolygonRasterizer: ") + description +
                    " must contain at least three vertices");
            }
            std::vector<cv::Point2d> imagePolygon = geometry.worldToImage(polygon);
            for (const cv::Point2d& point : imagePolygon)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                {
                    throw std::invalid_argument(
                        std::string("PolygonRasterizer: ") + description +
                        " cannot be represented in image coordinates");
                }
            }
            return imagePolygon;
        }

        static double cross(
            const cv::Point2d& a,
            const cv::Point2d& b,
            const cv::Point2d& c) noexcept
        {
            return (b.x - a.x) * (c.y - a.y) -
                   (b.y - a.y) * (c.x - a.x);
        }

        static double tolerance(
            const cv::Point2d& a,
            const cv::Point2d& b,
            const cv::Point2d& c) noexcept
        {
            const double scale = std::max({
                1.0,
                std::abs(a.x), std::abs(a.y),
                std::abs(b.x), std::abs(b.y),
                std::abs(c.x), std::abs(c.y)
            });
            return 1e-12 * scale * scale;
        }

        static bool pointOnSegment(
            const cv::Point2d& point,
            const cv::Point2d& a,
            const cv::Point2d& b) noexcept
        {
            if (std::abs(cross(a, b, point)) > tolerance(a, b, point))
                return false;
            const double coordinateTolerance = 1e-12 * std::max({
                1.0,
                std::abs(a.x), std::abs(a.y),
                std::abs(b.x), std::abs(b.y),
                std::abs(point.x), std::abs(point.y)
            });
            return point.x >= std::min(a.x, b.x) - coordinateTolerance &&
                   point.x <= std::max(a.x, b.x) + coordinateTolerance &&
                   point.y >= std::min(a.y, b.y) - coordinateTolerance &&
                   point.y <= std::max(a.y, b.y) + coordinateTolerance;
        }

        static bool pointInOrOnPolygon(
            const cv::Point2d& point,
            const std::vector<cv::Point2d>& polygon) noexcept
        {
            bool inside = false;
            for (std::size_t index = 0, previous = polygon.size() - 1;
                 index < polygon.size(); previous = index++)
            {
                const cv::Point2d& a = polygon[previous];
                const cv::Point2d& b = polygon[index];
                if (pointOnSegment(point, a, b))
                    return true;

                const bool crossesScanline = (a.y > point.y) != (b.y > point.y);
                if (crossesScanline)
                {
                    const double intersectionX =
                        a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
                    if (intersectionX > point.x)
                        inside = !inside;
                }
            }
            return inside;
        }

        static int orientation(
            const cv::Point2d& a,
            const cv::Point2d& b,
            const cv::Point2d& c) noexcept
        {
            const double value = cross(a, b, c);
            const double allowed = tolerance(a, b, c);
            return value > allowed ? 1 : (value < -allowed ? -1 : 0);
        }

        static bool segmentsIntersect(
            const cv::Point2d& a,
            const cv::Point2d& b,
            const cv::Point2d& c,
            const cv::Point2d& d) noexcept
        {
            const int abc = orientation(a, b, c);
            const int abd = orientation(a, b, d);
            const int cda = orientation(c, d, a);
            const int cdb = orientation(c, d, b);

            if (abc * abd < 0 && cda * cdb < 0)
                return true;
            return (abc == 0 && pointOnSegment(c, a, b)) ||
                   (abd == 0 && pointOnSegment(d, a, b)) ||
                   (cda == 0 && pointOnSegment(a, c, d)) ||
                   (cdb == 0 && pointOnSegment(b, c, d));
        }

        static std::vector<cv::Point2d> cellCorners(int imageX, int imageY)
        {
            return {
                { imageX - 0.5, imageY - 0.5 },
                { imageX + 0.5, imageY - 0.5 },
                { imageX + 0.5, imageY + 0.5 },
                { imageX - 0.5, imageY + 0.5 }
            };
        }

        static bool cellIsContainedByPolygon(
            int imageX,
            int imageY,
            const std::vector<cv::Point2d>& polygon)
        {
            const auto corners = cellCorners(imageX, imageY);
            return std::all_of(
                corners.begin(), corners.end(),
                [&polygon](const cv::Point2d& corner)
                {
                    return pointInOrOnPolygon(corner, polygon);
                });
        }

        static bool cellIntersectsPolygon(
            int imageX,
            int imageY,
            const std::vector<cv::Point2d>& polygon)
        {
            const auto corners = cellCorners(imageX, imageY);
            for (const cv::Point2d& corner : corners)
            {
                if (pointInOrOnPolygon(corner, polygon))
                    return true;
            }

            const double minimumX = imageX - 0.5;
            const double maximumX = imageX + 0.5;
            const double minimumY = imageY - 0.5;
            const double maximumY = imageY + 0.5;
            for (const cv::Point2d& vertex : polygon)
            {
                if (vertex.x >= minimumX && vertex.x <= maximumX &&
                    vertex.y >= minimumY && vertex.y <= maximumY)
                {
                    return true;
                }
            }

            for (std::size_t polygonIndex = 0; polygonIndex < polygon.size(); ++polygonIndex)
            {
                const cv::Point2d& a = polygon[polygonIndex];
                const cv::Point2d& b = polygon[(polygonIndex + 1) % polygon.size()];
                for (std::size_t cellIndex = 0; cellIndex < corners.size(); ++cellIndex)
                {
                    const cv::Point2d& c = corners[cellIndex];
                    const cv::Point2d& d = corners[(cellIndex + 1) % corners.size()];
                    if (segmentsIntersect(a, b, c, d))
                        return true;
                }
            }
            return false;
        }

        static void markIntersectingCells(
            OccupancyGrid& grid,
            const Polygon& polygon,
            CellState state)
        {
            const std::vector<cv::Point2d> imagePolygon =
                checkedImagePolygon(grid.geometry(), polygon, "obstacle");

            double minimumX = imagePolygon.front().x;
            double maximumX = minimumX;
            double minimumY = imagePolygon.front().y;
            double maximumY = minimumY;
            for (const cv::Point2d& point : imagePolygon)
            {
                minimumX = std::min(minimumX, point.x);
                maximumX = std::max(maximumX, point.x);
                minimumY = std::min(minimumY, point.y);
                maximumY = std::max(maximumY, point.y);
            }

            const int firstColumn = firstCandidateIndex(minimumX, grid.width());
            const int lastColumn = lastCandidateIndex(maximumX, grid.width());
            const int firstImageRow = firstCandidateIndex(minimumY, grid.height());
            const int lastImageRow = lastCandidateIndex(maximumY, grid.height());

            for (int imageRow = firstImageRow; imageRow <= lastImageRow; ++imageRow)
            {
                for (int column = firstColumn; column <= lastColumn; ++column)
                {
                    if (!cellIntersectsPolygon(column, imageRow, imagePolygon))
                        continue;
                    const GridCell cell{ column, grid.height() - 1 - imageRow };
                    grid.set(cell, state);
                }
            }
        }

        static int firstCandidateIndex(double minimum, int dimension) noexcept
        {
            const double candidate = std::ceil(minimum - 0.5);
            if (candidate <= 0.0)
                return 0;
            if (candidate >= static_cast<double>(dimension))
                return dimension;
            return static_cast<int>(candidate);
        }

        static int lastCandidateIndex(double maximum, int dimension) noexcept
        {
            const double candidate = std::floor(maximum + 0.5);
            if (candidate < 0.0)
                return -1;
            if (candidate >= static_cast<double>(dimension - 1))
                return dimension - 1;
            return static_cast<int>(candidate);
        }
    };
}
