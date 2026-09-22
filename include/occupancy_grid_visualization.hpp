/*
 * occupancy_grid_visualization.hpp - Reusable OpenCV rendering for occupancy grids.
 */
#pragma once

#include "a_star_grid_planner.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace astar
{
    /** Marker colors use OpenCV's BGR cv::Scalar channel order. */
    struct OccupancyGridMarker
    {
        GridCell cell;
        cv::Scalar color;
        int radiusPixels = 0;
    };

    /** Path colors use OpenCV's BGR cv::Scalar channel order. */
    struct OccupancyGridPath
    {
        std::vector<GridCell> cells;
        cv::Scalar color;
        int thicknessPixels = 2;
    };

    struct OccupancyGridRenderOptions
    {
        int pixelsPerCell = 8;
        bool drawCellBorders = true;
        cv::Scalar cellBorderColor{ 210, 210, 210 };
        std::vector<OccupancyGridPath> paths;
        std::vector<OccupancyGridMarker> markers;
    };

    /** Colors use OpenCV's BGR cv::Scalar channel order. */
    struct GridSearchStateRenderOptions
    {
        int pixelsPerCell = 8;
        bool drawCellBorders = true;
        cv::Scalar cellBorderColor{ 210, 210, 210 };
        cv::Scalar unseenColor{ 255, 255, 255 };
        cv::Scalar openColor{ 0, 220, 255 };
        cv::Scalar currentColor{ 0, 100, 255 };
        cv::Scalar expandedColor{ 255, 210, 120 };
        cv::Scalar pathColor{ 255, 120, 0 };
        cv::Scalar occupiedColor{ 0, 0, 0 };
        cv::Scalar startColor{ 0, 180, 0 };
        cv::Scalar goalColor{ 0, 0, 255 };
        int terminalRadiusPixels = 0;
    };

    /**
     * Renders free cells white and occupied cells black in a BGR image.
     * Overlay colors use OpenCV's BGR cv::Scalar channel order.
     */
    inline cv::Mat3b renderOccupancyGrid(
        const OccupancyGrid& grid,
        const OccupancyGridRenderOptions& options = {})
    {
        if (options.pixelsPerCell <= 0)
        {
            throw std::invalid_argument(
                "renderOccupancyGrid: pixelsPerCell must be positive");
        }
        if (grid.width() > std::numeric_limits<int>::max() / options.pixelsPerCell ||
            grid.height() > std::numeric_limits<int>::max() / options.pixelsPerCell)
        {
            throw std::invalid_argument(
                "renderOccupancyGrid: rendered image dimensions are too large");
        }

        cv::Mat1b monochrome;
        cv::bitwise_not(grid.imageView(), monochrome);

        cv::Mat3b display;
        cv::cvtColor(monochrome, display, cv::COLOR_GRAY2BGR);
        cv::resize(
            display,
            display,
            cv::Size(
                grid.width() * options.pixelsPerCell,
                grid.height() * options.pixelsPerCell),
            0.0,
            0.0,
            cv::INTER_NEAREST);

        if (options.drawCellBorders && options.pixelsPerCell >= 4)
        {
            for (int column = 0; column <= grid.width(); ++column)
            {
                const int x = std::min(
                    column * options.pixelsPerCell, display.cols - 1);
                cv::line(
                    display,
                    { x, 0 },
                    { x, display.rows - 1 },
                    options.cellBorderColor,
                    1);
            }
            for (int row = 0; row <= grid.height(); ++row)
            {
                const int y = std::min(
                    row * options.pixelsPerCell, display.rows - 1);
                cv::line(
                    display,
                    { 0, y },
                    { display.cols - 1, y },
                    options.cellBorderColor,
                    1);
            }
        }

        const auto cellCenter = [&grid, &options](const GridCell& cell)
        {
            const cv::Point pixel = grid.geometry().cellToImage(cell);
            return cv::Point(
                pixel.x * options.pixelsPerCell + options.pixelsPerCell / 2,
                pixel.y * options.pixelsPerCell + options.pixelsPerCell / 2);
        };

        for (const OccupancyGridPath& path : options.paths)
        {
            if (path.thicknessPixels <= 0)
            {
                throw std::invalid_argument(
                    "renderOccupancyGrid: path thickness must be positive");
            }

            std::vector<cv::Point> imagePath;
            imagePath.reserve(path.cells.size());
            for (const GridCell& cell : path.cells)
            {
                if (!grid.contains(cell))
                {
                    throw std::out_of_range(
                        "renderOccupancyGrid: path cell is outside the grid");
                }
                imagePath.push_back(cellCenter(cell));
            }

            if (imagePath.size() >= 2)
            {
                cv::polylines(
                    display,
                    imagePath,
                    false,
                    path.color,
                    path.thicknessPixels,
                    cv::LINE_AA);
            }
            else if (imagePath.size() == 1)
            {
                cv::circle(
                    display,
                    imagePath.front(),
                    std::max(1, path.thicknessPixels / 2),
                    path.color,
                    cv::FILLED,
                    cv::LINE_AA);
            }
        }

        for (const OccupancyGridMarker& marker : options.markers)
        {
            if (!grid.contains(marker.cell))
            {
                throw std::out_of_range(
                    "renderOccupancyGrid: marker cell is outside the grid");
            }
            if (marker.radiusPixels < 0)
            {
                throw std::invalid_argument(
                    "renderOccupancyGrid: marker radius cannot be negative");
            }

            const int radius = marker.radiusPixels > 0
                ? marker.radiusPixels
                : std::max(2, options.pixelsPerCell / 3);
            cv::circle(
                display,
                cellCenter(marker.cell),
                radius,
                marker.color,
                cv::FILLED,
                cv::LINE_AA);
        }

        return display;
    }

    inline void showOccupancyGrid(
        const OccupancyGrid& grid,
        const OccupancyGridRenderOptions& options = {},
        const std::string& windowName = "Occupancy Grid")
    {
        cv::imshow(windowName, renderOccupancyGrid(grid, options));
    }

    /** Renders one planner debug-state matrix as a viewable BGR image. */
    inline cv::Mat3b renderGridSearchState(
        const OccupancyGrid& grid,
        const cv::Mat1b& state,
        const GridCell& start,
        const GridCell& goal,
        const GridSearchStateRenderOptions& options = {})
    {
        if (state.rows != grid.height() || state.cols != grid.width())
        {
            throw std::invalid_argument(
                "renderGridSearchState: state dimensions must match the grid");
        }
        if (!grid.contains(start) || !grid.contains(goal))
        {
            throw std::out_of_range(
                "renderGridSearchState: terminal cell is outside the grid");
        }
        if (options.pixelsPerCell <= 0 || options.terminalRadiusPixels < 0)
        {
            throw std::invalid_argument(
                "renderGridSearchState: render sizes must be nonnegative and pixelsPerCell positive");
        }
        if (grid.width() > std::numeric_limits<int>::max() / options.pixelsPerCell ||
            grid.height() > std::numeric_limits<int>::max() / options.pixelsPerCell)
        {
            throw std::invalid_argument(
                "renderGridSearchState: rendered image dimensions are too large");
        }

        cv::Mat3b cellColors(grid.height(), grid.width());
        for (int row = 0; row < state.rows; ++row)
        {
            for (int column = 0; column < state.cols; ++column)
            {
                const auto cellState = static_cast<GridSearchCellState>(
                    state(row, column));
                cv::Scalar color;
                switch (cellState)
                {
                case GridSearchCellState::Unseen: color = options.unseenColor; break;
                case GridSearchCellState::Open: color = options.openColor; break;
                case GridSearchCellState::Current: color = options.currentColor; break;
                case GridSearchCellState::Expanded: color = options.expandedColor; break;
                case GridSearchCellState::Path: color = options.pathColor; break;
                case GridSearchCellState::Occupied: color = options.occupiedColor; break;
                default:
                    throw std::invalid_argument(
                        "renderGridSearchState: state image contains an unsupported value");
                }
                cellColors(row, column) = cv::Vec3b(
                    cv::saturate_cast<std::uint8_t>(color[0]),
                    cv::saturate_cast<std::uint8_t>(color[1]),
                    cv::saturate_cast<std::uint8_t>(color[2]));
            }
        }

        cv::Mat3b display;
        cv::resize(
            cellColors,
            display,
            cv::Size(
                grid.width() * options.pixelsPerCell,
                grid.height() * options.pixelsPerCell),
            0.0,
            0.0,
            cv::INTER_NEAREST);

        if (options.drawCellBorders && options.pixelsPerCell >= 4)
        {
            for (int column = 0; column <= grid.width(); ++column)
            {
                const int x = std::min(
                    column * options.pixelsPerCell, display.cols - 1);
                cv::line(
                    display,
                    { x, 0 },
                    { x, display.rows - 1 },
                    options.cellBorderColor,
                    1);
            }
            for (int row = 0; row <= grid.height(); ++row)
            {
                const int y = std::min(
                    row * options.pixelsPerCell, display.rows - 1);
                cv::line(
                    display,
                    { 0, y },
                    { display.cols - 1, y },
                    options.cellBorderColor,
                    1);
            }
        }

        const auto cellCenter = [&grid, &options](const GridCell& cell)
        {
            const cv::Point pixel = grid.geometry().cellToImage(cell);
            return cv::Point(
                pixel.x * options.pixelsPerCell + options.pixelsPerCell / 2,
                pixel.y * options.pixelsPerCell + options.pixelsPerCell / 2);
        };
        const int terminalRadius = options.terminalRadiusPixels > 0
            ? options.terminalRadiusPixels
            : std::max(2, options.pixelsPerCell / 3);
        cv::circle(
            display,
            cellCenter(start),
            terminalRadius,
            options.startColor,
            cv::FILLED,
            cv::LINE_AA);
        cv::circle(
            display,
            cellCenter(goal),
            terminalRadius,
            options.goalColor,
            cv::FILLED,
            cv::LINE_AA);
        return display;
    }

    inline void showGridSearchState(
        const OccupancyGrid& grid,
        const cv::Mat1b& state,
        const GridCell& start,
        const GridCell& goal,
        const GridSearchStateRenderOptions& options = {},
        const std::string& windowName = "Grid Search Debug")
    {
        cv::imshow(
            windowName,
            renderGridSearchState(grid, state, start, goal, options));
    }
}
