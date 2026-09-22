/*
 * occupancy_grid_visualization.hpp - Reusable OpenCV rendering for occupancy grids.
 */
#pragma once

#include "occupancy_grid.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace astar
{
    struct OccupancyGridMarker
    {
        GridCell cell;
        cv::Scalar color;
        int radiusPixels = 0;
    };

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

    /** Free cells render white and occupied cells render black. */
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
}
