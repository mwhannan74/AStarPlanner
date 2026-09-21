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

    struct OccupancyGridRenderOptions
    {
        int pixelsPerCell = 8;
        bool drawCellBorders = true;
        cv::Scalar cellBorderColor{ 210, 210, 210 };
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
        cv::bitwise_not(grid.image(), monochrome);

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

            const cv::Point pixel = grid.geometry().cellToImage(marker.cell);
            const cv::Point center(
                pixel.x * options.pixelsPerCell + options.pixelsPerCell / 2,
                pixel.y * options.pixelsPerCell + options.pixelsPerCell / 2);
            const int radius = marker.radiusPixels > 0
                ? marker.radiusPixels
                : std::max(2, options.pixelsPerCell / 3);
            cv::circle(
                display,
                center,
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
