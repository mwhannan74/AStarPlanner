/*
 * grid_line.hpp - Integer line rasterization for planning-grid cells.
 */
#pragma once

#include "occupancy_grid.hpp"

#include <cstdint>
#include <cstdlib>
#include <vector>

namespace astar
{
    /**
     * Rasterizes an ordered, one-cell-wide line with Bresenham's algorithm.
     * Both endpoints are included. This is not a conservative supercover line.
     */
    inline std::vector<GridCell> rasterizeGridLine(
        const GridCell& start,
        const GridCell& goal)
    {
        std::vector<GridCell> cells;

        GridCell current = start;
        const std::int64_t columnDistance =
            std::abs(static_cast<std::int64_t>(goal.column) - start.column);
        const std::int64_t rowDistance =
            -std::abs(static_cast<std::int64_t>(goal.row) - start.row);
        const int columnStep = start.column < goal.column ? 1 : -1;
        const int rowStep = start.row < goal.row ? 1 : -1;
        std::int64_t error = columnDistance + rowDistance;

        while (true)
        {
            cells.push_back(current);
            if (current == goal)
                break;

            const std::int64_t doubledError = 2 * error;
            if (doubledError >= rowDistance)
            {
                error += rowDistance;
                current.column += columnStep;
            }
            if (doubledError <= columnDistance)
            {
                error += columnDistance;
                current.row += rowStep;
            }
        }

        return cells;
    }
}
