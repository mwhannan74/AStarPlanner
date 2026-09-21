/*
 * a_star_grid_planner.hpp - Grid-planning entry point for AStarPlanner.
 */
#pragma once

#include "grid_line.hpp"

#include <vector>

namespace astar
{
    enum class GridPlanStatus
    {
        Success,
        StartOutsideGrid,
        GoalOutsideGrid
    };

    struct GridPlanResult
    {
        GridPlanStatus status;
        std::vector<GridCell> path;

        bool succeeded() const noexcept
        {
            return status == GridPlanStatus::Success;
        }
    };

    /**
     * Planning entry point for an occupancy grid.
     *
     * This bootstrap implementation validates the terminal cells and returns a
     * straight rasterized line. It intentionally does not inspect occupancy or
     * avoid obstacles yet; the implementation will be replaced by A* in a later
     * stage without changing the calling workflow.
     */
    class AStarGridPlanner
    {
    public:
        GridPlanResult plan(
            const OccupancyGrid& grid,
            const GridCell& start,
            const GridCell& goal) const
        {
            if (!grid.contains(start))
                return { GridPlanStatus::StartOutsideGrid, {} };
            if (!grid.contains(goal))
                return { GridPlanStatus::GoalOutsideGrid, {} };

            return {
                GridPlanStatus::Success,
                rasterizeGridLine(start, goal)
            };
        }
    };
}
