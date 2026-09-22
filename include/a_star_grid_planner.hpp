/*
 * a_star_grid_planner.hpp - Grid-planning entry point for AStarPlanner.
 */
#pragma once

#include "occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace astar
{
    enum class GridPlanStatus
    {
        Success,
        StartOutsideGrid,
        GoalOutsideGrid,
        StartOccupied,
        GoalOccupied,
        NoPath
    };

    inline const char* gridPlanStatusName(GridPlanStatus status) noexcept
    {
        switch (status)
        {
        case GridPlanStatus::Success: return "success";
        case GridPlanStatus::StartOutsideGrid: return "start is outside the grid";
        case GridPlanStatus::GoalOutsideGrid: return "goal is outside the grid";
        case GridPlanStatus::StartOccupied: return "start cell is occupied";
        case GridPlanStatus::GoalOccupied: return "goal cell is occupied";
        case GridPlanStatus::NoPath: return "no path exists";
        }
        return "unknown planning status";
    }

    enum class GridConnectivity
    {
        FourConnected
    };

    /** Search settings. Additional movement models can be added without changing plan(). */
    struct AStarOptions
    {
        GridConnectivity connectivity = GridConnectivity::FourConnected;
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
     * The initial implementation uses four-connected, unit-cost movement and a
     * Manhattan-distance heuristic. Returned paths include both terminal cells.
     */
    class AStarGridPlanner
    {
    public:
        GridPlanResult plan(
            const OccupancyGrid& grid,
            const GridCell& start,
            const GridCell& goal,
            const AStarOptions& options = {}) const
        {
            if (!grid.contains(start))
                return { GridPlanStatus::StartOutsideGrid, {} };
            if (!grid.contains(goal))
                return { GridPlanStatus::GoalOutsideGrid, {} };
            if (!grid.isTraversable(start))
                return { GridPlanStatus::StartOccupied, {} };
            if (!grid.isTraversable(goal))
                return { GridPlanStatus::GoalOccupied, {} };

            const std::size_t startIndex = cellIndex(grid, start);
            const std::size_t goalIndex = cellIndex(grid, goal);
            if (startIndex == goalIndex)
                return { GridPlanStatus::Success, { start } };

            const std::size_t cellCount =
                static_cast<std::size_t>(grid.width()) *
                static_cast<std::size_t>(grid.height());
            const std::size_t unreachable = std::numeric_limits<std::size_t>::max();
            std::vector<std::size_t> costs(cellCount, unreachable);
            std::vector<std::size_t> parents(cellCount, unreachable);
            std::priority_queue<OpenNode, std::vector<OpenNode>, LowerCostFirst> open;

            costs[startIndex] = 0;
            open.push({ manhattanDistance(start, goal), 0, 0, startIndex });

            while (!open.empty())
            {
                const OpenNode current = open.top();
                open.pop();
                if (current.costFromStart != costs[current.index])
                    continue;
                if (current.index == goalIndex)
                {
                    return {
                        GridPlanStatus::Success,
                        reconstructPath(grid, parents, startIndex, goalIndex)
                    };
                }

                const GridCell currentCell = indexCell(grid, current.index);
                for (const GridCell& neighbor : neighbors(currentCell, options))
                {
                    if (!grid.isTraversable(neighbor))
                        continue;

                    const std::size_t neighborIndex = cellIndex(grid, neighbor);
                    const std::size_t candidateCost = current.costFromStart + 1;
                    if (candidateCost >= costs[neighborIndex])
                        continue;

                    costs[neighborIndex] = candidateCost;
                    parents[neighborIndex] = current.index;
                    open.push({
                        candidateCost + manhattanDistance(neighbor, goal),
                        candidateCost,
                        lineDeviation(start, goal, neighbor),
                        neighborIndex
                    });
                }
            }

            return { GridPlanStatus::NoPath, {} };
        }

    private:
        struct OpenNode
        {
            std::size_t estimatedTotalCost;
            std::size_t costFromStart;
            std::uint64_t lineDeviation;
            std::size_t index;
        };

        struct LowerCostFirst
        {
            bool operator()(const OpenNode& lhs, const OpenNode& rhs) const noexcept
            {
                if (lhs.estimatedTotalCost != rhs.estimatedTotalCost)
                    return lhs.estimatedTotalCost > rhs.estimatedTotalCost;
                if (lhs.lineDeviation != rhs.lineDeviation)
                    return lhs.lineDeviation > rhs.lineDeviation;
                if (lhs.costFromStart != rhs.costFromStart)
                    return lhs.costFromStart < rhs.costFromStart;
                return lhs.index > rhs.index;
            }
        };

        static std::size_t cellIndex(
            const OccupancyGrid& grid,
            const GridCell& cell) noexcept
        {
            return static_cast<std::size_t>(cell.row) *
                static_cast<std::size_t>(grid.width()) +
                static_cast<std::size_t>(cell.column);
        }

        static GridCell indexCell(
            const OccupancyGrid& grid,
            std::size_t index) noexcept
        {
            const std::size_t width = static_cast<std::size_t>(grid.width());
            return {
                static_cast<int>(index % width),
                static_cast<int>(index / width)
            };
        }

        static std::size_t manhattanDistance(
            const GridCell& first,
            const GridCell& second) noexcept
        {
            const auto columnDistance = static_cast<std::size_t>(
                std::abs(first.column - second.column));
            const auto rowDistance = static_cast<std::size_t>(
                std::abs(first.row - second.row));
            return columnDistance + rowDistance;
        }

        static std::uint64_t lineDeviation(
            const GridCell& start,
            const GridCell& goal,
            const GridCell& cell) noexcept
        {
            const std::int64_t goalColumn =
                static_cast<std::int64_t>(goal.column) - start.column;
            const std::int64_t goalRow =
                static_cast<std::int64_t>(goal.row) - start.row;
            const std::int64_t cellColumn =
                static_cast<std::int64_t>(cell.column) - start.column;
            const std::int64_t cellRow =
                static_cast<std::int64_t>(cell.row) - start.row;
            const std::int64_t crossProduct =
                goalColumn * cellRow - goalRow * cellColumn;
            return static_cast<std::uint64_t>(
                crossProduct < 0 ? -crossProduct : crossProduct);
        }

        static std::array<GridCell, 4> neighbors(
            const GridCell& cell,
            const AStarOptions& options) noexcept
        {
            switch (options.connectivity)
            {
            case GridConnectivity::FourConnected:
                return { {
                    { cell.column + 1, cell.row },
                    { cell.column, cell.row + 1 },
                    { cell.column - 1, cell.row },
                    { cell.column, cell.row - 1 }
                } };
            }

            return { { cell, cell, cell, cell } };
        }

        static std::vector<GridCell> reconstructPath(
            const OccupancyGrid& grid,
            const std::vector<std::size_t>& parents,
            std::size_t startIndex,
            std::size_t goalIndex)
        {
            std::vector<GridCell> reversedPath;
            for (std::size_t index = goalIndex;; index = parents[index])
            {
                reversedPath.push_back(indexCell(grid, index));
                if (index == startIndex)
                    break;
            }
            std::reverse(reversedPath.begin(), reversedPath.end());
            return reversedPath;
        }
    };
}
