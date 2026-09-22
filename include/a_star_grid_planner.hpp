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
        FourConnected,
        EightConnected
    };

    /** Search settings. Additional movement models can be added without changing plan(). */
    struct AStarOptions
    {
        GridConnectivity connectivity = GridConnectivity::EightConnected;
        bool preventDiagonalCornerCutting = true;
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

    /** Converts an ordered grid path to the world positions of its cell centers. */
    inline std::vector<Point2> gridPathToWorld(
        const OccupancyGrid& grid,
        const std::vector<GridCell>& path)
    {
        std::vector<Point2> worldPath;
        worldPath.reserve(path.size());
        for (const GridCell& cell : path)
            worldPath.push_back(grid.geometry().cellCenterToWorld(cell));
        return worldPath;
    }

    /**
     * Planning entry point for an occupancy grid.
     *
     * Eight-connected movement is the default. Orthogonal moves cost 1,
     * diagonal moves cost sqrt(2), and the heuristic is selected to match the
     * configured connectivity. Returned paths include both terminal cells.
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
            const double unreachable = std::numeric_limits<double>::infinity();
            std::vector<double> costs(cellCount, unreachable);
            std::vector<std::size_t> parents(cellCount, NO_PARENT);
            std::priority_queue<OpenNode, std::vector<OpenNode>, LowerCostFirst> open;

            costs[startIndex] = 0;
            open.push({ heuristic(start, goal, options), 0.0, 0, startIndex });

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
                for (const NeighborOffset& offset : NEIGHBOR_OFFSETS)
                {
                    const bool diagonal = offset.column != 0 && offset.row != 0;
                    if (diagonal && options.connectivity != GridConnectivity::EightConnected)
                        continue;

                    const GridCell neighbor{
                        currentCell.column + offset.column,
                        currentCell.row + offset.row
                    };
                    if (!grid.isTraversable(neighbor))
                        continue;
                    if (diagonal && options.preventDiagonalCornerCutting &&
                        (!grid.isTraversable({ neighbor.column, currentCell.row }) ||
                         !grid.isTraversable({ currentCell.column, neighbor.row })))
                    {
                        continue;
                    }

                    const std::size_t neighborIndex = cellIndex(grid, neighbor);
                    const double candidateCost =
                        current.costFromStart + offset.movementCost;
                    if (candidateCost >= costs[neighborIndex])
                        continue;

                    costs[neighborIndex] = candidateCost;
                    parents[neighborIndex] = current.index;
                    open.push({
                        candidateCost + heuristic(neighbor, goal, options),
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
            double estimatedTotalCost;
            double costFromStart;
            std::uint64_t lineDeviation;
            std::size_t index;
        };

        struct NeighborOffset
        {
            int column;
            int row;
            double movementCost;
        };

        inline static constexpr double DIAGONAL_COST = 1.4142135623730950488;
        inline static constexpr std::size_t NO_PARENT =
            std::numeric_limits<std::size_t>::max();
        inline static constexpr std::array<NeighborOffset, 8> NEIGHBOR_OFFSETS{ {
            { 1, 0, 1.0 },
            { 1, 1, DIAGONAL_COST },
            { 0, 1, 1.0 },
            { -1, 1, DIAGONAL_COST },
            { -1, 0, 1.0 },
            { -1, -1, DIAGONAL_COST },
            { 0, -1, 1.0 },
            { 1, -1, DIAGONAL_COST }
        } };

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

        static double heuristic(
            const GridCell& first,
            const GridCell& second,
            const AStarOptions& options) noexcept
        {
            const auto columnDistance = static_cast<std::size_t>(
                std::abs(first.column - second.column));
            const auto rowDistance = static_cast<std::size_t>(
                std::abs(first.row - second.row));
            if (options.connectivity == GridConnectivity::EightConnected)
            {
                const std::size_t diagonalSteps =
                    std::min(columnDistance, rowDistance);
                const std::size_t orthogonalSteps =
                    std::max(columnDistance, rowDistance) - diagonalSteps;
                return static_cast<double>(orthogonalSteps) +
                    DIAGONAL_COST * static_cast<double>(diagonalSteps);
            }
            return static_cast<double>(columnDistance + rowDistance);
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
                if (parents[index] == NO_PARENT)
                {
                    throw std::logic_error(
                        "AStarGridPlanner: incomplete parent chain");
                }
            }
            std::reverse(reversedPath.begin(), reversedPath.end());
            return reversedPath;
        }
    };
}
