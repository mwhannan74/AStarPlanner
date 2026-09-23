/*
 * a_star_grid_planner.hpp - Grid-planning entry point for AStarPlanner.
 */
#pragma once

#include "occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace astar
{
    enum class GridPlanStatus
    {
        /** Path contains the requested start and goal cells. */
        Success,
        /** Start cell is outside the supplied planning grid. */
        StartOutsideGrid,
        /** Goal cell is outside the supplied planning grid. */
        GoalOutsideGrid,
        /** Start cell is occupied. */
        StartOccupied,
        /** Goal cell is occupied. */
        GoalOccupied,
        /** No path exists inside the supplied planning grid. */
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
        /** Orthogonal movement only, with unit step cost. */
        FourConnected,
        /** Orthogonal and diagonal movement, with diagonal cost sqrt(2). */
        EightConnected
    };

    enum class GridSearchAlgorithm
    {
        /** A* search using the connectivity-matched admissible heuristic. */
        AStar,
        /** Dijkstra search using a zero heuristic. */
        Dijkstra,
        /** Weighted A* search using a caller-selected heuristic weight. */
        WeightedAStar
    };

    inline const char* gridSearchAlgorithmName(
        GridSearchAlgorithm algorithm) noexcept
    {
        switch (algorithm)
        {
        case GridSearchAlgorithm::AStar: return "A*";
        case GridSearchAlgorithm::Dijkstra: return "Dijkstra";
        case GridSearchAlgorithm::WeightedAStar: return "Weighted A*";
        }
        return "unknown grid search algorithm";
    }

    /** Controls the search algorithm and movement model. */
    struct AStarOptions
    {
        GridSearchAlgorithm algorithm = GridSearchAlgorithm::AStar;
        /** Applies only to WeightedAStar and must be finite and at least 1. */
        double heuristicWeight = 1.0;
        GridConnectivity connectivity = GridConnectivity::EightConnected;
        /** Applies only to diagonal moves in an eight-connected search. */
        bool preventDiagonalCornerCutting = true;
        /** Records per-cell expansion counts for diagnostics at additional cost. */
        bool collectDetailedDiagnostics = false;
    };

    /** Search measurements collected by a planning request. */
    struct GridPlanDiagnostics
    {
        /** Final movement cost, or infinity when planning does not succeed. */
        double pathCost = std::numeric_limits<double>::infinity();
        /** Non-stale open-set entries removed for processing, including the goal. */
        std::size_t expandedNodes = 0;
        /** Open-set entries inserted, including the start and replacement entries. */
        std::size_t generatedNodes = 0;
        /** Largest number of entries held by the open set at one time. */
        std::size_t peakOpenSetSize = 0;
    };

    /** Optional detailed search measurements requested through AStarOptions. */
    struct GridSearchDetailedDiagnostics
    {
        /** Expansion count per cell in OpenCV image coordinates. */
        cv::Mat1i expansionCounts;
        /** Number of different cells expanded at least once. */
        std::size_t uniqueExpandedCells = 0;
        /** Expansion events after a cell's first expansion. */
        std::size_t repeatedExpansions = 0;
        /** Queue entries discarded because a better cost was already recorded. */
        std::size_t staleOpenSetEntries = 0;
        /** Largest expansion count recorded for one cell. */
        std::size_t maximumExpansionsPerCell = 0;
    };

    /** Planning outcome. Failed results contain an empty path and infinite path cost. */
    struct GridPlanResult
    {
        GridPlanStatus status = GridPlanStatus::NoPath;
        std::vector<GridCell> path;
        GridPlanDiagnostics diagnostics;
        /** Present only when collectDetailedDiagnostics is enabled. */
        std::optional<GridSearchDetailedDiagnostics> detailedDiagnostics;

        bool succeeded() const noexcept
        {
            return status == GridPlanStatus::Success;
        }
    };

    /** Values stored in the optional single-channel search-debug image. */
    enum class GridSearchCellState : std::uint8_t
    {
        Unseen = 0,
        Open = 1,
        Current = 2,
        Expanded = 3,
        Path = 4,
        Occupied = 255
    };

    /**
     * Synchronous view of the planner's reusable debug-state image.
     *
     * The image is valid only for the duration of the callback and is modified
     * after the callback returns. Clone it inside the callback when a frame must
     * be retained or transferred to another thread.
     */
    using GridSearchDebugCallback = std::function<void(const cv::Mat1b&)>;

    /**
     * Planning entry point for an occupancy grid.
     *
     * A* with eight-connected movement is the default. All algorithms use the
     * same graph search and priority equation g + weight * h. Dijkstra selects
     * weight 0, A* selects weight 1, and WeightedAStar uses heuristicWeight.
     * Orthogonal moves cost 1, diagonal moves cost sqrt(2), and the base heuristic
     * matches the configured connectivity. Diagonal moves cannot pass between
     * occupied orthogonal neighbors unless corner cutting is enabled. Returned
     * paths include both terminal cells and use deterministic straight-line
     * deviation as a tie-breaker between equal-cost candidates. Weighted A* may
     * return a non-optimal path in exchange for reducing search effort.
     *
     * Supplying @p debugCallback enables a reusable single-channel state image
     * and synchronous callbacks after each accepted expansion and after the
     * final path or exhausted search. An empty callback allocates no debug image.
     * Enabling AStarOptions::collectDetailedDiagnostics separately records a
     * per-cell expansion-count image and detailed counters in the result. This
     * allocation and tracking are disabled by default.
     *
     * @throws std::invalid_argument if options contain an unsupported search
     * algorithm or connectivity value.
     */
    class AStarGridPlanner
    {
    public:
        GridPlanResult plan(
            const OccupancyGrid& grid,
            const GridCell& start,
            const GridCell& goal,
            const AStarOptions& options = {},
            const GridSearchDebugCallback& debugCallback = {}) const
        {
            const double heuristicWeight = effectiveHeuristicWeight(options);
            if (options.connectivity != GridConnectivity::FourConnected &&
                options.connectivity != GridConnectivity::EightConnected)
            {
                throw std::invalid_argument(
                    "AStarGridPlanner: unsupported grid connectivity");
            }

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

            const std::size_t cellCount =
                static_cast<std::size_t>(grid.width()) *
                static_cast<std::size_t>(grid.height());
            const double unreachable = std::numeric_limits<double>::infinity();
            std::vector<double> costs(cellCount, unreachable);
            std::vector<std::size_t> parents(cellCount, NO_PARENT);
            std::priority_queue<OpenNode, std::vector<OpenNode>, LowerCostFirst> open;
            GridPlanDiagnostics diagnostics;
            std::optional<GridSearchDetailedDiagnostics> detailedDiagnostics;
            if (options.collectDetailedDiagnostics)
            {
                detailedDiagnostics.emplace();
                detailedDiagnostics->expansionCounts = cv::Mat1i(
                    grid.height(), grid.width(), 0);
            }
            const bool debugEnabled = static_cast<bool>(debugCallback);
            cv::Mat1b debugState;
            if (debugEnabled)
            {
                debugState = cv::Mat1b(
                    grid.height(),
                    grid.width(),
                    static_cast<std::uint8_t>(GridSearchCellState::Unseen));
                debugState.setTo(
                    static_cast<std::uint8_t>(GridSearchCellState::Occupied),
                    grid.imageView());
            }

            const auto setDebugState = [&grid, &debugState](
                const GridCell& cell,
                GridSearchCellState state)
            {
                const cv::Point pixel = grid.geometry().cellToImage(cell);
                debugState(pixel.y, pixel.x) = static_cast<std::uint8_t>(state);
            };

            costs[startIndex] = 0;
            open.push({
                heuristicWeight * heuristic(start, goal, options.connectivity),
                0.0,
                0,
                startIndex
            });
            diagnostics.generatedNodes = 1;
            diagnostics.peakOpenSetSize = 1;
            if (debugEnabled)
                setDebugState(start, GridSearchCellState::Open);

            while (!open.empty())
            {
                const OpenNode current = open.top();
                open.pop();
                if (current.costFromStart != costs[current.index])
                {
                    if (detailedDiagnostics)
                        ++detailedDiagnostics->staleOpenSetEntries;
                    continue;
                }
                ++diagnostics.expandedNodes;
                const GridCell currentCell = indexCell(grid, current.index);
                if (detailedDiagnostics)
                {
                    const cv::Point pixel = grid.geometry().cellToImage(currentCell);
                    int& expansionCount =
                        detailedDiagnostics->expansionCounts(pixel.y, pixel.x);
                    if (expansionCount == 0)
                        ++detailedDiagnostics->uniqueExpandedCells;
                    else
                        ++detailedDiagnostics->repeatedExpansions;
                    ++expansionCount;
                    detailedDiagnostics->maximumExpansionsPerCell = std::max(
                        detailedDiagnostics->maximumExpansionsPerCell,
                        static_cast<std::size_t>(expansionCount));
                }
                if (debugEnabled)
                    setDebugState(currentCell, GridSearchCellState::Current);
                if (current.index == goalIndex)
                {
                    diagnostics.pathCost = current.costFromStart;
                    std::vector<GridCell> path = reconstructPath(
                        grid, parents, startIndex, goalIndex);
                    if (debugEnabled)
                    {
                        debugCallback(debugState);
                        for (const GridCell& pathCell : path)
                            setDebugState(pathCell, GridSearchCellState::Path);
                        debugCallback(debugState);
                    }
                    return {
                        GridPlanStatus::Success,
                        std::move(path),
                        diagnostics,
                        std::move(detailedDiagnostics)
                    };
                }

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
                        candidateCost + heuristicWeight *
                            heuristic(neighbor, goal, options.connectivity),
                        candidateCost,
                        lineDeviation(start, goal, neighbor),
                        neighborIndex
                    });
                    ++diagnostics.generatedNodes;
                    diagnostics.peakOpenSetSize = std::max(
                        diagnostics.peakOpenSetSize, open.size());
                    if (debugEnabled)
                        setDebugState(neighbor, GridSearchCellState::Open);
                }
                if (debugEnabled)
                {
                    debugCallback(debugState);
                    setDebugState(currentCell, GridSearchCellState::Expanded);
                }
            }

            if (debugEnabled)
                debugCallback(debugState);
            return {
                GridPlanStatus::NoPath,
                {},
                diagnostics,
                std::move(detailedDiagnostics)
            };
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

        static double effectiveHeuristicWeight(const AStarOptions& options)
        {
            switch (options.algorithm)
            {
            case GridSearchAlgorithm::Dijkstra:
                return 0.0;
            case GridSearchAlgorithm::AStar:
                return 1.0;
            case GridSearchAlgorithm::WeightedAStar:
                if (!std::isfinite(options.heuristicWeight) ||
                    options.heuristicWeight < 1.0)
                {
                    throw std::invalid_argument(
                        "AStarGridPlanner: weighted A* heuristic weight must be finite and at least 1");
                }
                return options.heuristicWeight;
            }
            throw std::invalid_argument(
                "AStarGridPlanner: unsupported search algorithm");
        }

        static double heuristic(
            const GridCell& first,
            const GridCell& second,
            GridConnectivity connectivity) noexcept
        {
            const auto columnDistance = static_cast<std::size_t>(
                std::abs(first.column - second.column));
            const auto rowDistance = static_cast<std::size_t>(
                std::abs(first.row - second.row));
            if (connectivity == GridConnectivity::EightConnected)
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
