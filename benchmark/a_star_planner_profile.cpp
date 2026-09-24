/*
 * a_star_planner_profile.cpp - Isolated CPU-profiling workload for A* search.
 */

#include "a_star_grid_planner.hpp"
#include "a_star_planner.hpp"
#include "occupancy_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace astar;
    using Clock = std::chrono::steady_clock;

    constexpr int GRID_SIZE = 1000;
    constexpr int DEFAULT_ITERATIONS = 50;
    constexpr int BARRIER_COUNT = 6;
    constexpr double CELL_RESOLUTION_METERS = 25.0;
    constexpr double SAFETY_RADIUS_METERS = CELL_RESOLUTION_METERS;

    struct ProfileConfiguration
    {
        int iterations = DEFAULT_ITERATIONS;
        AStarTieBreakPolicy tieBreakPolicy =
            AStarTieBreakPolicy::StraightLineThenLargerG;
    };

    Polygon rectangle(
        double minimumX,
        double minimumY,
        double maximumX,
        double maximumY)
    {
        return {
            Point2(minimumX, minimumY),
            Point2(maximumX, minimumY),
            Point2(maximumX, maximumY),
            Point2(minimumX, maximumY)
        };
    }

    OccupancyGrid makeAlternatingBarrierGrid()
    {
        const double extent =
            static_cast<double>(GRID_SIZE) * CELL_RESOLUTION_METERS;
        std::vector<Polygon> obstacles;
        obstacles.reserve(BARRIER_COUNT);
        const double wallHalfWidth = std::max(
            CELL_RESOLUTION_METERS, 0.005 * extent);
        for (int barrier = 0; barrier < BARRIER_COUNT; ++barrier)
        {
            const double x = extent * static_cast<double>(barrier + 1) /
                static_cast<double>(BARRIER_COUNT + 1);
            const bool gapAtTop = barrier % 2 == 0;
            obstacles.push_back(rectangle(
                x - wallHalfWidth,
                gapAtTop ? 0.0 : 0.25 * extent,
                x + wallHalfWidth,
                gapAtTop ? 0.75 * extent : extent));
        }

        const PolygonEnvironment environment(
            rectangle(0.0, 0.0, extent, extent), obstacles);
        const GridGeometry geometry(
            Point2::Zero(), CELL_RESOLUTION_METERS, GRID_SIZE, GRID_SIZE);
        const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
            geometry,
            environment.operationArea(),
            environment.effectiveObstacles());
        return OccupancyGridInflator::inflate(
            masterGrid.subgrid(masterGrid.fullRegion()),
            SAFETY_RADIUS_METERS);
    }

    AStarTieBreakPolicy parseTieBreakPolicy(const std::string& value)
    {
        if (value == "straight-line")
            return AStarTieBreakPolicy::StraightLineThenLargerG;
        if (value == "larger-g-then-line")
            return AStarTieBreakPolicy::LargerGThenStraightLine;
        if (value == "larger-g")
            return AStarTieBreakPolicy::LargerGOnly;
        throw std::invalid_argument(
            "--tie-break must be straight-line, larger-g-then-line, or larger-g");
    }

    ProfileConfiguration parseArguments(int argc, char** argv)
    {
        ProfileConfiguration configuration;
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--iterations" && index + 1 < argc)
            {
                const std::string value = argv[++index];
                std::size_t parsedCharacters = 0;
                const unsigned long parsed = std::stoul(value, &parsedCharacters);
                if (parsedCharacters != value.size() || parsed == 0 ||
                    parsed > static_cast<unsigned long>(
                        std::numeric_limits<int>::max()))
                {
                    throw std::invalid_argument(
                        "--iterations must be a positive integer");
                }
                configuration.iterations = static_cast<int>(parsed);
            }
            else if (argument == "--tie-break" && index + 1 < argc)
            {
                configuration.tieBreakPolicy = parseTieBreakPolicy(argv[++index]);
            }
            else if (argument == "--help")
            {
                std::cout
                    << "Usage: a_star_planner_profile [--iterations COUNT]"
                       " [--tie-break POLICY]\n"
                    << "Runs only A* on the 1000 x 1000 alternating-barrier "
                       "scenario.\n"
                    << "Tie-break policies: straight-line, larger-g-then-line,"
                       " larger-g. The default is straight-line.\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument(
                    "unknown or incomplete argument: " + argument);
            }
        }
        return configuration;
    }
}

int main(int argc, char** argv)
{
    try
    {
        const ProfileConfiguration configuration = parseArguments(argc, argv);
        const OccupancyGrid grid = makeAlternatingBarrierGrid();
        const double extent =
            static_cast<double>(GRID_SIZE) * CELL_RESOLUTION_METERS;
        const auto start = grid.geometry().worldToCell(
            Point2(3.5 * CELL_RESOLUTION_METERS, 3.5 * CELL_RESOLUTION_METERS));
        const auto goal = grid.geometry().worldToCell(
            Point2(
                extent - 3.5 * CELL_RESOLUTION_METERS,
                extent - 3.5 * CELL_RESOLUTION_METERS));
        if (!start || !goal)
            throw std::runtime_error("profile terminal is outside the grid");

        AStarOptions options;
        options.algorithm = GridSearchAlgorithm::AStar;
        options.connectivity = GridConnectivity::EightConnected;
        options.preventDiagonalCornerCutting = true;
        options.tieBreakPolicy = configuration.tieBreakPolicy;

        const AStarGridPlanner planner;
        const GridPlanResult expected = planner.plan(grid, *start, *goal, options);
        if (!expected.succeeded())
        {
            throw std::runtime_error(
                std::string("warm-up search failed: ") +
                gridPlanStatusName(expected.status));
        }

        std::size_t resultChecksum = 0;
        const auto startTime = Clock::now();
        for (int iteration = 0; iteration < configuration.iterations; ++iteration)
        {
            const GridPlanResult result = planner.plan(grid, *start, *goal, options);
            if (result.status != expected.status ||
                result.path != expected.path ||
                result.diagnostics.pathCost != expected.diagnostics.pathCost ||
                result.diagnostics.expandedNodes !=
                    expected.diagnostics.expandedNodes ||
                result.diagnostics.generatedNodes !=
                    expected.diagnostics.generatedNodes ||
                result.diagnostics.peakOpenSetSize !=
                    expected.diagnostics.peakOpenSetSize)
            {
                throw std::runtime_error(
                    "profile search did not reproduce the warm-up result");
            }
            resultChecksum += result.path.size();
            resultChecksum ^= result.diagnostics.expandedNodes;
        }
        const auto endTime = Clock::now();
        const double totalMilliseconds =
            std::chrono::duration<double, std::milli>(endTime - startTime).count();

        std::cout
            << "AStarPlanner isolated CPU profile workload\n"
            << "Scenario: " << GRID_SIZE << " x " << GRID_SIZE
            << " alternating barriers\n"
            << "Algorithm: A*\n"
            << "Tie-break: "
            << aStarTieBreakPolicyName(configuration.tieBreakPolicy) << '\n'
            << "Iterations: " << configuration.iterations
            << " after one warm-up\n"
            << std::fixed << std::setprecision(3)
            << "Total planning time: " << totalMilliseconds << " ms\n"
            << "Mean planning time: "
            << totalMilliseconds /
                static_cast<double>(configuration.iterations) << " ms\n"
            << "Expanded nodes per plan: "
            << expected.diagnostics.expandedNodes << '\n'
            << "Generated nodes per plan: "
            << expected.diagnostics.generatedNodes << '\n'
            << "Peak open-set size: "
            << expected.diagnostics.peakOpenSetSize << '\n'
            << "Path cells: " << expected.path.size() << '\n'
            << "Path cost: " << expected.diagnostics.pathCost << '\n'
            << "Result checksum: " << resultChecksum << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "A* profile workload failed: " << error.what() << '\n';
        return 1;
    }
}
