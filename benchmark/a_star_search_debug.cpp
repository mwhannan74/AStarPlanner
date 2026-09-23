/*
 * a_star_search_debug.cpp - Visual diagnosis of the 500 x 500 barrier benchmark.
 */

#include "a_star_grid_planner.hpp"
#include "a_star_planner.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_visualization.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace astar;
    using Clock = std::chrono::steady_clock;

    constexpr int GRID_CELLS = 500;
    constexpr double CELL_RESOLUTION_METERS = 25.0;
    constexpr double SAFETY_RADIUS_METERS = CELL_RESOLUTION_METERS;
    constexpr double DEFAULT_WEIGHT = 1.5;

    const cv::Scalar HEAT_NEVER{ 255, 255, 255 };
    const cv::Scalar HEAT_ONCE{ 220, 80, 20 };
    const cv::Scalar HEAT_TWO_TO_THREE{ 255, 220, 0 };
    const cv::Scalar HEAT_FOUR_TO_SEVEN{ 0, 200, 0 };
    const cv::Scalar HEAT_EIGHT_TO_FIFTEEN{ 0, 230, 255 };
    const cv::Scalar HEAT_SIXTEEN_TO_THIRTY_ONE{ 0, 120, 255 };
    const cv::Scalar HEAT_THIRTY_TWO_OR_MORE{ 0, 0, 220 };
    const cv::Scalar HEAT_OCCUPIED{ 0, 0, 0 };

    struct Configuration
    {
        GridSearchAlgorithm algorithm = GridSearchAlgorithm::AStar;
        double heuristicWeight = DEFAULT_WEIGHT;
        std::size_t frameStride = 1000;
        bool display = true;
        bool animate = true;
        std::string outputPath;
    };

    Polygon rectangle(double minimumX, double minimumY,
        double maximumX, double maximumY)
    {
        return {
            Point2(minimumX, minimumY),
            Point2(maximumX, minimumY),
            Point2(maximumX, maximumY),
            Point2(minimumX, maximumY)
        };
    }

    const char* algorithmFileName(GridSearchAlgorithm algorithm)
    {
        switch (algorithm)
        {
        case GridSearchAlgorithm::AStar: return "astar";
        case GridSearchAlgorithm::Dijkstra: return "dijkstra";
        case GridSearchAlgorithm::WeightedAStar: return "weighted_astar";
        }
        return "unknown";
    }

    GridSearchAlgorithm parseAlgorithm(const std::string& value)
    {
        if (value == "astar")
            return GridSearchAlgorithm::AStar;
        if (value == "dijkstra")
            return GridSearchAlgorithm::Dijkstra;
        if (value == "weighted")
            return GridSearchAlgorithm::WeightedAStar;
        throw std::invalid_argument(
            "algorithm must be astar, dijkstra, or weighted");
    }

    Configuration parseArguments(int argc, char** argv)
    {
        Configuration configuration;
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--algorithm" && index + 1 < argc)
            {
                configuration.algorithm = parseAlgorithm(argv[++index]);
            }
            else if (argument == "--weight" && index + 1 < argc)
            {
                configuration.heuristicWeight = std::stod(argv[++index]);
            }
            else if (argument == "--output" && index + 1 < argc)
            {
                configuration.outputPath = argv[++index];
            }
            else if (argument == "--frame-stride" && index + 1 < argc)
            {
                configuration.frameStride = std::stoull(argv[++index]);
                if (configuration.frameStride == 0)
                    throw std::invalid_argument("frame stride must be positive");
            }
            else if (argument == "--no-animation")
            {
                configuration.animate = false;
            }
            else if (argument == "--no-display")
            {
                configuration.display = false;
                configuration.animate = false;
            }
            else if (argument == "--help")
            {
                std::cout
                    << "Usage: a_star_search_debug [options]\n"
                    << "  --algorithm astar|dijkstra|weighted\n"
                    << "  --weight VALUE         Weighted-A* heuristic weight (default 1.5)\n"
                    << "  --output FILE          Final debug image path\n"
                    << "  --frame-stride COUNT   Expansions between animation frames (default 1000)\n"
                    << "  --no-animation         Show only the final frame\n"
                    << "  --no-display           Save without opening a window\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument("unknown or incomplete argument: " + argument);
            }
        }

        if (configuration.algorithm == GridSearchAlgorithm::WeightedAStar &&
            (!std::isfinite(configuration.heuristicWeight) ||
             configuration.heuristicWeight < 1.0))
        {
            throw std::invalid_argument(
                "weighted-A* weight must be finite and at least 1");
        }
        if (configuration.outputPath.empty())
        {
            configuration.outputPath = std::string("astar_search_debug_") +
                algorithmFileName(configuration.algorithm) + ".png";
        }
        return configuration;
    }

    std::vector<Polygon> makeAlternatingBarriers(double extent)
    {
        std::vector<Polygon> obstacles;
        constexpr int barrierCount = 6;
        const double wallHalfWidth = std::max(
            CELL_RESOLUTION_METERS, 0.005 * extent);
        for (int barrier = 0; barrier < barrierCount; ++barrier)
        {
            const double x = extent * static_cast<double>(barrier + 1) /
                static_cast<double>(barrierCount + 1);
            const bool gapAtTop = barrier % 2 == 0;
            obstacles.push_back(rectangle(
                x - wallHalfWidth,
                gapAtTop ? 0.0 : 0.25 * extent,
                x + wallHalfWidth,
                gapAtTop ? 0.75 * extent : extent));
        }
        return obstacles;
    }

    std::string scientificValue(double value)
    {
        if (!std::isfinite(value))
            return "none";
        std::ostringstream stream;
        stream << std::scientific << std::setprecision(6) << value;
        return stream.str();
    }

    cv::Mat3b renderExpansionHeatMap(
        const OccupancyGrid& grid,
        const GridSearchDetailedDiagnostics& diagnostics,
        int pixelsPerCell)
    {
        cv::Mat3b heatMap(
            grid.height(),
            grid.width(),
            cv::Vec3b(
                static_cast<std::uint8_t>(HEAT_NEVER[0]),
                static_cast<std::uint8_t>(HEAT_NEVER[1]),
                static_cast<std::uint8_t>(HEAT_NEVER[2])));
        const auto paintRange = [&](int minimum, int maximum, const cv::Scalar& color)
        {
            cv::Mat1b mask;
            cv::inRange(
                diagnostics.expansionCounts,
                cv::Scalar(minimum),
                cv::Scalar(maximum),
                mask);
            heatMap.setTo(color, mask);
        };
        paintRange(1, 1, HEAT_ONCE);
        paintRange(2, 3, HEAT_TWO_TO_THREE);
        paintRange(4, 7, HEAT_FOUR_TO_SEVEN);
        paintRange(8, 15, HEAT_EIGHT_TO_FIFTEEN);
        paintRange(16, 31, HEAT_SIXTEEN_TO_THIRTY_ONE);
        paintRange(
            32,
            std::numeric_limits<int>::max(),
            HEAT_THIRTY_TWO_OR_MORE);
        heatMap.setTo(HEAT_OCCUPIED, grid.imageView());

        cv::Mat3b display;
        cv::resize(
            heatMap,
            display,
            cv::Size(
                grid.width() * pixelsPerCell,
                grid.height() * pixelsPerCell),
            0.0,
            0.0,
            cv::INTER_NEAREST);
        return display;
    }

    cv::Mat3b addLegend(
        const cv::Mat3b& searchImage,
        const cv::Mat3b& expansionHeatMap,
        const GridSearchStateRenderOptions& colors,
        const AStarOptions& options,
        const GridPlanResult& result,
        double baselineMilliseconds)
    {
        constexpr int imageGap = 8;
        constexpr int panelWidth = 480;
        cv::Mat3b output(
            searchImage.rows,
            searchImage.cols + imageGap + expansionHeatMap.cols + panelWidth,
            cv::Vec3b(255, 255, 255));
        searchImage.copyTo(output(cv::Rect(
            0, 0, searchImage.cols, searchImage.rows)));
        expansionHeatMap.copyTo(output(cv::Rect(
            searchImage.cols + imageGap,
            0,
            expansionHeatMap.cols,
            expansionHeatMap.rows)));

        const int left =
            searchImage.cols + imageGap + expansionHeatMap.cols + 20;
        int y = 35;
        const auto text = [&](const std::string& value, int step = 27)
        {
            cv::putText(output, value, { left, y }, cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
            y += step;
        };
        text("500 x 500 alternating barriers", 28);
        text("Left: final state", 24);
        text("Right: expansion-count heat map", 30);
        text(std::string("Algorithm: ") + gridSearchAlgorithmName(options.algorithm));
        if (options.algorithm == GridSearchAlgorithm::WeightedAStar)
        {
            std::ostringstream weight;
            weight << "Heuristic weight: " << options.heuristicWeight;
            text(weight.str());
        }
        {
            std::ostringstream timing;
            timing << std::fixed << std::setprecision(3)
                   << "Baseline search: " << baselineMilliseconds << " ms";
            text(timing.str());
        }
        text("Expanded: " + std::to_string(result.diagnostics.expandedNodes));
        text("Generated: " + std::to_string(result.diagnostics.generatedNodes));
        text("Peak open set: " +
            std::to_string(result.diagnostics.peakOpenSetSize));
        text("Path cells: " + std::to_string(result.path.size()));
        if (result.detailedDiagnostics)
        {
            const GridSearchDetailedDiagnostics& detailed =
                *result.detailedDiagnostics;
            text("Unique expanded: " +
                std::to_string(detailed.uniqueExpandedCells));
            text("Repeated expansions: " +
                std::to_string(detailed.repeatedExpansions));
            text("Stale queue entries: " +
                std::to_string(detailed.staleOpenSetEntries));
            text("Maximum cell expansions: " +
                std::to_string(detailed.maximumExpansionsPerCell));
            text("Post-expanded improvements: " +
                std::to_string(detailed.postExpansionCostImprovements));
            text("Minimum improvement: " + scientificValue(
                detailed.minimumPostExpansionCostImprovement));
            text("Maximum improvement: " + scientificValue(
                detailed.maximumPostExpansionCostImprovement));
            text("Maximum relative improvement: " + scientificValue(
                detailed.maximumRelativePostExpansionCostImprovement), 38);
        }
        text("Color key", 30);

        const auto legendEntry = [&](const cv::Scalar& color, const char* label)
        {
            cv::rectangle(
                output,
                cv::Rect(left, y - 15, 20, 20),
                color,
                cv::FILLED);
            cv::putText(output, label, { left + 30, y + 1 },
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
            y += 30;
        };
        legendEntry(colors.unseenColor, "Unseen free cell");
        legendEntry(colors.openColor, "Open/frontier");
        legendEntry(colors.currentColor, "Current expansion");
        legendEntry(colors.expandedColor, "Expanded");
        legendEntry(colors.pathColor, "Final path");
        legendEntry(colors.occupiedColor, "Occupied");
        legendEntry(colors.startColor, "Start marker");
        legendEntry(colors.goalColor, "Goal marker");

        y += 12;
        text("Fixed expansion-count colors", 26);
        const auto countLegendEntry = [&output](
            int x, int entryY, const cv::Scalar& color, const char* label)
        {
            cv::rectangle(
                output,
                cv::Rect(x, entryY - 14, 18, 18),
                color,
                cv::FILLED);
            cv::putText(output, label, { x + 25, entryY },
                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
        };
        const int secondColumn = left + 170;
        countLegendEntry(left, y, HEAT_NEVER, "0: never");
        countLegendEntry(secondColumn, y, HEAT_ONCE, "1");
        y += 27;
        countLegendEntry(left, y, HEAT_TWO_TO_THREE, "2-3");
        countLegendEntry(secondColumn, y, HEAT_FOUR_TO_SEVEN, "4-7");
        y += 27;
        countLegendEntry(left, y, HEAT_EIGHT_TO_FIFTEEN, "8-15");
        countLegendEntry(secondColumn, y, HEAT_SIXTEEN_TO_THIRTY_ONE, "16-31");
        y += 27;
        countLegendEntry(left, y, HEAT_THIRTY_TWO_OR_MORE, "32+");
        countLegendEntry(secondColumn, y, HEAT_OCCUPIED, "occupied");
        return output;
    }
}

int main(int argc, char** argv)
{
    try
    {
        const Configuration configuration = parseArguments(argc, argv);
        const double extent =
            static_cast<double>(GRID_CELLS) * CELL_RESOLUTION_METERS;
        const Polygon operationArea = rectangle(0.0, 0.0, extent, extent);
        const PolygonEnvironment environment(
            operationArea, makeAlternatingBarriers(extent));
        const GridGeometry geometry(
            Point2::Zero(), CELL_RESOLUTION_METERS, GRID_CELLS, GRID_CELLS);
        const OccupancyGrid masterGrid = PolygonRasterizer::rasterize(
            geometry,
            environment.operationArea(),
            environment.effectiveObstacles());
        const OccupancyGrid planningRegion = masterGrid.subgrid(
            masterGrid.fullRegion());
        const OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
            planningRegion, SAFETY_RADIUS_METERS);

        const Point2 worldStart(
            3.5 * CELL_RESOLUTION_METERS,
            3.5 * CELL_RESOLUTION_METERS);
        const Point2 worldGoal(
            extent - 3.5 * CELL_RESOLUTION_METERS,
            extent - 3.5 * CELL_RESOLUTION_METERS);
        const auto start = planningGrid.geometry().worldToCell(worldStart);
        const auto goal = planningGrid.geometry().worldToCell(worldGoal);
        if (!start || !goal)
            throw std::runtime_error("diagnostic terminals are outside the grid");

        AStarOptions options;
        options.algorithm = configuration.algorithm;
        options.heuristicWeight = configuration.heuristicWeight;

        const AStarGridPlanner planner;
        const auto baselineStart = Clock::now();
        const GridPlanResult baseline = planner.plan(
            planningGrid, *start, *goal, options);
        const auto baselineEnd = Clock::now();
        const double baselineMilliseconds =
            std::chrono::duration<double, std::milli>(
                baselineEnd - baselineStart).count();
        if (!baseline.succeeded())
        {
            throw std::runtime_error(
                std::string("baseline search failed: ") +
                gridPlanStatusName(baseline.status));
        }

        GridSearchStateRenderOptions renderOptions;
        renderOptions.pixelsPerCell = 2;
        renderOptions.drawCellBorders = false;
        renderOptions.terminalRadiusPixels = 5;

        const std::string windowName = std::string("AStarPlanner Search Debug - ") +
            gridSearchAlgorithmName(options.algorithm);
        std::size_t callbackCount = 0;
        cv::Mat1b finalState;
        const cv::Point goalPixel = planningGrid.geometry().cellToImage(*goal);
        const GridSearchDebugCallback debugCallback =
            [&](const cv::Mat1b& state)
            {
                ++callbackCount;
                const bool finalPath = state(goalPixel.y, goalPixel.x) ==
                    static_cast<std::uint8_t>(GridSearchCellState::Path);
                if (finalPath)
                    finalState = state.clone();

                if (configuration.display && configuration.animate &&
                    (finalPath || callbackCount % configuration.frameStride == 0))
                {
                    showGridSearchState(
                        planningGrid, state, *start, *goal,
                        renderOptions, windowName);
                    cv::waitKey(1);
                }
            };

        AStarOptions detailedOptions = options;
        detailedOptions.collectDetailedDiagnostics = true;
        const GridPlanResult visualized = planner.plan(
            planningGrid, *start, *goal, detailedOptions, debugCallback);
        if (!visualized.succeeded() || finalState.empty())
            throw std::runtime_error("instrumented search did not produce a final path image");
        if (visualized.path != baseline.path)
            throw std::runtime_error("instrumented and baseline paths differ");
        if (!visualized.detailedDiagnostics)
            throw std::runtime_error("instrumented search did not produce detailed diagnostics");

        const cv::Mat3b searchImage = renderGridSearchState(
            planningGrid, finalState, *start, *goal, renderOptions);
        const cv::Mat3b expansionHeatMap = renderExpansionHeatMap(
            planningGrid,
            *visualized.detailedDiagnostics,
            renderOptions.pixelsPerCell);
        const cv::Mat3b outputImage = addLegend(
            searchImage,
            expansionHeatMap,
            renderOptions,
            options,
            visualized,
            baselineMilliseconds);
        if (!cv::imwrite(configuration.outputPath, outputImage))
            throw std::runtime_error("could not save debug image");

        std::cout << std::fixed << std::setprecision(3)
                  << "Scenario: 500 x 500 alternating barriers\n"
                  << "Algorithm: " << gridSearchAlgorithmName(options.algorithm);
        if (options.algorithm == GridSearchAlgorithm::WeightedAStar)
            std::cout << " (weight " << options.heuristicWeight << ')';
        std::cout << "\nBaseline planning: " << baselineMilliseconds << " ms\n"
                  << "Expanded nodes: " << baseline.diagnostics.expandedNodes << '\n'
                  << "Generated nodes: " << baseline.diagnostics.generatedNodes << '\n'
                  << "Peak open-set size: " << baseline.diagnostics.peakOpenSetSize << '\n'
                  << "Path cells: " << baseline.path.size() << '\n'
                  << "Path cost: " << baseline.diagnostics.pathCost << '\n'
                  << "Unique expanded cells: "
                  << visualized.detailedDiagnostics->uniqueExpandedCells << '\n'
                  << "Repeated expansions: "
                  << visualized.detailedDiagnostics->repeatedExpansions << '\n'
                  << "Stale open-set entries: "
                  << visualized.detailedDiagnostics->staleOpenSetEntries << '\n'
                  << "Maximum expansions of one cell: "
                  << visualized.detailedDiagnostics->maximumExpansionsPerCell << '\n'
                  << "Post-expansion cost improvements: "
                  << visualized.detailedDiagnostics->postExpansionCostImprovements
                  << '\n'
                  << "Minimum post-expansion improvement: "
                  << scientificValue(visualized.detailedDiagnostics->
                        minimumPostExpansionCostImprovement) << '\n'
                  << "Maximum post-expansion improvement: "
                  << scientificValue(visualized.detailedDiagnostics->
                        maximumPostExpansionCostImprovement) << '\n'
                  << "Maximum relative post-expansion improvement: "
                  << scientificValue(visualized.detailedDiagnostics->
                        maximumRelativePostExpansionCostImprovement) << '\n'
                  << "Debug callbacks: " << callbackCount << '\n'
                  << "Saved debug image: " << configuration.outputPath << '\n';

        if (configuration.display)
        {
            cv::Mat3b displayImage = outputImage;
            constexpr int maximumWindowWidth = 1800;
            constexpr int maximumWindowHeight = 900;
            const double displayScale = std::min({
                1.0,
                static_cast<double>(maximumWindowWidth) / outputImage.cols,
                static_cast<double>(maximumWindowHeight) / outputImage.rows
            });
            if (displayScale < 1.0)
            {
                cv::resize(
                    outputImage,
                    displayImage,
                    cv::Size(),
                    displayScale,
                    displayScale,
                    cv::INTER_AREA);
            }
            cv::imshow(windowName, displayImage);
            std::cout << "Press any key in the debug window to close it.\n";
            cv::waitKey(0);
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Search diagnostic failed: " << error.what() << '\n';
        return 1;
    }
}
