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

    cv::Mat3b renderExpansionHeatMap(
        const OccupancyGrid& grid,
        const GridSearchDetailedDiagnostics& diagnostics,
        int pixelsPerCell)
    {
        cv::Mat1f logarithmicCounts;
        diagnostics.expansionCounts.convertTo(logarithmicCounts, CV_32F);
        cv::add(logarithmicCounts, cv::Scalar(1.0), logarithmicCounts);
        cv::log(logarithmicCounts, logarithmicCounts);

        cv::Mat1b intensity;
        cv::normalize(
            logarithmicCounts,
            intensity,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U);

        cv::Mat3b heatMap;
        cv::applyColorMap(intensity, heatMap, cv::COLORMAP_TURBO);
        cv::Mat1b neverExpanded;
        cv::compare(diagnostics.expansionCounts, 0, neverExpanded, cv::CMP_EQ);
        heatMap.setTo(cv::Scalar(255, 255, 255), neverExpanded);
        heatMap.setTo(cv::Scalar(0, 0, 0), grid.imageView());

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
        constexpr int panelWidth = 360;
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
                std::to_string(detailed.maximumExpansionsPerCell), 38);
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
        text("Expansion heat map uses log scale", 24);
        cv::Mat1b colorRamp(1, 256);
        for (int column = 0; column < colorRamp.cols; ++column)
            colorRamp(0, column) = static_cast<std::uint8_t>(column);
        cv::Mat3b coloredRamp;
        cv::applyColorMap(colorRamp, coloredRamp, cv::COLORMAP_TURBO);
        cv::resize(coloredRamp, coloredRamp, cv::Size(256, 18),
            0.0, 0.0, cv::INTER_NEAREST);
        coloredRamp.copyTo(output(cv::Rect(left, y, 256, 18)));
        y += 38;
        text("low expansion count        high", 24);
        text("white: never expanded; black: occupied");
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
