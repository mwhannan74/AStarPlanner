/*
 * a_star_planner_benchmark.cpp - Repeatable end-to-end performance measurements.
 */

#include "a_star_grid_planner.hpp"
#include "a_star_planner.hpp"
#include "occupancy_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace astar;
    using Clock = std::chrono::steady_clock;

    constexpr double CELL_RESOLUTION_METERS = 25.0;
    constexpr double SAFETY_RADIUS_METERS = CELL_RESOLUTION_METERS;
    constexpr double WEIGHTED_ASTAR_WEIGHT = 1.5;

    enum class ScenarioKind
    {
        Open,
        AlternatingBarriers,
        MixedOccupancy,
        NoPath
    };

    struct BenchmarkSize
    {
        int cells;
        int preparationRepetitions;
        int searchRepetitions;
    };

    struct Scenario
    {
        ScenarioKind kind;
        const char* name;
        Polygon operationArea;
        std::vector<Polygon> obstacles;
        std::vector<Point2> perceptionPoints;
        Point2 worldStart;
        Point2 worldGoal;
        bool pathExpected;
    };

    struct TimingSamples
    {
        std::vector<double> environment;
        std::vector<double> polygonRasterization;
        std::vector<double> pointRasterization;
        std::vector<double> fusion;
        std::vector<double> inflation;
        std::vector<double> total;
    };

    struct SearchMeasurements
    {
        std::vector<double> coordinateConversion;
        std::vector<double> search;
        std::vector<double> pathSimplification;
        std::vector<double> pathConversion;
        std::vector<double> total;
        GridPlanResult representativeResult;
    };

    struct RunConfiguration
    {
        bool quick = false;
        bool stress = false;
        std::string csvPath;
        AStarTieBreakPolicy tieBreakPolicy =
            AStarTieBreakPolicy::StraightLineThenLargerG;
    };

    double elapsedMilliseconds(Clock::time_point start, Clock::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    double percentile(std::vector<double> samples, double probability)
    {
        if (samples.empty())
            throw std::invalid_argument("percentile requires at least one sample");
        std::sort(samples.begin(), samples.end());
        const std::size_t rank = static_cast<std::size_t>(
            std::ceil(probability * static_cast<double>(samples.size())));
        return samples[std::max<std::size_t>(1, rank) - 1];
    }

    double median(const std::vector<double>& samples)
    {
        return percentile(samples, 0.5);
    }

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

    Scenario makeScenario(ScenarioKind kind, int cells)
    {
        const double extent = static_cast<double>(cells) * CELL_RESOLUTION_METERS;
        Scenario scenario{
            kind,
            "open",
            rectangle(0.0, 0.0, extent, extent),
            {},
            {},
            Point2(3.5 * CELL_RESOLUTION_METERS, 3.5 * CELL_RESOLUTION_METERS),
            Point2(extent - 3.5 * CELL_RESOLUTION_METERS,
                extent - 3.5 * CELL_RESOLUTION_METERS),
            true
        };

        if (kind == ScenarioKind::Open)
            return scenario;

        if (kind == ScenarioKind::NoPath)
        {
            scenario.name = "no_path";
            const double wallHalfWidth = std::max(
                CELL_RESOLUTION_METERS, 0.005 * extent);
            scenario.obstacles.push_back(rectangle(
                0.5 * extent - wallHalfWidth,
                0.0,
                0.5 * extent + wallHalfWidth,
                extent));
            scenario.pathExpected = false;
            return scenario;
        }

        const bool includePerception = kind == ScenarioKind::MixedOccupancy;
        scenario.name = includePerception
            ? "mixed_occupancy"
            : "alternating_barriers";
        constexpr int barrierCount = 6;
        const double wallHalfWidth = std::max(
            CELL_RESOLUTION_METERS, 0.005 * extent);
        for (int barrier = 0; barrier < barrierCount; ++barrier)
        {
            const double x = extent * static_cast<double>(barrier + 1) /
                static_cast<double>(barrierCount + 1);
            const bool gapAtTop = barrier % 2 == 0;
            const double minimumY = gapAtTop ? 0.0 : 0.25 * extent;
            const double maximumY = gapAtTop ? 0.75 * extent : extent;
            scenario.obstacles.push_back(rectangle(
                x - wallHalfWidth, minimumY,
                x + wallHalfWidth, maximumY));

            // The mixed case samples cells inside the polygon barriers. This
            // exercises point rasterization and fusion without changing the
            // search problem relative to the polygon-only barrier case.
            if (includePerception)
            {
                for (double pointX = x - wallHalfWidth +
                         0.5 * CELL_RESOLUTION_METERS;
                     pointX < x + wallHalfWidth;
                     pointX += CELL_RESOLUTION_METERS)
                {
                    for (double pointY = minimumY +
                             0.5 * CELL_RESOLUTION_METERS;
                         pointY < maximumY;
                         pointY += CELL_RESOLUTION_METERS)
                    {
                        scenario.perceptionPoints.emplace_back(pointX, pointY);
                    }
                }
            }
        }
        return scenario;
    }

    OccupancyGrid prepareGrid(
        const Scenario& scenario,
        int cells,
        TimingSamples* samples)
    {
        const auto totalStart = Clock::now();

        const auto environmentStart = Clock::now();
        const PolygonEnvironment environment(
            scenario.operationArea, scenario.obstacles);
        const auto environmentEnd = Clock::now();

        const GridGeometry geometry(
            Point2::Zero(), CELL_RESOLUTION_METERS, cells, cells);
        const auto polygonRasterizationStart = Clock::now();
        const OccupancyGrid polygonGrid = PolygonRasterizer::rasterize(
            geometry, environment);
        const auto polygonRasterizationEnd = Clock::now();

        double pointRasterizationMilliseconds = 0.0;
        double fusionMilliseconds = 0.0;
        std::optional<OccupancyGrid> fusedGrid;
        if (!scenario.perceptionPoints.empty())
        {
            const auto pointRasterizationStart = Clock::now();
            const OccupancyGrid perceptionGrid = PointRasterizer::rasterize(
                geometry, scenario.perceptionPoints);
            const auto pointRasterizationEnd = Clock::now();
            pointRasterizationMilliseconds = elapsedMilliseconds(
                pointRasterizationStart, pointRasterizationEnd);

            const auto fusionStart = Clock::now();
            fusedGrid = OccupancyGridFusion::occupiedUnion(
                polygonGrid, perceptionGrid);
            const auto fusionEnd = Clock::now();
            fusionMilliseconds = elapsedMilliseconds(fusionStart, fusionEnd);
        }

        const OccupancyGrid& occupancyGrid = fusedGrid
            ? *fusedGrid
            : polygonGrid;

        const auto inflationStart = Clock::now();
        OccupancyGrid planningGrid = OccupancyGridInflator::inflate(
            occupancyGrid, SAFETY_RADIUS_METERS);
        const auto inflationEnd = Clock::now();

        if (samples)
        {
            samples->environment.push_back(elapsedMilliseconds(
                environmentStart, environmentEnd));
            samples->polygonRasterization.push_back(elapsedMilliseconds(
                polygonRasterizationStart, polygonRasterizationEnd));
            samples->pointRasterization.push_back(
                pointRasterizationMilliseconds);
            samples->fusion.push_back(fusionMilliseconds);
            samples->inflation.push_back(elapsedMilliseconds(
                inflationStart, inflationEnd));
            samples->total.push_back(elapsedMilliseconds(totalStart, inflationEnd));
        }
        return planningGrid;
    }

    SearchMeasurements measureSearch(
        const OccupancyGrid& grid,
        const Scenario& scenario,
        const AStarOptions& options,
        int repetitions)
    {
        const AStarGridPlanner planner;

        const auto runOnce = [&](SearchMeasurements* measurements)
        {
            const auto totalStart = Clock::now();
            const auto conversionStart = Clock::now();
            const auto start = grid.geometry().worldToCell(scenario.worldStart);
            const auto goal = grid.geometry().worldToCell(scenario.worldGoal);
            const auto conversionEnd = Clock::now();
            if (!start || !goal)
                throw std::runtime_error("benchmark terminal is outside the planning grid");

            const auto searchStart = Clock::now();
            GridPlanResult result = planner.plan(grid, *start, *goal, options);
            const auto searchEnd = Clock::now();

            const auto pathConversionStart = Clock::now();
            const std::vector<Point2> worldPath = gridPathToWorld(grid, result.path);
            const std::vector<Point2> simplifiedWorldPath =
                gridPathToWorld(grid, result.simplifiedPath);
            const auto pathConversionEnd = Clock::now();

            if (result.succeeded() != scenario.pathExpected)
            {
                throw std::runtime_error(
                    std::string("unexpected planning result for scenario '") +
                    scenario.name + "': " + gridPlanStatusName(result.status));
            }

            if (measurements)
            {
                measurements->coordinateConversion.push_back(elapsedMilliseconds(
                    conversionStart, conversionEnd));
                measurements->search.push_back(elapsedMilliseconds(
                    searchStart, searchEnd));
                measurements->pathSimplification.push_back(
                    result.diagnostics.pathSimplificationMilliseconds);
                measurements->pathConversion.push_back(elapsedMilliseconds(
                    pathConversionStart, pathConversionEnd));
                measurements->total.push_back(elapsedMilliseconds(
                    totalStart, pathConversionEnd));
                measurements->representativeResult = std::move(result);
            }

            // Keep the conversion observable in optimized builds.
            return worldPath.size() + simplifiedWorldPath.size();
        };

        static volatile std::size_t convertedPathCellSink = 0;
        convertedPathCellSink = runOnce(nullptr); // Warm-up.

        SearchMeasurements measurements;
        for (int repetition = 0; repetition < repetitions; ++repetition)
            convertedPathCellSink = runOnce(&measurements);
        return measurements;
    }

    const char* algorithmCsvName(GridSearchAlgorithm algorithm)
    {
        switch (algorithm)
        {
        case GridSearchAlgorithm::AStar: return "astar";
        case GridSearchAlgorithm::Dijkstra: return "dijkstra";
        case GridSearchAlgorithm::WeightedAStar: return "weighted_astar";
        }
        return "unknown";
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

    std::string timestampText(const char* format)
    {
        const std::time_t now = std::time(nullptr);
        std::tm localTime{};
#if defined(_WIN32)
        localtime_s(&localTime, &now);
#else
        localtime_r(&now, &localTime);
#endif
        std::ostringstream stream;
        stream << std::put_time(&localTime, format);
        return stream.str();
    }

    std::string defaultCsvPath()
    {
        return "astar_benchmark_" + timestampText("%Y%m%d_%H%M%S") + ".csv";
    }

    const char* buildType()
    {
#if defined(NDEBUG)
        return "Release";
#else
        return "Debug";
#endif
    }

    RunConfiguration parseArguments(int argc, char** argv)
    {
        RunConfiguration configuration;
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--quick")
                configuration.quick = true;
            else if (argument == "--stress")
                configuration.stress = true;
            else if (argument == "--csv" && index + 1 < argc)
                configuration.csvPath = argv[++index];
            else if (argument == "--tie-break" && index + 1 < argc)
                configuration.tieBreakPolicy = parseTieBreakPolicy(argv[++index]);
            else if (argument == "--help")
            {
                std::cout
                    << "Usage: a_star_planner_benchmark [--quick] [--stress]"
                    << " [--csv FILE] [--tie-break POLICY]\n"
                    << "  --quick   Run only the 100 x 100 smoke benchmark.\n"
                    << "  --stress  Add a 2000 x 2000 benchmark case.\n"
                    << "  --csv     Select the output CSV path.\n"
                    << "  --tie-break  Select straight-line (default),"
                       " larger-g-then-line, or larger-g for A* and weighted A*.\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument("unknown or incomplete argument: " + argument);
            }
        }
        if (configuration.quick && configuration.stress)
            throw std::invalid_argument("--quick and --stress cannot be combined");
        if (configuration.csvPath.empty())
            configuration.csvPath = defaultCsvPath();
        return configuration;
    }

    std::vector<BenchmarkSize> benchmarkSizes(const RunConfiguration& configuration)
    {
        if (configuration.quick)
            return { { 100, 2, 2 } };

        std::vector<BenchmarkSize> sizes{
            { 100, 10, 20 },
            { 250, 8, 15 },
            { 500, 5, 8 },
            { 1000, 3, 3 }
        };
        if (configuration.stress)
            sizes.push_back({ 2000, 1, 1 });
        return sizes;
    }

    void writeCsvHeader(std::ofstream& csv)
    {
        csv
            << "timestamp,build_type,scenario,grid_width,grid_height,resolution_m,"
            << "safety_radius_m,algorithm,heuristic_weight,tie_break_policy,"
            << "preparation_repetitions,"
            << "search_repetitions,environment_median_ms,rasterization_median_ms,"
            << "inflation_median_ms,preparation_total_median_ms,"
            << "coordinate_conversion_median_ms,search_median_ms,search_p95_ms,"
            << "path_simplification_median_ms,path_conversion_median_ms,"
            << "request_total_median_ms,success,status,expanded_nodes,generated_nodes,"
            << "peak_open_set,path_cells,simplified_waypoints,path_cost,"
            << "polygon_obstacles,perception_points,"
            << "point_rasterization_median_ms,fusion_median_ms\n";
    }

    void writeCsvRow(
        std::ofstream& csv,
        const std::string& timestamp,
        const BenchmarkSize& size,
        const Scenario& scenario,
        const TimingSamples& preparation,
        const AStarOptions& options,
        const SearchMeasurements& search)
    {
        const GridPlanResult& result = search.representativeResult;
        csv << std::setprecision(9)
            << timestamp << ',' << buildType() << ',' << scenario.name << ','
            << size.cells << ',' << size.cells << ',' << CELL_RESOLUTION_METERS << ','
            << SAFETY_RADIUS_METERS << ',' << algorithmCsvName(options.algorithm) << ','
            << (options.algorithm == GridSearchAlgorithm::WeightedAStar
                    ? options.heuristicWeight :
                    options.algorithm == GridSearchAlgorithm::AStar ? 1.0 : 0.0)
            << ',' << aStarTieBreakPolicyName(options.tieBreakPolicy)
            << ',' << size.preparationRepetitions << ',' << size.searchRepetitions << ','
            << median(preparation.environment) << ','
            << median(preparation.polygonRasterization) << ','
            << median(preparation.inflation) << ','
            << median(preparation.total) << ','
            << median(search.coordinateConversion) << ','
            << median(search.search) << ','
            << percentile(search.search, 0.95) << ','
            << median(search.pathSimplification) << ','
            << median(search.pathConversion) << ','
            << median(search.total) << ','
            << (result.succeeded() ? "true" : "false") << ','
            << gridPlanStatusName(result.status) << ','
            << result.diagnostics.expandedNodes << ','
            << result.diagnostics.generatedNodes << ','
            << result.diagnostics.peakOpenSetSize << ','
            << result.path.size() << ',' << result.simplifiedPath.size() << ','
            << result.diagnostics.pathCost << ','
            << scenario.obstacles.size() << ','
            << scenario.perceptionPoints.size() << ','
            << median(preparation.pointRasterization) << ','
            << median(preparation.fusion) << '\n';
    }
}

int main(int argc, char** argv)
{
    try
    {
        const RunConfiguration configuration = parseArguments(argc, argv);
        std::ofstream csv(configuration.csvPath);
        if (!csv)
            throw std::runtime_error("could not open CSV output: " + configuration.csvPath);
        writeCsvHeader(csv);

        std::cout << "AStarPlanner benchmark (" << buildType() << ")\n"
                  << "Resolution: " << CELL_RESOLUTION_METERS
                  << " m/cell; safety radius: " << SAFETY_RADIUS_METERS << " m\n"
                  << "A*/weighted tie-break: "
                  << aStarTieBreakPolicyName(configuration.tieBreakPolicy) << '\n'
                  << "Times are medians after one warm-up; search p95 is also reported.\n";
#if !defined(NDEBUG)
        std::cout << "WARNING: Debug builds are not representative of planner performance.\n";
#endif

        std::vector<AStarOptions> algorithms{
            AStarOptions{ GridSearchAlgorithm::AStar, 1.0,
                GridConnectivity::EightConnected, true },
            AStarOptions{ GridSearchAlgorithm::Dijkstra, 1.0,
                GridConnectivity::EightConnected, true },
            AStarOptions{ GridSearchAlgorithm::WeightedAStar, WEIGHTED_ASTAR_WEIGHT,
                GridConnectivity::EightConnected, true }
        };
        algorithms[0].tieBreakPolicy = configuration.tieBreakPolicy;
        algorithms[2].tieBreakPolicy = configuration.tieBreakPolicy;
        const std::vector<ScenarioKind> scenarioKinds{
            ScenarioKind::Open,
            ScenarioKind::AlternatingBarriers,
            ScenarioKind::MixedOccupancy,
            ScenarioKind::NoPath
        };
        const std::string timestamp = timestampText("%Y-%m-%dT%H:%M:%S");

        for (const BenchmarkSize& size : benchmarkSizes(configuration))
        {
            for (ScenarioKind scenarioKind : scenarioKinds)
            {
                const Scenario scenario = makeScenario(scenarioKind, size.cells);

                // Warm the OpenCV and allocation paths before collecting preparation samples.
                static_cast<void>(prepareGrid(scenario, size.cells, nullptr));
                TimingSamples preparation;
                std::optional<OccupancyGrid> planningGrid;
                for (int repetition = 0;
                     repetition < size.preparationRepetitions;
                     ++repetition)
                {
                    planningGrid = prepareGrid(
                        scenario, size.cells, &preparation);
                }

                std::cout << "\n" << size.cells << " x " << size.cells
                          << "  " << scenario.name
                          << "  prep " << std::fixed << std::setprecision(3)
                          << median(preparation.total) << " ms"
                          << " (env " << median(preparation.environment)
                          << ", polygon "
                          << median(preparation.polygonRasterization)
                          << ", points "
                          << median(preparation.pointRasterization)
                          << ", fuse " << median(preparation.fusion)
                          << ", inflate " << median(preparation.inflation) << ")\n";
                std::cout << "  " << std::left << std::setw(17) << "algorithm"
                          << std::right << std::setw(12) << "median ms"
                          << std::setw(11) << "p95 ms"
                          << std::setw(13) << "simplify ms"
                          << std::setw(13) << "expanded"
                          << std::setw(13) << "generated"
                          << std::setw(11) << "peak open"
                          << std::setw(12) << "path cells"
                          << std::setw(13) << "waypoints\n";

                for (const AStarOptions& options : algorithms)
                {
                    const SearchMeasurements search = measureSearch(
                        *planningGrid, scenario, options, size.searchRepetitions);
                    const GridPlanResult& result = search.representativeResult;
                    std::ostringstream algorithmLabel;
                    algorithmLabel << gridSearchAlgorithmName(options.algorithm);
                    if (options.algorithm == GridSearchAlgorithm::WeightedAStar)
                        algorithmLabel << " " << options.heuristicWeight;

                    std::cout << "  " << std::left << std::setw(17)
                              << algorithmLabel.str()
                              << std::right << std::fixed << std::setprecision(3)
                              << std::setw(12) << median(search.search)
                              << std::setw(11) << percentile(search.search, 0.95)
                              << std::setw(13) << median(search.pathSimplification)
                              << std::setw(13) << result.diagnostics.expandedNodes
                              << std::setw(13) << result.diagnostics.generatedNodes
                              << std::setw(11) << result.diagnostics.peakOpenSetSize
                              << std::setw(12) << result.path.size()
                              << std::setw(13) << result.simplifiedPath.size() << '\n';

                    writeCsvRow(
                        csv, timestamp, size, scenario,
                        preparation, options, search);
                }
            }
        }

        std::cout << "\nCSV report: " << configuration.csvPath << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
