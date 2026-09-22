/*
 * a_star_planner_visualization.hpp – Polygon-environment plotting.
 *
 * ────
 * Core features
 *   • Renders an optional operation area plus normalized and clipped obstacles.
 *   • Highlights start and goal query points.
 *   • Keeps plotting support separate from the core planner header.
 * ────
 */
#pragma once

#include "a_star_planner.hpp"
#include "figure.h"

#include <utility>

namespace astar
{
    /**
     * @brief Visualize the retained polygon environment and terminal points.
     *
     * Draws the operation-area boundary when present, followed by obstacle
     * polygons, an optional world-coordinate path, and the start/goal markers.
     * When a path is supplied, the exact start and goal positions are added to
     * its ends; grid paths normally contain cell-center positions between them.
     *
     * @param environment Environment to render.
     * @param start     Start query point.
     * @param goal      Goal query point.
     * @param pixelSize Figure width/height in pixels.
     * @param outputFile Optional image filename written before showing the window.
     * @param path       Optional path in world coordinates.
     */
    inline void visualize(const PolygonEnvironment& environment,
        const Point2& start,
        const Point2& goal,
        int pixelSize = 1200,
        const std::string& outputFile = {},
        const std::vector<Point2>& path = {})
    {
        using namespace mpocv;

        Figure fig(pixelSize, pixelSize);

        ShapeStyle obstacleStyle;
        obstacleStyle.line_color = Color::Blue();
        obstacleStyle.thickness = 1.5f;
        obstacleStyle.fill_color = Color::Blue();
        obstacleStyle.fill_alpha = 0.1f;

        ShapeStyle clippedObstacleStyle;
        clippedObstacleStyle.line_color = Color::Magenta();
        clippedObstacleStyle.thickness = 2.5f;
        clippedObstacleStyle.fill_color = Color::Magenta();
        clippedObstacleStyle.fill_alpha = 0.25f;

        if (environment.hasOperationArea())
        {
            const auto& operationArea = environment.operationArea();
            std::vector<double> x;
            std::vector<double> y;
            x.reserve(operationArea.size() + 1);
            y.reserve(operationArea.size() + 1);

            for (const auto& point : operationArea)
            {
                x.push_back(point.x());
                y.push_back(point.y());
            }
            x.push_back(operationArea.front().x());
            y.push_back(operationArea.front().y());

            fig.plot(x, y, Color(0, 100, 70), 1.0f, "Operation area boundary");
        }

        const auto drawPolygons = [&fig](const std::vector<Polygon>& polygons,
            const ShapeStyle& style,
            const std::string& label)
        {
            for (std::size_t i = 0; i < polygons.size(); ++i)
            {
                const auto& poly = polygons[i];
                std::vector<double> x;
                std::vector<double> y;
                x.reserve(poly.size());
                y.reserve(poly.size());

                for (const auto& p : poly)
                {
                    x.push_back(p.x());
                    y.push_back(p.y());
                }

                fig.polygon(x, y, style);
                if (i == 0 && poly.size() >= 2)
                {
                    // MatPlotOpenCV polygon entries do not preserve the shape's
                    // outline color in the legend. Overlay one edge to create
                    // an accurate legend sample for this polygon group.
                    fig.plot(
                        { poly[0].x(), poly[1].x() },
                        { poly[0].y(), poly[1].y() },
                        style.line_color,
                        style.thickness,
                        label);
                }
            }
        };

        // Draw every normalized input obstacle, including portions outside the
        // operation area, then highlight positive-area clipped results.
        drawPolygons(
            environment.normalizedObstacles(), obstacleStyle, "Normalized obstacle");
        drawPolygons(
            environment.clippedObstacles(), clippedObstacleStyle, "Clipped obstacle");

        if (!path.empty())
        {
            std::vector<double> pathX;
            std::vector<double> pathY;
            pathX.reserve(path.size() + 2);
            pathY.reserve(path.size() + 2);
            pathX.push_back(start.x());
            pathY.push_back(start.y());
            for (const Point2& point : path)
            {
                pathX.push_back(point.x());
                pathY.push_back(point.y());
            }
            pathX.push_back(goal.x());
            pathY.push_back(goal.y());
            fig.plot(
                std::move(pathX),
                std::move(pathY),
                Color(255, 140, 0),
                2.5f,
                "Planned path");
        }

        // Draw query terminals last so they stay visible on top.
        fig.scatter({ start.x() }, { start.y() }, Color::Green(), 6.0f, "Start");
        fig.scatter({ goal.x() }, { goal.y() }, Color::Red(), 6.0f, "Goal");

        fig.grid(true);
        fig.equal_scale(true);
        fig.title(environment.hasOperationArea()
            ? "AStarPlanner Operation Area"
            : "AStarPlanner Environment");
        fig.legend(true, "southEast");
        if (!outputFile.empty())
            fig.save(outputFile);
        fig.show("AStarPlanner Environment");
    }
}
