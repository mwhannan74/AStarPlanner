/*
 * a_star_planner_visualization.hpp – Environment plotting for AStarPlanner.
 *
 * ────
 * Core features
 *   • Renders an optional operation area plus original and clipped obstacles.
 *   • Highlights start and goal query points.
 *   • Keeps plotting support separate from the core planner header.
 * ────
 */
#pragma once

#include "a_star_planner.hpp"
#include "figure.h"

namespace astar
{
    /**
     * @brief Visualize the retained polygon environment and terminal points.
     *
     * Draws the operation-area boundary when present, followed by obstacle
     * polygons and the start/goal markers. Occupancy-grid and path layers will
     * be added when the grid-based planner is implemented.
     *
     * @param planner   Environment to render.
     * @param start     Start query point.
     * @param goal      Goal query point.
     * @param pixelSize Figure width/height in pixels.
     * @param outputFile Optional image filename written before showing the window.
     */
    inline void visualize(const AStarPlanner& planner,
        const Point2& start,
        const Point2& goal,
        int pixelSize = 1200,
        const std::string& outputFile = {})
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

        if (planner.hasOperationArea())
        {
            const auto& operationArea = planner.operationArea();
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

        // Draw every original obstacle, including portions outside the
        // operation area, then highlight positive-area clipped results.
        drawPolygons(planner.originalObstacles(), obstacleStyle, "Original obstacle");
        drawPolygons(planner.clippedObstacles(), clippedObstacleStyle, "Clipped obstacle");

        // Draw query terminals last so they stay visible on top.
        fig.scatter({ start.x() }, { start.y() }, Color::Green(), 6.0f, "Start");
        fig.scatter({ goal.x() }, { goal.y() }, Color::Red(), 6.0f, "Goal");

        fig.grid(true);
        fig.equal_scale(true);
        fig.title(planner.hasOperationArea()
            ? "AStarPlanner Operation Area"
            : "AStarPlanner Environment");
        fig.legend(true, "southEast");
        if (!outputFile.empty())
            fig.save(outputFile);
        fig.show("AStarPlanner Environment");
    }
}
