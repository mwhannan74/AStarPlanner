/*
 * visibility_graph_visualization.hpp – Reusable plotting helpers for
 * vg::VisibilityGraph using MatPlotOpenCV.
 *
 * ────
 * Core features
 *   • Renders an optional operation area, original and clipped obstacles,
 *     graph edges, and an optional shortest path.
 *   • Highlights start and goal query points.
 *   • Keeps plotting support separate from the core planner header.
 *
 * Example:
 *   vg::VisibilityGraph vg(obstacles);
 *   vg.buildBasic();
 *   auto ids = vg.injectQueryPts(S, G);
 *   auto path = vg.shortestPath(ids.first, ids.second);
 *   vg::visualize(vg, S, G, path);
 * ────
 */
#pragma once

#include "visibility_graph.hpp"
#include "figure.h"

namespace vg
{
    /**
     * @brief Visualize a visibility graph, query points, and optional path.
     *
     * Draws the operation-area boundary when present, followed by obstacle
     * polygons, graph edges, the shortest path polyline if provided, and the
     * start/goal markers.
     *
     * @param graph     Visibility graph to render.
     * @param start     Start query point.
     * @param goal      Goal query point.
     * @param path      Optional path returned from shortestPath().
     * @param pixelSize Figure width/height in pixels.
     */
    inline void visualize(const VisibilityGraph& graph,
        const Point2& start,
        const Point2& goal,
        const std::vector<Point2>& path = {},
        int pixelSize = 1200)
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

        if (graph.hasOperationArea())
        {
            const auto& operationArea = graph.operationArea();
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
        drawPolygons(graph.originalObstacles(), obstacleStyle, "Original obstacle");
        drawPolygons(graph.clippedObstacles(), clippedObstacleStyle, "Clipped obstacle");

        // Draw each undirected graph edge once.
        const auto& adjacency = graph.adjacency();
        const auto& vertices = graph.vertices();
        const Color visibilityEdgeColor(70, 70, 70);
        bool visibilityEdgeLabeled = false;
        for (std::size_t i = 0; i < adjacency.size(); ++i)
        {
            const auto& pi = vertices[i].pos;
            for (const auto& e : adjacency[i])
            {
                if (i >= e.to) continue;

                const auto& pj = vertices[e.to].pos;
                fig.plot(
                    { pi.x(), pj.x() },
                    { pi.y(), pj.y() },
                    visibilityEdgeColor,
                    1.0f,
                    visibilityEdgeLabeled ? "" : "Visibility edge");
                visibilityEdgeLabeled = true;
            }
        }

        // Mark graph vertices explicitly so polygon corners that participate
        // in the visibility graph are distinguishable from decorative outlines.
        if (!vertices.empty())
        {
            std::vector<double> vertexX;
            std::vector<double> vertexY;
            vertexX.reserve(vertices.size());
            vertexY.reserve(vertices.size());

            for (const auto& vertex : vertices)
            {
                vertexX.push_back(vertex.pos.x());
                vertexY.push_back(vertex.pos.y());
            }

            fig.scatter(vertexX, vertexY, visibilityEdgeColor, 3.0f, "Graph vertex");
        }

        // Draw the shortest path and its waypoints when supplied.
        if (!path.empty())
        {
            const Color pathColor(255, 140, 0);
            std::vector<double> px;
            std::vector<double> py;
            px.reserve(path.size());
            py.reserve(path.size());

            for (const auto& pt : path)
            {
                px.push_back(pt.x());
                py.push_back(pt.y());
            }

            fig.plot(px, py, pathColor, 3.5f, "Path");
            fig.scatter(px, py, pathColor, 5.0f, "Path waypoint");
        }

        // Draw query terminals last so they stay visible on top.
        fig.scatter({ start.x() }, { start.y() }, Color::Green(), 6.0f, "Start");
        fig.scatter({ goal.x() }, { goal.y() }, Color::Red(), 6.0f, "Goal");

        fig.grid(true);
        fig.equal_scale(true);
        fig.title(graph.hasOperationArea()
            ? "Visibility Graph with Operation Area"
            : "Visibility Graph");
        fig.legend(true, "southEast");
        fig.show("Visibility Graph");
    }
}
