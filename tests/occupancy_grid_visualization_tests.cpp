#include "occupancy_grid_visualization.hpp"

#include <iostream>
#include <stdexcept>

namespace
{
    using namespace astar;

    void require(bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }
}

int main()
{
    try
    {
        const GridGeometry geometry(Point2(0.0, 0.0), 1.0, 2, 2);
        const Polygon lowerLeftObstacle{
            Point2(0.1, 0.1), Point2(0.9, 0.1),
            Point2(0.9, 0.9), Point2(0.1, 0.9)
        };
        const OccupancyGrid grid =
            PolygonRasterizer::rasterize(geometry, { lowerLeftObstacle });

        OccupancyGridRenderOptions options;
        options.pixelsPerCell = 4;
        options.drawCellBorders = false;
        options.markers.push_back({ { 1, 1 }, cv::Scalar(0, 0, 255), 1 });

        const cv::Mat3b image = renderOccupancyGrid(grid, options);
        require(image.rows == 8 && image.cols == 8,
            "rendered dimensions should scale with pixelsPerCell");

        const cv::Vec3b occupied = image(6, 2);
        require(occupied == cv::Vec3b(0, 0, 0),
            "occupied lower-left cell should render black at the lower left");

        const cv::Vec3b marker = image(2, 6);
        require(marker[2] > 200 && marker[0] < 30 && marker[1] < 30,
            "generic cell marker should use the requested color");

        OccupancyGridRenderOptions pathOptions;
        pathOptions.pixelsPerCell = 8;
        pathOptions.drawCellBorders = false;
        pathOptions.paths.push_back({
            { { 0, 0 }, { 1, 1 } },
            cv::Scalar(255, 0, 0),
            2
        });
        pathOptions.markers.push_back({ { 1, 1 }, cv::Scalar(0, 0, 255), 2 });
        const cv::Mat3b pathImage = renderOccupancyGrid(grid, pathOptions);
        const cv::Vec3b pathMiddle = pathImage(8, 8);
        require(pathMiddle[0] > 200 && pathMiddle[1] < 50 && pathMiddle[2] < 50,
            "ordered path cells should render as one OpenCV polyline");
        const cv::Vec3b pathEndMarker = pathImage(4, 12);
        require(pathEndMarker[2] > 200 && pathEndMarker[0] < 50,
            "markers should render on top of path overlays");

        OccupancyGridRenderOptions singleCellPath;
        singleCellPath.pixelsPerCell = 8;
        singleCellPath.drawCellBorders = false;
        singleCellPath.paths.push_back({
            { { 0, 1 } },
            cv::Scalar(0, 255, 0),
            2
        });
        const cv::Mat3b singleCellImage = renderOccupancyGrid(grid, singleCellPath);
        const cv::Vec3b singleCell = singleCellImage(4, 4);
        require(singleCell[1] > 200 && singleCell[0] < 50 && singleCell[2] < 50,
            "a one-cell path should remain visible");

        bool rejectedInvalidScale = false;
        try
        {
            OccupancyGridRenderOptions invalid;
            invalid.pixelsPerCell = 0;
            static_cast<void>(renderOccupancyGrid(grid, invalid));
        }
        catch (const std::invalid_argument&)
        {
            rejectedInvalidScale = true;
        }
        require(rejectedInvalidScale, "non-positive render scale should be rejected");

        bool rejectedInvalidPathCell = false;
        try
        {
            OccupancyGridRenderOptions invalid;
            invalid.paths.push_back({
                { { 0, 0 }, { 2, 1 } },
                cv::Scalar(255, 0, 0),
                2
            });
            static_cast<void>(renderOccupancyGrid(grid, invalid));
        }
        catch (const std::out_of_range&)
        {
            rejectedInvalidPathCell = true;
        }
        require(rejectedInvalidPathCell,
            "path cells outside the grid should be rejected");

        bool rejectedInvalidPathThickness = false;
        try
        {
            OccupancyGridRenderOptions invalid;
            invalid.paths.push_back({
                { { 0, 0 }, { 1, 1 } },
                cv::Scalar(255, 0, 0),
                0
            });
            static_cast<void>(renderOccupancyGrid(grid, invalid));
        }
        catch (const std::invalid_argument&)
        {
            rejectedInvalidPathThickness = true;
        }
        require(rejectedInvalidPathThickness,
            "non-positive path thickness should be rejected");

        std::cout << "Occupancy-grid visualization tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Occupancy-grid visualization test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
