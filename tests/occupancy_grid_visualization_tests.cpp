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
