#include <filter/example_filter.hpp>

#include <vector>

namespace diviner
{

void ExampleFilter::filter(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud)
{
    // Do no data processing :)
    std::cout << "  - filter: Number of points in point cloud: " << std::endl;
}

}
