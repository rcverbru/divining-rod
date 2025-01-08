#include <deskewer/example_deskewer.hpp>

#include <vector>

namespace diviner
{

void ExampleDeskewer::deskew(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const std::vector<diviner::Velocity> & velocities)
{
    // Do nothing
    // *point_cloud = *point_cloud;
}

}
