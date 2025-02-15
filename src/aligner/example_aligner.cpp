#include <aligner/example_aligner.hpp>

namespace diviner
{

geometry_msgs::Transform ExampleAligner::align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_)
{
    // Do nothing
    //*point_cloud = *point_cloud;

    // Aligner should be aligning the current scan with the current local map
    // Need to figure out how to do it and make an example version

    // return vehicle_alignment;
}

}