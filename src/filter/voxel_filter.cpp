#include <filter/voxel_filter.hpp>

#include <vector>

namespace diviner
{

void VoxelFilter::filter(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud)
{
    if(first_run)
    {
        // std::cout << "  - filter: First run" << std::endl;
        leaf_size = params_.leaf_size[0];

        first_run = false;
    }
    else
    {
        // std::cout << "  - filter: Second run" << std::endl;
        leaf_size = params_.leaf_size[1];

        first_run = true;
    }

    // std::vector<float> leaf_size = params_.leaf_size;
    if(params_.debug)
    {
        std::cout << "  - filter: Leaf size: " << leaf_size << std::endl;
        std::cout << "  - filter: num points incoming: " << point_cloud->size() << std::endl;
    }

    // Do data processing here...
    pcl::VoxelGrid<diviner::PointStamped> filter;
    filter.setInputCloud(point_cloud);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*point_cloud);

    if(params_.debug)
    {
        std::cout << "  - filter: num points outgoing: " << point_cloud->size() << std::endl;
    }
}

}
