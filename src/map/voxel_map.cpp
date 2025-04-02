#include <map/voxel_map.hpp>

namespace diviner
{

void VoxelMap::add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_)
{
    if(params_.debug)
    {
        std::cout << "  - map: Adding point cloud to map " << std::endl;
    }

    // std::cout << "  - map: " << local_map_pointcloud->octreeCanResize() << std::endl;

    if(params_.debug)
    {
        std::cout << "  - map: Number of points in pointcloud: " << point_cloud_->size() << std::endl;
        std::cout << "  - map: Number of points in map before adding: " << local_map_pointcloud->size() << std::endl;    
    }

    for(auto point : *point_cloud_)
    {
        local_map_pointcloud->push_back(point);
    }

    if(params_.debug)
    {
        std::cout << "  - map: Num points in map cloud: " << local_map_pointcloud->size() << std::endl;
    }

    if(params_.debug)
    {
        // for(int i = 0; i<local_map_pointcloud->getTreeDepth(); i++)
        // {
        //     std::cout << "  - map: Depth: " << i << " Size of Voxel: " << local_map_pointcloud->getVoxelSquaredDiameter(i) << std::endl;
        // }

        // std::cout << "  - map: Resolution: " << local_map_pointcloud->getResolution() << std::endl;
    }

}

pcl::PointCloud<diviner::PointStamped>::Ptr VoxelMap::get_data()
{
    return local_map_pointcloud;
}

void VoxelMap::get_voxels()
{
    //
}

void VoxelMap::clear_map()
{
    local_map_pointcloud->clear();
}

int VoxelMap::capacity()
{
    return 9;
}

size_t VoxelMap::size()
{
    //
    // return local_map_pointcloud->getLeafCount();
    return local_map_pointcloud->size();
}

void VoxelMap::trim_map()
{
    std::vector<diviner::PointStamped, Eigen::aligned_allocator<diviner::PointStamped>> voxel_centers;
    pcl::PointCloud<diviner::PointStamped>::Ptr cloud(new pcl::PointCloud<diviner::PointStamped>);

    std::cout << "  - map: Number of points in map before trimming: " << local_map_pointcloud->size() << std::endl;
    
    pcl::VoxelGrid<diviner::PointStamped> filter;
    filter.setInputCloud(local_map_pointcloud);
    filter.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    // filter.setMinimumPoint sNumberPerVoxel(params_.min_points_per_voxel);
    filter.filter(*local_map_pointcloud);

    std::cout << "  - map: Number of points in map after trimming: " << local_map_pointcloud->size() << std::endl;

    if(params_.debug)
    {
        std::cout << "  - map: Removing points outside of " << params_.max_distance << " meters." << std::endl;        
    }

}

}