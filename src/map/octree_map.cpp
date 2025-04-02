#include <map/octree_map.hpp>

namespace diviner
{

void OctreeMap::add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_)
{
    if(params_.debug)
    {
        std::cout << "  - map: Adding point cloud to map " << std::endl;
    }

    // std::cout << "  - map: " << local_map_octree->octreeCanResize() << std::endl;

    if(params_.debug)
    {
        std::cout << "  - map: Number of points in pointcloud: " << point_cloud_->size() << std::endl;
        std::cout << "  - map: Number of voxels in map before adding: " << local_map_octree->getLeafCount() << std::endl;    
    }

    local_map_octree->setInputCloud(point_cloud_);
    local_map_octree->addPointsFromInputCloud();

    for(auto point : *point_cloud_)
    {
        local_map_pointcloud->push_back(point);
    }

    if(params_.debug)
    {
        std::cout << "  - map: Num points in map cloud: " << local_map_pointcloud->size() << std::endl;
        std::cout << "  - map: Number of voxels in octree " << local_map_octree->getLeafCount() << std::endl;    
    }

    if(params_.debug)
    {
        for(int i = 0; i<local_map_octree->getTreeDepth(); i++)
        {
            std::cout << "  - map: Depth: " << i << " Size of Voxel: " << local_map_octree->getVoxelSquaredDiameter(i) << std::endl;
        }

        std::cout << "  - map: Resolution: " << local_map_octree->getResolution() << std::endl;
    }

}

pcl::PointCloud<diviner::PointStamped>::Ptr OctreeMap::get_data()
{
    return local_map_pointcloud;
}

void OctreeMap::get_voxels()
{
    //
}

void OctreeMap::clear_map()
{
    //
    local_map_octree->deleteTree();
}

int OctreeMap::capacity()
{
    //
    return 9;
}

size_t OctreeMap::size()
{
    //
    // return local_map_octree->getLeafCount();
    return local_map_pointcloud->size();
}

void OctreeMap::trim_map()
{
    // remove points over a certain distance away from the ego vehicle.
    // pcl::octree::AlignedPointTVector voxel_centers;

    // pcl::octree::AlignedPointTVector<diviner::PointStamped> voxel_centers;>
    std::vector<diviner::PointStamped, Eigen::aligned_allocator<diviner::PointStamped>> voxel_centers;
    // std::vector<Eigen::Vector3f> voxel_centers;
    pcl::PointCloud<diviner::PointStamped>::Ptr cloud(new pcl::PointCloud<diviner::PointStamped>);

    auto num_voxels = local_map_octree->getOccupiedVoxelCenters(voxel_centers);
    
    if(params_.debug)
    {
        std::cout << "  - map: Number of voxels in use " << num_voxels << std::endl;
    }

    for(int i = 0; i < num_voxels; i++)
    {
        std::cout << "  - map: Voxel " << i << " has center at: " << voxel_centers[i].x << ", " << voxel_centers[i].y << ", " << voxel_centers[i].z << std::endl;
        cloud->push_back(voxel_centers[i]);
        std::cout << "  - map: cloud size: " << cloud->size() << std::endl;
    }

    if(params_.debug)
    {
        std::cout << "  - map: Removing points outside of " << params_.max_distance << " meters." << std::endl;        
    }

}

}