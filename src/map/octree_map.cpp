#include <map/octree_map.hpp>

namespace diviner
{

void OctreeMap::add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_)
{
    //
    // if(params_.debug)
    // {
        std::cout << "  - map: Adding point cloud to map " << std::endl;
    // }

    std::cout << "  - map: Number of points in pointcloud: " << point_cloud_->size() << std::endl;
    std::cout << "  - map: Number of voxels in map before adding: " << local_map_octree->getLeafCount() << std::endl;

    local_map_octree->setInputCloud(point_cloud_);
    local_map_octree->addPointsFromInputCloud();

    std::cout << "  - map: Number of voxels in octree " << local_map_octree->getLeafCount() << std::endl;

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr data_cloud_(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "before indicies loop" << std::endl;

    for(auto &point : local_map_octree->getIndices())
    {
        //
        std::cout << "In get indicies loop" << std::endl;
    }

    // Create a single point
    diviner::PointStamped point;
    point.x = 1.0;  // Set the x-coordinate
    point.y = 2.0;  // Set the y-coordinate
    point.z = 3.0;  // Set the z-coordinate
    point.intensity = 50.0;  // Set the intensity

    // Add the point to the cloud
    data_cloud_->points.push_back(point);

    return data_cloud_;
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
    return 9;
}

void OctreeMap::trim_map()
{
    // remove points over a certain distance away from the ego vehicle.
    // pcl::octree::AlignedPointTVector voxel_centers;

    // pcl::octree::AlignedPointTVector<diviner::PointStamped> voxel_centers;>
    std::vector<diviner::PointStamped, Eigen::aligned_allocator<diviner::PointStamped>> voxel_centers;
    // std::vector<Eigen::Vector3f> voxel_centers;

    auto num_voxels = local_map_octree->getOccupiedVoxelCenters(voxel_centers);
    std::cout << "  - map: Number of voxels in use " << num_voxels << std::endl;

    // for(int i = 0; i < num_voxels; i++)
    // {
        // 
        // test_indices = local_map_octree->getIndices();
        // std::cout << "  - map: Number of Indices in Octree " << test_indices->size() << std::endl;
    // }

    if(params_.debug)
    {
        std::cout << "  - map: Removing points outside of " << params_.max_distance << " meters." << std::endl;        
    }

}

}