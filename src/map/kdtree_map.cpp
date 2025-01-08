#include <map/kdtree_map.hpp>

namespace diviner
{

// void KdTreeMap::add_points(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_)
// {
//     // add new points to local map
//     if(true)
//     {
//         std::cout << "  - mapper: Adding points in mapper" << std::endl;
//         std::cout << "  - mapper: Adding " << point_cloud->size() << " points to map." << std::endl;
//     }

//     kd_tree.setInputCloud(point_cloud);
//     std::cout << "  - mapper: added point cloud to test kdtree" << std::endl;

//     // map_ = std::make_shared<pcl::KdTreeFLANN<diviner::PointStamped>>();
//     // map_->setInputCloud(point_cloud);

// }

void KdTreeMap::trim_map()
{
    // remove points over a certain distance away from the ego vehicle.
    if(params_.debug)
    {
        std::cout << "  - mapper: Removing points " << params_.max_distance << " meters away." << std::endl;
    }

    // for(const auto point : point_cloud)
    // {
    //     // find points past max distance then remove them
    // }

}



}