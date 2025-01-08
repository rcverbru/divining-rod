#include <map/hash_map.hpp>

namespace diviner
{

// void HashMap::trim_map(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud)
void HashMap::trim_map()
{
    // Map trimmer function for removing points a certain distance away from the vehicle

    if(params_.debug)
    {
        std::cout << "Deleting the map" << std::endl;        
    }

    // for(const auto point : point_cloud)
    // {
    //     //
    // }
}

} // namespace diviner
