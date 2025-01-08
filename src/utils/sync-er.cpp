#include <utils/sync-er.hpp>

namespace diviner
{

bool Syncer::sync(pcl::PointCloud<diviner::PointStamped>::Ptr cloud_, std::queue<geometry_msgs::PoseStamped> vehicle_poses_queue_)
{
    // I'm going to commit arson...
    if(cloud_ != nullptr)
    {
        std::cout << "sync-er: cloud stamp" << cloud_->header.stamp << std::endl;
    }
    else
    {
        std::cout << "sync-er: Need to wait for new scan..." << std::endl;
    }
}

}