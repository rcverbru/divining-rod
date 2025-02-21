#ifndef UTILS__SYNCER_HPP
#define UTILS__SYNCER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>

namespace diviner
{

struct SyncerParams
{
    //
    bool debug = true;
};

class Syncer
{
    public:
        explicit Syncer(const SyncerParams & params) : params_(params){};
        ~Syncer() = default;
        
        // I want to make this the "wrapper" for converting msgs.
        diviner::synced_msgs sync(std::queue<pcl::PointCloud<diviner::PointStamped>::Ptr>* cloud_, std::queue<geometry_msgs::PoseStamped>& vehicle_poses_queue_);
    
    private:
        SyncerParams params_;

};

}

#endif