#ifndef UTILS__SYNCER_HPP
#define UTILS__SYNCER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>

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
    double max_sync_err_s = 0.05;
    bool debug = true;
};

class Syncer
{
    public:
        explicit Syncer(const SyncerParams & params) : params_(params){};
        ~Syncer() = default;
        
        diviner::SyncedMsgs sync(std::queue<pcl::PointCloud<diviner::PointStamped>> &cloud_, std::queue<geometry_msgs::PoseStamped> &vehicle_poses_queue_);
    
    private:
        SyncerParams params_;


        ros::Time cloud_timestamp;
        double prev_time_diff;

        pcl::PointCloud<diviner::PointStamped> cloud;


};

}

#endif