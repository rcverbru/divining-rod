#include <utils/syncer.hpp>

/*
GPS: 20 Hz -> vehicle_poses_queue_
Lidar: 10 Hz -> cloud
*/

namespace diviner
{

diviner::SyncedMsgs Syncer::sync(std::queue<pcl::PointCloud<diviner::PointStamped>> &cloud_queue_, std::queue<geometry_msgs::PoseStamped> &vehicle_poses_queue_)
{
    diviner::SyncedMsgs synced_out;
    
    double curr_time_diff, prev_time_diff;
    bool synced = false;

    // Set prev_time_diff to very high so it stops freaking out in the first run
    prev_time_diff = 1000000;

    // We're syncing only based off of one cloud scan so we can pull the first off into its own variable
    cloud = cloud_queue_.front();
    cloud_queue_.pop();

    cloud_timestamp.fromNSec(cloud.header.stamp);
    
    // std::cout << cloud_timestamp << std::endl;

    // Adding current cloud to synced out
    pcl::PointCloud<diviner::PointStamped>::Ptr cloud_ptr(new pcl::PointCloud<diviner::PointStamped>);
    *cloud_ptr = cloud;
    synced_out.cloud = cloud_ptr;

    // if(params_.debug)
    // {
    //     std::cout << "  - syncer: Cloud Queue Scan size = " << cloud_queue_.front().size() << std::endl;
    //     std::cout << "  - syncer: Cloud size = " << cloud.size() << std::endl;    
    // }

    while(!synced)
    {
        // std::cout << "  - syncer: In while loop" << std::endl;
        
        if(!cloud.empty())
        {
            // std::cout << "  - syncer: In cloud if statement" << std::endl;

            if(!vehicle_poses_queue_.empty())
            {
                

                if(params_.debug)
                {
                    std::cout << "  - syncer: In vehicle poses if statement" << std::endl;
                    std::cout << "  - syncer: vehicle poses queue size = " << vehicle_poses_queue_.size() << std::endl;
                    std::cout << "  - syncer: cloud stamp: " << cloud_timestamp << std::endl;
                    std::cout << "  - syncer: vehicle poses stamp: " << vehicle_poses_queue_.front().header.stamp << std::endl;
                    // std::cout << "  - syncer: max sync error in seconds = " << params_.max_sync_err_s << std::endl;
                    // std::cout << "  - syncer: cloud stamp upper = " << std::setprecision(20) << (cloud_timestamp.toSec() + params_.max_sync_err_s) << std::endl;
                    // std::cout << "  - syncer: cloud stamp lower = " << std::setprecision(20) << (cloud_timestamp.toSec() - params_.max_sync_err_s) << std::endl;
                }
                
                // If negative vehicle pose is newer than cloud stamp
                curr_time_diff = cloud_timestamp.toSec() - vehicle_poses_queue_.front().header.stamp.toSec();
                
                if(params_.debug)
                {
                    std::cout << "  - syncer: current sync time diff in seconds = " << curr_time_diff << std::endl;
                }

                if(curr_time_diff <= params_.max_sync_err_s && curr_time_diff >= -(params_.max_sync_err_s))
                {
                    if(params_.debug)
                    {
                        std::cout << "  - syncer: within threshold range" << std::endl;
                    }

                    // if(params_.debug)
                    // {
                    //     std::cout << "  - syncer: cloud queue before pop: " << cloud_queue_.size() << std::endl;
                    //     std::cout << "  - syncer: cloud timestamp front before pop: " << cloud_timestamp << std::endl;
                    //     std::cout << "  - syncer: gps queue before pop: " << vehicle_poses_queue_.size() << std::endl;
                    //     std::cout << "  - syncer: gps timestamp front before pop: " << vehicle_poses_queue_.front().header.stamp << std::endl;
                    //     std::cout << "  - syncer: vehicle queue x = " << vehicle_poses_queue_.front().pose.position.x << std::endl;
                    //     std::cout << "  - syncer: synced queue x = " << synced_out.gps.pose.position.x << std::endl;
                    // }
                        
                    if(std::abs(curr_time_diff) > std::abs(prev_time_diff))
                    {
                        if(params_.debug)
                        {
                            std::cout << "  - syncer: Current timestamp = " << curr_time_diff << std::endl;
                            std::cout << "  - syncer: Previous timestamp = " << prev_time_diff << std::endl;
                            std::cout << "  - syncer: Current timestamp is closest. Cloud and gps is synced." << std::endl;
                        }
                        
                        synced = true;
                    }
                    else
                    {
                        //Push to synced queue
                        synced_out.gps = vehicle_poses_queue_.front();
                        prev_time_diff = curr_time_diff;

                        // setup for next run through
                        vehicle_poses_queue_.pop();
                    }
                    
                    // if(params_.debug)
                    // {
                    //     std::cout << "  - syncer: Popping the cloud and gps" << std::endl;
                    //     std::cout << "  - syncer: cloud queue after pop: " << cloud_queue_.size() << std::endl;
                    //     std::cout << "  - syncer: gps queue after pop: " << vehicle_poses_queue_.size() << std::endl;
                    //     // std::cout << "  - syncer: gps timestamp front after pop: " << vehicle_poses_queue_.front().header.stamp << std::endl;
                    // }
                }
                else if(curr_time_diff > params_.max_sync_err_s)
                {
                    //Pop off the front vehicle_poses_queue_ item
                    vehicle_poses_queue_.pop();
                    if(params_.debug)
                    {
                        std::cout << "  - syncer: Current time difference is very positive. Popping ONLY the gps" << std::endl;
                    }
                }
                else if(curr_time_diff < -(params_.max_sync_err_s))
                {
                    if(params_.debug)
                    {
                        std::cout << "  - syncer: Current time difference is very negative." << std::endl;
                        std::cout << "  - syncer: Current timestamp = " << curr_time_diff << std::endl;
                        std::cout << "  - syncer: Previous timestamp = " << prev_time_diff << std::endl;
                        std::cout << "  - syncer: Previous timestamp is closest. Cloud and gps is synced." << std::endl;        
                    }

                    synced = true;
                }
                else
                {
                    std::cout << "  - syncer: IDK man the threshold ain't there bud..." << std::endl;
                }
            }
            else
            {
                // Exit condition for if there are no vehicle poses
                // std::cout << "  - syncer: No vehicle poses available... passing empty vehicle pose" << std::endl;
                synced = true;
                geometry_msgs::PoseStamped new_pose;

                // set beginning position to (0,0,0) and use as the beginning
                new_pose.pose.position.x = 0;
                new_pose.pose.position.y = 0;
                new_pose.pose.position.z = 0;
                new_pose.pose.orientation.x = 0;
                new_pose.pose.orientation.y = 0;
                new_pose.pose.orientation.z = 0;
                new_pose.pose.orientation.w = 1;
        
                new_pose.header.stamp = cloud_timestamp;
            }
        }
    }
    
    if(params_.debug)
    {
        std::cout << "  - syncer: synced veh pose stamp = " << synced_out.gps.header.stamp << std::endl;
        std::cout << "  - syncer: synced cloud stamp = " << synced_out.cloud->header.stamp << std::endl;            
    }
    
    return synced_out;
}

}

