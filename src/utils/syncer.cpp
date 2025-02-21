#include <utils/syncer.hpp>

/*
GPS: 20 Hz -> vehicle_poses_queue_
Lidar: 10 Hz -> cloud_
*/

namespace diviner
{

    diviner::synced_msgs Syncer::sync(std::queue<pcl::PointCloud<diviner::PointStamped>::Ptr>* cloud_, std::queue<geometry_msgs::PoseStamped>& vehicle_poses_queue_)  {

        uint64_t vpq_time_stamp = vehicle_poses_queue_.front().header.stamp.toNSec();
        diviner::synced_msgs synced_out;

        // !TODO! make this threshold a param that we can pass in through the launch file
        uint64_t threshold = 500000000;
        

        if(!cloud_->empty()){

            if(params_.debug){
                std::cout << "syncer: cloud stamp: " << cloud_->front()->header.stamp << std::endl;
                std::cout << "syncer: vehicle poses stamp: " << vehicle_poses_queue_.front().header.stamp.toNSec() << std::endl;
            }
            
            if (cloud_->front()->header.stamp <= vpq_time_stamp + threshold && cloud_->front()->header.stamp >= vpq_time_stamp - threshold)  {
                
                //Push to new synced queue/vector
                synced_out.cloud = cloud_->front();
                synced_out.gps.header.stamp = vehicle_poses_queue_.front().header.stamp;

                if(params_.debug)
                {
                
                std::cout << "syncer: cloud queue before pop: " << cloud_->size() << std::endl;
                std::cout << "syncer: cloud timestamp front before pop: " << cloud_->front()->header.stamp << std::endl;
                std::cout << "syncer: gps queue before pop: " << vehicle_poses_queue_.size() << std::endl;
                std::cout << "syncer: gps timestamp front before pop: " << vehicle_poses_queue_.front().header.stamp << std::endl;
                
                }

                cloud_->pop();
                vehicle_poses_queue_.pop();
                if(params_.debug)
                {
                std::cout << "Popping the cloud and gps" << std::endl
                }

                if(params_.debug)
                {
                
                    std::cout << "syncer: cloud queue after pop: " << cloud_->size() << std::endl;
                    if(!cloud_->empty()){
                        std::cout << "syncer: cloud timestamp front after pop: " << cloud_->front()->header.stamp << std::endl;
                    }
                    std::cout << "syncer: gps queue after pop: " << vehicle_poses_queue_.size() << std::endl;
                    std::cout << "syncer: gps timestamp front after pop: " << vehicle_poses_queue_.front().header.stamp << std::endl;

                }

            } else {

                //Pop off the front vehicle_poses_queue_ item
                vehicle_poses_queue_.pop();
                if(params_.debug){
                    std::cout << "Popping ONLY the gps" << std::endl;
                }

            }
            
        }  else  {

            std::cout << "syncer: Need to wait for new scan..." << std::endl;

        }

        return synced_out;
    }

}

