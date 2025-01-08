#include <diviner/diviner.hpp>

#include <vector>

namespace diviner
{

void Diviner::step(pcl::PointCloud<diviner::PointStamped>::Ptr cloud, geometry_msgs::TransformStamped gnss_to_map_, geometry_msgs::TransformStamped cloud_to_vehicle, std::vector<geometry_msgs::PoseStamped> veh_pose)
{
    if(debug_)
    {
        std::cout << "- diviner: Diviner is running in Debug mode" << std::endl;
    }
    if(first_scan)
    {
        // Start mapper location
        geometry_msgs::PoseStamped holder;
        aligner_->initialize(holder);
    }

    if(debug_)
    {
        //
        std::cout << "- diviner: number of poses: " << veh_pose.size() << std::endl;
        // std::cout << "Current Transform:" << std::endl;
        // std::cout << transform;
    }

    if(veh_pose.size() == 3)
    {
        vestimator_->estimate(velocities, gnss_to_map_, veh_pose);

        if(debug_)
        {
            std::cout << "- diviner: Finished estimating velocity" << std::endl;
        }
    }
    else
    {
        std::cout << "- diviner: Need to wait for " << 3 - veh_pose.size() << " more iterations before estimating velocity." << std::endl;
    }
    // vestimator_->publish_transform();

    if(velocities.size() == 3)
    {
        deskewer_->deskew(cloud, velocities);

        if(debug_)
        {
            // Move deskewed data into its own pointer that we can then publish for visualization
            // *deskewed_cloud = *cloud;
            std::cout << "- diviner: Done deskewing" << std::endl;
        }
    }
    else
    {
        std::cout << "- diviner: Need to wait for " << 3 - velocities.size() << " more iterations before deskewing cloud." << std::endl;
    }

    int num_points = cloud->size();
    std::cout << "- diviner: num points in point cloud: " << num_points << std::endl;
    // First downsampling
    filter_->filter(cloud);

    num_points = cloud->size();
    std::cout << "- diviner: num points in point cloud after filter: " << num_points << std::endl;

    // Second Downsampling
    // filter_->filter(cloud);

    // num_points = cloud->size();
    // std::cout << "- diviner: num points in point cloud after 2nd filter: " << num_points << std::endl;

    if(debug_)
    {
        std::cout << "- diviner: After filters" << std::endl;        
    }

    if(!first_scan)
    {
        // Align the points to the map inorder figure out the location
        aligner_->align(cloud, map_);

        if(debug_)
        {
            std::cout << "- diviner: After aligner" << std::endl;
        }
    }
    else
    {
        std::cout << "- diviner: Skipping aligner->align() due to nothing to align" << std::endl;
    }

    filter_->filter(cloud);

    // Add points to map
    map_->add_cloud(cloud);

    if(debug_)
    {
        std::cout << "- diviner: Added points to the map" << std::endl;
    }

    // Update our current position 
    aligner_->update_curr_pose(vehicle_alignment);

    // Set up car tf to publish to tf topic
    aligner_->find_tf();

    if(debug_)
    {
        std::cout << "- diviner: Completed tf computation" << std::endl;
    }

    // Remove points outside of a given radius
    map_->trim_map();

    // Set up for next step
    first_scan = false;

    std::cout << "- diviner: Done with stepping" << std::endl;
}

}