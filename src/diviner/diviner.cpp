#include <diviner/diviner.hpp>

#include <vector>

namespace diviner
{

void Diviner::step(pcl::PointCloud<diviner::PointStamped>::Ptr cloud, geometry_msgs::TransformStamped gnss_to_map_, geometry_msgs::TransformStamped cloud_to_vehicle, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> &veh_pose)
{
    if(debug_)
    {
        std::cout << "- diviner: Diviner is running in Debug mode" << std::endl;
    }
    if(first_scan)
    {
        // Start mapper location
        geometry_msgs::PoseStamped holder;
        aligner_->initialize(veh_pose);
    }

    if(debug_)
    {
        //
        std::cout << "- diviner: number of poses: " << veh_pose->size() << std::endl;
        // std::cout << "Current Transform:" << std::endl;
        // std::cout << transform;
    }

    if(veh_pose->size() == 3)
    {
        vestimator_->estimate(velocities, gnss_to_map_, *veh_pose);

        if(debug_)
        {
            std::cout << "- diviner: Finished estimating velocity" << std::endl;
        }

        // Need to make a vehicle position prediction to compare to point cloud alignment
        //vestimator_->predict(pred_veh_pose);
    }
    else
    {
        std::cout << "- diviner: Need to wait for " << 3 - veh_pose->size() << " more iterations before estimating velocity." << std::endl;
    }

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

    // holder idea for dsor if we feel like it (doubtful but the idea is here)
    // preprocessor_->process();

    int num_points = cloud->size();
    std::cout << "- diviner: num points in point cloud: " << num_points << std::endl;
    
    // First downsampling for alignment
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
        alignment_holder = aligner_->align(cloud, map_);

        if(debug_)
        {
            std::cout << "- diviner: Translation vector is (x = " << alignment_holder.translation.x 
            << ", y = " << alignment_holder.translation.y 
            << ", z = " << alignment_holder.translation.z << ")"
            << std::endl;
            std::cout << "- diviner: After aligner" << std::endl;
        }
    }
    else
    {
        std::cout << "- diviner: Skipping aligner->align() due to nothing to align" << std::endl;
    }

    // Second downsampling for map
    filter_->filter(cloud);

    // update point pose with alignment info
    // aligner_->update_points(cloud, vehicle_alignment);

    // Add points to map
    map_->add_cloud(cloud);

    if(debug_)
    {
        std::cout << "- diviner: Added points to the map" << std::endl;
    }

    // kahlman filter for position estimation smoothing
    // blender_->smoothie();

    // Update our current position 
    aligner_->update_curr_pose(alignment_holder, veh_pose);

    if(debug_)
    {
        std::cout << "- diviner: Number of vehicle_positions: " << veh_pose->size() << std::endl;
    }

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