#include <aligner/pcl_aligner.hpp>

namespace diviner
{

void PclAligner::initialize(geometry_msgs::PoseStamped veh_pose)
{
    // Initialization function for diviner first startup
    std::cout << "  - aligner: Inside aligner initialize function." << std::endl;

    if(false)
    {
        updated_vehicle_position.push_back(veh_pose);
    }
    else
    {
        std::cout << "  - aligner: Setting initial point." << std::endl;
        // set beginning position to (0,0,0) and use as the beginning
        veh_pose.pose.position.x = 0;
        veh_pose.pose.position.y = 0;
        veh_pose.pose.position.z = 0;
        veh_pose.pose.orientation.x = 0;
        veh_pose.pose.orientation.y = 0;
        veh_pose.pose.orientation.z = 0;
        veh_pose.pose.orientation.w = 0;

        veh_pose.header.stamp = ros::Time::now();

        updated_vehicle_position.push_back(veh_pose);
        std::cout << "  - aligner: Added pose to vehicle location vector." << std::endl;
    }
}

void PclAligner::align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_, std::shared_ptr<diviner::IMap> map_)
{
    // The Iterative Closest Point algorithm
    // pulled from pcl icp tutorial...

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    int iterations = 25;

    icp.setMaximumIterations(params_.num_iterations);
    icp.setInputSource(point_cloud_);
    icp.setInputTarget(map_->get_data());
    icp.align(*point_cloud_);

    std::cout << "  - aligner: Applied " << params_.num_iterations << " ICP iteration(s)" << std::endl;

    if(icp.hasConverged())
    {
        std::cout << "  - aligner: ICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "  - aligner: ICP transformation " << params_.num_iterations << " : cloud_icp -> local_map" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }

    // for(const auto point : point_cloud_)
    // {
    //     // add offset to points to match with map location
    // }
}

void PclAligner::add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_)
{
    // add new points to local map
    if(true)
    {
        std::cout << "  - aligner: Adding points in aligner" << std::endl;
        std::cout << "  - aligner: Adding " << point_cloud->size() << " points to map." << std::endl;
    }

    std::cout << "  - aligner: added point cloud to Octree" << std::endl;

    // map_ = std::make_shared<pcl::octree::OctreePointCloud<diviner::PointStamped>>();
    // map_->setInputCloud(point_cloud);

}

void apply_transform()
{
    //
    
}

void PclAligner::find_tf()
{
    // // Broadcast the tf ---
    // if (publish_odom_tf_) {
    //     geometry_msgs::msg::TransformStamped transform_msg;
    //     transform_msg.header.stamp = header.stamp;
    //     if (invert_odom_tf_) {
    //         transform_msg.header.frame_id = moving_frame;
    //         transform_msg.child_frame_id = lidar_odom_frame_;
    //         transform_msg.transform = tf2::sophusToTransform(pose.inverse());
    //     } else {
    //         transform_msg.header.frame_id = lidar_odom_frame_;
    //         transform_msg.child_frame_id = moving_frame;
    //         transform_msg.transform = tf2::sophusToTransform(pose);
    //     }
    //     tf_broadcaster_->sendTransform(transform_msg);
    // }

    // // publish odometry msg
    // nav_msgs::msg::Odometry odom_msg;
    // odom_msg.header.stamp = header.stamp;
    // odom_msg.header.frame_id = lidar_odom_frame_;
    // odom_msg.child_frame_id = moving_frame;
    // odom_msg.pose.pose = tf2::sophusToPose(pose);
    // odom_msg.pose.covariance.fill(0.0);
    // odom_msg.pose.covariance[0] = position_covariance_;
    // odom_msg.pose.covariance[7] = position_covariance_;
    // odom_msg.pose.covariance[14] = position_covariance_;
    // odom_msg.pose.covariance[21] = orientation_covariance_;
    // odom_msg.pose.covariance[28] = orientation_covariance_;
    // odom_msg.pose.covariance[35] = orientation_covariance_;
    // odom_publisher_->publish(std::move(odom_msg));
}

void PclAligner::update_curr_pose(const diviner::alignment icp_alignment)
{
    // Update the position based off of the ICP changes

    geometry_msgs::PoseStamped previous_pose;
    geometry_msgs::PoseStamped new_pose_holder;

    new_pose_holder.pose.position.x = previous_pose.pose.position.x + icp_alignment.x;
    new_pose_holder.pose.position.y = previous_pose.pose.position.y + icp_alignment.y;
    new_pose_holder.pose.position.z = previous_pose.pose.position.z + icp_alignment.z;
    new_pose_holder.pose.orientation.x = previous_pose.pose.orientation.x + icp_alignment.roll;
    new_pose_holder.pose.orientation.y = previous_pose.pose.orientation.y + icp_alignment.pitch;
    new_pose_holder.pose.orientation.z = previous_pose.pose.orientation.z + icp_alignment.yaw;
    new_pose_holder.pose.orientation.w = previous_pose.pose.orientation.w + icp_alignment.w;

    new_pose_holder.header.stamp = ros::Time::now();

    updated_vehicle_position.push_back(new_pose_holder);
}

}