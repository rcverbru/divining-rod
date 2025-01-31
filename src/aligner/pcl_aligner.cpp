#include <aligner/pcl_aligner.hpp>

namespace diviner
{

void PclAligner::initialize(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose)
{
    // Initialization function for diviner first startup
    std::cout << "  - aligner: Inside aligner initialize function." << std::endl;

    if(false)
    {
        std::cout << "  - aligner: Pulling GPS location." << std::endl;
        // need to bring in gps_pose queue and slap that boy in here
        //veh_pose.push_back(gps_pose);
        std::cout << "  - aligner: Added GPS pose to vehicle location vector." << std::endl;
    }
    else
    {
        std::cout << "  - aligner: Setting initial point." << std::endl;
        geometry_msgs::PoseStamped new_pose;

        // set beginning position to (0,0,0) and use as the beginning
        new_pose.pose.position.x = 0;
        new_pose.pose.position.y = 0;
        new_pose.pose.position.z = 0;
        new_pose.pose.orientation.x = 0;
        new_pose.pose.orientation.y = 0;
        new_pose.pose.orientation.z = 0;
        new_pose.pose.orientation.w = 0;

        new_pose.header.stamp = ros::Time::now();

        veh_pose->push_back(new_pose);
        std::cout << "  - aligner: Added 0 starting pose to vehicle location vector." << std::endl;
    }
}

Eigen::Matrix4d PclAligner::align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_, std::shared_ptr<diviner::IMap> map_)
{
    // The Iterative Closest Point algorithm
    // pulled from pcl icp tutorial...

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    // int iterations = 25;

    // std::cout << "  - aligner: " << point_cloud_->front() << std::endl;

    // need to set this to be a loop until convergence score is below a certain value

    if(params_.alignment_state == "set")
    {
        // Set static number of iterations
        icp.setMaximumIterations(params_.num_iterations);
        icp.setInputSource(point_cloud_);
        icp.setInputTarget(map_->get_data());
        icp.align(*point_cloud_);

        if(params_.debug)
        {
            std::cout << "  - aligner: Applied " << params_.num_iterations << " ICP iteration(s)" << std::endl;
        }
        // std::cout << "  - aligner: " << point_cloud_->front() << std::endl;

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
    }
    else if(params_.alignment_state == "automatic")
    {
        // Needs to be fixed cuz it erroring out
        int iterations_count = 0;
        icp.setInputSource(point_cloud_);
        icp.setInputTarget(map_->get_data());

        if(params_.convergence_criterion != 0)
        {
            while(icp.getFitnessScore() > params_.convergence_criterion)
            {
                icp.align(*point_cloud_);

                iterations_count++;

                if(iterations_count == params_.max_num_iterations)
                {
                    std::cout << "  - aligner: Reached max number of iterations. Stopping alignment..." << std::endl;
                    // break;
                }
            }
        }
        else
        {
            std::cout << "  - aligner: Fix the convergence criterion. " << std::endl;
        }

        if(params_.debug)
        {
            std::cout << "  - aligner: Applied " << iterations_count << " ICP iteration(s)" << std::endl;
        }

        if(icp.hasConverged())
        {
            std::cout << "  - aligner: ICP has converged, score is " << icp.getFitnessScore() << std::endl;
            std::cout << "  - aligner: ICP transformation " << iterations_count << " : cloud_icp -> local_map" << std::endl;
            transformation_matrix = icp.getFinalTransformation().cast<double>();
            print4x4Matrix (transformation_matrix);
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }

    }
    else
    {
        // std::cout << "  - aligner: Aligner run state not set" << std::endl;
        PCL_ERROR("\nAligner run state not set.\n");
    }

    return transformation_matrix;
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

// void PclAligner::update_points(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, diviner::Alignment vehicle_alignment)
// {
//     //
// }

void PclAligner::update_curr_pose(const diviner::Alignment icp_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose)
{
    // Update the position based off of the ICP changes
    if(params_.debug)
    {
        std::cout << "  - aligner: Updating Current Pose" << std::endl;
    }
    
    geometry_msgs::PoseStamped previous_pose;
    previous_pose = veh_pose->back();

    if(params_.debug)
    {
        std::cout << "  - aligner: Previous pose is (x = " << previous_pose.pose.position.x 
        << ", y = " << previous_pose.pose.position.y 
        << ", z = " << previous_pose.pose.position.z << ")"
        << std::endl;

        std::cout << "  - aligner: Pulled previous pose. Updating current pose..." << std::endl;
    }


    geometry_msgs::PoseStamped new_pose;

    new_pose.pose.position.x = previous_pose.pose.position.x + icp_alignment.transform.transform.translation.x;
    new_pose.pose.position.y = previous_pose.pose.position.y + icp_alignment.transform.transform.translation.y;
    new_pose.pose.position.z = previous_pose.pose.position.z + icp_alignment.transform.transform.translation.z;
    new_pose.pose.orientation.x = previous_pose.pose.orientation.x + icp_alignment.transform.transform.rotation.x;
    new_pose.pose.orientation.y = previous_pose.pose.orientation.y + icp_alignment.transform.transform.rotation.y;
    new_pose.pose.orientation.z = previous_pose.pose.orientation.z + icp_alignment.transform.transform.rotation.z;
    new_pose.pose.orientation.w = previous_pose.pose.orientation.w + icp_alignment.transform.transform.rotation.w;

    new_pose.header.stamp = ros::Time::now();

    if(params_.debug)
    {
        std::cout << "  - aligner: New pose is (x = " << previous_pose.pose.position.x 
        << ", y = " << new_pose.pose.position.y 
        << ", z = " << previous_pose.pose.position.z << ")"
        << std::endl;
    }

    veh_pose->push_back(new_pose);

    // if(veh_pose->size() > 3)
    // {
    //     veh_pose->pop();
    // }
}

}