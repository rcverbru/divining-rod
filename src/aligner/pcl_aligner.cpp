#include <aligner/pcl_aligner.hpp>

#include <pcl/io/pcd_io.h>
#include <iostream>

namespace diviner
{

void PclAligner::initialize(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose)
{
    // Initialization function for diviner first startup
    if(params_.debug)
    {
        std::cout << "  - aligner: Inside aligner initialize function." << std::endl;
    }

    if(false)
    {
        if(params_.debug)
        {
            std::cout << "  - aligner: Pulling GPS location." << std::endl;
        }

        // need to bring in gps_pose queue and slap that boy in here
        //veh_pose.push_back(gps_pose);
        
        if(params_.debug)
        {
            std::cout << "  - aligner: Added GPS pose to vehicle location vector." << std::endl;
        }
    }
    else
    {
        if(params_.debug)
        {
            std::cout << "  - aligner: Setting initial point." << std::endl;
        }

        geometry_msgs::PoseStamped new_pose;

        // set beginning position to (0,0,0) and use as the beginning
        new_pose.pose.position.x = 0;
        new_pose.pose.position.y = 0;
        new_pose.pose.position.z = 0;
        new_pose.pose.orientation.x = 0;
        new_pose.pose.orientation.y = 0;
        new_pose.pose.orientation.z = 0;
        new_pose.pose.orientation.w = 1;

        new_pose.header.stamp = ros::Time::now();

        veh_pose->emplace(veh_pose->begin(), new_pose);
        
        if(params_.debug)
        {
            std::cout << "  - aligner: Added 0 starting pose to vehicle location vector." << std::endl;
        }
    }
}

AlignmentTuple PclAligner::align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_, std::shared_ptr<diviner::IMap> map_)
{
    // The Iterative Closest Point algorithm
    // pulled from pcl icp tutorial...

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    // int iterations = 25;

    if(params_.debug)
    {
        std::cout << "  - aligner: Point Cloud Size = " << point_cloud_->size() << std::endl;
    }

    std::cout << "  - aligner: Saving updated scan PCD" << std::endl;
    pcl::io::savePCDFileASCII ("/home/rcv/dev/rse/localization_ws/start_pcd.pcd", *point_cloud_);

    if(params_.alignment_state == "set")
    {
        // Set static number of iterations
        icp.setMaximumIterations(params_.num_iterations);
        // icp.setEuclideanFitnessEpsilon(params_.euc_fit_epsilon);
        // icp.setTransformationEpsilon(params_.transform_epsilon);
        // icp.setMaxCorrespondenceDistance(params_.corr_dist);
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
            if(params_.debug)
            {
                std::cout << "  - aligner: ICP has converged, score is " << icp.getFitnessScore() << std::endl;
                std::cout << "  - aligner: ICP transformation " << params_.num_iterations << " : cloud_icp -> local_map" << std::endl;    
            }

            transformation_matrix = icp.getFinalTransformation().cast<double>();
            
            if(params_.debug)
            {
                print4x4Matrix (transformation_matrix);
            }
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }
    }
    else if(params_.alignment_state == "automatic")
    {
        if(params_.debug)
        {
            std::cout << "  - aligner: Automatic alignment loop" << std::endl;
        }

        int iterations_count = 0;
        icp.setInputSource(point_cloud_);
        icp.setInputTarget(map_->get_data());

        // std::cout << "  - aligner: current num iterations is " << iterations_count << std::endl;

        if(params_.convergence_criterion != 0)
        {
            // std::cout << "  - aligner: in if piece" << std::endl;

            do
            {
                // std::cout << "  - aligner: in while loop" << std::endl;
                icp.align(*point_cloud_);

                iterations_count++;

                if(params_.debug)
                {
                    std::cout << "  - aligner: Try " << iterations_count << " has current score " << icp.getFitnessScore() << std::endl;
                }
            }
            while(icp.getFitnessScore() > params_.convergence_criterion && iterations_count < params_.max_num_iterations);
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
            if(params_.debug)
            {
                std::cout << "  - aligner: ICP has converged, score is " << icp.getFitnessScore() << std::endl;
                std::cout << "  - aligner: ICP transformation " << iterations_count << " : cloud_icp -> local_map" << std::endl;    
            }
            
            transformation_matrix = icp.getFinalTransformation().cast<double>();
            
            if(params_.debug)
            {
                print4x4Matrix (transformation_matrix);
            }
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

    // pcl::transformPointCloud (*point_cloud_, *point_cloud_, transformation_matrix);

    geometry_msgs::Transform transform = matrix_to_transform(transformation_matrix);

    if(params_.debug)
    {
        std::cout << "  - aligner: Translation vector is (x = " << transform.translation.x 
        << ", y = " << transform.translation.y 
        << ", z = " << transform.translation.z << ")"
        << std::endl;    
    }

    std::cout << "  - aligner: Saving updated scan PCD" << std::endl;
    pcl::io::savePCDFileASCII ("/home/rcv/dev/rse/localization_ws/final_pcd.pcd", *point_cloud_);
    pcl::io::savePCDFileASCII ("/home/rcv/dev/rse/localization_ws/map_pcd.pcd", *map_->get_data());

    diviner::AlignmentStats stats;
    stats.correspondence = icp.getFitnessScore();
    stats.target_threshold = params_.convergence_criterion;
    return std::make_tuple(transform, stats);
}

void PclAligner::findTf()
{
    // set up tf to be passed to the broadcaster
}

void PclAligner::updatePoints(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, geometry_msgs::PoseStamped prev_pose)
{
    std::cout << "  - aligner: Points are being updated" << std::endl;
    geometry_msgs::TransformStamped transform;
    // need to convert pose stamped to transform for fancy reasons :)
    pose_to_transform(prev_pose, transform);
    
    // Move point cloud to position of last known location
    transform_point_cloud(transform, point_cloud);
}

void PclAligner::predictPointLocation(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const geometry_msgs::PoseStamped prev_pose, const std::vector<diviner::Velocity> velocity)
{
    geometry_msgs::PoseStamped predicted_pose;
    geometry_msgs::TransformStamped transform;

    double time_delta = 0.1; // seconds

    // Calculate predicted pose based off of previous pose and velocity
    predicted_pose.pose.position.x = prev_pose.pose.position.x + (velocity[0].linear.x * time_delta);
    predicted_pose.pose.position.y = prev_pose.pose.position.y + (velocity[0].linear.y * time_delta);
    predicted_pose.pose.position.z = prev_pose.pose.position.z + (velocity[0].linear.z * time_delta);

    pose_to_transform(predicted_pose, transform);
    transform_point_cloud(transform, point_cloud);
}

void PclAligner::updateCurrPose(const geometry_msgs::Transform icp_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose)
{
    // Update the position based off of the ICP changes
    if(params_.debug)
    {
        std::cout << "  - aligner: Updating Current Pose" << std::endl;
    }
    
    geometry_msgs::PoseStamped previous_pose;
    previous_pose = veh_pose->front();

    if(params_.debug)
    {
        std::cout << "  - aligner: Previous pose is (x = " << previous_pose.pose.position.x 
        << ", y = " << previous_pose.pose.position.y 
        << ", z = " << previous_pose.pose.position.z << ")"
        << std::endl;

        std::cout << "  - aligner: Previous pose orientation is (x = " << previous_pose.pose.orientation.x 
        << ", y = " << previous_pose.pose.orientation.y 
        << ", z = " << previous_pose.pose.orientation.z
        << ", w = " << previous_pose.pose.orientation.w << ")"
        << std::endl;

        std::cout << "  - aligner: Translation vector is (x = " << icp_alignment.translation.x 
        << ", y = " << icp_alignment.translation.y 
        << ", z = " << icp_alignment.translation.z << ")"
        << std::endl;

        std::cout << "  - aligner: Rotation vector is (x = " << icp_alignment.rotation.x 
        << ", y = " << icp_alignment.rotation.y 
        << ", z = " << icp_alignment.rotation.z
        << ", w = " << icp_alignment.rotation.w << ")"
        << std::endl;

        std::cout << "  - aligner: Pulled previous pose. Updating current pose..." << std::endl;
    }

    geometry_msgs::PoseStamped new_pose;

    new_pose.pose.position.x = previous_pose.pose.position.x + icp_alignment.translation.x;
    new_pose.pose.position.y = previous_pose.pose.position.y + icp_alignment.translation.y;
    new_pose.pose.position.z = 0;

    if(previous_pose.pose.orientation.x == 0 && previous_pose.pose.orientation.y == 0 && previous_pose.pose.orientation.z == 0 && previous_pose.pose.orientation.w == 1)
    {
        if(icp_alignment.rotation.x == 0 && icp_alignment.rotation.y == 0 && icp_alignment.rotation.z == 0 && icp_alignment.rotation.w == 0)
        {
            new_pose.pose.orientation = previous_pose.pose.orientation;
        }
        else
        {
            new_pose.pose.orientation = icp_alignment.rotation;
        }
    }
    else
    {
        tf2::Quaternion previous_q, icp_q, new_q;
        icp_q = transform_to_tf2(icp_alignment.rotation);
        previous_q = transform_to_tf2(previous_pose.pose.orientation);
        new_q = previous_q * icp_q;
        new_q.normalize();

        new_pose.pose.orientation = tf2::toMsg(new_q);
    }

    new_pose.header.stamp = ros::Time::now();

    if(params_.debug)
    {
        std::cout << "  - aligner: New pose is (x = " << new_pose.pose.position.x 
        << ", y = " << new_pose.pose.position.y 
        << ", z = " << new_pose.pose.position.z << ")"
        << std::endl;

        std::cout << "  - aligner: New pose orientation is (x = " << new_pose.pose.orientation.x 
        << ", y = " << new_pose.pose.orientation.y 
        << ", z = " << new_pose.pose.orientation.z
        << ", w = " << new_pose.pose.orientation.w << ")"
        << std::endl;
    }

    // std::cout << "  - aligner: New pose all: " << new_pose << std::endl;

    veh_pose->emplace(veh_pose->begin(), new_pose);
}

}
