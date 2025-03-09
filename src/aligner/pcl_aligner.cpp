#include <aligner/pcl_aligner.hpp>

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

geometry_msgs::Transform PclAligner::align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud_, std::shared_ptr<diviner::IMap> map_)
{
    // The Iterative Closest Point algorithm
    // pulled from pcl icp tutorial...

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    // int iterations = 25;

    if(params_.debug)
    {
        std::cout << "  - aligner: Point Cloud Size = " << point_cloud_->size() << std::endl;
    }

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

    geometry_msgs::Transform transform = matrix_to_transform(transformation_matrix);

    if(params_.debug)
    {
        std::cout << "  - aligner: Translation vector is (x = " << transform.translation.x 
        << ", y = " << transform.translation.y 
        << ", z = " << transform.translation.z << ")"
        << std::endl;    
    }

    return transform;
}

void PclAligner::findTf()
{
    // set up tf to be passed to the broadcaster
}

void PclAligner::updatePoints(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, diviner::Alignment vehicle_alignment)
{
    // Zero map for easier aligning before next step
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
    new_pose.pose.position.z = previous_pose.pose.position.z + icp_alignment.translation.z;

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