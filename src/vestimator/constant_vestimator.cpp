#include <vestimator/constant_vestimator.hpp>

namespace diviner
{

void ConstantVestimator::estimate(std::vector<diviner::Velocity> & velocities, geometry_msgs::TransformStamped transform_, std::vector<geometry_msgs::PoseStamped> &veh_pose)
{
    // Find Current Estimated Velocity
    if(params_.debug)
    {
        std::cout << "  - vestimate: Starting estimate" << std::endl;
        
        // find memory leak size :)
        std::cout << "  - vestimate: incoming vector size: " << veh_pose.size() << std::endl;
    }

    // set std::out precision to high enough that we can see reeeeeaaaaallly small number changes
    std::cout << std::setprecision(13);

    if(params_.debug)
    {
        // mini visual check to make sure the points aren't exactly the same
        std::cout << "  - vestimate: x0: " << veh_pose[0].pose.position.x << std::endl;
        std::cout << "  - vestimate: x1: " << veh_pose[0].pose.position.x << std::endl;
    }

    // create p (previous) translation vectors
    p.x = veh_pose[0].pose.position.x - veh_pose[1].pose.position.x;
    p.y = veh_pose[0].pose.position.y - veh_pose[1].pose.position.y;
    p.z = veh_pose[0].pose.position.z - veh_pose[1].pose.position.z;
    
    // create pp (previous previous) translation vectors
    pp.x = veh_pose[1].pose.position.x - veh_pose[2].pose.position.x;
    pp.y = veh_pose[1].pose.position.y - veh_pose[2].pose.position.y;
    pp.z = veh_pose[1].pose.position.z - veh_pose[2].pose.position.z;
    
    // Convert to eigen vectors
    t_p << p.x, p.y, p.z;
    t_pp << pp.x, pp.y, pp.z;

    if(params_.debug)
    {
        std::cout << "  - vestimate: Translation vector 1: " << std::endl;
        std::cout << t_p << std::endl;
        std::cout << "  - vestimate: Translation vector 2: " << std::endl;
        std::cout << t_pp << std::endl;
    }

    // Time to make rotation matrix :)
    // In the equation this is the pp (t-2) rotation matrix for linear velocity
    tf2::Quaternion q2(veh_pose[2].pose.orientation.x, veh_pose[2].pose.orientation.y, veh_pose[2].pose.orientation.z, veh_pose[2].pose.orientation.w);
    tf2::Matrix3x3 tf_rotation_matrix_2(q2);

    // Convert tf2 rotation matrix to Eigen matrix
    Eigen::Matrix3d R_t_2;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R_t_2(i, j) = tf_rotation_matrix_2[i][j];
        }
    }

    // std::cout << "before transpose" << std::endl;
    // R_t_2 = R_t_2.transpose();

    linear_velocity = R_t_2.transpose() * (t_p - t_pp);
    linear_velocity = linear_velocity / params_.scan_time;
    
    if(params_.debug)
    {
        std::cout << "  - vestimate: Linear velocity is (x: " << linear_velocity[0] 
        << ", y: " << linear_velocity[1]
        << ", z: " << linear_velocity[2] << ")" << std::endl;    
    }
    // std::cout << "or " << sqrt((linear_velocity[0])^2 + (linear_velocity[1])^2 + (linear_velocity[2])^2) << " m/s^2." << std::endl;

    // Time for rotational velocity
    // Need to make the p (t-1) rotation matrix
    tf2::Quaternion q1(veh_pose[1].pose.orientation.x, veh_pose[1].pose.orientation.y, veh_pose[1].pose.orientation.z, veh_pose[1].pose.orientation.w);
    tf2::Matrix3x3 tf_rotation_matrix_1(q1);

    // Convert tf2 rotation matrix to Eigen matrix
    Eigen::Matrix3d R_t_1;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R_t_1(i, j) = tf_rotation_matrix_1[i][j];
        }
    }

    // Make holder rotation matrix for angular calculations
    Eigen::Matrix3d eigen_rotation_matrix = R_t_2.transpose() - R_t_1;

    // Compute the log map of SO(3) for rotation part
    Eigen::Vector3d upper = logSO3(eigen_rotation_matrix);

    // Calculate angular velocity by log/scan time
    Eigen::Vector3d angular_velocity = upper/0.01;

    if(params_.debug)
    {
        std::cout << "  - vestimate: Angular velocity is (x: " << angular_velocity[0] 
        << ", y: " << angular_velocity[1]
        << ", z: " << angular_velocity[2] << ")" << std::endl;    
    }

    estimated_velocity.linear.x = linear_velocity[0];
    estimated_velocity.linear.y = linear_velocity[1];
    estimated_velocity.linear.z = linear_velocity[2];
    estimated_velocity.angular.x = angular_velocity[0];
    estimated_velocity.angular.y = angular_velocity[1];
    estimated_velocity.angular.z = angular_velocity[2];

    if(params_.debug)
    {
        std::cout << "Linear x: " << linear_velocity[0] << std::endl;
        std::cout << "Linear y: " << linear_velocity[1] << std::endl;
        std::cout << "Linear z: " << linear_velocity[2] << std::endl;
        std::cout << "Angular x: " << angular_velocity[0] << std::endl;
        std::cout << "Angular y: " << angular_velocity[1] << std::endl;
        std::cout << "Angular z: " << angular_velocity[2] << std::endl;    
    }

    // Add Velocity to vector
    velocities.emplace(velocities.begin(), estimated_velocity);

    if(params_.debug)
    {
        std::cout << "  - vestimate: Added current estimated velocity: " << std::endl
        << "linear: " << std::endl 
        << estimated_velocity.linear 
        << "angular: " << std::endl
        << estimated_velocity.angular;
    }
}

}