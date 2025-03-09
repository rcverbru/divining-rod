#include <vestimator/example_vestimator.hpp>

#include <vector>

namespace diviner
{

void ExampleVestimator::estimate(std::vector<diviner::Velocity> & velocities, std::vector<geometry_msgs::PoseStamped> &veh_pose)
{
    if(params_.debug)
    {
        std::cout << "Starting estimate" << std::endl;
    }
    // Just want to output something so we can test static bags
    new_velocity.linear.x = 0.0;
    new_velocity.linear.y = 0.0;
    new_velocity.linear.z = 0.0;
    new_velocity.angular.x = 0.0;
    new_velocity.angular.y = 0.0;
    new_velocity.angular.z = 0.0;

    velocities.push_back(new_velocity);

    if(params_.debug)
    {
        std::cout << "Added current estimated velocity: " << std::endl << new_velocity.linear << new_velocity.angular;
    }

}

}