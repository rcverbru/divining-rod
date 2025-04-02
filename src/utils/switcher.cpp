#include <utils/switcher.hpp>

namespace localization_node
{

void Switcher::checkStatus()
{
    // Need to have a status thing that we can check for how to continue with processing
}

void Switcher::checkGPS(geometry_msgs::PoseWithCovarianceStamped &vehicle_location)
{
    std::vector<double> curr_std_dev;

    // Get the covariance of the current point
    curr_std_dev = getGPSStdDev(vehicle_location);

    if(params_.debug)
    {
        std::cout << "- switcher: Checking status of vehicle location" << std::endl;
        std::cout << "- switcher: Size of vector = " << curr_std_dev.size() << std::endl;
        std::cout << "- switcher: Current position std_dev: (x = " << curr_std_dev.front()
        << ", y = " << curr_std_dev[curr_std_dev.front() + 1]
        << ", z = " << curr_std_dev[curr_std_dev.front() + 2]
        << ")" << std::endl;
        std::cout << "- switcher: Current max position std dev: " << params_.max_point_std_dev << std::endl;
    }

    for(const auto cov : curr_std_dev)
    {
        if(params_.debug)
        {
            std::cout << "- switcher: Current covariance: " << cov << std::endl;
        }
    }

    if(curr_std_dev.front() >= params_.max_point_std_dev || curr_std_dev[curr_std_dev.size()+1] >= params_.max_point_std_dev ||curr_std_dev[curr_std_dev.size()+2] >= params_.max_point_std_dev)
    {
        std::cout << "- switcher: Position standard deviation is high. Switching to lidar localization." << std::endl;
    }

    if(curr_std_dev[curr_std_dev.size()+3] >= params_.max_angle_std_dev || curr_std_dev[curr_std_dev.size()+4] >= params_.max_angle_std_dev ||curr_std_dev[curr_std_dev.size()+5] >= params_.max_angle_std_dev)
    {
        std::cout << "- switcher: Position standard deviation is high. Switching to lidar localization." << std::endl;
    }
}

geometry_msgs::TransformStamped Switcher::findMapTransform()
{
    // 
    geometry_msgs::TransformStamped transform;

    return transform;
}

geometry_msgs::Point Switcher::getGPSPoint(geometry_msgs::PoseWithCovarianceStamped &vehicle_location)
{
    geometry_msgs::Point point;
    point.x = vehicle_location.pose.pose.position.x;
    point.y = vehicle_location.pose.pose.position.y;
    point.z = vehicle_location.pose.pose.position.z;

    return point;
}

geometry_msgs::Quaternion Switcher::getGPSAngle(geometry_msgs::PoseWithCovarianceStamped &vehicle_location)
{
    geometry_msgs::Quaternion angle;
    angle.x = vehicle_location.pose.pose.orientation.x;
    angle.y = vehicle_location.pose.pose.orientation.y;
    angle.z = vehicle_location.pose.pose.orientation.z;
    angle.w = vehicle_location.pose.pose.orientation.w;

    return angle;
}

std::vector<double> Switcher::getGPSStdDev(geometry_msgs::PoseWithCovarianceStamped &vehicle_location)
{
    // 
    std::vector<double> std_dev;

    // Each covariance is 7 elements apart
    for(int i = 0; i < vehicle_location.pose.covariance.size(); i = i + 7)
    {
        std::cout << "- switcher: Covariance: " << sqrt(vehicle_location.pose.covariance[i]) << std::endl;
        std_dev.push_back(sqrt(vehicle_location.pose.covariance[i]));
    }

    return std_dev;
}

}