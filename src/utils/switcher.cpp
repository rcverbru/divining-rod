#include <utils/switcher.hpp>

namespace diviner
{

void Switcher::checkStatus()
{
    // Need to have a status thing that we can check for how to continue with processing
}

void Switcher::checkGPS(geometry_msgs::PoseWithCovarianceStamped &vehicle_location)
{
    //
    if(params_.debug)
    {
        std::cout << "- switcher: Checking status of vehicle location" << std::endl;
        std::cout << "- switcher: Current std_dev: " << sqrt(vehicle_location.pose.covariance[0]) << std::endl;
    }
}

void Switcher::findMapTransform()
{
    //

}

}