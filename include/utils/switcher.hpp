#ifndef UTILS__SWITCHER_HPP
#define UTILS__SWITCHER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <ros/ros.h>

namespace localization_node
{

struct rotation
{
    double x;
    double y;
    double z;
    double w;
};

struct SwitcherParams
{
    //
    double max_point_std_dev = 0.1;
    double max_angle_std_dev = 3.5;
    bool debug=false;
};

class Switcher
{
    public:
        explicit Switcher(const SwitcherParams & params) : params_(params){};
        ~Switcher() = default;
        
        /**
         * Checks the current status of how the system should be running
         * @param 
         * @return void 
         */
        void checkStatus();

        // lovely gps checker. if shits brok we hope this no brok
        void checkGPS(geometry_msgs::PoseWithCovarianceStamped & vehicle_location);

        // Find transform to map
        geometry_msgs::TransformStamped findMapTransform();



    private:
        SwitcherParams params_;

        geometry_msgs::Point getGPSPoint(geometry_msgs::PoseWithCovarianceStamped & vehicle_location);
        geometry_msgs::Quaternion getGPSAngle(geometry_msgs::PoseWithCovarianceStamped & vehicle_location);
        std::vector<double> getGPSStdDev(geometry_msgs::PoseWithCovarianceStamped & vehicle_location);

};

}

#endif