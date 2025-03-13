#ifndef UTILS__SWITCHER_HPP
#define UTILS__SWITCHER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <ros/ros.h>

namespace diviner
{

struct SwitcherParams
{
    //
    double max_std_dev = 0.1;
    bool debug=false;
};

class Switcher
{
    public:
        explicit Switcher(const SwitcherParams & params) : params_(params){};
        ~Switcher() = default;
        
        /**
         * Checks the current status of how the system should be running
         * @param void
         * @return 
         */
        void checkStatus();

        // lovely gps checker. if shits brok we hope this no brok
        void checkGPS(geometry_msgs::PoseWithCovarianceStamped & vehicle_location);

        // Find transform to map
        void findMapTransform();

    private:
        SwitcherParams params_;
};

}

#endif