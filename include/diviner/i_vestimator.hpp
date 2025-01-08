#ifndef DIVINER__I_VESTIMATOR_HPP
#define DIVINER__I_VESTIMATOR_HPP

#include <diviner/utils/types.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IVestimatorParams
{
    bool debug = false;
};

class IVestimator
{
    public:
        explicit IVestimator(const IVestimatorParams & params) : params_(params){};
        ~IVestimator() = default;

        /**
         * Estimates speed of vehicle based
         * @param velocities pointer to a vector of velocities
         * @return returns the velocity of the vehicle
         */
        virtual void estimate(std::vector<diviner::Velocity> & velocities, geometry_msgs::TransformStamped transform, std::vector<geometry_msgs::PoseStamped> &veh_pose) = 0;

    private:
        IVestimatorParams params_;

};

}

#endif