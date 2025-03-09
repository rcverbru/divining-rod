#ifndef VESTIMATOR__WHEEL_TICK_VESTIMATOR_HPP
#define VESTIMATOR__WHEEL_TICK_VESTIMATOR_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_vestimator.hpp>

namespace diviner
{

struct WheelTickVestimatorParams
{
    bool debug = false;
};

class WheelTickVestimator : public IVestimator
{
    public:
        explicit WheelTickVestimator(const Params<WheelTickVestimatorParams, IVestimatorParams> & params) :
        IVestimator(params.parent_params),
        params_(params.child_params){};
        ~WheelTickVestimator() = default;

        void estimate(std::vector<diviner::Velocity> & velocities, std::vector<geometry_msgs::PoseStamped> &veh_pose) override;
        
    private:
        WheelTickVestimatorParams params_;
};

}

#endif