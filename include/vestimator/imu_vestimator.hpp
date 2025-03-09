#ifndef VESTIMATOR__IMU_VESTIMATOR_HPP
#define VESTIMATOR__IMU_VESTIMATOR_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_vestimator.hpp>

namespace diviner
{

struct ImuVestimatorParams
{
    bool debug = false;
};

class ImuVestimator : public IVestimator
{
    public:
        explicit ImuVestimator(const Params<ImuVestimatorParams, IVestimatorParams> & params) :
        IVestimator(params.parent_params),
        params_(params.child_params){};
        ~ImuVestimator() = default;

        void estimate(std::vector<diviner::Velocity> & velocities, std::vector<geometry_msgs::PoseStamped> &veh_pose) override;

    private:
        ImuVestimatorParams params_;
};

}

#endif