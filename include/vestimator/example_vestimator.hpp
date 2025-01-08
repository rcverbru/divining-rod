#ifndef VESTIMATOR__EXAMPLE_VESTIMATOR_HPP
#define VESTIMATOR__EXAMPLE_VESTIMATOR_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_vestimator.hpp>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct ExampleVestimatorParams
{
    bool debug = false;
};

class ExampleVestimator : public IVestimator
{
    public:
        explicit ExampleVestimator(const Params<ExampleVestimatorParams, IVestimatorParams> & params) : 
        IVestimator(params.parent_params),
        params_(params.child_params){};
        ~ExampleVestimator() = default;

        void estimate(std::vector<diviner::Velocity> & velocities, geometry_msgs::TransformStamped transform, std::vector<geometry_msgs::PoseStamped> &veh_pose) override;

    private:
        ExampleVestimatorParams params_;
        diviner::Velocity new_velocity;
};

}

#endif