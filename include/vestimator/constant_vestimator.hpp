#ifndef VESTIMATOR__CONSTANT_VESTIMATOR_HPP
#define VESTIMATOR__CONSTANT_VESTIMATOR_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_vestimator.hpp>

#include <eigen3/Eigen/Core>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace diviner
{

struct ConstantVestimatorParams
{
    float scan_time = 0.01;
    bool debug = false;
};

inline
Eigen::Vector3d logSO3(const Eigen::Matrix3d & R)
{
    bool debug = false;
    
    double cos_theta = (R.trace() - 1) / 2.0;
    double theta = acos(cos_theta);

    if(debug)
    {
        std::cout << "    - logSO3: Cos Theta " << cos_theta << std::endl;
        std::cout << "    - logSO3: Theta " << theta << std::endl;
    }

    if (theta < 1e-6) {
        return Eigen::Vector3d::Zero();  // No rotation case
    }

    Eigen::Matrix3d log_R = (theta / (2.0 * sin(theta))) * (R - R.transpose());
    Eigen::Vector3d omega;
    omega << log_R(2, 1), log_R(0, 2), log_R(1, 0);
    std::cout << log_R(2,1) << ", " << log_R(0,2) << ", " << log_R(1,0) << std::endl;
    
    return omega;
};

class ConstantVestimator : public IVestimator
{
    public:
        explicit ConstantVestimator(const Params<ConstantVestimatorParams, IVestimatorParams> & params) :
        IVestimator(params.parent_params),
        params_(params.child_params){};
        ~ConstantVestimator() = default;

        void estimate(std::vector<diviner::Velocity> & velocities, std::vector<geometry_msgs::PoseStamped> &veh_pose) override;

    private:
        ConstantVestimatorParams params_;

        diviner::Point3d p, pp;
        Eigen::Vector3d t_p, t_pp;
        Eigen::Vector3d linear_velocity;

        float time_of_sweep; // Should expect to be around 0.05s or 0.1s may be different with cepton though
        geometry_msgs::Twist twist;

        diviner::Velocity estimated_velocity;
};

}

#endif