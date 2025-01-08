#ifndef DESKEWER__STANDARD_DESKEWER_HPP
#define DESKEWER__STANDARD_DESKEWER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_deskewer.hpp>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct StandardDeskewerParams
{
    bool debug = false;
};

class StandardDeskewer : public IDeskewer
{
    public:
        explicit StandardDeskewer(const Params<StandardDeskewerParams, IDeskewerParams> & params) :
        IDeskewer(params.parent_params),
        params_(params.child_params){};
        ~StandardDeskewer() = default;
        
        void deskew(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const std::vector<diviner::Velocity> & velocities) override;

    private:
        StandardDeskewerParams params_;

        pcl::PointCloud<diviner::PointStamped>::Ptr deskewed_cloud;
        int num_vel;
        diviner::Velocity current_velocity;
        diviner::PointStamped new_point;
};

}

#endif