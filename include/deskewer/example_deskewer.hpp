#ifndef DESKEWER__EXAMPLE_DESKEWER_HPP
#define DESKEWER__EXAMPLE_DESKEWER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_deskewer.hpp>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct ExampleDeskewerParams
{
    bool debug = false;
};

class ExampleDeskewer : public IDeskewer
{
    public:
        explicit ExampleDeskewer(const Params<ExampleDeskewerParams, IDeskewerParams> & params) :
        IDeskewer(params.parent_params),
        params_(params.child_params){};
        ~ExampleDeskewer() = default;
        
        void deskew(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const std::vector<diviner::Velocity> & velocities) override;

    private:
        ExampleDeskewerParams params_;
};

}

#endif