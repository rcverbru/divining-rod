#ifndef DIVINER__I_DESKEWER_HPP
#define DIVINER__I_DESKEWER_HPP

#include <diviner/utils/types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IDeskewerParams
{
    bool debug = false;
};

class IDeskewer
{
    public:
        explicit IDeskewer(const IDeskewerParams & params) : params_(params){};
        ~IDeskewer()=default;

        /**
         * Deskews a point cloud with a determined velocity of the vehicle
         * 
         * @param point_cloud Pointer to the current scan
         * @return Deskewed scan
         */
        virtual void deskew(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const std::vector<diviner::Velocity> & velocities) = 0;
    
    private:
        IDeskewerParams params_;
};

}

#endif