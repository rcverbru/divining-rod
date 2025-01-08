#ifndef DIVINER__I_FILTER_HPP
#define DIVINER__I_FILTER_HPP

#include <diviner/utils/types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IFilterParams
{
    bool debug = false;
};

class IFilter
{
    public:
        explicit IFilter(const IFilterParams & params) : params_(params){};
        ~IFilter()=default;

        /**
         * Filters down the lidar point cloud
         * 
         * @param point_cloud Pointer to the raw point cloud from the cepton
         * @return pointer to the filtered point cloud
         */
        virtual void filter(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud) = 0;

    private:
        IFilterParams params_;
};

}

#endif