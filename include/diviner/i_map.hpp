#ifndef DIVINER__I_MAP_HPP
#define DIVINER__I_MAP_HPP

#include <diviner/utils/types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IMapParams
{
    double max_distance;
    bool debug = false;
};

class IMap
{
    public:
        explicit IMap(const IMapParams & params) : params_(params){};
        ~IMap() = default;

        /**
         * Adds a cloud of points to the map
         * 
         * @param input_cloud pointer to the cloud that will be added to map
         * @return
         */
        virtual void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) = 0;

        virtual pcl::PointCloud<diviner::PointStamped>::Ptr get_data() = 0;

        /**
         * Clears all points from map
         * @param none
         */
        virtual void clear_map() = 0;

        virtual int capacity() = 0;

        virtual size_t size() = 0;

        /**
         * Trims down the current local map by removing points 
         * outside of the max distance range
         * 
         * might want to make this return the map but it should be fine if it doesn't
         */
        virtual void trim_map() = 0;

    private:
        IMapParams params_;
};

}

#endif