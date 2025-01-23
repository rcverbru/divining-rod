#ifndef DIVINER__DIVINER_HPP
#define DIVINER__DIVINER_HPP

#include <diviner/utils/types.hpp>

#include <diviner/i_aligner.hpp>
#include <diviner/i_deskewer.hpp>
#include <diviner/i_filter.hpp>
#include <diviner/i_map.hpp>
#include <diviner/i_vestimator.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <list>

namespace diviner
{

// struct DivinerParams
// {
//     std::shared_ptr<IAligner> aligner;
//     std::shared_ptr<IFilter> filter,
//     std::shared_ptr<IDeskewer> deskewer,
//     std::shared_ptr<IMapper> mapper,
//     std::shared_ptr<IVestimator> vestimator,
//     bool debug = false;
// };


class Diviner
{
    public:
        explicit Diviner(
            std::shared_ptr<IAligner> aligner,
            std::shared_ptr<IFilter> filter,
            std::shared_ptr<IDeskewer> deskewer,
            std::shared_ptr<IMap> map,
            std::shared_ptr<IVestimator> vestimator,
            bool debug = false) :
            aligner_(aligner),
            filter_(filter),
            deskewer_(deskewer),
            map_(map),
            vestimator_(vestimator), 
            debug_(debug){}
        ~Diviner() = default;
        
        /**
         * 
         * 
         * @param cloud current point cloud 
         */
        void step(pcl::PointCloud<diviner::PointStamped>::Ptr cloud, geometry_msgs::TransformStamped gnss_to_map_, geometry_msgs::TransformStamped cloud_to_vehicle, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose);

    private:
        std::shared_ptr<IAligner> aligner_;
        std::shared_ptr<IFilter> filter_;
        std::shared_ptr<IDeskewer> deskewer_;
        std::shared_ptr<IMap> map_;
        std::shared_ptr<IVestimator> vestimator_;
        bool debug_;
        bool first_scan = true;

        diviner::Pose current_pose;
        std::vector<diviner::Velocity> velocities;
        geometry_msgs::TransformStamped transform;

        diviner::alignment vehicle_alignment;

        pcl::PointCloud<diviner::PointStamped>::Ptr deskewed_cloud;
};

}

#endif