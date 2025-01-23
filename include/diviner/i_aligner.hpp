#ifndef DIVINER__I_ALIGNER_HPP
#define DIVINER__I_ALIGNER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IAlignerParams
{
    bool debug = false;
};

class IAligner
{
    public:
        explicit IAligner(const IAlignerParams & params) : params_(params){};
        ~IAligner()=default;

        /**
         * First thing that the diviner does when it first turns on
         * it checks to see if we have a gps position and adds the first location
         * to our pose lists.
         * @param veh_pose pose input from gps (if running)
         * @return initial pose list
         */
        virtual void initialize(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose) = 0;

        /**
         * Aligns the current scan with the local map
         * 
         * @param point_cloud Pointer to the current scan
         * @return returns the translation and rotation matrix to match scan with map
         */
        virtual void align(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) = 0;

        /**
         * Takes in new points and updates the current map with them
         * 
         * @param point_cloud Pointer to the current scan
         * @return Nothing
         */
        virtual void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) = 0;

        /**
         * 
         * 
         */
        //virtual void transform_current_scan() = 0;

        /**
         * Calculates transform for current position in the map frame
         * !! Might be able to be replaced by function below
         * 
         * @param tbd
         * @return current tf
         */
        virtual void find_tf() = 0;

        /**
         * Takes in the rotation and translation vectors from icp and updates
         * the current estimated position
         * 
         * @param vehicle_poses pointer to the vector of vehicle poses
         * @param translation_vector translation vector from icp
         * @param rotation_vector rotation vector from icp
         * @return maybe updated vehicle pose vector?
         */
        virtual void update_curr_pose(const diviner::alignment vehicle_alignment) = 0;

    private:
        IAlignerParams params_;
};

}

#endif