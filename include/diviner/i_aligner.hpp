#ifndef DIVINER__I_ALIGNER_HPP
#define DIVINER__I_ALIGNER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>

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
        virtual geometry_msgs::Transform align(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) = 0;

        /**
         * Calculates transform for current position in the map frame
         * !! Might be able to be replaced by function below
         * 
         * @param tbd
         * @return current tf
         */
        virtual void findTf() = 0;

        /**
         * Updates the points in the point cloud to match with their aligned locations in the map
         * 
         * @param point_cloud Pointer to the current scan
         * @param alignment alignment vector/matrix 
         */
        virtual void updatePoints(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, diviner::Alignment alignment) = 0;

        /**
         * Takes in the rotation and translation vectors from icp and updates
         * the current estimated position
         * 
         * @param vehicle_poses pointer to the vector of vehicle poses
         * @param translation_vector translation vector from icp
         * @param rotation_vector rotation vector from icp
         * @return maybe updated vehicle pose vector?
         */
        virtual void updateCurrPose(const geometry_msgs::Transform vehicle_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> updated_vehicle_position) = 0;

    private:
        IAlignerParams params_;
};

}

#endif