#ifndef ALIGNER__EXAMPLE_ALIGNER_HPP
#define ALIGNER__EXAMPLE_ALIGNER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_aligner.hpp>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct ExampleAlignerParams
{
    bool debug = false;
};

class ExampleAligner : public IAligner
{
    public:
        explicit ExampleAligner(const Params<ExampleAlignerParams, IAlignerParams> & params) : 
        IAligner(params.parent_params),
        params_(params.child_params){};
        ~ExampleAligner() = default;

        /**
         * First thing that the diviner does when it first turns on
         * it checks to see if we have a gps position and adds the first location
         * to our pose lists.
         * @param veh_pose pose input from gps (if running)
         * @return maybe pose list?
         */
        void initialize(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose) override {};

        Eigen::Matrix4d align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) override;

        /**
         * 
         * 
         */
        void find_tf() override {};

        /**
         * Takes in the translation and rotation matrix from alignment and updates the point cloud to match with the changes
         * 
         * @param point_cloud_ pointer to filtered scan that needs to be transformed
         * @param alignment translational and rotational matrix from aligner->align function
         */
        void update_points(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, diviner::Alignment alignment) override {};

        /**
         * Takes in the rotation and translation vectors from icp and updates
         * the current estimated position
         * 
         * @param vehicle_poses pointer to the vector of vehicle poses
         * @param translation_vector translation vector from icp
         * @param rotation_vector rotation vector from icp
         * @return maybe updated vehicle pose vector?
         */
        void update_curr_pose(const diviner::Alignment vehicle_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> updated_vehicle_position) override {};

    private:
        ExampleAlignerParams params_;
};

}

#endif