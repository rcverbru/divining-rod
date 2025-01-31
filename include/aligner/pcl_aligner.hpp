#ifndef ALIGNER__PCL_ALIGNER_HPP
#define ALIGNER__PCL_ALIGNER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_aligner.hpp>

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct PclAlignerParams
{
    std::string alignment_state = "set"; // set (use set iterations) or automatic (auto find alignment)
    int num_iterations = 2;
    int max_num_iterations = 50;
    double convergence_criterion = 0.05;
    bool debug = true;
};

inline
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("\tRotation matrix :\n");
  printf ("\t    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("\tR = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("\t    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("\tTranslation vector :\n");
  printf ("\tt = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
};

class PclAligner : public IAligner
{
    public:
        explicit PclAligner(const Params<PclAlignerParams, IAlignerParams> & params) : 
        IAligner(params.parent_params),
        params_(params.child_params){};
        ~PclAligner() = default;

        /**
         * First thing that the diviner does when it first turns on
         * it checks to see if we have a gps position and adds the first location
         * to our pose lists.
         * @param veh_pose pose input from gps (if running)
         * @return maybe pose list?
         */
        void initialize(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose) override;

        /**
         * Alignment function for the aligner interface.
         * Aligns the current scan to the local map
         * @param point_cloud scan to be aligned
         * @param map_ pointer to local_map
         * @return nuthin currently. may need to return alignment information
         */
        Eigen::Matrix4d align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) override;

        /**
         * 
         * 
         */
        void find_tf() override;

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
        void update_curr_pose(const diviner::Alignment vehicle_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose) override;

    private:
        PclAlignerParams params_;

        pcl::IterativeClosestPoint<diviner::PointStamped, diviner::PointStamped> icp;

};

}

#endif