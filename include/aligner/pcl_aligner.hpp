#ifndef ALIGNER__PCL_ALIGNER_HPP
#define ALIGNER__PCL_ALIGNER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_aligner.hpp>

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

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

inline
geometry_msgs::Transform matrix_to_transform(Eigen::Matrix4d matrix)
{
    bool debug = false;
    geometry_msgs::Quaternion q;
    geometry_msgs::Vector3 v;

    // Yoinked this from somewhere else
    float trace = matrix.coeff(0, 0) + matrix.coeff(1, 1) + matrix.coeff(2, 2);
    if( trace > 0 ) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = ( matrix.coeff(2, 1) - matrix.coeff(1, 2) ) * s;
        q.y = ( matrix.coeff(0, 2) - matrix.coeff(2, 0) ) * s;
        q.z = ( matrix.coeff(1, 0) - matrix.coeff(0, 1) ) * s;
    } else {
        if ( matrix.coeff(0, 0) > matrix.coeff(1, 1) && matrix.coeff(0, 0) > matrix.coeff(2, 2) ) {
            float s = 2.0f * sqrtf( 1.0f + matrix.coeff(0, 0) - matrix.coeff(1, 1) - matrix.coeff(2, 2));
            q.w = (matrix.coeff(2, 1) - matrix.coeff(1, 2) ) / s;
            q.x = 0.25f * s;
            q.y = (matrix.coeff(0, 1) + matrix.coeff(1, 0) ) / s;
            q.z = (matrix.coeff(0, 2) + matrix.coeff(2, 0) ) / s;
        } else if (matrix.coeff(1, 1) > matrix.coeff(2, 2)) {
            float s = 2.0f * sqrtf( 1.0f + matrix.coeff(1, 1) - matrix.coeff(0, 0) - matrix.coeff(2, 2));
            q.w = (matrix.coeff(0, 2) - matrix.coeff(2, 0) ) / s;
            q.x = (matrix.coeff(0, 1) + matrix.coeff(1, 0) ) / s;
            q.y = 0.25f * s;
            q.z = (matrix.coeff(1, 2) + matrix.coeff(2, 1) ) / s;
        } else {
            float s = 2.0f * sqrtf( 1.0f + matrix.coeff(2, 2) - matrix.coeff(0, 0) - matrix.coeff(1, 1) );
            q.w = (matrix.coeff(1, 0) - matrix.coeff(0, 1) ) / s;
            q.x = (matrix.coeff(0, 2) + matrix.coeff(2, 0) ) / s;
            q.y = (matrix.coeff(1, 2) + matrix.coeff(2, 1) ) / s;
            q.z = 0.25f * s;
        }
    }

    // Steal translation next
    v.x = matrix.coeff(0, 3);
    v.y = matrix.coeff(1, 3);
    v.z = matrix.coeff(2, 3);

    if(debug)
    {
        std::cout << "    - matrix function: Unadded translation vector is (x = " << v.x 
        << ", y = " << v.y 
        << ", z = " << v.z << ")"
        << std::endl;

        std::cout << "    - matrix function: Unadded rotation vector is (x = " << q.x 
        << ", y = " << q.y 
        << ", z = " << q.z
        << ", w = " << q.w << ")"
        << std::endl;
    }

    geometry_msgs::Transform transform;
    transform.translation = v;
    transform.rotation = q;

    if(debug)
    {
        std::cout << "    - matrix function: Return translation vector is (x = " << transform.translation.x 
        << ", y = " << transform.translation.y 
        << ", z = " << transform.translation.z << ")"
        << std::endl;
    }

    return transform;
}

inline
tf2::Quaternion transform_to_tf2(geometry_msgs::Quaternion quaternion)
{
    tf2::Quaternion tf2_quaternion;
    tf2::fromMsg(quaternion, tf2_quaternion);
    return tf2_quaternion;
}

inline
Eigen::Affine3f transform_stamped_to_eigen(const geometry_msgs::TransformStamped& transformStamped)
{
    Eigen::Affine3d transform = tf2::transformToEigen(transformStamped.transform);
    return transform.cast<float>(); // Convert to float for PCL compatibility
}

inline
void transform_point_cloud(const geometry_msgs::TransformStamped& transformStamped, pcl::PointCloud<diviner::PointStamped>::Ptr cloud)
{
    Eigen::Affine3f transform = transform_stamped_to_eigen(transformStamped);
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

inline
void pose_to_transform(const geometry_msgs::PoseStamped prev_pose, geometry_msgs::TransformStamped transform)
{
    transform.header = prev_pose.header;
    transform.transform.translation.x = prev_pose.pose.position.x;
    transform.transform.translation.y = prev_pose.pose.position.y;
    transform.transform.translation.z = prev_pose.pose.position.z;
    transform.transform.rotation = prev_pose.pose.orientation;
}

struct PclAlignerParams
{
    std::string alignment_state = "automatic"; // set (use set iterations) or automatic (auto find alignment)
    int num_iterations = 2;
    int max_num_iterations = 10;
    double convergence_criterion = 0.1;
    bool debug = true;
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
        geometry_msgs::Transform align(const pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, std::shared_ptr<diviner::IMap> map_) override;

        /**
         * 
         * 
         */
        void findTf() override;

        /**
         * Takes in the translation and rotation matrix from alignment and updates the point cloud to match with the changes
         * 
         * @param point_cloud_ pointer to incoming scan. If we have a previous pose estimate we are going to update the scan location
         * @param previous_pose previous pose
         */
        void updatePoints(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, geometry_msgs::PoseStamped previous_pose) override;

        // void predictPointLocation(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const geometry_msgs::PoseStamped prev_pose, const std::vector<diviner::Velocity> velocity);

        /**
         * Takes in the rotation and translation vectors from icp and updates
         * the current estimated position
         * 
         * @param vehicle_poses pointer to the vector of vehicle poses
         * @param translation_vector translation vector from icp
         * @param rotation_vector rotation vector from icp
         * @return maybe updated vehicle pose vector?
         */
        void updateCurrPose(const geometry_msgs::Transform vehicle_alignment, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose) override;

    private:
        PclAlignerParams params_;

        pcl::IterativeClosestPoint<diviner::PointStamped, diviner::PointStamped> icp;

};

}

#endif