#ifndef PREPROCESSORS__TRANSFORM_HPP
#define PREPROCESSORS__TRANSFORM_HPP

#include <diviner/utils/types.hpp>
#include <preprocessors/i_preprocessor.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <memory>
#include <list>

namespace preprocessor
{

struct TransformProcessorParams
{
    std::string target_frame = "base_link";
    std::string source_frame = "cepton2";
    geometry_msgs::TransformStamped target_transform;
    bool debug = false;
};

class TransformProcessor : public IPreprocessor
{
    public:
        explicit TransformProcessor(const Params<TransformProcessorParams, IPreprocessorParams> & params) :
        IPreprocessor(params.parent_params),
        params_(params.child_params){};
        ~TransformProcessor()=default;

        /**
         * Processing function to remove outliers from the input cloud
         *
         * @param input_cloud input cloud to process
         */
        void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;

        std::string getName() override
        {
            return "Transform Processor";
        }

    private:
        TransformProcessorParams params_;

        Eigen::Affine3f transformStampedToEigen(const geometry_msgs::TransformStamped& transformStamped) {
            Eigen::Affine3d transform = tf2::transformToEigen(transformStamped.transform);
            return transform.cast<float>(); // Convert to float for PCL compatibility
        }

        void transformPointCloud(const geometry_msgs::TransformStamped& transformStamped, pcl::PointCloud<diviner::PointStamped>::Ptr cloud) {
            Eigen::Affine3f transform = transformStampedToEigen(transformStamped);
            pcl::transformPointCloud(*cloud, *cloud, transform);
        }

};

}

#endif