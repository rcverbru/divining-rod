#include <preprocessors/transform.hpp>

namespace preprocessor
{

void TransformProcessor::process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud)
{
    // Transform the point cloud to the frame in the params
    input_cloud->header.frame_id = params_.target_frame;

    if(params_.debug)
    {
        std::cout << params_.target_transform << std::endl;
    }

    transformPointCloud(params_.target_transform, input_cloud);

    if(params_.debug)
    {
        std::cout << "  - transform_processor: Point Cloud Transformed" << std::endl;
    }
}

}