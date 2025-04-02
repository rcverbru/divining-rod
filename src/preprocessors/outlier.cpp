#include <preprocessors/outlier.hpp>

namespace preprocessor
{

void OutlierProcessor::process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud)
{
    // Remove Outliers

    if(params_.debug)
    {
        std::cout << "  - outlier_processor: Starting point cloud has " << input_cloud->size() << " points." << std::endl;
    }

    pcl::StatisticalOutlierRemoval<diviner::PointStamped> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*input_cloud);

    if(params_.debug)
    {
        std::cout << "  - outlier_processor: Final point cloud has " << input_cloud->size() << " points." << std::endl;
    }
}

}