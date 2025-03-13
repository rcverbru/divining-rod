#include <preprocessors/outlier.hpp>

namespace diviner
{

void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud)
{
    // Remove Outliers
    pcl::StatisticalOutlierRemoval<diviner::PointStamped> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*input_cloud);
}

}