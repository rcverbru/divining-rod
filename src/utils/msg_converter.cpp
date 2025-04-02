#include <utils/msg_converter.hpp>

namespace localization_node
{

pcl::PointCloud<diviner::PointStamped>::Ptr MsgConverter::convert(sensor_msgs::PointCloud2ConstPtr input_cloud)
{
    cepton_cloud_ = std::make_shared<std::vector<diviner::CeptonPoint>>();

    pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_ =
    pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);
    std_msgs::Header header;
    //
    from_ros(input_cloud, cepton_cloud_, header);

    if(params_.debug)
    {
        std::cout << "  - msgconverter: Converted " << cepton_cloud_->size() << " points to cepton points." << std::endl;
    }

    if(params_.point_type == "pointxyzi")
    {
        if(params_.debug)
        {
            // ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using pointxyzi");            
        }

        to_pointxyzi(cepton_cloud_, header, current_scan_);

        if(params_.debug)
        {
            std::cout << "  - msgconverter: Converted " << current_scan_->size() << " points to PCL PointXYZI points." << std::endl;
        }
    }
    else
    {
        // no point type given
        std::cout << "  - msgconverter: No point type given" << std::endl;
    }

    return current_scan_;
}

}