#ifndef UTILS__MSG_CONVERTER_HPP
#define UTILS__MSG_CONVERTER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace diviner
{

struct MsgConverterParams
{
    //
    std::string point_type;
    int avg_scan_time = 300; // in us (microseconds)
    bool debug;
};

class MsgConverter
{
    public:
        explicit MsgConverter(const MsgConverterParams & params) : params_(params){};
        ~MsgConverter() = default;
        
        // I want to make this the "wrapper" for converting msgs.
        pcl::PointCloud<diviner::PointStamped>::Ptr convert(sensor_msgs::PointCloud2ConstPtr input_cloud);

    private:
        std::shared_ptr<std::vector<diviner::CeptonPoint>> cepton_cloud_;
        pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_;
        std::shared_ptr<pcl::PointCloud<diviner::PointStamped>> temp_cloud_;

        MsgConverterParams params_;

        // Convert to PCL PointXYZI
        void to_pointxyzi(std::shared_ptr<std::vector<diviner::CeptonPoint>> cepton_cloud_, std_msgs::Header &header, pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_)
        {
            if(params_.debug)
            {
                std::cout << "    - to_pointxyzi: Starting pcl converter" << std::endl;
            }

            diviner::PointStamped pcl_point;

            if(params_.debug)
            {
                std::cout << "    - to_pointxyzi: time stamp:" << header.stamp << std::endl;
                std::cout << "    - to_pointxyzi: time stamp (in nsec):" << header.stamp.toNSec() << std::endl;
            }
            
            current_scan_->header.frame_id = header.frame_id;
            current_scan_->header.stamp = header.stamp.toNSec();

            for(auto &point : *cepton_cloud_)
            {
                pcl_point.x = point.x;
                pcl_point.y = point.y; 
                pcl_point.z = point.z;
                pcl_point.intensity = point.point_timestamp;

                current_scan_->push_back(pcl_point);
            }

            if(params_.debug)
            {
                std::cout << "    - to_pointxyzi: current stamp: " << current_scan_->header.stamp << std::endl;
            }

            if(params_.debug)
            {
                std::cout << "    - to_pointxyzi: Finished pcl converter" << std::endl;
            }
        }

        // Convert from ROS Sensor msg to Cepton point holder
        void from_ros(sensor_msgs::PointCloud2ConstPtr input_cloud, std::shared_ptr<std::vector<diviner::CeptonPoint>> cepton_cloud_, std_msgs::Header &cloud_header)
        {
            if(params_.debug)
            {
                std::cout << "    - from_ros: Starting ROS Conversion" << std::endl;
            }

            // Create temp holder point
            diviner::CeptonPoint curr_point;

            // std::cout << *input_cloud << std::endl;

            sensor_msgs::PointCloud2 temp_cloud = *input_cloud;
            // std::cout << temp_cloud.data.data() << std::endl;

            uint8_t point_timestamp = 0u;
            
            for(int i = 0; i < temp_cloud.width; i++)
            {
                int offset = i * 20;
                int num_fields = temp_cloud.fields.size();    

                for(int j = 0; j < num_fields; j++)
                {
                    // This should be rebuilt once it works...
                    switch(j)
                    {
                        case 0:
                            // x
                            // std::cout << temp_cloud.fields.data() << std::endl;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.x = *reinterpret_cast<float *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point x: " << curr_point.x << std::endl;
                            // offset = temp_cloud.fields.offset
                            break;
                        case 1:
                            // y
                            offset = offset + 4;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.y = *reinterpret_cast<float *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point y: " << curr_point.y << std::endl;
                            break;
                        case 2:
                            // z
                            offset = offset + 4;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.z = *reinterpret_cast<float *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point z: " << curr_point.z << std::endl;
                            break;
                        case 3:
                            // Reflectivity
                            offset = offset + 4;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.reflectivity = *reinterpret_cast<float *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point reflectivity: " << curr_point.reflectivity << std::endl;
                            break;
                        case 4:
                            // Relative Timestamp
                            offset = offset + 4;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.relative_timestamp = *reinterpret_cast<uint8_t *>(temp_cloud.data.data() + offset);
                            point_timestamp = point_timestamp + curr_point.relative_timestamp;
                            curr_point.point_timestamp = point_timestamp;
                            // std::cout << "Total time: " << (int)(curr_point.point_timestamp) << std::endl;
                            if (point_timestamp > 255)
                            {
                                std::cout << "    - from_ros: !!!!!!!! THIS IS BROKEY !!!!!!!!" << std::endl;
                            }
                            // curr_point.relative_timestamp = 1;
                            break;
                        case 5:
                            // Flags
                            offset = offset + 1;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.flags = *reinterpret_cast<uint8_t *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point flags: " << curr_point.flags << std::endl;
                            // curr_point.flags = 1;
                            break;
                        case 6:
                            // Channel ID
                            offset = offset + 1;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.channel_id = *reinterpret_cast<uint8_t *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point channel id: " << curr_point.channel_id << std::endl;
                            // curr_point.channel_id = 1;
                            break;
                        case 7:
                            // Valid
                            offset = offset + 1;
                            // std::cout << "Current offset: " << offset << std::endl;
                            curr_point.valid = *reinterpret_cast<uint8_t *>(temp_cloud.data.data() + offset);
                            // std::cout << "Curr point valid: " << curr_point.valid << std::endl;
                            curr_point.valid = temp_cloud.data[offset];
                            // std::cout << "Curr point valid: " << curr_point.valid << std::endl;
                            // curr_point.valid = 1;
                            break;
                        default:
                            std::cout << "    - from_ros: sumthin's bork" << std::endl;
                            break;
                    }
                }

                // std::cout << "Pushing back point" << std::endl;
                cepton_cloud_->push_back(curr_point);
                // std::cout << "Added point" << std::endl;

            }

            cloud_header = temp_cloud.header;
            
            if(params_.debug)
            {
                std::cout << "    - from_ros: " << cloud_header;
                std::cout << "    - from_ros: Finished ROS Conversion" << std::endl;
            }      
        }

};

}


#endif