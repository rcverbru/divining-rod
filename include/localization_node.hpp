#ifndef LOCALIZATION_NODE_HPP
#define LOCALIZATION_NODE_HPP

// Utils
#include <utils/msg_converter.hpp>
#include <utils/switcher.hpp>
#include <utils/sync-er.hpp>

// Diviner Includes
#include <diviner/diviner.hpp>

// Aligner Includes
#include <diviner/i_aligner.hpp>
#include <aligner/example_aligner.hpp>
#include <aligner/pcl_aligner.hpp>

// Deskewer Includes
#include <diviner/i_deskewer.hpp>
#include <deskewer/example_deskewer.hpp>
#include <deskewer/standard_deskewer.hpp>

// Filter Includes
#include <diviner/i_filter.hpp>
#include <filter/example_filter.hpp>
#include <filter/voxel_filter.hpp>

// Map Includes
#include <diviner/i_map.hpp>
// #include <map/example_map.hpp>
// #include <map/hash_map.hpp>
// #include <map/kdtree_map.hpp>
#include <map/octree_map.hpp>

// Velocity Estimator Includes
#include <diviner/i_vestimator.hpp>
#include <vestimator/example_vestimator.hpp>
#include <vestimator/constant_vestimator.hpp>
#include <vestimator/imu_vestimator.hpp>
#include <vestimator/wheel_tick_vestimator.hpp>

// ROS Stuff
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL Lidar Stuff
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GNSS
#include <novatel_oem7_msgs/BESTGNSSPOS.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <novatel_oem7_msgs/BESTUTM.h>
#include <novatel_oem7_msgs/CORRIMU.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <mutex>
#include <chrono>
#include <queue>
#include <thread>
#include <string>
#include <thread>
#include <vector>
#include <list>

namespace localization_node
{

const char LOCALIZATION_NODE[] = "localization_node";

inline
geometry_msgs::PoseStamped to_pose_stamped(const nav_msgs::Odometry & new_pose)
{
    geometry_msgs::PoseStamped temp;

    temp.header.stamp = new_pose.header.stamp;
    temp.pose.position.x = new_pose.pose.pose.position.x;
    temp.pose.position.y = new_pose.pose.pose.position.y;
    temp.pose.position.z = new_pose.pose.pose.position.z;
    temp.pose.orientation.x = new_pose.pose.pose.orientation.x;
    temp.pose.orientation.y = new_pose.pose.pose.orientation.y;
    temp.pose.orientation.z = new_pose.pose.pose.orientation.z;
    temp.pose.orientation.w = new_pose.pose.pose.orientation.w;

    return temp;
};

inline
ros::Time pclToRosTime(uint64_t pcl_timestamp) {
    uint32_t sec = pcl_timestamp; // seconds
    std::cout << "pcl to ros sec: " << sec << std::endl;
    uint32_t nsec = pcl_timestamp % static_cast<uint64_t>(1e9); // nanoseconds
    return ros::Time(sec, nsec);
};

struct LocalizationNodeParams
{
    std::string lidar_topic = "/cepton2/points_221339";
    std::string wheel_tick_topic;
    std::string gnss_topic = "/novatel/oem7/odom";
    std::string imu_topic = "/novatel/oem7/corrimu";
    std::string map_tf_topic = "/vehicle_pose";

    std::string local_map_topic = "localization/local_map";
    std::string position_topic = "localization/position";
    std::string deskewed_pub_topic = "localization/deskewed_cloud";
    std::string filtered_pub_topic = "localization/filtered_cloud";

    bool topic_debug = false;

    std::string map_frame = "map";
    std::string vehicle_frame = "base_link";
    std::string cepton_frame = "cepton2";
    std::string world_frame = "world";
    std::string gnss_frame = "gnss1";

    double diviner_pub_frequency_hz = 20.0;
    double vehicle_transform_frequency_hz = 20.0;
    double map_transform_frequency_hz = 20.0;

    std::string running_state; // GNSS only, Localization only, or Both
    bool debug = false;

    bool diviner_debug = false;

    std::string aligner;
    double convergence_criterion;
    bool aligner_debug = false;

    std::string deskewer;
    bool deskewer_debug = false;

    std::vector<float> leaf_size;
    std::string filter;
    bool filter_debug = false;

    double voxel_size;
    double max_distance;
    int max_points_per_voxel;
    std::string map;
    bool map_debug = false;

    std::string vestimator; // Constant, wheel ticks, or IMU based
    bool vestimator_debug = false;

    double max_sync_err;
    std::string point_type;
    bool converter_debug = false;
};

class LocalizationNode
{
    public:
        explicit LocalizationNode(
            std::shared_ptr<ros::NodeHandle> node_,
            const LocalizationNodeParams & ln_params);
    private:
        // ROS setup
        std::shared_ptr<ros::NodeHandle> node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        bool localization_running;
        std::mutex lidar_mtx_;
        std::mutex gnss_mtx_;
        std::mutex diviner_mtx_;

        // ROS Subscribers
        ros::Subscriber lidar_sub;
        ros::Subscriber gnss_sub;
        ros::Subscriber imu_sub;

        // ROS Publishers
        ros::Publisher localization_pub_;
        ros::Publisher position_pub_;
        ros::Publisher deskewed_pub_;
        ros::Publisher voxel_pub_;

        // ROS Timers
        ros::Timer diviner_timer_;
        ros::Timer vehicle_transform_timer_;

        // msg converter
        std::shared_ptr<diviner::MsgConverter> converter_;
        diviner::MsgConverterParams converter_params_;

        // Switcher
        // std::shared_ptr<diviner::Switcher> switcher_;
        // diviner::SwitcherParams switcher_params_;

        // // Sync-er
        // std::shared_ptr<diviner::Syncer> syncer_;
        // diviner::SyncerParams syncer_params_;
       
        // Setup for interfaces
        std::shared_ptr<diviner::Diviner> diviner_;
        std::shared_ptr<diviner::IAligner> aligner_;
        std::shared_ptr<diviner::IFilter> filter_;
        std::shared_ptr<diviner::IDeskewer> deskewer_;
        std::shared_ptr<diviner::IMap> map_;
        std::shared_ptr<diviner::IVestimator> vestimator_;

        // Setup for params
        const LocalizationNodeParams & ln_params_;
        // diviner::DivinerParams & diviner_params_;
        
        // Aligners
        diviner::Params<diviner::ExampleAlignerParams, diviner::IAlignerParams> example_aligner_params_;
        diviner::Params<diviner::PclAlignerParams, diviner::IAlignerParams> pcl_aligner_params_;
        
        // Deskewers
        diviner::Params<diviner::ExampleDeskewerParams, diviner::IDeskewerParams> example_deskewer_params_;
        // diviner::Params<diviner::StandardDeskewerParams, diviner::IDeskewerParams> standard_deskewer_params_;

        // Filters
        diviner::Params<diviner::ExampleFilterParams, diviner::IFilterParams> example_filter_params_;
        diviner::Params<diviner::VoxelFilterParams, diviner::IFilterParams> voxel_filter_params_;
        
        // Maps
        // diviner::Params<diviner::ExampleMapParams, diviner::IMapParams> example_map_params_;
        // diviner::Params<diviner::HashMapParams, diviner::IMapParams> hash_map_params_;
        // diviner::Params<diviner::KdTreeMapParams, diviner::IMapParams> kdtree_map_params_;
        diviner::Params<diviner::OctreeMapParams, diviner::IMapParams> octree_map_params_;
        
        // Vestimators
        diviner::Params<diviner::ExampleVestimatorParams, diviner::IVestimatorParams> example_vestimator_params_;
        diviner::Params<diviner::ConstantVestimatorParams, diviner::IVestimatorParams> constant_vestimator_params_;
        diviner::Params<diviner::WheelTickVestimatorParams, diviner::IVestimatorParams> wt_vestimator_params_;
        diviner::Params<diviner::ImuVestimatorParams, diviner::IVestimatorParams> imu_vestimator_params_;

        // Transform Publisher
        geometry_msgs::TransformStamped transform_out;
        void update_transforms();

        // Transforms
        geometry_msgs::TransformStamped cepton_to_vehicle_;
        geometry_msgs::TransformStamped world_to_map_;
        geometry_msgs::TransformStamped curr_transform_stamped_;
        geometry_msgs::TransformStamped base_link_to_map_stamped_;
        std::queue<geometry_msgs::PoseStamped> vehicle_poses_queue_;
        bool curr_transform_updated_ = false;
        void transform_cb(const ros::TimerEvent & event);

        // Diviner
        std::vector<geometry_msgs::PoseStamped> veh_pose;
        std::vector<diviner::IMUinfo> imu_vec;
        void diviner_cb(const ros::TimerEvent & event);

        // Lidar Cloud
        pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_;
        std::queue<pcl::PointCloud<diviner::PointStamped>::Ptr> lidar_queue_;
        void lidar_cb(const sensor_msgs::PointCloud2ConstPtr input_cloud_);

        // GNSS
        void gnss_cb(const diviner::GnssType gnss_pos);

        // IMU
        diviner::IMUinfo imu_info;
        std::queue<diviner::IMUinfo> imu_queue_;
        void imu_cb(const novatel_oem7_msgs::CORRIMU);
};


}

#endif // LOCALIZATION_NODE_HPP