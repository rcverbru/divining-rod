#ifndef LOCALIZATION_NODE_HPP
#define LOCALIZATION_NODE_HPP

// Utils
#include <utils/msg_converter.hpp>
#include <utils/switcher.hpp>
#include <utils/syncer.hpp>

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
#include <map/example_map.hpp>
// #include <map/hash_map.hpp>
// #include <map/kdtree_map.hpp>
#include <map/octree_map.hpp>
#include <map/voxel_map.hpp>

// Preprocessor Includes
#include <preprocessors/i_preprocessor.hpp>
#include <preprocessors/outlier.hpp>
#include <preprocessors/transform.hpp>

// Velocity Estimator Includes
#include <diviner/i_vestimator.hpp>
#include <vestimator/example_vestimator.hpp>
#include <vestimator/constant_vestimator.hpp>
#include <vestimator/imu_vestimator.hpp>
#include <vestimator/wheel_tick_vestimator.hpp>

// ROS Stuff
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

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

// #include <cassert>
#include <memory>
#include <mutex>
#include <chrono>
#include <queue>
#include <thread>
#include <string>
#include <thread>
#include <vector>
#include <list>

// #define assertm(exp, msg) assert((void(msg), exp))

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

// The new version of the above function
inline
geometry_msgs::PoseWithCovarianceStamped to_pose_with_covariance_stamped(const nav_msgs::Odometry & new_pose)
{
    geometry_msgs::PoseWithCovarianceStamped temp;

    temp.header.stamp = new_pose.header.stamp;
    temp.pose.pose.position.x = new_pose.pose.pose.position.x;
    temp.pose.pose.position.y = new_pose.pose.pose.position.y;
    temp.pose.pose.position.z = new_pose.pose.pose.position.z;
    temp.pose.pose.orientation.x = new_pose.pose.pose.orientation.x;
    temp.pose.pose.orientation.y = new_pose.pose.pose.orientation.y;
    temp.pose.pose.orientation.z = new_pose.pose.pose.orientation.z;
    temp.pose.pose.orientation.w = new_pose.pose.pose.orientation.w;
    temp.pose.covariance = new_pose.pose.covariance;

    return temp;
};

inline
ros::Time pcl_to_ros_time(uint64_t pcl_timestamp) {
    uint32_t sec = pcl_timestamp; // seconds
    std::cout << "pcl to ros sec: " << sec << std::endl;
    uint32_t nsec = pcl_timestamp % static_cast<uint64_t>(1e9); // nanoseconds
    return ros::Time(sec, nsec);
};

inline
void update_vehicle_pose(geometry_msgs::Pose & pose_out, const geometry_msgs::TransformStamped & transform_in, const geometry_msgs::TransformStamped & frame)
{
    // std::cout << "Incoming vehicle Pose: " << vehicle_pose << std::endl;
    // std::cout << "Incoming transform: " << transform_in << std::endl;
    // std::cout << "Incoming cepton->base_link transform: " << frame << std::endl;

    static tf2::Transform map_to_cepton_tf, cepton_to_base_link_tf;

    tf2::fromMsg(transform_in.transform, map_to_cepton_tf);
    tf2::fromMsg(frame.transform, cepton_to_base_link_tf);

    // std::cout << "tf info \nGnss->base_link" << frame << std::endl;
    // std::cout << "map->gnss" << map_to_cepton_tf << std::endl;
    // std::cout << "gnss->base_link" << frame << std::endl;

    auto cepton_to_map_tf = map_to_cepton_tf; // Location of map origin expressed in gnss frame
    auto base_link_to_map_tf = cepton_to_base_link_tf * cepton_to_map_tf; // Location of map origin expressed in base_link frame
    auto map_to_base_link_tf = base_link_to_map_tf; // Location of base_link origin expressed in map frame

    // auto cepton_to_map_tf = map_to_cepton_tf.inverse(); // Location of map origin expressed in gnss frame
    // auto base_link_to_map_tf = cepton_to_base_link_tf * cepton_to_map_tf; // Location of map origin expressed in base_link frame
    // auto map_to_base_link_tf = base_link_to_map_tf.inverse(); // Location of base_link origin expressed in map frame

    auto map_to_base_link = tf2::toMsg(map_to_base_link_tf);
    pose_out.position.x = map_to_base_link.translation.x;
    pose_out.position.y = map_to_base_link.translation.y;
    pose_out.position.z = map_to_base_link.translation.z;
    pose_out.orientation = map_to_base_link.rotation;
};

inline
tf2::Quaternion transform_to_tf2(geometry_msgs::Quaternion quaternion)
{
    tf2::Quaternion tf2_quaternion;
    tf2::fromMsg(quaternion, tf2_quaternion);
    return tf2_quaternion;
};

inline
void zero_static_tf(std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_, const std::string parent_frame, const std::string child_frame)
{
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = child_frame;
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;
    static_tf_br_->sendTransform(static_transformStamped);
}

struct LocalizationNodeParams
{
    // subscribers
    std::string lidar_topic = "/cepton2/points_221339";
    std::string wheel_tick_topic;
    std::string gnss_topic = "/novatel/oem7/odom";
    std::string imu_topic = "/novatel/oem7/corrimu";
    
    // publishers
    std::string local_map_topic = "local_map"; // topic for visualizing local map
    std::string gps_pose_topic = "gps_pose"; // localization based position
    std::string localization_pose_topic = "estimated_pose"; // estimated pose in the odom frame
    std::string path_topic = "previous_path";
    std::string map_tf_topic = "vehicle_pose"; // overwrite map tf topic
    std::string explicit_topic = "explicit";
    
    // debug publishers
    std::string deskewed_pub_topic = "deskewed_cloud"; 
    std::string filtered_pub_topic = "filtered_cloud";
    std::string octree_pub_topic = "octree_visual";

    bool lidar_cb_debug = false;
    bool gnss_cb_debug = false;
    bool topic_debug = false;
    bool explicit_status = false;

    // TF Frames
    std::string world_frame = "world";
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string vehicle_frame = "base_link";
    std::string cepton_frame = "cepton2";
    std::string gnss_frame = "gnss1";

    double diviner_pub_frequency_hz = 20.0;
    double syncer_pub_frequency_hz = 20.0;
    double vehicle_transform_frequency_hz = 20.0;
    double map_transform_frequency_hz = 20.0;
    double explicit_frequency_hz = 20.0;

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

    std::vector<std::string> processors;
    bool processor_debug = false;

    std::string vestimator; // Constant, wheel ticks, or IMU based
    bool vestimator_debug = false;

    std::string point_type;
    bool converter_debug = false;

    double max_angle_std_dev = 3.1 ;
    double max_point_std_dev = 0.1;
    bool switcher_debug = false;

    double max_sync_err_s;
    bool syncer_debug = false;
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
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_;

        bool localization_running;
        bool gps_running;
        std::mutex lidar_mtx_;
        std::mutex gnss_mtx_;
        std::mutex syncer_mtx_;
        std::mutex diviner_mtx_;

        // ROS Subscribers
        ros::Subscriber lidar_sub;
        ros::Subscriber gnss_sub;
        ros::Subscriber imu_sub;

        // ROS Publishers
        ros::Publisher localization_map_pub_;
        ros::Publisher gps_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher path_pub_;
        ros::Publisher map_tf_pub_;
        ros::Publisher deskewed_pub_;
        ros::Publisher voxel_pub_;
        ros::Publisher octree_pub_;
        ros::Publisher explicit_pub_;

        // ROS Timers
        ros::Timer explicit_timer_;
        ros::Timer diviner_timer_;
        ros::Timer syncer_timer_;
        ros::Timer vehicle_transform_timer_;

        // msg converter
        std::shared_ptr<localization_node::MsgConverter> converter_;
        localization_node::MsgConverterParams converter_params_;

        // Switcher
        std::shared_ptr<localization_node::Switcher> switcher_;
        localization_node::SwitcherParams switcher_params_;

        // Syncer
        std::shared_ptr<localization_node::Syncer> syncer_;
        localization_node::SyncerParams syncer_params_;
        
        // Setup for interfaces
        std::shared_ptr<diviner::Diviner> diviner_;
        std::shared_ptr<diviner::IAligner> aligner_;
        std::shared_ptr<diviner::IFilter> filter_;
        std::shared_ptr<diviner::IDeskewer> deskewer_;
        std::shared_ptr<diviner::IMap> map_;
        std::vector<std::shared_ptr<preprocessor::IPreprocessor>> processors_;
        std::shared_ptr<diviner::IVestimator> vestimator_;

        // Setup for params
        const LocalizationNodeParams & ln_params_;
        // diviner::DivinerParams & diviner_params_;
        
        // Aligners
        diviner::Params<diviner::ExampleAlignerParams, diviner::IAlignerParams> example_aligner_params_;
        diviner::Params<diviner::PclAlignerParams, diviner::IAlignerParams> pcl_aligner_params_;
        
        // Deskewers
        diviner::Params<diviner::ExampleDeskewerParams, diviner::IDeskewerParams> example_deskewer_params_;
        diviner::Params<diviner::StandardDeskewerParams, diviner::IDeskewerParams> standard_deskewer_params_;

        // Filters
        diviner::Params<diviner::ExampleFilterParams, diviner::IFilterParams> example_filter_params_;
        diviner::Params<diviner::VoxelFilterParams, diviner::IFilterParams> voxel_filter_params_;
        
        // Maps
        diviner::Params<diviner::ExampleMapParams, diviner::IMapParams> example_map_params_;
        // diviner::Params<diviner::HashMapParams, diviner::IMapParams> hash_map_params_;
        // diviner::Params<diviner::KdTreeMapParams, diviner::IMapParams> kdtree_map_params_;
        diviner::Params<diviner::OctreeMapParams, diviner::IMapParams> octree_map_params_;
        diviner::Params<diviner::VoxelMapParams, diviner::IMapParams> voxel_map_params_;
        
        // Processors
        preprocessor::Params<preprocessor::OutlierProcessorParams, preprocessor::IPreprocessorParams> outlier_processor_params_;
        preprocessor::Params<preprocessor::TransformProcessorParams,preprocessor::IPreprocessorParams> transform_processor_params_;

        // Vestimators
        diviner::Params<diviner::ExampleVestimatorParams, diviner::IVestimatorParams> example_vestimator_params_;
        diviner::Params<diviner::ConstantVestimatorParams, diviner::IVestimatorParams> constant_vestimator_params_;
        diviner::Params<diviner::WheelTickVestimatorParams, diviner::IVestimatorParams> wt_vestimator_params_;
        diviner::Params<diviner::ImuVestimatorParams, diviner::IVestimatorParams> imu_vestimator_params_;

        // gotta be explicit about diviner frame
        void explicit_cb(const ros::TimerEvent & event);

        // Transform Publisher
        bool oneshot = true;
        geometry_msgs::TransformStamped transform_out;
        void updateTransforms(geometry_msgs::PoseStamped &vehicle_location);

        // Transforms
        geometry_msgs::TransformStamped tf_published_;
        geometry_msgs::TransformStamped cepton_to_base_link_;
        geometry_msgs::TransformStamped base_link_to_cepton_;
        geometry_msgs::TransformStamped gnss_to_base_link_;
        geometry_msgs::TransformStamped gnss_to_cepton_;
        geometry_msgs::TransformStamped world_to_map_;
        geometry_msgs::TransformStamped base_link_to_map_stamped_;
        geometry_msgs::TransformStamped map_to_odom_tf;
        std::queue<geometry_msgs::PoseStamped> vehicle_poses_queue_;
        bool curr_transform_updated_ = false;
        void transform_cb(const ros::TimerEvent & event);

        // Diviner
        std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
        nav_msgs::Path est_prev_path, gps_prev_path;
        // std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> veh_pose( new std::vector<geometry_msgs::PoseStamped> )
        // auto veh_pose = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
        // std::vector<diviner::IMUinfo> imu_vec;
        void diviner_cb(const ros::TimerEvent & event);

        // Lidar Cloud
        pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_;
        std::queue<pcl::PointCloud<diviner::PointStamped>> lidar_queue_;
        void lidar_cb(const sensor_msgs::PointCloud2ConstPtr input_cloud_);

        // GNSS
        void gnss_cb(const diviner::GnssType gnss_pos);

        // Syncer
        std::queue<diviner::SyncedMsgs> synced_queue_;
        void syncer_cb(const ros::TimerEvent & event);

        // IMU
        diviner::IMUinfo imu_info;
        std::queue<diviner::IMUinfo> imu_queue_;
        // void imu_cb(const novatel_oem7_msgs::CORRIMU);
};


}

#endif // LOCALIZATION_NODE_HPP