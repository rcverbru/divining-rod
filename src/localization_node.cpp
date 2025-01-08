#include <localization_node.hpp>

namespace localization_node
{

LocalizationNode::LocalizationNode(
    std::shared_ptr<ros::NodeHandle> node,
    const LocalizationNodeParams & ln_params) : 
    node_(node),
    ln_params_(ln_params)
{
    // Set Running State
    if(ln_params_.running_state == "gnss")
    {
        localization_running = false;

        // runs only gnss system
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with only GNSS");
    }
    else if(ln_params_.running_state == "localization")
    {
        // runs only localization system
        localization_running = true;
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with only Localization System");
    }
    else if(ln_params_.running_state == "switching")
    {
        localization_running = true;

        // switches between both depending on gnss state
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with Automatic System Switching");
        
        // create gps standard deviation trackers
        // std::make_shared<diviner::

        // we should be able to subscribe to one of the gps topics and find the standard deviation of the gps
        // then we can set a certain std dev to be the cut off point to switch to lidar localization instead of 
        // running off the gps.
    }
    else if(ln_params_.running_state == "both")
    {
        // runs both at the same time
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with Both GNSS and Localization System");
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Running state %s is not valid", ln_params_.running_state.c_str());
    }

    // Set up Aligner
    if(ln_params_.aligner == "examplealigner")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Aligner");
        }

        example_aligner_params_.parent_params.debug = ln_params_.debug;
        aligner_ = std::make_shared<diviner::ExampleAligner>(example_aligner_params_);
    }
    if(ln_params_.aligner == "pclaligner")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using PCL Aligner");
        }

        pcl_aligner_params_.parent_params.debug = ln_params_.debug;
        aligner_ = std::make_shared<diviner::PclAligner>(pcl_aligner_params_);
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Aligner %s is not valid", ln_params_.aligner.c_str());
    }

    // Set up Deskewer
    if(ln_params_.deskewer == "exampledeskewer")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Deskewer");
        }

        example_deskewer_params_.parent_params.debug = ln_params_.debug;
        deskewer_ = std::make_shared<diviner::ExampleDeskewer>(example_deskewer_params_);
    }
    // else if(ln_params_.deskewer == "standarddeskewer")
    // {
    //     example_deskewer_params_.parent_params.debug = ln_params_.debug;
    //     deskewer_ = std::make_shared<diviner::StandardDeskewer>(standard_deskewer_params_);
    // }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Deskewer %s is not valid", ln_params_.deskewer.c_str());
    }

    // Set up Filter
    if(ln_params_.filter == "examplefilter")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Filter");
        }
        
        example_filter_params_.parent_params.debug = ln_params_.debug;
        filter_ = std::make_shared<diviner::ExampleFilter>(example_filter_params_);
    }
    else if(ln_params_.filter == "voxelfilter")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Voxel Filter");
        }

        voxel_filter_params_.parent_params.debug = ln_params_.debug;
        voxel_filter_params_.child_params.leaf_size = ln_params_.leaf_size;
        filter_ = std::make_shared<diviner::VoxelFilter>(voxel_filter_params_);
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Filter %s is not valid", ln_params_.filter.c_str());
    }

    // Set up Mapper
    if(ln_params_.map == "examplemap") {}
    // {
    //     if(ln_params_.debug)
    //     {
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Mapper");
    //     }

    //     example_map_params_.parent_params.debug = ln_params_.map_debug;
    //     map_ = std::make_shared<diviner::ExampleMap>(example_map_params_);
    // }
    // else if(ln_params_.map == "hashmap")
    // {
    //     if(ln_params_.debug)
    //     {
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Hash Mapper");
    //     }

    //     hash_map_params_.parent_params.debug = ln_params_.map_debug;
    //     map_ = std::make_shared<diviner::HashMap>(hash_map_params_);
    // }
    // else if(ln_params_.map == "kdtreemap")
    // {
    //     if(ln_params_.debug)
    //     {
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using KdTree Mapper");
    //     }

    //     kdtree_map_params_.parent_params.debug = ln_params_.map_debug;
    //     map_ = std::make_shared<diviner::KdTreeMap>(kdtree_map_params_);
    // }
    else if(ln_params_.map == "octreemap")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Octree Mapper");
        }

        octree_map_params_.parent_params.debug = ln_params_.map_debug;
        map_ = std::make_shared<diviner::OctreeMap>(octree_map_params_);
        // map_->setResolution(0.1)
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Mapper %s is not valid", ln_params_.map.c_str());
    }

    // Set Velocity tracking
    if(ln_params_.vestimator == "examplevestimator")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Velocity Estimator");
        }

        example_vestimator_params_.parent_params.debug = ln_params_.debug;
        example_vestimator_params_.child_params.debug = ln_params_.vestimator_debug;
        vestimator_ = std::make_shared<diviner::ExampleVestimator>(example_vestimator_params_);
    }
    else if(ln_params_.vestimator == "constant")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Constant Velocity Estimator");
        }

        constant_vestimator_params_.parent_params.debug = ln_params_.debug;
        constant_vestimator_params_.child_params.debug = ln_params_.vestimator_debug;
        vestimator_ = std::make_shared<diviner::ConstantVestimator>(constant_vestimator_params_);
    }
    else if(ln_params_.vestimator == "wheelticks")
    {
        //
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Wheel Tick Velocity Estimator");
        }
        
        wt_vestimator_params_.parent_params.debug = ln_params_.debug;
        wt_vestimator_params_.child_params.debug = ln_params_.vestimator_debug;
        vestimator_ = std::make_shared<diviner::WheelTickVestimator>(wt_vestimator_params_);
    }
    else if(ln_params_.vestimator == "imu")
    {
        //
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using IMU Velocity Estimator");
        }
        imu_vestimator_params_.parent_params.debug = ln_params_.debug;
        imu_vestimator_params_.child_params.debug = ln_params_.vestimator_debug;
        vestimator_ = std::make_shared<diviner::ImuVestimator>(imu_vestimator_params_);
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Velocity tracker %s is not valid", ln_params_.vestimator.c_str());
    }

    if(ln_params_.debug)
    {
        std::cout << "starting tf search" << std::endl;
    }

    // Moving to down here so that we can set up the map with the proper map type before passing it to this
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Might not need this...
    // bool world_to_map_updated = false;
    // while (!world_to_map_updated)
    // {
    //     try
    //     {
    //         world_to_map_ = tf_buffer_->lookupTransform(ln_params_.map_frame, ln_params_.world_frame, ros::Time(0));
    //         world_to_map_updated = true;
                        
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got world->map transform");
    //     }
    //     catch(tf2::TransformException & ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

    // bool cepton_to_vehicle_updated = false;
    // while (!cepton_to_vehicle_updated)
    // {
    //     try
    //     {
    //         cepton_to_vehicle_ = tf_buffer_->lookupTransform(ln_params_.vehicle_frame, ln_params_.cepton_frame, ros::Time(0));
    //         cepton_to_vehicle_updated = true;
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got cepton->vehicle transform");
    //     }
    //     catch(tf2::TransformException & ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

    if(diviner_ == nullptr)
    {
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Starting up...");
        diviner_ = std::make_shared<diviner::Diviner>(aligner_, filter_, deskewer_, map_, vestimator_, ln_params_.debug);
        
        // Set current scan
        pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_ = 
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);
    }

    // Create MsgConverter
    converter_params_.debug = ln_params_.converter_debug;
    converter_params_.point_type = ln_params_.point_type;
    converter_ = std::make_shared<diviner::MsgConverter>(converter_params_);

    // syncer_params_.debug = ln_params.debug;
    // syncer__ = std::make_shared<diviner::Syncer>(syncer_params_);


    // Subscribers
    lidar_sub = node_->subscribe<sensor_msgs::PointCloud2ConstPtr>(ln_params_.lidar_topic, 1, &LocalizationNode::lidar_cb, this);
    gnss_sub = node_->subscribe<diviner::GnssType>(ln_params_.gnss_topic, 1, &LocalizationNode::gnss_cb, this);
    imu_sub = node_->subscribe<novatel_oem7_msgs::CORRIMU>(ln_params_.imu_topic, 1, &LocalizationNode::imu_cb, this);

    // Publishers
    localization_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.local_map_topic, 1, true);
    position_pub_ = node_->advertise<geometry_msgs::Pose>(ln_params_.position_topic, 1, true);

    if(ln_params_.topic_debug)
    {
        deskewed_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.deskewed_pub_topic, 1, true);
        voxel_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.filtered_pub_topic, 1, true);
    }

    // Timers
    diviner_timer_ = node_->createTimer(ros::Duration(1.0 / ln_params_.diviner_pub_frequency_hz), &LocalizationNode::diviner_cb, this);
    // vehicle_transform_timer_ = node_->createTimer(ros::Duration(1.0 / ln_params_.vehicle_transform_frequency_hz), &LocalizationNode::transform_cb, this);

}

void LocalizationNode::update_transforms()
{
    // pull in current transform and check switching status
    // output the lovely transform for the rest of the vehicle
}

void LocalizationNode::transform_cb(const ros::TimerEvent & event)
{
    try
    {
        geometry_msgs::PoseStamped vehicle_pose_;

        curr_transform_stamped_ = tf_buffer_->lookupTransform(ln_params_.map_frame, ln_params_.vehicle_frame, ros::Time(0));
        base_link_to_map_stamped_ = tf_buffer_->lookupTransform(ln_params_.map_frame, ln_params_.vehicle_frame, ros::Time(0));

        vehicle_pose_.header = curr_transform_stamped_.header;
        vehicle_pose_.pose.position.x = curr_transform_stamped_.transform.translation.x;
        vehicle_pose_.pose.position.y = curr_transform_stamped_.transform.translation.y;
        vehicle_pose_.pose.position.z = curr_transform_stamped_.transform.translation.z;
        vehicle_pose_.pose.orientation = curr_transform_stamped_.transform.rotation;

        std::cout << vehicle_poses_queue_.size() << std::endl;
        if(vehicle_poses_queue_.size() == 0u)
        {
            // If pose queue empty add two extra poses
            vehicle_poses_queue_.push(vehicle_pose_);
            vehicle_poses_queue_.push(vehicle_pose_);
        }
        
        vehicle_poses_queue_.push(vehicle_pose_);
        std::cout << vehicle_poses_queue_.size() << std::endl; // should equal to 3 after this
        std::cout << "transform_cb: " << vehicle_pose_.pose.position.x << std::endl;

        // traversed_distance_since_update_ = find_2d_pose_distance(vehicle_pose_.pose, vehicle_pose_at_update_);

        curr_transform_updated_ = true;
    }
    catch(tf2::TransformException & ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

void LocalizationNode::diviner_cb(const ros::TimerEvent & event)
{
    diviner_mtx_.lock();
    
    if(ln_params_.debug)
    {
        std::cout << "diviner_cb: Inside diviner_cb." << std::endl;
    }

    if(current_scan_ != nullptr && current_scan_->size() > 0)
    {
        std::cout << "diviner_cb: " << current_scan_->size() << " points before stepping." << std::endl;

        if(map_ != nullptr)
        {
            std::cout << "diviner_cb: Map size: " << map_->capacity() << std::endl;
            // Place new pose at index 0 and then remove the oldest pose
            // veh_pose.emplace(veh_pose.begin(), vehicle_poses_queue_.front());
            // std::cout << "diviner_cb: Added pose to vector from queue. " << vehicle_poses_queue_.size() << std::endl;
            // vehicle_poses_queue_.pop();

            // // Remove oldest vehicle pose from vector
            // if(veh_pose.size() == 4)
            // {
            //     veh_pose.pop_back();
            //     std::cout << "diviner_cb: removing 1 pose from pose vector... " << std::endl;
            // }
            
            // std::cout << "diviner_cb: Num poses in vector: " << veh_pose.size() << std::endl;
            
            // might need to move this somewhere else depending how well this works...
            std::cout << "diviner_cb: Starting Diviner Step." << std::endl;
            diviner_->step(current_scan_, world_to_map_, cepton_to_vehicle_, veh_pose);
        }
        else
        {
            ROS_ERROR("diviner_cb: No map! Need to set map param!");
        }
    }
    else
    {
        std::cout << "diviner_cb: Need to wait for new scan..." << std::endl;
    }
    
    // if(map_ != nullptr)
    // {
    //     bool cloud_synced;
    //     std::cout << "diviner_cb: Num poses in queue: " << vehicle_poses_queue_.size() << std::endl;
    //     // cloud_synced = syncer_->sync(current_scan_, vehicle_poses_queue_);

    //     if(vehicle_poses_queue_.size() > 0)
    //     {

    //         if(current_scan_ != nullptr)
    //         {
    //             // std::cout << "diviner_cb: Inside of scan if" << std::endl;
    //             // while(!cloud_synced)
    //             // {
    //                 std::cout << "diviner_cb: Inside of sync loop" << std::endl;
                    
    //                 // need to also add imu sync...
    //                 diviner::IMUinfo temp_imu;
    //                 // temp_imu = 

    //                 auto vehicle_pose = vehicle_poses_queue_.front();
    //                 auto scan = *current_scan_;

    //                 std::cout << current_scan_->header.stamp << std::endl;
    //                 std::cout << (vehicle_pose.header.stamp).toNSec() << std::endl;

    //                 auto scan_time_stamp = pclToRosTime(scan.header.stamp);

    //                 std::cout << "diviner_cb: scan time in us: " << scan_time_stamp << std::endl;
    //                 std::cout << "diviner_cb: pose time in us: " << vehicle_pose.header.stamp << std::endl;
    //                 std::cout << "diviner_cb: imu time in us: " << temp_imu.stamp << std::endl;

    //                 float time_dif = (scan_time_stamp - vehicle_pose.header.stamp).toSec();
                    
    //                 // if(time_dif < ln_params_.max_sync_err && time_dif > -ln_params_.max_sync_err)
    //                 // {
    //                     std::cout << "diviner_cb: Current scan stamp: " << scan_time_stamp << std::endl;
    //                     std::cout << "diviner_cb: Current pose stamp: " << vehicle_pose.header.stamp << std::endl;
                        
    //                     // Current vehicle pose is within 
    //                     // cloud_synced = true;
    //                     std::cout << "diviner_cb: vector size: " << veh_pose.size() << std::endl;
    //                     if(veh_pose.size() == 0)
    //                     {
    //                         // Adding 2 extra points to assume start = 0 velocity
    //                         veh_pose.emplace(veh_pose.begin(), vehicle_poses_queue_.front());
    //                         veh_pose.emplace(veh_pose.begin(), vehicle_poses_queue_.front());
    //                     }
                       
    //                     // Place new pose at index 0 and then remove the oldest pose
    //                     veh_pose.emplace(veh_pose.begin(), vehicle_poses_queue_.front());
    //                     std::cout << "diviner_cb: Added pose to vector from queue. " << vehicle_poses_queue_.size() << std::endl;
    //                     vehicle_poses_queue_.pop();
    //                     std::cout << "diviner_cb: Removed 1 pose from queue. Current size: " << vehicle_poses_queue_.size() << std::endl;
                        
    //                     if(veh_pose.size() == 4)
    //                     {
    //                         veh_pose.pop_back();
    //                         std::cout << "diviner_cb: removing 1 pose from pose vector... " << std::endl;
    //                     }

    //                     std::cout << "diviner_cb: Num poses in vector: " << veh_pose.size() << std::endl;
    //                     // might need to move this somewhere else depending how well this works...
    //                     std::cout << "Starting Diviner Step." << std::endl;
    //                     diviner_->step(current_scan_, world_to_map_, cepton_to_vehicle_, veh_pose);
    //                 // }
    //                 // else
    //                 // {
    //                 //     // remove old vehicle poses
    //                 //     std::cout << "diviner_cb: Pose is too old, Deleting pose and trying next one." << std::endl;
    //                 //     vehicle_poses_queue_.pop();
    //                 // }

    //             // }
                
    //         }
    //         else
    //         {
    //             std::cout << "diviner_cb: Need to wait for new scan..." << std::endl;
    //         }
    //     }
    //     else
    //     {
    //         std::cout << "diviner_cb: Need to wait for poses..." << std::endl;
    //     }
    // }
    // else
    // {
    //     ROS_ERROR("diviner_cb: No map! Need to set map param!");

    // }
    
    diviner_mtx_.unlock();
}

void LocalizationNode::lidar_cb(const sensor_msgs::PointCloud2ConstPtr input_cloud)
{
    // add mutex for data
    lidar_mtx_.lock();

    // pcl::PointCloud<diviner::PointStamped>::Ptr current_scan(new pcl::PointCloud<diviner::PointStamped>);
    // pcl::fromROSMsg(*input_cloud, *current_scan);

    if(ln_params_.debug)
    {
        std::cout << "- lidar_cb: Clearing Current Scan" << std::endl;
    }

    if(current_scan_ != nullptr)
    {
        current_scan_->clear();
        if(ln_params_.debug)
        {
        std::cout << "- lidar_cb: Scan Cleared" << std::endl;
        }
    }
    else
    {
        if(ln_params_.debug)
        {
        std::cout << "- lidar_cb: Scan already empty" << std::endl;
        }
    }

    current_scan_ = converter_->convert(input_cloud);
    
    while(current_scan_ == nullptr)
    {
        std::cout << "- lidar_cb: Waiting for current scan" << std::endl;
    }

    std::cout << "- lidar_cb: Current scan has " << current_scan_->size() << " points." << std::endl;
    std::cout << "- lidar_cb: Current scan has time stamp: " << current_scan_->header.stamp << std::endl;

    lidar_queue_.push(current_scan_);
    std::cout << "- lidar_cb: Current queue has " << lidar_queue_.size() << " scans in queue." << std::endl;

    lidar_mtx_.unlock();
}

void LocalizationNode::gnss_cb(const diviner::GnssType gnss_pose)
{
    gnss_mtx_.lock();
    
    // TODO: add a conditional that checks if the covariance is below 0.0001
    // or take sqrt(covariance) and check std deviation
    
    geometry_msgs::PoseStamped vehicle_pose;
    geometry_msgs::PoseStamped map_vehicle_pose;
    
    // If diviner::GnssType is nav_msgs::OdometryConstPtr
    vehicle_pose = to_pose_stamped(*gnss_pose);
    tf2::doTransform(vehicle_pose, map_vehicle_pose, world_to_map_);
    
    // why tf does tf2 remove the stamp????? I'm going to lose it...
    map_vehicle_pose.header.stamp = vehicle_pose.header.stamp;

    if(ln_params_.debug)
    {
        std::cout << "- gnss_cb: number of queued poses: " << vehicle_poses_queue_.size() << std::endl;
    }

    if(vehicle_poses_queue_.size() == 0u)
    {
        // If pose queue empty add two extra poses
        vehicle_poses_queue_.push(map_vehicle_pose);
        vehicle_poses_queue_.push(map_vehicle_pose);
    }
    // add to poses queue
    vehicle_poses_queue_.push(map_vehicle_pose);
    
    if(ln_params_.debug)
    {
        std::cout << "- gnss_cb: New pose added, curr num vehicle poses in queue: " << vehicle_poses_queue_.size() << std::endl;
    }

    gnss_mtx_.unlock();
}

void LocalizationNode::imu_cb(const novatel_oem7_msgs::CORRIMU msg)
{
    // Take in imu data
    imu_info.roll = msg.roll_rate;
    imu_info.pitch = msg.pitch_rate;
    imu_info.yaw = msg.yaw_rate;
    imu_info.stamp = msg.header.stamp;
    
    // Add to imu queue
    imu_queue_.push(imu_info);
}

}