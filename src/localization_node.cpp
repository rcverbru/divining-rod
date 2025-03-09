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
        gps_running = true;

        // runs only gnss system
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with only GNSS");
    }
    else if(ln_params_.running_state == "localization")
    {
        // runs only localization system
        localization_running = true;
        gps_running = false;
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Running with only Localization System");
    }
    else if(ln_params_.running_state == "switching")
    {
        // in developement still
        localization_running = true;
        gps_running = true;

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
        example_aligner_params_.child_params.debug = ln_params_.aligner_debug;
        aligner_ = std::make_shared<diviner::ExampleAligner>(example_aligner_params_);
    }
    if(ln_params_.aligner == "pclaligner")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using PCL Aligner");
        }

        pcl_aligner_params_.parent_params.debug = ln_params_.debug;
        pcl_aligner_params_.child_params.debug = ln_params_.aligner_debug;
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
        example_deskewer_params_.child_params.debug = ln_params_.deskewer_debug;
        deskewer_ = std::make_shared<diviner::ExampleDeskewer>(example_deskewer_params_);
    }
    // else if(ln_params_.deskewer == "standarddeskewer")
    // {
    //     standard_deskewer_params_.parent_params.debug = ln_params_.debug;
    //     standard_deskewer_params_.child_params.debug = ln_params_.deskewer_debug;

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
        example_filter_params_.child_params.debug = ln_params_.filter_debug;
        filter_ = std::make_shared<diviner::ExampleFilter>(example_filter_params_);
    }
    else if(ln_params_.filter == "voxelfilter")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Voxel Filter");
        }

        voxel_filter_params_.parent_params.debug = ln_params_.debug;
        voxel_filter_params_.child_params.debug = ln_params_.filter_debug;
        voxel_filter_params_.child_params.leaf_size = ln_params_.leaf_size;
        filter_ = std::make_shared<diviner::VoxelFilter>(voxel_filter_params_);
    }
    else
    {
        ROS_WARN_NAMED(localization_node::LOCALIZATION_NODE, "Filter %s is not valid", ln_params_.filter.c_str());
    }

    // Set up Mapper
    if(ln_params_.map == "examplemap")
    {
        if(ln_params_.debug)
        {
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Using Example Mapper");
        }

        example_map_params_.parent_params.debug = ln_params_.map_debug;
        map_ = std::make_shared<diviner::ExampleMap>(example_map_params_);
    }
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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    // bool gnss_to_cepton_updated = false;
    // while (!gnss_to_cepton_updated)
    // {
    //     try
    //     {
    //         gnss_to_cepton_ = tf_buffer_->lookupTransform(ln_params_.cepton_frame, ln_params_.gnss_frame, ros::Time(0));
    //         gnss_to_cepton_updated = true;
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got cepton->vehicle transform");
    //     }
    //     catch(tf2::TransformException & ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

    // bool gnss_to_base_link_updated = false;
    // while (!gnss_to_base_link_updated)
    // {
    //     try
    //     {
    //         gnss_to_base_link_ = tf_buffer_->lookupTransform(ln_params_.vehicle_frame, ln_params_.gnss_frame, ros::Time(0));
    //         gnss_to_base_link_updated = true;
    //         ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got cepton->vehicle transform");
    //     }
    //     catch(tf2::TransformException & ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }
    
    bool cepton_to_base_link_updated = false;
    while (!cepton_to_base_link_updated)
    {
        try
        {
            cepton_to_base_link_ = tf_buffer_->lookupTransform(ln_params_.vehicle_frame, ln_params_.cepton_frame, ros::Time(0));
            cepton_to_base_link_updated = true;
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got cepton->vehicle transform");
        }
        catch(tf2::TransformException & ex)
        {
            ROS_WARN("%s", ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool base_link_to_cepton_updated = false;
    while (!base_link_to_cepton_updated)
    {
        try
        {
            base_link_to_cepton_ = tf_buffer_->lookupTransform(ln_params_.cepton_frame, ln_params_.vehicle_frame, ros::Time(0));
            base_link_to_cepton_updated = true;
            ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Got vehicle->cepton transform");
        }
        catch(tf2::TransformException & ex)
        {
            ROS_WARN("%s", ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if(diviner_ == nullptr)
    {
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Starting up...");
        diviner_ = std::make_shared<diviner::Diviner>(aligner_, filter_, deskewer_, map_, vestimator_, ln_params_.diviner_debug);
        
        // Set current scan
        pcl::PointCloud<diviner::PointStamped>::Ptr current_scan_ = 
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);
        // veh_pose = std::make_shared<std::vector<geometry_msgs::PointStamped>>();
    }

    // Create MsgConverter
    converter_params_.debug = ln_params_.converter_debug;
    converter_params_.point_type = ln_params_.point_type;
    converter_ = std::make_shared<diviner::MsgConverter>(converter_params_);

    syncer_params_.debug = ln_params.syncer_debug;
    // syncer_params_.max_sync_err_s = ln_params.max_sync_err_s;
    syncer_ = std::make_shared<diviner::Syncer>(syncer_params_);


    // Subscribers
    lidar_sub = node_->subscribe<sensor_msgs::PointCloud2ConstPtr>(ln_params_.lidar_topic, 1, &LocalizationNode::lidar_cb, this);
    gnss_sub = node_->subscribe<diviner::GnssType>(ln_params_.gnss_topic, 1, &LocalizationNode::gnss_cb, this);
    // imu_sub = node_->subscribe<novatel_oem7_msgs::CORRIMU>(ln_params_.imu_topic, 1, &LocalizationNode::imu_cb, this);

    // Publishers
    localization_map_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.local_map_topic, 1, true);
    gps_pub_ = node_->advertise<geometry_msgs::Pose>(ln_params_.gps_pose_topic, 1, true);
    pose_pub_ = node_->advertise<geometry_msgs::PoseStamped>(ln_params_.localization_pose_topic, 1, true);
    map_tf_pub_ = node_->advertise<geometry_msgs::Pose>(ln_params_.map_tf_topic, 1);

    if(ln_params_.topic_debug)
    {
        deskewed_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.deskewed_pub_topic, 1, true);
        voxel_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.filtered_pub_topic, 1, true);
        octree_pub_ = node_->advertise<sensor_msgs::PointCloud2>(ln_params_.octree_pub_topic, 1, true);
    }

    // Timers
    // std::cout << "Localization Node: localization_running bool = " << localization_running << std::endl;
    if(localization_running)
    {
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Starting Diviner Timers.");
        
        diviner_timer_ = node_->createTimer(ros::Duration(1.0 / ln_params_.diviner_pub_frequency_hz), &LocalizationNode::diviner_cb, this);
        syncer_timer_ = node_->createTimer(ros::Duration(1.0 / ln_params_.syncer_pub_frequency_hz), &LocalizationNode::syncer_cb, this);
    }
    else
    {
        ROS_INFO_NAMED(localization_node::LOCALIZATION_NODE, "Diviner not running. Letting gnss_cb handle updating transforms.");
        // output vehicle position data immediately
        // if(!vehicle_poses_queue_.empty())
        // {
        //     LocalizationNode::updateTransforms(vehicle_poses_queue_.front());
        //     vehicle_poses_queue_.pop();
        // }
    }

    // Old Timers
    // vehicle_transform_timer_ = node_->createTimer(ros::Duration(1.0 / ln_params_.vehicle_transform_frequency_hz), &LocalizationNode::transform_cb, this);

}

void LocalizationNode::updateTransforms(geometry_msgs::PoseStamped &vehicle_location)
{
    bool extreme_tf_debug = false;
    geometry_msgs::Pose updated_pose;

    // // Need to set up status checks and decide how we want to do this
    // switcher_->check_status();

    if(localization_running && !gps_running)
    {
        // Only localization mode dumps estimated pose into the odom frame
        // BUT doesn't search for a odom->map transform.
        // Also broadcasts the odom frame to the /tf topic.

        geometry_msgs::PoseStamped pose_holder;
        
        if(extreme_tf_debug)
        {
            std::cout << "- BEFORE doTransform: Transform cepton->base_link: \n" << cepton_to_base_link_ << std::endl;
            std::cout << "- BEFORE doTransform: Transform base_link->cepton: \n" << base_link_to_cepton_ << std::endl;
            std::cout << "- BEFORE doTransform: Before transform " << vehicle_location << std::endl;
            std::cout << "- BEFORE doTransform: Pose before do transform: \n" << pose_holder << std::endl;
        }

        tf2::doTransform(vehicle_location, pose_holder, cepton_to_base_link_);
        
        if(extreme_tf_debug)
        {
            std::cout << "- AFTER doTransform: Transform cepton->base_link: \n" << cepton_to_base_link_ << std::endl;
            std::cout << "- AFTER doTransform: Transform base_link->cepton: \n" << base_link_to_cepton_ << std::endl;
            std::cout << "- AFTER doTransform: after transform \n" << vehicle_location << std::endl;
            std::cout << "- AFTER doTransform: Pose pulled: \n" << pose_holder << std::endl;
        }

        tf_published_.header.stamp = ros::Time::now();
        tf_published_.header.frame_id = ln_params_.odom_frame;
        tf_published_.child_frame_id = ln_params_.cepton_frame;
        tf_published_.transform.translation.x = pose_holder.pose.position.x;
        tf_published_.transform.translation.y = pose_holder.pose.position.y;
        tf_published_.transform.translation.z = pose_holder.pose.position.z;
        tf_published_.transform.rotation = vehicle_location.pose.orientation;

        if(extreme_tf_debug)
        {
            std::cout << "before update_vehicle_pose - pose queue front: " << std::endl << vehicle_location << std::endl;
        }

        update_vehicle_pose(updated_pose, tf_published_, cepton_to_base_link_);

        if(extreme_tf_debug)
        {
            std::cout << "after update_vehicle_pose - pose queue front: " << std::endl << vehicle_location << std::endl;
        }
    }
    else if(!localization_running && gps_running)
    {
        geometry_msgs::PoseStamped pose_holder;

        tf2::doTransform(vehicle_location, pose_holder, world_to_map_);

        tf_published_.header.stamp = ros::Time::now();
        tf_published_.header.frame_id = ln_params_.map_frame;
        tf_published_.child_frame_id = ln_params_.gnss_frame;
        tf_published_.transform.translation.x = vehicle_location.pose.position.x;
        tf_published_.transform.translation.y = vehicle_location.pose.position.y;
        tf_published_.transform.translation.z = 0;
        tf_published_.transform.rotation = vehicle_location.pose.orientation;

        update_vehicle_pose(updated_pose, tf_published_, gnss_to_base_link_);
    }
    else if(localization_running && gps_running)
    {
        // TODO: Need to update to include odom->map tf

        geometry_msgs::PoseStamped pose_holder;

        tf2::doTransform(vehicle_location, pose_holder, cepton_to_base_link_);

        tf_published_.header.stamp = ros::Time::now();
        tf_published_.header.frame_id = ln_params_.odom_frame;
        tf_published_.child_frame_id = ln_params_.cepton_frame;
        tf_published_.transform.translation.x = pose_holder.pose.position.x;
        tf_published_.transform.translation.y = pose_holder.pose.position.y;
        tf_published_.transform.translation.z = pose_holder.pose.position.z;
        tf_published_.transform.rotation = vehicle_location.pose.orientation;

        update_vehicle_pose(updated_pose, tf_published_, cepton_to_base_link_);
    }
    else if(!localization_running && !gps_running)
    {
        std::cout << "- updateTransforms: I don't know how you did it \n"
        << "but you somehow turned off everything \n"
        << "and got a function call the transform updater off... Great job?" << std::endl;
    }
    else
    {
        std::cout << "- updateTransforms: Something has gone wacky" << std::endl;
    }

    // Publish separate vehicle_pose message to send to Dspace
    map_tf_pub_.publish(updated_pose);
    // std::cout << "published" << std::endl;

    // Publish transform to be used by everyone else
    // std::cout << tf_published_ << std::endl;

    tf_br_->sendTransform(tf_published_);
    // std::cout << "tf broadcasted" << std::endl;
}

void LocalizationNode::transform_cb(const ros::TimerEvent & event)
{
    try
    {
        geometry_msgs::PoseStamped vehicle_pose_;

        // curr_transform_stamped_ = tf_buffer_->lookupTransform(ln_params_.map_frame, ln_params_.vehicle_frame, ros::Time(0));
        // base_link_to_map_stamped_ = tf_buffer_->lookupTransform(ln_params_.map_frame, ln_params_.vehicle_frame, ros::Time(0));

        // vehicle_pose_.header = curr_transform_stamped_.header;
        // vehicle_pose_.pose.position.x = curr_transform_stamped_.transform.translation.x;
        // vehicle_pose_.pose.position.y = curr_transform_stamped_.transform.translation.y;
        // vehicle_pose_.pose.position.z = curr_transform_stamped_.transform.translation.z;
        // vehicle_pose_.pose.orientation = curr_transform_stamped_.transform.rotation;

        // std::cout << vehicle_poses_queue_.size() << std::endl;
        // // if(vehicle_poses_queue_.size() == 0u)
        // // {
        // //     // If pose queue empty add two extra poses
        // //     vehicle_poses_queue_.push(vehicle_pose_);
        // //     vehicle_poses_queue_.push(vehicle_pose_);
        // // }
        
        // vehicle_poses_queue_.push(vehicle_pose_);
        // std::cout << vehicle_poses_queue_.size() << std::endl; // should equal to 3 after this
        // std::cout << "transform_cb: " << vehicle_pose_.pose.position.x << std::endl;

        // // traversed_distance_since_update_ = find_2d_pose_distance(vehicle_pose_.pose, vehicle_pose_at_update_);

        curr_transform_updated_ = true;
    }
    catch(tf2::TransformException & ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

void LocalizationNode::diviner_cb(const ros::TimerEvent & event)
{
    assert((void("Diviner is running when not supposed to."), localization_running));
    // Dropping the mtx lock for the moment
    // Would love to try getting this to run across multiple threads to try and decrease processing time
    // diviner_mtx_.lock();
    
    if(ln_params_.debug)
    {
        std::cout << "diviner_cb: Inside diviner_cb." << std::endl;
    }

    if(!synced_queue_.empty())
    {
        std::cout << "diviner_cb: number of messages in synced queue: " << synced_queue_.size() << std::endl;
        std::cout << "diviner_cb: " << synced_queue_.front().cloud->size() << " points before stepping." << std::endl;

        if(synced_queue_.front().cloud != nullptr && synced_queue_.front().cloud->size() > 0)
        {    
            if(map_ != nullptr)
            {
                std::cout << "diviner_cb: Map size: " << map_->capacity() << std::endl;

                // might need to move this somewhere else depending how well this works...
                std::cout << "diviner_cb: Starting Diviner Step." << std::endl;
                diviner_->step(synced_queue_.front().cloud, world_to_map_, cepton_to_base_link_, veh_pose);

                
                if(ln_params_.debug)
                {
                    // std::cout << "Estimated Pose Time Stamp: " << veh_pose->front().header.stamp << std::endl;
                    std::cout << "Estimated Pose: " << 
                    "(x = " << veh_pose->front().pose.position.x <<
                    ", y = " << veh_pose->front().pose.position.y <<
                    ", z = " << veh_pose->front().pose.position.z << ")" << std::endl;
                    
                    std::cout << "GPS Time Stamp: " << synced_queue_.front().gps.header.stamp << std::endl;
                    std::cout << "Actual Pose: " <<
                    "(x = "  << synced_queue_.front().gps.pose.position.x <<
                    ", y = " << synced_queue_.front().gps.pose.position.y <<
                    ", z = " << synced_queue_.front().gps.pose.position.z << ")" << std::endl;

                    std::cout << "num veh poses: " << veh_pose->size() << std::endl;
                    std::cout << "before: \n" << veh_pose->front().pose << std::endl;
                }
                
                LocalizationNode::updateTransforms(veh_pose->front());
                
                if(ln_params_.debug)
                {
                    std::cout << "after: \n" << veh_pose->front().pose << std::endl;
                }

                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(*map_->get_data(), output);
                localization_map_pub_.publish(output);
                pose_pub_.publish(veh_pose->front());
                gps_pub_.publish(synced_queue_.front().gps);

                synced_queue_.pop();

                if(ln_params_.topic_debug)
                {
                    // deskewed_pub_.publish();
                    // voxel_pub_.publish();
                    // octree_pub_.publish();
                }
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
    }
    else
    {
        std::cout << "diviner_cb: Messages not synced... Need to wait for synced messages." << std::endl;
    }
    // diviner_mtx_.unlock();
}

void LocalizationNode::lidar_cb(const sensor_msgs::PointCloud2ConstPtr input_cloud)
{
    // add mutex for data
    lidar_mtx_.lock();

    if(ln_params_.lidar_cb_debug)
    {
        std::cout << "- lidar_cb: Clearing Current Scan" << std::endl;
    }

    if(current_scan_ != nullptr)
    {
        //add stuff here
        current_scan_->clear();
        if(ln_params_.lidar_cb_debug)
        {
        std::cout << "- lidar_cb: Scan Cleared" << std::endl;
        }
    }
    else
    {
        if(ln_params_.lidar_cb_debug)
        {
        std::cout << "- lidar_cb: Scan already empty" << std::endl;
        }
    }

    current_scan_ = converter_->convert(input_cloud);
    
    while(current_scan_ == nullptr)
    {
        std::cout << "- lidar_cb: Waiting for current scan" << std::endl;
    }

    if(ln_params_.lidar_cb_debug)
    {
        std::cout << "- lidar_cb: Current scan has " << current_scan_->size() << " points." << std::endl;
        std::cout << "- lidar_cb: Current scan has time stamp: " << current_scan_->header.stamp << std::endl;
    }
    

    lidar_queue_.push(*current_scan_);
    
    if(ln_params_.lidar_cb_debug)
    {
        std::cout << "- lidar_cb: Current queue has " << lidar_queue_.size() << " scans in queue." << std::endl;
        std::cout << "- lidar_cb: Current scan in queue has " << lidar_queue_.back().size() << " points." << std::endl;
        std::cout << "- lidar_cb: Current scan in queue has time stamp: " << lidar_queue_.back().header.stamp<< std::endl;
    }
    
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

    if(ln_params_.gnss_cb_debug)
    {
        std::cout << "gnss_cb: Current number of queued poses: " << vehicle_poses_queue_.size() << std::endl;
    }

    tf2::doTransform(vehicle_pose, map_vehicle_pose, base_link_to_cepton_);
    
    // why tf does tf2 remove the stamp????? I'm going to lose it...
    map_vehicle_pose.header.stamp = vehicle_pose.header.stamp;
    map_vehicle_pose.header.frame_id = gnss_pose->header.frame_id;

    if(ln_params_.gnss_cb_debug)
    {
        std::cout << "- gnss_cb: Current number of queued poses: " << vehicle_poses_queue_.size() << std::endl;
    }

    // if(vehicle_poses_queue_.size() == 0u)
    // {
    //     // If pose queue empty add two extra poses
    //     vehicle_poses_queue_.push(map_vehicle_pose);
    //     vehicle_poses_queue_.push(map_vehicle_pose);
    // }

    // add to poses queue
    vehicle_poses_queue_.push(map_vehicle_pose);
    
    if(ln_params_.gnss_cb_debug)
    {
        std::cout << "- gnss_cb: New pose added, curr num vehicle poses in queue: " << vehicle_poses_queue_.size() << std::endl;
        std::cout << "- gnss_cb: vehicle_pose x = " << vehicle_pose.pose.position.x << std::endl;
        std::cout << "- gnss_cb: map_vehicle_pose x = " << map_vehicle_pose.pose.position.x << std::endl;
        std::cout << "- gnss_cb: vehilcle_pose_queue x = " << vehicle_poses_queue_.back().pose.position.x << std::endl;
    }

    if(!localization_running)
    {
        LocalizationNode::updateTransforms(vehicle_poses_queue_.front());
        vehicle_poses_queue_.pop();
        
        if(ln_params_.gnss_cb_debug)
        {
            std::cout << "- gnss_cb: Popped vehicle poses queue and passed transform" << std::endl;
            std::cout << "- gnss_cb: Num poses still in queue = " << vehicle_poses_queue_.size() << std::endl;    
        }
    }

    gnss_mtx_.unlock();
}

void LocalizationNode::syncer_cb(const ros::TimerEvent & event)
{
    syncer_mtx_.lock();

    if(!lidar_queue_.empty())
    {
        if(ln_params_.syncer_debug)
        {
            std::cout << "- syncer_cb: Num scans in lidar queue: " << lidar_queue_.size() << std::endl;
            std::cout << "- syncer_cb: Num points in lidar queue front: " << lidar_queue_.front().size() << std::endl;
            std::cout << "- syncer_cb: Num points in lidar queue back: " << lidar_queue_.back().size() << std::endl;
            std::cout << "- syncer_cb: Num poses in poses queue: " << vehicle_poses_queue_.size() << std::endl;
        }

        if(!vehicle_poses_queue_.empty())
        {
            if(ln_params_.syncer_debug)
            {
                std::cout << "- syncer_cb: Syncer Syncing" << std::endl;
            }

            synced_queue_.push(syncer_->sync(lidar_queue_,vehicle_poses_queue_));

            if(ln_params_.syncer_debug)
            {
                std::cout << "- syncer_cb: Num msgs in synced queue after running = " << synced_queue_.size() << std::endl;
                std::cout << "- syncer_cb: Num scans in lidar queue: " << lidar_queue_.size() << std::endl;
                std::cout << "- syncer_cb: Num poses in poses queue: " << vehicle_poses_queue_.size() << std::endl;
                std::cout << "- syncer_cb: Scan size = " << synced_queue_.back().cloud->size() << std::endl;
            }
        }
        else
        {
            if(lidar_queue_.size() > 1)
            {
                std::cout << "- syncer_cb: No vehicle poses adding 0 pose " << std::endl;

                diviner::SyncedMsgs synced_out;

                // Add point cloud
                pcl::PointCloud<diviner::PointStamped>::Ptr cloud_ptr(new pcl::PointCloud<diviner::PointStamped>);
                *cloud_ptr = lidar_queue_.front();
                synced_out.cloud = cloud_ptr;

                // Add gps msg
                synced_out.gps.header.stamp.fromNSec(lidar_queue_.front().header.stamp);
                synced_out.gps.pose.position.x = 0.0;
                synced_out.gps.pose.position.y = 0.0;
                synced_out.gps.pose.position.z = 0.0;
                synced_out.gps.pose.orientation.x = 0.0;
                synced_out.gps.pose.orientation.y = 0.0;
                synced_out.gps.pose.orientation.z = 0.0;
                synced_out.gps.pose.orientation.w = 1.0;

                // Output "synced" msg
                synced_queue_.push(synced_out);

                lidar_queue_.pop();
            }
            else
            {
                std::cout << "- syncer_cb: No vehicle poses and only 1 scan in queue... waiting for new gps pose" << std::endl;
            }
        }
    }
    else
    {
        std::cout << "- syncer_cb: No lidar scans available... waiting for new scan" << std::endl;
    }


        // while (!lidar_queue_.empty() || !vehicle_poses_queue_.empty())
        // {
            
            // if (!lidar_queue_.empty() && vehicle_poses_queue_.empty())
            // {
            //     //Do something if lidar is NOT empty and gps is empty
            // } 
            // else if (lidar_queue_.empty() && !vehicle_poses_queue_.empty()) 
            // {
            //     //Do something if lidar is empty and gps is NOT empty
            //     //Temporarily dump extra vehicle poses
            //     //RYAN this does something but I have no clue if it is what it should 
            //     //The idea is too dump gps queue if no new lidar scans come in when the gps scans come in, but it might be 
            //     //doing it very wrong
            //     // while(!vehicle_poses_queue_.empty()){
            //     //     vehicle_poses_queue_.pop();
            //     //     std::cout << "syncer_cb: Dumping" << std::endl;
            //     // }
            // }
    
        // }
    
    syncer_mtx_.unlock();
}

// void LocalizationNode::imu_cb(const novatel_oem7_msgs::CORRIMU msg)
// {
//     // Take in imu data
//     imu_info.roll = msg.roll_rate;
//     imu_info.pitch = msg.pitch_rate;
//     imu_info.yaw = msg.yaw_rate;
//     imu_info.stamp = msg.header.stamp;
    
//     // Add to imu queue
//     imu_queue_.push(imu_info);
// }

}