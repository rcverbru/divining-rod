#include <localization_node.hpp>

#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init (argc, argv, "WDFAI Node");
    auto node = std::make_shared<ros::NodeHandle>("~");

    localization_node::LocalizationNodeParams ln_params;

    // Collecting ROS topics
    node->getParam("lidar_topic", ln_params.lidar_topic);
    node->getParam("wheel_tick_topic", ln_params.wheel_tick_topic);
    node->getParam("gnss_topic", ln_params.gnss_topic);
    node->getParam("map_tf_topic", ln_params.map_tf_topic);

    // Collecting ROS Timers
    node->getParam("diviner_pub_frequency_hz", ln_params.diviner_pub_frequency_hz);
    node->getParam("syncer_pub_frequency_hz", ln_params.syncer_pub_frequency_hz);

    // Collect LocalizationNode Parameters
    node->getParam("running_state", ln_params.running_state);
    node->getParam("debug", ln_params.debug);
    node->getParam("lidar_cb_debug", ln_params.lidar_cb_debug);
    node->getParam("gnss_cb_debug", ln_params.gnss_cb_debug);
    node->getParam("topic_debug", ln_params.topic_debug);

    // Diviner Params
    node->getParam("diviner_debug", ln_params.diviner_debug);

    // Aligner Params
    node->getParam("aligner", ln_params.aligner);
    node->getParam("convergence_criterion", ln_params.convergence_criterion);
    node->getParam("aligner_debug", ln_params.aligner_debug);

    node->getParam("deskewer", ln_params.deskewer);
    node->getParam("deskewer_debug", ln_params.deskewer_debug);

    // Filter Params
    node->getParam("leaf_size", ln_params.leaf_size);
    node->getParam("filter", ln_params.filter);
    node->getParam("filter_debug", ln_params.filter_debug);

    // Deskewer Params
    node->getParam("deskewer_debug", ln_params.deskewer_debug);

    // Mapper Params
    node->getParam("max_distance", ln_params.max_distance);
    node->getParam("max_points_per_voxel", ln_params.max_points_per_voxel);
    node->getParam("map", ln_params.map);
    node->getParam("map_debug", ln_params.map_debug);

    // Vestimator Params
    node->getParam("vestimator", ln_params.vestimator);
    node->getParam("vestimator_debug", ln_params.vestimator_debug);

    // Utils
    node->getParam("max_sync_err_s", ln_params.max_sync_err_s);
    node->getParam("point_type", ln_params.point_type);
    node->getParam("switcher_debug", ln_params.switcher_debug);
    node->getParam("syncer_debug", ln_params.syncer_debug);
    node->getParam("converter_debug", ln_params.converter_debug);


    auto localization_node = std::make_shared<localization_node::LocalizationNode>(node, ln_params);

    // add async spinner
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}