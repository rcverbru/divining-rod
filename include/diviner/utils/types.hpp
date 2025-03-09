#ifndef DIVINER__UTILS__TYPES_HPP
#define DIVINER__UTILS__TYPES_HPP

// Custom PCL Point
#define PCL_NO_PRECOMPILE
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

// TF shtuff
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

// PCL
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GNSS
#include <novatel_oem7_msgs/BESTGNSSPOS.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <novatel_oem7_msgs/BESTUTM.h>
#include <nav_msgs/Odometry.h>

// Other includes
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    uint8_t relative_timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (Point,         // here we assume a XYZ + "test" (as fields)
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uint8_t, relative_timestamp, relative_timestamp)
)

namespace diviner
{

// Types
typedef pcl::PointXYZI PointStamped;
typedef nav_msgs::OdometryConstPtr GnssType;

// // Point Field conversions
// uint8 INT8    = 1
// uint8 UINT8   = 2
// uint8 INT16   = 3
// uint8 UINT16  = 4
// uint8 INT32   = 5
// uint8 UINT32  = 6
// uint8 FLOAT32 = 7
// uint8 FLOAT64 = 8

struct CeptonPoint
{
    float x;
    float y;
    float z;
    float reflectivity;
    uint8_t relative_timestamp;
    uint8_t point_timestamp;
    uint8_t flags;
    uint8_t channel_id;
    uint8_t valid;
};

//
template<typename CHILD_PARAMS, typename PARENT_PARAMS>
struct Params
{
    CHILD_PARAMS child_params;
    PARENT_PARAMS parent_params;
};



struct Velocity
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
};

struct Point3d
{
    float x;
    float y;
    float z;
};

struct IMUinfo
{
    ros::Time stamp;
    float roll;
    float pitch;
    float yaw;
};

struct Alignment
{
    Eigen::Matrix4d transformation_matrix;
    geometry_msgs::TransformStamped transform;
};

struct SyncedMsgs
{
    geometry_msgs::PoseStamped gps;
    pcl::PointCloud<diviner::PointStamped>::Ptr cloud;
};

// struct LidarPoint
// {
//     // Header header;
//     float x;
//     float y;
//     float z;
//     // pcl::PointXYZ point;
//     uint8_t relative_timestamp;
// }; 

struct LidarPoint : public pcl::PointXYZ
{
    uint8_t relative_timestamp;
};

}

#endif