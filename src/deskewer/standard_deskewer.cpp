#include <deskewer/standard_deskewer.hpp>

namespace diviner
{

void StandardDeskewer::deskew(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud, const std::vector<diviner::Velocity> & velocities)
{
    // // p* = Exp(sw)p + sv
    // num_vel = velocities.size();
    // current_velocity = velocities->data(); // need to pull velocity[num_vel] to get most recent estimate

    // // s = time from first point return to time of current point
    // for(auto point : *point_cloud)
    // {
    //     new_point.x = exp(point.relative_timestamp * current_velocity.angular) * point.point.x + point.relative_timestamp * current_velocity.linear;
    //     new_point.y = exp(point.relative_timestamp * current_velocity.angular) * point.point.y + point.relative_timestamp * current_velocity.linear;
    //     new_point.z = exp(point.relative_timestamp * current_velocity.angular) * point.point.z + point.relative_timestamp * current_velocity.linear;

    //     deskewed_cloud->push_back(new_point);
    // }
    
    // Transfer deskewed_cloud back to main point_cloud pointer
    point_cloud = deskewed_cloud;
}

}