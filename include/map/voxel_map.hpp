#ifndef MAP__VOXEL_MAP_HPP
#define MAP__VOXEL_MAP_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct VoxelMapParams
{
    std::string voxel_frame = "cepton2";
    double min_points_per_voxel = 1;
    double voxel_size = 0.25;
    double max_distance = 40;
    bool output_map = false;
    bool debug = false;
};

class VoxelMap : public IMap
{
    public:
        explicit VoxelMap(const Params<VoxelMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params){
            local_map_pointcloud = boost::make_shared<pcl::PointCloud<diviner::PointStamped>>();
            local_map_pointcloud->header.frame_id = params_.voxel_frame;
        };
        ~VoxelMap()
        {
            if(local_map_pointcloud != nullptr)
            {
                clear_map();
            }
        }

        void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;

        pcl::PointCloud<diviner::PointStamped>::Ptr get_data() override;

        pcl::PointCloud<diviner::PointStamped>::Ptr apply_transform(geometry_msgs::Transform & transform, pcl::PointCloud<diviner::PointStamped>::Ptr cloud);

        void get_voxels();

        void clear_map() override;
        
        int capacity() override;
        
        size_t size() override;

        /**
         * Trims down the current local map by removing points 
         * outside of the max distance range
         * 
         * @param 
         */
        void trim_map() override;

    private:
        VoxelMapParams params_;

        pcl::PointCloud<diviner::PointStamped>::Ptr local_map_pointcloud;
        std::vector<diviner::PointStamped, Eigen::aligned_allocator<diviner::PointStamped>> voxel_centers;

};

}

#endif